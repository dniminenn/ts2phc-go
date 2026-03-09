package main

import (
	"flag"
	"log"
	"math"
	"os"
	"os/signal"
	"syscall"
	"time"

	"ts2phc-go/phc"
	"ts2phc-go/pmc"
	"ts2phc-go/pps"
	"ts2phc-go/servo"
)

var (
	todSource = flag.String("s", "generic", "Source of the PPS signal (generic, nmea, or /dev/ptpX)")
	ptpSink   = flag.String("c", "/dev/ptp0", "PHC time sink (like /dev/ptp0)")
	autoCfg   = flag.Bool("a", false, "Turn on autoconfiguration via ptp4l")

	nmeaSerial = flag.String("nmea-serialport", "/dev/ttyS0", "NMEA serial port")
	nmeaBaud   = flag.Int("nmea-baudrate", 9600, "NMEA serial baud rate")
	nmeaHost   = flag.String("nmea-host", "", "NMEA TCP host (overrides serial)")
	nmeaPort   = flag.String("nmea-port", "", "NMEA TCP port")
	nmeaDelay  = flag.Int("nmea-delay", 0, "NMEA delay correction (nanoseconds)")
	taiOffset  = flag.Int("tai-offset", 37, "TAI-UTC offset in seconds (37 since 2017-01-01)")
)

const (
	// Reject samples with offset exceeding this while servo is locked.
	// 5 consecutive outliers triggers a servo reset.
	outlierThresholdNS = 50000 // 50 µs
	outlierResetCount  = 5
)

type ClockState struct {
	sink         *pps.Sink
	state        uint16
	isTarget     bool
	outlierCount int
}

func main() {
	flag.Parse()

	log.SetFlags(0)
	log.Printf("ts2phc-go starting")

	var source pps.Source
	var err error

	// Initialize the Source
	if *todSource == "generic" {
		source = pps.NewGenericSource()
		log.Printf("Using generic source")
	} else if *todSource == "nmea" {
		source, err = pps.NewNMEASource(pps.NMEAConfig{
			SerialPort: *nmeaSerial,
			BaudRate:   *nmeaBaud,
			RemoteHost: *nmeaHost,
			RemotePort: *nmeaPort,
			DelayNS:    *nmeaDelay,
			TAIOffset:  *taiOffset,
		})
		if err != nil {
			log.Fatalf("Failed to open NMEA source: %v", err)
		}
		log.Printf("Using NMEA source")
	} else {
		source, err = pps.NewPHCSource(*todSource)
		if err != nil {
			log.Fatalf("Failed to open PHC source: %v", err)
		}
		log.Printf("Using PHC source %s", *todSource)
	}
	defer source.Destroy()

	// Initialize the Sink array (only 1 supported via simple CLI for now, but logic supports N)
	var sinks []*pps.Sink
	var clocks []*ClockState

	polarity := uint32(phc.PTP_RISING_EDGE | phc.PTP_FALLING_EDGE)
	sink, err := pps.NewSink(*ptpSink, 0, polarity)
	if err != nil {
		log.Fatalf("Failed to open PPS sink: %v", err)
	}
	defer sink.Destroy()

	// Configure the pin for EXTTS on channel 0
	pinDesc := phc.PinDesc{
		Index: 0,
		Func:  phc.PTP_PF_EXTTS,
		Chan:  0,
	}
	if err := sink.Device.SetPinFunc(pinDesc); err != nil {
		log.Printf("Warning: PTP_PIN_SETFUNC failed (may already be configured): %v", err)
	}

	if err := sink.Arm(); err != nil {
		log.Fatalf("Failed to arm sink: %v", err)
	}

	// Initialize servo with the PPS sync interval (1 second)
	sink.Servo.SyncInterval(1.0)

	sinks = append(sinks, sink)
	clocks = append(clocks, &ClockState{sink: sink, state: pmc.PS_SLAVE, isTarget: true})

	var agent *pmc.Agent
	if *autoCfg {
		agent, err = pmc.NewAgent("/var/run/ptp4l")
		if err != nil {
			log.Printf("Failed to connect to ptp4l pmc agent: %v. Running without it.", err)
		} else {
			defer agent.Close()
			log.Printf("Connected to ptp4l PMC agent, subscribing to events")
			agent.QueryDDS()
			agent.Subscribe()
		}
	}

	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	log.Printf("Entering main loop")

	for {
		select {
		case <-sigChan:
			log.Printf("Shutting down")
			return
		default:
			if agent != nil {
				// Non-blocking poll for port state changes
				port, state, err := agent.PollEvents()
				if err == nil {
					log.Printf("Port %d changed to state %d", port, state)
					// In a multi-port setup, we would update ClockState here
					// and run a ts2phc_reconfigure() equivalent to pick the reference.
				}
			}

			// Block until ALL sinks have received an event (or timeout)
			readyToSync, err := pps.PollSinks(sinks, 2000*time.Millisecond)
			if err != nil {
				log.Printf("PollSinks error: %v", err)
				time.Sleep(1 * time.Second)
				continue
			}

			if !readyToSync {
				// Timeout or ignored edge (e.g., dynamic pulse width trailing edge)
				continue
			}

			// ---------------------------------------------------------
			// ts2phc_synchronize_clocks logic
			// ---------------------------------------------------------

			sourceTs, err := source.GetPPSTime()
			if err != nil {
				continue
			}

			// Implicit timestamp assumes the previous start-of-second edge
			var srcEdge int64
			if sourceTs.Nanosecond() > 500000000 {
				srcEdge = sourceTs.Unix() + 1
			} else {
				srcEdge = sourceTs.Unix()
			}
			srcNanos := srcEdge * 1e9

			for _, c := range clocks {
				if !c.isTarget {
					continue
				}

				// Use the actual EXTTS hardware timestamp
				if !c.sink.IsAvailable {
					continue
				}
				c.sink.IsAvailable = false

				sinkNanos := c.sink.LastValidTS.Unix()*1e9 + int64(c.sink.LastValidTS.Nanosecond())

				offset := sinkNanos - srcNanos

				// Outlier rejection: if servo is locked, reject wild samples
				if c.sink.Servo.IsLocked() && math.Abs(float64(offset)) > outlierThresholdNS {
					c.outlierCount++
					log.Printf("[%s] OUTLIER offset %d (count %d/%d)",
						c.sink.Name, offset, c.outlierCount, outlierResetCount)
					if c.outlierCount >= outlierResetCount {
						log.Printf("[%s] too many outliers, resetting servo", c.sink.Name)
						c.sink.Servo.Reset()
						c.outlierCount = 0
					}
					continue
				}
				c.outlierCount = 0

				adjFrequency, state := c.sink.Servo.Sample(offset, uint64(sinkNanos), 1.0)

				log.Printf("[%s] offset %10d s%d freq %+7.0f",
					c.sink.Name, offset, state, -adjFrequency)

				switch state {
				case servo.Jump:
					if err := c.sink.Device.AdjFreq(-adjFrequency); err != nil {
						log.Printf("[%s] AdjFreq error: %v", c.sink.Name, err)
						c.sink.Servo.Reset()
						break
					}
					if err := c.sink.Device.StepTime(-offset); err != nil {
						log.Printf("[%s] StepTime error: %v", c.sink.Name, err)
						c.sink.Servo.Reset()
						break
					}
					c.sink.Servo.Reset()
				case servo.Locked, servo.LockedStable:
					if err := c.sink.Device.AdjFreq(-adjFrequency); err != nil {
						log.Printf("[%s] AdjFreq error: %v", c.sink.Name, err)
						c.sink.Servo.Reset()
					}
				}
			}
		}
	}
}
