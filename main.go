package main

import (
	"bufio"
	"bytes"
	"fmt"
	"log"
	"math"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"syscall"
	"time"

	"ts2phc-go/demux"
	"ts2phc-go/export"
	"ts2phc-go/gpsnmea"
	"ts2phc-go/metrics"
	"ts2phc-go/phc"
	"ts2phc-go/pmc"
	"ts2phc-go/pps"
	"ts2phc-go/servo"
	"ts2phc-go/ubx"

	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"github.com/spf13/cobra"
	"github.com/spf13/viper"
	"go.bug.st/serial"
)

const (
	outlierThresholdNS = 50000 // 50 µs
	outlierResetCount  = 5
)

var (
	cfgFile string
	root    = &cobra.Command{
		Use:   "ts2phc-go",
		Short: "GPS-disciplined PTP clock daemon",
		RunE:  run,
	}
)

func init() {
	cobra.OnInitialize(initConfig)
	f := root.PersistentFlags()
	f.StringVar(&cfgFile, "config", "", "config file (default $HOME/.ts2phc-go.yaml)")

	// GPS flags
	f.String("dev", "/dev/ttyACM0", "GPS serial device path")
	f.Int("baud", 115200, "GPS serial baud rate")
	f.Int("ant-cable-delay-ns", 38, "antenna cable delay in ns")

	// PTP sink flags
	f.StringP("sink", "c", "/dev/ptp0", "PHC device to discipline")
	f.BoolP("autocfg", "a", false, "enable ptp4l PMC autoconfiguration")
	f.Int("tai-offset", 37, "TAI-UTC offset in seconds")
	f.String("leapfile", "/usr/share/zoneinfo/leap-seconds.list", "leap seconds file (if present)")

	// Export flags
	f.String("tcp-addr", ":2948", "TCP NMEA export listen address for gpsd")
	f.Bool("tcp", true, "enable TCP NMEA export for gpsd")

	// Metrics flags
	f.String("metrics-addr", ":9100", "prometheus metrics listen address")
	f.Bool("metrics", true, "enable Prometheus metrics server")

	for _, key := range []string{
		"dev", "baud", "ant_cable_delay_ns",
		"sink", "autocfg", "tai_offset", "leapfile",
		"tcp_addr", "tcp",
		"metrics_addr", "metrics",
	} {
		_ = viper.BindPFlag(key, f.Lookup(key))
	}
	// Bind hyphenated flag names to underscored viper keys
	_ = viper.BindPFlag("ant_cable_delay_ns", f.Lookup("ant-cable-delay-ns"))
	_ = viper.BindPFlag("tai_offset", f.Lookup("tai-offset"))
	_ = viper.BindPFlag("tcp_addr", f.Lookup("tcp-addr"))
	_ = viper.BindPFlag("metrics_addr", f.Lookup("metrics-addr"))
}

func initConfig() {
	if cfgFile != "" {
		viper.SetConfigFile(cfgFile)
	} else {
		home, _ := os.UserHomeDir()
		viper.AddConfigPath(home)
		viper.SetConfigType("yaml")
		viper.SetConfigName(".ts2phc-go")
	}
	viper.SetEnvPrefix("TS2PHC")
	viper.AutomaticEnv()
	_ = viper.ReadInConfig()
}

func main() {
	log.SetFlags(log.Ltime | log.Lmicroseconds)
	if err := root.Execute(); err != nil {
		os.Exit(1)
	}
}

type clockState struct {
	sink         *pps.Sink
	state        uint16
	isTarget     bool
	outlierCount int
}

func run(cmd *cobra.Command, args []string) error {
	dev := viper.GetString("dev")
	baud := viper.GetInt("baud")
	antCableDelayNs := viper.GetInt("ant_cable_delay_ns")
	ptpSink := viper.GetString("sink")
	autoCfg := viper.GetBool("autocfg")
	taiOffset := viper.GetInt("tai_offset")
	leapfile := viper.GetString("leapfile")
	tcpAddr := viper.GetString("tcp_addr")
	enableTCP := viper.GetBool("tcp")
	metricsAddr := viper.GetString("metrics_addr")
	enableMetrics := viper.GetBool("metrics")

	if !cmd.Flags().Lookup("tai-offset").Changed && leapfile != "" {
		if v, err := loadTAIOffset(leapfile); err == nil {
			taiOffset = v
			log.Printf("TAI-UTC offset %d from %s", taiOffset, leapfile)
		} else if !os.IsNotExist(err) {
			log.Printf("leapfile %s: %v", leapfile, err)
		}
	}

	log.Printf("ts2phc-go starting")

	// --- GPS serial ---
	port, err := serial.Open(dev, &serial.Mode{BaudRate: baud})
	if err != nil {
		return fmt.Errorf("serial open %s: %w", dev, err)
	}
	defer port.Close()
	port.SetReadTimeout(2 * time.Second)
	log.Printf("serial: opened %s @ %d baud", dev, baud)

	if err := configureModule(port, antCableDelayNs); err != nil {
		return fmt.Errorf("configure: %w", err)
	}
	if _, err := port.Write(ubx.EncodePoll(ubx.ClassMON, ubx.IDMonVer)); err != nil {
		log.Printf("warn: failed to poll MON-VER: %v", err)
	}

	// --- TCP NMEA export for gpsd ---
	var tcpExport *export.TCPExport
	if enableTCP {
		tcpExport, err = export.NewTCP(tcpAddr)
		if err != nil {
			return fmt.Errorf("tcp: %w", err)
		}
		defer tcpExport.Close()
	}

	// --- Prometheus metrics ---
	var met *metrics.Metrics
	if enableMetrics {
		reg := prometheus.NewRegistry()
		met = metrics.New(reg)
		http.Handle("/metrics", promhttp.HandlerFor(reg, promhttp.HandlerOpts{}))
		go func() {
			log.Printf("metrics: listening on %s", metricsAddr)
			if err := http.ListenAndServe(metricsAddr, nil); err != nil {
				log.Printf("metrics http: %v", err)
			}
		}()
	}

	// --- UBX source (fed by demux handler) ---
	source := pps.NewUBXSource(taiOffset)

	h := &handler{
		source:  source,
		tcp:     tcpExport,
		metrics: met,
	}

	// --- PPS sink ---
	polarity := uint32(phc.PTP_RISING_EDGE | phc.PTP_FALLING_EDGE)
	sink, err := pps.NewSink(ptpSink, 0, polarity)
	if err != nil {
		return fmt.Errorf("open PPS sink: %w", err)
	}
	defer sink.Destroy()

	pinDesc := phc.PinDesc{
		Index: 0,
		Func:  phc.PTP_PF_EXTTS,
		Chan:  0,
	}
	if err := sink.Device.SetPinFunc(pinDesc); err != nil {
		log.Printf("Warning: PTP_PIN_SETFUNC failed (may already be configured): %v", err)
	}
	if err := sink.Arm(); err != nil {
		return fmt.Errorf("arm sink: %w", err)
	}
	sink.Servo.SyncInterval(1.0)

	sinks := []*pps.Sink{sink}
	clocks := []*clockState{{sink: sink, state: pmc.PS_SLAVE, isTarget: true}}

	// --- PMC agent (optional) ---
	var agent *pmc.Agent
	if autoCfg {
		agent, err = pmc.NewAgent("/var/run/ptp4l")
		if err != nil {
			log.Printf("PMC agent connect failed: %v. Running without it.", err)
		} else {
			defer agent.Close()
			log.Printf("Connected to ptp4l PMC agent")
			agent.QueryDDS()
			agent.Subscribe()
		}
	}

	// --- Start demux goroutine ---
	go func() {
		if err := demux.New(port, h).Run(); err != nil {
			log.Printf("demux: %v", err)
		}
	}()

	// --- Signal handling ---
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	log.Printf("entering main loop")

	// --- PPS discipline loop ---
	for {
		select {
		case <-sigChan:
			log.Printf("shutting down")
			return nil
		default:
			if agent != nil {
				if portNum, state, err := agent.PollEvents(); err == nil {
					log.Printf("port %d changed to state %d", portNum, state)
				}
			}

			readyToSync, err := pps.PollSinks(sinks, 2000*time.Millisecond)
			if err != nil {
				log.Printf("PollSinks error: %v", err)
				time.Sleep(1 * time.Second)
				continue
			}
			if !readyToSync {
				continue
			}

			sourceTs, err := source.GetPPSTime()
			if err != nil {
				continue
			}

			var srcEdge int64
			if sourceTs.Nanosecond() > 500000000 {
				srcEdge = sourceTs.Unix() + 1
			} else {
				srcEdge = sourceTs.Unix()
			}
			srcNanos := srcEdge * 1e9

			for _, c := range clocks {
				if !c.isTarget || !c.sink.IsAvailable {
					continue
				}
				c.sink.IsAvailable = false

				sinkNanos := c.sink.LastValidTS.Unix()*1e9 + int64(c.sink.LastValidTS.Nanosecond())
				offset := sinkNanos - srcNanos

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

				if met != nil {
					met.UpdateTS2PHC(c.sink.Name, float64(offset), -adjFrequency)
				}

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

func configureModule(port serial.Port, antCableDelayNs int) error {
	frame := ubx.EncodeValset(ubx.LayerRAM,
		ubx.CfgU1(ubx.CfgNavspgDynModel, 2),
		ubx.CfgI2(ubx.CfgTpAntCableDelay, int16(antCableDelayNs)),
		ubx.CfgU1(ubx.CfgMsgoutUbxNavPvtUSB, 1),
		ubx.CfgU1(ubx.CfgMsgoutUbxNavDopUSB, 1),
		ubx.CfgU1(ubx.CfgMsgoutUbxNavTimeUSB, 1),
		ubx.CfgU1(ubx.CfgMsgoutUbxNavClkUSB, 1),
		ubx.CfgU1(ubx.CfgMsgoutUbxNavSatUSB, 5),
		ubx.CfgU1(ubx.CfgMsgoutUbxTimTpUSB, 1),
		ubx.CfgU1(ubx.CfgMsgoutNmeaRmcUSB, 0),
		ubx.CfgU1(ubx.CfgMsgoutNmeaZdaUSB, 0),
		ubx.CfgU1(ubx.CfgMsgoutNmeaGgaUSB, 0),
		ubx.CfgU1(ubx.CfgMsgoutNmeaGllUSB, 0),
		ubx.CfgU1(ubx.CfgMsgoutNmeaGsaUSB, 0),
		ubx.CfgU1(ubx.CfgMsgoutNmeaGsvUSB, 0),
		ubx.CfgU1(ubx.CfgMsgoutNmeaVtgUSB, 0),
		ubx.CfgL(ubx.CfgUSBOutprotUBX, true),
		ubx.CfgL(ubx.CfgUSBOutprotNMEA, false),
		ubx.CfgL(ubx.CfgUSBInprotUBX, true),
	)

	if _, err := port.Write(frame); err != nil {
		return fmt.Errorf("write VALSET: %w", err)
	}
	log.Println("config: sent VALSET (RAM)")

	time.Sleep(200 * time.Millisecond)
	buf := make([]byte, 256)
	n, _ := port.Read(buf)
	if n > 0 {
		if bytes.Contains(buf[:n], []byte{ubx.SyncA, ubx.SyncB, ubx.ClassACK, ubx.IDAckAck}) {
			log.Println("config: ACK received")
		} else if bytes.Contains(buf[:n], []byte{ubx.SyncA, ubx.SyncB, ubx.ClassACK, ubx.IDAckNak}) {
			log.Println("config: NAK received — check key IDs")
		}
	}
	return nil
}

// handler implements demux.Handler, bridging UBX messages to the UBXSource, metrics, and TCP export.
type handler struct {
	source  *pps.UBXSource
	tcp     *export.TCPExport
	metrics *metrics.Metrics
	lastDOP *ubx.NavDOP
	lastSAT *ubx.NavSAT
}

func (h *handler) OnUBX(frame ubx.Frame) {
	switch frame.ClassID() {
	case ubx.MsgNavPVT:
		pvt, err := ubx.ParseNavPVT(frame.Payload)
		if err != nil {
			log.Printf("parse NAV-PVT: %v", err)
			return
		}
		if h.metrics != nil {
			h.metrics.UpdateNavPVT(pvt)
		}
		h.sendNMEA(pvt, h.lastSAT, h.lastDOP)

	case ubx.MsgNavTimeUTC:
		t, err := ubx.ParseNavTimeUTC(frame.Payload)
		if err != nil {
			log.Printf("parse NAV-TIMEUTC: %v", err)
			return
		}
		if h.metrics != nil {
			h.metrics.UpdateNavTimeUTC(t)
		}

	case ubx.MsgNavClock:
		c, err := ubx.ParseNavClock(frame.Payload)
		if err != nil {
			log.Printf("parse NAV-CLOCK: %v", err)
			return
		}
		if h.metrics != nil {
			h.metrics.UpdateNavClock(c)
		}

	case ubx.MsgNavDOP:
		dop, err := ubx.ParseNavDOP(frame.Payload)
		if err != nil {
			log.Printf("parse NAV-DOP: %v", err)
			return
		}
		h.lastDOP = dop
		if h.metrics != nil {
			h.metrics.UpdateNavDOP(dop)
		}

	case ubx.MsgNavSAT:
		sat, err := ubx.ParseNavSAT(frame.Payload)
		if err != nil {
			log.Printf("parse NAV-SAT: %v", err)
			return
		}
		if h.metrics != nil {
			h.metrics.UpdateNavSAT(sat)
		}
		h.lastSAT = sat
		h.sendGSV(sat)

	case ubx.MsgTimTP:
		tp, err := ubx.ParseTimTP(frame.Payload)
		if err != nil {
			log.Printf("parse TIM-TP: %v", err)
			return
		}
		h.source.SetTimTP(tp)
		if h.metrics != nil {
			h.metrics.UpdateTimTP(tp)
		}

	case ubx.MsgMonVer:
		ver, err := ubx.ParseMonVer(frame.Payload)
		if err != nil {
			log.Printf("parse MON-VER: %v", err)
			return
		}
		log.Printf("firmware: sw=%s hw=%s", ver.SwVersion, ver.HwVersion)
		for _, ext := range ver.Extensions {
			log.Printf("firmware: ext=%s", ext)
		}

	case ubx.MsgAckAck:
		ack, _ := ubx.ParseAck(frame.Payload)
		log.Printf("ACK-ACK cls=0x%02x msg=0x%02x", ack.ClsID, ack.MsgID)

	case ubx.MsgAckNak:
		ack, _ := ubx.ParseAck(frame.Payload)
		log.Printf("ACK-NAK cls=0x%02x msg=0x%02x", ack.ClsID, ack.MsgID)
	}
}

func (h *handler) sendNMEA(pvt *ubx.NavPVT, sat *ubx.NavSAT, dop *ubx.NavDOP) {
	if h.tcp == nil {
		return
	}
	broadcast := func(b []byte) {
		if len(b) > 0 {
			h.tcp.Broadcast(b)
		}
	}
	if pvt != nil {
		broadcast(gpsnmea.GGAFromPVT(pvt, dop))
		broadcast(gpsnmea.RMCFromPVT(pvt))
		for _, b := range gpsnmea.GSAFromPVT(pvt, sat, dop) {
			broadcast(b)
		}
		broadcast(gpsnmea.ZDAFromPVT(pvt))
		broadcast(gpsnmea.VTGFromPVT(pvt))
	}
}

func (h *handler) sendGSV(sat *ubx.NavSAT) {
	if h.tcp == nil {
		return
	}
	for _, b := range gpsnmea.GSVFromSAT(sat) {
		h.tcp.Broadcast(b)
	}
}

func (h *handler) OnNMEA(sentence []byte) {
	if h.tcp != nil {
		h.tcp.Broadcast(sentence)
	}
}

func loadTAIOffset(path string) (int, error) {
	f, err := os.Open(path)
	if err != nil {
		return 0, err
	}
	defer f.Close()

	sc := bufio.NewScanner(f)
	var last int
	var found bool

	for sc.Scan() {
		line := strings.TrimSpace(sc.Text())
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		fields := strings.Fields(line)
		if len(fields) < 2 {
			continue
		}
		v, err := strconv.Atoi(fields[1])
		if err != nil {
			continue
		}
		last = v
		found = true
	}
	if err := sc.Err(); err != nil {
		return 0, err
	}
	if !found {
		return 0, fmt.Errorf("no leap entries in %s", path)
	}
	return last, nil
}
