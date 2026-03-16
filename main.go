package main

import (
	"bufio"
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

	"ts2phc-go/metrics"
	"ts2phc-go/phc"
	"ts2phc-go/pmc"
	"ts2phc-go/pps"
	"ts2phc-go/servo"

	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"github.com/spf13/cobra"
	"github.com/spf13/viper"
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

	// gpsd source
	f.String("gpsd-addr", "localhost:2947", "gpsd JSON stream address")

	// PTP sink flags
	f.StringP("sink", "c", "/dev/ptp0", "PHC device to discipline")
	f.BoolP("autocfg", "a", false, "enable ptp4l PMC autoconfiguration")
	f.Int("tai-offset", 37, "TAI-UTC offset in seconds")
	f.String("leapfile", "/usr/share/zoneinfo/leap-seconds.list", "leap seconds file (if present)")

	// Metrics flags
	f.String("metrics-addr", ":9100", "prometheus metrics listen address")
	f.Bool("metrics", true, "enable Prometheus metrics server")

	for _, key := range []string{
		"gpsd_addr",
		"sink", "autocfg", "tai_offset", "leapfile",
		"metrics_addr", "metrics",
	} {
		_ = viper.BindPFlag(key, f.Lookup(key))
	}
	_ = viper.BindPFlag("gpsd_addr", f.Lookup("gpsd-addr"))
	_ = viper.BindPFlag("tai_offset", f.Lookup("tai-offset"))
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
	gpsdAddr := viper.GetString("gpsd_addr")
	ptpSink := viper.GetString("sink")
	autoCfg := viper.GetBool("autocfg")
	taiOffset := viper.GetInt("tai_offset")
	leapfile := viper.GetString("leapfile")
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

	// --- gpsd source ---
	adapter := &pps.GpsdMetricsAdapter{Metrics: met}
	source := pps.NewGpsdSource(gpsdAddr, taiOffset, adapter)

	go func() {
		for {
			if err := source.Run(); err != nil {
				log.Printf("gpsd: %v", err)
			}
			time.Sleep(2 * time.Second)
		}
	}()

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
