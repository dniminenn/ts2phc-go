package pps

import (
	"bufio"
	"encoding/json"
	"fmt"
	"log"
	"net"
	"sync"
	"time"

	"ts2phc-go/phc"
)

const maxTPAge = 2 * time.Second

// GpsdTPV is the subset of gpsd's TPV JSON we care about.
type GpsdTPV struct {
	Class  string  `json:"class"`
	Mode   int     `json:"mode"`
	Time   string  `json:"time"`
	Lat    float64 `json:"lat"`
	Lon    float64 `json:"lon"`
	AltMSL float64 `json:"alt"`
	AltHAE float64 `json:"altHAE"`
	Eph    float64 `json:"eph"`
	Epv    float64 `json:"epv"`
	Ept    float64 `json:"ept"`
	Speed  float64 `json:"speed"`
	Track  float64 `json:"track"`
	Climb  float64 `json:"climb"`
	Status int     `json:"status"`
	// Leapseconds is the receiver's broadcast GPS-UTC offset (18 as of
	// 2017). It is the one runtime cross-check on the configured TAI
	// offset that comes from the sky rather than a file: TAI-UTC must
	// equal Leapseconds+19. Zero means the field was absent.
	Leapseconds int `json:"leapseconds"`
}

// GpsdSKY is the subset of gpsd's SKY JSON we care about.
type GpsdSKY struct {
	Class      string       `json:"class"`
	GDOP       float64      `json:"gdop"`
	HDOP       float64      `json:"hdop"`
	PDOP       float64      `json:"pdop"`
	TDOP       float64      `json:"tdop"`
	VDOP       float64      `json:"vdop"`
	USat       int          `json:"uSat"`
	Satellites []GpsdSatSV  `json:"satellites"`
}

type GpsdSatSV struct {
	PRN    int     `json:"PRN"`
	GnssID int     `json:"gnssid"`
	SvID   int     `json:"svid"`
	El     float64 `json:"el"`
	Az     float64 `json:"az"`
	Ss     float64 `json:"ss"`
	Used   bool    `json:"used"`
}

// GpsdHandler is called for each parsed gpsd message.
type GpsdHandler interface {
	OnTPV(tpv *GpsdTPV)
	OnSKY(sky *GpsdSKY)
}

// GpsdSource gets PPS reference time from gpsd's JSON TPV stream.
type GpsdSource struct {
	mu       sync.Mutex
	tpTAI    time.Time
	tpRxTime time.Time
	tpValid  bool

	taiOffset int
	addr      string
	handler   GpsdHandler
}

func NewGpsdSource(addr string, taiOffsetSec int, handler GpsdHandler) *GpsdSource {
	return &GpsdSource{
		addr:      addr,
		taiOffset: taiOffsetSec,
		handler:   handler,
	}
}

// Run connects to gpsd and reads JSON messages until an error occurs.
// Should be called in a goroutine.
func (s *GpsdSource) Run() error {
	conn, err := net.DialTimeout("tcp", s.addr, 5*time.Second)
	if err != nil {
		return fmt.Errorf("gpsd connect %s: %w", s.addr, err)
	}
	defer conn.Close()

	watch := []byte(`?WATCH={"enable":true,"json":true}` + "\n")
	if _, err := conn.Write(watch); err != nil {
		return fmt.Errorf("gpsd watch: %w", err)
	}
	log.Printf("gpsd: connected to %s", s.addr)

	scanner := bufio.NewScanner(conn)
	scanner.Buffer(make([]byte, 64*1024), 64*1024)

	for scanner.Scan() {
		line := scanner.Bytes()
		if len(line) == 0 {
			continue
		}

		var peek struct {
			Class string `json:"class"`
		}
		if json.Unmarshal(line, &peek) != nil {
			continue
		}

		switch peek.Class {
		case "TPV":
			var tpv GpsdTPV
			if json.Unmarshal(line, &tpv) != nil {
				continue
			}
			s.processTPV(&tpv)
			if s.handler != nil {
				s.handler.OnTPV(&tpv)
			}
		case "SKY":
			var sky GpsdSKY
			if json.Unmarshal(line, &sky) != nil {
				continue
			}
			if s.handler != nil {
				s.handler.OnSKY(&sky)
			}
		}
	}

	if err := scanner.Err(); err != nil {
		return fmt.Errorf("gpsd read: %w", err)
	}
	return fmt.Errorf("gpsd: connection closed")
}

func (s *GpsdSource) processTPV(tpv *GpsdTPV) {
	if tpv.Mode < 2 || tpv.Time == "" {
		s.mu.Lock()
		s.tpValid = false
		s.mu.Unlock()
		return
	}

	t, err := time.Parse(time.RFC3339Nano, tpv.Time)
	if err != nil {
		return
	}

	// TPV.time is the epoch of the current fix. The next PPS edge fires
	// at the top of the following second. Round to the nearest second --
	// not truncate: a driver reporting the epoch a hair below the second
	// (x.999...) means that second, and truncation would label every
	// pulse one second early. Then add 1s and convert UTC to TAI.
	next := t.Round(time.Second).Add(time.Second)
	tai := next.Add(time.Duration(s.taiOffset) * time.Second)

	s.mu.Lock()
	wasValid := s.tpValid
	s.tpTAI = tai
	s.tpRxTime = time.Now()
	s.tpValid = true
	s.mu.Unlock()

	if !wasValid {
		log.Printf("gpsd: receiving valid time (mode=%d)", tpv.Mode)
	}
}

func (s *GpsdSource) GetPPSTime() (time.Time, error) {
	s.mu.Lock()
	tai := s.tpTAI
	rx := s.tpRxTime
	valid := s.tpValid
	s.mu.Unlock()

	if !valid {
		return time.Time{}, fmt.Errorf("gpsd: no valid TPV received yet")
	}
	if elapsed := time.Since(rx); elapsed > maxTPAge {
		return time.Time{}, fmt.Errorf("gpsd: stale TPV (%v)", elapsed)
	}
	return tai, nil
}

// TAINow estimates current TAI from the last TPV, independent of every clock
// this daemon disciplines -- which is the point: it is the guard's reference
// when the PHC, the system clock, or the PPS pipeline cannot be trusted.
// tpTAI labels the pulse AFTER the fix epoch, so at receive time true TAI was
// tpTAI minus one second plus gpsd's delivery latency within that second.
// The estimate is therefore biased low by that latency -- hundreds of
// milliseconds at worst, never a whole second -- and the elapsed term rides
// on Go's monotonic clock, immune to any system clock step.
func (s *GpsdSource) TAINow() (time.Time, bool) {
	s.mu.Lock()
	tai := s.tpTAI
	rx := s.tpRxTime
	valid := s.tpValid
	s.mu.Unlock()

	if !valid || time.Since(rx) > maxTPAge {
		return time.Time{}, false
	}
	return tai.Add(time.Since(rx) - time.Second), true
}

func (s *GpsdSource) GetClock() *phc.Device { return nil }
func (s *GpsdSource) Destroy()              {}
