package pps

import (
	"fmt"
	"log"
	"sync"
	"time"

	"ts2phc-go/phc"
	"ts2phc-go/ubx"
)

const maxTPAge = 2 * time.Second

// UBXSource implements Source using UBX-TIM-TP messages.
// TIM-TP gives the exact TAI second of the next timepulse, directly
// coupled to the PPS output pin. It is present whenever PPS is active.
type UBXSource struct {
	mu        sync.Mutex
	tpTAI     time.Time // TAI second of the timepulse
	tpRxTime  time.Time // time.Now() when TIM-TP was received
	tpValid   bool
	taiOffset int // TAI-UTC seconds (for UTC timebase)
}

func NewUBXSource(taiOffsetSec int) *UBXSource {
	return &UBXSource{
		taiOffset: taiOffsetSec,
	}
}

// SetTimTP is called by the demux handler on each TIM-TP message.
func (s *UBXSource) SetTimTP(tp *ubx.TimTP) {
	locked := !tp.TpNotLocked()

	s.mu.Lock()
	wasValid := s.tpValid
	if locked {
		s.tpTAI = tp.ToTAI(s.taiOffset)
		s.tpRxTime = time.Now()
		s.tpValid = true
	} else {
		s.tpValid = false
	}
	s.mu.Unlock()

	if locked && !wasValid {
		log.Printf("tim-tp: timepulse locked to GNSS")
	} else if !locked && wasValid {
		log.Printf("tim-tp: timepulse NOT locked (holdover/local)")
	}
}

func (s *UBXSource) GetPPSTime() (time.Time, error) {
	s.mu.Lock()
	tai := s.tpTAI
	rx := s.tpRxTime
	valid := s.tpValid
	s.mu.Unlock()

	if !valid {
		return time.Time{}, fmt.Errorf("tim-tp: no message received yet")
	}
	if elapsed := time.Since(rx); elapsed > maxTPAge {
		return time.Time{}, fmt.Errorf("tim-tp: stale (%v)", elapsed)
	}
	return tai, nil
}

func (s *UBXSource) GetClock() *phc.Device { return nil }
func (s *UBXSource) Destroy()              {}
