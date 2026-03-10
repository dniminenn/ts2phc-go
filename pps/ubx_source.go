package pps

import (
	"fmt"
	"sync"
	"time"

	"ts2phc-go/phc"
)

const maxPVTAge = 5 * time.Second

// UBXSource implements Source using UTC time from UBX NAV-PVT messages.
// The demux handler calls SetTime on each NavPVT; GetPPSTime returns the
// extrapolated UTC+TAI time for the PPS discipline loop.
type UBXSource struct {
	mu        sync.Mutex
	rxTime    time.Time // time.Now() when PVT was received
	utcTime   time.Time // UTC from NavPVT
	fixValid  bool
	taiOffset time.Duration
}

func NewUBXSource(taiOffsetSec int) *UBXSource {
	return &UBXSource{
		taiOffset: time.Duration(taiOffsetSec) * time.Second,
	}
}

// SetTime is called by the demux handler when a NavPVT with a valid fix arrives.
func (s *UBXSource) SetTime(utc time.Time, fixOK bool) {
	s.mu.Lock()
	s.rxTime = time.Now()
	s.utcTime = utc
	s.fixValid = fixOK
	s.mu.Unlock()
}

func (s *UBXSource) GetPPSTime() (time.Time, error) {
	s.mu.Lock()
	rx := s.rxTime
	utc := s.utcTime
	valid := s.fixValid
	s.mu.Unlock()

	if rx.IsZero() {
		return time.Time{}, fmt.Errorf("ubx: no PVT received yet")
	}
	if !valid {
		return time.Time{}, fmt.Errorf("ubx: no valid fix")
	}
	elapsed := time.Since(rx)
	if elapsed > maxPVTAge {
		return time.Time{}, fmt.Errorf("ubx: PVT stale (%v)", elapsed)
	}
	return utc.Add(elapsed).Add(s.taiOffset), nil
}

func (s *UBXSource) GetClock() *phc.Device { return nil }
func (s *UBXSource) Destroy()              {}
