package pps

import (
	"time"

	"ts2phc-go/phc"
)

type SourceType int

const (
	SourceGeneric SourceType = iota
	SourceNMEA
	SourcePHC
)

type Source interface {
	GetPPSTime() (time.Time, error)
	GetClock() *phc.Device
	Destroy()
}

// ----------------------------------------------------
// PHC PPS Source
// ----------------------------------------------------

type PHCSource struct {
	dev *phc.Device
}

func NewPHCSource(devPath string) (*PHCSource, error) {
	dev, err := phc.Open(devPath)
	if err != nil {
		return nil, err
	}
	return &PHCSource{dev: dev}, nil
}

func (s *PHCSource) GetPPSTime() (time.Time, error) {
	return s.dev.GetTime()
}

func (s *PHCSource) GetClock() *phc.Device {
	return s.dev
}

func (s *PHCSource) Destroy() {
	if s.dev != nil {
		s.dev.Close()
	}
}

// ----------------------------------------------------
// Generic PPS Source
// ----------------------------------------------------

type GenericSource struct {
	// Represents an external PPS without ToD information
}

func NewGenericSource() *GenericSource {
	return &GenericSource{}
}

func (s *GenericSource) GetPPSTime() (time.Time, error) {
	// Generic source relies on the local system time as a rough estimate
	// of the PPS start-of-second edge.
	now := time.Now().UTC()
	// Round to the nearest second
	var sec int64
	if now.Nanosecond() > 500000000 {
		sec = now.Unix() + 1
	} else {
		sec = now.Unix()
	}
	return time.Unix(sec, 0), nil
}

func (s *GenericSource) GetClock() *phc.Device {
	return nil
}

func (s *GenericSource) Destroy() {
}
