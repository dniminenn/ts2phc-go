package pps

import (
	"fmt"
	"time"

	"ts2phc-go/phc"
	"ts2phc-go/servo"
)

type FilterState int

const (
	FilterStateInit FilterState = iota
	FilterStateLocking
	FilterStateLocked
)

type Sink struct {
	Name        string
	Device      *phc.Device
	Channel     uint32
	Polarity    uint32

	// Dynamic Edge Filtering State
	state       FilterState
	lastEvent   time.Time
	pulseWidth  time.Duration
	pulsePeriod time.Duration

	// Servo state
	Servo servo.Servo
}

func NewSink(name string, chanIdx uint32, polarity uint32) (*Sink, error) {
	dev, err := phc.Open(name)
	if err != nil {
		return nil, err
	}

	caps, err := dev.GetCaps()
	if err != nil {
		dev.Close()
		return nil, fmt.Errorf("failed to get caps: %v", err)
	}

	s := servo.NewPiServo(0, float64(caps.MaxAdj), 1.0)

	return &Sink{
		Name:        name,
		Device:      dev,
		Channel:     chanIdx,
		Polarity:    polarity,
		state:       FilterStateInit,
		pulsePeriod: time.Second,
		Servo:       s,
	}, nil
}

func (s *Sink) Arm() error {
	return s.Device.RequestExtts(s.Channel, s.Polarity)
}

func (s *Sink) Destroy() {
	s.Device.DisableExtts(s.Channel)
	s.Device.Close()
}

// Returns the true 1PPS edge and a boolean indicating if it's a valid edge to sync against.
func (s *Sink) ProcessEvent(event phc.ExttsEvent, forceIgnore bool) (time.Time, bool) {
	now := time.Unix(event.Time.Sec, int64(event.Time.NSec))

	if s.Polarity != (phc.PTP_RISING_EDGE|phc.PTP_FALLING_EDGE) {
		// Hardware filters edges for us. Trust all incoming events.
		return now, !forceIgnore
	}

	if forceIgnore {
		s.lastEvent = now
		return now, false
	}

	if s.state == FilterStateInit {
		s.lastEvent = now
		s.state = FilterStateLocking
		return now, false // Need at least two events to determine the short pulse width
	}

	delta := now.Sub(s.lastEvent)
	s.lastEvent = now

	// We assume a 1PPS signal (1s period) and that the pulse width is < 0.5s.
	// Therefore, the sequence of deltas will look like: 
	// [PulseWidth], [1s - PulseWidth], [PulseWidth], [1s - PulseWidth]...

	isShortEdge := delta < (s.pulsePeriod / 2)

	if s.state == FilterStateLocking {
		if isShortEdge {
			s.pulseWidth = delta
			// We just saw the end of the short pulse.
			// This means the previous event was the true start of the second.
			// We can lock onto the PATTERN now: the TRUE edge is followed immediately by a short delta.
			// By definition, the edge that *caused* this short delta is the UNWANTED trailing edge of the pulse.
			s.state = FilterStateLocked
			// fmt.Printf("[%s] Locked dynamic edge. Pulse width: %v\n", s.Name, s.pulseWidth)
			return now, false
		} else {
			// This was the long gap between pulses. The current edge 'now' is the true start
			// of the next pulse (the true 1s marker), but we need to see the short gap to be sure 
			// we are locked onto the correct phase.
		}
		return now, false
	}

	if s.state == FilterStateLocked {
		// If the delta is short, this is the trailing edge of the pulse. Ignore it.
		if isShortEdge {
			// Update the running measurement of the pulse width
			s.pulseWidth = (s.pulseWidth + delta) / 2
			return now, false
		}

		// If the delta is long, this is the true start of the new 1 second pulse.
		// Calculate the jitter/drift against the expected period (1 second - pulseWidth)
		expectedDelta := s.pulsePeriod - s.pulseWidth
		drift := delta - expectedDelta
		
		// If the drift is massive relative to the expected 1s period, we lost lock 
		// (e.g., missed an interrupt, cable disconnected).
		if drift > 100*time.Millisecond || drift < -100*time.Millisecond {
			// fmt.Printf("[%s] Lost lock. Re-initializing filter.\n", s.Name)
			s.state = FilterStateInit
			return now, false
		}

		// This is a valid, true start-of-second edge.
		return now, true
	}

	return now, false
}
