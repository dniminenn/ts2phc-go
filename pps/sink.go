package pps

import (
	"fmt"
	"log"
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
	Name     string
	Device   *phc.Device
	Channel  uint32
	Polarity uint32

	// Dynamic Edge Filtering State
	state       FilterState
	lastEvent   time.Time
	pulseWidth  time.Duration
	pulsePeriod time.Duration

	// singleEdge is set when Arm() fell back to a rising-only or falling-only
	// EXTTS request. In that mode the card emits exactly one edge per second, so
	// the two-edge pulse-width filter would never observe a short delta and would
	// stay in FilterStateLocking forever; ProcessEvent uses a period check instead.
	singleEdge bool

	// Last validated EXTTS hardware timestamp
	LastValidTS time.Time
	IsAvailable bool

	// Servo state
	Servo servo.Servo
}

func NewSink(name string, chanIdx uint32, polarity uint32, stepThresh, firstStepThresh, discontinuity float64) (*Sink, error) {
	dev, err := phc.Open(name)
	if err != nil {
		return nil, err
	}

	caps, err := dev.GetCaps()
	if err != nil {
		dev.Close()
		return nil, fmt.Errorf("failed to get caps: %v", err)
	}

	fadj, err := dev.GetFreq()
	if err != nil {
		fadj = 0
	}

	cfg := servo.DefaultPiServoConfig()
	cfg.StepThreshold = stepThresh
	cfg.FirstStepThresh = firstStepThresh
	cfg.Discontinuity = discontinuity
	s := servo.NewPiServo(-fadj, float64(caps.MaxAdj), cfg)

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
	// Disable, isolate channel mask, drain stale events, then re-arm.
	// Matches linuxptp ts2phc_pps_sink_create() init sequence.
	s.Device.DisableExtts(s.Channel)

	if err := s.Device.ClearExttsMask(); err == nil {
		s.Device.EnableExttsMaskSingle(s.Channel)
	}

	if err := s.Device.DrainExtts(); err != nil {
		return fmt.Errorf("drain extts fifo: %w", err)
	}

	err := s.Device.RequestExtts(s.Channel, s.Polarity)
	if err == nil {
		return nil
	}

	err2 := s.Device.RequestExtts(s.Channel, phc.PTP_RISING_EDGE)
	if err2 == nil {
		fmt.Printf("[%s] both-edge EXTTS failed, armed rising-edge only\n", s.Name)
		s.Polarity = phc.PTP_RISING_EDGE
		s.singleEdge = true
		return nil
	}

	err3 := s.Device.RequestExtts(s.Channel, phc.PTP_FALLING_EDGE)
	if err3 == nil {
		fmt.Printf("[%s] rising-edge EXTTS failed, armed falling-edge only\n", s.Name)
		s.Polarity = phc.PTP_FALLING_EDGE
		s.singleEdge = true
		return nil
	}

	return fmt.Errorf("all EXTTS arm attempts failed (both: %v, rising: %v, falling: %v)", err, err2, err3)
}

func (s *Sink) Destroy() {
	s.Device.DisableExtts(s.Channel)
	s.Device.Close()
}

// ProcessEvent runs the dynamic edge filter on every event.
// The filter always runs because some cards deliver both edges regardless
// of the polarity requested in the EXTTS request.
func (s *Sink) ProcessEvent(event phc.ExttsEvent, forceIgnore bool) (time.Time, bool) {
	now := time.Unix(event.Time.Sec, int64(event.Time.NSec))

	if forceIgnore {
		s.lastEvent = now
		return now, false
	}

	// Single-edge mode: exactly one edge per second, so every event is a true
	// start-of-second. There is no trailing edge to discard; we only sanity-check
	// that consecutive edges are ~1s apart and reject obvious glitches/missed
	// interrupts. The two-edge filter below is bypassed entirely.
	if s.singleEdge {
		if s.state == FilterStateInit {
			s.lastEvent = now
			s.state = FilterStateLocked
			return now, false // need a prior event to measure the period
		}
		delta := now.Sub(s.lastEvent)
		s.lastEvent = now
		drift := delta - s.pulsePeriod
		if drift > 100*time.Millisecond || drift < -100*time.Millisecond {
			return now, false
		}
		return now, true
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
			log.Printf("[%s] edge filter locked, pulse width %v", s.Name, s.pulseWidth)
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
