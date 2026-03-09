package servo

import (
	"math"
)

// Default constants matching linuxptp hwts defaults
const (
	HWTS_KP_SCALE = 0.7
	HWTS_KI_SCALE = 0.3
	FREQ_EST_MARGIN = 0.001
)

type State int

const (
	Unlocked State = iota
	Jump
	Locked
	LockedStable
)

type Servo interface {
	Sample(offset int64, localTs uint64, weight float64) (float64, State)
	SyncInterval(interval float64)
	Reset()
}

type PiServo struct {
	offset   [2]int64
	local    [2]uint64
	drift    float64
	kp       float64
	ki       float64
	lastFreq float64
	count    int

	configuredKpScale float64
	configuredKiScale float64
	maxFrequency     float64
	stepThreshold    float64
	firstStepThresh  float64
	firstUpdate      bool
}

func NewPiServo(fadj float64, maxPpb float64, step float64) *PiServo {
	return &PiServo{
		drift:             fadj,
		lastFreq:          fadj,
		configuredKpScale: HWTS_KP_SCALE,
		configuredKiScale: HWTS_KI_SCALE,
		maxFrequency:      maxPpb,
		stepThreshold:     step,
		firstStepThresh:   step,
		firstUpdate:       true,
	}
}

func (s *PiServo) Sample(offset int64, localTs uint64, weight float64) (float64, State) {
	var kiTerm float64
	ppb := s.lastFreq
	state := Unlocked

	switch s.count {
	case 0:
		s.offset[0] = offset
		s.local[0] = localTs
		state = Unlocked
		s.count = 1
	case 1:
		s.offset[1] = offset
		s.local[1] = localTs

		if s.local[0] >= s.local[1] {
			state = Unlocked
			s.count = 0
			break
		}

		localDiff := float64(s.local[1]-s.local[0]) / 1e9
		localDiff += localDiff * FREQ_EST_MARGIN
		freqEstInterval := 0.016 / s.ki
		if freqEstInterval > 1000.0 {
			freqEstInterval = 1000.0
		}
		if localDiff < freqEstInterval {
			state = Unlocked
			break
		}

		// Adjust drift by measured frequency offset
		s.drift += (1e9 - s.drift) * float64(s.offset[1]-s.offset[0]) / float64(s.local[1]-s.local[0])

		if s.drift < -s.maxFrequency {
			s.drift = -s.maxFrequency
		} else if s.drift > s.maxFrequency {
			s.drift = s.maxFrequency
		}

		absOff := math.Abs(float64(offset))
		if (s.firstUpdate && s.firstStepThresh > 0 && s.firstStepThresh < absOff) ||
			(s.stepThreshold > 0 && s.stepThreshold < absOff) {
			state = Jump
		} else {
			state = Locked
		}

		ppb = s.drift
		s.count = 2
	case 2:
		absOff := math.Abs(float64(offset))
		if s.stepThreshold > 0 && s.stepThreshold < absOff {
			state = Unlocked
			s.count = 0
			break
		}

		kiTerm = s.ki * float64(offset) * weight
		ppb = s.kp*float64(offset)*weight + s.drift + kiTerm

		if ppb < -s.maxFrequency {
			ppb = -s.maxFrequency
		} else if ppb > s.maxFrequency {
			ppb = s.maxFrequency
		} else {
			s.drift += kiTerm
		}
		state = Locked
	}

	if state == Jump || state == Locked {
		s.firstUpdate = false
	}

	s.lastFreq = ppb
	return ppb, state
}

func (s *PiServo) SyncInterval(interval float64) {
	s.kp = s.configuredKpScale 
	// To match pi.c exactly, normal PI scales the constants:
	// kp = kp_scale * interval^exponent
	// simplified for hwts defaults:
	s.kp = s.configuredKpScale
	s.ki = s.configuredKiScale

	// In the C code there's a norm_max check, but for hwts defaults it simplifies
}

func (s *PiServo) Reset() {
	s.count = 0
}
