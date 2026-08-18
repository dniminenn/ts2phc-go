// Package servo implements a PI clock servo.
//
// This file is a Go port of the PI clock servo used by the Linux PTP
// project's ts2phc utility (linuxptp), and is therefore a derivative work.
// It is based on the original implementation by:
//
//	Linux PTP project (linuxptp),
//
// Copyright (C) 2011 Richard Cochran.
//
// This file is licensed under the terms of the GNU General Public License,
// version 2 or (at your option) any later version (GPL-2.0-or-later).
package servo

import (
	"math"
)

const (
	HWTS_KP_SCALE   = 0.7
	HWTS_KI_SCALE   = 0.3
	FREQ_EST_MARGIN = 0.001

	DEFAULT_KP_EXPONENT = -0.3
	DEFAULT_KI_EXPONENT = 0.4
	DEFAULT_KP_NORM_MAX = 0.7
	DEFAULT_KI_NORM_MAX = 0.3

	MAX_KP_NORM_MAX = 1.0
	MAX_KI_NORM_MAX = 2.0
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
	IsLocked() bool
}

type PiServo struct {
	offset   [2]int64
	local    [2]uint64
	drift    float64
	kp       float64
	ki       float64
	lastFreq float64
	count    int

	kpScale    float64
	kpExponent float64
	kpNormMax  float64
	kiScale    float64
	kiExponent float64
	kiNormMax  float64

	maxFrequency    float64
	stepThreshold   float64
	firstStepThresh float64
	discontinuity   float64
	plausibleFreq   float64
	firstUpdate     bool

	offsetThreshold  int64
	numOffsetValues  int
	currOffsetValues int
}

type PiServoConfig struct {
	KpScale    float64
	KpExponent float64
	KpNormMax  float64
	KiScale    float64
	KiExponent float64
	KiNormMax  float64

	StepThreshold   float64
	FirstStepThresh float64
	// Discontinuity is an offset magnitude (ns) above which the servo assumes
	// the clock was reset underneath it rather than that it has drifted.
	Discontinuity float64
	// PlausibleFreq (ppb) bounds the two-sample drift estimate. The outlier
	// gate only protects a locked servo; while unlocked, a mislabeled
	// sample pair (a one-second label race during relock) implies a drift
	// of ~5e8 ppb, which the maxFrequency clamp turns into a full-rate slew
	// -- the clock then physically travels a second-scale excursion before
	// anything recovers. No real crystal is beyond a few hundred ppm; an
	// estimate outside this bound is a bad pair, to be discarded, not
	// slewed. 0 disables.
	PlausibleFreq float64

	OffsetThreshold int64
	NumOffsetValues int
}

func DefaultPiServoConfig() PiServoConfig {
	return PiServoConfig{
		KpScale:    HWTS_KP_SCALE,
		KpExponent: DEFAULT_KP_EXPONENT,
		KpNormMax:  DEFAULT_KP_NORM_MAX,
		KiScale:       HWTS_KI_SCALE,
		KiExponent:    DEFAULT_KI_EXPONENT,
		KiNormMax:     DEFAULT_KI_NORM_MAX,
		PlausibleFreq: 200000, // +/-200 ppm: generous for any real crystal
	}
}

func NewPiServo(fadj float64, maxPpb float64, cfg PiServoConfig) *PiServo {
	return &PiServo{
		drift:            fadj,
		lastFreq:         fadj,
		kpScale:          cfg.KpScale,
		kpExponent:       cfg.KpExponent,
		kpNormMax:        cfg.KpNormMax,
		kiScale:          cfg.KiScale,
		kiExponent:       cfg.KiExponent,
		kiNormMax:        cfg.KiNormMax,
		maxFrequency:     maxPpb,
		stepThreshold:    cfg.StepThreshold,
		firstStepThresh:  cfg.FirstStepThresh,
		discontinuity:    cfg.Discontinuity,
		plausibleFreq:    cfg.PlausibleFreq,
		firstUpdate:      true,
		offsetThreshold:  cfg.OffsetThreshold,
		numOffsetValues:  cfg.NumOffsetValues,
		currOffsetValues: cfg.NumOffsetValues,
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
		s.count = 1

	case 1:
		s.offset[1] = offset
		s.local[1] = localTs

		if s.local[0] >= s.local[1] {
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
			break
		}

		est := s.drift + (1e9-s.drift)*float64(s.offset[1]-s.offset[0])/float64(s.local[1]-s.local[0])
		if s.plausibleFreq > 0 && (est > s.plausibleFreq || est < -s.plausibleFreq) {
			// One of the two samples is garbage (a mislabeled second during
			// relock). Discard the pair and estimate again from scratch;
			// slewing at an impossible rate is how a one-second label race
			// becomes a one-second physical excursion.
			s.count = 0
			break
		}
		s.drift = est

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
		// An offset this large is not servo error, it is the clock having been
		// reset underneath us. A NIC link renegotiation resets the PHC on igb,
		// and any cable bump, switch reboot or autoneg glitch will do it.
		//
		// Without this the servo is trapped: step_threshold defaults to 0, so
		// the check below never fires once locked, and firstStepThresh applies
		// only to the very first update. The servo then slews forever against a
		// discontinuity it can never close, while reporting a healthy
		// sub-microsecond offset -- which is exactly how a box ends up serving
		// time 37 seconds wrong with every indicator green.
		//
		// Reset() re-arms firstUpdate, so the next sample takes the first-step
		// path and steps the clock back into range.
		if s.discontinuity > 0 && absOff > s.discontinuity {
			s.Reset()
			break
		}
		if s.stepThreshold > 0 && s.stepThreshold < absOff {
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

	// Post-processing matching servo_sample() in servo.c
	switch state {
	case Unlocked:
		s.currOffsetValues = s.numOffsetValues
	case Jump:
		s.currOffsetValues = s.numOffsetValues
		s.firstUpdate = false
	case Locked:
		if s.checkOffsetThreshold(offset) {
			state = LockedStable
		}
		s.firstUpdate = false
	}

	s.lastFreq = ppb
	return ppb, state
}

func (s *PiServo) checkOffsetThreshold(offset int64) bool {
	if s.offsetThreshold == 0 {
		return false
	}
	absOff := offset
	if absOff < 0 {
		absOff = -absOff
	}
	if absOff < s.offsetThreshold {
		if s.currOffsetValues > 0 {
			s.currOffsetValues--
		}
	} else {
		s.currOffsetValues = s.numOffsetValues
	}
	return s.currOffsetValues == 0
}

func (s *PiServo) SyncInterval(interval float64) {
	s.kp = s.kpScale * math.Pow(interval, s.kpExponent)
	if s.kp > s.kpNormMax/interval {
		s.kp = s.kpNormMax / interval
	}

	s.ki = s.kiScale * math.Pow(interval, s.kiExponent)
	if s.ki > s.kiNormMax/interval {
		s.ki = s.kiNormMax / interval
	}
}

func (s *PiServo) Reset() {
	s.count = 0
	s.firstUpdate = true
}

func (s *PiServo) IsLocked() bool {
	return s.count >= 2
}
