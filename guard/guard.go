// Package guard enforces the one invariant every other layer can silently
// betray: the PHC must agree with GPS-derived TAI to the second, always.
//
// History, because each incident moved the failure up one layer:
//
//   - 2026-08-14: a link renegotiation reset the PHC underneath a locked
//     servo, which then slewed forever against 37 seconds it could never
//     close. Fixed by the discontinuity guard in servo/pi.go -- a guard that
//     lives inside the sample pipeline.
//
//   - 2026-08-18: the same class of adapter reset ALSO wiped the SDP pin
//     function and the EXTTS request, so no samples arrived at all. The
//     pipeline -- and the discontinuity guard with it -- went blind, and the
//     box served time 37 seconds wrong for over an hour with every indicator
//     green: the servo printed nothing, chrony saw a healthy refclock, and
//     the PHC free-ran on the wrong timescale.
//
// The lesson: a guard that lives inside the measurement pipeline dies with
// the pipeline. This one does not. It compares a direct PHC read against
// gpsd's idea of TAI on every loop tick, needing nothing from EXTTS, the
// servo, or the kernel event FIFO. It also notices the pipeline's own death
// -- no validated pulse for too long -- and demands a re-arm, because the pin
// routing the pipeline was armed with may no longer exist.
//
// The two layers deliberately overlap: sub-second discipline errors belong to
// the servo and its discontinuity guard, whole-second timescale displacement
// belongs here, and pipeline death is caught here even when the timescale is
// still right.
package guard

import "time"

// Action is what the caller must do right now.
type Action int

const (
	// None: the invariant holds and pulses are flowing.
	None Action = iota
	// Rearm: no validated pulse for too long. Restore the pin function and
	// the EXTTS request; the adapter may have dropped both.
	Rearm
	// Step: the PHC is whole seconds away from GPS TAI and has been for
	// several consecutive ticks. Step it back by StepAmount and reset the
	// servo. Waiting for the pipeline would mean serving wrong time for as
	// long as the pipeline stays dead, which history shows can be forever.
	Step
)

type Config struct {
	// PulseTimeout is the validated-pulse silence that triggers a re-arm,
	// and the retry spacing between re-arm attempts.
	PulseTimeout time.Duration
	// AbsThreshold is the |PHC - TAI| beyond which a tick counts as a
	// timescale violation. It must sit above the reference estimate's own
	// slop (sub-second delivery latency plus the occasional one-second TPV
	// mislabel) and below the smallest displacement worth catching; between
	// those, the servo's discontinuity guard owns the range when pulses
	// flow, and drift can't cross it quickly when they don't.
	AbsThreshold time.Duration
	// Violations is how many consecutive violating ticks are required
	// before Step. A single tick is a bad TPV; five in a row is a clock.
	Violations int
}

func DefaultConfig() Config {
	return Config{
		PulseTimeout: 5 * time.Second,
		AbsThreshold: 2 * time.Second,
		Violations:   5,
	}
}

type Guard struct {
	cfg        Config
	lastPulse  time.Time
	lastRearm  time.Time
	violations int
}

// New starts the pulse clock at now: a daemon that never sees a single pulse
// must still reach Rearm on its own.
func New(cfg Config, now time.Time) *Guard {
	return &Guard{cfg: cfg, lastPulse: now}
}

// Pulse records that a validated PPS edge was processed this cycle.
func (g *Guard) Pulse(now time.Time) {
	g.lastPulse = now
}

// PulseAge is how long the pipeline has been silent.
func (g *Guard) PulseAge(now time.Time) time.Duration {
	return now.Sub(g.lastPulse)
}

// Check runs once per loop tick. phcValid/taiValid gate their readings:
// a Step is only ever issued from two live readings, never from a guess --
// a missing reference must look like a fault, not like agreement.
// The returned Duration is the measured PHC-TAI displacement (zero when
// either reading is invalid), for logging and metrics on every tick.
func (g *Guard) Check(now time.Time, phc time.Time, phcValid bool, tai time.Time, taiValid bool) (Action, time.Duration) {
	var err time.Duration
	if phcValid && taiValid {
		err = phc.Sub(tai)
		if err > g.cfg.AbsThreshold || err < -g.cfg.AbsThreshold {
			g.violations++
			if g.violations >= g.cfg.Violations {
				g.violations = 0
				return Step, err
			}
		} else {
			g.violations = 0
		}
	} else {
		g.violations = 0
	}

	if now.Sub(g.lastPulse) > g.cfg.PulseTimeout && now.Sub(g.lastRearm) > g.cfg.PulseTimeout {
		g.lastRearm = now
		return Rearm, err
	}
	return None, err
}

// StepAmount converts a measured displacement into the step to apply: the
// nearest whole second, negated. The sub-second phase is the servo's
// property -- it may be nanoseconds-good even while the second count is 37
// wrong -- and the estimate's own sub-second part is delivery slop, not
// measurement. Every failure this guard exists for (driver reset to the
// system clock, TAI/UTC/GPS confusion, leap mishandling) displaces the clock
// by whole seconds, so the whole-second step corrects the timescale without
// disturbing a phase the servo may have spent hours refining.
func StepAmount(err time.Duration) time.Duration {
	return -err.Round(time.Second)
}
