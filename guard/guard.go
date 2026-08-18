// Package guard enforces the invariant every other layer can silently betray:
// the PHC must agree with GPS-derived TAI to the second, always.
//
// History, because each incident moved the failure up one layer:
//
//   - 2026-08-14: a link renegotiation reset the PHC underneath a locked
//     servo, which slewed forever against 37 seconds it could never close.
//     Fixed by the discontinuity guard in servo/pi.go -- inside the pipeline.
//
//   - 2026-08-18: the same class of adapter reset ALSO wiped the SDP pin
//     function and the EXTTS request, so no samples arrived at all. The
//     pipeline -- and the discontinuity guard with it -- went blind, and the
//     box served time 37 seconds wrong for over an hour, all green.
//
// A guard that lives inside the measurement pipeline dies with the pipeline.
// This one compares a direct PHC read against gpsd's idea of TAI on every
// loop tick, needing nothing from EXTTS, the servo, or the kernel event
// FIFO. It also notices the pipeline's own death (no validated pulse for too
// long) and demands a re-arm, because the pin routing the pipeline was armed
// with may no longer exist.
//
// The reference (pps.GpsdSource.TAINow) is biased low by the fix-to-delivery
// latency of the newest TPV. That bias is unbounded in principle, so it is
// measured rather than assumed: while the servo is locked on validated
// pulses the PHC is within microseconds of pulse-truth, which makes the
// guard's raw error a direct observation of the bias. Calibrate feeds those
// observations into an EMA; Check judges the bias-compensated error. Until
// enough calibration samples exist, a wider uncompensated threshold applies.
//
// Structural limits, stated so nobody rediscovers them in production:
// the reference shares gpsd and the TAI offset with the pulse-labeling path,
// so an error common to both (receiver serving wrong UTC, wrong leap count)
// is invisible here and must be caught by the external cross-checks
// (ntpqual, chrony's NTP sources, the TPV leapseconds consistency metric).
// And a sustained delivery latency that happens to sit within Ambiguity of a
// whole second can still produce a wrong step; the ambiguity gate reduces
// that to a coincidence, it cannot eliminate it.
package guard

import "time"

// Action is what the caller must do right now.
type Action int

const (
	// None: the invariant holds, or there is nothing valid to judge.
	None Action = iota
	// Rearm: no validated pulse for too long. Restore the pin function and
	// the EXTTS request; the adapter may have dropped both.
	Rearm
	// Step: the PHC is whole seconds from GPS TAI, confirmed across spaced
	// violations. Step by StepAmount, reset the servo, clear any latched
	// pre-step sample. Waiting for the pipeline would mean serving wrong
	// time for as long as it stays dead, which history shows is forever.
	Step
	// Refuse: the displacement is confirmed but stepping would be a guess
	// -- beyond MaxStep, or not close enough to a whole second for the
	// rounding to be trusted. A wrong step is the one unforgivable act.
	// Alarm loudly; a human or a healthier reference finishes the job.
	Refuse
)

type Config struct {
	// PulseTimeout is the validated-pulse silence that triggers a re-arm,
	// and the retry spacing between attempts.
	PulseTimeout time.Duration
	// AbsThreshold is the violation threshold on the bias-compensated
	// error once calibrated. Real displacements are whole seconds; the
	// compensated error of a healthy clock is EMA noise.
	AbsThreshold time.Duration
	// UncalThreshold applies to the raw error before calibration. It must
	// sit above the worst plausible uncompensated bias.
	UncalThreshold time.Duration
	// Violations is how many violating ticks are required before acting.
	// Invalid ticks (no reference, no PHC read) leave the count where it
	// is: every counted violation was a real observation, and a flapping
	// fix must not reset the count between them. Only a healthy reading
	// clears it.
	Violations int
	// ViolationSpacing is the minimum time between counted violations, so
	// an EXTTS glitch storm ticking the loop at tens of Hz cannot burn the
	// whole count against a single TPV.
	ViolationSpacing time.Duration
	// MaxStep bounds a corrective step. Every displacement this guard
	// exists for is tens of seconds; an apparent error beyond this says
	// the reference is broken, not the clock.
	MaxStep time.Duration
	// Ambiguity is how close the compensated error must be to a whole
	// second for a step to be trusted. Real faults displace by whole
	// seconds; a fractional residue this large means the bias model has
	// failed and the step could land a second short -- or on a healthy
	// clock.
	Ambiguity time.Duration
	// BiasAlpha is the calibration EMA weight per sample.
	BiasAlpha float64
	// BiasSamples is how many calibration samples make the bias trusted.
	BiasSamples int
	// MaxBias rejects calibration samples beyond plausibility, so a
	// mislabeled interval cannot poison the model.
	MaxBias time.Duration
}

func DefaultConfig() Config {
	return Config{
		PulseTimeout:     5 * time.Second,
		AbsThreshold:     750 * time.Millisecond,
		UncalThreshold:   2 * time.Second,
		Violations:       5,
		ViolationSpacing: 500 * time.Millisecond,
		MaxStep:          500 * time.Second,
		Ambiguity:        150 * time.Millisecond,
		BiasAlpha:        0.05,
		BiasSamples:      30,
		MaxBias:          1500 * time.Millisecond,
	}
}

type Guard struct {
	cfg           Config
	lastPulse     time.Time
	lastRearm     time.Time
	lastViolation time.Time
	violations    int
	bias          float64 // seconds, EMA of raw error while servo-locked
	biasN         int
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

// Calibrate feeds one bias observation: the raw PHC-minus-TAINow error taken
// on a tick where the servo just consumed a validated sample in a locked
// state. In that state the PHC is within the outlier gate (50 us) of
// pulse-truth, so the raw error IS the reference bias to within noise.
// Callers must not feed anything else.
func (g *Guard) Calibrate(raw time.Duration) {
	if raw > g.cfg.MaxBias || raw < -g.cfg.MaxBias {
		return
	}
	s := raw.Seconds()
	if g.biasN == 0 {
		g.bias = s
	} else {
		g.bias += g.cfg.BiasAlpha * (s - g.bias)
	}
	if g.biasN < g.cfg.BiasSamples {
		g.biasN++
	}
}

// Calibrated reports whether enough samples back the bias model.
func (g *Guard) Calibrated() bool { return g.biasN >= g.cfg.BiasSamples }

// Bias is the current reference-bias estimate.
func (g *Guard) Bias() time.Duration {
	return time.Duration(g.bias * float64(time.Second))
}

// Check runs once per loop tick. phcValid/taiValid gate their readings: an
// action is only ever taken from two live readings, never from a guess -- a
// missing reference must look like a fault, not like agreement. The returned
// Duration is the error the decision was made on (bias-compensated once
// calibrated, raw before), zero when either reading is invalid.
func (g *Guard) Check(now time.Time, phc time.Time, phcValid bool, tai time.Time, taiValid bool) (Action, time.Duration) {
	var err time.Duration
	if phcValid && taiValid {
		raw := phc.Sub(tai)
		threshold := g.cfg.UncalThreshold
		err = raw
		if g.Calibrated() {
			threshold = g.cfg.AbsThreshold
			err = raw - g.Bias()
		}
		if err > threshold || err < -threshold {
			if g.lastViolation.IsZero() || now.Sub(g.lastViolation) >= g.cfg.ViolationSpacing {
				g.lastViolation = now
				g.violations++
			}
			if g.violations >= g.cfg.Violations {
				g.violations = 0
				if !g.stepTrustworthy(err) {
					return Refuse, err
				}
				return Step, err
			}
		} else {
			g.violations = 0
		}
	}
	// Invalid readings leave the count untouched: see Config.Violations.

	if now.Sub(g.lastPulse) > g.cfg.PulseTimeout && now.Sub(g.lastRearm) > g.cfg.PulseTimeout {
		g.lastRearm = now
		return Rearm, err
	}
	return None, err
}

// StepAmount converts a measured displacement into the step to apply: the
// nearest whole second, negated. Sub-second phase is the servo's property --
// it may be nanoseconds-good even while the second count is 37 wrong -- and
// the correction must not disturb it.
func StepAmount(err time.Duration) time.Duration {
	return -err.Round(time.Second)
}

// stepTrustworthy is the last check before the one action that can make
// things worse. Magnitude beyond MaxStep means the reference is broken (a
// stale estimate surviving a suspend, a replayed stream). A compensated
// error further than Ambiguity from a whole second means the bias model has
// failed -- the step could land a second short and quiesce wrong, or hit a
// healthy clock.
func (g *Guard) stepTrustworthy(err time.Duration) bool {
	step := StepAmount(err)
	if step > g.cfg.MaxStep || step < -g.cfg.MaxStep {
		return false
	}
	frac := err - err.Round(time.Second)
	if frac < 0 {
		frac = -frac
	}
	return frac <= g.cfg.Ambiguity
}
