package guard

import (
	"testing"
	"time"
)

var t0 = time.Date(2026, 8, 18, 17, 26, 23, 0, time.UTC) // the moment the link bounced

// TestAdapterResetBlindsPipelineThenGuardRecovers replays 2026-08-18: an
// adapter reset steps the PHC 37 seconds (TAI to UTC) and simultaneously
// kills the EXTTS stream, so the servo-level discontinuity guard never sees a
// single sample. The guard must (a) demand a re-arm once the silence exceeds
// the timeout, and (b) step the PHC back after the violation count is met --
// without ever needing the pipeline.
func TestAdapterResetBlindsPipelineThenGuardRecovers(t *testing.T) {
	g := New(DefaultConfig(), t0)
	tai := t0

	// Healthy: pulses flowing, PHC on TAI.
	for i := 0; i < 3; i++ {
		now := t0.Add(time.Duration(i) * time.Second)
		g.Pulse(now)
		if a, _ := g.Check(now, now, true, now, true); a != None {
			t.Fatalf("tick %d: healthy state produced action %v", i, a)
		}
	}

	// The reset: PHC jumps to UTC (TAI-37s), pulses stop. The PHC stays
	// displaced until the step lands, and the pipeline stays dead
	// throughout, so both actions must appear.
	var sawRearm, sawStep bool
	var stepErr time.Duration
	phcDisplacement := -37 * time.Second
	for i := 3; i < 30; i++ {
		now := t0.Add(time.Duration(i) * time.Second)
		phc := now.Add(phcDisplacement)
		a, err := g.Check(now, phc, true, tai.Add(now.Sub(t0)), true)
		switch a {
		case Rearm:
			sawRearm = true
		case Step:
			sawStep = true
			stepErr = err
			phcDisplacement += StepAmount(err) // the caller applies the step
		}
	}
	if !sawRearm {
		t.Error("pipeline dead beyond the timeout, but no Rearm was demanded")
	}
	if !sawStep {
		t.Fatal("PHC sat 37 s from TAI and the guard never stepped it")
	}
	if got := StepAmount(stepErr); got != 37*time.Second {
		t.Errorf("step amount = %v, want exactly +37s", got)
	}
}

// TestGuardIgnoresReferenceSlop: the TAI estimate carries sub-second delivery
// latency and, on a bad TPV, a one-second mislabel. Neither may ever step the
// clock -- a phantom step IS the failure this daemon exists to prevent.
func TestGuardIgnoresReferenceSlop(t *testing.T) {
	g := New(DefaultConfig(), t0)
	offsets := []time.Duration{
		0, -300 * time.Millisecond, time.Second, -time.Second,
		900 * time.Millisecond, time.Second, time.Second, -time.Second,
		time.Second, time.Second, time.Second, time.Second, time.Second,
	}
	for i, off := range offsets {
		now := t0.Add(time.Duration(i) * time.Second)
		g.Pulse(now)
		if a, _ := g.Check(now, now.Add(off), true, now, true); a == Step {
			t.Fatalf("tick %d: %v of reference slop caused a step", i, off)
		}
	}
}

// TestGuardNeedsConsecutiveViolations: one good reading resets the count. A
// clock that is really displaced violates every tick; anything intermittent
// is the reference, not the clock.
func TestGuardNeedsConsecutiveViolations(t *testing.T) {
	cfg := DefaultConfig()
	g := New(cfg, t0)
	tick := 0
	check := func(off time.Duration) Action {
		now := t0.Add(time.Duration(tick) * time.Second)
		tick++
		g.Pulse(now)
		a, _ := g.Check(now, now.Add(off), true, now, true)
		return a
	}
	for i := 0; i < cfg.Violations-1; i++ {
		if a := check(-37 * time.Second); a == Step {
			t.Fatalf("stepped after only %d violations", i+1)
		}
	}
	if a := check(0); a == Step {
		t.Fatal("stepped on a healthy reading")
	}
	for i := 0; i < cfg.Violations-1; i++ {
		if a := check(-37 * time.Second); a == Step {
			t.Fatalf("count survived a healthy reading: stepped after %d", i+1)
		}
	}
	if a := check(-37 * time.Second); a != Step {
		t.Fatalf("did not step after %d consecutive violations", cfg.Violations)
	}
}

// TestGuardNeverStepsWithoutAReference: gpsd invalid or the PHC unreadable
// means there is nothing to judge against. The guard must hold, forever if
// necessary. Stepping on a guess is how 37-second nightmares are made.
func TestGuardNeverStepsWithoutAReference(t *testing.T) {
	g := New(DefaultConfig(), t0)
	for i := 0; i < 100; i++ {
		now := t0.Add(time.Duration(i) * time.Second)
		g.Pulse(now)
		a, _ := g.Check(now, now.Add(-37*time.Second), true, time.Time{}, false)
		if a == Step {
			t.Fatal("stepped with no valid TAI reference")
		}
		a, _ = g.Check(now, time.Time{}, false, now, true)
		if a == Step {
			t.Fatal("stepped with no valid PHC reading")
		}
	}
}

// TestRearmIsRateLimited: a silent pipeline gets one re-arm per timeout
// window, not one per tick. Hammering ioctls at a wedged device helps nobody.
func TestRearmIsRateLimited(t *testing.T) {
	cfg := DefaultConfig()
	g := New(cfg, t0)
	rearms := 0
	// 60 seconds of silence, ticking every second, PHC healthy.
	for i := 1; i <= 60; i++ {
		now := t0.Add(time.Duration(i) * time.Second)
		if a, _ := g.Check(now, now, true, now, true); a == Rearm {
			rearms++
		}
	}
	want := int(60 / (cfg.PulseTimeout / time.Second)) // one per window
	if rearms == 0 || rearms > want {
		t.Errorf("60s of silence produced %d re-arms, want 1..%d", rearms, want)
	}
}

// TestGuardQuiescesAfterRecovery: once the step lands and pulses return,
// the guard must go silent again.
func TestGuardQuiescesAfterRecovery(t *testing.T) {
	g := New(DefaultConfig(), t0)
	// Drive it to a step.
	i := 0
	for {
		now := t0.Add(time.Duration(i) * time.Second)
		a, _ := g.Check(now, now.Add(-37*time.Second), true, now, true)
		i++
		if a == Step {
			break
		}
		if i > 100 {
			t.Fatal("never stepped")
		}
	}
	// Recovered: pulses flow, PHC agrees.
	for j := 0; j < 20; j++ {
		now := t0.Add(time.Duration(i+j) * time.Second)
		g.Pulse(now)
		if a, _ := g.Check(now, now.Add(3*time.Millisecond), true, now, true); a != None {
			t.Fatalf("recovered state still produced action %v", a)
		}
	}
}

// calibrate feeds n identical bias observations.
func calibrate(g *Guard, bias time.Duration, n int) {
	for i := 0; i < n; i++ {
		g.Calibrate(bias)
	}
}

// TestGuardCatchesTwoSecondDisplacementOnceCalibrated replays the f1279c3
// incident magnitude: a PHC exactly 2 s off, with a realistic 400 ms
// reference bias eating into the raw error. The uncompensated guard is blind
// to this band forever (raw error -1.6 s, under every sane raw threshold);
// the calibrated guard must catch it and step exactly 2 s.
func TestGuardCatchesTwoSecondDisplacementOnceCalibrated(t *testing.T) {
	g := New(DefaultConfig(), t0)
	bias := 400 * time.Millisecond
	calibrate(g, bias, DefaultConfig().BiasSamples)
	if !g.Calibrated() {
		t.Fatal("not calibrated after BiasSamples observations")
	}

	var stepped time.Duration
	for i := 0; i < 20; i++ {
		now := t0.Add(time.Duration(i) * time.Second)
		phc := now.Add(-2 * time.Second).Add(bias) // raw = -2s + bias
		if a, err := g.Check(now, phc, true, now, true); a == Step {
			stepped = StepAmount(err)
			break
		} else if a == Refuse {
			t.Fatalf("tick %d: refused a clean whole-second displacement", i)
		}
	}
	if stepped != 2*time.Second {
		t.Fatalf("step = %v, want exactly +2s", stepped)
	}
}

// TestStepSurvivesReferenceFlicker: a 37 s displacement with every third
// reference reading invalid (gpsd reconnect churn, TPV cadence racing the
// staleness gate). Violations must accumulate across the gaps -- a hard
// reset on invalid ticks would let a flapping fix mask the fault forever.
func TestStepSurvivesReferenceFlicker(t *testing.T) {
	g := New(DefaultConfig(), t0)
	for i := 0; i < 30; i++ {
		now := t0.Add(time.Duration(i) * time.Second)
		phc := now.Add(-37 * time.Second)
		taiValid := i%3 != 2
		if a, _ := g.Check(now, phc, true, now, taiValid); a == Step {
			return
		}
	}
	t.Fatal("flickering reference starved the guard; no step in 30 ticks")
}

// TestGlitchStormCannotBurnViolationsOnOneReference: an EXTTS glitch storm
// ticks the loop at tens of Hz, all judging the same TPV. Five "violations"
// inside half a second are one observation, not five; no step may fire.
func TestGlitchStormCannotBurnViolationsOnOneReference(t *testing.T) {
	g := New(DefaultConfig(), t0)
	for i := 0; i < 9; i++ {
		now := t0.Add(time.Duration(i) * 50 * time.Millisecond)
		phc := now.Add(-37 * time.Second)
		if a, _ := g.Check(now, phc, true, now, true); a == Step || a == Refuse {
			t.Fatalf("tick %d (%dms in): acted on violations burned against one reference window", i, i*50)
		}
	}
}

// TestFractionalErrorRefusedNotStepped: sustained delivery lag makes a
// healthy clock read +2.5 s. The error is confirmed but sits nowhere near a
// whole second once compensated -- stepping would displace a correct clock.
// The guard must Refuse (alarm), never Step.
func TestFractionalErrorRefusedNotStepped(t *testing.T) {
	g := New(DefaultConfig(), t0)
	calibrate(g, 100*time.Millisecond, DefaultConfig().BiasSamples)
	var refused bool
	for i := 0; i < 20; i++ {
		now := t0.Add(time.Duration(i) * time.Second)
		phc := now.Add(2600 * time.Millisecond) // comp = +2.5s, frac 0.5
		a, _ := g.Check(now, phc, true, now, true)
		if a == Step {
			t.Fatal("stepped a healthy clock on a fractional (lag-shaped) error")
		}
		if a == Refuse {
			refused = true
		}
	}
	if !refused {
		t.Fatal("confirmed fractional error produced no Refuse alarm")
	}
}

// TestOversizedStepRefused: an error beyond MaxStep means the reference is
// broken (stale estimate after suspend, replayed stream), not the clock.
func TestOversizedStepRefused(t *testing.T) {
	g := New(DefaultConfig(), t0)
	var refused bool
	for i := 0; i < 20; i++ {
		now := t0.Add(time.Duration(i) * time.Second)
		phc := now.Add(-3600 * time.Second)
		a, _ := g.Check(now, phc, true, now, true)
		if a == Step {
			t.Fatal("stepped an hour on a reference that cannot plausibly be right")
		}
		if a == Refuse {
			refused = true
		}
	}
	if !refused {
		t.Fatal("oversized error produced no Refuse alarm")
	}
}

// TestCalibrationRejectsImplausibleBias: a mislabeled interval (whole-second
// wrong labels) must not poison the bias model.
func TestCalibrationRejectsImplausibleBias(t *testing.T) {
	g := New(DefaultConfig(), t0)
	calibrate(g, 200*time.Millisecond, DefaultConfig().BiasSamples)
	calibrate(g, -37*time.Second, 100) // poison attempt
	if b := g.Bias(); b < 100*time.Millisecond || b > 300*time.Millisecond {
		t.Fatalf("bias %v corrupted by implausible samples", b)
	}
}

// TestDefaultConfigPinned: silently widening a threshold or the violation
// count is a one-line change that would pass every behavioral test while
// enlarging the blind window. Pin the numbers; changing them must be loud.
func TestDefaultConfigPinned(t *testing.T) {
	c := DefaultConfig()
	want := Config{
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
	if c != want {
		t.Fatalf("DefaultConfig drifted: got %+v want %+v", c, want)
	}
}

// TestStepAmountRoundsToWholeSeconds: the step must correct the second count
// and leave the servo's phase alone, whatever fraction rides on the estimate.
func TestStepAmountRoundsToWholeSeconds(t *testing.T) {
	cases := []struct {
		err  time.Duration
		want time.Duration
	}{
		{-37 * time.Second, 37 * time.Second},
		{-37*time.Second - 400*time.Millisecond, 37 * time.Second},
		{-36*time.Second - 600*time.Millisecond, 37 * time.Second},
		{37 * time.Second, -37 * time.Second},
		{18*time.Second + 200*time.Millisecond, -18 * time.Second},
		{-19*time.Second + 100*time.Millisecond, 19 * time.Second},
	}
	for _, c := range cases {
		if got := StepAmount(c.err); got != c.want {
			t.Errorf("StepAmount(%v) = %v, want %v", c.err, got, c.want)
		}
	}
}
