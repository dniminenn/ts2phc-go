package pps

import (
	"testing"
	"time"

	"ts2phc-go/guard"
)

// TestTAINowStepComposition pins the arithmetic chain the guard's corrective
// step depends on: TPV epoch -> processTPV labeling -> TAINow -> guard error
// -> StepAmount. Every guard unit test fabricates the TAI value directly, so
// a one-second bias slipped into TAINow (the -1s adjustment dropped or
// doubled, the +1s next-edge label changed) would pass that whole suite
// while making every real corrective step land a second wrong and quiesce
// below threshold -- the exact silently-wrong failure class of 90ab05c.
// This test drives the real code and asserts the corrected clock lands on
// true TAI.
func TestTAINowStepComposition(t *testing.T) {
	const taiOffset = 37
	s := NewGpsdSource("", taiOffset, nil)

	fixEpoch := time.Date(2026, 8, 18, 20, 0, 0, 0, time.UTC)
	s.processTPV(&GpsdTPV{Mode: 3, Time: fixEpoch.Format(time.RFC3339Nano)})

	tai, ok := s.TAINow()
	if !ok {
		t.Fatal("TAINow invalid immediately after a valid TPV")
	}

	// Synthetic delivery latency is ~0 (processTPV to TAINow in
	// microseconds), so true TAI now is fixEpoch + taiOffset (+ the
	// microseconds of test execution, absorbed by the tolerance).
	trueTAI := fixEpoch.Add(taiOffset * time.Second)
	if d := tai.Sub(trueTAI); d < -50*time.Millisecond || d > 50*time.Millisecond {
		t.Fatalf("TAINow is %v from true TAI; the labeling arithmetic is biased", d)
	}

	// A PHC displaced -37s (the incident) measured against this reference
	// must produce a step that lands it back on true TAI, not a second off.
	phc := trueTAI.Add(-37 * time.Second)
	gerr := phc.Sub(tai)
	step := guard.StepAmount(gerr)
	if step != 37*time.Second {
		t.Fatalf("step = %v for a -37s displacement, want exactly +37s", step)
	}
	if d := phc.Add(step).Sub(trueTAI); d < -50*time.Millisecond || d > 50*time.Millisecond {
		t.Fatalf("corrected clock is %v from true TAI", d)
	}
}

// TestProcessTPVRoundsFractionalEpoch: a driver reporting the fix epoch a
// hair below the second (x.999...) means that second; truncation would label
// every pulse one second early and the servo would step the PHC wrong.
func TestProcessTPVRoundsFractionalEpoch(t *testing.T) {
	s := NewGpsdSource("", 37, nil)
	s.processTPV(&GpsdTPV{Mode: 3, Time: "2026-08-18T20:00:00.999Z"})
	tai, ok := s.TAINow()
	if !ok {
		t.Fatal("TAINow invalid")
	}
	want := time.Date(2026, 8, 18, 20, 0, 1+37, 0, time.UTC)
	if d := tai.Sub(want); d < -50*time.Millisecond || d > 50*time.Millisecond {
		t.Fatalf("x.999 epoch mislabeled by %v", d)
	}
}

// TestTAINowStaleAfterMaxTPAge: with no fresh TPV the reference must go
// invalid, never serve a stale estimate as truth.
func TestTAINowStaleAfterMaxTPAge(t *testing.T) {
	s := NewGpsdSource("", 37, nil)
	s.processTPV(&GpsdTPV{Mode: 3, Time: "2026-08-18T20:00:00.000Z"})
	s.mu.Lock()
	s.tpRxTime = time.Now().Add(-maxTPAge - time.Second)
	s.mu.Unlock()
	if _, ok := s.TAINow(); ok {
		t.Fatal("stale TPV still served as a valid TAI reference")
	}
}
