package servo

import "testing"

// TestDiscontinuityRecoversWithSteppingDisabled reproduces the failure that put
// bigchron 37 seconds out on 2026-08-14.
//
// step_threshold defaults to 0 (stepping disabled once locked) and
// first_step_threshold applies only to the very first update. A NIC link
// renegotiation resets the PHC underneath the running servo, and before this
// guard the servo would slew forever against a discontinuity it could never
// close -- while reporting a healthy sub-microsecond offset the whole time.
func TestDiscontinuityRecoversWithSteppingDisabled(t *testing.T) {
	cfg := DefaultPiServoConfig()
	cfg.StepThreshold = 0       // the shipping default: stepping disabled
	cfg.FirstStepThresh = 20000 // 20 us, first update only
	cfg.Discontinuity = 1e9     // 1 s

	s := NewPiServo(0, 1e6, cfg)

	// Drive it to the locked steady state with small offsets.
	for i := 0; i < 8; i++ {
		s.Sample(int64(100+i), uint64(i+1)*2000*uint64(1e9), 1.0)
	}
	if !s.IsLocked() {
		t.Fatalf("servo did not reach locked state; count=%d", s.count)
	}

	// The PHC is reset underneath us: a 37 second discontinuity.
	_, state := s.Sample(-37*int64(1e9), 20000*uint64(1e9), 1.0)

	if s.IsLocked() {
		t.Error("servo stayed locked across a 37 s discontinuity; it will slew forever")
	}
	if !s.firstUpdate {
		t.Error("firstUpdate was not re-armed, so the next sample cannot step")
	}
	_ = state

	// Next samples must take the first-step path and reach Jump.
	var sawJump bool
	for i := 0; i < 4; i++ {
		_, st := s.Sample(-37*int64(1e9), uint64(22000+i*2000)*uint64(1e9), 1.0)
		if st == Jump {
			sawJump = true
			break
		}
	}
	if !sawJump {
		t.Error("servo never issued a step after the discontinuity")
	}
}

// TestDiscontinuityDoesNotFireOnNormalError: ordinary servo error, even large,
// must still be slewed rather than stepped. Stepping on routine error would
// make the clock jumpy and defeat the point of a servo.
func TestDiscontinuityDoesNotFireOnNormalError(t *testing.T) {
	cfg := DefaultPiServoConfig()
	cfg.StepThreshold = 0
	cfg.FirstStepThresh = 20000
	cfg.Discontinuity = 1e9

	s := NewPiServo(0, 1e6, cfg)
	for i := 0; i < 8; i++ {
		s.Sample(int64(100+i), uint64(i+1)*2000*uint64(1e9), 1.0)
	}
	if !s.IsLocked() {
		t.Fatal("servo did not lock")
	}
	// 10 ms is huge for a servo but far below a clock reset.
	s.Sample(10*int64(1e6), 20000*uint64(1e9), 1.0)
	if !s.IsLocked() {
		t.Error("a 10 ms error tripped the discontinuity guard; it must only catch clock resets")
	}
}

// TestWithoutGuardServoStaysTrapped documents the defect this guard fixes, so
// that removing the guard fails the suite rather than silently reintroducing a
// 37-second outage. With Discontinuity disabled and StepThreshold at its
// shipping default of 0, the servo remains "locked" across a clock reset and
// will slew against it forever.
func TestWithoutGuardServoStaysTrapped(t *testing.T) {
	cfg := DefaultPiServoConfig()
	cfg.StepThreshold = 0
	cfg.FirstStepThresh = 20000
	cfg.Discontinuity = 0 // guard disabled: the old behaviour

	s := NewPiServo(0, 1e6, cfg)
	for i := 0; i < 8; i++ {
		s.Sample(int64(100+i), uint64(i+1)*2000*uint64(1e9), 1.0)
	}
	if !s.IsLocked() {
		t.Fatal("servo did not lock")
	}
	s.Sample(-37*int64(1e9), 20000*uint64(1e9), 1.0)
	if !s.IsLocked() {
		t.Fatal("unexpected: servo recovered without the guard")
	}
	if s.firstUpdate {
		t.Fatal("unexpected: firstUpdate re-armed without the guard")
	}
	// Trapped: locked, unable to step, slewing against 37 s forever.
}
