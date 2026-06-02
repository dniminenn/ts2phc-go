package phc

import (
	"fmt"
	"os"
	"reflect"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/unix"
)

// Constants for PTP ioctls
const (
	PTP_CLOCK_GETCAPS       = 0x80503d01
	PTP_EXTTS_REQUEST       = 0x40103d02
	PTP_PEROUT_REQUEST      = 0x40383d03
	PTP_ENABLE_PPS          = 0x40043d04
	PTP_SYSOFF_REQUEST      = 0xc0503d05
	PTP_PIN_GETFUNC         = 0xc0603d06
	PTP_PIN_SETFUNC         = 0x40603d07
	PTP_SYS_OFFSET          = 0x40403d05 // _IOW('=', 5, struct ptp_sys_offset)
	PTP_SYS_OFFSET_PRECISE  = 0xc0403d08
	PTP_SYS_OFFSET_EXTENDED = 0xc0483d09

	// NOTE: This is INTENTIONALLY not the real PTP_EXTTS_REQUEST2 number
	// (which is _IOW('=', 11, ...) == 0x40103d0b). Value 0x40103d0a matches no
	// kernel command, so RequestExtts's v2 attempt always returns ENOTTY and
	// falls back to the v1 PTP_EXTTS_REQUEST. That fallback is deliberate: the
	// v1 path never sets PTP_STRICT_FLAGS, and the i210/igb driver requires that
	// — it rejects both-edge requests (EOPNOTSUPP) under strict flags, and the
	// sink's dynamic edge filter depends on receiving both edges. Do not "fix"
	// this to 0x40103d0b without re-validating EXTTS capture on the i210.
	PTP_EXTTS_REQUEST2 = 0x40103d0a // see NOTE above

	PTP_MASK_CLEAR_ALL  = 0x20003d13 // _IO('=', 19)
	PTP_MASK_EN_SINGLE  = 0x40043d14 // _IOW('=', 20, uint32)
	PTP_MASK_DIS_SINGLE = 0x40043d15 // _IOW('=', 21, uint32)
)

// Clock flags
const (
	PTP_ENABLE_FEATURE = 1 << 0
	PTP_RISING_EDGE    = 1 << 1
	PTP_FALLING_EDGE   = 1 << 2
	PTP_STRICT_FLAGS   = 1 << 3
)

// Pin functions
const (
	PTP_PF_NONE    = 0
	PTP_PF_EXTTS   = 1
	PTP_PF_PEROUT  = 2
	PTP_PF_PHYSYNC = 3
)

type ClockTime struct {
	Sec  int64
	NSec uint32
	_    uint32 // Padding
}

type ExttsRequest struct {
	Index uint32
	Flags uint32
	Rsv0  [2]uint32
}

type ExttsEvent struct {
	Time  ClockTime
	Index uint32
	Flags uint32
	Rsv0  [2]uint32
}

type PinDesc struct {
	Name  [64]byte
	Index uint32
	Func  uint32
	Chan  uint32
	Rsv0  [5]uint32
}

// ClockCaps mirrors struct ptp_clock_caps from <linux/ptp_clock.h>.
// Field order and count must match the kernel exactly: PTP_CLOCK_GETCAPS
// copies sizeof(struct ptp_clock_caps) == 80 bytes (20 ints) into this
// buffer, so a short struct gets memory written past its end.
type ClockCaps struct {
	MaxAdj            int32
	NAlarm            int32
	NExtTs            int32
	NPerOut           int32
	PPS               int32
	NPins             int32
	CrossTimestamping int32 // Since Linux 5.8
	AdjustPhase       int32
	MaxPhaseAdj       int32
	Rsv               [11]int32
}

type Device struct {
	file *os.File
}

func Open(devPath string) (*Device, error) {
	f, err := os.OpenFile(devPath, os.O_RDWR, 0)
	if err != nil {
		return nil, fmt.Errorf("failed to open %s: %w", devPath, err)
	}
	return &Device{file: f}, nil
}

func (d *Device) Close() error {
	return d.file.Close()
}

func (d *Device) Fd() uintptr {
	return d.file.Fd()
}

// ClockID returns the POSIX clock ID for this PHC device.
// This is equivalent to the kernel's FD_TO_CLOCKID macro:
//
//	((~(clockid_t)(fd) << 3) | 3)
func (d *Device) ClockID() int32 {
	fd := int32(d.file.Fd())
	return (^fd << 3) | 3
}

func (d *Device) Name() string {
	return d.file.Name()
}

func (d *Device) GetTime() (time.Time, error) {
	var ts unix.Timespec
	err := unix.ClockGettime(d.ClockID(), &ts)
	if err != nil {
		return time.Time{}, fmt.Errorf("clock_gettime failed: %w", err)
	}
	return time.Unix(int64(ts.Sec), int64(ts.Nsec)), nil
}

func (d *Device) GetFreq() (float64, error) {
	var tx unix.Timex
	_, err := unix.ClockAdjtime(d.ClockID(), &tx)
	if err != nil {
		return 0, fmt.Errorf("clock_adjtime read freq failed: %w", err)
	}
	return float64(tx.Freq) / 65.536, nil
}

func (d *Device) AdjFreq(ppb float64) error {
	var tx unix.Timex
	tx.Modes = unix.ADJ_FREQUENCY
	setSignedField(&tx, "Freq", int64(ppb*65.536)) // ppb to scaled ppm
	_, err := unix.ClockAdjtime(d.ClockID(), &tx)
	if err != nil {
		return fmt.Errorf("clock_adjtime ADJ_FREQUENCY failed: %w", err)
	}
	return nil
}

// StepTime steps the PHC clock by the given offset in nanoseconds.
// Uses clock_adjtime with ADJ_SETOFFSET | ADJ_NANO.
func (d *Device) StepTime(offsetNS int64) error {
	var tx unix.Timex
	tx.Modes = unix.ADJ_SETOFFSET | unix.ADJ_NANO

	sign := int64(1)
	step := offsetNS
	if step < 0 {
		sign = -1
		step = -step
	}
	setSignedField(&tx.Time, "Sec", sign*(step/1e9))
	setSignedField(&tx.Time, "Usec", sign*(step%1e9))

	// The kernel requires tv_usec to be non-negative
	if tx.Time.Usec < 0 {
		tx.Time.Sec -= 1
		tx.Time.Usec += 1000000000
	}

	_, err := unix.ClockAdjtime(d.ClockID(), &tx)
	if err != nil {
		return fmt.Errorf("clock_adjtime ADJ_SETOFFSET failed: %w", err)
	}
	return nil
}

func setSignedField(ptr any, field string, value int64) {
	v := reflect.ValueOf(ptr).Elem().FieldByName(field)
	if !v.IsValid() || !v.CanSet() {
		return
	}
	min := -(int64(1) << (v.Type().Bits() - 1))
	max := (int64(1) << (v.Type().Bits() - 1)) - 1
	if value < min {
		value = min
	} else if value > max {
		value = max
	}
	v.SetInt(value)
}

func (d *Device) GetCaps() (ClockCaps, error) {
	var caps ClockCaps
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.file.Fd(), uintptr(PTP_CLOCK_GETCAPS), uintptr(unsafe.Pointer(&caps)))
	if errno != 0 {
		return caps, fmt.Errorf("ioctl PTP_CLOCK_GETCAPS failed: %v", errno)
	}
	return caps, nil
}

func (d *Device) SetPinFunc(desc PinDesc) error {
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.file.Fd(), uintptr(PTP_PIN_SETFUNC), uintptr(unsafe.Pointer(&desc)))
	if errno != 0 {
		return fmt.Errorf("ioctl PTP_PIN_SETFUNC failed: %v", errno)
	}
	return nil
}

func (d *Device) RequestExtts(index uint32, flags uint32) error {
	req := ExttsRequest{
		Index: index,
		Flags: flags | PTP_ENABLE_FEATURE,
	}
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.file.Fd(), uintptr(PTP_EXTTS_REQUEST2), uintptr(unsafe.Pointer(&req)))
	if errno != 0 {
		// Fallback to older ioctl if V2 fails
		_, _, errno2 := syscall.Syscall(syscall.SYS_IOCTL, d.file.Fd(), uintptr(PTP_EXTTS_REQUEST), uintptr(unsafe.Pointer(&req)))
		if errno2 != 0 {
			return fmt.Errorf("ioctl PTP_EXTTS_REQUEST failed: %v (v2 error was %v)", errno2, errno)
		}
	}
	return nil
}

func (d *Device) DisableExtts(index uint32) error {
	req := ExttsRequest{
		Index: index,
		Flags: 0, // No PTP_ENABLE_FEATURE means disable
	}
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.file.Fd(), uintptr(PTP_EXTTS_REQUEST2), uintptr(unsafe.Pointer(&req)))
	if errno != 0 {
		_, _, errno2 := syscall.Syscall(syscall.SYS_IOCTL, d.file.Fd(), uintptr(PTP_EXTTS_REQUEST), uintptr(unsafe.Pointer(&req)))
		if errno2 != 0 {
			return fmt.Errorf("ioctl PTP_EXTTS_REQUEST(disable) failed: %v", errno2)
		}
	}
	return nil
}

func (d *Device) ClearExttsMask() error {
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.file.Fd(), uintptr(PTP_MASK_CLEAR_ALL), 0)
	if errno != 0 {
		return fmt.Errorf("ioctl PTP_MASK_CLEAR_ALL failed: %v", errno)
	}
	return nil
}

func (d *Device) EnableExttsMaskSingle(index uint32) error {
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.file.Fd(), uintptr(PTP_MASK_EN_SINGLE), uintptr(unsafe.Pointer(&index)))
	if errno != 0 {
		return fmt.Errorf("ioctl PTP_MASK_EN_SINGLE failed: %v", errno)
	}
	return nil
}

func (d *Device) ReadExttsEvent() (ExttsEvent, error) {
	var event ExttsEvent
	b := (*[unsafe.Sizeof(event)]byte)(unsafe.Pointer(&event))[:]
	n, err := d.file.Read(b)
	if err != nil {
		return event, err
	}
	if n != int(unsafe.Sizeof(event)) {
		return event, fmt.Errorf("short read %d, expected %d", n, unsafe.Sizeof(event))
	}
	return event, nil
}

// DrainExtts reads and discards all pending EXTTS events from the FIFO.
func (d *Device) DrainExtts() error {
	pfd := []unix.PollFd{{
		Fd:     int32(d.file.Fd()),
		Events: unix.POLLIN | unix.POLLPRI,
	}}
	for {
		n, err := unix.Poll(pfd, 0)
		if err != nil {
			if err == unix.EINTR {
				continue
			}
			return fmt.Errorf("poll drain: %w", err)
		}
		if n == 0 {
			return nil
		}
		if _, err := d.ReadExttsEvent(); err != nil {
			return err
		}
	}
}
