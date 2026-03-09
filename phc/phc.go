package phc

import (
	"fmt"
	"os"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/unix"
)

// Constants for PTP ioctls
const (
	PTP_CLOCK_GETCAPS    = 0x80503d01
	PTP_EXTTS_REQUEST    = 0x40183d02
	PTP_PEROUT_REQUEST   = 0x40383d03
	PTP_ENABLE_PPS       = 0x40043d04
	PTP_SYSOFF_REQUEST   = 0xc0503d05
	PTP_PIN_GETFUNC      = 0xc0603d06
	PTP_PIN_SETFUNC      = 0x40603d07
	PTP_SYS_OFFSET       = 0x40403d05 // _IOW('=', 5, struct ptp_sys_offset)
	PTP_SYS_OFFSET_PRECISE = 0xc0403d08
	PTP_SYS_OFFSET_EXTENDED = 0xc0483d09

	PTP_CLOCK_GETTIME  = 0xc0103d01 // _IOR('=', 1, struct ptp_clock_time)
	PTP_CLOCK_SETTIME  = 0x40103d02 // _IOW('=', 2, struct ptp_clock_time)
	PTP_CLOCK_ADJTIME  = 0x40103d03 // _IOW('=', 3, struct ptp_clock_time)
	PTP_CLOCK_ADJFREQ  = 0x40083d04 // _IOW('=', 4, int32)

	PTP_EXTTS_REQUEST2 = 0x40183d0a // _IOW('=', 10, struct ptp_extts_request)

	PTP_MASK_CLEAR_ALL = 0x20003d13 // _IO('=', 19)
	PTP_MASK_EN_SINGLE = 0x40043d14 // _IOW('=', 20, uint32)
	PTP_MASK_DIS_SINGLE= 0x40043d15 // _IOW('=', 21, uint32)
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
	PTP_PF_NONE   = 0
	PTP_PF_EXTTS  = 1
	PTP_PF_PEROUT = 2
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

type ClockCaps struct {
	MaxAdj    int32
	NAlarm    int32
	NExtTs    int32
	NPerOut   int32
	NPins     int32
	PPS       int32
	NTimeStamps int32 // Since Linux 5.8
	NProgMac  int32
	Rsv       [11]int32
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

func (d *Device) Name() string {
	return d.file.Name()
}

func (d *Device) GetTime() (time.Time, error) {
	var ct ClockTime
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.file.Fd(), uintptr(PTP_CLOCK_GETTIME), uintptr(unsafe.Pointer(&ct)))
	if errno != 0 {
		return time.Time{}, fmt.Errorf("ioctl PTP_CLOCK_GETTIME failed: %v", errno)
	}
	return time.Unix(ct.Sec, int64(ct.NSec)), nil
}

func (d *Device) AdjFreq(ppb float64) error {
	adj := int32(ppb)
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.file.Fd(), uintptr(PTP_CLOCK_ADJFREQ), uintptr(unsafe.Pointer(&adj)))
	if errno != 0 {
		return fmt.Errorf("ioctl PTP_CLOCK_ADJFREQ failed: %v", errno)
	}
	return nil
}

func (d *Device) StepTime(offsetNS int64) error {
	var tx unix.Timex
	tx.Modes = unix.ADJ_SETOFFSET | unix.ADJ_NANO
	tx.Time.Sec = offsetNS / 1e9
	tx.Time.Usec = offsetNS % 1e9

	// To use clock_adjtime we need the posix clock id.
	// However, we can also use PTP_CLOCK_ADJTIME directly with a struct ptp_clock_time.
	// Since we are adjusting by an offset, let's use the standard posix clock_adjtime via unix.ClockAdjtime.
	// But getting the dynamic posix clock ID for the opened fd is tricky in Go.
	// Let's use the simpler PTP ioctl:
	var ct ClockTime
	ct.Sec = offsetNS / 1e9
	ct.NSec = uint32(offsetNS % 1e9)
	// Handle negative offsets carefully
	if offsetNS < 0 {
		ct.Sec = (offsetNS / 1e9) - 1
		ct.NSec = uint32(1e9 + (offsetNS % 1e9))
		if ct.NSec == 1e9 {
			ct.Sec++
			ct.NSec = 0
		}
	}

	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.file.Fd(), uintptr(PTP_CLOCK_ADJTIME), uintptr(unsafe.Pointer(&ct)))
	if errno != 0 {
		return fmt.Errorf("ioctl PTP_CLOCK_ADJTIME failed: %v", errno)
	}
	return nil
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
