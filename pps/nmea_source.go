package pps

import (
	"bufio"
	"fmt"
	"io"
	"log"
	"net"
	"os"
	"sync"
	"time"

	"golang.org/x/sys/unix"

	"ts2phc-go/nmea"
	"ts2phc-go/phc"
)

const maxRMCAge = 5 * time.Second

type NMEAConfig struct {
	SerialPort string
	BaudRate   int
	RemoteHost string
	RemotePort string
	DelayNS    int
	TAIOffset  int
}

type NMEASource struct {
	mu       sync.Mutex
	rxTime   time.Time // time.Now() at RMC receipt (monotonic reading used by time.Since)
	rmcTime  time.Time // UTC from RMC sentence
	fixValid bool

	delay     time.Duration
	taiOffset time.Duration
	conn      io.ReadCloser
	wg        sync.WaitGroup
}

func NewNMEASource(cfg NMEAConfig) (*NMEASource, error) {
	var conn io.ReadCloser
	var err error

	if cfg.RemoteHost != "" && cfg.RemotePort != "" {
		addr := net.JoinHostPort(cfg.RemoteHost, cfg.RemotePort)
		conn, err = net.DialTimeout("tcp", addr, 5*time.Second)
		if err != nil {
			return nil, fmt.Errorf("nmea tcp %s: %w", addr, err)
		}
		log.Printf("NMEA: connected to %s", addr)
	} else {
		conn, err = openSerial(cfg.SerialPort, cfg.BaudRate)
		if err != nil {
			return nil, err
		}
		log.Printf("NMEA: opened %s at %d baud", cfg.SerialPort, cfg.BaudRate)
	}

	s := &NMEASource{
		delay:     time.Duration(cfg.DelayNS) * time.Nanosecond,
		taiOffset: time.Duration(cfg.TAIOffset) * time.Second,
		conn:      conn,
	}
	s.wg.Add(1)
	go s.monitor()
	return s, nil
}

func (s *NMEASource) monitor() {
	defer s.wg.Done()
	r := bufio.NewReader(s.conn)
	for {
		line, err := r.ReadString('\n')
		if err != nil {
			log.Printf("NMEA: read error: %v", err)
			return
		}
		rmc, ok := nmea.ParseRMC(line)
		if !ok {
			continue
		}
		now := time.Now()
		s.mu.Lock()
		s.rxTime = now
		s.rmcTime = rmc.Time
		s.fixValid = rmc.FixValid
		s.mu.Unlock()
	}
}

// GetPPSTime returns the current UTC estimate derived from the last RMC fix
// plus monotonic elapsed time since receipt, plus the configured delay correction.
func (s *NMEASource) GetPPSTime() (time.Time, error) {
	s.mu.Lock()
	rx := s.rxTime
	rmc := s.rmcTime
	valid := s.fixValid
	s.mu.Unlock()

	if rx.IsZero() {
		return time.Time{}, fmt.Errorf("nmea: no data received yet")
	}
	if !valid {
		return time.Time{}, fmt.Errorf("nmea: no valid fix")
	}
	elapsed := time.Since(rx) // monotonic
	if elapsed > maxRMCAge {
		return time.Time{}, fmt.Errorf("nmea: rmc stale (%v)", elapsed)
	}
	return rmc.Add(elapsed).Add(s.delay).Add(s.taiOffset), nil
}

func (s *NMEASource) GetClock() *phc.Device { return nil }

func (s *NMEASource) Destroy() {
	s.conn.Close()
	s.wg.Wait()
}

// --- serial port helper ---

func openSerial(path string, baud int) (io.ReadCloser, error) {
	fd, err := unix.Open(path, unix.O_RDWR|unix.O_NOCTTY|unix.O_NONBLOCK, 0)
	if err != nil {
		return nil, fmt.Errorf("open %s: %w", path, err)
	}

	t, err := unix.IoctlGetTermios(fd, unix.TCGETS)
	if err != nil {
		// Not a tty (FIFO, regular file, etc.) — just use it as-is.
		if err := unix.SetNonblock(fd, false); err != nil {
			unix.Close(fd)
			return nil, fmt.Errorf("set blocking %s: %w", path, err)
		}
		log.Printf("NMEA: %s is not a tty, skipping termios setup", path)
		return os.NewFile(uintptr(fd), path), nil
	}

	t.Iflag &^= unix.IGNBRK | unix.BRKINT | unix.PARMRK | unix.ISTRIP |
		unix.INLCR | unix.IGNCR | unix.ICRNL | unix.IXON
	t.Oflag &^= unix.OPOST
	t.Lflag &^= unix.ECHO | unix.ECHONL | unix.ICANON | unix.ISIG | unix.IEXTEN
	t.Cflag &^= unix.CSIZE | unix.PARENB
	t.Cflag |= unix.CS8 | unix.CLOCAL | unix.CREAD

	speed, ok := baudConst(baud)
	if !ok {
		unix.Close(fd)
		return nil, fmt.Errorf("unsupported baud rate %d", baud)
	}
	t.Cflag &^= unix.CBAUD
	t.Cflag |= speed
	t.Ispeed = speed
	t.Ospeed = speed

	t.Cc[unix.VMIN] = 1
	t.Cc[unix.VTIME] = 0

	if err := unix.IoctlSetTermios(fd, unix.TCSETS, t); err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("tcsets %s: %w", path, err)
	}

	if err := unix.SetNonblock(fd, false); err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("set blocking %s: %w", path, err)
	}

	return os.NewFile(uintptr(fd), path), nil
}

func baudConst(baud int) (uint32, bool) {
	switch baud {
	case 4800:
		return unix.B4800, true
	case 9600:
		return unix.B9600, true
	case 19200:
		return unix.B19200, true
	case 38400:
		return unix.B38400, true
	case 57600:
		return unix.B57600, true
	case 115200:
		return unix.B115200, true
	default:
		return 0, false
	}
}
