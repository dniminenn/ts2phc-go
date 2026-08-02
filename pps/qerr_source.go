package pps

import (
	"fmt"
	"log"
	"net"
	"sync"
	"time"

	"ts2phc-go/ubx"
)

// QErrSource recovers the receiver's per-pulse quantization error (UBX-TIM-TP
// qErr) from gpsd's raw passthrough. The timepulse can only land on the
// receiver's clock grid; qErr reports the sub-grid placement error of each
// pulse (a few ns on an M8N/M9N sawtooth), and TIM-TP carries the GPS
// week/tow of the pulse it describes, so each correction is matched to its
// pulse by UTC second — a stale or missing message can never smear the wrong
// pulse.
//
// This is a second, read-only gpsd connection (?WATCH raw=2): raw UBX frames
// interleave with NMEA text and would corrupt the JSON line scanner on the
// main connection.
type QErrSource struct {
	addr      string
	gpsUTCOff int64 // GPS-UTC offset in seconds (taiOffset - 19)

	mu   sync.Mutex
	ring [8]struct {
		sec int64 // UTC second of the pulse this qErr describes
		ns  float64
	}
	head int

	// OnTimTP, if set, is invoked for every valid TIM-TP (metrics hook).
	OnTimTP func(*ubx.TimTP)
}

const gpsEpochUnix = 315964800

func NewQErrSource(addr string, taiOffset int) *QErrSource {
	return &QErrSource{addr: addr, gpsUTCOff: int64(taiOffset) - 19}
}

// Run connects to gpsd and scans the raw stream for TIM-TP frames until an
// error occurs. Call in a goroutine and retry on return, like GpsdSource.
func (q *QErrSource) Run() error {
	conn, err := net.DialTimeout("tcp", q.addr, 5*time.Second)
	if err != nil {
		return fmt.Errorf("gpsd(raw) connect %s: %w", q.addr, err)
	}
	defer conn.Close()

	if _, err := conn.Write([]byte(`?WATCH={"enable":true,"raw":2}` + "\n")); err != nil {
		return fmt.Errorf("gpsd(raw) watch: %w", err)
	}
	log.Printf("gpsd(raw): connected to %s for TIM-TP qErr", q.addr)

	// Byte-stream UBX scanner: accumulate, resync on 0xB5 0x62, ignore the
	// interleaved NMEA text.
	buf := make([]byte, 0, 4096)
	chunk := make([]byte, 2048)
	for {
		n, err := conn.Read(chunk)
		if err != nil {
			return fmt.Errorf("gpsd(raw) read: %w", err)
		}
		buf = append(buf, chunk[:n]...)
		for {
			// find sync
			i := 0
			for i+1 < len(buf) && !(buf[i] == ubx.SyncA && buf[i+1] == ubx.SyncB) {
				i++
			}
			buf = buf[i:]
			if len(buf) < ubx.HeaderLen {
				break
			}
			length := int(buf[4]) | int(buf[5])<<8
			total := ubx.HeaderLen + length + ubx.ChecksumLen
			if length > 1024 { // implausible: drop sync bytes and resync
				buf = buf[2:]
				continue
			}
			if len(buf) < total {
				break
			}
			frame, err := ubx.Decode(buf[:total])
			buf = buf[total:]
			if err != nil {
				continue
			}
			if frame.ClassID() == ubx.MsgTimTP {
				q.handle(frame.Payload)
			}
		}
		if len(buf) > 8192 { // runaway garbage: hard resync
			buf = buf[:0]
		}
	}
}

func (q *QErrSource) handle(payload []byte) {
	tp, err := ubx.ParseTimTP(payload)
	if err != nil || tp.QErrInvalid() {
		return
	}
	sec := gpsEpochUnix + int64(tp.Week)*604800 + int64(tp.TowMS)/1000
	if !tp.TimeBaseUTC() {
		sec -= q.gpsUTCOff
	}
	q.mu.Lock()
	q.ring[q.head].sec = sec
	q.ring[q.head].ns = float64(tp.QErr) / 1000.0
	q.head = (q.head + 1) % len(q.ring)
	q.mu.Unlock()
	if q.OnTimTP != nil {
		q.OnTimTP(tp)
	}
}

// Lookup returns the qErr (ns) for the pulse landing on the given UTC second.
func (q *QErrSource) Lookup(utcSec int64) (float64, bool) {
	q.mu.Lock()
	defer q.mu.Unlock()
	for i := range q.ring {
		if q.ring[i].sec == utcSec {
			return q.ring[i].ns, true
		}
	}
	return 0, false
}
