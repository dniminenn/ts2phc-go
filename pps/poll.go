package pps

import (
	"fmt"
	"time"

	"golang.org/x/sys/unix"
)

// PollSinks multiplexes read events from all configured PHC sinks. 
// It guarantees that all sinks have reported an event for the given cycle 
// before allowing the main loop to proceed and synchronize everything
// against the reference clock's source timestamp.
func PollSinks(sinks []*Sink, timeout time.Duration) (bool, error) {
	if len(sinks) == 0 {
		return false, nil
	}

	pollFds := make([]unix.PollFd, len(sinks))
	collectedEvents := make([]int, len(sinks))

	for i, s := range sinks {
		pollFds[i].Fd = int32(s.Device.Fd())
		pollFds[i].Events = unix.POLLIN | unix.POLLPRI
	}

	allSinksHaveEvents := false
	ignoreAny := false

	timeoutMs := int(timeout.Milliseconds())

	for !allSinksHaveEvents {
		n, err := unix.Poll(pollFds, timeoutMs)
		if err != nil {
			if err == unix.EINTR {
				return false, nil
			}
			return false, fmt.Errorf("unix.Poll failed: %w", err)
		}

		if n == 0 {
			// Timeout
			return false, nil
		}

		for i := range sinks {
			if pollFds[i].Revents&unix.POLLERR != 0 {
				return false, fmt.Errorf("poll error on sink %s", sinks[i].Name)
			}

			if pollFds[i].Revents&(unix.POLLIN|unix.POLLPRI) != 0 {
				s := sinks[i]
				
				// Read the raw event
				event, err := s.Device.ReadExttsEvent()
				if err != nil {
					return false, fmt.Errorf("read extts failed on sink %s: %w", s.Name, err)
				}
				
				// Apply dynamic edge filter
				// The boolean returned indicates if the event is a TRUE 1PPS edge and should be processed
				_, validEdge := s.ProcessEvent(event, false)

				if !validEdge {
					ignoreAny = true
				}

				// We collect the event anyway (even if ignored) because we do not want
				// edge events from different pulses to pile up and mix across sinks in the poll loop.
				collectedEvents[i]++
			}
		}

		allSinksHaveEvents = true
		for i := range sinks {
			if collectedEvents[i] == 0 {
				allSinksHaveEvents = false
				break
			}
		}
	}

	// If ANY of the sinks reported an ignored edge (e.g., the falling edge of the pulse width),
	// the entire synchronization cycle should be skipped for this second to maintain sync accuracy.
	if ignoreAny {
		return false, nil // "true" edge not found, skip sync
	}

	return true, nil // Ready to synchronize
}
