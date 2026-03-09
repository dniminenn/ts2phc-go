package nmea

import (
	"fmt"
	"strconv"
	"strings"
	"time"
)

type RMC struct {
	Time     time.Time
	FixValid bool
}

func checksumValid(s string) bool {
	star := strings.LastIndexByte(s, '*')
	if star < 1 || star+3 > len(s) {
		return false
	}
	var csum byte
	for i := 1; i < star; i++ {
		csum ^= s[i]
	}
	var want byte
	if _, err := fmt.Sscanf(s[star+1:star+3], "%02x", &want); err != nil {
		return false
	}
	return csum == want
}

// ParseRMC parses a $GxRMC NMEA sentence (any talker ID: GP, GN, GL, …).
// Returns the UTC time-of-day, fix validity, and whether the parse succeeded.
func ParseRMC(line string) (RMC, bool) {
	line = strings.TrimRight(line, "\r\n")
	if len(line) < 6 || line[0] != '$' {
		return RMC{}, false
	}
	if !checksumValid(line) {
		return RMC{}, false
	}
	star := strings.LastIndexByte(line, '*')
	body := line[1:star]

	if len(body) < 5 || body[0] != 'G' || body[2:5] != "RMC" {
		return RMC{}, false
	}

	fields := strings.Split(body, ",")
	if len(fields) < 10 {
		return RMC{}, false
	}

	timeStr := fields[1] // hhmmss[.sss]
	status := fields[2]  // A=valid, V=void
	dateStr := fields[9] // ddmmyy

	if len(timeStr) < 6 || len(dateStr) != 6 {
		return RMC{}, false
	}

	hour, err := strconv.Atoi(timeStr[0:2])
	if err != nil {
		return RMC{}, false
	}
	min, err := strconv.Atoi(timeStr[2:4])
	if err != nil {
		return RMC{}, false
	}
	sec, err := strconv.Atoi(timeStr[4:6])
	if err != nil {
		return RMC{}, false
	}

	var nsec int
	if dot := strings.IndexByte(timeStr, '.'); dot >= 0 {
		frac := timeStr[dot+1:]
		for len(frac) < 9 {
			frac += "0"
		}
		nsec, _ = strconv.Atoi(frac[:9])
	}

	// Leap second 60 → ambiguous 59 (matches linuxptp behaviour)
	if sec == 60 {
		sec = 59
	}

	day, _ := strconv.Atoi(dateStr[0:2])
	mon, _ := strconv.Atoi(dateStr[2:4])
	year, _ := strconv.Atoi(dateStr[4:6])

	t := time.Date(2000+year, time.Month(mon), day, hour, min, sec, nsec, time.UTC)
	return RMC{Time: t, FixValid: status == "A"}, true
}
