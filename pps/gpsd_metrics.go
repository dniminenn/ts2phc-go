package pps

import (
	"log"
	"sync"
	"ts2phc-go/metrics"
	"ts2phc-go/ubx"
)

// GpsdMetricsAdapter implements GpsdHandler and feeds gpsd JSON data
// into the existing metrics package via its UBX-typed Update methods.
type GpsdMetricsAdapter struct {
	Metrics *metrics.Metrics
	// TAIOffset is the TAI-UTC the daemon is disciplining with. Each TPV
	// carrying the receiver's broadcast leapseconds cross-checks it:
	// TAI-UTC must equal leapseconds+19. A mismatch means a stale/wrong
	// leapfile or a receiver still on its firmware-default leap count --
	// either way the timescale is wrong by whole seconds and the guard,
	// sharing the same offset, cannot see it. This metric is the alarm.
	TAIOffset int

	mu          sync.Mutex
	pdop100     uint16
	leapWarned  bool
}

func (a *GpsdMetricsAdapter) OnTPV(tpv *GpsdTPV) {
	if a.Metrics == nil {
		return
	}

	if tpv.Leapseconds != 0 && a.TAIOffset != 0 {
		ok := a.TAIOffset == tpv.Leapseconds+19
		a.Metrics.UpdateLeapConsistency(ok)
		a.mu.Lock()
		warn := !ok && !a.leapWarned
		a.leapWarned = !ok
		a.mu.Unlock()
		if warn {
			log.Printf("LEAP MISMATCH: configured TAI-UTC %d but receiver broadcasts leapseconds %d (implies %d); the timescale is wrong by whole seconds and the guard cannot see it",
				a.TAIOffset, tpv.Leapseconds, tpv.Leapseconds+19)
		}
	}

	a.mu.Lock()
	pdop100 := a.pdop100
	a.mu.Unlock()

	pvt := &ubx.NavPVT{
		FixType: uint8(tpv.Mode),
		Lat:     int32(tpv.Lat * 1e7),
		Lon:     int32(tpv.Lon * 1e7),
		HMSL:    int32(tpv.AltMSL * 1000),
		HAcc:    uint32(tpv.Eph * 1000),
		VAcc:    uint32(tpv.Epv * 1000),
		GSpeed:  int32(tpv.Speed * 1000),
		HeadMot: int32(tpv.Track * 1e5),
		PDOP:    pdop100,
	}
	a.Metrics.UpdateNavPVT(pvt)

	tAcc := uint32(0)
	if tpv.Ept > 0 {
		tAcc = uint32(tpv.Ept * 1e9)
	}
	valid := uint8(0)
	if tpv.Mode >= 2 && tpv.Time != "" {
		valid = 0x04 // UTC valid
	}
	a.Metrics.UpdateNavTimeUTC(&ubx.NavTimeUTC{
		TAcc:  tAcc,
		Valid: valid,
	})
}

func (a *GpsdMetricsAdapter) OnSKY(sky *GpsdSKY) {
	if a.Metrics == nil {
		return
	}

	a.mu.Lock()
	a.pdop100 = uint16(sky.PDOP * 100)
	a.mu.Unlock()

	dop := &ubx.NavDOP{
		GDOP: uint16(sky.GDOP * 100),
		PDOP: uint16(sky.PDOP * 100),
		TDOP: uint16(sky.TDOP * 100),
		VDOP: uint16(sky.VDOP * 100),
		HDOP: uint16(sky.HDOP * 100),
	}
	a.Metrics.UpdateNavDOP(dop)

	// gpsd interleaves DOP-only SKY reports that carry no satellites array (and no
	// uSat); those are a normal part of its report cadence, NOT signal loss. Only
	// update satellite series from SKY reports that actually carry satellites, so
	// the routine empty reports don't wipe the per-satellite series or zero the
	// used count. A genuine loss still shows: a populated SKY with everything
	// used=false drives usedCount (and gps_satellites_used) to 0.
	if len(sky.Satellites) > 0 {
		svs := make([]ubx.SatInfo, len(sky.Satellites))
		for i, sv := range sky.Satellites {
			var flags uint32
			if sv.Used {
				flags |= 0x08
			}
			svs[i] = ubx.SatInfo{
				GnssID: uint8(sv.GnssID),
				SvID:   uint8(sv.SvID),
				Cno:    uint8(sv.Ss),
				Elev:   int8(sv.El),
				Azim:   int16(sv.Az),
				Flags:  flags,
			}
		}
		sat := &ubx.NavSAT{
			NumSvs: uint8(len(svs)),
			Svs:    svs,
		}
		a.Metrics.UpdateNavSAT(sat)
	}
}
