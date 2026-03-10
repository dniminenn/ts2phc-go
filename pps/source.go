package pps

import (
	"time"

	"ts2phc-go/phc"
)

type Source interface {
	GetPPSTime() (time.Time, error)
	GetClock() *phc.Device
	Destroy()
}
