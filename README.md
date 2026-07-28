# ts2phc-go

**Writeup:** [Detecting a frozen PTP and GPS clock with ts2phc in Go](https://dnim.dev/blog/ts2phc-go-frozen-clock)

A single-daemon GPS-disciplined PTP clock synchronizer for Linux. Reads GPS time from gpsd, disciplines a PTP Hardware Clock (PHC) via 1PPS/EXTTS, and exports Prometheus metrics for both GPS state and clock sync quality.

It replaces linuxptp's `ts2phc` daemon (it still relies on `gpsd` for the GPS receiver; see [Architecture](#architecture)), and was written specifically to handle the **Intel i210**, whose EXTTS quirks vanilla `ts2phc` does not cope with. See [The i210 (and why this exists)](#the-i210-and-why-this-exists).

## The i210 (and why this exists)

This daemon exists because the Intel **i210** (igb driver), a common cheap NIC for GPS-PPS-on-SDP grandmasters, does not play nicely with stock `ts2phc` when the GPS 1PPS is wired to a Software-Definable Pin. The non-obvious behaviors it works around:

- **GPS PPS on SDP0.** The PPS is fed to pin `SDP0`, which must be programmed as an EXTTS input (`PTP_PF_EXTTS`) before events arrive. The daemon does this via `PTP_PIN_SETFUNC` at startup.
- **The igb driver rejects strict-flag EXTTS.** The modern `PTP_EXTTS_REQUEST2` ioctl carries `PTP_STRICT_FLAGS`, and igb returns `EOPNOTSUPP` for a both-edge request under strict flags. So the daemon **deliberately uses the legacy `PTP_EXTTS_REQUEST` (v1) path**, which the driver accepts. The `PTP_EXTTS_REQUEST2` constant in `phc/phc.go` is intentionally set to a *non-matching* ioctl number so the v2 attempt fails fast and falls back to v1. There is a NOTE comment at the constant; do not "fix" it to the canonical value without re-validating capture on the i210.
- **The i210 timestamps *both* edges** of the PPS pulse regardless of the polarity requested. The daemon's **dynamic edge filter** (`pps/sink.go`) locks onto the pulse pattern (a clean lock logs `edge filter locked, pulse width ~10ms`) and discards the trailing edge, keeping only the true start-of-second. A NIC that honors single-edge requests is also handled, via a separate period-check path.

If you are running on hardware that honors EXTTS edge selection and strict flags, none of the above hurts; the v1 path and the edge filter still produce correct locks.

## Architecture

```
gpsd (/dev/ttyACM0)          PHC (/dev/ptp0)
        │                            │
        ▼                            │
   ┌──────────┐                      │
   │ GpsdSrc  │  JSON :2947          │
   │ TPV/SKY  │                      │
   └────┬─────┘                      │
        │                            ▼
        ▼                    ┌──────────────┐
   ┌─────────┐               │  PPS Sink    │
   │  main   │──TAI time───▶ │  EXTTS poll  │
   │  loop   │               │  PI servo    │
   │         │               │  AdjFreq/    │
   │         │               │  StepTime    │
   └──┬──────┘               └──────────────┘
      │
      └─────▶ Prometheus :9100/metrics
```

- **GpsdSource** connects to gpsd's JSON stream, parses TPV messages for UTC time, converts to TAI.
- **PPS discipline loop** waits for EXTTS events on the PHC, computes offset against the GPS-derived reference time, and steers the hardware clock via a PI servo.
- **Prometheus metrics** cover GPS fix quality, satellite tracking, and ts2phc offset/frequency.
- gpsd owns the serial port and handles SHM for chrony/ntpd.

## Requirements

- Linux with a PTP-capable NIC (EXTTS support)
- gpsd running and connected to a u-blox GNSS receiver (`ubxcfg` supports M8N and M9N; tested with NEO-M9N)
- Go 1.25+
- Root or `CAP_SYS_TIME` for PHC ioctls

## Build

```bash
go build -o ts2phc-go .
go build -o ubxcfg ./cmd/ubxcfg
```

## Usage

```bash
# 1. Configure the receiver (once, or via udev rule)
sudo ./ubxcfg --dev /dev/ttyACM0

# 2. Start gpsd on the serial port
gpsd /dev/ttyACM0

# 3. Start ts2phc-go
sudo ./ts2phc-go --sink /dev/ptp0
```

### CLI Flags

| Flag | Short | Default | Description |
|------|-------|---------|-------------|
| `--gpsd-addr` | | `localhost:2947` | gpsd JSON stream address |
| `--sink` | `-c` | `/dev/ptp0` | PHC device to discipline |
| `--autocfg` | `-a` | `false` | Enable ptp4l PMC autoconfiguration |
| `--tai-offset` | | `37` | TAI-UTC offset in seconds (overridden by `--leapfile` if not explicitly set) |
| `--leapfile` | | `/usr/share/zoneinfo/leap-seconds.list` | Leap seconds file to derive TAI-UTC offset, if present |
| `--step-threshold` | | `0.0` | Step the clock when offset exceeds this many seconds (`0.0` disables ongoing steps; the servo slews instead) |
| `--first-step-threshold` | | `0.00002` | On the first update after start or a servo reset, step instead of slew if offset exceeds this many seconds (default 20 us) |
| `--pin-index` | | `0` | SDP pin index carrying the GPS PPS (0 for i210 SDP0, 2 for TimeHAT i226 SDP2) |
| `--gm-mgmt` | | `false` | Manage ptp4l's announced clockClass over its management socket: 6 locked, 7 holdover, 248 free-run |
| `--gm-holdover-sec` | | `3600` | Seconds of GPS loss tolerated in holdover (clockClass 7) before demoting to free-run (248) |
| `--metrics-addr` | | `:9100` | Prometheus metrics listen address |
| `--metrics` | | `true` | Enable Prometheus metrics server |
| `--config` | | `~/.ts2phc-go.yaml` | Config file path |

### Config File

All flags can be set in a YAML config file (`~/.ts2phc-go.yaml` by default, or `--config /path/to/file.yaml`):

```yaml
gpsd_addr: "localhost:2947"
sink: /dev/ptp0
tai_offset: 37
leapfile: /usr/share/zoneinfo/leap-seconds.list
metrics_addr: ":9100"
```

Environment variables with the `TS2PHC_` prefix also work (e.g. `TS2PHC_SINK=/dev/ptp1`).

### ubxcfg: Receiver Configuration Tool

A standalone tool for configuring u-blox receivers. It supports both M8N and M9N receivers with automatic detection (`MON-VER`) and optional manual mode override.

```bash
sudo ./ubxcfg --dev /dev/ttyACM0 --mode auto --dynmodel 2 --ant-cable-delay-ns 38
```

| Flag | Default | Description |
|------|---------|-------------|
| `--dev` | `/dev/ttyACM0` | Serial device |
| `--baud` | `115200` | Baud rate (`m8n`: tried first during baud probe) |
| `--mode` | `auto` | Receiver mode (`auto`, `m8n`, `m9n`) |
| `--dynmodel` | `2` | Dynamic model (0=portable, 2=stationary) |
| `--ant-cable-delay-ns` | `38` | Antenna cable delay in ns (M8N + M9N) |

Behavior by mode:
- `m9n`: uses `UBX-CFG-VALSET` and saves to `RAM+BBR+FLASH`, configures dynamic model, antenna cable delay, UBX message output (NAV-PVT, NAV-DOP, NAV-TIMEUTC, NAV-CLOCK, NAV-SAT, TIM-TP, NAV-SIG), and USB UBX/NMEA protocols.
- `m8n`: probes common baud rates, upgrades receiver baud to `115200` if needed, applies `UBX-CFG-GNSS` (Galileo on, QZSS off), applies `UBX-CFG-MSG` NMEA profile (GGA/RMC/ZDA on; GLL/GSA/GSV/VTG off), applies antenna cable delay through `UBX-CFG-TP5`, applies `UBX-CFG-NAV5` dynamic model, then persists with `UBX-CFG-CFG/save` to `BBR+FLASH`.

Examples:

```bash
# Auto-detect receiver type (recommended)
sudo ./ubxcfg --dev /dev/ttyACM0 --mode auto --dynmodel 2

# Force M8N path
sudo ./ubxcfg --dev /dev/ttyACM0 --mode m8n --dynmodel 2

# Force M9N path
sudo ./ubxcfg --dev /dev/ttyACM0 --mode m9n --dynmodel 2 --ant-cable-delay-ns 38
```

### systemd

```ini
[Unit]
Description=GPS-disciplined PTP clock daemon
After=gpsd.service
Wants=gpsd.service

[Service]
ExecStart=/usr/local/bin/ts2phc-go --sink /dev/ptp0
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

## PPS Discipline

The PPS discipline loop follows the same algorithm as linuxptp's `ts2phc`:

1. **EXTTS polling**: `poll()` on the PHC file descriptor, waiting for hardware-timestamped PPS edges.
2. **Dynamic edge filter**: Handles NICs that deliver both rising and falling edges regardless of the polarity requested. Locks onto the pulse pattern and ignores trailing edges.
3. **Offset calculation**: `offset = PHC_timestamp - GPS_UTC_timestamp` (with TAI correction).
4. **PI servo**: Proportional-integral controller computes frequency adjustment. States: `Unlocked` → `Jump` (step + reset) → `Locked` → `LockedStable`.
5. **Outlier rejection**: While locked, samples with `|offset| > 50µs` are rejected. 5 consecutive outliers trigger a servo reset.
6. **Clock steering**: `AdjFreq()` for frequency, `StepTime()` for phase jumps. Stepping is threshold-gated (see `--step-threshold` / `--first-step-threshold`): by default the servo only steps on the first update after start or a reset if the offset exceeds 20 us, and slews continuously thereafter (ongoing stepping is disabled). On a warm restart the offset is usually sub-microsecond, so it slews rather than steps.

## Prometheus Metrics

All metrics are served on `--metrics-addr` (default `:9100`) at `/metrics`.

### GPS Metrics (namespace: `gps_`)

| Metric | Type | Description |
|--------|------|-------------|
| `gps_fix_type` | gauge | GNSS fix type (0=none, 2=2D, 3=3D) |
| `gps_satellites_used` | gauge | Number of SVs used in navigation solution |
| `gps_latitude_degrees` | gauge | Latitude in degrees |
| `gps_longitude_degrees` | gauge | Longitude in degrees |
| `gps_altitude_msl_meters` | gauge | Altitude above mean sea level |
| `gps_horizontal_accuracy_meters` | gauge | Horizontal accuracy estimate |
| `gps_vertical_accuracy_meters` | gauge | Vertical accuracy estimate |
| `gps_speed_mps` | gauge | Ground speed in m/s |
| `gps_heading_degrees` | gauge | Vehicle heading |
| `gps_heading_accuracy_degrees` | gauge | Heading accuracy |
| `gps_velocity_north_mps` | gauge | North velocity component |
| `gps_velocity_east_mps` | gauge | East velocity component |
| `gps_velocity_down_mps` | gauge | Down velocity component |
| `gps_pdop` | gauge | Position dilution of precision |
| `gps_hdop` | gauge | Horizontal dilution of precision |
| `gps_vdop` | gauge | Vertical dilution of precision |
| `gps_tdop` | gauge | Time dilution of precision |
| `gps_gdop` | gauge | Geometric dilution of precision |
| `gps_time_accuracy_ns` | gauge | UTC time accuracy in nanoseconds |
| `gps_time_valid` | gauge | 1 if UTC time is valid |
| `gps_clock_bias_ns` | gauge | Receiver clock bias in ns |
| `gps_clock_drift_nps` | gauge | Receiver clock drift in ns/s |
| `gps_clock_accuracy_ns` | gauge | Clock accuracy in ns |
| `gps_freq_accuracy_pps` | gauge | Frequency accuracy in ps/s |
| `gps_timepulse_quantization_error_ps` | gauge | Timepulse quantization error in ps |
| `gps_satellite_cno_dbhz` | gauge | C/N0 per satellite (labels: `gnss`, `svid`) |
| `gps_satellite_elevation_degrees` | gauge | Elevation per satellite |
| `gps_satellite_azimuth_degrees` | gauge | Azimuth per satellite |
| `gps_satellite_used` | gauge | 1 if satellite used in solution |

### Clock Discipline Metrics (namespace: `ts2phc_`)

| Metric | Labels | Description |
|--------|--------|-------------|
| `ts2phc_offset_ns` | `clock` | PHC clock offset from GPS in nanoseconds |
| `ts2phc_freq_ppb` | `clock` | Frequency adjustment applied to PHC in ppb |

Note: GPS metrics that require UBX-specific data not available via gpsd JSON (clock bias/drift, time accuracy, timepulse quantization error) will remain at zero.

## Package Layout

```
.
├── main.go           Entry point, cobra CLI, gpsd source, PPS discipline loop
├── cmd/ubxcfg/       Standalone u-blox receiver configuration tool
├── ubx/              UBX protocol: frame encode/decode, message parsers, CFG-VALSET
├── metrics/          Prometheus gauges for GPS + ts2phc
├── pps/              PPS source/sink, EXTTS polling, edge filtering, gpsd client
├── phc/              PHC device access (ioctls, EXTTS, clock adjustment)
├── servo/            PI servo for clock discipline
└── pmc/              PTP Management Client (optional ptp4l integration)
```

## License

This project is a Go implementation of the `ts2phc` utility from the Linux PTP
project (`linuxptp`), and is therefore a derivative work.

It is distributed under the terms of the GNU General Public License, version 2
or (at your option) any later version (GPL-2.0-or-later), the same license as
linuxptp.

For the full license text, see `https://www.gnu.org/licenses/old-licenses/gpl-2.0.txt`.
