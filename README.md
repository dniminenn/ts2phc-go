# ts2phc-go

A single-daemon GPS-disciplined PTP clock synchronizer for Linux. Reads UBX from a u-blox GNSS receiver (e.g. NEO-M9N), disciplines a PTP Hardware Clock (PHC) via 1PPS/EXTTS, serves NMEA over TCP for gpsd/cgps, and exports Prometheus metrics for both GPS state and clock sync quality.

Replaces the need for separate `ts2phc` + GPS daemon services.

## Architecture

```
Serial (/dev/ttyACM0)          PHC (/dev/ptp0)
        │                            │
        ▼                            │
   ┌─────────┐                       │
   │  demux  │  UBX/NMEA stream      │
   └────┬────┘                       │
        │                            │
        ▼                            ▼
   ┌─────────┐               ┌──────────────┐
   │ handler │──NavPVT UTC──▶│  PPS Sink    │
   │         │               │  EXTTS poll  │
   │         │               │  PI servo    │
   │         │               │  AdjFreq/    │
   │         │               │  StepTime    │
   └──┬──┬───┘               └──────────────┘
      │  │
      │  └──▶ TCP :2948 ──▶ gpsd / cgps
      │
      └─────▶ Prometheus :9100/metrics
```

- **Demux goroutine** reads the mixed UBX/NMEA serial stream, dispatches frames to the handler.
- **Handler** extracts UTC time from NAV-PVT and feeds it to the UBX source, updates GPS metrics, and broadcasts generated NMEA sentences to TCP clients.
- **PPS discipline loop** waits for EXTTS events on the PHC, computes offset against the GPS-derived reference time, and steers the hardware clock via a PI servo.
- **TCP export** generates NMEA (GGA, RMC, GSA, GSV, ZDA, VTG) from UBX and serves it on `:2948` for gpsd.
- **Prometheus metrics** cover GPS fix quality, satellite tracking, clock bias/drift, and ts2phc offset/frequency.

## Requirements

- Linux with a PTP-capable NIC (EXTTS support)
- u-blox GNSS receiver with UBX protocol (tested with NEO-M9N)
- Go 1.25+
- Root or `CAP_SYS_TIME` for PHC ioctls

## Build

```bash
go build -o ts2phc-go .
```

## Usage

```bash
sudo ./ts2phc-go --dev /dev/ttyACM0 --sink /dev/ptp0
```

### CLI Flags

| Flag | Short | Default | Description |
|------|-------|---------|-------------|
| `--dev` | | `/dev/ttyACM0` | GPS serial device path |
| `--baud` | | `115200` | GPS serial baud rate |
| `--ant-cable-delay-ns` | | `38` | Antenna cable delay in nanoseconds |
| `--sink` | `-c` | `/dev/ptp0` | PHC device to discipline |
| `--autocfg` | `-a` | `false` | Enable ptp4l PMC autoconfiguration |
| `--tai-offset` | | `37` | TAI-UTC offset in seconds (overridden by `--leapfile` if not explicitly set) |
| `--leapfile` | | `/usr/share/zoneinfo/leap-seconds.list` | Leap seconds file to derive TAI-UTC offset, if present |
| `--tcp-addr` | | `:2948` | TCP NMEA export listen address |
| `--tcp` | | `true` | Enable TCP NMEA export for gpsd |
| `--metrics-addr` | | `:9100` | Prometheus metrics listen address |
| `--metrics` | | `true` | Enable Prometheus metrics server |
| `--config` | | `~/.ts2phc-go.yaml` | Config file path |

### Config File

All flags can be set in a YAML config file (`~/.ts2phc-go.yaml` by default, or `--config /path/to/file.yaml`):

```yaml
dev: /dev/ttyACM0
baud: 115200
sink: /dev/ptp0
tai_offset: 37
ant_cable_delay_ns: 38
leapfile: /usr/share/zoneinfo/leap-seconds.list
tcp_addr: ":2948"
metrics_addr: ":9100"
```

Environment variables with the `TS2PHC_` prefix also work (e.g. `TS2PHC_DEV=/dev/ttyACM1`).

### gpsd Integration

Point gpsd at the TCP NMEA stream:

```bash
gpsd tcp://localhost:2948
```

Then use `cgps`, `gpsmon`, or any gpsd client as usual.

### systemd

```ini
[Unit]
Description=GPS-disciplined PTP clock daemon
After=network.target

[Service]
ExecStart=/usr/local/bin/ts2phc-go --dev /dev/ttyACM0 --sink /dev/ptp0
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

## GPS Module Configuration

On startup, the daemon sends a UBX-CFG-VALSET to the receiver (RAM layer) to configure:

- **Dynamic model**: Stationary (2)
- **Antenna cable delay**: from `--ant-cable-delay-ns`
- **UBX messages enabled on USB**: NAV-PVT, NAV-DOP, NAV-TIMEUTC, NAV-CLOCK, NAV-SAT (every 5th epoch), TIM-TP
- **NMEA output disabled on USB**: all native NMEA sentences suppressed (NMEA is generated from UBX internally)
- **Protocol**: UBX-only output, UBX input

No flash writes are made — the configuration is RAM-only and resets on receiver power cycle.

## PPS Discipline

The PPS discipline loop follows the same algorithm as linuxptp's `ts2phc`:

1. **EXTTS polling**: `poll()` on the PHC file descriptor, waiting for hardware-timestamped PPS edges.
2. **Dynamic edge filter**: Handles NICs that deliver both rising and falling edges regardless of the polarity requested. Locks onto the pulse pattern and ignores trailing edges.
3. **Offset calculation**: `offset = PHC_timestamp - GPS_UTC_timestamp` (with TAI correction).
4. **PI servo**: Proportional-integral controller computes frequency adjustment. States: `Unlocked` → `Jump` (step + reset) → `Locked` → `LockedStable`.
5. **Outlier rejection**: While locked, samples with `|offset| > 50µs` are rejected. 5 consecutive outliers trigger a servo reset.
6. **Clock steering**: `AdjFreq()` for frequency, `StepTime()` for phase jumps (only in `Jump` state).

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

## Package Layout

```
.
├── main.go           Entry point, cobra CLI, GPS init, PPS discipline loop
├── ubx/              UBX protocol: frame encode/decode, message parsers, CFG-VALSET
├── demux/            Serial stream demuxer (UBX/NMEA dispatch)
├── gpsnmea/          NMEA sentence generation from UBX (GGA, RMC, GSA, GSV, ZDA, VTG)
├── export/           TCP fan-out server for gpsd
├── metrics/          Prometheus gauges for GPS + ts2phc
├── pps/              PPS source/sink abstractions, EXTTS polling, edge filtering
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
