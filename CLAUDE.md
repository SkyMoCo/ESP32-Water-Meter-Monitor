# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

PlatformIO/Arduino firmware for an **ESP32-C3 DevKitM-1** in the well pumphouse at the ranch. It counts reed-switch pulses from an EKM 3/4" water meter, reads a 0-100 PSI discharge pressure sensor, and publishes to Mosquitto on `ranchpi4` (192.168.0.10:1883). Single source file: [src/main.cpp](src/main.cpp). Hardware, wiring and the reasoning behind the flow front end are in [ReadMe.md](ReadMe.md) — keep the two in sync when pins or circuits change.

The board is a **C3**, not a classic ESP32. A previous pass ported the code to a DOIT DevKit V1 by mistake (upload failed with "This chip is ESP32-C3, not ESP32"). Pin choices are C3-specific: flow GPIO 4, pressure GPIO 3 (must be ADC1 = GPIO 0-4), external status LED GPIO 10. GPIO 2/8/9 are strapping pins and 18/19 are USB.

## Build / upload / check

`pio` is not on PATH in the Bash tool; use the PlatformIO venv directly:

```bash
~/.platformio/penv/Scripts/pio run              # build
~/.platformio/penv/Scripts/pio run -t upload --upload-port COM3   # build + flash
```

The board was on COM3 (CP210x) on 2026-10-17; it has also shown up as COM8 — check `pio device list`. `python` is not on PATH either; use `~/.platformio/penv/Scripts/python`.

`pio device monitor` is interactive; to grab boot output non-interactively, reset via RTS and read with pyserial:

```bash
~/.platformio/penv/Scripts/python -c "
import serial, time
s=serial.Serial('COM3',115200,timeout=1)
s.dtr=False; s.rts=True; time.sleep(0.1); s.rts=False
end=time.time()+30
while time.time()<end:
    l=s.readline()
    if l: print(l.decode(errors='replace').rstrip())
"
```

`include/secrets.h` (gitignored) holds WiFi SSID/password and `MQTT_BROKER`; copy from `secrets.h.template`. Never commit it, and don't echo its contents. Only `include/secrets.h` is ignored — a copy anywhere else (e.g. `src/`) would be committed.

The pulse total is the only thing kept in NVS, so `pio run -t erase -t upload` is a safe way to zero it.

## MQTT topics

- `ranch/water/flow` (every 15 s, `PUBLISH_MS`) — `gpm`, `gallons_total`, `cf_total`, `pulse_count`, `suspect`. `gpm` is timed between pulses, decays while a pulse is overdue, and reads 0 after `FLOW_STOP_MS` (2 min).
- `ranch/water/pressure` (every 15 s) — `psi`, `volts` (sensor-side volts, divider undone)
- `ranch/water/diag` (every 60 s, `DIAG_MS`, counters cover the minute) — `edges`, `accepted`, `glitches`, `too_fast`, `glitch_max_ms`, `accept_min_ms`, `pin`, `rssi`. First place to look for noise: clean front end means `edges ≈ 2 × accepted`; `glitch_max_ms` approaching `SETTLE_MS` (20) means noise is close to faking a pulse.
- `ranch/water/status` — one boot message

## Working with the ranch

The hardware and broker are at a remote ranch; the user is often at home and can't touch either. Don't SSH to `ranchpi4` — hand over copy-pasteable commands and wait for output. Collect anything needing physical access into a "next time at the ranch" list rather than blocking on it.

## Current status (2026-10-19)

Latest firmware (commit `c76dc3c`) is flashed on the board and running with the meter connected.

**Done:**
- **Calibration:** the meter's reed output is 0.748 gal (0.1 cf) per pulse, so `PULSES_PER_CF = 10`. The earlier 100 read every flow 10× low.
- **Mains hum:** at `MIN_LOW_MS` 5 ms, 60 Hz hum passed as closures and read ~135 GPM. Settle time is now 20 ms; real closures last 2–3 s.
- **Counting:** the pin is sampled every 1 ms by an `esp_timer` and only changes state after holding a level for `SETTLE_MS` (20 ms); a pulse counts on settled closed → open. This replaced the edge ISR, which dropped a pulse when it misread the level at a real release (seen as a gap of exactly two periods).
- **NVS total reset to 0** on 2026-10-17 (chip erase); the old ~121 M pulse total was noise. `gallons_total` now counts from that date, not the meter register.
- **Pressure sensor is live:** reads ~28–40 PSI as the pump cycles.
- Verified with a sprinkler zone running (before the sampler change): one pulse per 4–7 s, ~6.5–11 GPM depending on pressure, all spikes rejected.

**Open items:**
- **Sampler build (`c76dc3c`) not yet checked under flow.** Expect steady pulse gaps (no double-length gaps), `edges ≈ 2 × accepted` per minute, `glitch_max_ms` well under 20.
- **Pressure ceiling ~79 PSI.** With 4.7k/10k the pin can reach 3.06 V but the C3 ADC is only calibrated to ~2.5 V. Fine for a typical well; swap the 4.7k for 10k (and update `PRESSURE_DIVIDER_RATIO`) for full range.
- **Total vs register:** optionally offset `gallons_total` to the meter's register reading; there is no set-total command yet.
