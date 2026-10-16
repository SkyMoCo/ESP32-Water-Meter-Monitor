# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

PlatformIO/Arduino firmware for an **ESP32-C3 DevKitM-1** in the well pumphouse at the ranch. It counts reed-switch pulses from an EKM 3/4" water meter, reads a 0-100 PSI discharge pressure sensor, and publishes to Mosquitto on `ranchpi4` (192.168.0.10:1883). Single source file: [src/main.cpp](src/main.cpp). Hardware, wiring and the reasoning behind the flow front end are in [ReadMe.md](ReadMe.md) — keep the two in sync when pins or circuits change.

The board is a **C3**, not a classic ESP32. A previous pass ported the code to a DOIT DevKit V1 by mistake (upload failed with "This chip is ESP32-C3, not ESP32"). Pin choices are C3-specific: flow GPIO 4, pressure GPIO 3 (must be ADC1 = GPIO 0-4), external status LED GPIO 10. GPIO 2/8/9 are strapping pins and 18/19 are USB.

## Build / upload / check

`pio` is not on PATH in the Bash tool; use the PlatformIO venv directly:

```bash
~/.platformio/penv/Scripts/pio run              # build
~/.platformio/penv/Scripts/pio run -t upload    # build + flash (board on COM8)
```

`pio device monitor` is interactive; to grab boot output non-interactively, reset via RTS and read with pyserial:

```bash
~/.platformio/penv/Scripts/python -c "
import serial, time
s=serial.Serial('COM8',115200,timeout=1)
s.dtr=False; s.rts=True; time.sleep(0.1); s.rts=False
end=time.time()+30
while time.time()<end:
    l=s.readline()
    if l: print(l.decode(errors='replace').rstrip())
"
```

`include/secrets.h` (gitignored) holds WiFi SSID/password and `MQTT_BROKER`; copy from `secrets.h.template`. Never commit it, and don't echo its contents.

## MQTT topics (every 5 s unless noted)

- `ranch/water/flow` — `gpm`, `gallons_total`, `cf_total`, `pulse_count`, `suspect`
- `ranch/water/diag` — `edges`, `accepted`, `short_lows`, `too_fast`, `pin`, `rssi`. First place to look for noise: clean front end means `edges ≈ 2 × accepted`, and zero edges when idle.
- `ranch/water/pressure` — `psi`, `volts` (sensor-side volts, divider undone)
- `ranch/water/status` — one boot message

## Working with the ranch

The hardware and broker are at a remote ranch; the user is often at home and can't touch either. Don't SSH to `ranchpi4` — hand over copy-pasteable commands and wait for output. Collect anything needing physical access into a "next time at the ranch" list rather than blocking on it.

## Current status (2026-10-16)

Latest firmware (commit `6bcaa31`) is flashed and verified over serial: joins WiFi, connects to MQTT, publishes on all topics.

**Fixed:**
- Phantom flow ("maxed out" with nothing on pin 4). Cause was a floating input — the internal pull-up was disabled on the assumption the external 1k R1 was always fitted. The internal pull-up is now always on (harmless in parallel with 1k). Verified: with pin 4 open, `edges: 0`, `pin: 1`, `gpm: 0.00`.
- Pulses are qualified in the ISR by width (`MIN_LOW_MS` 5 ms) and spacing (`MIN_PERIOD_MS` 20 ms), counted on release.
- Pressure divider ratio set to the fitted **4.7k (sensor→pin) / 10k (pin→GND)**.

**Open items:**
- **Junk NVS total.** The board restores ~120,067,973 pulses (≈ 8.98 M gal) — accumulated noise from the floating-pin era, not water. It persists across reboots and is published in `gallons_total`. Needs a one-time reset (to 0 or to the meter's register reading); user hasn't decided yet.
- **Pressure reads −12.5 PSI at 0.000 V** — nothing driving GPIO 3. Expected if the sensor isn't connected; if it is, check its 5 V supply and the divider. A live sensor should give ~0.34 V at the pin at 0 PSI.
- **Pressure ceiling ~79 PSI.** With 4.7k/10k the pin can reach 3.06 V but the C3 ADC is only calibrated to ~2.5 V. Fine for a typical well; swap the 4.7k for 10k (and update `PRESSURE_DIVIDER_RATIO`) for full range.
- **`PULSES_PER_CF = 100` is unverified** — inferred from MQTT history. Calibrate against a measured volume / the register face.
- **Front end not re-tested with the meter connected** since this firmware. Watch `ranch/water/diag` while the pump cycles: edges climbing with the pump but flat when off points at contactor noise (see ReadMe "If this is not enough").
