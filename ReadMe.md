# ESP32 Water Meter Monitor

Firmware for an ESP32 that counts pulses from a flow meter on the well pump
and publishes flow rate / cumulative usage to MQTT.

## Hardware

- ESP32-C3 DevKitM-1
- EKM 3/4" pulse water meter — a dry-contact (normally open) reed switch, no
  polarity — wired to GPIO 4 through its factory 24" armored lead. The ESP32
  mounts at the end of that lead, so the entire run is about two feet.
- Status LED: external LED on GPIO 10, flashed briefly on each MQTT publish
  (GPIO 10 -> 220-330Ω resistor -> anode, cathode -> GND).
- 0-100 PSI pressure sensor (0.5-4.5V linear output, 5V supply) on the well
  pump discharge line, powered from the DevKit's `5V` pin (USB 5 V — only
  live because this board runs off a USB wall adapter; would need a separate
  5V supply if it were ever run from a raw 3.3V source instead). Signal wired
  to **GPIO 3** through a 4.7kΩ (sensor -> pin) / 10kΩ (pin -> GND) divider,
  mapping 0.5-4.5V to ~0.34-3.06V. Sensor GND shares the ESP32's GND.

### Flow input circuit

Wiring the reed to a GPIO on the *internal* pull-up does not work here. The
ESP32's internal pull-up is ~45 kΩ, so only ~70 µA of capacitively coupled
noise fakes an edge, and the ESP32 sits a couple of feet from the pump motor
and its contactor. The symptom is a pegged flow rate with zero actual flow.

Grounding does not fix it, and the armored lead is not a fix either. Grounding
addresses common-mode noise riding on both conductors; this is capacitive
injection into a high-impedance node, which has no ground path to interrupt.
The armor is mechanical protection, and unless it is terminated 360° in a
conductive gland it is landed by a pigtail wire, whose inductance makes it
nearly useless against broadband contactor noise. Both were tried.

The fix is source impedance, not shielding:

```
3V3 ──[ R1 1k ]──┬─────────────── GPIO 4
                 │
                 ├──[ C1 100nF ]── GND
                 │
                 └──[ R2 100R ]──══ armored pair ══ [ REED ] ══ GND
```

| Ref | Part | Note |
|---|---|---|
| R1 | 1 kΩ | pull-up to 3V3 — **required** for noise immunity. Sets 3.3 mA. (The internal pull-up stays on too, so the pin idles HIGH rather than floating if R1 or the meter is disconnected.) |
| C1 | 100 nF | pin to GND. 100 µs with R1 — swallows transients, invisible against a ~100 ms closure. |
| R2 | 100 Ω | limits C1's discharge into the reed on closure; contact-life insurance. |

- **R1 is the whole fix.** At 45 kΩ it takes ~70 µA to fake an edge; at 1 kΩ it
  takes 3.3 mA. A contactor spike through a few pF delivers roughly half a
  milliamp — comfortably over the first threshold, nowhere near the second.
- C1 filters fast spikes so the 1 ms samples see a cleaner level; the
  firmware's 20 ms settle time rejects anything longer that still gets through.
- Land the armor on ESP32 GND at the enclosure entry, **one point only**. It
  will not fix anything, but don't build a loop with it either.
- 3.3 mA is adequate wetting current for a sealed glass reed. (Oxide buildup is
  an open-contact problem; an inert-gas-filled reed does not need the ~10 mA a
  relay contact would.)

**If this is not enough**, `ranch/water/diag` will say so: `edges` climbing
while the pump cycles but flat while it is off points at the contactor. The
next step is a snubber across the contactor coil, or moving the ESP32's supply
off that circuit — suppress at the source rather than hardening the receiver.
An isolated 24 V current loop into an optocoupler is the escalation beyond
that, but it is aimed at long runs and lightning exposure, and buys nothing
over two feet inside one enclosure.

### Pressure input

Two constraints drive that pin and that divider, and both are easy to get
wrong:

- **It must be an ADC1 pin (GPIO 0-4 on the C3).** ADC2 (GPIO 5) is shared
  with the WiFi radio and unreliable once WiFi is up.
- **The ADC saturates well below the rail.** At 11 dB attenuation it is only
  factory-calibrated up to roughly 2500 mV, not the full 3.3 V,
  so a divider sized to just clear 3.3 V flattens the top of the pressure
  range. With the 4.7k/10k divider, readings top out at ~2.5 V on the pin,
  which is ~79 PSI — fine for a typical well system. Swapping the 4.7k for
  10k (and updating `PRESSURE_DIVIDER_RATIO`) would cover the full 0-100 PSI.
  Never wire the 4.5 V sensor output to a GPIO directly.

## Firmware

Source: [src/main.cpp](src/main.cpp)

- Samples the flow pin every `SAMPLE_MS` (1 ms) from a high-priority
  `esp_timer`, and only changes the contact's state once the pin has held the
  new level for `SETTLE_MS` (20 ms) straight. A pulse counts on each settled
  closed → open change, no sooner than `MIN_PERIOD_MS` (100 ms) after the last.
  Noise and hum can't hold a level for 20 ms (a 60 Hz half-cycle is 8.3 ms;
  at the earlier 5 ms threshold hum read as a steady ~135 GPM), while real
  closures last seconds. The earlier edge-interrupt version read the pin level
  once per edge, so one misread at a real release could silently drop that
  pulse; sampling can't. The timer task outranks `loop()`, so nothing is lost
  while WiFi reconnects.
- Converts pulses to cubic feet using `PULSES_PER_CF`, and to gallons at
  7.48052 gal/cf (calibrate against a measured volume for your specific meter)
- Publishes JSON (`{"gpm":...,"gallons_total":...,"cf_total":...,
  "pulse_count":...,"suspect":...}`) to `ranch/water/flow` every `PUBLISH_MS`
  (15 s default); `suspect` is true when the rate exceeds `MAX_PLAUSIBLE_GPM`.
  `gpm` comes from the time between pulses (a 5 s window at 0.748 gal/pulse
  could only show multiples of ~9 GPM); it decays while the next pulse is
  overdue and reads 0 after `FLOW_STOP_MS` (2 min) without one.
- Publishes counters to `ranch/water/diag` every `DIAG_MS` (60 s), covering that minute
  (`{"edges":...,"accepted":...,"glitches":...,"too_fast":...,
  "glitch_max_ms":...,"accept_min_ms":...,"pin":...,"rssi":...}`). With a
  clean front end `edges` is almost exactly 2× `accepted`, one close plus one
  open per pulse. `glitches` are excursions that reverted before `SETTLE_MS`;
  `glitch_max_ms` creeping toward 20 means noise is close to faking a pulse.
- Reads the pressure sensor via `analogReadMilliVolts` (uses the ESP32's
  built-in ADC calibration), undoes the divider, and maps 0.5-4.5V to 0-100
  PSI; publishes JSON (`{"psi":...,"volts":...}`) to `ranch/water/pressure`
  on the same interval
- Publishes a boot status message to `ranch/water/status`
- Persists the cumulative pulse count to NVS (flash) every `NVS_SAVE_MS`
  (5 min default) so total usage survives power loss/reboot; skips the write
  when the count hasn't moved, so an idle well doesn't burn flash
- Reconnects WiFi and MQTT automatically if either drops

## Setup

1. Copy [include/secrets.h.template](include/secrets.h.template) to
   `include/secrets.h` and fill in your WiFi SSID/password and MQTT broker
   address. `secrets.h` is gitignored — never commit real credentials.
2. Set `PULSES_PER_CF` in [src/main.cpp](src/main.cpp) for your meter. This
   meter's reed output is 0.748 gal (0.1 cf) per pulse, so the value is 10; the
   earlier default of 100 read flow 10x low.
   Note that a mechanical reed cannot produce the 450 pulses/gallon this repo
   previously assumed — that would be 150 Hz at the meter's rated flow.
3. If your broker requires auth, uncomment `MQTT_USER`/`MQTT_PASS` in
   `main.cpp`.
4. Build and upload with [PlatformIO](https://platformio.org/) (env:
   `esp32-c3-devkitm-1`, see [platformio.ini](platformio.ini)).

## Repo layout

- `src/` — firmware entry point
- `include/` — headers, including the secrets template
- `lib/` — project-local libraries (currently empty)
- `test/` — PlatformIO test directory (currently empty)
