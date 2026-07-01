# ESP32 Water Meter Monitor

Firmware for an ESP32-C3 that counts pulses from a flow meter on the well pump
and publishes flow rate / cumulative usage to MQTT.

## Hardware

- ESP32-C3 DevKitM-1 (no onboard user LED — confirmed by scanning GPIO2/8/9/10,
  only the red power LED lit)
- Pulse-output flow meter wired to GPIO 4 (GPIO 18/19 are USB D-/D+ on the
  ESP32-C3, so they're avoided)
- External status LED on GPIO 10 (220-330Ω resistor -> LED anode -> cathode ->
  GND), flashed briefly on each MQTT publish

## Firmware

Source: [src/main.cpp](src/main.cpp)

- Counts flow-meter pulses in an IRAM interrupt handler with a software
  debounce (`DEBOUNCE_US`, default 500 µs) to avoid double-counting
- Converts pulses to gallons using `PULSES_PER_GALLON` (calibrate against a
  measured volume for your specific meter)
- Publishes JSON (`{"gpm":...,"gallons_total":...,"pulse_count":...}`) to
  `ranch/water/flow` every `PUBLISH_MS` (5s default)
- Publishes a boot status message to `ranch/water/status`
- Persists the cumulative pulse count to NVS (flash) every `NVS_SAVE_MS`
  (5 min default) so total usage survives power loss/reboot
- Reconnects WiFi and MQTT automatically if either drops

## Setup

1. Copy [include/secrets.h.template](include/secrets.h.template) to
   `include/secrets.h` and fill in your WiFi SSID/password and MQTT broker
   address. `secrets.h` is gitignored — never commit real credentials.
2. Calibrate `PULSES_PER_GALLON` in [src/main.cpp](src/main.cpp) against your
   actual meter (actual_pulses / measured_gallons).
3. If your broker requires auth, uncomment `MQTT_USER`/`MQTT_PASS` in
   `main.cpp`.
4. Build and upload with [PlatformIO](https://platformio.org/) (env:
   `esp32-c3-devkitm-1`, see [platformio.ini](platformio.ini)).

## Repo layout

- `src/` — firmware entry point
- `include/` — headers, including the secrets template
- `lib/` — project-local libraries (currently empty)
- `test/` — PlatformIO test directory (currently empty)
