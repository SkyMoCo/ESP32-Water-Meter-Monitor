#include <Arduino.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <driver/gpio.h>   // gpio_get_level() is IRAM-safe; digitalRead() is not guaranteed to be

// ── Flow input ────────────────────────────────────────────────────────────────
// EKM 3/4" pulse meter: dry-contact (normally open) reed switch, no polarity,
// on its factory 24" armored lead.  The ESP32 mounts at the end of that lead,
// so the whole run is ~2 ft -- and a couple of feet from the pump contactor.
//
//   3V3 -> R1 1k -> FLOW_PIN, C1 100nF from FLOW_PIN to GND,
//   FLOW_PIN -> R2 100R -> cable -> reed -> GND.
//   Contact closed -> pin pulled LOW.
//
// R1 REPLACES the internal pull-up, and is the entire fix for the phantom
// pulses the first build counted.  The internal pull-up is ~45k, where only
// ~70 uA of capacitively coupled noise fakes an edge; at 1k it takes 3.3 mA.
// Grounding and the cable armor were both tried first and neither helped --
// capacitive injection into a high-impedance node has no ground path to break.
// C1 (100 us with R1) keeps short transients from flooding this ISR with edges.
#define FLOW_PIN            4        // not a strapping pin on the C3 (GPIO2/8/9 are) and not
                                     // USB (GPIO18/19 are D-/D+)
#define FLOW_ACTIVE_LOW     1        // 1 = contact closure drives the pin LOW (reed to GND, pull-up to 3V3)
                                     // 0 = closure drives it HIGH (only if an inverting front end is ever added)

// Pulse qualification.  A real closure holds the line asserted for tens of ms:
// at 100 pulses/cf a 3/4" meter running at its ~20 GPM rating turns over 4.5
// times a second (224 ms period), so the closure is ~100 ms.  Anything asserted
// for less than MIN_LOW_MS is contact bounce or noise, not water.
#define MIN_LOW_MS          5        // 20x margin against the shortest plausible real closure
#define MIN_PERIOD_MS       20       // reject a second pulse sooner than this (>50 Hz is not physical here)
#define MAX_PLAUSIBLE_GPM   30.0f    // above the meter's rating -> flag the sample as suspect

// ── Calibration ───────────────────────────────────────────────────────────────
// CALIBRATE: run a known volume through the meter and set this to
// actual_pulses / measured_cubic_feet.  100 is what the existing MQTT history
// implies; confirm it against the meter's register face before trusting totals.
#define PULSES_PER_CF     100.0f
#define GAL_PER_CF        7.48052f

// 0-100 PSI sensor (0.5-4.5V linear output) on well pump discharge, read via a
// 4.7k (sensor->pin) / 10k (pin->GND) divider so 0.5-4.5V maps to ~0.34-3.06V.
// The C3's ADC is only calibrated up to ~2500 mV at 11 dB, so readings
// saturate at ~2.5V on the pin = ~3.68V from the sensor = ~79 PSI.  Fine for
// a well system running well below that; swap the 4.7k for 10k (and update
// the ratio) if the full 0-100 PSI span is ever needed.
#define PRESSURE_PIN            3      // ADC1_CH3.  MUST be ADC1 (GPIO0-4): ADC2 (GPIO5) is owned
                                       // by the WiFi radio and unreliable while connected.
#define PRESSURE_DIVIDER_RATIO  (10.0f / (4.7f + 10.0f))  // R2 / (R1 + R2)
#define PRESSURE_V_MIN          0.5f   // sensor output at 0 PSI
#define PRESSURE_V_MAX          4.5f   // sensor output at 100 PSI
#define PRESSURE_PSI_MAX        100.0f

#define LED_PIN           10         // external LED -- board has no usable onboard LED; wire
                                     // GPIO10 -> 220-330R resistor -> LED anode -> cathode -> GND

#include "secrets.h"   // copy secrets.h.template → secrets.h and fill in your values

#define MQTT_PORT             1883
#define MQTT_TOPIC            "ranch/water/flow"
#define MQTT_TOPIC_PRESSURE   "ranch/water/pressure"
#define MQTT_TOPIC_STATUS     "ranch/water/status"
#define MQTT_TOPIC_DIAG       "ranch/water/diag"
#define MQTT_CLIENT_ID        "pumphouse-esp32"
// Uncomment if broker requires auth:
// #define MQTT_USER   "user"
// #define MQTT_PASS   "pass"

#define PUBLISH_MS   5000     // publish interval
#define NVS_SAVE_MS  300000   // flush total pulses to flash every 5 min (only if it changed)

#define MIN_LOW_US     (MIN_LOW_MS * 1000ULL)
#define MIN_PERIOD_US  (MIN_PERIOD_MS * 1000ULL)

// ── ISR state ─────────────────────────────────────────────────────────────────
// portMUX protects 32/64-bit variables shared between ISR and main loop.
static portMUX_TYPE      isr_mux        = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t intervalPulses = 0;
static volatile uint64_t totalPulses    = 0;

// Diagnostics: every edge is accounted for, so the published counters say
// whether the front end is clean or still picking up noise.
static volatile uint32_t edgeCount     = 0;   // interrupts seen
static volatile uint32_t shortLowCount = 0;   // asserted too briefly -- noise, or normal contact bounce
static volatile uint32_t tooFastCount  = 0;   // valid-looking but impossibly soon after the last accepted pulse

static volatile uint64_t assertedUs    = 0;   // start of the closure currently in progress, 0 if none
static volatile uint64_t lastAcceptUs  = 0;

// ── ISR ───────────────────────────────────────────────────────────────────────
// Qualification happens entirely in here, so nothing is lost while loop() is
// blocked in wifiEnsure(). A pulse is accepted only on RELEASE, and only if the
// contact stayed asserted for MIN_LOW_MS -- a transient cannot satisfy that.
void IRAM_ATTR onEdge() {
    const uint64_t now      = esp_timer_get_time();
    const int      level    = gpio_get_level((gpio_num_t)FLOW_PIN);
#if FLOW_ACTIVE_LOW
    const bool     asserted = (level == 0);
#else
    const bool     asserted = (level != 0);
#endif

    portENTER_CRITICAL_ISR(&isr_mux);
    ++edgeCount;
    if (asserted) {
        // Start of a candidate closure. Bounce mid-closure just restarts the
        // timing, which costs at most one pulse's worth of width, never a count.
        assertedUs = now;
    } else if (assertedUs != 0) {
        const uint64_t heldUs = now - assertedUs;
        assertedUs = 0;
        if (heldUs < MIN_LOW_US) {
            ++shortLowCount;
        } else if (lastAcceptUs != 0 && (now - lastAcceptUs) < MIN_PERIOD_US) {
            ++tooFastCount;
        } else {
            lastAcceptUs = now;
            ++intervalPulses;
            ++totalPulses;
        }
    }
    // A release with assertedUs == 0 means we booted mid-closure; ignore it
    // rather than crediting a pulse of unknown length.
    portEXIT_CRITICAL_ISR(&isr_mux);
}

// ── Peripherals ───────────────────────────────────────────────────────────────
static WiFiClient   wifiClient;
static PubSubClient mqtt(wifiClient);
static Preferences  prefs;
static uint64_t     lastSavedTotal = 0;

// ── Helpers ───────────────────────────────────────────────────────────────────
static void wifiEnsure() {
    if (WiFi.status() == WL_CONNECTED) return;
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_STA);
    Serial.printf("WiFi connecting to \"%s\"", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; ++i) {
        delay(500);
        Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED)
        Serial.printf(" OK  IP=%s\n", WiFi.localIP().toString().c_str());
    else
        Serial.printf(" FAILED  status=%d\n", WiFi.status());
}

static void mqttEnsure() {
    if (mqtt.connected()) return;
    Serial.print("MQTT connecting... ");
#if defined(MQTT_USER) && defined(MQTT_PASS)
    bool ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
#else
    bool ok = mqtt.connect(MQTT_CLIENT_ID);
#endif
    Serial.println(ok ? "OK" : "FAILED");
}

// Skips the write when nothing moved -- a well that sits idle would otherwise
// burn a flash write every 5 minutes for no reason.
static void nvsFlush() {
    portENTER_CRITICAL(&isr_mux);
    uint64_t total = totalPulses;
    portEXIT_CRITICAL(&isr_mux);
    if (total == lastSavedTotal) return;
    prefs.putULong64("total", total);
    lastSavedTotal = total;
}

// Reads the pressure sensor, undoes the divider, and maps 0.5-4.5V to 0-100 PSI.
static float readPressurePsi(float* outSensorVolts) {
    const uint32_t mv       = analogReadMilliVolts(PRESSURE_PIN);  // calibrated mV at the pin
    const float    vAtPin   = mv / 1000.0f;
    const float    vSensor  = vAtPin / PRESSURE_DIVIDER_RATIO;
    if (outSensorVolts) *outSensorVolts = vSensor;
    return (vSensor - PRESSURE_V_MIN) / (PRESSURE_V_MAX - PRESSURE_V_MIN) * PRESSURE_PSI_MAX;
}

// ── Arduino ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // Restore cumulative pulse count from NVS (survives power loss / reboot).
    prefs.begin("flow", /*readOnly=*/false);
    const uint64_t saved = prefs.getULong64("total", 0);
    portENTER_CRITICAL(&isr_mux);
    totalPulses = saved;
    portEXIT_CRITICAL(&isr_mux);
    lastSavedTotal = saved;
    Serial.printf("Resumed: %llu pulses (%.1f cf, %.1f gal)\n",
                  saved, saved / PULSES_PER_CF, saved / PULSES_PER_CF * GAL_PER_CF);

    pinMode(LED_PIN,  OUTPUT);
    // Internal pull-up always on, alongside R1: in parallel with 1k it changes
    // nothing, but with R1 missing or the front end unplugged the pin idles
    // HIGH (no flow) instead of floating and counting hum as pulses.
    pinMode(FLOW_PIN, INPUT_PULLUP);
    // CHANGE, not FALLING: the pulse is qualified by how long it stays
    // asserted, which needs both edges.
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), onEdge, CHANGE);

    analogSetPinAttenuation(PRESSURE_PIN, ADC_11db);  // widest range; calibrated to ~2.5V,
                                                      // i.e. ~79 PSI with the 4.7k/10k divider

    wifiEnsure();
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setKeepAlive(60);
    mqttEnsure();

    if (mqtt.connected()) {
        char statusMsg[160];
        snprintf(statusMsg, sizeof(statusMsg),
                 "boot pin=%d active_low=%d pulses_per_cf=%.0f min_low_ms=%d "
                 "total_pulses=%llu total_cf=%.2f",
                 FLOW_PIN, FLOW_ACTIVE_LOW, PULSES_PER_CF, MIN_LOW_MS,
                 saved, saved / PULSES_PER_CF);
        mqtt.publish(MQTT_TOPIC_STATUS, statusMsg);
        Serial.println(statusMsg);
    }
}

void loop() {
    static unsigned long lastPublish = 0;
    static unsigned long lastNvsSave = 0;

    wifiEnsure();
    if (WiFi.status() == WL_CONNECTED) {
        mqttEnsure();
        mqtt.loop();
    }

    const unsigned long now = millis();

    if (now - lastPublish >= PUBLISH_MS) {
        lastPublish = now;

        portENTER_CRITICAL(&isr_mux);
        const uint32_t n         = intervalPulses;
        intervalPulses           = 0;
        const uint64_t tot       = totalPulses;
        const uint32_t edges     = edgeCount;     edgeCount     = 0;
        const uint32_t shortLows = shortLowCount; shortLowCount = 0;
        const uint32_t tooFast   = tooFastCount;  tooFastCount  = 0;
        portEXIT_CRITICAL(&isr_mux);

        const float cfTotal      = tot / PULSES_PER_CF;
        const float totalGallons = cfTotal * GAL_PER_CF;
        const float gpm          = (n / PULSES_PER_CF) * GAL_PER_CF / (PUBLISH_MS / 60000.0f);
        const bool  suspect      = gpm > MAX_PLAUSIBLE_GPM;

        char payload[192];
        snprintf(payload, sizeof(payload),
                 "{\"gpm\":%.2f,\"gallons_total\":%.2f,\"cf_total\":%.2f,"
                 "\"pulse_count\":%llu,\"suspect\":%s}",
                 gpm, totalGallons, cfTotal, (unsigned long long)tot,
                 suspect ? "true" : "false");
        Serial.println(payload);

        // edges should be almost exactly 2x accepted when the front end is
        // clean (one assert + one release per pulse). A large short_lows count
        // with zero accepted means noise is still getting in.
        char diagPayload[160];
        snprintf(diagPayload, sizeof(diagPayload),
                 "{\"edges\":%lu,\"accepted\":%lu,\"short_lows\":%lu,"
                 "\"too_fast\":%lu,\"pin\":%d,\"rssi\":%d}",
                 (unsigned long)edges, (unsigned long)n, (unsigned long)shortLows,
                 (unsigned long)tooFast, gpio_get_level((gpio_num_t)FLOW_PIN),
                 WiFi.RSSI());
        Serial.println(diagPayload);

        float       sensorVolts = 0.0f;
        const float psi         = readPressurePsi(&sensorVolts);
        char pressurePayload[64];
        snprintf(pressurePayload, sizeof(pressurePayload),
                 "{\"psi\":%.1f,\"volts\":%.3f}", psi, sensorVolts);
        Serial.println(pressurePayload);

        if (mqtt.connected()) {
            digitalWrite(LED_PIN, HIGH);
            mqtt.publish(MQTT_TOPIC,          payload,         /*retained=*/false);
            mqtt.publish(MQTT_TOPIC_DIAG,     diagPayload,     /*retained=*/false);
            mqtt.publish(MQTT_TOPIC_PRESSURE, pressurePayload, /*retained=*/false);
            delay(100);
            digitalWrite(LED_PIN, LOW);
        }
    }

    if (now - lastNvsSave >= NVS_SAVE_MS) {
        lastNvsSave = now;
        nvsFlush();
    }
}
