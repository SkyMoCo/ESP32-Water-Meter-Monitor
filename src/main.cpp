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
// at 10 pulses/cf (0.748 gal/pulse) a 3/4" meter running at its ~20 GPM rating
// pulses every 2.2 s, so the closure is on the order of a second.  A ~11 GPM
// sprinkler zone measured one magnet revolution (= one pulse) per ~4 s.  Anything asserted
// for less than MIN_LOW_MS is contact bounce or noise, not water.
//
// MIN_LOW_MS must also clear a half-cycle of mains hum (8.3 ms at 60 Hz, 10 ms
// at 50 Hz).  At 5 ms, hum coupled onto the line passed as real closures, and
// since MIN_PERIOD only drops every other one, it read as a steady ~135 GPM.
#define MIN_LOW_MS          20       // 2x a mains half-cycle; still well under a real closure
#define MIN_PERIOD_MS       100      // 10 Hz = 450 GPM at 10 pulses/cf, far above anything this meter can do
#define MAX_PLAUSIBLE_GPM   30.0f    // above the meter's rating -> flag the sample as suspect

// ── Calibration ───────────────────────────────────────────────────────────────
// Meter's reed output is 0.748 gal/pulse = 0.1 cf/pulse (one pulse per magnet
// revolution).  The earlier value of 100 undercounted flow 10x.
#define PULSES_PER_CF     10.0f
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
#define FLOW_STOP_MS 120000   // no pulse for this long -> report 0 GPM (floor of ~0.37 GPM at 0.748 gal/pulse)

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
static volatile uint32_t maxShortUs    = 0;   // longest rejected closure this interval -- how close noise/bounce gets to MIN_LOW
static volatile uint32_t minAcceptUs   = 0;   // shortest accepted closure this interval, 0 if none

static volatile uint64_t assertedUs    = 0;   // start of the closure currently in progress, 0 if none
static volatile uint64_t lastAcceptUs  = 0;
static volatile uint64_t lastPeriodUs  = 0;   // time between the last two accepted pulses, 0 until there are two

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
        const uint32_t held32 = heldUs > UINT32_MAX ? UINT32_MAX : (uint32_t)heldUs;
        if (heldUs < MIN_LOW_US) {
            ++shortLowCount;
            if (held32 > maxShortUs) maxShortUs = held32;
        } else if (lastAcceptUs != 0 && (now - lastAcceptUs) < MIN_PERIOD_US) {
            ++tooFastCount;
        } else {
            if (lastAcceptUs != 0) lastPeriodUs = now - lastAcceptUs;
            lastAcceptUs = now;
            if (minAcceptUs == 0 || held32 < minAcceptUs) minAcceptUs = held32;
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
        const uint32_t shortMax  = maxShortUs;    maxShortUs    = 0;
        const uint32_t acceptMin = minAcceptUs;   minAcceptUs   = 0;
        const uint64_t lastPulse = lastAcceptUs;
        const uint64_t period    = lastPeriodUs;
        portEXIT_CRITICAL(&isr_mux);

        const float cfTotal      = tot / PULSES_PER_CF;
        const float totalGallons = cfTotal * GAL_PER_CF;

        // GPM from the time between pulses rather than pulses per window: at
        // 0.748 gal/pulse a 5 s window can only show multiples of ~9 GPM. Once
        // the next pulse is overdue, the elapsed time is used instead, so the
        // reading decays as flow slows and drops to 0 after FLOW_STOP_MS.
        const uint64_t sinceUs   = lastPulse ? esp_timer_get_time() - lastPulse : 0;
        const uint64_t spanUs    = period > sinceUs ? period : sinceUs;
        const float    gpm       = (period == 0 || spanUs > FLOW_STOP_MS * 1000ULL)
                                   ? 0.0f
                                   : (GAL_PER_CF / PULSES_PER_CF) * 60e6f / (float)spanUs;
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
        // short_max_ms / accept_min_ms show where real closures and rejects
        // actually fall relative to MIN_LOW_MS.
        char diagPayload[224];
        snprintf(diagPayload, sizeof(diagPayload),
                 "{\"edges\":%lu,\"accepted\":%lu,\"short_lows\":%lu,"
                 "\"too_fast\":%lu,\"short_max_ms\":%.1f,\"accept_min_ms\":%.1f,"
                 "\"pin\":%d,\"rssi\":%d}",
                 (unsigned long)edges, (unsigned long)n, (unsigned long)shortLows,
                 (unsigned long)tooFast, shortMax / 1000.0f, acceptMin / 1000.0f,
                 gpio_get_level((gpio_num_t)FLOW_PIN),
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
