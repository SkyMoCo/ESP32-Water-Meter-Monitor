#include <Arduino.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>

// ── Config ────────────────────────────────────────────────────────────────────
#define FLOW_PIN          4           // GPIO 18/19 are USB D-/D+ on ESP32-C3, use 4 instead
#define LED_PIN           10          // external LED -- board has no onboard LED; wire
                                       // GPIO10 -> 220-330R resistor -> LED anode -> cathode -> GND
#define PULSES_PER_GALLON 450.0f      // CALIBRATE: actual_pulses / measured_gallons
// At 50 GPM: 375 Hz → 2.67 ms period.  DEBOUNCE_US must be < 1333 µs (half-period).
// Start at 500 µs; if you still see double-counts, lower to 200 µs.
#define DEBOUNCE_US       500

// 0-100 PSI sensor (0.5-4.5V linear output) on well pump discharge, read via a
// 4.7k (sensor->pin) / 10k (pin->GND) divider so 0.5-4.5V maps to ~0.34-3.06V,
// safely inside the ESP32-C3 ADC's 3.3V max input.
#define PRESSURE_PIN            3      // ADC1_CH3 -- GPIO0/4/10/18/19 already spoken for; avoid GPIO2 (boot strap)
#define PRESSURE_DIVIDER_RATIO  (10.0f / (4.7f + 10.0f))  // R2 / (R1 + R2)
#define PRESSURE_V_MIN          0.5f   // sensor output at 0 PSI
#define PRESSURE_V_MAX          4.5f   // sensor output at 100 PSI
#define PRESSURE_PSI_MAX        100.0f

#include "secrets.h"   // copy secrets.h.template → secrets.h and fill in your values

#define MQTT_PORT             1883
#define MQTT_TOPIC            "ranch/water/flow"
#define MQTT_TOPIC_PRESSURE   "ranch/water/pressure"
#define MQTT_TOPIC_STATUS     "ranch/water/status"
#define MQTT_CLIENT_ID        "pumphouse-esp32"
// Uncomment if broker requires auth:
// #define MQTT_USER   "user"
// #define MQTT_PASS   "pass"

#define PUBLISH_MS   5000     // publish interval
#define NVS_SAVE_MS  300000   // flush total pulses to flash every 5 min

// ── ISR state ─────────────────────────────────────────────────────────────────
// portMUX protects 32/64-bit variables shared between ISR and main loop.
static portMUX_TYPE     isr_mux        = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t intervalPulses = 0;
static volatile uint64_t totalPulses    = 0;

// ── ISR ───────────────────────────────────────────────────────────────────────
void IRAM_ATTR onPulse() {
    // Software debounce: ignore edges within DEBOUNCE_US of the previous one.
    // Runs entirely in IRAM so it survives cache misses during flash operations.
    static uint64_t lastUs = 0;
    const  uint64_t now    = esp_timer_get_time();
    if ((now - lastUs) < DEBOUNCE_US) return;
    lastUs = now;
    portENTER_CRITICAL_ISR(&isr_mux);
    ++intervalPulses;
    ++totalPulses;
    portEXIT_CRITICAL_ISR(&isr_mux);
}

// ── Peripherals ───────────────────────────────────────────────────────────────
static WiFiClient   wifiClient;
static PubSubClient mqtt(wifiClient);
static Preferences  prefs;

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

static void nvsFlush() {
    portENTER_CRITICAL(&isr_mux);
    uint64_t total = totalPulses;
    portEXIT_CRITICAL(&isr_mux);
    prefs.putULong64("total", total);
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
    Serial.printf("Resumed: %llu pulses (%.1f gal)\n", saved, saved / PULSES_PER_GALLON);

    pinMode(LED_PIN,  OUTPUT);
    pinMode(FLOW_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), onPulse, FALLING);

    analogSetPinAttenuation(PRESSURE_PIN, ADC_11db);  // full 0-3.3V range for the divided signal

    wifiEnsure();
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setKeepAlive(60);
    mqttEnsure();

    if (mqtt.connected()) {
        char statusMsg[96];
        snprintf(statusMsg, sizeof(statusMsg),
                 "boot pin=%d pulses_per_gal=%.0f total_pulses=%llu total_gal=%.1f",
                 FLOW_PIN, PULSES_PER_GALLON, saved, saved / PULSES_PER_GALLON);
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
        const uint32_t n   = intervalPulses;
        intervalPulses     = 0;
        const uint64_t tot = totalPulses;
        portEXIT_CRITICAL(&isr_mux);

        const float gpm          = (n / PULSES_PER_GALLON) / (PUBLISH_MS / 60000.0f);
        const float totalGallons = tot / PULSES_PER_GALLON;

        char payload[96];
        snprintf(payload, sizeof(payload),
                 "{\"gpm\":%.2f,\"gallons_total\":%.3f,\"pulse_count\":%llu}",
                 gpm, totalGallons, (unsigned long long)tot);
        Serial.println(payload);

        float       sensorVolts = 0.0f;
        const float psi         = readPressurePsi(&sensorVolts);
        char pressurePayload[64];
        snprintf(pressurePayload, sizeof(pressurePayload),
                 "{\"psi\":%.1f,\"volts\":%.3f}", psi, sensorVolts);
        Serial.println(pressurePayload);

        if (mqtt.connected()) {
            digitalWrite(LED_PIN, HIGH);
            mqtt.publish(MQTT_TOPIC, payload, /*retained=*/false);
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
