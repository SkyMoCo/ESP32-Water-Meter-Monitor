#include <Arduino.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>

// ── Config ────────────────────────────────────────────────────────────────────
#define FLOW_PIN          4           // GPIO 18/19 are USB D-/D+ on ESP32-C3, use 4 instead
#define LED_PIN           2           // built-in LED on esp32dev
#define PULSES_PER_GALLON 450.0f      // CALIBRATE: actual_pulses / measured_gallons
// At 50 GPM: 375 Hz → 2.67 ms period.  DEBOUNCE_US must be < 1333 µs (half-period).
// Start at 500 µs; if you still see double-counts, lower to 200 µs.
#define DEBOUNCE_US       500
const char* WIFI_SSID     = "ForeverShop";
const char* WIFI_PASSWORD = "@1680Division";
const char* MQTT_BROKER   = "172.26.0.11";   // your existing Mosquitto broker

#define MQTT_PORT      1883
#define MQTT_TOPIC     "pumphouse/flow"
#define MQTT_CLIENT_ID "pumphouse-esp32"
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

    wifiEnsure();
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setKeepAlive(60);
    mqttEnsure();

    if (mqtt.connected()) {
        char statusMsg[96];
        snprintf(statusMsg, sizeof(statusMsg),
                 "boot pin=%d pulses_per_gal=%.0f total_pulses=%llu total_gal=%.1f",
                 FLOW_PIN, PULSES_PER_GALLON, saved, saved / PULSES_PER_GALLON);
        mqtt.publish("pumphouse/flow/status", statusMsg);
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

        char payload[72];
        snprintf(payload, sizeof(payload),
                 "{\"gpm\":%.2f,\"total_gallons\":%.3f}", gpm, totalGallons);
        Serial.println(payload);

        if (mqtt.connected()) {
            digitalWrite(LED_PIN, HIGH);
            mqtt.publish(MQTT_TOPIC, payload, /*retained=*/false);
            delay(100);
            digitalWrite(LED_PIN, LOW);
        }
    }

    if (now - lastNvsSave >= NVS_SAVE_MS) {
        lastNvsSave = now;
        nvsFlush();
    }
}
