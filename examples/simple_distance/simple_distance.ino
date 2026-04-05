#include <Arduino.h>
#include <HCSR04MultiEcho.h>

// ピン設定
constexpr int PIN_TRIG   = 3;
constexpr int PIN_ANALOG = 5;

HCSR04MultiEcho sensor(PIN_TRIG, PIN_ANALOG);

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) delay(10);

    sensor.beginDMA();             // 83.3kHz DMA サンプリング
    sensor.setCaptureTime(12000);  // 12ms (最大約1.2m)

    Serial.println("=== Simple Distance ===");
}

void loop() {
    int count = sensor.capture();

    if (count > 0) {
        Serial.printf("Distance: %.1f cm  (raw=%.1f, echoes=%d)\n",
                      sensor.getDistance(), sensor.getRawDistance(), count);
    } else {
        Serial.println("No echo detected");
    }

    delay(100);
}
