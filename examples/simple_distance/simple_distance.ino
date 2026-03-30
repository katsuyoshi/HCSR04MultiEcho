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

    sensor.begin();
    Serial.println("=== Simple Distance (Strongest Echo) ===");
}

void loop() {
    int count = sensor.capture();

    if (count > 0) {
        // 最大振幅のエコーを探す
        int best = 0;
        for (int i = 1; i < count; i++) {
            if (sensor.getEcho(i).amplitude > sensor.getEcho(best).amplitude) {
                best = i;
            }
        }
        const Echo& e = sensor.getEcho(best);
        Serial.printf("Distance: %.1f cm  (amp=%u, echoes=%d)\n",
                      e.distance_cm, e.amplitude, count);
    } else {
        Serial.println("No echo detected");
    }

    delay(200);
}
