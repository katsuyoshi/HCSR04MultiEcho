#include <Arduino.h>
#include <HCSR04MultiEcho.h>

// ===== ピン設定 (STAMP S3、配線に合わせて変更) =====
// 接続図 (3.3V 運用):
//   HC-SR04 VCC     ← 3.3V
//   HC-SR04 Trigger ← GPIO3
//   HC-SR04 Echo    → (未使用、3.3Vでは動作せず)
//   LM324 Pin7 → GPIO5 (ADC) 直結
//
constexpr int PIN_TRIG   = 3;
constexpr int PIN_ANALOG = 5;

HCSR04MultiEcho sensor(PIN_TRIG, PIN_ANALOG);

static bool running = true;
static bool waveform_mode = false;

// ===== 生波形 CSV 出力 =====
void dumpWaveform() {
    Serial.println("--- RAW ---");
    Serial.printf("tx_end_us=%lu\n", (unsigned long)sensor.getTxEndUs());
    Serial.println("idx,adc,envelope,time_us");
    uint32_t n = sensor.getSampleCount();
    const uint16_t* wf  = sensor.getWaveform();
    const uint16_t* env = sensor.getEnvelope();
    const uint32_t* ts  = sensor.getTimestamps();
    for (uint32_t i = 0; i < n; i++) {
        Serial.printf("%lu,%u,%u,%lu\n",
                      (unsigned long)i, wf[i], env[i],
                      (unsigned long)ts[i]);
    }
    Serial.println("--- END ---");
}

// ===== 生波形 Teleplot出力 =====
void dumpWaveformTeleplot() {
    uint32_t n = sensor.getSampleCount();
    const uint16_t* wf  = sensor.getWaveform();
    const uint16_t* env = sensor.getEnvelope();
    for (uint32_t i = 0; i < n; i++) {
        Serial.printf(">adc:%u\n>envelope:%u\n", wf[i], env[i]);
    }
}

// ===== 検出エコー出力 =====
void printEchoes() {
    int count = sensor.getEchoCount();
    Serial.printf("Echoes: %d  (captured %lu samples, tx_end=%lu us)\n",
                  count, (unsigned long)sensor.getSampleCount(),
                  (unsigned long)sensor.getTxEndUs());
    for (int i = 0; i < count; i++) {
        const Echo& e = sensor.getEcho(i);
        Serial.printf("  #%d: %6.1f cm  amp=%4u  t=%lu us\n",
                      i + 1, e.distance_cm, e.amplitude,
                      (unsigned long)e.time_us);
    }
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) delay(10);

    sensor.begin();

    Serial.println("=== HC-SR04 Multi-Echo Analyzer ===");
    Serial.printf("Pins: TRIG=%d  ANALOG=%d\n", PIN_TRIG, PIN_ANALOG);
    Serial.println("Auto-repeat mode. Send 's' to stop, 'g' to go.");
    Serial.println("  r = dump raw CSV once, w = Teleplot once, W = Teleplot continuous");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
        case 's': running = false; waveform_mode = false; Serial.println("Stopped."); break;
        case 'g': running = true;  waveform_mode = false; Serial.println("Running."); break;
        case 'W': running = true;  waveform_mode = true;  Serial.println("Waveform mode."); break;
        case 'r':
            sensor.capture();
            dumpWaveform();
            break;
        case 'w':
            sensor.capture();
            dumpWaveformTeleplot();
            break;
        }
    }

    if (!running) { delay(10); return; }

    int count = sensor.capture();

    if (waveform_mode) {
        dumpWaveformTeleplot();
    } else {
        for (int i = 0; i < count; i++) {
            const Echo& e = sensor.getEcho(i);
            Serial.printf(">echo%d_cm:%.1f\n", i + 1, e.distance_cm);
            Serial.printf(">echo%d_amp:%u\n", i + 1, e.amplitude);
        }
        Serial.printf(">echo_count:%d\n", count);
        Serial.printf(">tx_end:%lu\n", (unsigned long)sensor.getTxEndUs());
        for (int i = 0; i < count; i++) {
            const Echo& e = sensor.getEcho(i);
            Serial.printf("  A%d:%.1fcm(%u)", i + 1, e.distance_cm, e.amplitude);
        }
        if (count == 0) Serial.print("  no echo");
        Serial.println();
    }

    delay(100);
}
