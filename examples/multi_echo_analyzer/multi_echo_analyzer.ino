#include <Arduino.h>
#include <HCSR04MultiEcho.h>

// ===== ピン設定 (配線に合わせて変更) =====
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
    Serial.println("idx,adc,envelope,slope,time_us");
    uint32_t n = sensor.getSampleCount();
    const uint16_t* wf  = sensor.getWaveform();
    const uint16_t* env = sensor.getEnvelope();
    const uint32_t* ts  = sensor.getTimestamps();
    for (uint32_t i = 0; i < n; i++) {
        int16_t slope = 0;
        if (i > 0 && i + 1 < n) {
            slope = ((int16_t)env[i + 1] - (int16_t)env[i - 1]) / 2;
        }
        Serial.printf("%lu,%u,%u,%d,%lu\n",
                      (unsigned long)i, wf[i], env[i], slope,
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
        int16_t slope = 0;
        if (i > 0 && i + 1 < n) {
            slope = ((int16_t)env[i + 1] - (int16_t)env[i - 1]) / 2;
        }
        Serial.printf(">adc:%u\n>envelope:%u\n>slope:%d\n", wf[i], env[i], slope);
    }
}

// ===== キャプチャ情報サマリー =====
void dumpInfo() {
    int count = sensor.capture();
    uint32_t n = sensor.getSampleCount();
    const uint16_t* env = sensor.getEnvelope();
    const uint32_t* ts  = sensor.getTimestamps();

    Serial.println("--- INFO ---");
    Serial.printf("samples=%lu, tx_time_us=%lu, echoes=%d\n",
                  (unsigned long)n, (unsigned long)sensor.getTxTimeUs(), count);

    for (int i = 0; i < count; i++) {
        const Echo& e = sensor.getEcho(i);
        Serial.printf("  echo%d: %.1f cm  amp=%u  slope=%d  time_us=%lu  idx=%u\n",
                      i + 1, e.distance_cm, e.amplitude, e.max_slope,
                      (unsigned long)e.time_us, e.sample_idx);
    }
    Serial.println("--- END ---");
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) delay(10);

    sensor.beginDMA();             // 83.3kHz DMA サンプリング
    sensor.setCaptureTime(14000);  // 最大1.5m + マージン

    Serial.println("=== HC-SR04 Multi-Echo Analyzer ===");
    Serial.printf("Pins: TRIG=%d  ANALOG=%d\n", PIN_TRIG, PIN_ANALOG);
    Serial.println("Auto-repeat mode. Send 's' to stop, 'g' to go.");
    Serial.println("  r = dump raw CSV, w = Teleplot once, W = Teleplot continuous");
    Serial.println("  i = capture info (summary)");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
        case 's': running = false; waveform_mode = false; Serial.println("Stopped."); break;
        case 'g': running = true;  waveform_mode = false; sensor.resetFilter(); Serial.println("Running."); break;
        case 'W': running = true;  waveform_mode = true;  Serial.println("Waveform mode."); break;
        case 'r':
            sensor.capture();
            dumpWaveform();
            break;
        case 'w':
            sensor.capture();
            dumpWaveformTeleplot();
            break;
        case 'i':
            dumpInfo();
            break;
        }
    }

    if (!running) { delay(10); return; }

    int count = sensor.capture();

    if (waveform_mode) {
        dumpWaveformTeleplot();
    } else {
        if (count > 0) {
            Serial.printf(">distance:%.1f\n", sensor.getRawDistance());
            Serial.printf(">dist_median:%.1f\n", sensor.getDistance());
        }
        Serial.printf(">echo_count:%d\n", count);
    }

    delay(100);
}
