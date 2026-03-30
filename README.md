# HCSR04MultiEcho

HC-SR04 超音波センサーの内部 LM324 アナログ信号をタップし、ソフトウェアエンベロープ検波でマルチエコーを検出する Arduino ライブラリ。

通常の Echo ピンでは最初のエコーしか取得できませんが、本ライブラリを使うと複数物体の同時検出、2次・3次反射エコー、エコー振幅の取得が可能です。

## 対応ハードウェア

- ESP32-S3 (M5Stack STAMP S3 等)
- HC-SR04 超音波センサー (LM324 Pin7 からアナログ信号をタップ)

## 接続

```
HC-SR04 VCC     ← 3.3V (または 5V + 分圧)
HC-SR04 Trigger ← ESP32 GPIO (任意)
HC-SR04 Echo    → (未使用)
LM324 Pin7      → ESP32 ADC GPIO (直結)
```

**3.3V 運用時**: LM324 Pin7 出力は ADC レンジ内に収まるため直結可能。信号振幅が大きく SNR が良好。

**5V 運用時**: LM324 Pin7 出力が ADC レンジを超えるため、分圧回路が必要。

### LM324 Pin7 の位置

HC-SR04 基板上の LM324 (SOIC-14) の Pin 7 (U2B 出力) にワイヤーをはんだ付けします。

```
受信アナログ信号経路:
  RXトランスデューサ → U2D (初段アンプ) → U2C (BPF) → U2B (2段目アンプ) → U2A (コンパレータ)
                                                         ↑
                                                    Pin7: タップポイント
```

## インストール

### PlatformIO

`platformio.ini` に以下を追加:

```ini
lib_deps =
    https://github.com/katsuyoshi/HCSR04MultiEcho.git
```

## 使い方

```cpp
#include <Arduino.h>
#include <HCSR04MultiEcho.h>

HCSR04MultiEcho sensor(/*trigPin=*/3, /*analogPin=*/5);

void setup() {
    Serial.begin(115200);
    sensor.begin();
}

void loop() {
    int count = sensor.capture();

    for (int i = 0; i < count; i++) {
        const Echo& e = sensor.getEcho(i);
        Serial.printf("Echo #%d: %.1f cm (amp=%u)\n",
                      i + 1, e.distance_cm, e.amplitude);
    }

    delay(100);
}
```

### 最大振幅エコーの距離を取得

```cpp
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
        Serial.printf("Distance: %.1f cm  (amp=%u)\n", e.distance_cm, e.amplitude);
    }

    delay(200);
}
```

## API

### コンストラクタ

```cpp
HCSR04MultiEcho(int trigPin, int analogPin)
```

### 初期化

| メソッド | 説明 |
|---|---|
| `begin(resolution)` | ADC 初期化。resolution はデフォルト 12bit |

### パラメータ設定

| メソッド | デフォルト | 説明 |
|---|---|---|
| `setCaptureTime(us)` | 30000 | キャプチャ時間 (μs)。30ms ≈ 約5m |
| `setEnvelopeWindow(n)` | 5 | エンベロープ抽出ウィンドウ幅 (サンプル数) |
| `setNoiseThreshold(val)` | 500 | エコー検出の閾値 |
| `setTxDetectThreshold(val)` | 500 | TX バースト検出の閾値 |
| `setTxSearchLimit(us)` | 3000 | TX 終了を探す範囲 (μs) |
| `setTxFallback(us)` | 1500 | TX 自動検出失敗時のフォールバック (μs) |
| `setMinEchoGap(n)` | 20 | エコー間の最小サンプル間隔 |
| `setSpeedOfSound(v)` | 0.0343 | 音速 (cm/μs)。20℃ で 0.0343 |

### 測定

| メソッド | 説明 |
|---|---|
| `capture()` | トリガー送信 → 波形キャプチャ → エンベロープ抽出 → エコー検出を一括実行。エコー数を返す |

### 結果取得

| メソッド | 説明 |
|---|---|
| `getEchoCount()` | 検出されたエコー数 |
| `getEcho(index)` | Echo 構造体への参照 (distance_cm, amplitude, time_us, sample_idx) |
| `getTxEndUs()` | 検出された TX バースト終了時刻 (μs) |

### 生データアクセス

| メソッド | 説明 |
|---|---|
| `getSampleCount()` | キャプチャされたサンプル数 |
| `getWaveform()` | 生 ADC 値の配列 |
| `getEnvelope()` | エンベロープ値の配列 |
| `getTimestamps()` | タイムスタンプの配列 (μs) |

### バッファサイズ設定

`platformio.ini` の `build_flags` で変更可能:

```ini
build_flags =
    -DHCSR04_MAX_SAMPLES=3000
    -DHCSR04_MAX_ECHOES=20
```

## Echo 構造体

```cpp
struct Echo {
    float    distance_cm;  // 距離 (cm)
    uint16_t amplitude;    // エンベロープピーク値
    uint16_t sample_idx;   // ピークのサンプルインデックス
    uint32_t time_us;      // TX終了からの経過時間 (μs)
};
```

## 原理

1. HC-SR04 の LM324 Pin7 (U2B 出力) から 40kHz キャリア付きのアナログ受信信号を取得
2. ESP32 の ADC で最速ループサンプリング (~16.5kHz)
3. スライディングウィンドウ (max-min) でソフトウェアエンベロープ抽出
4. TX バースト終了を自動検出し、それ以降のエンベロープピークをエコーとして検出
5. 各エコーの距離・振幅を算出

## ライセンス

MIT
