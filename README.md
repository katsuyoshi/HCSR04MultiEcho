# HCSR04MultiEcho

HC-SR04 超音波センサーの内部 LM324 アナログ信号をタップし、ソフトウェアエンベロープ検波でマルチエコーを検出する Arduino ライブラリ。

通常の Echo ピンでは最初のエコーしか取得できませんが、本ライブラリを使うと複数物体の同時検出、2次・3次反射エコー、エコー振幅の取得が可能です。

ADC DMA による 83.3kHz 高速サンプリングと 13 点中央値フィルタにより、sd=0.28cm の安定した距離測定を実現します。

## 対応ハードウェア

- ESP32-S3 (M5Stack STAMP S3 等)
- HC-SR04 超音波センサー (LM324 Pin7 からアナログ信号をタップ)

## 接続

```
                  HC-SR04
                 ┌────────────────────┐
                 │  ┌──────┐  LM324  │
  3.3V ──────────┤  │ TX/RX│  (SOIC) │
                 │  └──────┘  ┌────┐ │
                 │            │ U2B│ │
  ESP32 GPIO ────┤ Trigger    │Pin7├─┼──── ESP32 ADC GPIO
                 │            └────┘ │
            x ──┤ Echo (未使用)      │
                 │                    │
  GND ───────────┤ GND               │
                 └────────────────────┘
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

### 基本

```cpp
#include <Arduino.h>
#include <HCSR04MultiEcho.h>

HCSR04MultiEcho sensor(/*trigPin=*/3, /*analogPin=*/5);

void setup() {
    Serial.begin(115200);
    sensor.beginDMA();              // 83.3kHz DMA サンプリング
    sensor.setCaptureTime(12000);   // 12ms (最大約1.2m)
}

void loop() {
    sensor.setTemperature(25.0);  // 気温 25℃ で音速補正
    int count = sensor.capture();

    if (count > 0) {
        Serial.printf("Distance: %.1f cm\n", sensor.getDistance());
    }

    delay(100);
}
```

`beginDMA()` で ADC DMA を初期化し、`capture()` でトリガー送信からエコー検出・中央値フィルタまで一括実行します。`getDistance()` で中央値フィルタ後の安定した距離が得られます。

### マルチエコーの取得

```cpp
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

### analogRead モード

DMA を使わず `analogRead()` でサンプリングするモードも利用できます。実効サンプリングレートは ~16.5kHz となり精度は低下します。

```cpp
sensor.begin();  // analogRead モード
```

## API

### コンストラクタ

```cpp
HCSR04MultiEcho(int trigPin, int analogPin)
```

### 初期化

| メソッド | 説明 |
|---|---|
| `beginDMA(sampleRate)` | DMA モード初期化。sampleRate はデフォルト 83333 (83.3kHz) |
| `begin(resolution)` | analogRead モード初期化。resolution はデフォルト 12bit |

### パラメータ設定

| メソッド | デフォルト | 説明 |
|---|---|---|
| `setCaptureTime(us)` | 30000 | キャプチャ時間 (μs)。`2 × 最大距離(cm) / 0.0343 + 5000` で算出 |
| `setEnvelopeWindow(n)` | 5 | エンベロープ抽出ウィンドウ幅 (サンプル数) |
| `setEnvelopeSmoothing(n)` | 5 | エンベロープ移動平均窓 (0=無効) |
| `setNoiseThreshold(val)` | 500 | エコー検出の閾値 |
| `setTxFallback(us)` | 1500 | TX 自動検出失敗時のフォールバック (μs) |
| `setMinEchoGap(n)` | 20 | エコー間の最小サンプル間隔 |
| `setSpeedOfSound(v)` | 0.0343 | 音速 (cm/μs)。20℃ で 0.0343 |
| `setTemperature(celsius)` | - | 温度 (℃) から音速を自動設定 |
| `setMedianFilterSize(n)` | 13 | 中央値フィルタ窓サイズ (0=無効、最大 HCSR04_MAX_MEDIAN) |

### キャプチャ時間の目安

| 最大距離 | 推奨 captureTime |
|---|---|
| 50cm | 8,000 μs |
| 100cm | 12,000 μs |
| 200cm | 17,000 μs |
| 400cm (HC-SR04最大) | 28,000 μs |

### 測定

| メソッド | 説明 |
|---|---|
| `capture()` | トリガー送信 → 波形キャプチャ → エンベロープ抽出 → エコー検出 → 中央値フィルタを一括実行。エコー数を返す |

### 距離取得

| メソッド | 説明 |
|---|---|
| `getDistance()` | 中央値フィルタ後の距離 (cm)。エコーなし: NAN |
| `getRawDistance()` | フィルタ前の生の距離 (cm)。エコーなし: NAN |
| `getBestEchoIndex()` | 最大振幅エコーのインデックス (-1=なし) |
| `isFilterReady()` | 中央値フィルタバッファが満杯か |
| `resetFilter()` | 中央値フィルタバッファをクリア |

### マルチエコー取得

| メソッド | 説明 |
|---|---|
| `getEchoCount()` | 検出されたエコー数 |
| `getEcho(index)` | Echo 構造体への参照 |
| `getTxPeakUs()` | TX バーストピーク時刻 (μs、距離計算の基準点) |
| `getTxEndUs()` | TX 残響終了時刻 (μs) |

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
    -DHCSR04_MAX_MEDIAN=21
```

## Echo 構造体

```cpp
struct Echo {
    float    distance_cm;  // 距離 (cm)
    uint16_t amplitude;    // エンベロープピーク値
    uint16_t sample_idx;   // ピークのサンプルインデックス
    uint32_t time_us;      // TXピークからの経過時間 (μs)
};
```

## 処理パイプライン

1. ADC DMA で高速サンプリング (83.3kHz)
2. スライディングウィンドウ (max-min) でエンベロープ抽出
3. 移動平均でエンベロープ平滑化
4. 2パス TX 検出: 1st pass で全ピーク検出 → 最初のピーク = TX バースト → TX 終了検出 → 2nd pass でエコーのみ検出
5. 各エコーの距離・振幅算出 (TX ピーク基準)
6. 13 点中央値フィルタで測定値を安定化 (sd: 2.95 → 0.28cm)

## ライセンス

MIT
