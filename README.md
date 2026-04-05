# HCSR04MultiEcho

HC-SR04 超音波センサーの内部 LM324 アナログ信号をタップし、ソフトウェアエンベロープ検波でマルチエコーを検出する Arduino ライブラリ。

通常の Echo ピンでは最初のエコーしか取得できませんが、本ライブラリを使うと複数物体の同時検出、2次・3次反射エコー、エコー振幅の取得が可能です。

ADC DMA による 83.3kHz 高速サンプリング、スロープベースのエコー検出、TX バースト自動検出、13 点中央値フィルタにより、sd=0.09cm の安定した距離測定を実現します。

## 対応ハードウェア

- ESP32-S3 (M5Stack STAMP S3 等)
- ESP32-C3
- HC-SR04 超音波センサー (LM324 Pin7 からアナログ信号をタップ)

## 接続

```
                  HC-SR04
                 ┌────────────────────┐
                 │  ┌──────┐   LM324  │
  3.3V ──────────┤  │ TX/RX│   (SOIC) │
                 │  └──────┘   ┌────┐ │
                 │             │ U2B│ │
  ESP32 GPIO ────┤ Trigger     │Pin7├─┼──── ESP32 ADC GPIO
                 │             └────┘ │
             x ──┤ Echo(Unused)       │
                 │                    │
  GND ───────────┤ GND                │
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
        Serial.printf("Echo #%d: %.1f cm (amp=%u, slope=%d)\n",
                      i + 1, e.distance_cm, e.amplitude, e.max_slope);
    }

    delay(100);
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
| `beginDMA(sampleRate)` | ADC DMA 初期化。sampleRate はデフォルト 83333 (83.3kHz) |

### パラメータ設定

| メソッド | デフォルト | 説明 |
|---|---|---|
| `setCaptureTime(us)` | 30000 | キャプチャ時間 (μs)。`2 × 最大距離(cm) / 0.0343 + 5000` で算出 |
| `setNoiseThreshold(val)` | 500 | エコー検出の最小振幅閾値。ノイズフロア (`val/4`) もこの値から導出 |
| `setSlopeThreshold(val)` | 30 | エコー検出のスロープ閾値 (3点中心差分)。エンベロープの立ち上がり傾きがこの値以上でエコー候補 |
| `setSlopeConfirmCount(n)` | 3 | スロープ閾値超えの連続確認数。ノイズ耐性を調整 |
| `setEnvelopeSmoothing(n)` | 5 | エンベロープ移動平均窓 (0=無効) |
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
| `capture()` | トリガー送信 → 波形キャプチャ → エンベロープ抽出 → TX検出 → エコー検出 → 中央値フィルタを一括実行。エコー数を返す |

### 距離取得

| メソッド | 説明 |
|---|---|
| `getDistance()` | 中央値フィルタ後の距離 (cm)。エコーなし: NAN |
| `getRawDistance()` | フィルタ前の生の距離 (cm)。エコーなし: NAN |
| `getBestEchoIndex()` | 最大スロープエコーのインデックス (-1=なし) |
| `isFilterReady()` | 中央値フィルタバッファが満杯か |
| `resetFilter()` | 中央値フィルタバッファをクリア |

### マルチエコー取得

| メソッド | 説明 |
|---|---|
| `getEchoCount()` | 検出されたエコー数 (TX バースト除外後) |
| `getEcho(index)` | Echo 構造体への参照 |
| `getTxTimeUs()` | TX バースト検出時刻 (μs)。距離計算の基準点。未検出時 0 |

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
    int16_t  max_slope;    // 立ち上がり最大スロープ
    uint16_t sample_idx;   // スロープ上昇開始のサンプルインデックス
    uint32_t time_us;      // TX バーストからの経過時間 (μs)
};
```

## 処理パイプライン

1. ADC DMA で高速サンプリング (83.3kHz)
2. DC 除去 + 全波整流でエンベロープ抽出
3. 移動平均でエンベロープ平滑化
4. TX バースト自動検出 (DMA ウォームアップスキップ後、低閾値スロープスキャン)
5. スロープベースのエコー検出 (2状態: IDLE / IN_ECHO)
   - IDLE: スロープが閾値以上を連続N回超えたらエコー開始
   - IN_ECHO: エンベロープがノイズフロア以下を連続5回でエコー終了
   - ダブルローブ (同一エコー内の2ピーク) は1エコーとして扱う
6. TX 領域のエコーを除外し、TX 基準で距離を再計算
7. 最大スロープエコーを選択し、13 点中央値フィルタで安定化 (sd: 1.4 → 0.09cm)

## ライセンス

MIT
