#ifndef HCSR04_MULTI_ECHO_H
#define HCSR04_MULTI_ECHO_H

#include <Arduino.h>
#include <math.h>
#include "driver/adc.h"
#include "soc/soc_caps.h"

#ifndef HCSR04_MAX_SAMPLES
#define HCSR04_MAX_SAMPLES 4000
#endif

#ifndef HCSR04_MAX_ECHOES
#define HCSR04_MAX_ECHOES 10
#endif

#ifndef HCSR04_MAX_MEDIAN
#define HCSR04_MAX_MEDIAN 21
#endif

struct Echo {
    float    distance_cm;
    uint16_t amplitude;
    int16_t  max_slope;    // 立ち上がり最大スロープ
    uint16_t sample_idx;
    uint32_t time_us;      // TXピークからの相対時間
};

class HCSR04MultiEcho {
public:
    HCSR04MultiEcho(int trigPin, int analogPin);

    // 初期化 (ADC DMA サンプリング)
    void beginDMA(uint32_t sampleRate = 83333);

    // パラメータ設定
    void setCaptureTime(uint32_t us);
    void setNoiseThreshold(uint16_t val);
    void setMinEchoGap(int gap);
    void setSpeedOfSound(float v);
    void setTemperature(float celsius);  // 温度(℃)から音速を設定
    void setEnvelopeSmoothing(int window);
    void setMedianFilterSize(int size);  // デフォルト13, 0=無効, 1-HCSR04_MAX_MEDIAN
    void setSlopeThreshold(uint16_t val);
    void setSlopeConfirmCount(int count);

    // 測定実行 (トリガー → キャプチャ → エンベロープ → エコー検出 → フィルタ)
    // returns echo_count
    int capture();

    // 結果取得
    int           getEchoCount() const;    // TX除外後のエコー数
    const Echo&   getEcho(int index) const;
    uint32_t      getTxTimeUs() const;     // TX バースト検出時刻 (μs), 未検出時 0

    // 距離取得 (最大スロープエコー基準)
    float getDistance() const;         // フィルタ後 (無効時は生値)。エコーなし: NAN
    float getRawDistance() const;      // 生値。エコーなし: NAN
    int   getBestEchoIndex() const;    // 最大スロープエコーのインデックス (-1=なし)
    bool  isFilterReady() const;       // フィルタバッファが満杯か
    void  resetFilter();

    // 生データアクセス (CSV出力・Teleplot等に使用)
    uint32_t        getSampleCount() const;
    const uint16_t* getWaveform() const;
    const uint16_t* getEnvelope() const;
    const uint32_t* getTimestamps() const;

private:
    int _trigPin;
    int _analogPin;

    // パラメータ (デフォルト値はコンストラクタで設定)
    uint32_t _captureUs;
    uint16_t _noiseThreshold;
    int      _minEchoGap;
    float    _speedOfSound;
    int      _envelopeSmoothing;
    uint16_t _slopeThreshold;     // スロープ閾値 (default 30)
    int      _slopeConfirmCount;  // 連続確認サンプル数 (default 3)

    // DMA設定
    uint32_t _dmaSampleRate;
    float    _dmaSamplePeriodUs;

    // バッファ
    uint16_t _waveform[HCSR04_MAX_SAMPLES];
    uint32_t _timestamps[HCSR04_MAX_SAMPLES];
    uint16_t _envelope[HCSR04_MAX_SAMPLES];
    uint32_t _numCaptured;
    uint32_t _txTimeUs;
    Echo     _echoes[HCSR04_MAX_ECHOES];
    int      _echoCount;

    // 中央値フィルタ
    int   _medianSize;
    float _medianBuf[HCSR04_MAX_MEDIAN];
    int   _medianIdx;
    int   _medianCount;
    int   _bestEchoIndex;
    float _rawDistance;
    float _filteredDistance;

    // 内部処理
    void  sendTrigger();
    void  captureWaveform();
    void  extractEnvelope();
    int   findEchoes();
    float updateMedian(float val);
};

#endif
