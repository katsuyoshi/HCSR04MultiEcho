#ifndef HCSR04_MULTI_ECHO_H
#define HCSR04_MULTI_ECHO_H

#include <Arduino.h>
#include "driver/adc.h"
#include "soc/soc_caps.h"

#ifndef HCSR04_MAX_SAMPLES
#define HCSR04_MAX_SAMPLES 4000
#endif

#ifndef HCSR04_MAX_ECHOES
#define HCSR04_MAX_ECHOES 10
#endif

struct Echo {
    float    distance_cm;
    uint16_t amplitude;
    uint16_t sample_idx;
    uint32_t time_us;      // TXピークからの相対時間
};

class HCSR04MultiEcho {
public:
    HCSR04MultiEcho(int trigPin, int analogPin);

    // 初期化 (pinMode, ADC設定)
    void begin(int resolution = 12);
    void beginDMA(uint32_t sampleRate = 83333);

    // パラメータ設定
    void setCaptureTime(uint32_t us);
    void setEnvelopeWindow(int window);
    void setNoiseThreshold(uint16_t val);
    void setTxDetectThreshold(uint16_t val);
    void setTxSearchLimit(uint32_t us);
    void setTxFallback(uint32_t us);
    void setMinEchoGap(int gap);
    void setSpeedOfSound(float v);
    void setEnvelopeSmoothing(int window);  // エンベロープ移動平均窓 (0=無効)

    // 測定実行 (トリガー → キャプチャ → エンベロープ → エコー検出)
    // returns echo_count
    int capture();

    // 結果取得
    int           getEchoCount() const;
    const Echo&   getEcho(int index) const;
    uint32_t      getTxPeakUs() const;   // TXバーストピーク時刻 (距離計算基準)
    uint32_t      getTxEndUs() const;    // TX残響終了時刻 (ブランキング用)

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
    int      _envelopeWindow;
    uint16_t _noiseThreshold;
    uint16_t _txDetectThreshold;
    uint32_t _txSearchLimitUs;
    uint32_t _txFallbackUs;
    int      _minEchoGap;
    float    _speedOfSound;
    int      _envelopeSmoothing;  // エンベロープ移動平均窓 (0=無効)

    // DMA設定
    bool     _useDMA;
    uint32_t _dmaSampleRate;
    float    _dmaSamplePeriodUs;

    // バッファ
    uint16_t _waveform[HCSR04_MAX_SAMPLES];
    uint32_t _timestamps[HCSR04_MAX_SAMPLES];
    uint16_t _envelope[HCSR04_MAX_SAMPLES];
    uint32_t _numCaptured;
    uint32_t _txPeakUs;    // TXバーストピーク時刻 (距離計算基準)
    uint32_t _txEndUs;     // TX残響終了時刻 (ブランキング用)
    Echo     _echoes[HCSR04_MAX_ECHOES];
    int      _echoCount;

    // 内部処理
    void sendTrigger();
    void captureWaveform();
    void captureWaveformDMA();
    void extractEnvelope();
    void findTxEnd();
    int  findEchoes();
};

#endif
