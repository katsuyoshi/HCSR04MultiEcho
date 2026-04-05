#include "HCSR04MultiEcho.h"

HCSR04MultiEcho::HCSR04MultiEcho(int trigPin, int analogPin)
    : _trigPin(trigPin)
    , _analogPin(analogPin)
    , _captureUs(30000)
    , _noiseThreshold(500)
    , _minEchoGap(20)
    , _speedOfSound(0.0343f)
    , _envelopeSmoothing(5)
    , _slopeThreshold(30)
    , _slopeConfirmCount(3)
    , _useDMA(false)
    , _dmaSampleRate(0)
    , _dmaSamplePeriodUs(0)
    , _numCaptured(0)
    , _txTimeUs(0)
    , _echoCount(0)
    , _medianSize(13)
    , _medianIdx(0)
    , _medianCount(0)
    , _bestEchoIndex(-1)
    , _rawDistance(NAN)
    , _filteredDistance(NAN)
{
}

void HCSR04MultiEcho::begin(int resolution) {
    pinMode(_trigPin, OUTPUT);
    digitalWrite(_trigPin, LOW);
    analogReadResolution(resolution);
    analogSetAttenuation(ADC_11db);
}

void HCSR04MultiEcho::beginDMA(uint32_t sampleRate) {
    pinMode(_trigPin, OUTPUT);
    digitalWrite(_trigPin, LOW);

    _useDMA = true;
    _dmaSampleRate = sampleRate;
    _dmaSamplePeriodUs = 1000000.0f / sampleRate;

    // ESP32-C3: GPIO0-4 → ADC1_CH0-CH4
    int ch = _analogPin;

    adc_digi_init_config_t dma_cfg = {};
    dma_cfg.max_store_buf_size = 16384;
    dma_cfg.conv_num_each_intr = 256;
    dma_cfg.adc1_chan_mask = (1 << ch);
    dma_cfg.adc2_chan_mask = 0;
    ESP_ERROR_CHECK(adc_digi_initialize(&dma_cfg));

    adc_digi_configuration_t dig_cfg = {};
    dig_cfg.conv_limit_en = false;
    dig_cfg.conv_limit_num = 255;
    dig_cfg.sample_freq_hz = sampleRate;
    dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;

    adc_digi_pattern_config_t pattern[1] = {};
    pattern[0].atten = ADC_ATTEN_DB_12;
    pattern[0].channel = ch;
    pattern[0].unit = 0;
    pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    dig_cfg.adc_pattern = pattern;
    dig_cfg.pattern_num = 1;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
}

// パラメータ設定
void HCSR04MultiEcho::setCaptureTime(uint32_t us)       { _captureUs = us; }
void HCSR04MultiEcho::setNoiseThreshold(uint16_t val)    { _noiseThreshold = val; }
void HCSR04MultiEcho::setMinEchoGap(int gap)             { _minEchoGap = gap; }
void HCSR04MultiEcho::setSpeedOfSound(float v)           { _speedOfSound = v; }
void HCSR04MultiEcho::setTemperature(float celsius)      { _speedOfSound = (331.3f + 0.606f * celsius) / 10000.0f; }
void HCSR04MultiEcho::setEnvelopeSmoothing(int window)    { _envelopeSmoothing = window; }
void HCSR04MultiEcho::setSlopeThreshold(uint16_t val)     { _slopeThreshold = val; }
void HCSR04MultiEcho::setSlopeConfirmCount(int count)     { _slopeConfirmCount = count; }

void HCSR04MultiEcho::setMedianFilterSize(int size) {
    _medianSize = (size > HCSR04_MAX_MEDIAN) ? HCSR04_MAX_MEDIAN : size;
    resetFilter();
}

// 測定実行
int HCSR04MultiEcho::capture() {
    captureWaveform();
    extractEnvelope();

    // TX バースト検出 (DMAウォームアップ後、低閾値スロープ3連続で検出)
    _txTimeUs = 0;
    {
        const uint32_t txSearchSkip  = 500;   // DMAウォームアップスキップ
        const uint32_t txSearchLimit = 2000;
        const int16_t  txSlopeTh = 3;         // ノイズ ±1-2、TX 3+
        const int      txSustained = 3;       // 3連続で確定
        int sustained = 0;
        for (uint32_t i = 1; i + 1 < _numCaptured; i++) {
            if (_timestamps[i] < txSearchSkip) continue;
            if (_timestamps[i] > txSearchLimit) break;
            int16_t slope = ((int16_t)_envelope[i + 1] - (int16_t)_envelope[i - 1]) / 2;
            if (slope >= txSlopeTh) {
                if (++sustained >= txSustained) {
                    _txTimeUs = _timestamps[i - sustained + 1];
                    break;
                }
            } else {
                sustained = 0;
            }
        }
    }

    findEchoes();

    // TX 領域のエコーを除外 + TX 基準で距離再計算
    if (_txTimeUs > 0) {
        uint32_t txMargin = _txTimeUs + 1500;  // TX開始 + 1.5ms
        int writeIdx = 0;
        for (int i = 0; i < _echoCount; i++) {
            if (_echoes[i].time_us > txMargin) {
                uint32_t dt = _echoes[i].time_us - _txTimeUs;
                _echoes[writeIdx] = _echoes[i];
                _echoes[writeIdx].time_us = dt;
                _echoes[writeIdx].distance_cm = dt * _speedOfSound / 2.0f;
                writeIdx++;
            }
        }
        _echoCount = writeIdx;
    }

    // 最大スロープエコーの選択 + 中央値フィルタ
    if (_echoCount > 0) {
        _bestEchoIndex = 0;
        for (int i = 1; i < _echoCount; i++) {
            if (_echoes[i].max_slope > _echoes[_bestEchoIndex].max_slope)
                _bestEchoIndex = i;
        }
        _rawDistance = _echoes[_bestEchoIndex].distance_cm;
        if (_medianSize > 0) {
            _filteredDistance = updateMedian(_rawDistance);
        } else {
            _filteredDistance = _rawDistance;
        }
    } else {
        _bestEchoIndex = -1;
        _rawDistance = NAN;
    }

    return _echoCount;
}

// 結果取得
int HCSR04MultiEcho::getEchoCount() const { return _echoCount; }

const Echo& HCSR04MultiEcho::getEcho(int index) const {
    return _echoes[index];
}

uint32_t HCSR04MultiEcho::getTxTimeUs() const { return _txTimeUs; }

// 距離取得
float HCSR04MultiEcho::getDistance() const        { return _filteredDistance; }
float HCSR04MultiEcho::getRawDistance() const      { return _rawDistance; }
int   HCSR04MultiEcho::getBestEchoIndex() const    { return _bestEchoIndex; }
bool  HCSR04MultiEcho::isFilterReady() const       { return _medianCount >= _medianSize; }

void HCSR04MultiEcho::resetFilter() {
    _medianIdx = 0;
    _medianCount = 0;
    _filteredDistance = NAN;
}

// 生データアクセス
uint32_t        HCSR04MultiEcho::getSampleCount() const { return _numCaptured; }
const uint16_t* HCSR04MultiEcho::getWaveform() const    { return _waveform; }
const uint16_t* HCSR04MultiEcho::getEnvelope() const    { return _envelope; }
const uint32_t* HCSR04MultiEcho::getTimestamps() const  { return _timestamps; }

// ===== HC-SR04 トリガーパルス送信 (10μs) =====
void HCSR04MultiEcho::sendTrigger() {
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
}

// ===== 波形キャプチャ (analogRead or DMA) =====
void HCSR04MultiEcho::captureWaveform() {
    if (_useDMA) {
        captureWaveformDMA();
        return;
    }

    sendTrigger();

    uint32_t t0 = micros();
    uint32_t i = 0;

    while (i < HCSR04_MAX_SAMPLES) {
        uint32_t now = micros();
        if ((now - t0) >= _captureUs) break;
        _waveform[i] = analogRead(_analogPin);
        _timestamps[i] = now - t0;
        i++;
    }
    _numCaptured = i;
}

// ===== DMA波形キャプチャ =====
void HCSR04MultiEcho::captureWaveformDMA() {
    _numCaptured = 0;

    esp_err_t ret = adc_digi_start();
    if (ret != ESP_OK) return;

    // DMAウォームアップ: 起動直後の不正データを読み捨て
    uint8_t warmup[64];
    uint32_t warmupLen = 0;
    adc_digi_read_bytes(warmup, sizeof(warmup), &warmupLen, 10);

    sendTrigger();

    uint32_t t0 = micros();
    uint8_t buf[256];

    while ((micros() - t0) < _captureUs && _numCaptured < HCSR04_MAX_SAMPLES) {
        uint32_t readLen = 0;
        ret = adc_digi_read_bytes(buf, sizeof(buf), &readLen, 10);
        if (ret == ESP_OK) {
            for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= readLen;
                 i += SOC_ADC_DIGI_RESULT_BYTES) {
                if (_numCaptured >= HCSR04_MAX_SAMPLES) break;
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&buf[i];
                _waveform[_numCaptured] = p->type2.data;
                _timestamps[_numCaptured] = (uint32_t)(_numCaptured * _dmaSamplePeriodUs);
                _numCaptured++;
            }
        }
    }

    adc_digi_stop();
}

// ===== ソフトウェアエンベロープ抽出 (全波整流方式) =====
// 1. DC オフセット推定 (全サンプル平均)
// 2. DC 除去 + 全波整流: |raw[i] - dc_offset|
// 3. LPF (移動平均) でエンベロープ平滑化
void HCSR04MultiEcho::extractEnvelope() {
    if (_numCaptured == 0) return;

    // 1. DC オフセット推定
    uint32_t dcSum = 0;
    for (uint32_t i = 0; i < _numCaptured; i++) {
        dcSum += _waveform[i];
    }
    int16_t dcOffset = (int16_t)(dcSum / _numCaptured);

    // 2. DC 除去 + 全波整流
    for (uint32_t i = 0; i < _numCaptured; i++) {
        int16_t val = (int16_t)_waveform[i] - dcOffset;
        _envelope[i] = (uint16_t)(val < 0 ? -val : val);
    }

    // 3. LPF (移動平均スムージング)
    if (_envelopeSmoothing > 1) {
        int smHalf = _envelopeSmoothing / 2;
        uint16_t* temp = new uint16_t[_numCaptured];
        for (uint32_t i = 0; i < _numCaptured; i++) {
            uint32_t sum = 0;
            int cnt = 0;
            int s = (int)i - smHalf;
            int e = (int)i + smHalf;
            if (s < 0) s = 0;
            if (e >= (int)_numCaptured) e = _numCaptured - 1;
            for (int j = s; j <= e; j++) {
                sum += _envelope[j];
                cnt++;
            }
            temp[i] = (uint16_t)(sum / cnt);
        }
        memcpy(_envelope, temp, _numCaptured * sizeof(uint16_t));
        delete[] temp;
    }
}

// ===== 中央値フィルタ =====
float HCSR04MultiEcho::updateMedian(float val) {
    _medianBuf[_medianIdx] = val;
    _medianIdx = (_medianIdx + 1) % _medianSize;
    if (_medianCount < _medianSize) _medianCount++;

    float sorted[HCSR04_MAX_MEDIAN];
    for (int i = 0; i < _medianCount; i++) sorted[i] = _medianBuf[i];
    for (int i = 0; i < _medianCount - 1; i++) {
        for (int j = i + 1; j < _medianCount; j++) {
            if (sorted[j] < sorted[i]) {
                float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t;
            }
        }
    }
    return sorted[_medianCount / 2];
}

// ===== エンベロープからマルチエコー検出 (スロープ検出方式) =====
// スロープ(傾き)でエコー先頭を検出、ノイズフロア到達でエコー終了
// ダブルローブ(同一エコー内の2つのピーク)は1エコーとして扱う
int HCSR04MultiEcho::findEchoes() {
    _echoCount = 0;
    if (_numCaptured < 3) return 0;

    const uint16_t noiseFloor = _noiseThreshold / 4;
    const uint16_t minAmplitude = _noiseThreshold;
    const int16_t slopeTh = (int16_t)_slopeThreshold;
    const int slopeConfirm = _slopeConfirmCount;
    const int belowRequired = 5;  // ノイズフロア連続サンプル数

    bool in_echo = false;
    uint16_t peak = 0;
    int16_t  max_slope = 0;  // 立ち上がり最大スロープ
    int rise_idx = 0;        // スロープ上昇開始インデックス
    int sustained = 0;       // スロープ閾値超え連続カウント
    int gap = _minEchoGap;   // 前エコー終了からのギャップ
    int below_count = 0;     // ノイズフロア以下の連続カウント

    for (uint32_t i = 1; i + 1 < _numCaptured && _echoCount < HCSR04_MAX_ECHOES; i++) {
        uint16_t val = _envelope[i];

        // 3点中心差分でスロープ計算 (追加バッファなし)
        int16_t slope = ((int16_t)_envelope[i + 1] - (int16_t)_envelope[i - 1]) / 2;

        if (!in_echo) {
            // --- IDLE: スロープ上昇でエコー検出 ---
            gap++;
            if (slope >= slopeTh && gap >= _minEchoGap) {
                sustained++;
                if (sustained == 1) {
                    rise_idx = i;  // 上昇開始点を記録
                    max_slope = slope;
                }
                if (slope > max_slope) max_slope = slope;
                if (sustained >= slopeConfirm) {
                    // エコー確定
                    in_echo = true;
                    peak = val;
                    below_count = 0;
                }
            } else {
                sustained = 0;
            }
        } else {
            // --- IN_ECHO: ピーク追跡 + ノイズフロアでエコー終了 ---
            if (val > peak) {
                peak = val;
            }
            if (slope > max_slope) max_slope = slope;

            if (val < noiseFloor) {
                below_count++;
                if (below_count >= belowRequired) {
                    // エコー終了 → 記録
                    if (peak >= minAmplitude) {
                        uint32_t t = _timestamps[rise_idx];
                        _echoes[_echoCount].time_us     = t;
                        _echoes[_echoCount].distance_cm = t * _speedOfSound / 2.0f;
                        _echoes[_echoCount].amplitude   = peak;
                        _echoes[_echoCount].max_slope   = max_slope;
                        _echoes[_echoCount].sample_idx  = (uint16_t)rise_idx;
                        _echoCount++;
                    }
                    in_echo = false;
                    peak = 0;
                    max_slope = 0;
                    gap = 0;
                    sustained = 0;
                    below_count = 0;
                }
            } else {
                below_count = 0;
            }
        }
    }

    // バッファ末尾まで続くエコー
    if (in_echo && _echoCount < HCSR04_MAX_ECHOES && peak >= minAmplitude) {
        uint32_t t = _timestamps[rise_idx];
        _echoes[_echoCount].time_us     = t;
        _echoes[_echoCount].distance_cm = t * _speedOfSound / 2.0f;
        _echoes[_echoCount].amplitude   = peak;
        _echoes[_echoCount].max_slope   = max_slope;
        _echoes[_echoCount].sample_idx  = (uint16_t)rise_idx;
        _echoCount++;
    }

    return _echoCount;
}
