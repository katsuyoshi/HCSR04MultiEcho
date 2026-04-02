#include "HCSR04MultiEcho.h"

HCSR04MultiEcho::HCSR04MultiEcho(int trigPin, int analogPin)
    : _trigPin(trigPin)
    , _analogPin(analogPin)
    , _captureUs(30000)
    , _envelopeWindow(5)
    , _noiseThreshold(500)
    , _txFallbackUs(1500)
    , _minEchoGap(20)
    , _speedOfSound(0.0343f)
    , _envelopeSmoothing(5)
    , _useDMA(false)
    , _dmaSampleRate(0)
    , _dmaSamplePeriodUs(0)
    , _numCaptured(0)
    , _txPeakUs(0)
    , _txEndUs(0)
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

    // ESP32-S3: GPIO1-10 → ADC1_CH0-CH9
    int ch = _analogPin - 1;

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
void HCSR04MultiEcho::setEnvelopeWindow(int window)      { _envelopeWindow = window; }
void HCSR04MultiEcho::setNoiseThreshold(uint16_t val)    { _noiseThreshold = val; }
void HCSR04MultiEcho::setTxFallback(uint32_t us)         { _txFallbackUs = us; }
void HCSR04MultiEcho::setMinEchoGap(int gap)             { _minEchoGap = gap; }
void HCSR04MultiEcho::setSpeedOfSound(float v)           { _speedOfSound = v; }
void HCSR04MultiEcho::setTemperature(float celsius)      { _speedOfSound = (331.3f + 0.606f * celsius) / 10000.0f; }
void HCSR04MultiEcho::setEnvelopeSmoothing(int window)    { _envelopeSmoothing = window; }

void HCSR04MultiEcho::setMedianFilterSize(int size) {
    _medianSize = (size > HCSR04_MAX_MEDIAN) ? HCSR04_MAX_MEDIAN : size;
    resetFilter();
}

// 測定実行
int HCSR04MultiEcho::capture() {
    captureWaveform();
    extractEnvelope();

    // 1st pass: ブランキングなしで全ピーク検出
    _txPeakUs = 0;
    _txEndUs = 0;
    findEchoes();

    if (_echoCount > 0) {
        // 最初のピーク = TXバースト
        uint16_t txIdx = _echoes[0].sample_idx;
        _txPeakUs = _timestamps[txIdx];

        // TX終了: ピーク以降でエンベロープが _minEchoGap 連続で閾値以下
        _txEndUs = _timestamps[txIdx] + _txFallbackUs;  // フォールバック
        int lowCount = 0;
        for (uint32_t i = txIdx; i < _numCaptured; i++) {
            if (_envelope[i] < _noiseThreshold / 2) {
                lowCount++;
                if (lowCount >= _minEchoGap) {
                    _txEndUs = _timestamps[i - _minEchoGap + 1];
                    break;
                }
            } else {
                lowCount = 0;
            }
        }

        // 2nd pass: TX後のエコーのみ検出
        findEchoes();
    }

    // 最大振幅エコーの選択 + 中央値フィルタ
    if (_echoCount > 0) {
        _bestEchoIndex = 0;
        for (int i = 1; i < _echoCount; i++) {
            if (_echoes[i].amplitude > _echoes[_bestEchoIndex].amplitude)
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
        // エコーなし時はフィルタに値を入れない
    }

    return _echoCount;
}

// 結果取得
int HCSR04MultiEcho::getEchoCount() const { return _echoCount; }

const Echo& HCSR04MultiEcho::getEcho(int index) const {
    return _echoes[index];
}

uint32_t HCSR04MultiEcho::getTxPeakUs() const { return _txPeakUs; }
uint32_t HCSR04MultiEcho::getTxEndUs() const  { return _txEndUs; }

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

// ===== ソフトウェアエンベロープ抽出 =====
// スライディングウィンドウ内の max - min でキャリア振幅を推定
// _envelopeSmoothing > 0 の場合、移動平均でスムージング
void HCSR04MultiEcho::extractEnvelope() {
    int half = _envelopeWindow / 2;
    for (uint32_t i = 0; i < _numCaptured; i++) {
        uint16_t vmin = 4095, vmax = 0;
        int start = (int)i - half;
        int end   = (int)i + half;
        if (start < 0) start = 0;
        if (end >= (int)_numCaptured) end = _numCaptured - 1;
        for (int j = start; j <= end; j++) {
            if (_waveform[j] < vmin) vmin = _waveform[j];
            if (_waveform[j] > vmax) vmax = _waveform[j];
        }
        _envelope[i] = vmax - vmin;
    }

    // 移動平均スムージング
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

// ===== エンベロープからマルチエコー検出 =====
int HCSR04MultiEcho::findEchoes() {
    _echoCount = 0;
    bool in_echo = false;
    uint16_t peak = 0;
    int peak_idx = 0;
    int gap = _minEchoGap;

    for (uint32_t i = 0; i < _numCaptured && _echoCount < HCSR04_MAX_ECHOES; i++) {
        if (_timestamps[i] < _txEndUs) continue;

        uint16_t val = _envelope[i];

        if (!in_echo && val > _noiseThreshold && gap >= _minEchoGap) {
            in_echo = true;
            peak = val;
            peak_idx = i;
        } else if (in_echo) {
            if (val > peak) {
                peak = val;
                peak_idx = i;
            }
            if (val < _noiseThreshold / 2) {
                uint32_t dt = _timestamps[peak_idx] - _txPeakUs;
                _echoes[_echoCount].time_us     = dt;
                _echoes[_echoCount].distance_cm = dt * _speedOfSound / 2.0f;
                _echoes[_echoCount].amplitude   = peak;
                _echoes[_echoCount].sample_idx  = (uint16_t)peak_idx;
                _echoCount++;
                in_echo = false;
                gap = 0;
            }
        }
        if (!in_echo) gap++;
    }

    // バッファ末尾まで続くエコー
    if (in_echo && _echoCount < HCSR04_MAX_ECHOES) {
        uint32_t dt = _timestamps[peak_idx] - _txPeakUs;
        _echoes[_echoCount].time_us     = dt;
        _echoes[_echoCount].distance_cm = dt * _speedOfSound / 2.0f;
        _echoes[_echoCount].amplitude   = peak;
        _echoes[_echoCount].sample_idx  = (uint16_t)peak_idx;
        _echoCount++;
    }

    return _echoCount;
}
