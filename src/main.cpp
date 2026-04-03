#include <Arduino.h>
#include <M5Unified.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <vl53l4cd_class.h>

#include <array>
#include <cmath>

#include "config.h"

namespace {

namespace settings {

// ToF センサの I2C アドレス
constexpr uint8_t kTofAddress = 0x29;
// M5StickS3 Port.A の I2C ピン
constexpr int kPortASdaPin = 9;
constexpr int kPortASclPin = 10;
// シリアルステータス出力周期 [ms]
constexpr uint32_t kStatusIntervalMs = 500;
// 画面更新の最短周期 [ms]
constexpr uint32_t kDisplayIntervalMs = 100;
// VL53L0X の timing budget [us]
constexpr uint32_t kVl53TimingBudgetUs = 20000;
// VL6180X / VL53L4CD の ready 待ち timeout [ms]
constexpr uint32_t kVl6180TimeoutMs = 80;
// sampleHz 計算窓 [ms]
constexpr uint32_t kSampleRateIntervalMs = 1000;
// グラフ履歴長
constexpr size_t kHistorySize = 120;
// 何サンプルごとに画面更新するか
constexpr uint32_t kDisplayRefreshBatchCount = 10;

}  // namespace settings

constexpr uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

constexpr uint16_t kColorBg = color565(0, 0, 0);
constexpr uint16_t kColorText = color565(240, 240, 240);
constexpr uint16_t kColorGrid = color565(48, 48, 48);
constexpr uint16_t kColorDistance = color565(0, 220, 120);
constexpr uint16_t kColorTimeout = color565(220, 60, 60);
constexpr uint16_t kColorFreq = color565(100, 180, 255);

enum class SensorType : uint8_t {
  None,
  Vl53L0X,
  Vl6180X,
  Vl53L4CD,
};

enum class AudioOutputMode : uint8_t {
  InternalSpeaker,
  ExternalI2S,
};

struct Sample {
  uint32_t timestampMs = 0;
  uint16_t distanceMm = 0;
  bool valid = false;
  bool timeout = false;
};

VL53L0X gVl53;
TwoWire gTofWire = TwoWire(0);
VL53L4CD gVl53L4cd(&gTofWire, -1);
M5Canvas gCanvas(&M5.Display);

SensorType gSensorType = SensorType::None;
AudioOutputMode gAudioMode = AudioOutputMode::InternalSpeaker;
Sample gLatestSample;
m5::speaker_config_t gInternalSpeakerConfig;
m5::speaker_config_t gExternalSpeakerConfig;

uint32_t gLastStatusMs = 0;
uint32_t gLastDisplayMs = 0;
uint32_t gSampleRateWindowStartMs = 0;
uint32_t gSampleCountInWindow = 0;
float gSampleRateHz = 0.0f;
float gCurrentFrequencyHz = 0.0f;
float gBaselineDistanceMm = 0.0f;
float gWaveHeightMm = 0.0f;
float gEstimatedWaveFrequencyHz = 0.0f;
uint32_t gDisplayDirtySamples = 0;
bool gCanvasReady = false;
bool gMuted = false;
bool gSpeakerReady = false;

std::array<Sample, settings::kHistorySize> gHistory = {};
std::array<float, settings::kHistorySize> gWaveSignalHistory = {};
std::array<std::array<int8_t, config::audio::kAudioChunkSamples>, config::audio::kAudioBufferCount> gAudioBuffers = {};
size_t gHistoryHead = 0;
size_t gHistoryCount = 0;
size_t gAudioBufferIndex = 0;
float gSmoothedWaveAmplitudeMm = 0.0f;
float gSynthPhase = 0.0f;
float gSynthCurrentFrequencyHz = 0.0f;
float gSynthTargetFrequencyHz = 0.0f;
float gSynthCurrentAmplitude = 0.0f;
float gSynthTargetAmplitude = 0.0f;

const char* sensorName() {
  switch (gSensorType) {
    case SensorType::Vl53L0X:
      return "VL53L0X";
    case SensorType::Vl6180X:
      return "VL6180X";
    case SensorType::Vl53L4CD:
      return "VL53L4CD";
    default:
      return "NONE";
  }
}

const char* audioModeName() {
  switch (gAudioMode) {
    case AudioOutputMode::InternalSpeaker:
      return "INTERNAL";
    case AudioOutputMode::ExternalI2S:
      return "EXTERNAL";
    default:
      return "UNKNOWN";
  }
}

bool probeAddress(uint8_t address) {
  gTofWire.beginTransmission(address);
  return gTofWire.endTransmission() == 0;
}

bool writeVl6180Reg8(uint16_t reg, uint8_t value) {
  gTofWire.beginTransmission(settings::kTofAddress);
  gTofWire.write(static_cast<uint8_t>(reg >> 8));
  gTofWire.write(static_cast<uint8_t>(reg & 0xFF));
  gTofWire.write(value);
  return gTofWire.endTransmission() == 0;
}

bool readVl6180Reg8(uint16_t reg, uint8_t& value) {
  gTofWire.beginTransmission(settings::kTofAddress);
  gTofWire.write(static_cast<uint8_t>(reg >> 8));
  gTofWire.write(static_cast<uint8_t>(reg & 0xFF));
  if (gTofWire.endTransmission(false) != 0) {
    return false;
  }
  if (gTofWire.requestFrom(static_cast<int>(settings::kTofAddress), 1) != 1) {
    return false;
  }
  value = gTofWire.read();
  return true;
}

void loadVl6180Tuning() {
  writeVl6180Reg8(0x0207, 0x01);
  writeVl6180Reg8(0x0208, 0x01);
  writeVl6180Reg8(0x0096, 0x00);
  writeVl6180Reg8(0x0097, 0xFD);
  writeVl6180Reg8(0x00E3, 0x01);
  writeVl6180Reg8(0x00E4, 0x03);
  writeVl6180Reg8(0x00E5, 0x02);
  writeVl6180Reg8(0x00E6, 0x01);
  writeVl6180Reg8(0x00E7, 0x03);
  writeVl6180Reg8(0x00F5, 0x02);
  writeVl6180Reg8(0x00D9, 0x05);
  writeVl6180Reg8(0x00DB, 0xCE);
  writeVl6180Reg8(0x00DC, 0x03);
  writeVl6180Reg8(0x00DD, 0xF8);
  writeVl6180Reg8(0x009F, 0x00);
  writeVl6180Reg8(0x00A3, 0x3C);
  writeVl6180Reg8(0x00B7, 0x00);
  writeVl6180Reg8(0x00BB, 0x3C);
  writeVl6180Reg8(0x00B2, 0x09);
  writeVl6180Reg8(0x00CA, 0x09);
  writeVl6180Reg8(0x0198, 0x01);
  writeVl6180Reg8(0x01B0, 0x17);
  writeVl6180Reg8(0x01AD, 0x00);
  writeVl6180Reg8(0x00FF, 0x05);
  writeVl6180Reg8(0x0100, 0x05);
  writeVl6180Reg8(0x0199, 0x05);
  writeVl6180Reg8(0x01A6, 0x1B);
  writeVl6180Reg8(0x01AC, 0x3E);
  writeVl6180Reg8(0x01A7, 0x1F);
  writeVl6180Reg8(0x0030, 0x00);
  writeVl6180Reg8(0x0011, 0x10);
  writeVl6180Reg8(0x010A, 0x30);
  writeVl6180Reg8(0x003F, 0x46);
  writeVl6180Reg8(0x0031, 0xFF);
  writeVl6180Reg8(0x0041, 0x63);
  writeVl6180Reg8(0x002E, 0x01);
  writeVl6180Reg8(0x001B, 0x09);
  writeVl6180Reg8(0x003E, 0x31);
  writeVl6180Reg8(0x0014, 0x24);
}

bool initVl6180() {
  uint8_t modelId = 0;
  if (!readVl6180Reg8(0x0000, modelId) || modelId != 0xB4) {
    return false;
  }
  uint8_t freshOutOfReset = 0;
  if (!readVl6180Reg8(0x0016, freshOutOfReset)) {
    return false;
  }
  if (freshOutOfReset == 1) {
    loadVl6180Tuning();
    if (!writeVl6180Reg8(0x0016, 0x00)) {
      return false;
    }
  }
  return true;
}

bool initVl53() {
  gVl53.setBus(&gTofWire);
  gVl53.setTimeout(settings::kVl6180TimeoutMs);
  if (!gVl53.init()) {
    return false;
  }
  gVl53.setMeasurementTimingBudget(settings::kVl53TimingBudgetUs);
  gVl53.startContinuous();
  return true;
}

bool initVl53L4Cd() {
  gVl53L4cd.begin();
  if (gVl53L4cd.InitSensor() != VL53L4CD_ERROR_NONE) {
    return false;
  }
  if (gVl53L4cd.VL53L4CD_SetRangeTiming(10, 0) != VL53L4CD_ERROR_NONE) {
    return false;
  }
  if (gVl53L4cd.VL53L4CD_StartRanging() != VL53L4CD_ERROR_NONE) {
    return false;
  }
  return true;
}

bool detectSensor() {
  gSensorType = SensorType::None;
  gTofWire.end();
  gTofWire.begin(settings::kPortASdaPin, settings::kPortASclPin, 400000U);
  gTofWire.setTimeOut(20);

  if (!probeAddress(settings::kTofAddress)) {
    return false;
  }
  if (initVl6180()) {
    gSensorType = SensorType::Vl6180X;
    return true;
  }
  if (initVl53L4Cd()) {
    gSensorType = SensorType::Vl53L4CD;
    return true;
  }
  if (initVl53()) {
    gSensorType = SensorType::Vl53L0X;
    return true;
  }
  return false;
}

Sample readVl53Sample() {
  Sample sample;
  sample.timestampMs = millis();
  sample.distanceMm = gVl53.readRangeContinuousMillimeters();
  sample.timeout = gVl53.timeoutOccurred();
  sample.valid = !sample.timeout;
  return sample;
}

Sample readVl6180Sample() {
  Sample sample;
  sample.timestampMs = millis();

  if (!writeVl6180Reg8(0x0018, 0x01)) {
    sample.timeout = true;
    return sample;
  }

  uint8_t status = 0;
  const uint32_t startMs = millis();
  while (millis() - startMs < settings::kVl6180TimeoutMs) {
    if (!readVl6180Reg8(0x004F, status)) {
      sample.timeout = true;
      return sample;
    }
    if ((status & 0x07U) == 0x04U) {
      break;
    }
    delay(1);
  }

  if ((status & 0x07U) != 0x04U) {
    sample.timeout = true;
    return sample;
  }

  uint8_t distance = 0;
  uint8_t rangeStatus = 0;
  if (!readVl6180Reg8(0x0062, distance) || !readVl6180Reg8(0x004D, rangeStatus)) {
    sample.timeout = true;
    return sample;
  }

  writeVl6180Reg8(0x0015, 0x07);
  const uint8_t errorCode = (rangeStatus >> 4) & 0x0F;
  sample.distanceMm = distance;
  sample.valid = (errorCode == 0) && (distance != 255);
  sample.timeout = false;
  return sample;
}

Sample readVl53L4CdSample() {
  Sample sample;
  sample.timestampMs = millis();

  uint8_t ready = 0;
  const uint32_t startMs = millis();
  while (millis() - startMs < settings::kVl6180TimeoutMs) {
    if (gVl53L4cd.VL53L4CD_CheckForDataReady(&ready) != VL53L4CD_ERROR_NONE) {
      sample.timeout = true;
      return sample;
    }
    if (ready) {
      break;
    }
    delay(1);
  }

  if (!ready) {
    sample.timeout = true;
    return sample;
  }

  VL53L4CD_Result_t result = {};
  if (gVl53L4cd.VL53L4CD_GetResult(&result) != VL53L4CD_ERROR_NONE) {
    sample.timeout = true;
    return sample;
  }
  gVl53L4cd.VL53L4CD_ClearInterrupt();

  sample.distanceMm = result.distance_mm;
  sample.valid = (result.range_status == 0);
  sample.timeout = false;
  return sample;
}

Sample acquireSample() {
  switch (gSensorType) {
    case SensorType::Vl53L0X:
      return readVl53Sample();
    case SensorType::Vl6180X:
      return readVl6180Sample();
    case SensorType::Vl53L4CD:
      return readVl53L4CdSample();
    default:
      return {};
  }
}

void appendHistory(const Sample& sample) {
  const size_t index = (gHistoryHead + gHistoryCount) % settings::kHistorySize;
  gHistory[index] = sample;
  if (gHistoryCount < settings::kHistorySize) {
    ++gHistoryCount;
  } else {
    gHistoryHead = (gHistoryHead + 1) % settings::kHistorySize;
  }
}

void updateSampleRate() {
  const uint32_t now = millis();
  if (gSampleRateWindowStartMs == 0) {
    gSampleRateWindowStartMs = now;
  }
  ++gSampleCountInWindow;
  const uint32_t elapsedMs = now - gSampleRateWindowStartMs;
  if (elapsedMs >= settings::kSampleRateIntervalMs) {
    gSampleRateHz = (1000.0f * static_cast<float>(gSampleCountInWindow)) /
                    static_cast<float>(elapsedMs);
    gSampleCountInWindow = 0;
    gSampleRateWindowStartMs = now;
  }
}

float samplePeriodMs() {
  if (gSampleRateHz <= 0.0f) {
    return 0.0f;
  }
  return 1000.0f / gSampleRateHz;
}

float waveFrequencyToToneFrequency(float waveFrequencyHz) {
  const float scaled = waveFrequencyHz * config::audio::kWaveToToneFrequencyScale;
  return std::max(config::audio::kMinToneHz,
                  std::min(config::audio::kMaxToneHz, scaled));
}

uint8_t waveHeightToVolume(float waveHeightMm) {
  if (waveHeightMm < config::audio::kWaveHeightNoiseFloorMm) {
    return 0;
  }
  const float clamped = std::max(0.0f, std::min(config::audio::kWaveHeightMaxMm, waveHeightMm));
  const float norm = clamped / config::audio::kWaveHeightMaxMm;
  return static_cast<uint8_t>(norm * 255.0f + 0.5f);
}

void updateWaveState() {
  if (!gLatestSample.valid || gLatestSample.timeout) {
    gWaveHeightMm = 0.0f;
    gSmoothedWaveAmplitudeMm +=
        (0.0f - gSmoothedWaveAmplitudeMm) * config::audio::kWaveAmplitudeSmoothingAlpha;
    gEstimatedWaveFrequencyHz = 0.0f;
    return;
  }

  const float distanceMm = static_cast<float>(gLatestSample.distanceMm);
  if (gBaselineDistanceMm <= 0.0f) {
    gBaselineDistanceMm = distanceMm;
  } else {
    gBaselineDistanceMm += (distanceMm - gBaselineDistanceMm) * config::audio::kBaselineAlpha;
  }

  const float waveSignalMm = gBaselineDistanceMm - distanceMm;
  if (gHistoryCount > 0) {
    const size_t latestIndex = (gHistoryHead + gHistoryCount - 1) % settings::kHistorySize;
    gWaveSignalHistory[latestIndex] = waveSignalMm;
  }

  if (gSampleRateHz < 1.0f || gHistoryCount < config::audio::kFftWindowSize) {
    gWaveHeightMm = 0.0f;
    gSmoothedWaveAmplitudeMm +=
        (0.0f - gSmoothedWaveAmplitudeMm) * config::audio::kWaveAmplitudeSmoothingAlpha;
    gEstimatedWaveFrequencyHz = 0.0f;
    return;
  }

  std::array<float, config::audio::kFftWindowSize> signal = {};
  float mean = 0.0f;
  for (size_t i = 0; i < config::audio::kFftWindowSize; ++i) {
    const size_t index =
        (gHistoryHead + gHistoryCount - config::audio::kFftWindowSize + i) % settings::kHistorySize;
    signal[i] = gWaveSignalHistory[index];
    mean += signal[i];
  }
  mean /= static_cast<float>(config::audio::kFftWindowSize);

  float windowSum = 0.0f;
  for (size_t i = 0; i < config::audio::kFftWindowSize; ++i) {
    signal[i] -= mean;
    const float window =
        0.5f - 0.5f * cosf((2.0f * PI * static_cast<float>(i)) /
                           static_cast<float>(config::audio::kFftWindowSize - 1));
    signal[i] *= window;
    windowSum += window;
  }

  const float binHz = gSampleRateHz / static_cast<float>(config::audio::kFftWindowSize);
  const int minBin = std::max<int>(1, static_cast<int>(ceilf(config::audio::kMinWaveFrequencyHz / binHz)));
  const int maxBin = std::min<int>(static_cast<int>(config::audio::kFftWindowSize / 2),
                                   static_cast<int>(floorf(config::audio::kMaxWaveFrequencyHz / binHz)));
  if (minBin > maxBin) {
    gWaveHeightMm = 0.0f;
    gSmoothedWaveAmplitudeMm +=
        (0.0f - gSmoothedWaveAmplitudeMm) * config::audio::kWaveAmplitudeSmoothingAlpha;
    gEstimatedWaveFrequencyHz = 0.0f;
    return;
  }

  float bestMagnitude = 0.0f;
  float totalMagnitude = 0.0f;
  int magnitudeCount = 0;
  int bestBin = 0;
  for (int bin = minBin; bin <= maxBin; ++bin) {
    float real = 0.0f;
    float imag = 0.0f;
    for (size_t n = 0; n < config::audio::kFftWindowSize; ++n) {
      const float phase =
          (2.0f * PI * static_cast<float>(bin) * static_cast<float>(n)) /
          static_cast<float>(config::audio::kFftWindowSize);
      real += signal[n] * cosf(phase);
      imag -= signal[n] * sinf(phase);
    }
    const float magnitude = sqrtf(real * real + imag * imag);
    totalMagnitude += magnitude;
    ++magnitudeCount;
    if (magnitude > bestMagnitude) {
      bestMagnitude = magnitude;
      bestBin = bin;
    }
  }

  if (bestBin <= 0 || magnitudeCount == 0) {
    gWaveHeightMm = 0.0f;
    gSmoothedWaveAmplitudeMm +=
        (0.0f - gSmoothedWaveAmplitudeMm) * config::audio::kWaveAmplitudeSmoothingAlpha;
    gEstimatedWaveFrequencyHz = 0.0f;
    return;
  }

  const float meanMagnitude = totalMagnitude / static_cast<float>(magnitudeCount);
  if (meanMagnitude <= 0.0f ||
      bestMagnitude < meanMagnitude * config::audio::kWavePeakRatioGate) {
    gWaveHeightMm = 0.0f;
    gSmoothedWaveAmplitudeMm +=
        (0.0f - gSmoothedWaveAmplitudeMm) * config::audio::kWaveAmplitudeSmoothingAlpha;
    gEstimatedWaveFrequencyHz = 0.0f;
    return;
  }

  const float detectedWaveFrequencyHz = static_cast<float>(bestBin) * binHz;
  const float detectedWaveAmplitudeMm = (2.0f * bestMagnitude) / std::max(1.0f, windowSum);
  gSmoothedWaveAmplitudeMm +=
      (detectedWaveAmplitudeMm - gSmoothedWaveAmplitudeMm) * config::audio::kWaveAmplitudeSmoothingAlpha;
  gWaveHeightMm = gSmoothedWaveAmplitudeMm;

  if (gWaveHeightMm < config::audio::kWaveHeightNoiseFloorMm) {
    gEstimatedWaveFrequencyHz = 0.0f;
    return;
  }

  if (gEstimatedWaveFrequencyHz <= 0.0f) {
    gEstimatedWaveFrequencyHz = detectedWaveFrequencyHz;
  } else {
    gEstimatedWaveFrequencyHz +=
        (detectedWaveFrequencyHz - gEstimatedWaveFrequencyHz) *
        config::audio::kWaveFrequencySmoothingAlpha;
  }
}

bool applySpeakerConfig(const m5::speaker_config_t& cfg) {
  M5.Speaker.stop();
  M5.Speaker.end();
  M5.Speaker.config(cfg);
  if (!M5.Speaker.begin()) {
    return false;
  }
  M5.Speaker.setVolume(config::audio::kMasterVolume);
  M5.Speaker.setChannelVolume(config::audio::kToneChannel, 255);
  return true;
}

void updateTone() {
  const bool shouldPlay = gSpeakerReady && !gMuted && gLatestSample.valid && !gLatestSample.timeout;
  if (!shouldPlay) {
    gCurrentFrequencyHz = 0.0f;
    gSynthTargetFrequencyHz = 0.0f;
    gSynthTargetAmplitude = 0.0f;
    return;
  }

  const float nextFrequencyHz = (gEstimatedWaveFrequencyHz > 0.0f)
                                    ? waveFrequencyToToneFrequency(gEstimatedWaveFrequencyHz)
                                    : 0.0f;
  const uint8_t volume = waveHeightToVolume(gWaveHeightMm);
  if (nextFrequencyHz <= 0.0f) {
    gCurrentFrequencyHz = 0.0f;
    gSynthTargetFrequencyHz = 0.0f;
    gSynthTargetAmplitude = 0.0f;
    return;
  }
  gCurrentFrequencyHz = nextFrequencyHz;
  gSynthTargetFrequencyHz = nextFrequencyHz;
  gSynthTargetAmplitude = static_cast<float>(volume) / 255.0f;
}

void fillAudioChunk(std::array<int8_t, config::audio::kAudioChunkSamples>& buffer) {
  constexpr float kTwoPi = 2.0f * PI;
  for (size_t i = 0; i < config::audio::kAudioChunkSamples; ++i) {
    gSynthCurrentFrequencyHz +=
        (gSynthTargetFrequencyHz - gSynthCurrentFrequencyHz) * config::audio::kSynthFreqSmoothingPerSample;
    gSynthCurrentAmplitude +=
        (gSynthTargetAmplitude - gSynthCurrentAmplitude) * config::audio::kSynthAmpSmoothingPerSample;

    gSynthPhase += kTwoPi * gSynthCurrentFrequencyHz / static_cast<float>(config::audio::kSampleRate);
    if (gSynthPhase >= kTwoPi) {
      gSynthPhase = fmodf(gSynthPhase, kTwoPi);
    }

    const float sample = sinf(gSynthPhase) * gSynthCurrentAmplitude * 120.0f;
    buffer[i] = static_cast<int8_t>(lroundf(std::max(-127.0f, std::min(127.0f, sample))));
  }
}

void serviceAudio() {
  if (!gSpeakerReady) {
    return;
  }
  while (M5.Speaker.isPlaying(config::audio::kToneChannel) < 2) {
    auto& buffer = gAudioBuffers[gAudioBufferIndex];
    fillAudioChunk(buffer);
    M5.Speaker.playRaw(buffer.data(), buffer.size(), config::audio::kSampleRate, false, 1,
                       config::audio::kToneChannel, false);
    gAudioBufferIndex = (gAudioBufferIndex + 1) % config::audio::kAudioBufferCount;
  }
}

void switchAudioMode(AudioOutputMode mode) {
  gAudioMode = mode;
  gSpeakerReady = applySpeakerConfig(mode == AudioOutputMode::InternalSpeaker
                                         ? gInternalSpeakerConfig
                                         : gExternalSpeakerConfig);
  gSynthTargetFrequencyHz = 0.0f;
  gSynthTargetAmplitude = 0.0f;
  gSynthCurrentFrequencyHz = 0.0f;
  gSynthCurrentAmplitude = 0.0f;
  gSynthPhase = 0.0f;
  updateTone();
}

void toggleAudioMode() {
  switchAudioMode(gAudioMode == AudioOutputMode::InternalSpeaker
                      ? AudioOutputMode::ExternalI2S
                      : AudioOutputMode::InternalSpeaker);
}

void reprobeSensor() {
  if (gSensorType == SensorType::Vl53L0X) {
    gVl53.stopContinuous();
  }
  if (gSensorType == SensorType::Vl53L4CD) {
    gVl53L4cd.VL53L4CD_StopRanging();
  }
  detectSensor();
  gLatestSample = {};
  gHistoryHead = 0;
  gHistoryCount = 0;
  gDisplayDirtySamples = 0;
  gSampleRateWindowStartMs = millis();
  gSampleCountInWindow = 0;
  gSampleRateHz = 0.0f;
  gCurrentFrequencyHz = 0.0f;
  gBaselineDistanceMm = 0.0f;
  gWaveHeightMm = 0.0f;
  gEstimatedWaveFrequencyHz = 0.0f;
  gSmoothedWaveAmplitudeMm = 0.0f;
  gSynthCurrentFrequencyHz = 0.0f;
  gSynthTargetFrequencyHz = 0.0f;
  gSynthCurrentAmplitude = 0.0f;
  gSynthTargetAmplitude = 0.0f;
  gSynthPhase = 0.0f;
}

template <typename T>
void drawGraphPanel(T& gfx, int32_t x, int32_t y, int32_t w, int32_t h) {
  gfx.drawRect(x, y, w, h, kColorGrid);
  if (gHistoryCount < 2) {
    return;
  }

  uint32_t minValue = UINT32_MAX;
  uint32_t maxValue = 0;
  for (size_t i = 0; i < gHistoryCount; ++i) {
    const Sample& sample = gHistory[(gHistoryHead + i) % settings::kHistorySize];
    if (!sample.valid) {
      continue;
    }
    minValue = min<uint32_t>(minValue, sample.distanceMm);
    maxValue = max<uint32_t>(maxValue, sample.distanceMm);
  }

  if (minValue == UINT32_MAX) {
    minValue = 0;
    maxValue = 1;
  } else if (minValue == maxValue) {
    maxValue = minValue + 1;
  }

  const int32_t innerX = x + 1;
  const int32_t innerY = y + 1;
  const int32_t innerW = w - 2;
  const int32_t innerH = h - 2;

  for (size_t i = 1; i < gHistoryCount; ++i) {
    const Sample& prev = gHistory[(gHistoryHead + i - 1) % settings::kHistorySize];
    const Sample& curr = gHistory[(gHistoryHead + i) % settings::kHistorySize];
    if (!prev.valid || !curr.valid) {
      continue;
    }

    const int32_t x0 = innerX + static_cast<int32_t>(((i - 1) * (innerW - 1)) / max<size_t>(1, gHistoryCount - 1));
    const int32_t x1 = innerX + static_cast<int32_t>((i * (innerW - 1)) / max<size_t>(1, gHistoryCount - 1));
    const int32_t y0 = innerY + innerH - 1
                     - static_cast<int32_t>(((prev.distanceMm - minValue) * (innerH - 1)) / (maxValue - minValue));
    const int32_t y1 = innerY + innerH - 1
                     - static_cast<int32_t>(((curr.distanceMm - minValue) * (innerH - 1)) / (maxValue - minValue));
    gfx.drawLine(x0, y0, x1, y1, kColorDistance);
  }

  for (size_t i = 0; i < gHistoryCount; ++i) {
    const Sample& sample = gHistory[(gHistoryHead + i) % settings::kHistorySize];
    if (!sample.timeout) {
      continue;
    }
    const int32_t px = innerX + static_cast<int32_t>((i * (innerW - 1)) / max<size_t>(1, gHistoryCount - 1));
    gfx.drawFastVLine(px, innerY, innerH, kColorTimeout);
  }

  gfx.setTextColor(kColorText, kColorBg);
  gfx.setCursor(x + 4, y + 2);
  gfx.printf("mm %lu-%lu", static_cast<unsigned long>(minValue), static_cast<unsigned long>(maxValue));
}

void updateDisplay(bool force = false) {
  const uint32_t now = millis();
  if (!force) {
    if (gDisplayDirtySamples < settings::kDisplayRefreshBatchCount || now - gLastDisplayMs < settings::kDisplayIntervalMs) {
      return;
    }
  }
  gLastDisplayMs = now;
  gDisplayDirtySamples = 0;

  auto& gfx = gCanvasReady ? static_cast<lgfx::LGFXBase&>(gCanvas)
                           : static_cast<lgfx::LGFXBase&>(M5.Display);
  gfx.fillScreen(kColorBg);
  gfx.setTextColor(kColorText, kColorBg);
  gfx.setCursor(0, 0);
  gfx.printf("TOF %s  %s\n", sensorName(), audioModeName());
  gfx.printf("dist: %4u mm %s\n", gLatestSample.distanceMm, gLatestSample.timeout ? "timeout" : "ok");
  gfx.printf("freq: %6.2f Hz %s\n", static_cast<double>(gCurrentFrequencyHz), gMuted ? "mute" : "live");
  gfx.printf("wavef:%5.2f Hz\n", static_cast<double>(gEstimatedWaveFrequencyHz));
  gfx.printf("wave: %4.2f / 5.00 mm\n", static_cast<double>(gWaveHeightMm));
  gfx.printf("osc : %6.2f Hz %4.2f\n", static_cast<double>(gSynthCurrentFrequencyHz),
             static_cast<double>(gSynthCurrentAmplitude));
  gfx.printf("gate: %4.2f mm\n", static_cast<double>(config::audio::kWaveHeightNoiseFloorMm));
  gfx.setTextColor(kColorFreq, kColorBg);
  gfx.printf("rate: %5.1fHz %5.2fms\n", static_cast<double>(gSampleRateHz), static_cast<double>(samplePeriodMs()));
  gfx.setTextColor(kColorText, kColorBg);
  gfx.printf("A click: out  hold: probe\n");
  gfx.printf("B click: mute  hold: out\n");

  drawGraphPanel(gfx, 0, 48, M5.Display.width(), M5.Display.height() - 48);

  if (gCanvasReady) {
    gCanvas.pushSprite(0, 0);
  }
}

void printStatus() {
  const uint32_t now = millis();
  if (now - gLastStatusMs < settings::kStatusIntervalMs) {
    return;
  }
  gLastStatusMs = now;

  Serial.print("tof=");
  Serial.print(sensorName());
  Serial.print(" dist=");
  Serial.print(gLatestSample.distanceMm);
  Serial.print(" valid=");
  Serial.print(gLatestSample.valid ? "yes" : "no");
  Serial.print(" timeout=");
  Serial.print(gLatestSample.timeout ? "yes" : "no");
  Serial.print(" sampleHz=");
  Serial.print(gSampleRateHz, 1);
  Serial.print(" sampleMs=");
  Serial.print(samplePeriodMs(), 2);
  Serial.print(" waveMm=");
  Serial.print(gWaveHeightMm, 2);
  Serial.print(" waveHz=");
  Serial.print(gEstimatedWaveFrequencyHz, 2);
  Serial.print(" freqHz=");
  Serial.print(gCurrentFrequencyHz, 2);
  Serial.print(" oscHz=");
  Serial.print(gSynthCurrentFrequencyHz, 2);
  Serial.print(" oscAmp=");
  Serial.print(gSynthCurrentAmplitude, 3);
  Serial.print(" audio=");
  Serial.print(audioModeName());
  Serial.print(" muted=");
  Serial.println(gMuted ? "yes" : "no");
}

}  // namespace

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  M5.Display.setTextSize(1);

  gCanvas.setColorDepth(16);
  gCanvasReady = gCanvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr;
  if (gCanvasReady) {
    gCanvas.setTextSize(1);
  }

  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("HydroSynth ToF audio test");

  gInternalSpeakerConfig = M5.Speaker.config();
  gInternalSpeakerConfig.sample_rate = config::audio::kSampleRate;

  gExternalSpeakerConfig = gInternalSpeakerConfig;
  gExternalSpeakerConfig.pin_data_out = config::audio::kExternalDataOutPin;
  gExternalSpeakerConfig.pin_bck = config::audio::kExternalBclkPin;
  gExternalSpeakerConfig.pin_ws = config::audio::kExternalLrckPin;
  gExternalSpeakerConfig.i2s_port = I2S_NUM_1;
  gExternalSpeakerConfig.use_dac = false;
  gExternalSpeakerConfig.buzzer = false;
  gExternalSpeakerConfig.stereo = false;
  gExternalSpeakerConfig.sample_rate = config::audio::kSampleRate;

  gSampleRateWindowStartMs = millis();
  gSpeakerReady = applySpeakerConfig(gInternalSpeakerConfig);
  M5.Speaker.setChannelVolume(config::audio::kToneChannel, 255);
  detectSensor();
  updateDisplay(true);
}

void loop() {
  M5.update();

  if (M5.BtnA.wasClicked()) {
    toggleAudioMode();
    updateDisplay(true);
  }
  if (M5.BtnA.wasHold()) {
    reprobeSensor();
    updateDisplay(true);
  }
  if (M5.BtnB.wasClicked()) {
    gMuted = !gMuted;
    updateTone();
    updateDisplay(true);
  }
  if (M5.BtnB.wasHold()) {
    toggleAudioMode();
    updateDisplay(true);
  }

  if (gSensorType != SensorType::None) {
    gLatestSample = acquireSample();
    appendHistory(gLatestSample);
      updateSampleRate();
      updateWaveState();
      updateTone();
      ++gDisplayDirtySamples;
  } else {
    gLatestSample = {};
    gWaveHeightMm = 0.0f;
    gEstimatedWaveFrequencyHz = 0.0f;
    updateTone();
  }

  serviceAudio();
  printStatus();
  updateDisplay();
}
