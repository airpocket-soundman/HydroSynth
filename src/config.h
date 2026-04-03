#pragma once

#include <cstdint>

namespace config::audio {

// 外部 I2S 出力用 GPIO
constexpr int kExternalBclkPin = 5;
constexpr int kExternalLrckPin = 6;
constexpr int kExternalDataOutPin = 7;

// 音声出力サンプルレート [Hz]
constexpr uint32_t kSampleRate = 44100;

// 連続オシレータが一度に生成するサンプル数
constexpr size_t kAudioChunkSamples = 256;

// 連続再生キュー用のバッファ本数
constexpr size_t kAudioBufferCount = 3;

// マスターボリューム 0-255
constexpr uint8_t kMasterVolume = 255;

// 使用する仮想チャンネル番号
constexpr int kToneChannel = 0;

// 距離から周波数へ変換するときの最小周波数 [Hz]
constexpr float kMinToneHz = 220.0f;

// 距離から周波数へ変換するときの最大周波数 [Hz]
constexpr float kMaxToneHz = 880.0f;

// 波紋周波数を可聴域へ持ち上げる倍率
constexpr float kWaveToToneFrequencyScale = 110.0f;

// 波高がこの値に達したら最大音量にする [mm]
constexpr float kWaveHeightMaxMm = 5.0f;

// これ未満の微小振動はノイズとして無視する [mm]
constexpr float kWaveHeightNoiseFloorMm = 0.3f;

// 基準水位の追従速度。小さいほど微小な変位を拾いやすい
constexpr float kBaselineAlpha = 0.02f;

// 波紋周波数推定に使う最小周波数 [Hz]
constexpr float kMinWaveFrequencyHz = 0.8f;

// 波紋周波数推定に使う最大周波数 [Hz]
constexpr float kMaxWaveFrequencyHz = 8.0f;

// FT に使う窓長サンプル数
constexpr size_t kFftWindowSize = 64;

// 主ピークが平均スペクトルよりどれだけ強ければ採用するか
constexpr float kWavePeakRatioGate = 2.2f;

// 推定した波紋周波数の平滑化係数
constexpr float kWaveFrequencySmoothingAlpha = 0.25f;

// 推定した主成分エネルギーの平滑化係数
constexpr float kWaveAmplitudeSmoothingAlpha = 0.2f;

// 連続オシレータの周波数追従速度
constexpr float kSynthFreqSmoothingPerSample = 0.0025f;

// 連続オシレータの振幅追従速度
constexpr float kSynthAmpSmoothingPerSample = 0.004f;

}  // namespace config::audio
