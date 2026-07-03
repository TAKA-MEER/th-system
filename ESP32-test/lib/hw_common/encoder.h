#pragma once
#include <Arduino.h>
#include "pins.h"

// ============================================================
// エンコーダモジュール
// A-B 相インクリメンタルエンコーダ（割り込み駆動）
// th_ws/esp32/src/encoder.h と同一実装（ベンチ試験用に独立コピー）
// ============================================================

namespace Encoder {

extern volatile long countLeft;
extern volatile long countRight;

void init();

void IRAM_ATTR isrLeftA();
void IRAM_ATTR isrLeftB();
void IRAM_ATTR isrRightA();
void IRAM_ATTR isrRightB();

// カウンタをアトミックに読み取り & リセット
long readAndResetLeft();
long readAndResetRight();

// カウント数 → 走行距離 [m] 変換 (ホイール径・分解能は pins.h 参照)
float countsToMeters(long counts);

} // namespace Encoder
