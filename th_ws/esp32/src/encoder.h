#pragma once
#include <Arduino.h>
#include "config.h"

// ============================================================
// エンコーダモジュール
// A-B 相インクリメンタルエンコーダ（割り込み駆動）
// ============================================================

namespace Encoder {

// カウンタ (volatile: ISR からアクセスされるため)
extern volatile long countLeft;
extern volatile long countRight;

void init();

// ISR (各ピンに attachInterrupt で登録)
void IRAM_ATTR isrLeftA();
void IRAM_ATTR isrLeftB();
void IRAM_ATTR isrRightA();
void IRAM_ATTR isrRightB();

// カウンタをアトミックに読み取り & リセット
long readAndResetLeft();
long readAndResetRight();

} // namespace Encoder
