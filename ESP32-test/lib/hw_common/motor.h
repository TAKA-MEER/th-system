#pragma once
#include <Arduino.h>
#include "pins.h"

// ============================================================
// モータードライバモジュール (Cytron MD10C: DIR + PWM)
// th_ws/esp32/src/motor.h と同一実装（ベンチ試験用に独立コピー）
// ============================================================

namespace Motor {

void init();

// speed: -1.0 (最大後退) 〜 +1.0 (最大前進) の正規化値
void setLeft(float speed);
void setRight(float speed);

// 両輪を即座に停止 (PWM=0)
void stopAll();

} // namespace Motor
