#pragma once
#include <Arduino.h>
#include "config.h"

// ============================================================
// モータードライバモジュール (Cytron MD10C: DIR + PWM)
// ============================================================

namespace Motor {

void init();

// speed: -1.0 (最大後退) 〜 +1.0 (最大前進) の正規化値
void setLeft(float speed);
void setRight(float speed);

// 両輪を即座に停止 (PWM=0)
void stopAll();

} // namespace Motor
