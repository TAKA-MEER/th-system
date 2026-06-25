#pragma once
#include <Arduino.h>

// ============================================================
// 汎用 PID コントローラ
// ============================================================

class PID {
public:
    PID(float kp, float ki, float kd, float outMin, float outMax, float iTermMax)
        : kp_(kp), ki_(ki), kd_(kd),
          outMin_(outMin), outMax_(outMax), iTermMax_(iTermMax),
          iTerm_(0.0f), prevError_(0.0f) {}

    // setpoint: 目標値, measured: 計測値, dt: 経過時間 [s]
    // 戻り値: 操作量 (outMin 〜 outMax)
    float compute(float setpoint, float measured, float dt) {
        if (dt <= 0.0f) return 0.0f;

        float error = setpoint - measured;

        // 積分項 (ワインドアップ防止)
        iTerm_ += ki_ * error * dt;
        iTerm_  = constrain(iTerm_, -iTermMax_, iTermMax_);

        // 微分項
        float dTerm = kd_ * (error - prevError_) / dt;
        prevError_  = error;

        float output = kp_ * error + iTerm_ + dTerm;
        return constrain(output, outMin_, outMax_);
    }

    void reset() {
        iTerm_     = 0.0f;
        prevError_ = 0.0f;
    }

    void setGains(float kp, float ki, float kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

private:
    float kp_, ki_, kd_;
    float outMin_, outMax_, iTermMax_;
    float iTerm_, prevError_;
};
