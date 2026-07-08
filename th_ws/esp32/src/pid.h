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
          iTerm_(0.0f), prevError_(0.0f), firstCall_(true) {}

    // setpoint: 目標値, measured: 計測値, dt: 経過時間 [s]
    // 戻り値: 操作量 (outMin 〜 outMax)
    float compute(float setpoint, float measured, float dt) {
        if (dt <= 0.0f) return 0.0f;

        // 目標速度がちょうど0の場合はPID計算を経由せず強制的に出力0にする
        // (停止時のにじり出し・積分ワインドアップを防ぐフェイルセーフ)
        if (setpoint == 0.0f) {
            iTerm_     = 0.0f;
            prevError_ = 0.0f;
            firstCall_ = true;
            return 0.0f;
        }

        float error = setpoint - measured;

        // 微分項
        // reset() 直後の1周期目は prevError_ が未確定 (0) のため、目標値の
        // ステップ変化がそのままD項の急峻な微分キックになる。初回はD項を無効化する。
        float dTerm = firstCall_ ? 0.0f : kd_ * (error - prevError_) / dt;
        firstCall_  = false;
        prevError_  = error;

        // 進行方向と逆向きの出力(逆転ブレーキ)を禁止する非対称クランプ。
        // 逆転ブレーキによる暴れ(発振)を避けるため、目標速度と同符号側のみ許可する。
        float loMin = (setpoint >= 0.0f) ? 0.0f    : outMin_;
        float loMax = (setpoint >= 0.0f) ? outMax_ : 0.0f;

        // 積分項 + アンチワインドアップ: 出力が飽和する場合は積分項を更新しない
        float iTermCandidate = constrain(iTerm_ + ki_ * error * dt, -iTermMax_, iTermMax_);
        float rawOutput = kp_ * error + iTermCandidate + dTerm;
        float output = constrain(rawOutput, loMin, loMax);
        if (output == rawOutput) {
            iTerm_ = iTermCandidate;
        }
        return output;
    }

    void reset() {
        iTerm_     = 0.0f;
        prevError_ = 0.0f;
        firstCall_ = true;
    }

    void setGains(float kp, float ki, float kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

private:
    float kp_, ki_, kd_;
    float outMin_, outMax_, iTermMax_;
    float iTerm_, prevError_;
    bool  firstCall_;
};
