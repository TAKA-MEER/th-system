#pragma once
#include <Arduino.h>

// ============================================================
// 汎用 PID コントローラ
// ============================================================

class PID {
public:
    PID(float kp, float ki, float kd, float outMin, float outMax, float iTermMax,
        float kff = 0.0f, float outRampRate = 0.0f)
        : kp_(kp), ki_(ki), kd_(kd), kff_(kff),
          // kff: フィードフォワードゲイン [PWM/(m/s)]。0 で従来動作
          outMin_(outMin), outMax_(outMax), iTermMax_(iTermMax),
          // outRampRate: 出力(PWM)のスルーレート上限 [PWM/s]。0 で従来動作
          // (無制限)。目標速度側のランプ(main.cpp の rampToward)とは別に、
          // PID 計算後の出力そのものの変化率を抑える。実機計測(2026-09-11)で
          // 目標速度側のランプだけでは KP/KFF 由来のオーバーシュートが実出力の
          // 急変に直結すると確認されたため追加（Spec-safety.md §3（
          // 手動ジョグの発進・停止でモーター出力が急変する」参照）。
          // **setpoint==0 の停止指令には適用されない**（compute() 内で
          // 即時0を返す。障害物ブレーキ等の緊急停止も setpoint=0 を通る
          // ため、ランプで緩めると brake_accel_mps2 の実測前提が崩れる。
          // Spec-safety.md §3「安全のための停止は鈍らせない」参照）。
          // 効くのは非ゼロ setpoint 間の変化（発進・速度変更・正逆転）のみ。
          outRampRate_(outRampRate),
          iTerm_(0.0f), prevError_(0.0f), firstCall_(true), prevOutput_(0.0f) {}

    // setpoint: 目標値, measured: 計測値, dt: 経過時間 [s]
    // 戻り値: 操作量 (outMin 〜 outMax)
    float compute(float setpoint, float measured, float dt) {
        if (dt <= 0.0f) return 0.0f;
        const float maxStep = outRampRate_ * dt;

        // 目標速度がちょうど0の場合、比例・積分・微分項は経由させず即座に
        // 出力を0にする(停止時のにじり出し・積分ワインドアップを防ぐ
        // フェイルセーフ、従来どおり)。setpoint=0 は通常のジョグ解放だけで
        // なく障害物ブレーキ・estop・fault_lock・stale 由来の緊急停止も
        // すべてこの分岐を通るため、outRampRate_ で緩めてはいけない
        // (2026-09-14: 一時 outRampRate_ を適用していたところ、実機で
        // ダンボールに軽く接触する事故が発生。brake_accel_mps2=1.18 は
        // この分岐が即時0を返す前提で実測した値であり、ランプ化すると
        // 制動距離がその前提より伸びる。Spec-safety.md §3「障害物ブレーキが本来
        // より遅く制動していた回帰」参照)。outRampRate_ は非ゼロ
        // setpoint 間の変化(速度変更・発進・正逆転切替)にのみ適用する。
        if (setpoint == 0.0f) {
            iTerm_      = 0.0f;
            prevError_  = 0.0f;
            firstCall_  = true;
            prevOutput_ = 0.0f;
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
        // 出力スルーレート: 前回出力からの変化幅でさらに絞る。0 のときは無制限
        // (従来動作)。ここで絞ることで、次の if (output == rawOutput) の
        // アンチワインドアップ判定が「出力が実際にすり減らされた」ことを正しく
        // 検知し、積分項が的外れに積み上がるのを防ぐ（main.cpp 側で事後に
        // スルー制限すると、この判定をすり抜けて積分が空回りする）。
        if (maxStep > 0.0f) {
            loMin = max(loMin, prevOutput_ - maxStep);
            loMax = min(loMax, prevOutput_ + maxStep);
        }

        // フィードフォワード項: 目標速度に比例した基準出力を先に与える。
        // モーターの駆動には誤差ゼロでも一定の PWM が必要なため、PID だけだと
        // 積分項が積み上がるまで大幅に出力不足になる (実機検証 2026-07-11:
        // 指令 0.1 m/s に対し 4 秒かけて 0.07 m/s までしか到達しなかった)。
        float ffTerm = kff_ * setpoint;

        // 積分項 + アンチワインドアップ: 出力が飽和する場合は積分項を更新しない
        float iTermCandidate = constrain(iTerm_ + ki_ * error * dt, -iTermMax_, iTermMax_);
        float rawOutput = ffTerm + kp_ * error + iTermCandidate + dTerm;
        float output = constrain(rawOutput, loMin, loMax);
        if (output == rawOutput) {
            iTerm_ = iTermCandidate;
        }
        prevOutput_ = output;
        return output;
    }

    void reset() {
        iTerm_     = 0.0f;
        prevError_ = 0.0f;
        firstCall_ = true;
        prevOutput_ = 0.0f;
    }

    void setGains(float kp, float ki, float kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

private:
    float kp_, ki_, kd_, kff_;
    float outMin_, outMax_, iTermMax_;
    float outRampRate_;
    float iTerm_, prevError_;
    bool  firstCall_;
    float prevOutput_;
};
