#include <Arduino.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include "motor.h"
#include "encoder.h"

// ============================================================
// Stage 4: 指定時間・指定距離への到達 (速度 PID ゲイン調整)
//
// 目標距離・目標時間から目標平均速度を算出し、左右独立の速度 PID
// で追従走行する。ゲインはシリアルからその場で変更でき、再書き込み
// なしにチューニングを繰り返せる。
// 初期ゲインは th_ws/esp32/src/config.h の未検証値と同一。
//
// シリアルコマンド (行入力, Enter で確定):
//   t <meters> <seconds>   指定距離を指定時間で走行 (例: t 1.0 2.0)
//                          距離を負値にすると後退 (例: t -1.0 2.0)
//   kpl/kil/kdl <v>        左輪 PID ゲイン設定
//   kpr/kir/kdr <v>        右輪 PID ゲイン設定
//   iml/imr <v>             左/右輪 積分項上限(iTermMax)設定
//   a <mps2>                目標速度のランプ加速度設定 (m/s^2, 起動直後の振動緩和用)
//   s                      走行中断
//   h / ?                  ヘルプ表示
//
// 安全: 目標時間の3倍 + 5秒 (上限30秒) を超えても目標距離に
//       達しない場合は異常とみなし自動停止する。
// ============================================================

namespace {

constexpr uint32_t CTRL_PERIOD = CTRL_PERIOD_MS;  // pins.h (100ms)
constexpr float    DT_S        = CTRL_PERIOD / 1000.0f;
constexpr uint32_t MAX_SAFETY_TIMEOUT_MS = 30000;

// th_ws/esp32/src/pid.h と同一実装（ベンチ試験用に独立コピー）
class PID {
public:
    PID(float kp, float ki, float kd, float outMin, float outMax, float iTermMax)
        : kp_(kp), ki_(ki), kd_(kd),
          outMin_(outMin), outMax_(outMax), iTermMax_(iTermMax),
          iTerm_(0.0f), prevError_(0.0f), firstCall_(true) {}

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

        // reset() 直後の1周期目は prevError_ が未確定 (0) のため、目標値の
        // ステップ変化がそのままD項の急峻な微分キックになる。初回はD項を無効化する。
        float dTerm = firstCall_ ? 0.0f : kd_ * (error - prevError_) / dt;
        firstCall_ = false;
        prevError_ = error;

        // 進行方向と逆向きの出力(逆転ブレーキ)を禁止する非対称クランプ。
        // 逆転ブレーキによる暴れ(発振)を避けるため、目標速度と同符号側のみ許可する。
        float loMin = (setpoint >= 0.0f) ? 0.0f    : outMin_;
        float loMax = (setpoint >= 0.0f) ? outMax_ : 0.0f;

        // アンチワインドアップ: 出力が飽和する場合は積分項を更新しない
        float iTermCandidate = constrain(iTerm_ + ki_ * error * dt, -iTermMax_, iTermMax_);
        float rawOutput = kp_ * error + iTermCandidate + dTerm;
        float output = constrain(rawOutput, loMin, loMax);
        if (output == rawOutput) {
            iTerm_ = iTermCandidate;
        }
        return output;
    }

    void reset() { iTerm_ = 0.0f; prevError_ = 0.0f; firstCall_ = true; }
    void setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
    void getGains(float& kp, float& ki, float& kd) const { kp = kp_; ki = ki_; kd = kd_; }
    void setItermMax(float v) { iTermMax_ = v; }
    float getItermMax() const { return iTermMax_; }

private:
    float kp_, ki_, kd_;
    float outMin_, outMax_, iTermMax_;
    float iTerm_, prevError_;
    bool  firstCall_;
};

PID g_pidLeft (PID_KP_LEFT,  PID_KI_LEFT,  PID_KD_LEFT,  PID_OUT_MIN, PID_OUT_MAX, PID_ITERM_MAX);
PID g_pidRight(PID_KP_RIGHT, PID_KI_RIGHT, PID_KD_RIGHT, PID_OUT_MIN, PID_OUT_MAX, PID_ITERM_MAX);

bool     g_active       = false;
double   g_targetDistM  = 0.0;
double   g_targetTimeS  = 0.0;
float    g_targetSpeed  = 0.0f;    // m/s (最終目標)
float    g_rampSpeed    = 0.0f;    // m/s (PIDに渡す、加速度制限された現在の目標値)
float    g_rampAccel    = 1.5f;    // m/s^2 (起動直後の急加速による振動を抑えるためのランプ加速度)
uint32_t g_startMs      = 0;
uint32_t g_lastCtrlMs   = 0;
uint32_t g_safetyMs     = MAX_SAFETY_TIMEOUT_MS;
double   g_distLeft     = 0.0;
double   g_distRight    = 0.0;

char     g_lineBuf[32];
uint8_t  g_lineLen = 0;

void printGains() {
    float kp, ki, kd;
    g_pidLeft.getGains(kp, ki, kd);
    Serial.printf("  left  Kp=%.2f Ki=%.2f Kd=%.2f\n", kp, ki, kd);
    g_pidRight.getGains(kp, ki, kd);
    Serial.printf("  right Kp=%.2f Ki=%.2f Kd=%.2f\n", kp, ki, kd);
    Serial.printf("  itermMax left=%.2f right=%.2f\n", g_pidLeft.getItermMax(), g_pidRight.getItermMax());
    Serial.printf("  ramp  accel=%.2fm/s^2\n", g_rampAccel);
}

void printHelp() {
    Serial.println(F("--- Stage4: timed distance (PID tuning) ---"));
    Serial.println(F("  t <meters> <seconds> : run to target distance in target time"));
    Serial.println(F("  kpl/kil/kdl <v>      : set left  PID gain"));
    Serial.println(F("  kpr/kir/kdr <v>      : set right PID gain"));
    Serial.println(F("  iml/imr <v>          : set left/right iTermMax"));
    Serial.println(F("  a <mps2>             : set target-speed ramp accel"));
    Serial.println(F("  s                    : abort"));
    Serial.println(F("  current gains:"));
    printGains();
}

void stopDrive(const char* reason) {
    Motor::stopAll();
    if (g_active) {
        double avg = (g_distLeft + g_distRight) / 2.0;
        double distErr = avg - g_targetDistM;
        double elapsedS = (millis() - g_startMs) / 1000.0;
        double timeErr = elapsedS - g_targetTimeS;
        Serial.printf("[STOP] reason=%s targetDist=%.4fm actualAvg=%.4fm distErr=%.4fm "
                      "targetTime=%.2fs elapsed=%.2fs timeErr=%.2fs\n",
                      reason, g_targetDistM, avg, distErr, g_targetTimeS, elapsedS, timeErr);
        printGains();
    }
    g_active = false;
}

void startDrive(double distM, double timeS) {
    if (distM == 0.0 || timeS <= 0.0) {
        Serial.println(F("[ERR] distance must be != 0, time must be > 0"));
        return;
    }
    Motor::stopAll();
    Encoder::readAndResetLeft();
    Encoder::readAndResetRight();
    g_pidLeft.reset();
    g_pidRight.reset();
    g_distLeft    = 0.0;
    g_distRight   = 0.0;
    g_targetDistM = distM;
    g_targetTimeS = timeS;
    g_targetSpeed = (float)(distM / timeS);
    g_rampSpeed   = 0.0f;
    g_active      = true;
    g_startMs     = millis();
    g_lastCtrlMs  = g_startMs;

    uint32_t bound = (uint32_t)(timeS * 3.0 * 1000.0) + 5000;
    g_safetyMs = (bound < MAX_SAFETY_TIMEOUT_MS) ? bound : MAX_SAFETY_TIMEOUT_MS;

    Serial.printf("[START] dist=%.4fm time=%.2fs targetSpeed=%.4fm/s (safety-abort in %lums)\n",
                  distM, timeS, g_targetSpeed, (unsigned long)g_safetyMs);
    printGains();
}

void handleLine(char* line) {
    while (*line == ' ') line++;

    if (line[0] == 't' && (line[1] == ' ' || line[1] == '\0')) {
        double d = 0.0, t = 0.0;
        if (sscanf(line + 1, "%lf %lf", &d, &t) == 2) {
            startDrive(d, t);
        } else {
            Serial.println(F("[ERR] usage: t <meters> <seconds>"));
        }
    } else if (strncmp(line, "kpl", 3) == 0 || strncmp(line, "kil", 3) == 0 || strncmp(line, "kdl", 3) == 0 ||
               strncmp(line, "kpr", 3) == 0 || strncmp(line, "kir", 3) == 0 || strncmp(line, "kdr", 3) == 0) {
        float v = atof(line + 3);
        float kp, ki, kd;
        bool isLeft = (line[2] == 'l');
        PID& pid = isLeft ? g_pidLeft : g_pidRight;
        pid.getGains(kp, ki, kd);
        if (line[1] == 'p') kp = v;
        else if (line[1] == 'i') ki = v;
        else if (line[1] == 'd') kd = v;
        pid.setGains(kp, ki, kd);
        Serial.printf("[SET] %s gains: Kp=%.2f Ki=%.2f Kd=%.2f\n", isLeft ? "left" : "right", kp, ki, kd);
    } else if (strncmp(line, "iml", 3) == 0 || strncmp(line, "imr", 3) == 0) {
        float v = atof(line + 3);
        bool isLeft = (line[2] == 'l');
        if (v > 0.0f) {
            PID& pid = isLeft ? g_pidLeft : g_pidRight;
            pid.setItermMax(v);
            Serial.printf("[SET] %s itermMax=%.2f\n", isLeft ? "left" : "right", v);
        } else {
            Serial.println(F("[ERR] usage: iml/imr <v> (> 0)"));
        }
    } else if (line[0] == 'a' && (line[1] == ' ' || line[1] == '\0')) {
        float v = atof(line + 1);
        if (v > 0.0f) {
            g_rampAccel = v;
            Serial.printf("[SET] ramp accel=%.2fm/s^2\n", g_rampAccel);
        } else {
            Serial.println(F("[ERR] usage: a <mps2> (> 0)"));
        }
    } else if (line[0] == 's') {
        stopDrive("manual");
    } else if (line[0] == 'h' || line[0] == '?') {
        printHelp();
    } else if (line[0] != '\0') {
        Serial.println(F("[ERR] unknown command (h for help)"));
    }
}

void pollSerial() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            g_lineBuf[g_lineLen] = '\0';
            handleLine(g_lineBuf);
            g_lineLen = 0;
        } else if (g_lineLen < sizeof(g_lineBuf) - 1) {
            g_lineBuf[g_lineLen++] = c;
        }
    }
}

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Motor::init();
    Encoder::init();
    Motor::stopAll();
    Serial.println(F("TH ESP32-test Stage4: timed distance (PID tuning)"));
    printHelp();
}

void loop() {
    pollSerial();

    uint32_t now = millis();

    if (g_active && (now - g_startMs) >= g_safetyMs) {
        stopDrive("safety-timeout(target-not-reached)");
        return;
    }

    if (g_active && (now - g_lastCtrlMs) >= CTRL_PERIOD) {
        g_lastCtrlMs = now;

        long dLeft  = Encoder::readAndResetLeft();
        long dRight = Encoder::readAndResetRight();
        float distStepL = Encoder::countsToMeters(dLeft);
        float distStepR = Encoder::countsToMeters(dRight);
        g_distLeft  += distStepL;
        g_distRight += distStepR;

        float velL = distStepL / DT_S;
        float velR = distStepR / DT_S;

        // 目標速度への急激なステップ入力がP項の急崩壊(=起動直後の振動)を招くため、
        // PIDに渡す目標値は加速度制限してランプさせる。target/rampとも前進なら正、
        // 後退なら負なので、両方向とも0から離れる向きにランプすればよい。
        float rampStep = g_rampAccel * DT_S;
        if (g_rampSpeed < g_targetSpeed) {
            g_rampSpeed = min(g_rampSpeed + rampStep, g_targetSpeed);
        } else if (g_rampSpeed > g_targetSpeed) {
            g_rampSpeed = max(g_rampSpeed - rampStep, g_targetSpeed);
        }

        float outL = g_pidLeft.compute(g_rampSpeed, velL, DT_S);
        float outR = g_pidRight.compute(g_rampSpeed, velR, DT_S);

        Motor::setLeft(outL / 255.0f);
        Motor::setRight(outR / 255.0f);

        double avg = (g_distLeft + g_distRight) / 2.0;
        Serial.printf("[CTRL] t=%5lums avg=%.4fm targetV=%.3f rampV=%.3f velL=%.3f velR=%.3f outL=%.1f outR=%.1f\n",
                      (unsigned long)(now - g_startMs), avg, g_targetSpeed, g_rampSpeed, velL, velR, outL, outR);

        bool reached = (g_targetDistM >= 0.0) ? (avg >= g_targetDistM) : (avg <= g_targetDistM);
        if (reached) {
            stopDrive("target-reached");
        }
    }
}
