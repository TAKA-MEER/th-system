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
//   kpl/kil/kdl <v>        左輪 PID ゲイン設定
//   kpr/kir/kdr <v>        右輪 PID ゲイン設定
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
          iTerm_(0.0f), prevError_(0.0f) {}

    float compute(float setpoint, float measured, float dt) {
        if (dt <= 0.0f) return 0.0f;
        float error = setpoint - measured;
        iTerm_ += ki_ * error * dt;
        iTerm_  = constrain(iTerm_, -iTermMax_, iTermMax_);
        float dTerm = kd_ * (error - prevError_) / dt;
        prevError_ = error;
        float output = kp_ * error + iTerm_ + dTerm;
        return constrain(output, outMin_, outMax_);
    }

    void reset() { iTerm_ = 0.0f; prevError_ = 0.0f; }
    void setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
    void getGains(float& kp, float& ki, float& kd) const { kp = kp_; ki = ki_; kd = kd_; }

private:
    float kp_, ki_, kd_;
    float outMin_, outMax_, iTermMax_;
    float iTerm_, prevError_;
};

PID g_pidLeft (PID_KP_LEFT,  PID_KI_LEFT,  PID_KD_LEFT,  PID_OUT_MIN, PID_OUT_MAX, PID_ITERM_MAX);
PID g_pidRight(PID_KP_RIGHT, PID_KI_RIGHT, PID_KD_RIGHT, PID_OUT_MIN, PID_OUT_MAX, PID_ITERM_MAX);

bool     g_active       = false;
double   g_targetDistM  = 0.0;
double   g_targetTimeS  = 0.0;
float    g_targetSpeed  = 0.0f;    // m/s
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
}

void printHelp() {
    Serial.println(F("--- Stage4: timed distance (PID tuning) ---"));
    Serial.println(F("  t <meters> <seconds> : run to target distance in target time"));
    Serial.println(F("  kpl/kil/kdl <v>      : set left  PID gain"));
    Serial.println(F("  kpr/kir/kdr <v>      : set right PID gain"));
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
    if (distM <= 0.0 || timeS <= 0.0) {
        Serial.println(F("[ERR] distance and time must be > 0"));
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

        float outL = g_pidLeft.compute(g_targetSpeed, velL, DT_S);
        float outR = g_pidRight.compute(g_targetSpeed, velR, DT_S);

        Motor::setLeft(outL / 255.0f);
        Motor::setRight(outR / 255.0f);

        double avg = (g_distLeft + g_distRight) / 2.0;
        Serial.printf("[CTRL] t=%5lums avg=%.4fm targetV=%.3f velL=%.3f velR=%.3f outL=%.1f outR=%.1f\n",
                      (unsigned long)(now - g_startMs), avg, g_targetSpeed, velL, velR, outL, outR);

        if (avg >= g_targetDistM) {
            stopDrive("target-reached");
        }
    }
}
