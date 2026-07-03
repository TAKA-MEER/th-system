#include <Arduino.h>
#include "motor.h"
#include "encoder.h"

// ============================================================
// Stage 2: 直進性確認 (オープンループ)
//
// 左右モーターに同一 duty を同時出力し、PID なしの素の機械特性
// として直進性・左右の走行距離差を定量化する。
// ここで得られる左右差は Stage 4 のゲイン調整の基礎データになる。
//
// シリアルコマンド (1文字, Enter 不要):
//   g       前進開始 (左右同時・同一 duty)
//   G       後退開始
//   1-9     duty を 10%-90% に設定
//   0       duty を 100% に設定
//   s / (space) 停止
//   h / ?   ヘルプ表示
//
// 安全: 1回の走行は SAFETY_TIMEOUT_MS で自動停止する。
// ============================================================

namespace {

constexpr uint32_t SAFETY_TIMEOUT_MS = 8000;   // 1回の走行の最大継続時間
constexpr uint32_t PRINT_PERIOD_MS   = 100;    // 表示周期

bool     g_active   = false;
bool     g_reverse  = false;
float    g_duty     = 0.3f;            // デフォルト 30%
uint32_t g_startMs  = 0;
uint32_t g_lastPrintMs = 0;
double   g_distLeft  = 0.0;            // 累積走行距離 [m]
double   g_distRight = 0.0;

void printHelp() {
    Serial.println(F("--- Stage2: straight line (open loop) ---"));
    Serial.println(F("  g   : go forward (both wheels, same duty)"));
    Serial.println(F("  G   : go backward"));
    Serial.println(F("  1-9 : duty 10-90%   0 : duty 100%"));
    Serial.println(F("  s   : stop"));
    Serial.printf("  current duty = %.0f%%\n", g_duty * 100.0f);
}

void printSummary(const char* reason) {
    double diff = g_distLeft - g_distRight;
    double avg  = (g_distLeft + g_distRight) / 2.0;
    double diffPct = (avg != 0.0) ? (diff / avg * 100.0) : 0.0;
    Serial.printf("[STOP] reason=%s distLeft=%.4fm distRight=%.4fm diff=%.4fm (%.1f%%)\n",
                  reason, g_distLeft, g_distRight, diff, diffPct);
}

void stopDrive(const char* reason) {
    Motor::stopAll();
    if (g_active) {
        printSummary(reason);
    }
    g_active = false;
}

void startDrive(bool reverse) {
    Motor::stopAll();
    Encoder::readAndResetLeft();
    Encoder::readAndResetRight();
    g_distLeft  = 0.0;
    g_distRight = 0.0;
    g_reverse   = reverse;
    g_active    = true;
    g_startMs   = millis();
    g_lastPrintMs = g_startMs;

    float speed = reverse ? -g_duty : g_duty;
    Motor::setLeft(speed);
    Motor::setRight(speed);

    Serial.printf("[START] %s duty=%.0f%% (auto-stop in %ums)\n",
                  reverse ? "REVERSE" : "FORWARD", g_duty * 100.0f, (unsigned)SAFETY_TIMEOUT_MS);
}

void handleChar(char c) {
    if (c >= '1' && c <= '9') {
        g_duty = (c - '0') / 10.0f;
        Serial.printf("[SET] duty=%.0f%%\n", g_duty * 100.0f);
    } else if (c == '0') {
        g_duty = 1.0f;
        Serial.println(F("[SET] duty=100%"));
    } else if (c == 'g') {
        startDrive(false);
    } else if (c == 'G') {
        startDrive(true);
    } else if (c == 's' || c == ' ') {
        stopDrive("manual");
    } else if (c == 'h' || c == '?') {
        printHelp();
    }
}

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Motor::init();
    Encoder::init();
    Motor::stopAll();
    Serial.println(F("TH ESP32-test Stage2: straight line (open loop)"));
    printHelp();
}

void loop() {
    while (Serial.available() > 0) {
        handleChar((char)Serial.read());
    }

    uint32_t now = millis();

    if (g_active && (now - g_startMs) >= SAFETY_TIMEOUT_MS) {
        stopDrive("safety-timeout");
    }

    if (g_active && (now - g_lastPrintMs) >= PRINT_PERIOD_MS) {
        g_lastPrintMs = now;
        long dLeft  = Encoder::readAndResetLeft();
        long dRight = Encoder::readAndResetRight();
        g_distLeft  += Encoder::countsToMeters(dLeft);
        g_distRight += Encoder::countsToMeters(dRight);

        double diff = g_distLeft - g_distRight;
        Serial.printf("[POS] t=%5lums distLeft=%.4fm distRight=%.4fm diff=%.4fm\n",
                      (unsigned long)(now - g_startMs), g_distLeft, g_distRight, diff);
    }
}
