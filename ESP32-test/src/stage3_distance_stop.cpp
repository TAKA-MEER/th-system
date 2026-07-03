#include <Arduino.h>
#include <cstring>
#include <cstdlib>
#include "motor.h"
#include "encoder.h"

// ============================================================
// Stage 3: 指定距離での停止
//
// Stage2 と同じオープンループの左右同時駆動で走行し、エンコーダ
// 積算距離 (左右平均) が指定距離に達した時点で停止する。
// 慣性によるオーバーシュート量を実測する。
//
// シリアルコマンド (行入力, Enter で確定):
//   d <meters>     指定距離だけ前進して停止 (例: d 1.0)
//   duty <0-100>   駆動 duty[%] を設定 (デフォルト 30)
//   s              走行中断
//   h / ?          ヘルプ表示
//
// 安全: 目標距離に達しないまま SAFETY_TIMEOUT_MS を超えたら
//       異常とみなし自動停止する (配線/モーター異常の検知用)。
// ============================================================

namespace {

constexpr uint32_t SAFETY_TIMEOUT_MS = 15000;  // 目標未達時の強制停止
constexpr uint32_t PRINT_PERIOD_MS   = 100;

bool     g_active     = false;
float    g_duty       = 0.3f;      // デフォルト 30%
double   g_targetM    = 0.0;
uint32_t g_startMs    = 0;
uint32_t g_lastPrintMs = 0;
double   g_distLeft   = 0.0;
double   g_distRight  = 0.0;

char     g_lineBuf[32];
uint8_t  g_lineLen = 0;

void printHelp() {
    Serial.println(F("--- Stage3: distance stop ---"));
    Serial.println(F("  d <meters>   : drive forward and stop at target distance"));
    Serial.println(F("  duty <0-100> : set drive duty[%]"));
    Serial.println(F("  s            : abort"));
    Serial.printf("  current duty=%.0f%%\n", g_duty * 100.0f);
}

void stopDrive(const char* reason) {
    Motor::stopAll();
    if (g_active) {
        double avg = (g_distLeft + g_distRight) / 2.0;
        double overshoot = avg - g_targetM;
        uint32_t elapsed = millis() - g_startMs;
        Serial.printf("[STOP] reason=%s target=%.4fm actualAvg=%.4fm overshoot=%.4fm "
                      "distLeft=%.4fm distRight=%.4fm elapsed=%lums\n",
                      reason, g_targetM, avg, overshoot, g_distLeft, g_distRight,
                      (unsigned long)elapsed);
    }
    g_active = false;
}

void startDrive(double targetM) {
    if (targetM <= 0.0) {
        Serial.println(F("[ERR] target distance must be > 0"));
        return;
    }
    Motor::stopAll();
    Encoder::readAndResetLeft();
    Encoder::readAndResetRight();
    g_distLeft  = 0.0;
    g_distRight = 0.0;
    g_targetM   = targetM;
    g_active    = true;
    g_startMs   = millis();
    g_lastPrintMs = g_startMs;

    Motor::setLeft(g_duty);
    Motor::setRight(g_duty);

    Serial.printf("[START] target=%.4fm duty=%.0f%% (safety-abort in %ums)\n",
                  g_targetM, g_duty * 100.0f, (unsigned)SAFETY_TIMEOUT_MS);
}

void handleLine(char* line) {
    // 前後の空白を無視して先頭トークンを判定
    while (*line == ' ') line++;

    if (line[0] == 'd' && (line[1] == ' ' || line[1] == '\0')) {
        double v = atof(line + 1);
        startDrive(v);
    } else if (strncmp(line, "duty", 4) == 0) {
        double v = atof(line + 4);
        if (v > 0.0 && v <= 100.0) {
            g_duty = (float)(v / 100.0);
            Serial.printf("[SET] duty=%.0f%%\n", g_duty * 100.0f);
        } else {
            Serial.println(F("[ERR] duty must be 1-100"));
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
    Serial.println(F("TH ESP32-test Stage3: distance stop"));
    printHelp();
}

void loop() {
    pollSerial();

    uint32_t now = millis();

    if (g_active && (now - g_startMs) >= SAFETY_TIMEOUT_MS) {
        stopDrive("safety-timeout(target-not-reached)");
    }

    if (g_active && (now - g_lastPrintMs) >= PRINT_PERIOD_MS) {
        g_lastPrintMs = now;
        long dLeft  = Encoder::readAndResetLeft();
        long dRight = Encoder::readAndResetRight();
        g_distLeft  += Encoder::countsToMeters(dLeft);
        g_distRight += Encoder::countsToMeters(dRight);
        double avg = (g_distLeft + g_distRight) / 2.0;

        Serial.printf("[POS] t=%5lums avg=%.4fm distLeft=%.4fm distRight=%.4fm target=%.4fm\n",
                      (unsigned long)(now - g_startMs), avg, g_distLeft, g_distRight, g_targetM);

        if (avg >= g_targetM) {
            stopDrive("target-reached");
        }
    }
}
