#include <Arduino.h>
#include "motor.h"
#include "encoder.h"

// ============================================================
// Stage 1: モーター単体動作 + エンコーダ反応確認
//
// 左右のモーターを個別に正転/逆転させ、エンコーダのカウントが
// 期待どおりの符号・大きさで反応するかを確認する。
// duty を振って、既知の未解決課題（左モーター振動・右モーター
// 間欠動作）がどの duty 帯で再現するかも記録できるようにする。
//
// シリアルコマンド (1文字, Enter 不要):
//   l / L   左モーター 正転 / 逆転 を開始
//   r / R   右モーター 正転 / 逆転 を開始
//   1-9     duty を 10%-90% に設定
//   0       duty を 100% に設定
//   s / (space) 停止
//   h / ?   ヘルプ表示
//
// 安全: 1回の駆動は SAFETY_TIMEOUT_MS で自動停止する。
//       継続したい場合は同じコマンドを再送すること。
// ============================================================

namespace {

constexpr uint32_t SAFETY_TIMEOUT_MS = 5000;   // 1回の駆動の最大継続時間
constexpr uint32_t PRINT_PERIOD_MS   = 100;    // エンコーダ表示周期
constexpr float    KICK_DUTY         = 0.6f;   // 起動キック duty (スティクション対策検証用)
constexpr uint32_t KICK_MS           = 150;    // 起動キック継続時間

enum class Drive { NONE, LEFT_FWD, LEFT_REV, RIGHT_FWD, RIGHT_REV };

Drive    g_active      = Drive::NONE;
float    g_duty        = 0.3f;          // デフォルト 30% (安全のため低め)
bool     g_kickEnabled = false;         // 起動キック有効/無効
uint32_t g_startMs     = 0;
uint32_t g_lastPrintMs = 0;

// Welford のオンライン分散計算 (駆動中の対象輪カウント/周期のばらつき)
struct RunningStats {
    long   n    = 0;
    double mean = 0.0;
    double m2   = 0.0;

    void reset() { n = 0; mean = 0.0; m2 = 0.0; }

    void add(double x) {
        n++;
        double delta = x - mean;
        mean += delta / n;
        double delta2 = x - mean;
        m2 += delta * delta2;
    }

    double variance() const { return (n > 1) ? (m2 / (n - 1)) : 0.0; }
    double stddev()   const { return sqrt(variance()); }
};

RunningStats g_stats;

void printHelp() {
    Serial.println(F("--- Stage1: motor + encoder check ---"));
    Serial.println(F("  l/L : left  motor fwd/rev"));
    Serial.println(F("  r/R : right motor fwd/rev"));
    Serial.println(F("  1-9 : duty 10-90%   0 : duty 100%"));
    Serial.println(F("  k   : toggle start-kick (stiction workaround)"));
    Serial.println(F("  s   : stop"));
    Serial.printf("  current duty = %.0f%%  kick=%s (duty=%.0f%% %ums)\n",
                  g_duty * 100.0f, g_kickEnabled ? "ON" : "OFF",
                  KICK_DUTY * 100.0f, (unsigned)KICK_MS);
}

const char* driveName(Drive d) {
    switch (d) {
        case Drive::LEFT_FWD:  return "LEFT_FWD";
        case Drive::LEFT_REV:  return "LEFT_REV";
        case Drive::RIGHT_FWD: return "RIGHT_FWD";
        case Drive::RIGHT_REV: return "RIGHT_REV";
        default:               return "NONE";
    }
}

void stopDrive(const char* reason) {
    Motor::stopAll();
    if (g_active != Drive::NONE) {
        Serial.printf("[STOP] %s reason=%s samples=%ld mean=%.2f cnt/%ums stddev=%.2f\n",
                      driveName(g_active), reason, g_stats.n, g_stats.mean,
                      (unsigned)PRINT_PERIOD_MS, g_stats.stddev());
    }
    g_active = Drive::NONE;
}

void applyDrive(Drive d, float magnitude) {
    switch (d) {
        case Drive::LEFT_FWD:  Motor::setLeft(magnitude);    break;
        case Drive::LEFT_REV:  Motor::setLeft(-magnitude);   break;
        case Drive::RIGHT_FWD: Motor::setRight(magnitude);   break;
        case Drive::RIGHT_REV: Motor::setRight(-magnitude);  break;
        default: break;
    }
}

void startDrive(Drive d) {
    Motor::stopAll();
    g_active = d;

    // スティクション対策検証用: 一瞬だけ高duty で起動してから目標duty に落とす。
    // このキック中の回転量は計測(g_stats)に含めない。
    if (g_kickEnabled) {
        applyDrive(d, KICK_DUTY);
        delay(KICK_MS);
    }

    // 古いカウントを破棄してから計測を開始する
    Encoder::readAndResetLeft();
    Encoder::readAndResetRight();
    g_stats.reset();
    applyDrive(d, g_duty);
    g_startMs = millis();
    g_lastPrintMs = g_startMs;

    Serial.printf("[START] %s duty=%.0f%% kick=%s (auto-stop in %ums)\n",
                  driveName(d), g_duty * 100.0f, g_kickEnabled ? "ON" : "OFF",
                  (unsigned)SAFETY_TIMEOUT_MS);
}

void handleChar(char c) {
    if (c >= '1' && c <= '9') {
        g_duty = (c - '0') / 10.0f;
        Serial.printf("[SET] duty=%.0f%%\n", g_duty * 100.0f);
    } else if (c == '0') {
        g_duty = 1.0f;
        Serial.println(F("[SET] duty=100%"));
    } else if (c == 'l') {
        startDrive(Drive::LEFT_FWD);
    } else if (c == 'L') {
        startDrive(Drive::LEFT_REV);
    } else if (c == 'r') {
        startDrive(Drive::RIGHT_FWD);
    } else if (c == 'R') {
        startDrive(Drive::RIGHT_REV);
    } else if (c == 's' || c == ' ') {
        stopDrive("manual");
    } else if (c == 'k') {
        g_kickEnabled = !g_kickEnabled;
        Serial.printf("[SET] kick-start=%s (duty=%.0f%%, %ums)\n",
                      g_kickEnabled ? "ON" : "OFF", KICK_DUTY * 100.0f, (unsigned)KICK_MS);
    } else if (c == 'h' || c == '?') {
        printHelp();
    }
    // 改行等その他の文字は無視
}

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Motor::init();
    Encoder::init();
    Motor::stopAll();
    Serial.println(F("TH ESP32-test Stage1: motor + encoder check"));
    printHelp();
}

void loop() {
    while (Serial.available() > 0) {
        handleChar((char)Serial.read());
    }

    uint32_t now = millis();

    if (g_active != Drive::NONE && (now - g_startMs) >= SAFETY_TIMEOUT_MS) {
        stopDrive("safety-timeout");
    }

    if (g_active != Drive::NONE && (now - g_lastPrintMs) >= PRINT_PERIOD_MS) {
        g_lastPrintMs = now;
        long dLeft  = Encoder::readAndResetLeft();
        long dRight = Encoder::readAndResetRight();

        long driven = (g_active == Drive::LEFT_FWD || g_active == Drive::LEFT_REV) ? dLeft : dRight;
        g_stats.add((double)driven);

        Serial.printf("[ENC] t=%5lums  dLeft=%6ld  dRight=%6ld\n",
                      (unsigned long)(now - g_startMs), dLeft, dRight);
    }
}
