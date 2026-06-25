#include "encoder.h"

namespace Encoder {

volatile long countLeft  = 0;
volatile long countRight = 0;

void init() {
    pinMode(ENC_LEFT_A,  INPUT_PULLUP);
    pinMode(ENC_LEFT_B,  INPUT_PULLUP);
    pinMode(ENC_RIGHT_A, INPUT_PULLUP);
    pinMode(ENC_RIGHT_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  isrLeftA,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B),  isrLeftB,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), isrRightB, CHANGE);
}

// ── 左エンコーダ ISR ─────────────────────────────────────────
void IRAM_ATTR isrLeftA() {
    bool a = digitalRead(ENC_LEFT_A);
    bool b = digitalRead(ENC_LEFT_B);
    countLeft += (a == b) ? 1 : -1;
}
void IRAM_ATTR isrLeftB() {
    bool a = digitalRead(ENC_LEFT_A);
    bool b = digitalRead(ENC_LEFT_B);
    countLeft += (a != b) ? 1 : -1;
}

// ── 右エンコーダ ISR ─────────────────────────────────────────
void IRAM_ATTR isrRightA() {
    bool a = digitalRead(ENC_RIGHT_A);
    bool b = digitalRead(ENC_RIGHT_B);
    countRight += (a == b) ? 1 : -1;
}
void IRAM_ATTR isrRightB() {
    bool a = digitalRead(ENC_RIGHT_A);
    bool b = digitalRead(ENC_RIGHT_B);
    countRight += (a != b) ? 1 : -1;
}

// ── アトミック読み取り & リセット ────────────────────────────
long readAndResetLeft() {
    noInterrupts();
    long v = countLeft;
    countLeft = 0;
    interrupts();
    return v;
}

long readAndResetRight() {
    noInterrupts();
    long v = countRight;
    countRight = 0;
    interrupts();
    return v;
}

} // namespace Encoder
