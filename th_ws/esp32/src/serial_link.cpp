#include "serial_link.h"
#include <cstring>

namespace SerialLink {

namespace {

const uint8_t TYPE_WHEEL_CMD      = 0x01;
const uint8_t TYPE_WHEEL_FEEDBACK = 0x02;
const uint8_t TYPE_ESTOP_HW       = 0x03;
const uint8_t TYPE_IMU_DATA       = 0x04;

const uint8_t SYNC1 = 0xAA;
const uint8_t SYNC2 = 0x55;
const uint8_t MAX_PAYLOAD = 64;

void (*wheelCmdCallback)(float, float) = nullptr;

// ── CRC8 (poly=0x07, init=0x00, MSB-first, xorout無し) ──────────────
// th_ws/src/th_esp32_bridge/th_esp32_bridge/serial_framer.py の crc8() と
// ビット単位で一致させること。1バイトずつ畳み込む形にしてあるので、
// crc8_update(0, ...) をLENバイト→PAYLOADの順に適用すれば
// crc8(bytes([len]) + payload) と同じ値になる。
uint8_t crc8Update(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                            : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

// ── 受信: バイト単位のステートマシン ─────────────────────────────
// serial_framer.py の SerialFrameDecoder と等価な考え方(sync探索→LEN→
// PAYLOAD→CRC、CRC不一致やLEN超過は先頭からの探索に戻る)。組込み側は
// バッファを巻き戻せないため、不正フレームを検知したら sync 探索を
// 最初からやり直す(Python側のような「1バイトだけ進めて再探索」より粗いが、
// 実運用のノイズ(起動バナー等)からの復帰には十分)。
enum class RxState { SEARCH_SYNC1, SEARCH_SYNC2, READ_LEN, READ_PAYLOAD, READ_CRC };

RxState rxState = RxState::SEARCH_SYNC1;
uint8_t rxLen = 0;
uint8_t rxIdx = 0;
uint8_t rxCrc = 0;
uint8_t rxPayload[MAX_PAYLOAD];

void dispatchPayload(const uint8_t* payload, uint8_t len) {
    if (len == 9 && payload[0] == TYPE_WHEEL_CMD) {
        float left, right;
        memcpy(&left,  payload + 1, sizeof(float));
        memcpy(&right, payload + 5, sizeof(float));
        if (wheelCmdCallback) wheelCmdCallback(left, right);
    }
    // 現状ラズパイ→ESP32方向は WHEEL_CMD のみ。他のtagは黙って無視する。
}

void feedByte(uint8_t b) {
    switch (rxState) {
    case RxState::SEARCH_SYNC1:
        if (b == SYNC1) rxState = RxState::SEARCH_SYNC2;
        break;
    case RxState::SEARCH_SYNC2:
        if (b == SYNC2) {
            rxState = RxState::READ_LEN;
        } else if (b != SYNC1) {
            rxState = RxState::SEARCH_SYNC1;
        }
        // b == SYNC1 ならこのバイトを新しい sync1 候補として SEARCH_SYNC2 に留まる
        break;
    case RxState::READ_LEN:
        if (b > MAX_PAYLOAD) {
            rxState = RxState::SEARCH_SYNC1;
            break;
        }
        rxLen = b;
        rxIdx = 0;
        rxCrc = crc8Update(0, b);
        rxState = (rxLen == 0) ? RxState::READ_CRC : RxState::READ_PAYLOAD;
        break;
    case RxState::READ_PAYLOAD:
        rxPayload[rxIdx++] = b;
        rxCrc = crc8Update(rxCrc, b);
        if (rxIdx >= rxLen) rxState = RxState::READ_CRC;
        break;
    case RxState::READ_CRC:
        if (b == rxCrc) dispatchPayload(rxPayload, rxLen);
        rxState = RxState::SEARCH_SYNC1;
        break;
    }
}

// ── 送信 ──────────────────────────────────────────────────────────
void sendFrame(const uint8_t* payload, uint8_t len) {
    uint8_t crc = crc8Update(0, len);
    for (uint8_t i = 0; i < len; i++) crc = crc8Update(crc, payload[i]);
    Serial.write(SYNC1);
    Serial.write(SYNC2);
    Serial.write(len);
    Serial.write(payload, len);
    Serial.write(crc);
}

void packFloat(uint8_t* buf, size_t offset, float value) {
    memcpy(buf + offset, &value, sizeof(float));
}

} // namespace

void onWheelCmd(void (*callback)(float, float)) { wheelCmdCallback = callback; }

void init() {
    rxState = RxState::SEARCH_SYNC1;
    rxIdx = 0;
}

void loop() {
    while (Serial.available() > 0) {
        feedByte(static_cast<uint8_t>(Serial.read()));
    }
}

void sendWheelFeedback(float left, float right, float dtSec) {
    uint8_t buf[13];
    buf[0] = TYPE_WHEEL_FEEDBACK;
    packFloat(buf, 1, left);
    packFloat(buf, 5, right);
    packFloat(buf, 9, dtSec);
    sendFrame(buf, sizeof(buf));
}

void sendEstopHw(bool active, uint8_t flags) {
    uint8_t buf[3] = {
        TYPE_ESTOP_HW,
        static_cast<uint8_t>(active ? 1 : 0),
        flags
    };
    sendFrame(buf, sizeof(buf));
}

void sendImuData(float qw, float qx, float qy, float qz,
                  float wx, float wy, float wz,
                  float ax, float ay, float az,
                  uint8_t calibStatus) {
    uint8_t buf[42];
    buf[0] = TYPE_IMU_DATA;
    packFloat(buf, 1,  qw); packFloat(buf, 5,  qx);
    packFloat(buf, 9,  qy); packFloat(buf, 13, qz);
    packFloat(buf, 17, wx); packFloat(buf, 21, wy); packFloat(buf, 25, wz);
    packFloat(buf, 29, ax); packFloat(buf, 33, ay); packFloat(buf, 37, az);
    buf[41] = calibStatus;
    sendFrame(buf, sizeof(buf));
}

} // namespace SerialLink
