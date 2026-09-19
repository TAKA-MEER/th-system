#pragma once
#include <Arduino.h>

// ============================================================
// serial_link.h — ESP32 ⇔ ラズパイ中継(pi_serial_relay) シリアルプロトコル
//
// 2026-09-05: WiFi + WebSocket (ws_link.h) を廃止し、USB-UART (Serial, UART0)
// でラズパイに直結する構成に変更(VISION.md「ESP32の無線化をやめ、ラズパイ経由の
// シリアル接続にする」参照)。ラズパイ上の pi_serial_relay がこの反対側を持ち、
// PC の esp32_bridge (WebSocketサーバー、無変更) へ中継する。
//
// ws_protocol フレーム自体 (th_ws/src/th_esp32_bridge/th_esp32_bridge/ws_protocol.py)
// は一切変更しない。[1 byte type tag][固定長 payload] のバイト列をそのまま
// 「PAYLOAD」として下記エンベロープに封入するだけ。
//
// エンベロープ形式 (シリアル区間だけの追加層。sync2バイト分はCRC計算に含まない):
//   byte 0        : 0xAA (sync1)
//   byte 1        : 0x55 (sync2)
//   byte 2        : LEN  (payload のバイト数。最大 64)
//   byte 3..3+LEN-1: PAYLOAD (ws_protocol フレームそのまま)
//   byte 3+LEN    : CRC8( [LENバイト] + [PAYLOAD] )  ※sync 2バイトは含めない
//
// CRC8: poly=0x07, init=0x00, MSB-first, xorout無し。
// th_ws/src/th_esp32_bridge/th_esp32_bridge/serial_framer.py の crc8() と
// ビット単位で一致させること(検算済みテストベクタは同ファイル参照)。
//
// WHEEL_CMD (0x01) だけがラズパイ→ESP32方向。ESP32→ラズパイは WHEEL_FEEDBACK
// (0x02) / ESTOP_HW (0x03) / IMU_DATA (0x04)。
//
// WiFi/WebSocket 時代にあった「切断イベントで即座にモーターを0にする」
// (onDisconnect) は、シリアルには接続/切断という概念が無いため対応するものが
// 無い。従来からある WATCHDOG_MS ベースの独立監視 (main.cpp の
// watchdogTripped、最後の WHEEL_CMD 受信からの経過時間だけを見る) がそのまま
// 同じ保護を提供するため、onDisconnect 相当の仕組みは意図的に持たない。
// ============================================================

namespace SerialLink {

void init();          // Serial は main.cpp 側で begin 済みの前提 (状態初期化のみ)
void loop();          // 毎ループ呼び出し (非ブロッキング。受信バイトを読めるだけ読む)

// WHEEL_CMD 受信時に呼ばれる (targetLeft/targetRight 更新用)
void onWheelCmd(void (*callback)(float left, float right));

// dtSec: この left/right を算出した制御周期[s]。中継・ブリッジ側はこれを
// オドメトリの積分区間として使うため、速度算出に使った値をそのまま渡すこと。
void sendWheelFeedback(float left, float right, float dtSec);
// flags: ESTOP_HW のファーム構成フラグ。bit0 = bypass_active。残りビットは予約(0)。
void sendEstopHw(bool active, uint8_t flags);
void sendImuData(float qw, float qx, float qy, float qz,
                  float wx, float wy, float wz,
                  float ax, float ay, float az,
                  uint8_t calibStatus);

} // namespace SerialLink
