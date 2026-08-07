#pragma once
#include <Arduino.h>
#include <WebSocketsClient.h>

// ============================================================
// ws_link.h — ESP32 ⇔ esp32_bridge WebSocket バイナリプロトコル
//
// フレーム形式: [1 byte type tag][固定長 payload] (little-endian)
// th_ws/src/th_esp32_bridge/th_esp32_bridge/ws_protocol.py と
// バイト配置を必ず一致させること。
//
//   WHEEL_CMD      (0x01) bridge→ESP32  float32 left, float32 right (9 bytes)
//   WHEEL_FEEDBACK (0x02) ESP32→bridge  float32 left, float32 right (9 bytes)
//   ESTOP_HW       (0x03) ESP32→bridge  uint8 active                (2 bytes)
//   IMU_DATA       (0x04) ESP32→bridge  float32 qw,qx,qy,qz,
//                                       wx,wy,wz, ax,ay,az,
//                                       uint8 calib_status          (42 bytes)
//
// esp32_bridge (PC側) が WebSocket サーバー、ESP32 がクライアント。
// wifi_credentials.h の WIFI_SSID/WIFI_PASSWORD/WS_SERVER_HOST/WS_SERVER_PORT
// を使って接続する。
// ============================================================

namespace WsLink {

enum class LinkState { CONNECTING, CONNECTED };

void init();          // WiFi + WebSocket 接続開始 (非ブロッキング)
void loop();          // 毎ループ呼び出し (非ブロッキング)
bool isConnected();

// WHEEL_CMD 受信時に呼ばれる (targetLeft/targetRight 更新用)
void onWheelCmd(void (*callback)(float left, float right));
// WS切断検知時に呼ばれる (即座にモーター停止するため)
void onDisconnect(void (*callback)());

// dtSec: この left/right を算出した制御周期[s]。ブリッジ側はこれを
// オドメトリの積分区間として使うため、速度算出に使った値をそのまま渡すこと
// (2026-08-06 追加。ws_protocol.py のフレーム定義と必ず一致させる)。
void sendWheelFeedback(float left, float right, float dtSec);
void sendEstopHw(bool active);
void sendImuData(float qw, float qx, float qy, float qz,
                  float wx, float wy, float wz,
                  float ax, float ay, float az,
                  uint8_t calibStatus);

} // namespace WsLink
