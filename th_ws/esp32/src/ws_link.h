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

void sendWheelFeedback(float left, float right);
void sendEstopHw(bool active);

} // namespace WsLink
