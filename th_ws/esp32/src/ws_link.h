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
//   WHEEL_FEEDBACK (0x02) ESP32→bridge  float32 left, float32 right, float32 dt (13 bytes)
//   ESTOP_HW       (0x03) ESP32→bridge  uint8 active, uint8 flags (3 bytes)
//                                       flags bit0 = bypass_active
//                                       (ESTOP_BENCH_TEST_BYPASS が有効か)。
//                                       残りビットは予約(0)。
//                                       旧形式(2 bytes・flags なし)も送出しうる
//                                       個体があるため、受信側(esp32_bridge)は
//                                       長さで判別し旧形式を「flags 不明」として
//                                       安全側に倒すこと(WP-ESP32-01 E-1)。
//   IMU_DATA       (0x04) ESP32→bridge  float32 qw,qx,qy,qz,
//                                       wx,wy,wz, ax,ay,az,
//                                       uint8 calib_status          (42 bytes)
//   LED_STATE      (0x05) bridge→ESP32  uint8 state                 (未実装。表のみ)
//                                       0=起動中/1=運用可/2=フォルト/3=非常停止
//   BATTERY        (0x06) ESP32→bridge  uint16 millivolt             (未実装。表のみ)
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
// flags: ESTOP_HW のファーム構成フラグ。bit0 = bypass_active
// (config.h の ESTOP_BENCH_TEST_BYPASS が有効かどうか)。残りビットは予約(0)。
void sendEstopHw(bool active, uint8_t flags);
void sendImuData(float qw, float qx, float qy, float qz,
                  float wx, float wy, float wz,
                  float ax, float ay, float az,
                  uint8_t calibStatus);

} // namespace WsLink
