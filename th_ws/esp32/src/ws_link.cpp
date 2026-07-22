#include "ws_link.h"
#include <WiFi.h>
#include <cstring>
#include "wifi_credentials.h"

namespace WsLink {

namespace {

const uint8_t TYPE_WHEEL_CMD      = 0x01;
const uint8_t TYPE_WHEEL_FEEDBACK = 0x02;
const uint8_t TYPE_ESTOP_HW       = 0x03;
const uint8_t TYPE_IMU_DATA       = 0x04;

WebSocketsClient client;
LinkState state = LinkState::CONNECTING;
void (*wheelCmdCallback)(float, float) = nullptr;
void (*disconnectCallback)() = nullptr;

bool wifiWasConnected = false;
unsigned long lastWifiLogMs = 0;

// TCP接続がRSTで切れた場合、arduinoWebSockets ライブラリの WStype_DISCONNECTED
// 検知(ping/pongハートビートを含む)が効かず「接続中」のまま固まるケースが
// 実機検証で確認された。ライブラリの検知に頼らず、時間ベースで強制的に
// 接続を張り直す(最も確実な保険)。
// 実機検証 (2026-07-11): 15秒に設定していたところ、リフレッシュ後の再接続が
// portproxy/WSL 経由で間欠的に失敗し、約45秒の通信断が頻発した。
// 死んだ接続の検知は enableHeartbeat の ping/pong (3秒周期・約7秒で切断) が
// 担うため、時間ベースの強制再接続は「最後の保険」として長めに設定する。
unsigned long connectedSinceMs = 0;
const unsigned long FORCE_RECONNECT_MS = 300000;  // 接続後この時間で強制的に再接続

// 送信の連続失敗も補助的な検知手段として残す
int consecutiveSendFailures = 0;
const int MAX_SEND_FAILURES = 5;

void forceReconnect(const char* reason) {
    Serial.printf("[WS] %s。強制的に再接続します\n", reason);
    client.disconnect();
    client.begin(WS_SERVER_HOST, WS_SERVER_PORT, "/");
    if (state == LinkState::CONNECTED && disconnectCallback) disconnectCallback();
    state = LinkState::CONNECTING;
    consecutiveSendFailures = 0;
}

void handleSendResult(bool ok) {
    if (ok) {
        consecutiveSendFailures = 0;
        return;
    }
    consecutiveSendFailures++;
    if (consecutiveSendFailures >= MAX_SEND_FAILURES) {
        forceReconnect("送信連続失敗を検知");
    }
}

void packFloat(uint8_t* buf, size_t offset, float value) {
    memcpy(buf + offset, &value, sizeof(float));
}

float unpackFloat(const uint8_t* buf, size_t offset) {
    float value;
    memcpy(&value, buf + offset, sizeof(float));
    return value;
}

void handleEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
    case WStype_CONNECTED:
        state = LinkState::CONNECTED;
        connectedSinceMs = millis();
        Serial.printf("[WS] esp32_bridge に接続しました (%s)\n", (const char*)payload);
        break;
    case WStype_DISCONNECTED:
        Serial.println("[WS] 切断されました。再接続を試みます");
        if (state == LinkState::CONNECTED && disconnectCallback) disconnectCallback();
        state = LinkState::CONNECTING;
        break;
    case WStype_BIN:
        if (length == 9 && payload[0] == TYPE_WHEEL_CMD) {
            float left  = unpackFloat(payload, 1);
            float right = unpackFloat(payload, 5);
            if (wheelCmdCallback) wheelCmdCallback(left, right);
        }
        break;
    case WStype_ERROR:
        Serial.printf("[WS] エラー (type=%d, len=%u)\n", (int)type, (unsigned)length);
        break;
    default:
        Serial.printf("[WS] 未処理イベント type=%d len=%u\n", (int)type, (unsigned)length);
        break;
    }
}

} // namespace

bool isConnected() { return state == LinkState::CONNECTED; }

void onWheelCmd(void (*callback)(float, float)) { wheelCmdCallback = callback; }
void onDisconnect(void (*callback)()) { disconnectCallback = callback; }

void init() {
#if WIFI_AP_MODE
    Serial.printf("[WiFi] SoftAP '%s' を起動します...\n", AP_SSID);
    // WIFI_AP 単独だと WebSocketsClient (WiFiClient) の outbound 接続が
    // 内部的に失敗するESP32 Arduino既知の挙動があるため、STAは使わないが
    // WIFI_AP_STA にして TCP/IP スタックを両インターフェース分フル初期化する
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("[WiFi] AP IP=");
    Serial.println(WiFi.softAPIP());
    // AP モードでも、対向の esp32_bridge (PC) が同じAPネットワークの
    // クライアントとして WS_SERVER_HOST:WS_SERVER_PORT で待ち受けていれば
    // ESP32(AP)からその宛先へ outbound 接続できる。
    client.begin(WS_SERVER_HOST, WS_SERVER_PORT, "/");
    client.onEvent(handleEvent);
    client.setReconnectInterval(2000);
    client.enableHeartbeat(3000, 2000, 2);
    Serial.printf("[WS] 接続先: %s:%u\n", WS_SERVER_HOST, WS_SERVER_PORT);
#else
    Serial.printf("[WiFi] SSID '%s' に接続中...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    client.begin(WS_SERVER_HOST, WS_SERVER_PORT, "/");
    client.onEvent(handleEvent);
    client.setReconnectInterval(2000);
    // ping/pong ハートビート: TCP RST 等でライブラリが WStype_DISCONNECTED を
    // 検知できないケースが実機検証で確認されたため、能動的な生存確認で
    // 「接続中のまま固まる」状態からの復帰を保証する
    // (3秒毎にping、2秒以内にpongが無ければタイムアウト、2回連続で切断)
    client.enableHeartbeat(3000, 2000, 2);
    Serial.printf("[WS] 接続先: %s:%u\n", WS_SERVER_HOST, WS_SERVER_PORT);
#endif
}

void loop() {
#if WIFI_AP_MODE
    // AP接続クライアント数を定期ログ出力
    static unsigned long lastApLogMs = 0;
    if (millis() - lastApLogMs > 5000) {
        lastApLogMs = millis();
        Serial.printf("[WiFi-AP] 接続クライアント数=%d\n", WiFi.softAPgetStationNum());
    }
    client.loop();
    if (state == LinkState::CONNECTED &&
        millis() - connectedSinceMs > FORCE_RECONNECT_MS) {
        forceReconnect("定期リフレッシュ");
    }
#else
    if (WiFi.status() != WL_CONNECTED) {
        if (state == LinkState::CONNECTED && disconnectCallback) disconnectCallback();
        state = LinkState::CONNECTING;
        wifiWasConnected = false;
        // 2秒おきに接続試行中であることを知らせる(書き込み後にシリアルモニタで
        // 「フリーズしていないか」「SSIDが違うだけか」を切り分けやすくする)
        if (millis() - lastWifiLogMs > 2000) {
            lastWifiLogMs = millis();
            Serial.println("[WiFi] 接続待ち...");
        }
        return;  // WiFi 未接続時は WebSocketsClient::loop() を呼ばない
    }
    if (!wifiWasConnected) {
        wifiWasConnected = true;
        Serial.print("[WiFi] 接続しました IP=");
        Serial.println(WiFi.localIP());
    }
    client.loop();

    if (state == LinkState::CONNECTED &&
        millis() - connectedSinceMs > FORCE_RECONNECT_MS) {
        forceReconnect("定期リフレッシュ");
    }
#endif
}

void sendWheelFeedback(float left, float right) {
    if (!isConnected()) return;
    uint8_t buf[9];
    buf[0] = TYPE_WHEEL_FEEDBACK;
    packFloat(buf, 1, left);
    packFloat(buf, 5, right);
    handleSendResult(client.sendBIN(buf, sizeof(buf)));
}

void sendEstopHw(bool active) {
    if (!isConnected()) return;
    uint8_t buf[2] = { TYPE_ESTOP_HW, static_cast<uint8_t>(active ? 1 : 0) };
    handleSendResult(client.sendBIN(buf, sizeof(buf)));
}

void sendImuData(float qw, float qx, float qy, float qz,
                  float wx, float wy, float wz,
                  float ax, float ay, float az,
                  uint8_t calibStatus) {
    if (!isConnected()) return;
    uint8_t buf[42];
    buf[0] = TYPE_IMU_DATA;
    packFloat(buf, 1,  qw); packFloat(buf, 5,  qx);
    packFloat(buf, 9,  qy); packFloat(buf, 13, qz);
    packFloat(buf, 17, wx); packFloat(buf, 21, wy); packFloat(buf, 25, wz);
    packFloat(buf, 29, ax); packFloat(buf, 33, ay); packFloat(buf, 37, az);
    buf[41] = calibStatus;
    handleSendResult(client.sendBIN(buf, sizeof(buf)));
}

} // namespace WsLink
