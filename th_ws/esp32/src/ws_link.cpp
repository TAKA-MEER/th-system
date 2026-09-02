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

// ── ハートビート設定 ────────────────────────────────────────
// 2026-09-02 実機計測: PC↔AP↔ESP32 の 2.4GHz リンクは平常時でも
//   ロス 22〜30% / RTT min 3.7ms・avg 455ms・max 2834ms
// という状態（構内 AP が同じ ch1 に 10 局。docs/network.md 参照）。
// 旧設定 (3000, 2000, 2) は「2 秒以内に pong が返らないのが 2 回続いたら死」
// ＝ 約 4〜5 秒の電波停止で切断判定になり、RTT が 2〜3 秒に伸びる本環境では
// 生きている接続を定期的に叩き落としていた。切断のたびに TCP を張り直して
// 復帰に約 9.5 秒かかる（handshake 失敗を挟む）ため、実害は切断そのものより
// 再接続待ちの方が大きい。
//
// 本当に死んだ接続は、10Hz で送っている WHEEL_FEEDBACK の送信失敗
// (MAX_SEND_FAILURES = 5 → 約 0.5 秒) の方が速く確実に検知できるので、
// ハートビートは「最後の保険」として大きく緩める。
// モーター安全は ESP32 側ウォッチドッグ (WATCHDOG_MS=600) が独立して担保する。
const uint32_t HEARTBEAT_PING_MS    = 10000;  // ping 間隔
const uint32_t HEARTBEAT_PONG_MS    = 5000;   // pong 待ち
const uint8_t  HEARTBEAT_MAX_MISSES = 3;      // 連続失敗で切断 (計 約30秒)

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
    case WStype_PING:
    case WStype_PONG:
        // ハートビートの正常な往復。ログしない。
        //
        // 2026-09-02 実機: ここを default: に落として毎回 Serial.printf して
        // いたため、UART が飽和し 20 秒のキャプチャで 26 万行・2.3MB が溜まって
        // いた。Serial への書き込みは TX バッファが埋まるとブロックするため、
        // シリアルモニタを繋いでいなくても loop() が止まり、WiFi/WS スタックが
        // 餓死して切断を誘発する。PONG は正常系なので黙らせる。
        break;
    default:
        // 想定外イベントだけを、レート制限つきで残す（洪水を再発させない）。
        {
            static unsigned long lastUnhandledLogMs = 0;
            if (millis() - lastUnhandledLogMs > 1000) {
                lastUnhandledLogMs = millis();
                Serial.printf("[WS] 未処理イベント type=%d len=%u\n",
                              (int)type, (unsigned)length);
            }
        }
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
    WiFi.setSleep(false);   // STA 側のモデムスリープ無効化 (STA 分岐と同じ理由)
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("[WiFi] AP IP=");
    Serial.println(WiFi.softAPIP());
    // AP モードでも、対向の esp32_bridge (PC) が同じAPネットワークの
    // クライアントとして WS_SERVER_HOST:WS_SERVER_PORT で待ち受けていれば
    // ESP32(AP)からその宛先へ outbound 接続できる。
    client.begin(WS_SERVER_HOST, WS_SERVER_PORT, "/");
    client.onEvent(handleEvent);
    client.setReconnectInterval(2000);
    client.enableHeartbeat(HEARTBEAT_PING_MS, HEARTBEAT_PONG_MS,
                           HEARTBEAT_MAX_MISSES);
    Serial.printf("[WS] 接続先: %s:%u\n", WS_SERVER_HOST, WS_SERVER_PORT);
#else
    Serial.printf("[WiFi] SSID '%s' に接続中...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    // WiFi モデムスリープを無効化する。既定 (WIFI_PS_MIN_MODEM) では DTIM
    // ビーコン間で無線を落とすため、下り小パケットが数百 ms〜数秒遅延し、
    // 2026-09-02 実機で観測した RTT max 2.8 秒・ロス 25% の主因になる。
    // 走行中は常時給電なので省電力は不要。setSleep は begin() の前に呼ぶ。
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    client.begin(WS_SERVER_HOST, WS_SERVER_PORT, "/");
    client.onEvent(handleEvent);
    client.setReconnectInterval(2000);
    // ping/pong ハートビート: TCP RST 等でライブラリが WStype_DISCONNECTED を
    // 検知できないケースが実機検証で確認されたため、能動的な生存確認で
    // 「接続中のまま固まる」状態からの復帰を保証する。
    // 値の根拠は上の HEARTBEAT_* の定義コメントを参照。
    client.enableHeartbeat(HEARTBEAT_PING_MS, HEARTBEAT_PONG_MS,
                           HEARTBEAT_MAX_MISSES);
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

void sendWheelFeedback(float left, float right, float dtSec) {
    if (!isConnected()) return;
    uint8_t buf[13];
    buf[0] = TYPE_WHEEL_FEEDBACK;
    packFloat(buf, 1, left);
    packFloat(buf, 5, right);
    // 速度は counts * distPerCount / dtSec で求めているので、このフレームが
    // 表す走行時間はまさに dtSec。ブリッジ側で到着時刻から推測させると
    // WiFi の遅延がそのままオドメトリ誤差になるため明示的に送る。
    packFloat(buf, 9, dtSec);
    handleSendResult(client.sendBIN(buf, sizeof(buf)));
}

void sendEstopHw(bool active, uint8_t flags) {
    if (!isConnected()) return;
    uint8_t buf[3] = {
        TYPE_ESTOP_HW,
        static_cast<uint8_t>(active ? 1 : 0),
        flags
    };
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
