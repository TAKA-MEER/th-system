# ESP32 ファームウェア

[← README に戻る](../README.md)

## ビルドと書き込み

### PC に USB 直結する場合 (推奨・最速)

```bash
cd esp32
pio run                    # ビルドのみ
pio run --target upload    # ビルド + 書き込み (COM ポート自動検出。明示は --upload-port COM4)
pio device monitor         # シリアルモニタ (115200 baud)
```

### ラズパイ経由で書き込む場合 (ESP32 がラズパイに USB 接続されているとき)

PC でビルドした `firmware.bin` を scp し、ラズパイ上の esptool で書き込む
(esptool 一式は `~/esptool_env/site-packages` に配置済み):

```bash
# PC 側
cd esp32 && pio run
scp .pio/build/esp32dev/firmware.bin mirs2602@192.168.4.2:~/th_firmware.bin

# ラズパイ側 (baud は 115200 — 460800 は失敗する)
PYTHONPATH=~/esptool_env/site-packages python3 -m esptool \
  --port /dev/ttyUSB1 --baud 115200 --chip esp32 write-flash 0x10000 ~/th_firmware.bin
```

> **⚠ 書き込み中は AP が落ちる** = ssh も切れる。ラズパイ側では `setsid nohup ... &` で
> 実行し、完了後に各クライアントの WiFi 再接続([network.md](network.md))を行うこと。

### シリアルモニタの注意

シリアルポートを開くと DTR/RTS の自動リセット回路で **ESP32 が再起動する(AP も落ちる)**。
走行中・通信確認中はシリアルを開かない。開いた後は PC/ラズパイの WiFi 再接続が必要。

## WiFi 設定 (`wifi_credentials.h`)

`esp32/src/wifi_credentials.h.example` をコピーして作成(.gitignore 対象)。現行の AP 構成:

```cpp
#define WIFI_AP_MODE      1
#define AP_SSID           "th-esp32-ap"
#define AP_PASSWORD       "<APパスワード>"   // WPA2 は 8 文字以上

// esp32_bridge (PC 側) の待ち受け先。th_esp32_bridge/config/params.yaml の ws_port と一致させる
#define WS_SERVER_HOST    "192.168.4.50"
#define WS_SERVER_PORT    8766
```

`WIFI_AP_MODE 0` にすると従来の STA モード(既存ホットスポットに接続)に戻る。

## 通信仕様 (実装済みの挙動)

- `wheel_feedback` は `/cmd_vel` の有無に関わらず**毎周期(10 Hz)送信**される
  (停止中に止めると safety_monitor が ESP32_DISCONNECTED を誤検知し odom/TF も途絶するため)
- WebSocket は接続 5 分ごとに**定期リフレッシュ**(意図的な再接続)。ログの周期的な
  接続/切断は正常。死活検知は 3 秒周期の ping/pong ハートビート
- ウォッチドッグ: `wheel_cmd` が 600ms 途絶するとモーターを強制停止(ROS 非依存の最終安全。
  esp32_bridge が 20Hz キープアライブで再送するため、TCP 再送タイムアウトを踏まえ
  300ms→600ms に緩和。2026-08-05・`docs/architecture.md`「ESP32側の二重フェイルセーフ」)

## 速度制御 (PID + フィードフォワード)

`config.h` の主要パラメータ:

```txt
PID_KFF = 280 [PWM/(m/s)]   フィードフォワード。モーターは速度比例の基準 PWM が必要で、
                            PID 単独だと積分が積み上がるまで大幅な出力不足になる
                            (実測: FF なしだと指令 0.1 m/s に対し 4 秒で 0.07 m/s 止まり)。
PID_KP/KI/KD (左右別)       FF が基準を出し、PID は誤差補正を担う
TARGET_RAMP_ACCEL_MPS2      目標速度のランプ (起動時振動の防止)
WATCHDOG_MS = 600           wheel_cmd 途絶 → モーター停止
```

チューニング手順:

1. FF から合わせる: 定常速度/PWM の実測から `PID_KFF` を逆算(過大にすると
   PID の補正幅 `PID_ITERM_MAX` を超えて長時間オーバーシュートする)
2. Kp: 振動しない範囲で上げる → Ki: 定常偏差を消す → Kd: 過渡応答
3. 左右で別々に調整(旋回精度に影響)

## 開発ボード単体での通信テスト (ROS2/Docker 不要)

モーター等を未配線のボード単体で、書き込みと WiFi/WebSocket 疎通を確認できる:

```bash
# 1. PC 側 (websockets だけ pip で)
pip install websockets
python th_ws/esp32/tools/ws_test_server.py --send-test-cmd

# 2. ESP32 側
cd th_ws/esp32 && pio run --target upload && pio device monitor
```

- **書き込み確認**: 起動バナー(`TH System ESP32 Firmware (WebSocket)` + ビルド日時)
- **通信確認**: ESP32 側に `[WS] esp32_bridge に接続しました`、サーバー側に
  `[受信] ESTOP_HW ...`。`--send-test-cmd` で PC→ESP32 方向(`[WHEEL_CMD] ...`)も確認可
- **注意**: E-Stop スイッチ未配線だと GPIO34 がフローティングになり `ESTOP_HW` が
  不安定になりうる(通信テスト自体には影響なし)。安定させたい場合のみ一時的に
  `config.h` の `ESTOP_BENCH_TEST_BYPASS` を定義し、試験後は必ず戻す

## E-Stop 配線

```txt
物理スイッチ (モーター電断端子)
    ├── GPIO 34 (ESP32)
    └── GND
```

`config.h` の `ESTOP_LOW_ACTIVE` をスイッチ OFF 時の論理に合わせる。
物理スイッチは電気的にモータードライバ電源も遮断するため、ROS2 非依存の停止が同時に効く。
