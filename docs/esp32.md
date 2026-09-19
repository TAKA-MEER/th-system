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
scp .pio/build/esp32dev/firmware.bin mirs2602@192.168.5.1:~/th_firmware.bin

# ラズパイ側 (baud は 115200 — 460800 は失敗する)
PYTHONPATH=~/esptool_env/site-packages python3 -m esptool \
  --port /dev/ttyUSB1 --baud 115200 --chip esp32 write-flash 0x10000 ~/th_firmware.bin
```

> **⚠ 書き込み中、ラズパイが `pi_serial_relay` でポートを掴んでいると esptool が
> 失敗する。** `sudo systemctl stop rpi-serial-relay` してから書き込み、終わったら
> `sudo systemctl start rpi-serial-relay`（再起動時に ESP32 側の起動バナーが
> 1回出るのは正常。書き込み後の初回起動）。

### シリアルモニタの注意

シリアルポートを開くと DTR/RTS の自動リセット回路で **ESP32 が再起動する**。
走行中・`pi_serial_relay` 稼働中はシリアルを開かない
（書き込み時も esptool 自身が意図的にこれを使って ESP32 をブートローダへ
落とすので問題ないが、**シリアルモニタと `pi_serial_relay` を同時に同じポートへ
繋ごうとしない**こと。ポートは同時に1プロセスしか開けない）。

## ラズパイ接続 (シリアル)

**2026-09-05 以降、ESP32 は WiFi を一切使わない。** USB-UART でラズパイに直結し、
ラズパイ上の `pi_serial_relay`（`th_ws/scripts/pi_serial_relay.py`）が PC の
`esp32_bridge` へ WebSocket クライアントとして接続する（旧`wifi_credentials.h`は
廃止・削除済み）。

構成の詳細・切り分け手順・導入手順は [network.md](network.md)「ラズパイ:
pi_serial_relay の導入」参照。要点だけ書くと:

- ESP32 とラズパイの間は USB-UART 1本（フラッシュに使っているケーブルと同じ）。
  ラズパイ側のポートは **`/dev/serial/by-id/...` で固定**すること
  （RPLIDAR も同じラズパイの USB-UART で、列挙順は挿抜のたびに入れ替わりうる）。
- プロトコルは既存の `ws_protocol.py` フレームを変更せず、シリアル区間だけに
  `serial_framer.py`（sync `0xAA 0x55` + len + CRC8 のエンベロープ）を被せる。
  ブート時 ASCII バナーが混ざっても resync して後続フレームを正しく拾う。
- ウォッチドッグ(600ms)・E-Stop は無変更。WiFi 切断イベントで即座にモーターを
  0 にする最適化（旧`onDisconnect`）はシリアルには対応する概念が無いため無くなったが、
  従来からある `WATCHDOG_MS` ベースの独立監視がそのまま同じ保護を提供する。

## 通信仕様 (実装済みの挙動)

- `wheel_feedback` は `/cmd_vel` の有無に関わらず**毎周期(10 Hz)送信**される
  (停止中に止めると safety_monitor が ESP32_DISCONNECTED を誤検知し odom/TF も途絶するため)
- `wheel_feedback` には左右速度に加えて**その速度を算出した制御周期 `dt_sec`** を載せる
  (2026-08-06、9→13 byte)。esp32_bridge はこれをオドメトリの積分区間として使う。
  到着時刻から推測すると通信側の遅延がそのまま yaw ドリフトになるため。
  ブリッジは旧形式(9 byte)も受理するので、**書き込み前の個体でもそのまま動く**
  (公称周期にフォールバックするだけ)。ただし旋回精度の改善は書き込み後に効く
- ウォッチドッグ: `wheel_cmd` が 600ms 途絶するとモーターを強制停止(ROS 非依存の最終安全。
  シリアル化後も値は変更していない。2026-08-05・`docs/architecture.md`「ESP32側の二重フェイルセーフ」)

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

## 開発ボード単体での通信テスト (ROS2/Docker/ラズパイ 不要)

モーター等を未配線のボード単体で、PCへ直接USB接続したまま書き込みとシリアル
プロトコル疎通を確認できる(`pi_serial_relay` を経由しない、PC直結での単体テスト):

```bash
# 1. ESP32 側
cd th_ws/esp32 && pio run --target upload

# 2. PC 側 (pyserial だけ pip で)
pip install pyserial
python3 th_ws/esp32/tools/serial_test.py /dev/ttyUSB0 --send-test-cmd
```

- **書き込み確認**: 起動バナー(`TH System ESP32 Firmware (Serial)` + ビルド日時)を
  そのまま画面に流す(バイナリフレームに混じったASCIIノイズとして表示されるだけで無害)
- **通信確認**: `[受信] WHEEL_FEEDBACK ...` / `ESTOP_HW ...` がデコードされて表示される。
  `--send-test-cmd` で PC→ESP32 方向(`WHEEL_CMD`)も確認可(速度0を送るだけなので安全)
- **注意**: E-Stop スイッチ未配線だと GPIO34 がフローティングになり `ESTOP_HW` が
  不安定になりうる(通信テスト自体には影響なし)。安定させたい場合のみ一時的に
  `config.h` の `ESTOP_BENCH_TEST_BYPASS` を定義し、試験後は必ず戻す
- ラズパイ経由(`pi_serial_relay` + `esp32_bridge`)の最終疎通確認は
  [network.md](network.md)「ラズパイ: pi_serial_relay の導入」の手順で別途行うこと

## E-Stop 配線

```txt
物理スイッチ (モーター電断端子)
    ├── GPIO 34 (ESP32)
    └── GND
```

`config.h` の `ESTOP_LOW_ACTIVE` をスイッチ OFF 時の論理に合わせる。
物理スイッチは電気的にモータードライバ電源も遮断するため、ROS2 非依存の停止が同時に効く。
