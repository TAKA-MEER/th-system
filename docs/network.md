# ネットワーク構成と復旧手順 (ラズパイ AP)

[← README に戻る](../README.md)

> **2026-09-02 に実機で全面的に確認・書き直した。** それ以前のこのファイルは
> 「ESP32 を AP にする」構成を前提にしていたが、**その構成はもう使っていない。**
> 192.168.4.x の IP・`th-esp32-ap` という SSID が出てくる記述を見かけたら古い。
>
> **2026-09-05 追記: ESP32 は無線を一切使わなくなった。** 第4回3階教示再生走行
> 試験で ESP32 の WiFi 通信エラーが頻発し教示中に衝突を繰り返したため、
> ESP32↔PC 間の無線 WebSocket を廃止し、**ESP32 はラズパイへ USB-UART で直結**、
> ラズパイ上の `pi_serial_relay` が代わりに WiFi 経由で PC の `esp32_bridge` へ
> 接続する構成に変更した（VISION.md「ESP32の無線化をやめ、ラズパイ経由の
> シリアル接続にする」）。これまでの WiFi 対策の積み重ね（本ファイルの以下の
> 記述）にもかかわらず ESP32 の切断はほぼランダムに発生しており、過去の
> 「改善した」という判断は検証run間のブレを誤認していた可能性が高いと判断した。
> **以下の「PC↔ラズパイ」に関する記述は今も有効**（`/scan` に加えてこの区間で
> ESP32 のデータも運ぶようになった）。**「ESP32↔PC」に関する記述（WS 直結・
> `wifi_credentials.h` 等）は歴史的経緯として残すが、もう実体が無い。**

## 構成の全体像

**ラズパイを WiFi AP にし、PC をそのクライアントとして接続する**構成。
ESP32 はラズパイに USB-UART で直結し、無線区間には出てこない。

```txt
ラズパイ (WiFi AP + LiDAR 配信 + pi_serial_relay)  192.168.5.1  SSID: th-rpi-ap (2.4GHz ch1)
  │  USB-UART (/dev/serial/by-id/...)
  ├── ESP32 (駆動用)                                (WiFi 不使用。シリアル直結のみ)
  │
  └── PC (Ubuntu, ROS2 コンテナ)   192.168.5.50   固定IP (内蔵 Intel カード wlo1)
        esp32_bridge が :8766 を待ち受け(pi_serial_relay がここへ接続) / rosbridge :9090

PC のインターネットは別系統:
  Elecom WDC-433SU2M2 (5GHz専用)  DHCP          SSID: NCT-WL-ST (5GHz ch36)
        ← 既定経路 (default route) はこちら
```

RPLIDAR S1 → ラズパイの `rplidar_ros` → `/scan` (`ROS_DOMAIN_ID=10`)。
ラズパイ側は systemd で自動起動する（ESP32 用の `pi_serial_relay` も同様。後述）。

### なぜこの構成なのか

- PC↔ラズパイの WiFi 区間は、以前から `/scan` を実測ロス 0%・RTT 2.2ms で安定
  運用している経路（下記「使ってはいけないアダプタ」参照）。ESP32 のデータも
  この**既に信頼性が確認済みの経路**に相乗りさせることで、ESP32 自身の WiFi
  スタックという不安定要因そのものを消す。
- インターネットを 5GHz の別アダプタに逃がすことで、**PC が 2.4GHz を 2 枚同時に
  使う状態（機内共存干渉）を避けている。**
- `WDC-433SU2M2` は 5GHz 専用（実測: 見える AP 24 件すべて 5GHz、2.4GHz は 0 件）。
  **ロボット回線には使えない。インターネット専用。**

### PC 側の NetworkManager プロファイル（設定済み・autoconnect 有効）

| デバイス | ドライバ | 接続名 | 接続先 | IP |
| --- | --- | --- | --- | --- |
| `wlo1`（内蔵 Intel） | iwlwifi | `th-rpi-ap-wlo1` | `th-rpi-ap` | **固定 192.168.5.50/24**・`never-default`・powersave 無効 |
| `wlx3897a478b19d`（Elecom） | rtl8821au | `net5g` | `NCT-WL-ST` | DHCP・**既定経路** |

- **固定 IP にしている理由**: ラズパイの `pi_serial_relay`（`--ws-host`）が
  `192.168.5.50` 決め打ちだから。DHCP にすると `pi_serial_relay` が繋ぎに
  来られなくなる。
- `never-default` を付けているのは、ロボット AP（インターネットなし）が既定経路を
  奪わないようにするため。
- Ubuntu は `/etc/NetworkManager/conf.d/*powersave*` で `wifi.powersave = 3`（省電力ON）が
  既定。ロボット用接続だけ `802-11-wireless.powersave 2` で無効化してある。

```bash
nmcli -f NAME,DEVICE,STATE connection show --active   # どちらが上がっているか
ip -4 addr show wlo1                                  # 192.168.5.50/24 になっているか
ip route | head -3                                    # default が net5g 側か
```

### 使ってはいけないアダプタ

**AIC8800 系 USB ドングル（`wlx6c1ff789d5d4`）は撤去済み。戻さないこと。**
2026-09-02 の対照実験（同じ AP・同じ ch1・同じ部屋・同じ時刻）:

| 条件 | 上り | 下り | ロス | avg RTT | max RTT |
| --- | --- | --- | --- | --- | --- |
| AIC8800 ドングル単独 | 3.35 Mbps | 6.71 Mbps | 18% | 151 ms | 970 ms |
| **内蔵 Intel `wlo1` 単独** | **28.6 Mbps** | **31.9 Mbps** | **0%** | **2.2 ms** | **11 ms** |

内蔵カードは**混雑した ch1 のまま** ロス 0% / RTT 2.2ms を出す。
→ **ch1 の混雑は律速ではない。チャネル移設（ch6 等）は無意味。**
旧 `th-rpi-ap` プロファイルは撤去したドングルに束縛されたまま残っている（無害・発動しない）。

### 通信の性質（実機検証で確立した前提）

| 事項 | 内容 |
| --- | --- |
| WS 接続 | `pi_serial_relay`（ラズパイ）が `esp32_bridge`（PC）へ接続しに行くクライアント。ESP32 自身は WS を持たない。切断時は `pi_serial_relay` 側が再接続ループする（`th_ws/scripts/pi_serial_relay.py`） |
| シリアル区間の完全性 | ESP32↔ラズパイ間は `serial_framer.py`（sync+len+CRC8 のエンベロープ）で境界と破損検知を行う。ブート時 ASCII バナー等が混ざっても resync して後続フレームを拾う |
| 多重接続 | `esp32_bridge` は新しい WS 接続を受けたら**古い接続を明示的に閉じる**。「接続が 2 本ある」状態にはならない |
| PC↔ラズパイの実測 | 3 分ソークでロス **0%**、`/scan_filtered` **10.07Hz・最大ギャップ 0.12s・標準偏差 0.006s**（2026-09-02。ESP32 のデータもこの経路に相乗りする） |
| DDS ディスカバリ | 現行 AP では**マルチキャストで正常に動く**。ユニキャストピア設定は使わない（後述の落とし穴を参照） |
| `/scan` の QoS | センサストリームなので**必ず `qos_profile_sensor_data`（BEST_EFFORT）で購読する。** RELIABLE で購読すると 1 件も届かない |

---

## 復旧手順（症状別）

### PC が AP に繋がらない / `ping 192.168.5.1` が通らない

```bash
nmcli connection up th-rpi-ap-wlo1
ping -c3 192.168.5.1
```

- デバイスが見えない: `nmcli device status` で `wlo1` が `unavailable` なら
  `nmcli radio wifi on` / `rfkill list`。
- IP が 192.168.5.50 でない: プロファイルが別のもの（旧 `th-rpi-ap` 等）で上がっている。
  `nmcli -f NAME,DEVICE connection show --active` で確認して張り直す。
- **PC が 2.4GHz を 2 枚同時に使う状態にしないこと**（内蔵をモバイルホットスポットに
  するなど）。機内共存干渉でロボット回線が劣化する。

### `pi_serial_relay` が WS に繋がらない（`esp32_bridge` に「接続:」ログが出ない）

1. ラズパイ側で `pi_serial_relay` が動いているか: `systemctl status rpi-serial-relay`
2. ラズパイが PC に到達できるか: `ping -c3 192.168.5.50`
3. **PC 側で 8766 を誰が持っているか**: `ss -tlnp | grep 8766`
   （**古い `esp32_bridge` が生き残っていてポートを奪っている**のが実際にあった。次項参照）
4. 実際に繋ぎに来ている相手を見る: `ss -tnp | grep 8766`
   → ここに出る IP がラズパイ（192.168.5.1）。**`/system/trigger` に繋ぎに来る IP と混同しないこと**
5. `pi_serial_relay` のログ (`journalctl -u rpi-serial-relay -f`) で `--ws-host`/`--ws-port` が
   正しいか確認する

### ESP32 と `pi_serial_relay` の間が繋がらない（`wheel_feedback` が全く来ない）

1. ラズパイで ESP32 が見えているか: `ls -l /dev/serial/by-id/`
   （**RPLIDAR も同じ機構の USB-UART。`/dev/ttyUSB0` のような列挙順依存の指定は
   ある日 LiDAR と入れ替わる。** 必ず `by-id` のシリアル番号込みパスで指定する）
2. `pi_serial_relay` の `--serial-port` がそのパスと一致しているか
3. `journalctl -u rpi-serial-relay -f` で `esp32_bridge へ接続しました` は出ているのに
   フィードバックが来ない場合、ESP32 側が起動していない/焼き込み前の可能性
   （`docs/esp32.md` の書き込み確認バナー参照）

> **シリアルポートを開くと DTR/RTS 自動リセット回路で ESP32 が再起動する。**
> `pi_serial_relay.py` は `open()` 前に DTR/RTS を明示的に落として開くことで
> これを避けている（systemd unit の `ExecStartPre=stty ... -hupcl` も参照）。
> 手元でシリアルモニタ（`pio device monitor` 等）を別途開くと、そのツール自身が
> DTR/RTS を上げて再起動を誘発しうるので、走行中・`pi_serial_relay` 稼働中は
> 開かないこと。

### `/scan` がコンテナに届かない（`LIDAR_LOST` が消えない）

1. ラズパイ側でノードが生きているか: `ssh` して `systemctl status rplidar`、
   ローカルで `ros2 topic hz /scan`
2. **ラズパイのネットワークを切り替えた直後はノード再起動が必要**
   （FastDDS は起動時の IP に固着する）: `sudo systemctl restart rplidar`
3. `ROS_DOMAIN_ID=10` の一致（環境変数はプロセス起動時にのみ読まれる）
4. マルチキャスト疎通: コンテナ側 `ros2 multicast receive` / ラズパイ側 `ros2 multicast send`
5. **時刻ズレ**: `/scan` は届いているのに SLAM が捨てる場合はこれ
   （`Message Filter dropping message` が症状）

### `/scan` は 10Hz 出ているのに `/scan_filtered` が無音

2026-09-01 に踏んだ複合バグ。**点群表示も slam_toolbox の地図生成も動かないのに
`ros2 topic hz /scan` は正常**、という紛らわしい症状になる。原因は 2 つ:

1. **`bringup.launch.py` が `lidar_filter` に渡していた `FASTRTPS_DEFAULT_PROFILES_FILE`
   （`config/fastdds_profile.xml`）のユニキャスト初期ピアが `192.168.4.2` 固定だった。**
   ネットワークが 192.168.5.x へ移ったことで**存在しないサブネット**を指すようになり、
   これが逆にディスカバリを壊していた。→ `additional_env` を外し、マルチキャスト
   ディスカバリに戻した（現行 AP では正常）。別 AP で不安定なら
   `fastdds_profile.xml` の `<address>` を現ラズパイ IP に直して再度渡す。
2. **`lidar_filter` の `/scan` 購読が既定 QoS（RELIABLE）だった。**
   センサストリームは `qos_profile_sensor_data`（BEST_EFFORT）で購読する。

### 全体が不調（CLI が topic を見つけない・ノード間が部分的に不通）

**ノードを `kill -9` で落とすことを繰り返すと、コンテナ内の DDS ディスカバリが壊れる。**
症状は「ノードは起動しログも出ているのに、他プロセスからサービス/トピックが一切
見つからない」。

```bash
docker exec th_robot bash -lc 'ls /dev/shm | wc -l'   # ROS プロセス 0 なのに大量なら該当
docker restart th_robot                               # /dev/shm の掃除だけでは直らないことがある
```

デバッグ用ノードは必ず `kill -TERM` で落とすこと。

---

## プロセスを止めるときの落とし穴（実際に何度も踏んでいる）

- **`pkill -TERM -f "ros2 launch ..."` は launch 親しか殺さず、子ノードは生き残る。**
  「止めたはずなのにポートが埋まっている」「修正したのに古い挙動のまま」はこれ。
  古い `esp32_bridge` が残って 8766 を、rosbridge が 9090 を奪った実例がある。
  `ps -eo pid,args` で ROS 関連を拾って **PID 指定で TERM** すること。
- **`pkill -f <パターン>` は自分のシェルを殺す。** `docker exec th_robot bash -lc '... pkill -f X ...'`
  ではパターンが `-lc` の引数文字列全体にマッチする。ホスト側でも同じ
  （`pkill -f vite` で exit 144 になり後続コマンドが実行されなかった）。**PID 指定で止める。**
- **`th_robot` コンテナはユーザーが実機作業中のセッションであることがある。**
  何かを止める前に `docker exec th_robot ps -eo pid,etimes,args` で稼働中のプロセスを
  確認し、自分が起動したものだけを止めること。

---

## ラズパイ: pi_serial_relay の導入

**まだ実機に一度も導入していない（2026-09-05 時点。ESP32 は導入までの間 PC に
USB 接続してファームを開発する）。** ESP32 をラズパイへ物理的に繋ぎ替えたら:

0. **(必須・最初に1回)** `rplidar.service` が `/dev/ttyUSB0` のような列挙順依存の
   パスのままになっていないか確認する:
   ```bash
   ssh mirs2602@192.168.5.1 'systemctl cat rplidar | grep serial_port'
   ```
   `/dev/ttyUSB0` 等になっていたら、ESP32 を挿した瞬間に列挙順が入れ替わって
   LiDAR と ESP32 を取り違える恐れがある。`docs/setup.md` §6-1 の手順で
   `/dev/serial/by-id/...` を指定する形に直してから先へ進む。
1. ESP32 の USB を PC からラズパイへ挿し替え、by-id パスを確認する:
   ```bash
   ssh mirs2602@192.168.5.1 'ls -l /dev/serial/by-id/'
   ```
   RPLIDAR（既存の CP2102）とは別のエントリが増えているはずなので、それが
   ESP32 のパス（例: `usb-...-if00-port0`）。**このパスを控える。**
2. 中継スクリプトと必要ライブラリをラズパイへ配置する:
   ```bash
   scp th_ws/src/th_esp32_bridge/th_esp32_bridge/serial_framer.py \
       th_ws/scripts/pi_serial_relay.py \
       mirs2602@192.168.5.1:~/
   ssh mirs2602@192.168.5.1 'pip3 install --user pyserial websockets'
   ```
   `mirs2602` が `dialout` グループに入っているか確認する（入っていないと
   `pi_serial_relay.py` の `open()` も unit の `ExecStartPre=stty` も
   permission denied で失敗し、症状は「サービスが起動しない」としか出ない）:
   ```bash
   ssh mirs2602@192.168.5.1 'groups | grep -q dialout && echo OK || \
     (sudo usermod -aG dialout mirs2602 && echo "追加した。再ログインが必要")'
   ```
3. systemd unit を配置する（`th_ws/scripts/rpi-serial-relay.service` をコピーし、
   `<SERIAL_BY_ID_PATH>` を手順1のパスに書き換えてから転送・有効化）:
   ```bash
   scp th_ws/scripts/rpi-serial-relay.service mirs2602@192.168.5.1:/tmp/
   ssh mirs2602@192.168.5.1 '
     sudo sed -i "s#<SERIAL_BY_ID_PATH>#実際のパス#g" /tmp/rpi-serial-relay.service
     sudo cp /tmp/rpi-serial-relay.service /etc/systemd/system/
     sudo systemctl daemon-reload
     sudo systemctl enable --now rpi-serial-relay
     systemctl status rpi-serial-relay --no-pager
   '
   ```
4. **導入直後に必ず確認すること（advisor 指摘・2026-09-05）**: `pi_serial_relay`
   の起動・再起動のたびに ESP32 が誤って再リセットされていないか。ESP32 の
   起動バナーは電源投入直後の一度しか出ないはずなので、`journalctl -u
   rpi-serial-relay --since "5 min ago"` を見ながらサービスを
   `sudo systemctl restart rpi-serial-relay` し、その直後の `wheel_feedback` が
   途切れず続く（＝ESP32 が再起動していない）ことを確認する。再起動している
   兆候があれば `stty -F <path> -hupcl` が効いているか、ケーブル/ドライバの
   自動リセット回路の仕様を疑う。
5. `ros2 topic hz /esp32/wheel_feedback`（PC 側コンテナ内）が安定して出続けることを
   確認する。

---

## 参考

| 知りたいこと | 参照 |
| --- | --- |
| 初回セットアップ | [docs/setup.md](setup.md) |
| ESP32 ファーム・書き込み | [docs/esp32.md](esp32.md) |
| 日々の運用 | [docs/operation.md](operation.md) |
| 開発中に踏んだ環境の癖の一覧 | [CLAUDE.md](../CLAUDE.md)「環境の癖・注意点」 |
