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
> 接続する構成に変更した（[Spec.md](plan/spec/Spec.md) §6.1「ESP32 は無線を使わない」）。これまでの WiFi 対策の積み重ね（本ファイルの以下の
> 記述）にもかかわらず ESP32 の切断はほぼランダムに発生しており、過去の
> 「改善した」という判断は検証run間のブレを誤認していた可能性が高いと判断した。
> **以下の「PC↔ラズパイ」に関する記述は今も有効**（`/scan` に加えてこの区間で
> ESP32 のデータも運ぶようになった）。**「ESP32↔PC」に関する記述（WS 直結・
> `wifi_credentials.h` 等）は歴史的経緯として残すが、もう実体が無い。**

## 構成の全体像

**ラズパイを WiFi AP にし、PC をそのクライアントとして接続する**構成。
ESP32（駆動用）はラズパイに USB-UART で直結し、無線区間には出てこない。

> **2026-10-21 から AP は 2 バンド。**ラズパイの内蔵無線（2.4GHz）と、ラズパイに挿した
> Elecom WDC-433SU2M2（5GHz）の 2 つの AP を `br0` で 1 つのネットワーク（192.168.5.0/24）に
> 束ねている。**PC は 5GHz 側、UnitV2（カメラ）と昇降機の ESP32 は 2.4GHz 側**につなぐ。
> IP は以前と同じなので、`pi_serial_relay` も UnitV2・昇降機 ESP32 も設定を変えていない。
> 導入手順は「ラズパイ: 2 バンド AP の導入」。

```txt
ラズパイ (AP + LiDAR 配信 + pi_serial_relay)   br0 = 192.168.5.1/24（DHCP もここ）
  ├── wlan0 (内蔵 brcmfmac)    SSID: th-rpi-ap  2.4GHz ch1・20MHz   … UnitV2・昇降機 ESP32
  ├── wlx3897a478b19d (Elecom) SSID: th-rpi-5g  5GHz ch44+48・40MHz … PC だけ（屋内専用）
  ├── eth0                     10.42.0.2（有線の退路 th-wired。br0 には入れない）
  └── USB-UART ── ESP32 (駆動用)                 (WiFi 不使用。シリアル直結のみ)

PC (Ubuntu, ROS2 コンテナ)   192.168.5.50   固定IP
  屋内: AIC8800 ドングル wlx6c1ff789d5d4 → th-rpi-5g（ロボット専用。挿せば自動でつながる）
  屋外: 内蔵 Intel wlo1 → th-rpi-ap（2.4GHz。ap_band_switch.sh outdoor）
  esp32_bridge が :8766 を待ち受け(pi_serial_relay がここへ接続) / rosbridge :9090

PC のインターネットは別系統（屋内のみ）:
  内蔵 Intel wlo1  DHCP  保存済みの Wi-Fi（NCT-WL-ST など）に自動接続  ← 既定経路
```

RPLIDAR S1 → ラズパイの `rplidar_ros` → `/scan` (`ROS_DOMAIN_ID=10`)。
ラズパイ側は systemd で自動起動する（ESP32 用の `pi_serial_relay` も同様。後述）。

### なぜこの構成なのか

- **UnitV2 の映像は、AP が 1 つだと同じチャネルを 2 回通る**（UnitV2→ラズパイ、ラズパイ→PC）。
  2.4GHz・20MHz・1 ストリームの上限は約 28Mbps なので、映像に使えるのは実効でその半分だった。
  PC を 5GHz に分けると、映像は 2.4GHz と 5GHz を 1 回ずつ通るだけになる。
- UnitV2 と昇降機の ESP32 は 2.4GHz にしか対応していないので、内蔵無線は 2.4GHz のまま残す。
- 2026-10-21 の実測（同じ位置。bringup 停止中）:

  | 区間 | 上り | 下り | 備考 |
  | --- | --- | --- | --- |
  | 旧: PC↔内蔵 2.4GHz | 約 28 Mbps | 約 28 Mbps | RTT 1.5ms |
  | 新: PC↔内蔵 2.4GHz（hostapd） | 約 35 Mbps | 約 44 Mbps | RTT 平均 3.6ms。UnitV2 の映像の上限はここ（UnitV2 自身の無線はこれより遅いかもしれない） |
  | 新: PC↔Elecom 5GHz（40MHz） | **約 95 Mbps** | **約 59 Mbps** | リンク 135Mbit/s |
  | 新: 同上・PC のインターネットに負荷 | 約 77 Mbps | 約 34 Mbps | インターネット側が 5GHz ch36 のとき |
  | **新: PC の AIC8800↔Elecom 5GHz**（いまの屋内構成） | 約 48〜63 Mbps | **約 76 Mbps** | 5 分 ロス 0%・平均 1.2ms・最大 71ms。内蔵 wlo1 は ch36 でインターネット中 |
  | 参考: PC の AIC8800↔内蔵 2.4GHz | 約 9〜10 Mbps | 約 12〜14 Mbps | 2 分 ロス 0.5%・平均 38ms・最大 185ms。**使わない** |

- PC の中で 5GHz の無線が 2 枚並ぶ（ロボット ch44+48・インターネット ch36）ので、インターネットに
  負荷をかけると下りが落ちることがある。それでも旧構成より速い。

### PC 側の NetworkManager プロファイル（設定済み）

**屋内ではロボット回線＝AIC8800、インターネット＝内蔵 wlo1**（2026-10-21）。AIC8800 は 2.4GHz が苦手だが
5GHz では内蔵より遅延が小さかったので、ロボットを使うときに挿すロボット専用のドングルにした。
内蔵 wlo1 は保存済みの Wi-Fi に自由につながってよい（ロボット AP が消えても回線を奪い合わない）。

| デバイス | 接続名 | 接続先 | IP | 自動接続 |
| --- | --- | --- | --- | --- |
| `wlx6c1ff789d5d4`（AIC8800・aic8800 の DKMS） | `th-rpi-5g-aic` | `th-rpi-5g` | **固定 192.168.5.50/24**・`never-default`・powersave 無効 | **有効**（優先度 20・`autoconnect-retries 0`。挿して約 4 秒、AP が戻って約 3 秒でつながる） |
| `wlo1`（内蔵 Intel） | `th-rpi-ap-wlo1` | `th-rpi-ap`（2.4GHz） | 同上 | 無効（屋外モードで `ap_band_switch.sh` が有効にする） |
| `wlo1`（内蔵 Intel） | `th-rpi-5g-wlo1` | `th-rpi-5g` | 同上 | 無効（AIC8800 が無いときの予備。手で `nmcli connection up`） |
| `wlo1`（内蔵 Intel） | `NCT-WL-ST` ほか保存済みの Wi-Fi | — | DHCP・**既定経路** | 有効 |

- **ロボット AP に 2 枚同時につながないこと**（どちらも 192.168.5.50 なので IP が重なる）。切り替えは
  `ap_band_switch.sh` に任せる。AIC8800 に紐づいたインターネット用のプロファイル（`NCT-WL-ST 1`・
  `SPW_X11_mkhtm 1`）と旧 `th-rpi-ap`（AIC8800 に束縛）は `autoconnect no` にした（AIC8800 がロボット AP
  以外を拾わないように。旧 `th-rpi-ap` は wlo1 と同じ .50 を名乗って IP を奪ったことがある）。
- `nmcli device connect wlo1` は `autoconnect no` のロボット用プロファイルを選び直すことがある（IP が重なった）。
  wlo1 をインターネットに戻すときは `nmcli device set wlo1 autoconnect yes` で NM に選ばせる。

- **固定 IP にしている理由**: ラズパイの `pi_serial_relay`（`--ws-host`）が
  `192.168.5.50` 決め打ちだから。DHCP にすると `pi_serial_relay` が繋ぎに
  来られなくなる。
- `never-default` を付けているのは、ロボット AP（インターネットなし）が既定経路を
  奪わないようにするため。
- Ubuntu は `/etc/NetworkManager/conf.d/*powersave*` で `wifi.powersave = 3`（省電力ON）が
  既定。ロボット用接続だけ `802-11-wireless.powersave 2` で無効化してある。

```bash
nmcli -f NAME,DEVICE,STATE connection show --active   # どちらが上がっているか
ip -br addr | grep 192.168.5.50                       # 192.168.5.50 を持つのが 1 枚だけか
ip route | head -3                                    # default がインターネット用ドングル側か
nmcli -t -f DEVICE,CONNECTION dev | grep -E '^wl'     # 屋内: AIC8800 が th-rpi-5g-aic
```

### ロボット回線に使ってはいけないアダプタ

**AIC8800 系 USB ドングル（`wlx6c1ff789d5d4`）を 2.4GHz のロボット回線に使わないこと。**
（5GHz では問題ない。2026-10-21 に測り直して屋内のロボット専用にした。上の表）
2026-09-02 の対照実験（同じ AP・同じ ch1・同じ部屋・同じ時刻）:

| 条件 | 上り | 下り | ロス | avg RTT | max RTT |
| --- | --- | --- | --- | --- | --- |
| AIC8800 ドングル単独 | 3.35 Mbps | 6.71 Mbps | 18% | 151 ms | 970 ms |
| **内蔵 Intel `wlo1` 単独** | **28.6 Mbps** | **31.9 Mbps** | **0%** | **2.2 ms** | **11 ms** |

内蔵カードは**混雑した ch1 のまま** ロス 0% / RTT 2.2ms を出す。
→ **ch1 の混雑は律速ではない。チャネル移設（ch6 等）は無意味。**

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
nmcli connection up th-rpi-5g-aic      # 屋内（AIC8800 を挿す）。無ければ th-rpi-5g-wlo1
ping -c3 192.168.5.1
```

- `th-rpi-5g` が見えない（`nmcli dev wifi list ifname wlx6c1ff789d5d4`）: ラズパイの 5GHz 側が落ちている。
  有線（`ssh mirs2602@10.42.0.2`）で入り、`systemctl status hostapd@wlx3897a478b19d`・
  `lsmod | grep 8821au`・`iw reg get`（`country JP` か）を見る。急ぐなら予備の
  `nmcli connection up th-rpi-ap-wlo1`（2.4GHz。映像の帯域は旧構成と同じに戻る）。
- デバイスが見えない: `nmcli device status` で `wlo1` が `unavailable` なら
  `nmcli radio wifi on` / `rfkill list`。
- IP が 192.168.5.50 でない: プロファイルが別のもの（旧 `th-rpi-ap` 等）で上がっている。
  `nmcli -f NAME,DEVICE connection show --active` で確認して張り直す。
- **PC が 2.4GHz を 2 枚同時に使う状態にしないこと**（内蔵をモバイルホットスポットに
  するなど）。機内共存干渉でロボット回線が劣化する。
- UnitV2・昇降機 ESP32 が繋がらない: 2.4GHz 側。`systemctl status hostapd@wlan0` と
  `sudo iw dev wlan0 station dump`。
- **wlo1 をロボット回線にしているとき（屋外モード・AIC8800 が無いとき）にロボット AP が消えると、
  wlo1 が保存済みの別の Wi-Fi（学校の `NCT-WL-ST` など）へ移り、AP が戻っても戻ってこない**
  （2026-10-21 に 2 回発生）。`nmcli -t -f DEVICE,CONNECTION dev | grep wlo1` で確かめて手で戻す。
  屋内で AIC8800 を使っていればこの問題は起きない（AIC8800 にはロボット用しか自動接続が無い）。
- 2 バンド構成そのものを疑うときは、ラズパイで `sudo bash rpi_dualband_ap_revert.sh` で旧構成
  （NM の `th-rpi-ap` 単独）に戻せる。PC は `th-rpi-ap-wlo1` を `autoconnect yes` に戻す。

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
2. `journalctl -u rpi-serial-relay` の `ESP32 のポート = ...` がそのパスか（ESP32 交換直後は、LiDAR 以外の USB-UART がちょうど 1 つ見えているか）
3. `journalctl -u rpi-serial-relay -f` で `esp32_bridge へ接続しました` は出ているのに
   フィードバックが来ない場合、ESP32 側が起動していない/焼き込み前の可能性
   （`docs/esp32.md` の書き込み確認バナー参照）

> **シリアルポートを開くと DTR/RTS 自動リセット回路で ESP32 が再起動する。**
> `pi_serial_relay.py` は `open()` 前に DTR/RTS を明示的に落として開くことで
> これを避けている（ラッパー `rpi_serial_relay_run.sh` の `stty -hupcl` も参照）。
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

## ラズパイ: rplidar の自動復帰の導入（USB 抜き差し対策）

実機 2026-10-13: LiDAR の USB を抜いて挿し直すと `/scan` が戻らず `LIDAR_LOST` が 982 秒続いた
（`rplidar_node` が抜く前のデバイスを握ったまま、エラーも終了もしないため `Restart=always` が働かない）。
`th_ws/scripts/rpi_rplidar_run.sh` が by-id のリンクを 1 秒ごとに見張り、変わったら `rplidar_node` を
止めて終了する → systemd が立て直す。**挿し直しから約 10 秒以内に `/scan` が戻る**（Spec-safety §3.5.6）。
**導入は人が行う**（ラズパイは非対話 ssh の `sudo` が使えない）。

1. スクリプトとユニットをラズパイへ送る（PC 側）:
   ```bash
   scp th_ws/scripts/rpi_rplidar_run.sh th_ws/scripts/rplidar.service mirs2602@192.168.5.1:~/
   ```
2. ラズパイにログインして（`ssh mirs2602@192.168.5.1`）、現行ユニットを控えてから入れ替える:
   ```bash
   chmod +x ~/rpi_rplidar_run.sh
   systemctl cat rplidar > ~/rplidar.service.bak      # 戻す用。Environment 行が新ユニットと同じか見比べる
   sudo cp ~/rplidar.service /etc/systemd/system/rplidar.service
   sudo systemctl daemon-reload && sudo systemctl restart rplidar.service
   ```
3. 確認: ラズパイで `journalctl -u rplidar.service -f` を見ながら LiDAR の USB を抜いて挿す。
   `デバイスが変わった` → `起動:` のログが出て、**約 10 秒以内に** PC 側で `ros2 topic hz /scan` が戻る
   （画面の `LIDAR_LOST` が解消する）。by-id のパスが違う個体は `rpi_rplidar_run.sh` の
   `DEV_DEFAULT`（または unit に `Environment=RPLIDAR_DEVICE=...`）を直す。
4. 戻し方: `sudo cp ~/rplidar.service.bak /etc/systemd/system/rplidar.service`（`systemctl cat` の先頭の
   `# /etc/...` 行は消してから）→ `sudo systemctl daemon-reload && sudo systemctl restart rplidar.service`。
   元の `ExecStart` は `rplidar.service`（リポジトリ）のコメントにも残してある。

注意: `rpi_set_scan_mode.sh`（scan_mode の切り替え）は `ExecStart=` 行を置き換える作りで、この新ユニットに
使うとスクリプト経由でなくなる。scan_mode を変えるときは `rpi_rplidar_run.sh` の既定コマンドを直す。

ESP32 の `pi_serial_relay` は同じ問題を持たない見込み（`serial.SerialException` で終了して systemd に
立て直させる作り。実機での抜き差しは未確認）。

---

## ラズパイ: スピーカー再生の導入

機体（ラズパイ）の 3.5 mm 端子のスピーカーで周囲に報知する（Spec-safety §11）。
PC 側の `speaker_announcer`（bringup で起動）が `/speaker/cue`（2 Hz）で「いま鳴らすもの」
を出し、ラズパイの `rpi_speaker_play.py` が wav をループ再生する。**導入は人が行う**
（ラズパイは非対話 ssh の `sudo` が使えない）。

1. 再生スクリプト・音源・ユニットをラズパイへ送る（PC 側）:
   ```bash
   scp th_ws/scripts/rpi_speaker_play.py th_ws/scripts/rpi_speaker_logic.py \
       th_ws/scripts/rpi_speaker.service mirs2602@192.168.5.1:~/
   ssh mirs2602@192.168.5.1 'mkdir -p ~/speaker_wavs'
   scp th_ws/audio/speaker/*.wav mirs2602@192.168.5.1:~/speaker_wavs/
   ```
2. ラズパイにログインして（`ssh mirs2602@192.168.5.1`）、現行ユニットを控えてから入れ替える:
   ```bash
   chmod +x ~/rpi_speaker_play.py
   paplay ~/speaker_wavs/drive.wav    # 音が出ること（出なければ下の切り分け）
   sudo cp ~/rpi_speaker.service /etc/systemd/system/rpi_speaker.service
   sudo systemctl daemon-reload && sudo systemctl enable --now rpi_speaker.service
   ```
   `FASTRTPS_DEFAULT_PROFILES_FILE` は実機の現行ユニット（`systemctl cat rplidar`）と
   見比べて、違う行があれば現行に合わせる（`rpi_speaker.service` の雛形の注意書き）。
3. 確認: PC 側で `ros2 topic hz /speaker/status` が 1 Hz 出る。画面（S-50 → 一般 →
   「スピーカー」）で「試しに鳴らす」を押して音が出る。走らせると「再生中: drive」になる。
4. 戻し方・止め方: `sudo systemctl disable --now rpi_speaker.service`（走行は止まらない。
   報知は安全装置ではないため）。`paplay` で鳴らないときの切り分け:
   - PulseAudio はデスクトップ自動ログインのセッション（uid 1000）が前提。
     自動ログインが無効・セッションが無いと `paplay` は届かない。
     そのときはユニットの `ExecStart` の `--backend pulse` を `--backend alsa` に変える
     （`aplay`＋`amixer -c 0 set Headphone N%` で鳴らす）。
     `pactl info` でサーバに繋がるかも見られる（ユーザーセッションの PulseAudio が要る）。
   - それでも鳴らなければ 3.5 mm 端子の挿し・`alsamixer` のミュート（`MM`）を見る。

---

## ラズパイ: pi_serial_relay の導入

**2026-09-05 に実機へ導入済み**（ESP32 をラズパイへ挿し替え → ファーム書き込み →
`pi_serial_relay` を systemd 登録 → `/esp32/wheel_feedback` が約100ms±20msで届くことを確認）。
現在の実機の状態:

| 項目 | 値 |
| --- | --- |
| ESP32 のポート | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`（`/dev/ttyUSB1`） |
| RPLIDAR のポート | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_0b443775f827c8419a0592b44475a0a2-if00-port0`（`/dev/ttyUSB0`） |
| サービス | `rpi-serial-relay.service`（`enable` 済み・`Restart=always`） |
| Python ライブラリ | `~/pi_relay_env/site-packages`（`PYTHONPATH` で読ませる。下記の理由で pip 不可） |

> **【解消・2026-09-07】** `rplidar.service` を by-id 指定へ修正済み（上表の値）。
> 修正前は `/dev/ttyUSB0` 直指定のままだった（2026-09-06 時点の第5回試験時も未実施
> だったが、たまたま列挙順が保たれていて問題は出なかった。[教示再生走行試験.md](教示再生走行試験.md) §6.3）。

以下は**新しい機体・新しいラズパイへ導入し直すとき**の手順:

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
2. 中継スクリプトと必要ライブラリをラズパイへ配置する。**このラズパイは
   `ip route` に default gateway が無くインターネットに出られない
   （2026-09-05 確認）ので `pip3` は使えない（そもそも `pip3` コマンド自体が
   入っていない）。開発機（インターネットに出られる側）で wheel を落として
   展開したものを scp で送る:**
   ```bash
   # 開発機側: 対象アーキテクチャ(ラズパイ=aarch64)向けにwheelを取得
   pip3 install --user platformio   # 未導入なら(pio run に必要)。pipも同梱される
   mkdir -p /tmp/pi_relay_wheels && cd /tmp/pi_relay_wheels
   pip3 download --no-deps --platform manylinux2014_aarch64 \
     --python-version 310 --implementation cp --only-binary=:all: \
     pyserial websockets
   mkdir -p site-packages
   for f in *.whl; do python3 -m zipfile -e "$f" site-packages/; done
   tar czf site-packages.tgz -C site-packages .

   # ラズパイへ転送・展開
   scp site-packages.tgz mirs2602@192.168.5.1:~/pi_relay_env.tgz
   ssh mirs2602@192.168.5.1 '
     mkdir -p ~/pi_relay_env/site-packages
     tar xzf ~/pi_relay_env.tgz -C ~/pi_relay_env/site-packages
     PYTHONPATH=~/pi_relay_env/site-packages python3 -c "import serial, websockets; print(\"OK\", serial.VERSION, websockets.__version__)"
   '

   # 中継スクリプト本体
   scp th_ws/src/th_esp32_bridge/th_esp32_bridge/serial_framer.py \
       th_ws/scripts/pi_serial_relay.py \
       mirs2602@192.168.5.1:~/
   ```
   `websockets` は 13 以降で `import websockets` だけでは `websockets.exceptions`
   が生えない（遅延import）。`pi_serial_relay.py` は明示的に
   `import websockets.exceptions` しているので問題ないが、手元で対話的に
   確認する際は注意（2026-09-05 実機で踏んだ）。
   `serial_framer.py` はラズパイへの**ベタコピー**（`th_ws/src/th_esp32_bridge/`
   側の正本を変更したら、その都度この scp をやり直すこと。忘れると片方だけ
   古いままになり、症状は「通信が全滅」という分かりにくい形で出る）。

   `mirs2602` が `dialout` グループに入っているか確認する（入っていないと
   `pi_serial_relay.py` の `open()` もラッパーの `stty` も
   permission denied で失敗し、症状は「サービスが起動しない」としか出ない）:
   ```bash
   ssh mirs2602@192.168.5.1 'groups | grep -q dialout && echo OK || \
     (sudo usermod -aG dialout mirs2602 && echo "追加した。再ログインが必要")'
   ```
3. ラッパーと systemd unit を配置する。**ESP32 の by-id パスは unit に書かない**
   （`rpi_serial_relay_run.sh` が `/dev/serial/by-id/` のうち LiDAR 以外の USB-UART を
   ESP32 として自動で選ぶ。ESP32 を交換しても編集は要らない。LiDAR を交換したときだけ、
   `rplidar.service` と一緒にラッパー内の LiDAR ID を直す）:
   ```bash
   scp th_ws/scripts/rpi_serial_relay_run.sh th_ws/scripts/rpi-serial-relay.service mirs2602@192.168.5.1:/tmp/
   ssh mirs2602@192.168.5.1 '
     cp /tmp/rpi_serial_relay_run.sh ~/ && chmod +x ~/rpi_serial_relay_run.sh
     sudo cp /tmp/rpi-serial-relay.service /etc/systemd/system/
     sudo systemctl daemon-reload
     sudo systemctl enable --now rpi-serial-relay
     systemctl status rpi-serial-relay --no-pager
   '
   ```
   起動ログに `rpi_serial_relay_run: ESP32 のポート = ...` が出る。候補が 0 個なら 60 秒待って
   終了、2 個以上なら取り違え防止のため起動せず候補を列挙して終了する（`Restart=always` で再試行）。
4. **導入直後に必ず確認すること（advisor 指摘・2026-09-05）**: `pi_serial_relay`
   の起動・再起動のたびに ESP32 が誤って再リセットされていないか。ESP32 の
   起動バナーは電源投入直後の一度しか出ないはずなので、`journalctl -u
   rpi-serial-relay --since "5 min ago"` を見ながらサービスを
   `sudo systemctl restart rpi-serial-relay` し、その直後の `wheel_feedback` が
   途切れず続く（＝ESP32 が再起動していない）ことを確認する。再起動している
   兆候があれば `stty -F <path> -hupcl` が効いているか、ケーブル/ドライバの
   自動リセット回路の仕様を疑う。
5. `ros2 topic hz /esp32/wheel_feedback`（PC 側コンテナ内）が安定して出続けることを
   確認する。**約100ms間隔（標準偏差 0.003〜0.01s 程度）で並ぶのが正常。**
   2件がほぼ同時に届いた直後に約200msの空白、というバーストが**毎周期**続く場合は
   異常で、下記のいずれかを疑う。

### 導入時に実際に踏んだこと（2026-09-05）

| 症状 | 原因 | 対処 |
| --- | --- | --- |
| `wheel_feedback` が毎周期バーストする（標準偏差 0.04〜0.08s） | `pi_serial_relay` の `ser.read` が固定50msポーリングになっており、ESP32 の100ms送信周期とビート（うなり）を起こしていた。pyserial の `read(size)` は「timeout の間 size バイト溜まるまで待つ」実装なので、`size > 1` に有限 timeout を付けると必ず timeout の粒度で足止めされる | `read(1, timeout=None)` で1バイト待ち → `in_waiting` 分をまとめて読む2段構えに変更（[`ad3960d`](https://github.com/TAKA-MEER/th-system/commit/ad3960d)）。**教示再生のふらつきの直接原因だった** |
| サービスが起動しない | （旧構成）unit に固定した by-id が ESP32 交換で消えた（2026-10-14）。現行はラッパーが自動選択するので起きない | `journalctl -u rpi-serial-relay` でラッパーの「候補が 0／複数」メッセージを見る。`ls -l /dev/serial/by-id/` で LiDAR 以外が 1 つか確認 |
| `pip3: command not found` | ラズパイに `pip3` が無く、`ip route` に default gateway も無い（インターネット非接続） | 上記の wheel 手配り方式（手順2） |

ESP32 の誤リセット（ラッパーの `stty -hupcl` と `open()` 前の DTR/RTS 落としが
効いているか）は、サービス再起動時に `/safety/fault_lock` が `false` のまま
・`wheel_feedback` が途切れないことで確認した（2026-09-05）。

---

## ラズパイ: 2 バンド AP の導入

2026-10-21 に導入した。構成と理由は冒頭の「構成の全体像」。**作業の前に有線の退路
（PC の USB-LAN ↔ ラズパイ eth0、`ssh mirs2602@10.42.0.2`）をつなぐこと。**無線の設定を
誤ると機体に触れなくなる。ラズパイはインターネットに出られないので、要るものは PC で
取ってきて `scp` する。

1. **USB 無線ドングルの抜き差しはラズパイの電源を切ってから。**動いている最中に挿したら
   USB が固まり、ラズパイごと応答しなくなった（2026-10-21、PLANEX で発生）。
2. **8821au ドライバをビルドする**（Elecom WDC-433SU2M2 は Linux 標準のドライバに無い）。
   PC にある DKMS のソース `/usr/src/rtl8821au-5.12.5.2` を `tar` でラズパイの `~/src/` へ送り、
   `make -j4`（約 8 分。ヘッダ `linux-headers-$(uname -r)` と build-essential はラズパイに入っている）。
   ```bash
   sudo install -D -m 644 8821au.ko /lib/modules/$(uname -r)/updates/8821au.ko && sudo depmod -a
   # /etc/modprobe.d/8821au.conf
   options 8821au rtw_country_code=JP rtw_led_ctrl=1 rtw_power_mgnt=0 rtw_ips_mode=0
   # /etc/modprobe.d/cfg80211.conf
   options cfg80211 ieee80211_regdom=JP
   ```
   **カーネルが変わったらビルドし直す**（`/lib/modules/<版>/updates/` に置いたものは別の版では読まれない）。
3. **`iw` と `hostapd` を入れる**。PC で ports.ubuntu.com の jammy arm64 版を取って `sudo dpkg -i`
   （2026-10-21 は `iw_5.16-1build1`・`hostapd_2.10-6ubuntu2.4`。依存はラズパイに揃っていた）。
4. ラズパイで `sudo bash rpi_setup_dualband_ap.sh`（[`th_ws/scripts/`](../th_ws/scripts/rpi_setup_dualband_ap.sh)）。
   hostapd を AP ごとに `hostapd@wlan0`・`hostapd@wlx3897a478b19d` で起動し、2 つの無線を NM の
   管理から外し、NM のブリッジ `th-br0`（192.168.5.1/24・shared）に束ねる。SSID・パスワード・ch は
   旧 `th-rpi-ap` と同じ。DHCP は昇降機 ESP32（.142）・UnitV2（.129）・PC（.50）を MAC で固定している（`/etc/NetworkManager/dnsmasq-shared.d/th-reserve.conf`）。
   旧 NM の `th-rpi-ap` は `autoconnect no` で残す（戻すときに使う）。
5. PC に `th-rpi-5g-aic`（AIC8800）と予備の `th-rpi-5g-wlo1` を作り、`th-rpi-ap-wlo1` を
   `autoconnect no` にする（上の「PC 側の NetworkManager プロファイル」）。
6. 確かめること: PC（5GHz）から昇降機 ESP32（`ping 192.168.5.142`）が通る（2 つの AP が 1 つの
   ネットワークになっている証拠）／`iw dev wlx3897a478b19d info` が `channel 48 … width: 40 MHz`／
   `iw reg get` が `country JP`／コンテナで `ros2 topic hz /scan` が 10Hz／ラズパイを再起動しても
   全部戻る。戻すときは `sudo bash rpi_dualband_ap_revert.sh`。

### 屋外で使うとき（電波法）

**5GHz の W52（ch36〜48）は、登録局でない限り屋外で使えない**（免許不要の無線 LAN として使えるのは屋内だけ。
W53 も屋内専用。W56 は屋外可だが DFS が必須で、走行中にレーダーを検知すると通信が止まるので使わない）。
屋外では **5GHz の AP を止め、PC も内蔵 2.4GHz の `th-rpi-ap`（ch1）につなぐ**。映像と PC が同じ
チャネルを分け合うので、映像の帯域は 2 バンド化の前と同じに戻る。2.4GHz は屋外でも使える。

```bash
cd th_ws/scripts
bash ap_band_switch.sh outdoor   # PC で実行。ラズパイの sudo パスワードを聞かれる
bash ap_band_switch.sh indoor    # 屋内に戻す
bash ap_band_switch.sh status
```

- ラズパイ側は `~/rpi_ap_band.sh`（`hostapd@wlx3897a478b19d` の enable/disable）。モードはラズパイを
  再起動しても残る。PC 側は、屋外では AIC8800 を外して wlo1 を `th-rpi-ap-wlo1` へ、屋内では
  wlo1 をインターネットに戻して AIC8800 を `th-rpi-5g-aic` へつなぐ（AIC8800 が無ければ wlo1 を 5GHz へ）。
  屋外モードの間は PC のインターネットが無い。
- 切り替え中は PC↔ラズパイの無線が数秒〜1 分切れる。**bringup 中にやらない。**
- Elecom を 2.4GHz の別チャネルの AP にする案は使えない。**WDC-433SU2M2 は 5GHz 専用**で、ドライバは
  2.4GHz 対応と表示し hostapd も `AP-ENABLED` を出すのに、電波が出ない（2026-10-21 に ch11 で確認）。

### 導入時に実際に踏んだこと（2026-10-21）

| 症状 | 原因 | 対処 |
| --- | --- | --- |
| 5GHz の AP が `Failed to start AP functionality` | 規制ドメインが `00`（world）。5GHz で自分から電波を出せない | `cfg80211.conf` で JP（すぐ試すなら `sudo iw reg set JP`） |
| 5GHz の ping が平均 70〜100ms | 8821au ドライバ独自の省電力 | `rtw_power_mgnt=0 rtw_ips_mode=0` |
| NM の AP だと 20MHz にしかならない | NM 1.36 の AP は幅を指定できない | hostapd の `ht_capab=[HT40-]`（ch48 指定で 44+48 の 40MHz。ドライバを読み直すと主チャネルの表示が 44 になることがあるが同じ帯域） |
| 2.4GHz 側に誰も繋がれない（`denied association (code=38)`） | hostapd に `ieee80211w=0` を明示すると brcmfmac のファームウェアが拒否する | 内蔵側の設定からその行を消す（Elecom 側は書いてあっても繋がる） |
| ラズパイ再起動後、PC が 2.4GHz を拾って 5GHz に戻らない | 5GHz の AP の起動が遅い。予備の 2.4GHz が自動接続だった | 予備を `autoconnect no`、5GHz 側を `autoconnect-retries 0` |
| 再起動後に `rplidar.service` が起動しない | `time-sync.target` 待ち。ラズパイの時刻源は PC（`NTP=192.168.5.50`、PC の chrony）で、PC に届くまで待ち続ける | PC が AP につながれば同期して起動する。`timedatectl` の `System clock synchronized` を見る |
| PLANEX GW-USValue-EZ（RTL8188CUS）を 2.4GHz の AP にした | 約 15〜20Mbps しか出ず、カーネルに UBSAN（`rtl8192cu/trx.c:502`）が出た | 不採用。映像の上限が旧構成と変わらない |

---

## 参考

### 周りの Wi-Fi・チャネルの混み具合を調べる

[`th_ws/scripts/wifi_check/README.md`](../th_ws/scripts/wifi_check/README.md) の道具で、周囲の AP とチャネルごとの見え方を確認できます。
会場の下見にも、機体との通信が不安定なときの切り分けにも使えます。
NetworkManager の一覧を表示するもので、チャネル変更や通信速度の測定はしません。測定結果は保存しません。

| 知りたいこと | 参照 |
| --- | --- |
| 初回セットアップ | [docs/setup.md](setup.md) |
| ESP32 ファーム・書き込み | [docs/esp32.md](esp32.md) |
| 日々の運用 | [docs/operation.md](operation.md) |
| 開発中に踏んだ環境の癖の一覧 | [CLAUDE.md](../CLAUDE.md)「環境の癖・注意点」 |
