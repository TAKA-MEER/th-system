# ネットワーク構成と復旧手順 (ラズパイ AP)

[← README に戻る](../README.md)

> **2026-09-02 に実機で全面的に確認・書き直した。** それ以前のこのファイルは
> 「ESP32 を AP にする」構成を前提にしていたが、**その構成はもう使っていない。**
> 192.168.4.x の IP・`th-esp32-ap` という SSID が出てくる記述を見かけたら古い。

## 構成の全体像

**ラズパイを WiFi AP にし、PC と ESP32 をそのクライアントとして接続する**構成。

```txt
ラズパイ (WiFi AP + LiDAR 配信)   192.168.5.1    SSID: th-rpi-ap (2.4GHz ch1)
  ├── PC (Ubuntu, ROS2 コンテナ)  192.168.5.50   固定IP (内蔵 Intel カード wlo1)
  │     esp32_bridge が :8766 を待ち受け / rosbridge :9090
  └── ESP32 (駆動用, STA 子機)    192.168.5.125  DHCP
        WS 接続先: 192.168.5.50:8766 (ファームに決め打ち)

PC のインターネットは別系統:
  Elecom WDC-433SU2M2 (5GHz専用)  DHCP          SSID: NCT-WL-ST (5GHz ch36)
        ← 既定経路 (default route) はこちら
```

RPLIDAR S1 → ラズパイの `rplidar_ros` → `/scan` (`ROS_DOMAIN_ID=10`)。
ラズパイ側は systemd で自動起動する。

### なぜこの構成なのか

- **ESP32 は 2.4GHz 専用**なので、ロボット回線は 2.4GHz でしか組めない。
  AP を 5GHz 化することはできない。
- インターネットを 5GHz の別アダプタに逃がすことで、**PC が 2.4GHz を 2 枚同時に
  使う状態（機内共存干渉）を避けている。** これを解消しただけで ESP32 の WS 切断が
  194 秒ごと → 0 回になった。
- `WDC-433SU2M2` は 5GHz 専用（実測: 見える AP 24 件すべて 5GHz、2.4GHz は 0 件）。
  **ロボット回線には使えない。インターネット専用。**

### PC 側の NetworkManager プロファイル（設定済み・autoconnect 有効）

| デバイス | ドライバ | 接続名 | 接続先 | IP |
| --- | --- | --- | --- | --- |
| `wlo1`（内蔵 Intel） | iwlwifi | `th-rpi-ap-wlo1` | `th-rpi-ap` | **固定 192.168.5.50/24**・`never-default`・powersave 無効 |
| `wlx3897a478b19d`（Elecom） | rtl8821au | `net5g` | `NCT-WL-ST` | DHCP・**既定経路** |

- **固定 IP にしている理由**: ESP32 ファームの `WS_SERVER_HOST` が
  `192.168.5.50` 決め打ちだから。DHCP にすると ESP32 が繋ぎに来られなくなる。
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
| WS ハートビート | ESP32 は 10 秒ごとに ping、5 秒以内に pong が来なければミス計上、3 回連続で再接続（`esp32/src/ws_link.cpp`）。**定期的な意図的再接続はもう無い**（旧 5 分周期の仕様は撤去済み） |
| 多重接続 | `esp32_bridge` は新しい WS 接続を受けたら**古い接続を明示的に閉じる**。「接続が 2 本ある」状態にはならない |
| 現在の実測 | 3 分ソークで ESP32 の WS 切断 **0 回**、`ESP32_DISCONNECTED` は起動時の 3 回のみ、`/scan_filtered` **10.07Hz・最大ギャップ 0.12s・標準偏差 0.006s** |
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

### ESP32 が WS に繋がらない（`esp32_bridge` に「接続:」ログが出ない）

1. ESP32 が AP に居るか: ラズパイ側で `ip neigh` / PC 側で `ping -c3 192.168.5.125`
2. **PC 側で 8766 を誰が持っているか**: `ss -tlnp | grep 8766`
   （**古い `esp32_bridge` が生き残っていてポートを奪っている**のが実際にあった。次項参照）
3. 実際に繋ぎに来ている相手を見る: `ss -tnp | grep 8766`
   → ここに出る IP が ESP32。**`/system/trigger` に繋ぎに来る IP と混同しないこと**
4. ESP32 側のファーム設定: `WIFI_AP_MODE 0`（STA）、`WS_SERVER_HOST 192.168.5.50`

> **シリアルモニタの注意**: シリアルポートを開くと DTR/RTS 自動リセット回路で
> **ESP32 が再起動する**。走行中・通信確認中は開かないこと。

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

## 参考

| 知りたいこと | 参照 |
| --- | --- |
| 初回セットアップ | [docs/setup.md](setup.md) |
| ESP32 ファーム・書き込み | [docs/esp32.md](esp32.md) |
| 日々の運用 | [docs/operation.md](operation.md) |
| 開発中に踏んだ環境の癖の一覧 | [CLAUDE.md](../CLAUDE.md)「環境の癖・注意点」 |
