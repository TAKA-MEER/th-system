# 初回セットアップ

[← README に戻る](../README.md)

新しい PC・ラズパイ・ESP32 で環境を作るときの手順。**上から順に実施する。**
PC は **Ubuntu 22.04 が本線**（2026-08-20 から開発機は Ubuntu）。Windows（WSL2）で
動かす場合は [付録 A](#付録-a-windows--wsl2-の-pc-で動かす場合) を見る。
一度セットアップ済みの環境での**毎回の起動手順は [README](../README.md#毎回の起動手順) と
[使い方.md](使い方.md) を参照**。

> 2026-11-02 に全面的に書き直した。それ以前の版は Windows（WSL2）を本線にし、PC の
> ロボット回線を「内蔵 Wi-Fi で 2.4GHz」としていた。いまは屋内なら AIC8800 ドングルで
> 5GHz につなぐ（[network.md](network.md)）。

## 目次

0. [全体の流れ](#0-全体の流れ)
1. [PC: 前提のソフトを入れる](#1-pc-前提のソフトを入れる)
2. [PC: ロボット回線](#2-pc-ロボット回線)
3. [PC: DR-SPAAM の重みファイル](#3-pc-dr-spaam-の重みファイル)
4. [PC: 時刻同期（ラズパイへ時刻を配る）](#4-pc-時刻同期ラズパイへ時刻を配る)
5. [PC: setup.sh（コンテナ・WebUI のビルド）](#5-pc-setupshコンテナwebui-のビルド)
6. [ラズパイ](#6-ラズパイ)
7. [ESP32: ファームウェア書き込み](#7-esp32-ファームウェア書き込み)
8. [起動と Web UI](#8-起動と-web-ui)
- [付録 A: Windows + WSL2 の PC で動かす場合](#付録-a-windows--wsl2-の-pc-で動かす場合)
- [付録 B: Docker Desktop 併用（非推奨）](#付録-b-docker-desktop-併用非推奨)
- [付録 C: LiDAR を PC に直結する場合の udev ルール](#付録-c-lidar-を-pc-に直結する場合の-udev-ルール)

---

## 0. 全体の流れ

| # | どこで | やること | 自動化 |
| --- | --- | --- | --- |
| 1 | PC | git・Docker・Node.js を入れ、docker グループに入って再ログイン | 手作業 |
| 2 | PC | ロボット回線の NetworkManager プロファイルを作る | 手作業（`setup.sh` は有無を確かめるだけ） |
| 3 | PC | DR-SPAAM の重みファイルを置く | 手作業 |
| 4 | PC | chrony で時刻を配る | `pc_setup_ntp_server.sh`（sudo） |
| 5 | PC | イメージ・ワークスペース・WebUI をビルド | **`th_ws/setup.sh`** |
| 6 | ラズパイ | LiDAR 配信・AP・`pi_serial_relay`・時刻 | スクリプト（sudo・対話セッション） |
| 7 | ESP32 | ファームを書き込み、ラズパイへ差し替え | `pio run --target upload` |
| 8 | PC | `./start.sh` で起動し、画面を開く | `th_ws/start.sh` |

**`setup.sh` は道具そのもの（Docker・Node など）を入れない。**入れたばかりの Ubuntu で
いきなり実行すると 1 段目の `docker compose build` で止まる。1〜4 を先に済ませる。
既にラズパイと ESP32 が組み上がった機体に PC だけ足すなら、6・7 は飛ばしてよい。

---

## 1. PC: 前提のソフトを入れる

インターネットにつながった状態で行う（現場のロボット AP はインターネットに出られない）。

### 1-1. git とリポジトリ

```bash
sudo apt-get update
sudo apt-get install -y git curl ca-certificates
git clone <リポジトリの URL> th-system
```

### 1-2. Docker Engine

**Docker Desktop ではなく Docker Engine を使う**（Docker Desktop の内部ネットワーク経由だと、
外部 LAN の機器からの長い接続が数秒〜十数秒で切れるのを実機で確認した。付録 B）。
手順は[公式ドキュメント](https://docs.docker.com/engine/install/ubuntu/)どおり:

```bash
# 競合する古いパッケージを削除（なければエラーは無視してよい）
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do
  sudo apt-get remove -y $pkg
done

# 公式 GPG 鍵とリポジトリを追加
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Docker Engine 本体
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker

# sudo なしで docker を使う（setup.sh・start.sh は sudo を付けずに呼ぶ）
sudo usermod -aG docker $USER
```

**`usermod` のあとは一度ログアウトしてログインし直す**（反映されるまで `permission denied` になる）。

```bash
docker version           # Server セクションが出れば OK
docker compose version
```

### 1-3. Node.js（18 以上）

WebUI のビルド（`npm ci` → `vite build`）と配信（`vite preview`）に要る。
**Ubuntu 22.04 の apt で入る `nodejs` は v12 で、vite 4 が動かない。**NodeSource から入れる:

```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs
node --version   # v18 以上であること（開発機は v24）
```

### 1-4. PlatformIO（ESP32 を書き込む PC だけ）

```bash
sudo apt-get install -y pipx && pipx ensurepath
pipx install platformio
```

**`pip3 install platformio` は使わない。**ホストの pytest と同じ環境に入ると依存の `anyio` が
pytest のプラグインとして読まれ、テストが全滅する（CLAUDE.md「環境の癖」）。

---

## 2. PC: ロボット回線

全体像・理由・復旧手順は [network.md](network.md)。ここでは新しい PC で作るものだけ書く。

| 場所 | ロボット回線 | インターネット |
| --- | --- | --- |
| **屋内** | AIC8800 USB ドングル → `th-rpi-5g`（5GHz） | 内蔵 `wlo1` → 保存済みの Wi-Fi |
| **屋外** | 内蔵 `wlo1` → `th-rpi-ap`（2.4GHz） | なし |

屋外で 5GHz（W52）は電波法で使えない。切り替えは `th_ws/scripts/ap_band_switch.sh indoor|outdoor`
（[network.md](network.md)「屋外で使うとき」）。

**ロボット回線は固定 IP `192.168.5.50/24`。**ラズパイの `pi_serial_relay` がこの IP に
決め打ちで接続しに来るので、DHCP にしてはいけない。

### 2-1. AIC8800 のドライバ

AIC8800 は Linux 標準のドライバに無い。開発機には DKMS で `aic8800/1.0.0` が入っている
（`dkms status` で確認）。**新しい PC への導入手順はまだ記録が無い。**ドングル付属または
メーカー配布のドライバを DKMS で入れ、入れたら手順をこの節に書き足すこと。
挿したあとの名前は `nmcli device status` で確かめる（開発機では `wlx6c1ff789d5d4`）。

### 2-2. NetworkManager のプロファイル

`<5GHz の IF>`・`<内蔵の IF>` は `nmcli device status` で見た名前に置き換える
（開発機では `wlx6c1ff789d5d4`・`wlo1`）。パスフレーズは 2 つの AP で同じ。

```bash
# 屋内: AIC8800 → th-rpi-5g（挿せば自動でつながる）
nmcli connection add type wifi ifname <5GHz の IF> con-name th-rpi-5g-aic ssid th-rpi-5g
nmcli connection modify th-rpi-5g-aic \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<APパスフレーズ>' \
  ipv4.method manual ipv4.addresses 192.168.5.50/24 \
  ipv4.never-default yes ipv6.method disabled \
  connection.autoconnect yes connection.autoconnect-priority 20 \
  connection.autoconnect-retries 0 \
  802-11-wireless.powersave 2

# 屋外: 内蔵 → th-rpi-ap（ap_band_switch.sh outdoor が上げる。ふだんは自動接続しない）
nmcli connection add type wifi ifname <内蔵の IF> con-name th-rpi-ap-wlo1 ssid th-rpi-ap
nmcli connection modify th-rpi-ap-wlo1 \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<APパスフレーズ>' \
  ipv4.method manual ipv4.addresses 192.168.5.50/24 \
  ipv4.never-default yes ipv6.method disabled \
  connection.autoconnect no \
  802-11-wireless.powersave 2

# 予備: AIC8800 が無いとき、内蔵で 5GHz へ（手で nmcli connection up する）
nmcli connection add type wifi ifname <内蔵の IF> con-name th-rpi-5g-wlo1 ssid th-rpi-5g
nmcli connection modify th-rpi-5g-wlo1 \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<APパスフレーズ>' \
  ipv4.method manual ipv4.addresses 192.168.5.50/24 \
  ipv4.never-default yes ipv6.method disabled \
  connection.autoconnect no \
  802-11-wireless.powersave 2
```

- `ipv4.never-default yes` — ロボット AP はインターネットを持たないので、既定経路を奪わせない。
- `802-11-wireless.powersave 2`（無効）— Ubuntu は省電力 ON が既定で、有効のままだと受信が途切れる。
- **ロボット AP に 2 枚同時につながない**（どちらも 192.168.5.50 なので IP が重なる）。
- AIC8800 に、インターネット用の Wi-Fi のプロファイルを紐づけない（ロボット AP 以外を拾う）。
- **AIC8800 を 2.4GHz のロボット回線に使わない**（ロス 18%・RTT 最大 970ms。network.md「ロボット回線に使ってはいけないアダプタ」）。

確認（ロボットの電源が入っていれば）:

```bash
nmcli -t -f DEVICE,CONNECTION dev | grep -E '^wl'   # 屋内: AIC8800 が th-rpi-5g-aic
ip -br addr | grep 192.168.5.50                     # 192.168.5.50 を持つのが 1 枚だけ
ip route | head -3                                  # default がインターネット側
ping -c 3 192.168.5.1
```

---

## 3. PC: DR-SPAAM の重みファイル

git には入っていない（`.gitignore` 対象）。無いと人物検知が動かない。

1. [重みファイルをダウンロード](https://drive.google.com/drive/folders/1Wl2nC8lJ6s9NI1xtWwmxeAUnuxDiiM4W)
   （`ckpt_jrdb_ann_ft_dr_spaam_e20.pth` だけあれば動く）
2. リポジトリの `th_ws/dr_spaam_weights/` に置く
3. docker-compose の bind mount で自動的にコンテナへ反映される

---

## 4. PC: 時刻同期（ラズパイへ時刻を配る）

**ラズパイには RTC が無く、ロボット AP 配下ではインターネットにも出られない。**ラズパイは
PC（`NTP=192.168.5.50`）から時刻をもらう。PC が配っていないと次のことが起きる:

- ラズパイの時計が過去に戻ったまま（実際に 3 日ずれた）になり、**DDS のディスカバリ自体が成立しない**
  （下の「時計がずれると DDS ごと壊れる」）。
- PC とラズパイの時計が 1 秒近くずれると、SLAM / AMCL / costmap が `/scan` を「未来のデータ」として
  全部捨て、地図ができない（実測で 0.9 秒の遅れを確認）。
- ラズパイの `rplidar.service` は `time-sync.target` を待つので、時刻が届くまで LiDAR が起動しない。

Ubuntu 既定の `systemd-timesyncd` はクライアント専用で時刻を配れない。**PC に `chrony` を入れて配る。**

```bash
# ① PC（一度だけ。インターネットにつながった状態で）
sudo bash th_ws/scripts/pc_setup_ntp_server.sh

# ② ラズパイ（一度だけ。PC で scp しておく）
scp th_ws/scripts/rpi_fix_clock.sh mirs2602@192.168.5.1:/tmp/
ssh -t mirs2602@192.168.5.1 'sudo bash /tmp/rpi_fix_clock.sh'
```

- ① は `chrony` を入れ（`systemd-timesyncd` は自動で外れる）、`192.168.5.0/24` と有線の退路
  `10.42.0.0/24` に配るよう設定する。`local stratum 10` を入れるので、**PC が上流に
  つながっていなくても配れる**（現場は隔離 AP のため必須）。
- ラズパイ側の `/etc/systemd/timesyncd.conf`:

  ```ini
  [Time]
  NTP=192.168.5.50
  FallbackNTP=
  RootDistanceMaxSec=30
  ```

- ② は `systemd-time-wait-sync` を有効にし、`rplidar.service` に `time-sync.target` 待ちを足す。
- 確認: ラズパイで `timedatectl status` が `System clock synchronized: yes`。

### 時計がずれると DDS ごと壊れる（実機で 2 回遭遇・2026-08-20）

症状は次のとおりで、原因が時刻だと気づきにくい。

- `rplidar` サービスは `active`、ログも正常（health OK / Start）
- しかし **ラズパイ自身で `ros2 node list --no-daemon` を叩いても何も出ない**
- PC ↔ ラズパイの DDS 用 UDP は `tcpdump` で**双方向に流れている**
- それでも参加者としてマッチしない

**時刻を合わせた直後に `/scan` が見えるようになる。**ただし合わせるだけでは足りず、
**既に起動してしまったノードは回復しない**ので、`rplidar.service` に
`After=... time-sync.target` / `Wants=time-sync.target` を入れて順序を強制する
（`rpi_fix_clock.sh` が行う。`systemd-time-wait-sync` は既定で `disabled` だった）。

---

## 5. PC: setup.sh（コンテナ・WebUI のビルド）

1〜4 が済んだら、`th_ws/` で一度だけ実行する。何度実行しても壊れない。

```bash
cd th_ws
./setup.sh --dry-run   # 何をするかだけ表示する
./setup.sh
```

`setup.sh` がやること:

1. `docker compose build`（初回は DR-SPAAM / torch の取得で時間がかかる）
2. コンテナ `th_robot` を起動し、中で `colcon build --symlink-install`
3. WebUI の `npm ci` → `npm run build`（本番ビルド `web_ui/dist/`）
4. 実行時データの置き場 `th_ws/data/`（コンテナの `/root/th_data`）を作る
5. PlatformIO の有無を確かめる（入れはしない）
6. ロボット回線のプロファイル（`th-rpi-5g-aic`・`th-rpi-ap-wlo1`）の有無を確かめる（作りはしない）

ラズパイ側の導入はしない（sudo のパスワードが要るため。6 節）。

> **注意**: `docker compose up -d` でコンテナが**作り直される**と（compose の設定を変えたとき等）、
> コンテナ内でビルドした `install/` は消えるので `colcon build --symlink-install` のやり直しが要る
> （`./start.sh --build` でもよい）。`docker restart th_robot` だけなら消えない。

---

## 6. ラズパイ

ラズパイは**インターネットに出られず `pip3` も無い**。要るものは PC で取ってきて `scp` する。
`sudo` はパスワードが要るので、**対話の SSH セッション（`ssh -t`）で行う。**
**無線の設定を触る前に、有線の退路（PC の USB-LAN ↔ ラズパイ eth0、`ssh mirs2602@10.42.0.2`）を
つなぐこと。**誤ると機体に触れなくなる。

| 順 | やること | 手順 |
| --- | --- | --- |
| 6-1 | LiDAR の `/scan` 配信（systemd） | 下の 6-1 |
| 6-2 | AP（2.4GHz）→ 2 バンド化（5GHz を足して `br0` で束ねる） | 下の 6-2 |
| 6-3 | 時刻を PC からもらう | 4 節の ② |
| 6-4 | `pi_serial_relay`（ESP32 ↔ PC の中継） | [network.md](network.md)「ラズパイ: pi_serial_relay の導入」 |
| 6-5 | rplidar の自動復帰（USB 抜き差し対策） | [network.md](network.md)「ラズパイ: rplidar の自動復帰の導入」 |
| 6-6 | スピーカー再生 | [network.md](network.md)「ラズパイ: スピーカー再生の導入」 |

### 6-1. LiDAR 配信（systemd で自動起動）

RPLIDAR S1 はラズパイに USB 接続し、`rplidar_ros`（`~/ros2_ws`）で `/scan` を配信する。
前提: `ROS_DOMAIN_ID=10`（コンテナと一致）、**`frame_id:=laser_link` 必須**
（既定の `laser` のままだと TF がつながらず SLAM / Nav2 / 脚検知が動かない）。

```bash
sudo tee /etc/systemd/system/rplidar.service << 'EOF'
[Unit]
Description=RPLIDAR S1 /scan publisher (TH system, frame_id=laser_link)
After=network.target

[Service]
Type=simple
User=mirs2602
Environment=ROS_DOMAIN_ID=10
ExecStart=/bin/bash -lc "source /opt/ros/humble/setup.bash && source /home/mirs2602/ros2_ws/install/setup.bash && exec ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_0b443775f827c8419a0592b44475a0a2-if00-port0 -p serial_baudrate:=256000 -p frame_id:=laser_link -p angle_compensate:=true -p scan_mode:=Standard"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now rplidar
```

- **`serial_port` は `/dev/ttyUSB0` ではなく `/dev/serial/by-id/...` を指定する。**ESP32 も
  ラズパイの USB-UART に直結しているので、`/dev/ttyUSB0`/`1` は起動のたびに LiDAR と ESP32 が
  入れ替わりうる。上の値はこの個体のパス（2026-09-05 確認）。**別個体・別ケーブルに替えたら
  `ls -l /dev/serial/by-id/` で確かめて置き換える。**
- 時刻待ち（`time-sync.target`）は 4 節の `rpi_fix_clock.sh` が足す。

#### LiDAR の `scan_mode`

`scan_mode:=Standard` を使う。既定の DenseBoost は点数が多く、DR-SPAAM の CPU 推論が落ちて
歩行者を見失いやすい（実測: Standard で DR-SPAAM が 3.7 → 6.5 Hz、知覚の遅延が 348 → 179 ms。
`docs/plan/detailed/data/meas06/README.md`）。

> **★ `scan_mode` を変えたら `scan_expected_points` を必ず測り直すこと。**
> 起動時の疎通判定は `/scan` の点数の**厳密一致**を要求する。合わないと `evt.link_ok` が出ず、
> **`INIT` から永久に出られない**（WebUI では「全デバイス状態確認中」のまま止まる）。
> 2026-08-21 と 08-31 の 2 回、`registry.yaml` の直し忘れで実際に踏んだ。
> 現在の正しい値は **`720`（Standard・10.01 Hz・0.4993°/点）**。
> **`gazebo_plugins.xacro` の `<samples>` も同じ値に揃える**（ずれると Gazebo 側の
> `connectivity_checker` が通らない）。
>
> ```bash
> # コンテナ内。100 スキャン測って一定かどうかまで見る
> python3 scripts/check_scan_points.py 100
> # → 出た値を registry.yaml の scan_expected_points に入れる
> ```

稼働中の unit が `ros2 launch rplidar_ros rplidar_s1_launch.py` を叩いていると `scan_mode` を
指定できない（この launch は引数として宣言していない）。`scripts/rpi_set_scan_mode.sh` が
`ExecStart` を `ros2 run` 形式へ差し替える。

### 6-2. AP（ラズパイを親機にする）

**ラズパイ自身が親機**。いまの実機は 2 バンド（内蔵 2.4GHz `th-rpi-ap`＋USB ドングル 5GHz
`th-rpi-5g` を `br0` で 1 つのネットワーク 192.168.5.0/24 に束ねる）。

1. まず 2.4GHz の AP を作る（`th_ws/scripts/rpi_setup_ap.sh` をラズパイで実行）:

   ```bash
   sudo bash rpi_setup_ap.sh th-rpi-ap '<APパスフレーズ>' 1   # SSID / パスフレーズ / ch
   ip addr show wlan0 | grep 'inet '                         # 192.168.5.1 になっていること
   sudo bash rpi_setup_ap.sh --revert                        # AP を止める
   ```

2. 次に [network.md](network.md)「ラズパイ: 2 バンド AP の導入」で 5GHz を足す
   （8821au ドライバのビルド・規制ドメイン JP・`rpi_setup_dualband_ap.sh`）。

- UnitV2（カメラ）と昇降機の ESP32 は 2.4GHz 専用なので、内蔵無線は 2.4GHz のまま残す。
- hostapd の内蔵側に `ieee80211w=0` を書かない（brcmfmac が接続を status 38 で拒否する）。
- USB 無線ドングルは**ラズパイの電源を切ってから**抜き差しする（動作中に挿したらラズパイごと固まった）。

---

## 7. ESP32: ファームウェア書き込み

手順の詳細・チューニングは [esp32.md](esp32.md)。初回の要点:

1. PC に USB 接続して `cd th_ws/esp32 && pio run --target upload`
   （ESP32 は無線を使わない。WiFi の設定ファイルは不要）
2. シリアルモニタで起動バナー（`TH System ESP32 Firmware (Serial)`）が出れば書き込み成功
3. 書き込みが済んだら、ESP32 の USB ケーブルを **PC からラズパイへ差し替える**
4. ラズパイ側に `pi_serial_relay` を導入する（初回のみ。6-4）

---

## 8. 起動と Web UI

```bash
cd th_ws
./start.sh            # 起動（実機・ロボット回線につないだ状態で）
./start.sh --help     # 引数の一覧
```

操作の詳細は [使い方.md](使い方.md)。WebUI は `start.sh` が `web_ui/dist/` を
`vite preview` で配信する（ポート 5173）ので、`npm run dev` を別に立てる必要は無い。
WebUI を手で作り直すとき（`./start.sh --build` でもよい）:

```bash
cd th_ws/web_ui
npm ci
npm run build   # → dist/ ができる
```

- roslib.js はローカル同梱（`web_ui/public/roslib.min.js`）なので、インターネットの無い AP 配下でも動く。
- タブレットはロボット AP（`th-rpi-ap` か `th-rpi-5g`。`br0` で 1 つのネットワーク）につなぎ、
  `http://192.168.5.50:5173` を開く。rosbridge（9090）は bringup が自動起動する。

### 観客向け表示（デモ展示。[voice-and-audience.md](voice-and-audience.md) §1）

```
http://localhost:5173/?view=audience
```

**この PC 上で localhost で開き、PC の映像出力をディスプレイ / プロジェクタへ回すこと。**
タブレット側の別タブで開いてはいけない —— ブラウザがバックグラウンドタブの
`setInterval` を間引くため手動ジョグの publish が止まり、AP の帯域も倍使う。
localhost で開けば rosbridge 接続も localhost に閉じ、Wi-Fi を一切通らない。

観客画面は読み取り専用で音も鳴らさない。表示レイヤ（地図 / 点群 / 検出候補 / 追跡対象 /
経路 / 走行軌跡）は画面左下の凡例をクリックするか、キー `1`〜`6` で個別に切り替えられる。

---

## 付録 A: Windows + WSL2 の PC で動かす場合

本線ではない（2026-08-20 から開発機は Ubuntu）。Windows で動かすときに、1〜5 節に**足す**もの。
WSL2 で Docker 検証を回すときの癖（`FASTDDS_BUILTIN_TRANSPORTS=SHM` など）は CLAUDE.md「環境の癖」。

### A-1. `.wslconfig`（ミラーネットワーク＋アイドル終了の無効化）

`%USERPROFILE%\.wslconfig` を作る（既にあれば追記）:

```ini
[wsl2]
networkingMode=mirrored
vmIdleTimeout=2000000000   # ms。-1 は不正値で逆効果なので大きな正値を使う
```

- `networkingMode=mirrored`: WSL2 が Windows と同じ IP を共有し、ラズパイから直接届くようになる
  （Windows 11＋新しめの WSL が必要。`wsl --version` で確認）。
- `vmIdleTimeout`: **WSL は対話セッションが無くなると約 60 秒で VM ごと自動終了**し、
  docker・コンテナ・ROS2 ノードが全部巻き添えで止まる。これを実質無効にする。
  運用中は WSL のターミナルを 1 枚開いたままにしておくとより確実。

反映（他の WSL セッションも全部終わるので注意）: `wsl --shutdown`。開き直したら
`ip addr show eth0 | grep 'inet '` で Windows と同じ IP になっていることを確かめる。

### A-2. WSL2（Ubuntu）内の Docker Engine

1-2 節と同じ手順を WSL2 の Ubuntu 内で行う。`/etc/wsl.conf` に次があること（`systemctl` が使える）:

```ini
[boot]
systemd=true
```

Docker Desktop の残骸が干渉するときは `rm -f ~/.docker/config.json && docker context use default`。
以降の `docker compose ...` は **WSL2 の Ubuntu から**実行する（Windows 側の docker は使わない）。

### A-3. Wi-Fi（ロボット AP への接続）

物理の Wi-Fi アダプタで接続する（モバイルホットスポット等の仮想アダプタは mirrored の対象外）。
管理者 PowerShell で:

```powershell
Set-NetConnectionProfile -InterfaceAlias "<アダプタ名>" -NetworkCategory Private
netsh wlan set profileparameter name=th-rpi-ap connectionmode=auto

# 固定 IP 192.168.5.50（ラズパイの pi_serial_relay がこの IP に接続しに来るため必須）
Get-NetIPAddress -InterfaceAlias "<アダプタ名>" -AddressFamily IPv4 | Remove-NetIPAddress -Confirm:$false
New-NetIPAddress -InterfaceAlias "<アダプタ名>" -IPAddress 192.168.5.50 -PrefixLength 24
```

Windows はインターネットの無い AP に接続中、**約 60 秒周期のバックグラウンドスキャンで
Wi-Fi を微断**させる。走行時は止める:

```powershell
netsh wlan set autoconfig enabled=no interface="<アダプタ名>"   # 走行前
netsh wlan set autoconfig enabled=yes interface="<アダプタ名>"  # 終了後
```

> **⚠** この設定は再起動後も残り、**無効のままだと Wi-Fi の再接続自体ができない**。
> 必ず「① enabled=yes → ② ロボット AP に接続 → ③ enabled=no」の順で行う。

`esp32_bridge` は 8766 をコンテナ内で直接待ち受けるので、**portproxy は不要かつ有害**
（残っていると Windows が 8766 を横取りする）:

```powershell
netsh interface portproxy show all   # 空であること
netsh interface portproxy delete v4tov4 listenport=8766 listenaddress=192.168.5.50
```

### A-4. Windows Firewall

管理者 PowerShell で受信許可を作る:

```powershell
# ラズパイからの DDS（/scan 等）と NTP（UDP 123）
New-NetFirewallRule -DisplayName "TH-System Pi (inbound UDP)" -Direction Inbound -Action Allow -Protocol UDP -RemoteAddress 192.168.5.1 -Profile Any
New-NetFirewallRule -DisplayName "TH-System Pi (inbound TCP)" -Direction Inbound -Action Allow -Protocol TCP -RemoteAddress 192.168.5.1 -Profile Any
# pi_serial_relay → esp32_bridge の WebSocket
New-NetFirewallRule -DisplayName "TH-System esp32_bridge WS" -Direction Inbound -Protocol TCP -LocalPort 8766 -Action Allow -Profile Any
```

### A-5. 時刻同期（W32Time を NTP サーバーにする）

mirrored では Windows の W32Time が UDP 123 を占有しているので、WSL 側の chrony は
`Could not open NTP socket on 0.0.0.0:123` でバインドできない（4 節の chrony は使えない）。
代わりに W32Time に配らせる:

```powershell
# 管理者 PowerShell（一度だけ）
Set-ItemProperty -Path 'HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\NtpServer' -Name Enabled -Value 1
Set-ItemProperty -Path 'HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\Config' -Name AnnounceFlags -Value 5
Restart-Service w32time
w32tm /resync /force   # 自身も外部 NTP に同期させておく
w32tm /query /status   # Source が time.windows.com 等になっていること
```

ラズパイ側の `timesyncd.conf` は 4 節と同じ（`NTP=192.168.5.50`）。`RootDistanceMaxSec=30` に
緩めているのは、W32Time が `RootDispersion` を実測より悲観的に（常に数秒オーダーで）申告し、
既定の 5 のままだと `Server has too large root distance. Disconnecting.` で拒否されるため
（実際の同期精度は 0.3 秒未満）。w32tm の精度は ±0.5 秒程度なので、ずれが残るときは
`w32tm /stripchart /computer:time.windows.com /samples:3 /dataonly` で測り、
`Set-Date -Date (Get-Date).AddSeconds(<ずれ>)` で直接補正する。

### A-6. USB シリアル（WSL から ESP32 を書き込む場合だけ）

PlatformIO を Windows 側で使うなら不要。WSL 側から書き込むなら `usbipd-win` で転送する:

```powershell
winget install usbipd                                          # 初回のみ
& "$env:ProgramFiles\usbipd-win\usbipd.exe" list               # busid 確認
& "$env:ProgramFiles\usbipd-win\usbipd.exe" bind --busid <id>  # 初回のみ（管理者）
& "$env:ProgramFiles\usbipd-win\usbipd.exe" attach --wsl --busid <id>  # 接続のたび
```

---

## 付録 B: Docker Desktop 併用（非推奨）

1-2 節の理由で非推奨だが、Docker Desktop の WSL 統合を使い続ける場合の既知の回避策:

- **ネイティブ Engine と併用しない**: 同じディストロで WSL 統合が有効なままだと、
  `network_mode: host` でもコンテナが Docker Desktop の内部ネットワーク（`192.168.65.0/24`）に
  閉じ込められる。Settings → Resources → WSL Integration でトグル OFF → アプリ再起動 →
  `rm -f ~/.docker/config.json && docker context use default`。
  確認: `mount | grep docker-desktop` が空であること。
- **`/dev/lidar` がコンテナから見えない**: udev リンクは別ディストロから見えないため、
  `docker-compose.override.yml`（.gitignore 対象）で生デバイス `/dev/ttyUSB0` を直接渡す。
- **RViz2 が起動直後に落ちる**（`libGL error: failed to create drawable`）:
  override で `LIBGL_ALWAYS_SOFTWARE=1` を設定しソフトウェアレンダリングに強制する。
  X11 は WSLg 標準搭載のため VcXsrv 等は不要。

---

## 付録 C: LiDAR を PC に直結する場合の udev ルール

いまの構成では LiDAR も ESP32 もラズパイにつなぐので不要（`setup.sh` も入れない）。
PC に直結する構成に戻すときだけ使う:

```bash
lsusb                           # VID:PID を確認
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"
sudo cp th_ws/udev/99-th-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/lidar               # シンボリックリンク確認
```
