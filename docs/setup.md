# 初回セットアップ

[← README に戻る](../README.md)

新しい PC / ラズパイ / ESP32 で環境を作るときの手順。上から順に実施する。
一度セットアップ済みの環境での**毎回の起動手順は [README](../README.md#毎回の起動手順) を参照**。

## 目次

1. [PC: WSL2 + Docker Engine](#1-pc-wsl2--docker-engine)
2. [PC: WiFi (ラズパイ AP への接続)](#2-pc-wifi-ラズパイ-ap-への接続)
3. [PC: Windows Firewall](#3-pc-windows-firewall)
4. [PC: 時刻同期](#4-pc-時刻同期)
5. [PC: コンテナのビルド](#5-pc-コンテナのビルド)
6. [ラズパイ: LiDAR 配信](#6-ラズパイ-lidar-配信)
7. [ESP32: ファームウェア書き込み](#7-esp32-ファームウェア書き込み)
8. [Web UI](#8-web-ui)
9. [Linux 実機 (Ubuntu) の場合](#9-linux-実機-ubuntu-の場合)
10. [レガシー環境向け (Docker Desktop 併用)](#10-レガシー環境向け-docker-desktop-併用)

---

## 1. PC: WSL2 + Docker Engine

このリポジトリは Docker Compose 前提。**Docker Desktop ではなくネイティブの Docker Engine を推奨**。
理由: Docker Desktop の内部ネットワークプロキシ経由だと、ESP32 のような外部 LAN デバイスからの
長時間接続が数秒〜十数秒で切れる非対称な切断を実機で確認したため。

### 1-1. `.wslconfig` (ミラーネットワーク + アイドル終了無効化)

`%USERPROFILE%\.wslconfig` を作成(既にあれば追記):

```ini
[wsl2]
networkingMode=mirrored
vmIdleTimeout=2000000000   # ms。-1 は不正値で逆効果なので大きな正値を使う
```

- `networkingMode=mirrored`: WSL2 が Windows と同じ IP を共有し、ESP32/ラズパイから直接届くようになる
  (Windows 11 + 新しめの WSL が必要。`wsl --version` で確認)
- `vmIdleTimeout`: **WSL は対話セッションが無くなると約60秒で VM ごと自動終了**し、
  docker・コンテナ・ROS2 ノードが全て巻き添えで死ぬ(「esp32_bridge が黙って消える」の原因)。
  これを実質無効化する。**Windows 再起動でも安全のため、運用中は WSL ターミナルを1枚開いたままにしておく**とより確実。

反映(他の WSL セッションも全て終了するので注意):

```powershell
wsl --shutdown
```

再度 WSL2 を開き、IP が Windows ホストと同じになっていることを確認:

```bash
ip addr show eth0 | grep 'inet '
```

### 1-2. WSL2 (Ubuntu) 内に Docker Engine をインストール

[9. Linux 実機の場合](#9-linux-実機-ubuntu-の場合) と全く同じ手順を WSL2 の Ubuntu 内で実行する。
WSL2 (Ubuntu) は systemd 対応なので `systemctl` がそのまま使える。`/etc/wsl.conf` に以下があることを確認:

```ini
[boot]
systemd=true
```

Docker Desktop の残骸が干渉する場合はリセット:

```bash
rm -f ~/.docker/config.json
docker context use default
```

### 1-3. 動作確認

```bash
docker version           # Server セクションが出れば OK
docker compose version
```

以降の `docker compose ...` は**この WSL2 (Ubuntu) ターミナルから**実行する
(Windows 側 docker / Docker Desktop は使わない)。

---

## 2. PC: WiFi (ラズパイ AP への接続)

ネットワーク全体像は [network.md](network.md) 参照。**現行はラズパイが AP**
(SSID `th-rpi-ap` / 2.4GHz)、PC と ESP32 がどちらも子機。PC は
**物理の内蔵 WiFi カード**で繋ぐ(USB ドングルは実測で品質が出ない。network.md の対照実験)。

AP 側(ラズパイ)の構築は `th_ws/scripts/rpi_setup_ap.sh`。
WPA2/CCMP 固定・**PMF (802.11w) 無効**が必須(ESP32 が接続要求すら送れなくなる)。

### 2-1. 接続プロファイル (Linux / NetworkManager)

ロボット回線は**固定 IP `192.168.5.50/24`** にする。ESP32 ファームの
`WS_SERVER_HOST` がこの IP 決め打ちのため必須。

```bash
nmcli connection add type wifi ifname wlo1 con-name th-rpi-ap-wlo1 ssid th-rpi-ap
nmcli connection modify th-rpi-ap-wlo1 \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<APパスフレーズ>' \
  ipv4.method manual ipv4.addresses 192.168.5.50/24 \
  ipv4.never-default yes ipv6.method disabled \
  connection.autoconnect yes connection.autoconnect-priority 10 \
  802-11-wireless.powersave 2
nmcli connection up th-rpi-ap-wlo1
```

- `ipv4.never-default yes` — ロボット AP はインターネットを持たないので、
  既定経路を奪わせない。
- `802-11-wireless.powersave 2`(無効) — Ubuntu は
  `/etc/NetworkManager/conf.d/*powersave*` で省電力 ON が既定。有効のままだと
  受信ギャップが伸びる。
- `ifname` は環境で異なる。`nmcli device status` で内蔵カードの名前を確認する。

### 2-2. インターネットは別アダプタ (5GHz) に逃がす

**PC が 2.4GHz を 2 枚同時に使う状態を作らないこと**(機内共存干渉でロボット回線が
劣化する)。インターネットは 5GHz 専用アダプタで別 AP に繋ぎ、既定経路はそちらに置く。

```bash
nmcli device wifi connect '<5GHz の SSID>' password '<パスフレーズ>' ifname wlx........ name net5g
ip route | head -1        # default が 5GHz 側になっていること
```

### 2-3. Windows + WSL2 の PC で動かす場合

物理 WiFi アダプタで `th-rpi-ap` に接続する(モバイルホットスポット等の仮想アダプタは
mirrored networking の対象外なので不可)。管理者 PowerShell で:

```powershell
Set-NetConnectionProfile -InterfaceAlias "<アダプタ名>" -NetworkCategory Private
netsh wlan set profileparameter name=th-rpi-ap connectionmode=auto

# 固定 IP 192.168.5.50 (ESP32 ファームがこの IP に繋ぎに来るため必須)
Get-NetIPAddress -InterfaceAlias "<アダプタ名>" -AddressFamily IPv4 | Remove-NetIPAddress -Confirm:$false
New-NetIPAddress -InterfaceAlias "<アダプタ名>" -IPAddress 192.168.5.50 -PrefixLength 24
```

Windows はインターネットの無い AP 接続中、**約60秒周期のバックグラウンドスキャンで
WiFi を微断**させる。走行時は止める:

```powershell
netsh wlan set autoconfig enabled=no interface="<アダプタ名>"   # 走行前
netsh wlan set autoconfig enabled=yes interface="<アダプタ名>"  # 終了後
```

> **⚠ 重要**: この設定は PC 再起動後も残り、**無効のままだと WiFi の再接続自体ができない**
> (`接続できません。アダプターで WLAN 自動構成が無効になっています`)。
> 必ず「① enabled=yes → ② `th-rpi-ap` に接続 → ③ enabled=no」の順で行うこと。

esp32_bridge は 8766 をコンテナ内で直接待ち受けるため **portproxy は不要かつ有害**
(残っていると Windows が 8766 を横取りする):

```powershell
netsh interface portproxy show all   # 空であること
netsh interface portproxy delete v4tov4 listenport=8766 listenaddress=192.168.5.50
```

---

## 3. PC: Windows Firewall

管理者 PowerShell で以下の受信許可を作成する:

```powershell
# ラズパイからの DDS (/scan 等) 受信許可。IP を変えたらルールも更新すること
New-NetFirewallRule -DisplayName "TH-System Pi LiDAR (inbound UDP)" -Direction Inbound -Action Allow -Protocol UDP -RemoteAddress 192.168.5.1 -Profile Any
New-NetFirewallRule -DisplayName "TH-System Pi LiDAR (inbound TCP)" -Direction Inbound -Action Allow -Protocol TCP -RemoteAddress 192.168.5.1 -Profile Any

# ESP32 からの WebSocket 受信許可
New-NetFirewallRule -DisplayName "TH-System ESP32 WS Bridge" -Direction Inbound -Protocol TCP -LocalPort 8765,8766 -Action Allow -Profile Any
```

---

## 4. PC: 時刻同期

**Windows の時計が実時間から 1 秒近くズレていると、SLAM / AMCL / costmap がラズパイ配信の
/scan を「未来のデータ」として全て破棄し、地図が一切生成されない**(実測で 0.9 秒の遅れを確認)。

```powershell
# 即時同期 (管理者)。w32tm の精度は ±0.5 秒程度しかない点に注意
w32tm /resync /force
# ズレの実測
w32tm /stripchart /computer:time.windows.com /samples:3 /dataonly
# w32tm で直りきらない場合は実測値ぶん直接補正
Set-Date -Date (Get-Date).AddSeconds(0.7)
```

> **【2026-08-20 更新】開発機が Ubuntu になり、以下の W32Time 手順は使えなくなった。**
> 現行の手順は本節末尾の「Ubuntu 開発機での恒久対策」を見ること。W32Time の記述は
> Windows 開発機を使う場合のみ有効な参考情報として残す。

恒久対策(Windows 開発機の場合): AP 配下ではラズパイがインターネットに出られず NTP 同期できないため、
**PC の Windows Time サービス (W32Time) を NTP サーバー化し、ラズパイの systemd-timesyncd をそこに向ける**。

当初は「PC (WSL) 側に chrony サーバーを立てる」案を検討したが、WSL2 の `networkingMode=mirrored`
(1-1 節参照)では **Windows 本体の W32Time が UDP 123 を既に占有しており、WSL 側の chronyd が
`Could not open NTP socket on 0.0.0.0:123` でバインドできず断念した**(mirrored モードは
ポート空間を Windows ホストと共有するため)。ラズパイ側もこの AP 配下ではインターネットに出られず
`apt install chrony` ができない(既定で入っている `systemd-timesyncd` を使う)。

```powershell
# PC: 管理者 PowerShell で W32Time を NTP サーバーとして有効化(一度だけ)
Set-ItemProperty -Path 'HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\NtpServer' -Name Enabled -Value 1
Set-ItemProperty -Path 'HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\Config' -Name AnnounceFlags -Value 5
Restart-Service w32time
w32tm /resync /force   # 自身も time.windows.com 等の外部 NTP に同期させておく
w32tm /query /status   # Source が time.windows.com になっていること
```

ファイアウォールは 3 節の `TH-System Pi LiDAR (inbound UDP)` ルール(192.168.5.1 からの UDP を
ポート指定なしで許可)がそのまま UDP 123 もカバーするため追加設定は不要。

```bash
# ラズパイ側: /etc/systemd/timesyncd.conf
[Time]
NTP=192.168.5.50
FallbackNTP=
RootDistanceMaxSec=30
```

```bash
sudo systemctl restart systemd-timesyncd
sudo systemctl enable systemd-timesyncd
timedatectl status   # "System clock synchronized: yes" になること
```
### Ubuntu 開発機での恒久対策（2026-08-20・現行）

開発機が Ubuntu になったため、上記 W32Time の手順は使えない。**PC 側に `chrony` を立てて配る。**

```bash
# ① PC（一度だけ）
sudo bash th_ws/scripts/pc_setup_ntp_server.sh

# ② ラズパイ（一度だけ）
ssh -t mirs2602@192.168.5.1 'sudo bash /tmp/rpi_fix_clock.sh'   # 先に scp しておく
```

Ubuntu 既定の `systemd-timesyncd` は**クライアント専用で時刻を配れない**。`chrony` を入れると
`systemd-timesyncd` は自動で削除される。`local stratum 10` を入れるので、**PC が上流に
繋がっていなくても配れる**（現場は隔離 AP のため必須）。ラズパイ側の
`/etc/systemd/timesyncd.conf`（`NTP=192.168.5.50`）は上記のままでよい。

なお WSL2 mirrored モードで chronyd が 123/udp にバインドできなかった問題は、
**ネイティブ Ubuntu では起きない**（Windows の W32Time が居ないため）。

#### 時計がずれると DDS ごと壊れる（実機で 2 回遭遇・2026-08-20）

ラズパイには**ハードウェア RTC が無く**、起動のたびに時計が過去へ戻る（実際に 3 日ずれた）。
このとき **DDS のディスカバリ自体が成立しない**。症状は次のとおりで、原因が時刻だと気づきにくい。

- `rplidar` サービスは `active`、ログも正常（health OK / Start）
- しかし **ラズパイ自身で `ros2 node list --no-daemon` を叩いても何も出ない**
- PC ↔ ラズパイの DDS 用 UDP は `tcpdump` で**双方向に流れている**
- それでも参加者としてマッチしない

**時刻を合わせた直後に `/scan` が見えるようになる。**ただし合わせるだけでは足りず、
**既に起動してしまったノードは回復しない**ので、`rplidar.service` に
`After=... time-sync.target` / `Wants=time-sync.target` を入れて順序を強制する
（`rpi_fix_clock.sh` が行う。`systemd-time-wait-sync` は既定で `disabled` だった）。

#### LiDAR の `scan_mode`

> **★ `scan_mode` を変えたら `scan_expected_points` を必ず測り直すこと。**
> 起動時の疎通判定（`DetailedDesign-state.md` §12.2 行 3）は `/scan` の点数の
> **厳密一致**を要求する。合わないと `evt.link_ok` が出ず、**`INIT` から永久に
> 出られない**（WebUI では「全デバイス状態確認中」のまま止まる）。
> **2026-08-21 に実際に踏んだ。**8/20 に DenseBoost → Standard へ替えた際、
> `registry.yaml` の `scan_expected_points`（1080＝DenseBoost の実測値）を
> 直し忘れていた。
>
> ```bash
> # コンテナ内。100 スキャン測って一定かどうかまで見る
> python3 scripts/check_scan_points.py 100
> # → 出た値を registry.yaml の scan_expected_points に入れる
> ```
>
> **2026-08-31 に同じ事故が再発していたことが判明した。**8/20 の
> DenseBoost → Standard 変更に対する `registry.yaml` の追従が結局入っておらず、
> 11 日間 `1080` のまま残っていた（実機は `720`）。実機で `INIT` から出ようと
> した時点で発覚。現在の正しい値は **`720`（Standard・10.01 Hz・0.4993°/点）**。
> **`gazebo_plugins.xacro` の `<samples>` も同じ値に揃えること**——ここが
> ずれると今度は Gazebo 側の `connectivity_checker` が通らなくなる
> （実際に WP-TEST-01 で sim 側を `1080` に合わせてしまい、sim が実機から
> 離れる方向へずれていた）。


稼働中の unit が `ros2 launch rplidar_ros rplidar_s1_launch.py` を叩いていると
**`scan_mode` を指定できない**（この launch ファイルは引数として宣言していない）。
`scripts/rpi_set_scan_mode.sh` が `ExecStart` を `ros2 run` 形式へ差し替える。

`Standard` と `DenseBoost` の実測差は `docs/plan/detailed/data/meas06/README.md`。
**Standard で DR-SPAAM が 3.7 → 6.5 Hz、知覚の遅延が 348 → 179 ms になった。**


`RootDistanceMaxSec` を既定の 5 から 30 に緩めているのは、W32Time が `RootDispersion` を
実測より悲観的に(常に数秒オーダーで)申告する仕様のため、既定値のままだと
`Server has too large root distance. Disconnecting.` で systemd-timesyncd に拒否されるから。
実際の同期精度(Pi/PC 間の実測差)は 1 秒未満(実測 0.3 秒未満、SSH 往復遅延込み)で、
1 節冒頭の 0.9 秒しきい値に対して十分な余裕がある。

既知の注意点: AP 配下の Wi-Fi リンクは瞬断することがあり(2-3 節の autoconfig 問題等)、
timesyncd の単発リトライがその瞬間に当たると `Timed out waiting for reply` になるが、
指数バックオフで自動的にリトライし続けるため実運用上は問題ない。

---

## 5. PC: コンテナのビルド

```bash
# WSL2 (Ubuntu) で、リポジトリの th_ws/ にて
docker compose build        # イメージビルド (初回は DR-SPAAM/torch 取得で時間がかかる)
docker compose up -d        # コンテナ起動 (restart: unless-stopped で常駐)

# コンテナ内で th_* パッケージをビルド (イメージには外部依存のみビルド済み)
docker exec -it th_robot bash
cd /root/th_ws && colcon build --symlink-install
source install/setup.bash
```

> **注意**: `docker compose up -d` でコンテナが**再作成**されると(compose 設定変更時など)、
> コンテナ内でビルドした install/ は消えるため `colcon build` のやり直しが必要。
> 単なる再起動 (`docker restart th_robot`) では消えない。

DR-SPAAM の重みファイル配置(初回のみ):

1. [重みファイルをダウンロード](https://drive.google.com/drive/folders/1Wl2nC8lJ6s9NI1xtWwmxeAUnuxDiiM4W)
   (`ckpt_jrdb_ann_ft_dr_spaam_e20.pth` だけあれば動く)
2. リポジトリの `th_ws/dr_spaam_weights/` に置く(.gitignore 対象)
3. docker-compose の bind mount で自動的にコンテナへ反映される

---

## 6. ラズパイ: LiDAR 配信

RPLIDAR S1 はラズパイに USB 接続し、`rplidar_ros`(`~/ros2_ws`)で `/scan` を配信する。

前提: `ROS_DOMAIN_ID=10`(コンテナと一致)、**`frame_id:=laser_link` 必須**
(既定の `laser` のままだと TF が繋がらず SLAM/Nav2/脚検知が動かない)。

### 6-1. systemd サービス化 (再起動後も自動復帰)

```bash
sudo tee /etc/systemd/system/rplidar.service << 'EOF'
[Unit]
Description=RPLIDAR S1 /scan publisher (TH system, frame_id=laser_link)
After=network.target

[Service]
Type=simple
User=mirs2602
Environment=ROS_DOMAIN_ID=10
ExecStart=/bin/bash -lc "source /opt/ros/humble/setup.bash && source /home/mirs2602/ros2_ws/install/setup.bash && exec ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000 -p frame_id:=laser_link -p angle_compensate:=true -p scan_mode:=Standard"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now rplidar
```

- `scan_mode:=Standard` を推奨: 既定の DenseBoost は点数が多く DR-SPAAM の CPU 推論が
  約2Hz まで落ちて歩行者を見失いやすい。Standard(点数半減)で追跡が安定する。
- **ラズパイ再起動で `/dev/ttyUSB0` ⇄ `/dev/ttyUSB1` が入れ替わることがある**。
  起動失敗(`Error, code: 80008004`)時は `ls /dev/ttyUSB*` でポートを確認して差し替えるか、
  udev ルール(`udev/99-th-robot.rules` 参照)で `/dev/lidar` に固定する。

### 6-2. WiFi (ラズパイを AP にする)

**ラズパイ自身が親機**。`th_ws/scripts/rpi_setup_ap.sh` をラズパイ上で実行する。

```bash
sudo bash rpi_setup_ap.sh th-rpi-ap '<APパスフレーズ>' 1   # SSID / パスフレーズ / ch
sudo bash rpi_setup_ap.sh --revert                        # AP を止める
ip addr show wlan0 | grep 'inet '                         # 192.168.5.1 になっていること
```

> **⚠ 事前に有線の退路を作っておくこと**(無線設定を誤ると機体に触れなくなる)。
> 手順はスクリプト冒頭のコメントに書いてある。
>
> WPA2/CCMP 固定・**PMF (802.11w) 無効**が必須。PMF が有効だと ESP32 (Arduino) は
> 接続要求すら送れず、シールド基板ではシリアルログも読めないため切り分けが困難。
>
> `hostapd` / `dnsmasq` パッケージは不要(NetworkManager の共有モードが
> `dnsmasq-base` を使う)。現場の AP は外に出られず `apt` が使えないので重要な性質。

---

## 7. ESP32: ファームウェア書き込み

手順の詳細・チューニングは [esp32.md](esp32.md) 参照。初回の要点:

1. `esp32/src/wifi_credentials.h.example` を `wifi_credentials.h` にコピーし、
   **STA (子機) 設定**(`WIFI_AP_MODE 0`, `WIFI_SSID "th-rpi-ap"`, `WIFI_PASSWORD`,
   `WS_SERVER_HOST "192.168.5.50"`, `WS_SERVER_PORT 8766`)を記入
2. PC に USB 接続して `cd esp32 && pio run --target upload`
3. シリアルモニタで AP への接続と取得 IP(DHCP。通常 `192.168.5.125`)が出れば起動成功

### Windows での USB シリアル (usbipd — WSL から書き込む場合のみ)

PlatformIO を Windows 側で使うなら不要。WSL 側から書き込む場合は `usbipd-win` で転送:

```powershell
winget install usbipd                                          # 初回のみ
& "$env:ProgramFiles\usbipd-win\usbipd.exe" list               # busid 確認
& "$env:ProgramFiles\usbipd-win\usbipd.exe" bind --busid <id>  # 初回のみ (管理者)
& "$env:ProgramFiles\usbipd-win\usbipd.exe" attach --wsl --busid <id>  # 接続のたび
```

---

## 8. Web UI

```bash
# コンテナの外 (Windows または WSL2) で、初回のみ
cd th_ws/web_ui
npm install

# 起動
npm run dev    # → http://localhost:5173
```

roslib.js はローカル同梱(`web_ui/public/roslib.min.js`)のため、
インターネットの無い AP 配下でもタブレットから動く。
タブレットは `th-rpi-ap` に接続し `http://192.168.5.50:5173` を開く。
rosbridge(9090)は bringup が自動起動する。

### 観客向け表示（デモ展示。[docs/voice-and-audience.md](voice-and-audience.md) §1）

```
http://localhost:5173/?view=audience
```

**この PC 上で localhost で開き、PC の映像出力をディスプレイ/プロジェクタへ回すこと。**
タブレット側の別タブで開いてはいけない —— ブラウザがバックグラウンドタブの
`setInterval` を間引くため手動ジョグの publish が止まり、AP の帯域も倍使う。
localhost で開けば rosbridge 接続も localhost に閉じ、WiFi を一切通らない。

観客画面は読み取り専用で音も鳴らさない。表示レイヤ（地図/点群/検出候補/追跡対象/
経路/走行軌跡）は画面左下の凡例をクリックするか、キー `1`〜`6` で個別に切り替えられる。

---

## 9. Linux 実機 (Ubuntu) の場合

一括セットアップスクリプトがある(udev ルール適用・イメージビルド等をまとめて実行):

```bash
# th_ws/ で
bash setup.sh
```

以下は手動で行う場合の内訳。Docker は[公式ドキュメント](https://docs.docker.com/engine/install/ubuntu/)準拠:

```bash
# 競合する古いパッケージを削除 (なければエラーは無視してよい)
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do
  sudo apt-get remove -y $pkg
done

# 公式 GPG 鍵とリポジトリを追加
sudo apt-get update
sudo apt-get install -y ca-certificates curl
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
sudo usermod -aG docker $USER   # 反映には再ログイン
```

udev ルール(LiDAR を実機に直結する場合):

```bash
lsusb                           # VID:PID を確認
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"
sudo cp udev/99-th-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/lidar               # シンボリックリンク確認
```

---

## 10. レガシー環境向け (Docker Desktop 併用)

**非推奨**(冒頭の理由参照)だが、Docker Desktop の WSL 統合を使い続ける場合の既知の回避策:

- **ネイティブ Engine と併用しない**: 同じディストロで WSL 統合が有効なままだと、
  `network_mode: host` でもコンテナが Docker Desktop 内部ネットワーク(`192.168.65.0/24`)に
  閉じ込められる。Settings → Resources → WSL Integration でトグル OFF →アプリ再起動→
  `rm -f ~/.docker/config.json && docker context use default`。
  確認: `mount | grep docker-desktop` が空であること。
- **`/dev/lidar` がコンテナから見えない**: udev リンクは別ディストロから見えないため、
  `docker-compose.override.yml`(.gitignore 対象)で生デバイス `/dev/ttyUSB0` を直接渡す。
- **RViz2 が起動直後に落ちる**(`libGL error: failed to create drawable`):
  override で `LIBGL_ALWAYS_SOFTWARE=1` を設定しソフトウェアレンダリングに強制する。
  X11 は WSLg 標準搭載のため VcXsrv 等は不要。
