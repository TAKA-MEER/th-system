# TH システム — 配電盤上部確認ロボット 移動機構

## 構成概要

| レイヤー | 実装 |
| --- | --- |
| ハードウェア制御 | ESP32 (PlatformIO + Arduino + WebSocket クライアント) |
| ROS2 ブリッジ | `th_esp32_bridge` (Python) |
| 安全管理 | `th_safety` / `twist_mux` (C++) |
| 状態管理 | `th_mode_manager` (C++) |
| 認識 | `th_perception` (Python) |
| 計画・追従 | `th_planning` (Python) |
| ナビゲーション | Nav2 + SLAM Toolbox + robot_localization |
| UI | React + rosbridge WebSocket |

---

## 現行の実機ネットワーク構成（採用構成）

**ESP32 を WiFi AP にし、PC とラズパイをそのクライアントとして接続する**構成を採用している
（経緯・詳細は後述「ESP32 を AP にして PC⇔ラズパイを接続する構成」参照）。

```txt
ESP32 (駆動用, WiFi AP)          192.168.4.1   SSID: th-esp32-ap
  ├── PC (Windows+WSL2, ROS2)   192.168.4.50  (固定IP必須)
  │     ESP32 → WS 接続先: 192.168.4.50:8766 (コンテナ内 esp32_bridge が直接待ち受け)
  └── ラズパイ (LiDAR 配信)      192.168.4.x   (DHCP。AP再起動でIPが変わりうる)
        RPLIDAR S1 を USB 接続し rplidar_ros で /scan を配信
```

- LiDAR は PC ではなく**ラズパイに接続**する。PC 側は `lidar_source:=network` で起動する。
- ラズパイ側の LiDAR 起動コマンド（**`frame_id:=laser_link` が必須**。既定の `laser` のままだと
  TF が繋がらず SLAM/Nav2 がスキャンを使えない）:

  ```bash
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=10
  ros2 launch rplidar_ros rplidar_s1_launch.py frame_id:=laser_link
  ```

- ESP32 ファームウェアは接続 5 分ごとに WebSocket を**意図的に再接続**する
  （「定期リフレッシュ」。TCP の無言死に対する最後の保険。通常の死活検知は
  3 秒周期の ping/pong ハートビートが担う。esp32_bridge のログに周期的な
  接続/切断が出るのは正常）。

---

## 初回セットアップ

```bash
# リポジトリルートで実行
bash setup.sh
```

---

## Docker 環境のセットアップ

このリポジトリは Docker Compose 前提。**Docker Desktop ではなくネイティブの Docker Engine を使うことを推奨する**。理由: Windows で Docker Desktop の内部ネットワークプロキシ経由だと、ESP32 のような外部LANデバイスからコンテナへの長時間接続が数秒〜十数秒で切れる不具合を実機検証で確認した(コンテナ側は切断を検知するが ESP32 側は気づかない非対称な切断で、Docker を介さず直接ホストでWebSocketサーバーを動かすと同じ条件で安定することを確認済み)。ネイティブの Docker Engine(Linux 標準の `iptables` ベースの DNAT)ならこの問題を回避できる。実機(Ubuntu)と開発機(Windows+WSL2)を同じ Docker Engine 構成に揃えることで、環境差分による不具合も減らせる。

### Linux (Ubuntu 実機)

[Docker公式ドキュメント](https://docs.docker.com/engine/install/ubuntu/)に準拠した、公式 apt リポジトリからのインストール:

```bash
# 競合する古いパッケージを削除(なければエラーは無視してよい)
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

# Docker Engine 本体のインストール
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# サービスを有効化・起動
sudo systemctl enable --now docker

# sudo なしで docker コマンドを使えるようにする(反映には一度ログインし直しが必要)
sudo usermod -aG docker $USER
```

### Windows (WSL2)

Docker Desktop は使わず、**WSL2 の Ubuntu ディストリビューション内にネイティブの Docker Engine を直接インストールする**。

#### 1. WSL2 をミラーネットワークモードにする

デフォルトの WSL2 は NAT 接続のため、WSL2 内の IP アドレスが Windows ホストや LAN と別セグメントになり、ESP32 などの外部LANデバイスから到達できない。ミラーモードにすると WSL2 が Windows ホストと同じ IP を共有し、LAN 上に直接乗る(Windows 11 + 比較的新しい WSL バージョンが必要。`wsl --version` で確認)。

`%USERPROFILE%\.wslconfig` を作成(既にある場合は追記):

```ini
[wsl2]
networkingMode=mirrored
```

設定を反映するため WSL を再起動(**他の WSL 端末セッションも全て終了する**ので注意):

```powershell
wsl --shutdown
```

再度 WSL2 を開き、IP が Windows ホストと同じ(LAN の実IPアドレス)になっていることを確認する:

```bash
ip addr show eth0 | grep 'inet '
```

Docker Desktop を併用している場合、`wsl --shutdown` 後に WSL 統合が切れることがある。Docker Desktop アプリを再起動すれば直るが、本手順ではそもそも Docker Desktop 自体を使わないため無視してよい。

#### 2. WSL2 (Ubuntu) 内に Docker Engine をネイティブインストール

WSL2 の Ubuntu ディストリビューション内で、上記「Linux (Ubuntu 実機)」と**全く同じ手順**を実行する(Docker公式リポジトリ経由。Docker Desktop の WSL 統合機能は使わない)。

WSL2 (Ubuntu) は systemd に対応しているため `systemctl` がそのまま使える。`/etc/wsl.conf` に以下が必要(Ubuntu on WSL のデフォルトで有効な場合が多いが、念のため確認する):

```ini
[boot]
systemd=true
```

変更した場合は `wsl --shutdown` してから WSL2 を開き直す。

インストール後、Docker Desktop 由来の設定が残っていると干渉することがあるため、`~/.docker/config.json` をリセットしておく:

```bash
rm -f ~/.docker/config.json
docker context use default
```

#### 3. 動作確認

```bash
docker version   # Server セクションが表示されれば OK (Client だけならデーモンが起動していない)
docker compose version
```

以降は **この WSL2 (Ubuntu) のターミナルから** `docker compose ...` を実行する(Windows 側の `docker` コマンドや Docker Desktop アプリは使わない)。VSCode を使う場合は「Remote - WSL」拡張機能でこの WSL2 ディストリビューションを直接開くと作業しやすい。

### Docker Desktop の WSL 統合が残っている場合の注意

Docker Desktop をこれまで使っていた環境では、ネイティブ Docker Engine をインストールした後も **同じ WSL ディストリビューションで Docker Desktop の WSL 統合が有効なまま** になっていることがある。この状態では `network_mode: host` を指定してもコンテナが Docker Desktop 内部の仮想ネットワーク(`192.168.65.0/24` 等、`services1` という名前の仮想IFが目印)に閉じ込められ、外部LANデバイス(ラズパイ・ESP32等)への到達性が失われる。

以下の手順ですべて解消しておくこと。

1. Docker Desktop の **Settings → Resources → WSL Integration** で、対象ディストリビューション(例: `Ubuntu`)のトグルを **OFF** にする
2. Docker Desktop アプリを完全に終了(タスクトレイから Quit)してから再度起動する(トグル変更の反映にはアプリ再起動が必要)
3. WSL2 側で以下を実行し、Docker Desktop 由来の設定を消す

   ```bash
   rm -f ~/.docker/config.json
   docker context use default
   ```

4. 確認: `/mnt/wsl/docker-desktop/*` のマウントが無いこと、`docker-compose` コマンドが `/mnt/wsl/docker-desktop/...` のシンボリックリンクになっていないこと

   ```bash
   mount | grep docker-desktop   # 何も出なければ OK
   docker run --rm --network host busybox ip addr show   # ホストと同じIPが出れば host networking は正常
   ```

### ラズパイ等ネットワーク経由の LiDAR を使う場合

RPLIDAR をロボット本体ではなく別マシン(ラズパイ等)に接続し、そちらが配信する `/scan` をネットワーク経由で受信する構成にも対応している。**現行の実機構成はこちら**。

```bash
# ローカルの sllidar_node 起動を止め、外部が配信する /scan を使う
ros2 launch th_bringup bringup.launch.py lidar_source:=network
ros2 launch th_bringup slam.launch.py    lidar_source:=network
```

ラズパイ側は `rplidar_ros` パッケージ（`~/ros2_ws`）で配信する。**`frame_id` の既定値は
`laser` のため、必ず `frame_id:=laser_link` を指定すること**（本システムの URDF / TF は
`laser_link` を前提としており、既定のままだと SLAM・Nav2・脚検知がスキャンを座標変換できない）:

```bash
# ラズパイ側
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch rplidar_ros rplidar_s1_launch.py frame_id:=laser_link
```

これが機能するには、以下 3 点がすべて満たされている必要がある(実機検証で判明した詰まりどころ)。

1. **同一LANセグメントに接続すること**: Windows のモバイルホットスポット/ICS 仮想アダプタ(`192.168.137.0/24` 等)は WSL2 の mirrored networking のミラー対象**外**であるため、ラズパイをそこに接続しても WSL2/コンテナ側から到達できない。ラズパイは、WSL2 が実際にミラーしている物理 Wi-Fi/Ethernet アダプタと**同じ**ネットワークに接続すること。

   ```bash
   # WSL2 側で実際にミラーされているセグメントを確認
   ip addr show eth0 | grep 'inet '
   ```

2. **`ROS_DOMAIN_ID` の一致 + プロセス再起動**: ラズパイ側で `ROS_DOMAIN_ID` をコンテナ側(`10`)に合わせること。**環境変数はプロセス起動時にしか読み込まれない**ため、`.bashrc` に追記しただけでは既に起動済みの `/scan` 配信プロセスには反映されない。設定変更後は当該プロセス(またはラズパイ自体)を再起動すること。

3. **Windows Firewall のインバウンド許可**: 接続先 Wi-Fi のネットワークプロファイルが「パブリック」の場合、Windows Firewall はデフォルトで未承諾のインバウンド UDP/TCP をブロックし、DDS のディスカバリ・データ配信が届かない。ラズパイの IP からの着信を許可するルールを追加する(管理者 PowerShell)。

   ```powershell
   New-NetFirewallRule -DisplayName "TH-System Pi LiDAR (inbound UDP)" -Direction Inbound -Action Allow -Protocol UDP -RemoteAddress <ラズパイのIP> -Profile Any
   New-NetFirewallRule -DisplayName "TH-System Pi LiDAR (inbound TCP)" -Direction Inbound -Action Allow -Protocol TCP -RemoteAddress <ラズパイのIP> -Profile Any
   ```

   ラズパイの IP が変わった場合は `RemoteAddress` を更新すること。

動作確認:

```bash
ros2 topic list | grep scan
ros2 topic hz /scan   # 10Hz 前後で来ていれば OK
```

`/scan` が来ない場合は、上記 1〜3 に加えて前節「Docker Desktop の WSL 統合が残っている場合の注意」も確認すること。

---

## ESP32 ファームウェア

### 初回ビルド

```bash
cd esp32
pio run                    # ビルドのみ
pio run --target upload    # ビルド + 書き込み
pio device monitor         # シリアルモニタ (115200 baud)
```

### WiFi 認証情報の設定

1. `esp32/src/wifi_credentials.h.example` を `esp32/src/wifi_credentials.h` としてコピー
2. `WIFI_SSID` / `WIFI_PASSWORD` を実際のホットスポットの値に、`WS_SERVER_HOST` / `WS_SERVER_PORT` を `esp32_bridge` (`config/params.yaml` の `ws_host`/`ws_port`) に合わせて書き換える
3. `pio run` を実行(`wifi_credentials.h` は `.gitignore` 対象のためコミットされない)

### 開発ボード単体での書き込み・通信テスト

モーター・エンコーダ・E-Stopスイッチを未配線のESP32開発ボードだけで、ROS2/colcon/Docker を使わずに「書き込みが成功したか」「WiFi/WebSocket通信ができるか」を確認できる。

```bash
# 1. PC側 (ROS2/Dockerは不要。websockets だけ pip でインストール)
pip install websockets
python th_ws/esp32/tools/ws_test_server.py --send-test-cmd

# 2. ESP32側
cd th_ws/esp32
pio run --target upload
pio device monitor
```

- **書き込み確認**: シリアルモニタに起動バナー(`TH System ESP32 Firmware (WebSocket)` とビルド日時)が出れば書き込み成功。
- **通信確認**: `[WiFi] 接続しました IP=...` → `[WS] esp32_bridge に接続しました` の順にログが出て、`ws_test_server.py` 側にも `[接続] ESP32 接続: ...` と `[受信] ESTOP_HW ...` が周期的に表示されれば ESP32→PC 方向の通信は正常。`--send-test-cmd` を付けているとPC側が2秒おきにテスト用の `WHEEL_CMD` を送信するので、ESP32シリアルモニタに `[WHEEL_CMD] left=... right=...` が出れば PC→ESP32 方向も確認できる(モーター未配線でも安全)。
- **注意**: `config.h` の `ESTOP_BENCH_TEST_BYPASS` は現在無効化されているため、E-Stopスイッチ未配線だと GPIO34 がフローティングになり `ESTOP_HW` の値が不安定になりうる(`WHEEL_FEEDBACK` は E-Stop 中もウォッチドッグ作動中も毎周期送信される)。これは既知の挙動で、通信テスト自体には影響しない。動力系を配線せず確実に安定した挙動を見たい場合のみ、一時的に `ESTOP_BENCH_TEST_BYPASS` を再定義して試験し、試験後は必ず戻すこと。

### E-Stop 配線

```tree
物理スイッチ (モーター電断端子)
    └── GPIO 34 (ESP32)
    └── GND
```

`config.h` の `ESTOP_LOW_ACTIVE` をスイッチ OFF 時の論理に合わせて設定すること。

---

## ESP32 を AP にして PC⇔ラズパイを接続する構成

ラズパイ自身を WiFi AP にする構成(前節「ラズパイ等ネットワーク経由の LiDAR を使う場合」)では、
AP自身が生成した通信が PC 側に届かないという原因不明の非対称なネットワーク障害が実機で発生した
ことがある。これを回避する代替構成として、**駆動用ESP32(または代替のESP32開発ボード)を WiFi AP にし、
PC とラズパイの両方をそのAPのクライアントとして接続する**方法がある。両者が対等なステーションになるため、
AP自身が発信元になる非対称性の問題が起きにくい。

### 1. ESP32ファームウェアをAPモードで書き込む

`th_ws/esp32/src/wifi_credentials.h` に以下を追加(既存のSTA用定義は削除しない。`WIFI_AP_MODE` で切替):

```cpp
#define WIFI_AP_MODE      1
#define AP_SSID           "th-esp32-ap"
#define AP_PASSWORD       "<APパスワード>"   // WPA2は8文字以上必須。実際のパスワードに置き換えること

// esp32_bridge (PC側) がAPネットワーク上で待ち受けるIP。
// PC側の固定IP設定 (下記3.) に合わせること。
#define WS_SERVER_HOST    "192.168.4.50"
#define WS_SERVER_PORT    8766
```

```bash
cd th_ws/esp32
pio run --target upload
pio device monitor   # [WiFi] AP IP=192.168.4.1 が出れば起動成功
```

### 2. PC を ESP32 の AP に接続する

物理WiFiアダプタ(モバイルホットスポット等の仮想アダプタは不可。WSL2 mirrored networking が
ミラーするのは物理アダプタのみ)で `th-esp32-ap` に接続し、ネットワークプロファイルを
「プライベート」に設定する。

```powershell
Set-NetConnectionProfile -InterfaceAlias "<アダプタ名>" -NetworkCategory Private
```

さらに WiFi プロファイルを**自動接続**に設定すること(既定で「手動接続」になっていると、
瞬断後に Windows が再接続せず、ESP32 側から見て「クライアント数=0」のまま
WebSocket が数分間つながらない事象が実機で発生した):

```powershell
netsh wlan set profileparameter name=th-esp32-ap connectionmode=auto
```

切り分けのヒント: ESP32 のシリアルログに 5 秒ごとに `[WiFi-AP] 接続クライアント数=N` が
出る。これが 0 に落ちる場合は ESP32 側ではなく **PC/ラズパイ側の WiFi が切れている**。

**既知の問題**: Windows はインターネットの無い AP に接続中、約60秒周期で
バックグラウンドスキャンを行い WiFi が微断する（→ WS が切断され再接続に数十秒かかる）
ことを実機で確認した。運用時は管理者 PowerShell で該当アダプタのスキャンを止める:

```powershell
netsh wlan set autoconfig enabled=no interface="<アダプタ名>"   # 走行前
netsh wlan set autoconfig enabled=yes interface="<アダプタ名>"  # 終了後
```

**シリアルモニタの注意**: シリアルポートを開くと DTR/RTS の自動リセット回路により
**ESP32 が再起動する**（AP も一瞬落ちる）。走行中にシリアルモニタを接続しないこと。

### 3. PC に固定IPを設定する(重要)

ESP32のSoftAP DHCPは**再起動のたびにリース状態がリセットされ、PCに割り当てるIPが変わりうる**
(`.2` → `.4` など)。ESP32を再書き込みするたびにportproxy設定やIPが食い違って通信不能になるため、
必ず固定IPを設定すること(管理者権限が必要)。

```powershell
Get-NetIPAddress -InterfaceAlias "<アダプタ名>" -AddressFamily IPv4 | Remove-NetIPAddress -Confirm:$false
New-NetIPAddress -InterfaceAlias "<アダプタ名>" -IPAddress 192.168.4.50 -PrefixLength 24 -DefaultGateway 192.168.4.1
```

**注意**: `Remove-NetIPAddress`/`New-NetIPAddress` の実行直後にWiFi接続自体が切断されることがある。
`netsh wlan show interfaces` で `State: connected` に戻っているか必ず確認し、切れていたら
`netsh wlan connect name="th-esp32-ap" interface="<アダプタ名>"` で再接続すること。

### 4. Windows Firewall の設定

- 上記2.で「プライベート」プロファイルに設定済みでも、**既存の `python.exe` 受信許可ルールが
  「パブリック」プロファイル限定になっている場合がある**(Windowsが過去に別のネットワークで
  自動作成したルールが残っているケース)。`Get-NetFirewallRule -DisplayName "python.exe"` で
  `Profile` を確認し、Private が含まれていなければ以下のように専用ルールを追加する。

```powershell
New-NetFirewallRule -DisplayName "TH-System ESP32 WS Bridge" -Direction Inbound -Protocol TCP -LocalPort 8765,8766 -Action Allow -Profile Any
```

### 5. WebSocket 待ち受けポート (portproxy は不要)

`esp32_bridge` は WSL2/Docker内で `0.0.0.0:8766` を直接待ち受ける
(`th_esp32_bridge/config/params.yaml` の `ws_port: 8766`)。WSL2 mirrored networking で
外部デバイス (ESP32) からコンテナ内のこのポートへ直接届くことを実機検証で確認済み。

**かつては `netsh portproxy` で 8766→8765 の転送を使っていたが廃止した。**
古い portproxy エントリが残っていると Windows 側が 8766 を横取りして通信不能になるため、
必ず削除すること(管理者 PowerShell):

```powershell
netsh interface portproxy show all   # エントリが残っていないか確認
netsh interface portproxy delete v4tov4 listenport=8766 listenaddress=192.168.4.50
```

補足: portproxy のリスナーは Windows 再起動や IP 再設定で無言で機能停止することが
実機で確認されており(エントリ表示は残るのに LISTEN しない)、恒久構成には不向き。

### 6. ラズパイをESP32のAPに接続する

ラズパイ側で通常のWiFi子機としてSSID `th-esp32-ap` に接続する(NetworkManager等でPiが
物理アクセス可能な状態から操作すること。SSH接続はAP切替直後に一旦切れるため、Piから新しい
IPアドレスを確認して伝えてもらい、そのIPで `ssh` し直す)。

```bash
ip addr show wlan0 | grep 'inet '   # ESP32のAP経由でDHCP取得したIPを確認
```

### 7. `/scan` の受信確認 (README 143〜181行目の3条件を適用)

ラズパイ側で `ROS_DOMAIN_ID` をコンテナ側 (`10`) に一致させて `rplidar_node` を起動し、
WSL2/コンテナ側で `ros2 topic hz /scan` が10Hz前後で来ることを確認する。手順は本README
「ラズパイ等ネットワーク経由の LiDAR を使う場合」節と同じ(同一LANセグメント・
`ROS_DOMAIN_ID`一致・Windows Firewall許可の3条件)。

### 8. `esp32_bridge` との通信確認

```bash
# コンテナ内
ros2 run th_esp32_bridge esp32_bridge.py --ros-args --params-file src/th_esp32_bridge/config/params.yaml
ros2 topic hz /safety/estop_hw   # WSクライアントが接続していれば10Hz前後で届く
```

`/esp32/wheel_feedback` は `/cmd_vel` の有無に関わらず毎周期(10 Hz)送信される
(かつては停止中に送信を止めていたが、待機中に `safety_monitor` が
`ESP32_DISCONNECTED` を誤検知し odom/TF も途絶するため、常時送信に修正した)。
なお ESP32 は 5 分ごとに WebSocket を定期リフレッシュするため、
接続/切断ログが周期的に出るのは正常。

---

## udev デバイスパスの確認と設定

### Linux（実機環境）

```bash
# デバイスを接続してから
lsusb                           # VID:PID を確認
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"

# udev/99-th-robot.rules を編集後
sudo cp udev/99-th-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 確認 (ESP32 は WebSocket 通信になったため /dev/esp32 のパススルーは不要)
ls -la /dev/lidar
```

### Windows (WSL2) での USB デバイス転送（ESP32 ファームウェア書き込み用）

ESP32 との実行時通信は WebSocket（WiFi 経由）のため、Docker コンテナに USB デバイスを渡す必要はない。ここで転送するのは `pio run --target upload` / `pio device monitor`（**WSL2 ホスト側**、コンテナの外で実行）のためだけ。WSL2 は Windows の USB デバイスを自動で引き継がないため、`usbipd-win` でブリッジが必要。

#### 初回セットアップ（1 回のみ）

```powershell
# 1. usbipd-win をインストール（管理者 PowerShell）
winget install usbipd

# 2. デバイス一覧を確認（busid をメモ）
& "$env:ProgramFiles\usbipd-win\usbipd.exe" list
# → "Silicon Labs CP210x USB to UART Bridge" の BUSID（例: 2-1）を確認

# 3. bind（管理者として 1 回のみ実行）
& "$env:ProgramFiles\usbipd-win\usbipd.exe" bind --busid 2-1
```

WSL2 内で udev ルールを作成（1 回のみ、`/dev/esp32` という分かりやすい名前で参照できるようにするだけで必須ではない）:

```bash
# /dev/esp32 シンボリックリンクを自動作成するルール
printf 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="esp32", MODE="0666"\n' \
  | sudo tee /etc/udev/rules.d/99-esp32.rules
sudo chmod 644 /etc/udev/rules.d/99-esp32.rules
sudo udevadm control --reload-rules
```

#### 毎回の手順（ESP32 を接続・再起動後）

```powershell
# 管理者 PowerShell で 1 行のみ
& "$env:ProgramFiles\usbipd-win\usbipd.exe" attach --wsl --busid 2-1
```

この `attach` は `pio run --target upload` / `pio device monitor` を実行するときだけ必要。`docker compose run --rm th_robot bash` でのコンテナ起動やロボットの通常運用には不要（ESP32 はコンテナの外、WSL2 ホスト側から直接ファームウェアを書き込む）。

| タイミング | 必要な操作 |
| --- | --- |
| Windows 再起動後 | `attach` のみ（`bind` は保持される） |
| ESP32 抜き差し後 | `attach` のみ |
| WSL2 再起動後 | `attach` のみ |

---

### Windows (WSL2) + Docker Desktop での LiDAR 接続と RViz2 表示

推奨は本書冒頭の「ネイティブ Docker Engine」構成だが、**Docker Desktop の WSL 統合を
使い続ける場合**（切り替えが手間、または既存環境を壊したくない場合）に実機検証で
判明した手順・回避策をまとめる。

#### 1. LiDAR を WSL2 にアタッチ（ESP32 と同じ usbipd 手順）

```powershell
# デバイス一覧確認（管理者不要）
& "$env:ProgramFiles\usbipd-win\usbipd.exe" list
# → "Silicon Labs CP210x USB to UART Bridge" の busid を確認 (例: 2-1)

# 管理者 PowerShell で bind は初回のみ、attach は接続/再起動のたびに必要
& "$env:ProgramFiles\usbipd-win\usbipd.exe" bind --busid <busid>
& "$env:ProgramFiles\usbipd-win\usbipd.exe" attach --wsl --busid <busid>
```

WSL2 (Ubuntu) 側で `/dev/lidar`（99-th-robot.rules の udev シンボリックリンク）が
見えれば OK:

```bash
wsl -d Ubuntu -- ls -la /dev/lidar /dev/ttyUSB0
```

#### 2. Docker Desktop 特有の問題: `/dev/lidar` がコンテナから見えない

Docker Desktop の WSL 統合はコンテナを別の WSL ディストロ（`docker-desktop`）内で
実行するため、`Ubuntu` ディストロの udev が作った `/dev/lidar` シンボリックリンクは
コンテナから見えない（生デバイス `/dev/ttyUSB0` 自体は WSL2 が全ディストロで共有する
カーネルの devtmpfs に載るため見える）。`th_ws/docker-compose.override.yml`
（`.gitignore` 対象・個人環境向け）で生デバイスを直接渡すことで回避する:

```yaml
# th_ws/docker-compose.override.yml
services:
  th_robot:
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    environment:
      - LIBGL_ALWAYS_SOFTWARE=1   # 次項参照
```

ノード起動時は `serial_port:=/dev/ttyUSB0`（`/dev/lidar` ではなく）を指定する。
ネイティブ Docker Engine 構成に切り替えればこの補正は不要になる。

#### 3. RViz2 がクラッシュする場合（`libGL error: failed to create drawable`）

Docker Desktop の WSL 統合下では、rviz2 がハードウェア OpenGL 経由で描画しようとすると
`libGL error: failed to create drawable` を出して**起動直後にプロセスごと終了**する
（ウィンドウが一瞬出て消える、または全く出ない）。`LIBGL_ALWAYS_SOFTWARE=1` で
ソフトウェアレンダリング（llvmpipe）に強制すると安定する（上記 override に記載済み）。
Gazebo は逆にハードウェア支援があった方が快適なため、この設定は
`docker-compose.yml` 本体には入れず override 限定にしている。

X11 表示自体は **VcXsrv 等の追加インストールは不要**。最近の WSL は WSLg
（`DISPLAY=:0`、`/tmp/.X11-unix/X0`）を標準搭載しており、`docker-compose.yml` の
既存の `DISPLAY` 環境変数 + `/tmp/.X11-unix` bind mount だけで GUI がホストの
Windows デスクトップにそのまま表示される。

#### 4. person_tracker（脚検知）だけを素早く動作確認する

`bringup.launch.py` はNav2やESP32ブリッジ等も一括起動するため、脚検知パイプラインだけを
素早く確認したい場合は以下を個別に起動する（コンテナ内、複数ターミナル/バックグラウンド）:

```bash
source /opt/ros/humble/setup.bash && source /root/th_ws/install/setup.bash

# 1. LiDAR
ros2 run sllidar_ros2 sllidar_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000 \
  -p frame_id:=laser_link -p angle_compensate:=true -p scan_mode:=Standard &

# 2. 死角フィルタ
ros2 run th_perception lidar_filter.py &

# 3. DR-SPAAM + PersonTracker（単一LiDARなのでTF不要、target_frame=laser_link）
ros2 launch leg_detection_bringup leg_detection.launch.py \
  scan_topic:=/scan_filtered target_frame:=laser_link \
  scan_frame:=laser_link odom_frame:=laser_link \
  use_rviz:=false autostart:=true &

# 4. /person/status へのブリッジ
ros2 run th_perception person_tracker_bridge.py &

# 5. 確認
ros2 topic hz /dr_spaam/dr_spaam_detections
ros2 topic echo sobits_follower/multiple_sensor_person_tracking/following_position --once
ros2 topic echo /person/status --once

# 6. 可視化（Docker Desktop環境ではソフトウェアレンダリング必須。上記override参照）
rviz2   # Displays に LaserScan(/scan_filtered), PoseArray(/dr_spaam/dr_spaam_detections),
        # MarkerArray(sobits_follower/multiple_sensor_person_tracking/tracker_marker) を追加
        # Fixed Frame は laser_link
```

`target_frame:=base_link`（本番の `bringup.launch.py` と同じ設定）で確認したい場合は
`robot_state_publisher` 等で `base_link → laser_link` の TF を先に流しておくこと。

---

## 起動手順

### Step 1: 地図作成 (初回のみ)

```bash
# コンテナ起動
docker compose run --rm th_robot bash

# ビルド (コンテナ内。イメージには外部依存パッケージのみビルド済みのため、
# th_* パッケージは初回に必ずビルドが必要)
cd /root/th_ws
colcon build --symlink-install
source install/setup.bash

# (任意) 駆動系だけのキーボード操作テスト — LiDAR・安全監視なしの最小構成。
# 実機での動作確認済み。ESP32 が AP として起動し PC が接続済みであること
ros2 launch th_bringup esp32_keyboard_test.launch.py

# SLAM 起動 (コンテナ内。LiDAR はラズパイ側で起動しておく — 冒頭の実機構成参照)
ros2 launch th_bringup slam.launch.py lidar_source:=network

# 別ターミナルでタブレット UI を起動し、手動操作で全エリアを走る
# 地図が完成したら保存
ros2 run nav2_map_server map_saver_cli -f /root/th_ws/src/th_bringup/maps/th_map
```

### Step 2: LiDAR 死角の実測と設定

```bash
# /scan を RViz2 で確認
ros2 run rviz2 rviz2

# ロボットを静止させ、アルミ角柱が映る角度帯を読み取る
# src/th_bringup/config/perception_params.yaml の blind_angle_ranges を更新
```

### Step 3: オドメトリキャリブレーション

```bash
# 直進キャリブレーション (床に 2m のマーキングを準備)
ros2 run th_calibration linear_calib.py --ros-args -p distance:=2.0

# 旋回キャリブレーション
ros2 run th_calibration rotation_calib.py --ros-args -p turns:=1

# 補正値を適用
ros2 run th_calibration apply_calib.py --ros-args \
  -p wheel_radius:=<補正後の値> -p wheel_base:=<補正後の値>
```

### Step 4: フル起動

```bash
# 地図指定で起動 (SLAM マッピングモードは map_yaml を空のまま)
# 現行構成では LiDAR はラズパイ配信のため lidar_source:=network を付ける
ros2 launch th_bringup bringup.launch.py \
  lidar_source:=network \
  map_yaml:=/root/th_ws/src/th_bringup/maps/th_map.yaml \
  use_stub:=false

# テスト時 (試験員追従スタブ使用)
ros2 launch th_bringup bringup.launch.py lidar_source:=network use_stub:=true
```

### Step 5: Web UI（タブレット UI）

#### 初回のみ: 依存パッケージをインストール

```bash
# Docker コンテナの外（Windows または WSL2）で実行
cd web_ui
npm install
```

#### 起動

```bash
cd web_ui
npm run dev
# → http://localhost:5173 で開発サーバーが起動
```

#### タブレットからアクセス

1. PC と タブレットを同じ Wi-Fi ネットワークに接続する
2. PC の IP アドレスを確認する

   ```bash
   # Windows
   ipconfig | findstr "IPv4"
   # WSL2 / Linux
   ip addr show | grep "inet " | grep -v 127
   ```

3. タブレットのブラウザで `http://<PCのIP>:5173` を開く

#### rosbridge の接続先変更

デフォルトでは `ws://<現在のホスト>:9090` に接続しようとする。
タブレットから別の PC（ロボット本体）の rosbridge に接続したい場合は
`web_ui/src/hooks/useRosbridge.js` の `url` を修正する:

```js
// 例: ロボットの IP が 192.168.1.100 の場合
const url = 'ws://192.168.1.100:9090';
```

rosbridge は Docker コンテナ内で `bringup.launch.py` 起動時に自動で立ち上がる（ポート 9090）。

#### UI でできること

| 操作 | 説明 |
| --- | --- |
| 追従開始 | IDLE → FOLLOWING モードへ切替 |
| 軌跡追従(マップ不要) | IDLE/MANUAL → FOLLOWING_MAPLESS モードへ切替(地図・Nav2不要) |
| 手動操作 | MANUAL モードへ切替・仮想ジョイスティック操作 |
| 配電盤移動 | MOVING_TO_PANEL モードへ切替・目的地選択 |
| 緊急停止 | ESTOP モードへ即時遷移 |
| モード表示 | 現在のロボットモードをリアルタイム表示 |
| フォルト表示 | `LIDAR_LOST` / `ESP32_DISCONNECTED` 等のアラート表示 |

---

## モード操作

| モード | 状態 | タブレット操作 |
| --- | --- | --- |
| IDLE | 静止待機 | 起動時の初期状態 |
| FOLLOWING | 試験員自動追従(地図・Nav2使用) | 「追従開始」ボタン |
| FOLLOWING_MAPLESS | 試験員自動追従(地図・Nav2不要) | 「軌跡追従(マップ不要)」ボタン |
| MOVING_TO_PANEL | 配電盤移動中 | 配電盤ボタン |
| AT_PANEL | 配電盤前作業中 | 自動遷移 |
| MANUAL | 手動操作 | 「手動操作」ボタン |
| ESTOP | 緊急停止 | 緊急停止ボタン / 物理スイッチ |

---

## フォルト対応

| フォルト | 原因 | 対処 |
| --- | --- | --- |
| `LIDAR_LOST` | LiDAR データ途絶 | ケーブル・ドライバ確認 → 再起動 |
| `ESP32_DISCONNECTED` | ESP32 通信途絶 (WiFi/WebSocket) | AP への WiFi 接続・portproxy/固定IP 設定確認 → ESP32 再起動 |
| `PERSON_TRACKER_LOST` | 追従データ途絶 | person_tracker ノード確認 |

フォルト発生時は `twist_mux` が即座にモーター出力をゼロにし、
`mode_manager` が IDLE に遷移します。復帰には **タブレットで明示的な操作** が必要です。

---

## ディレクトリ構成

```tree
th_ws/
├── Dockerfile
├── docker-compose.yml
├── setup.sh
├── scripts/
│   └── run_tests.sh              # テスト実行スクリプト
├── udev/
│   └── 99-th-robot.rules
├── esp32/                        # PlatformIO プロジェクト
│   ├── platformio.ini
│   └── src/
│       ├── main.cpp              # WebSocket クライアントループ・自動再接続・E-Stop
│       ├── config.h              # ピン・パラメータ定義
│       ├── ws_link.h / .cpp      # WebSocket バイナリプロトコル・接続管理
│       ├── wifi_credentials.h    # WiFi/WSサーバー接続情報 (.gitignore 対象)
│       ├── encoder.h / .cpp      # A-B 相エンコーダ（割り込み駆動）
│       ├── motor.h / .cpp        # Cytron MD10C 制御（LEDC PWM）
│       └── pid.h                 # 汎用 PID コントローラ
├── src/                          # ROS2 ワークスペース
│   ├── th_system_msgs/           # カスタム型定義（全ノード共有）
│   ├── th_esp32_bridge/          # ESP32 ↔ ROS2 ブリッジ・オドメトリ
│   ├── th_safety/                # safety_monitor + twist_mux 設定
│   ├── th_mode_manager/          # FSM（8 状態・全遷移ルール）
│   ├── th_perception/            # lidar_filter・person_predictor・tracker_stub・person_tracker_bridge

│   ├── th_planning/              # follow_planner・panel_navigator・manual_handler・crawler_teleop
│   │   └── th_planning/          # Python パッケージ（テスト可能なコアロジック）
│   │       ├── follow_planner_core.py
│   │       └── mapless_follow_core.py
│   ├── th_calibration/           # オドメトリキャリブツール
│   ├── th_description/           # URDF (base_link→laser_link 等の TF 定義)
│   ├── th_bringup/               # 起動設定・地図・パラメータ
│   ├── multiple_observation_kalman_filter/  # vendored (human_kenchi, プライベートリポジトリ)
│   ├── multiple_sensor_person_tracking/     # vendored (human_kenchi) — PersonTracker (leg モード)
│   ├── leg_detection_bringup/               # vendored (human_kenchi) — DR-SPAAM+Tracker launch
│   └── th_testing/               # 単体・統合テスト
│       └── test/
│           ├── conftest.py
│           ├── test_follow_planner_logic.py
│           ├── test_mapless_follow_logic.py
│           ├── test_mode_transitions.py
│           ├── test_safety_monitor.py
│           ├── test_twist_mux_priority.py
│           ├── test_fault_detection.py
│           └── test_simulation_scenarios.py
└── web_ui/                       # React タブレット UI
    ├── index.html
    ├── package.json
    └── src/
        ├── App.jsx / App.css
        ├── main.jsx
        └── hooks/
            └── useRosbridge.js
```

---

---

---

## Gazebo シミュレーション

## 概要

実機（ESP32・RPLIDAR S1）がなくても、Gazebo Classic 上で全追従ロジックを視覚的に確認できます。
`sim:=true/false` の引数一つで実機とシミュレーションを切り替えられます。

```txt
実機モード:   sllidar_node + esp32_bridge(WebSocketサーバー) が起動
シミュレーション: Gazebo が /odom・/scan を発行し上記3ノードは不要
共通:         mode_manager・follow_planner・Nav2・rosbridge 等は同一ノードが動く
```

---

## 起動コマンド早見表

```bash
# 1. SLAM で地図を作りながらシミュレーション（最初はこれ）
ros2 launch th_bringup gazebo.launch.py

# 2. 既存地図でナビゲーション＋追従シミュレーション
ros2 launch th_bringup gazebo.launch.py \
  slam:=false map_yaml:=/root/th_ws/src/th_bringup/maps/th_map.yaml

# 3. walk.dae がない環境（cylinder モデルで代替）
ros2 launch th_bringup gazebo.launch.py \
  world:=$(ros2 pkg prefix th_bringup)/share/th_bringup/worlds/panel_room_no_actor.world

# 4. URDF だけ RViz2 で確認
ros2 launch th_description display.launch.py

# 5. キーボード手動操作（別ターミナルで実行）
ros2 launch th_bringup teleop.launch.py           # /cmd_vel_nav 経由
ros2 launch th_bringup teleop.launch.py direct:=true  # /cmd_vel 直接（SLAM 用）

# 6. 実機モード（Gazebo なし）
ros2 launch th_bringup gazebo.launch.py sim:=false \
  map_yaml:=/root/th_ws/src/th_bringup/maps/th_map.yaml
```

---

## Step by Step：初回シミュレーション実行手順

### Step 1: Docker コンテナで起動

**Linux ホストの場合:**

```bash
xhost +local:docker            # X11 転送を許可
docker compose run --rm th_robot bash
```

**Windows (WSL2) の場合:**

```bash
# VcXsrv や X410 などの X サーバーを起動してから
export DISPLAY=:0
docker compose run --rm th_robot bash
```

> **補足:** 現行構成では LiDAR はラズパイ側に接続するため、`docker-compose.yml` の
> `devices:` (`/dev/lidar`) はコメントアウト済み。PC に LiDAR を USB 直結する場合のみ
> コメントを解除する。

### Step 2: ビルド

```bash
# コンテナ内
cd /root/th_ws
colcon build --symlink-install   # 初回または C++ 変更時
source install/setup.bash
```

> Dockerfile 内でビルド済みのため、通常は不要。Python スクリプトの変更はシンボリックリンクで即時反映される。C++ パッケージ (`th_safety` など) を変更した場合のみ再ビルドが必要。

### Step 3: SLAM で地図を作成

```bash
# ターミナル 1: シミュレーション起動（初回は SLAM モード）
ros2 launch th_bringup gazebo.launch.py

# ターミナル 2: クローラーテレオペ（別 xterm ウィンドウが開く）
ros2 launch th_bringup teleop.launch.py direct:=true
```

xterm ウィンドウをクリックしてフォーカスを与えてからキー操作:

| キー | 動作 |
| --- | --- |
| `w` | 直進前進 |
| `s` | 直進後退 |
| `q` / `e` | 超信地旋回 左 / 右（その場旋回） |
| `a` / `d` | 緩旋回前進 左 / 右（円弧走行） |
| `z` / `c` | 緩旋回後退 左 / 右（円弧走行） |
| `x` / Space | 停止 |
| `+` / `-` | 並進速度を ±0.05 m/s |
| `[` / `]` | 旋回速度を ±0.1 rad/s |

RViz2 で `/map` に地図が表示されたら部屋全体を走り回り、地図を保存:

```bash
# ターミナル 3
ros2 run nav2_map_server map_saver_cli \
  -f /root/th_ws/src/th_bringup/maps/th_map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10000.0
```

### Step 4: 追従動作の確認

```bash
# ターミナル 1: 地図ありで起動（AMCL 自己位置推定）
ros2 launch th_bringup gazebo.launch.py \
  slam:=false map_yaml:=/root/th_ws/src/th_bringup/maps/th_map.yaml
```

起動から約 10 秒後（Nav2 が active になってから）、モードを FOLLOWING に切替:

```bash
# ターミナル 2
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 2, requester: 'cli'}"
```

Gazebo ウィンドウで Inspector（試験員役）が移動し、ロボットが追従することを確認する。Inspector は Gazebo 起動 1 秒後から配電盤前を往復し始める。

> FOLLOWING モード中に手動操作したい場合は `requested_mode: 5` (MANUAL) に切替え、別ターミナルで `ros2 launch th_bringup teleop.launch.py` を実行する。

### Step 5: MAP不要の軌跡追従モード（FOLLOWING_MAPLESS）の確認

`follow_planner_mapless.py` は Nav2・SLAM 占有格子を一切使わず、`/odom`（ホイールオドメトリ）と `/scan_filtered`（死角マスク済み生 LiDAR スキャン）だけで動く追従モード。地図読み込み・AMCL・Nav2 のゴール計画が不要なため、**地図を作る前でも検証できる**。

`RobotMode.msg` に `FOLLOWING_MAPLESS` 定数を追加しているため、検証前に **`th_system_msgs` を含むフルビルドが必須**:

```bash
cd /root/th_ws
colcon build --symlink-install
source install/setup.bash
```

起動（SLAM/地図の有無に関わらず動作確認できる。Nav2 自体はこの起動コマンドで引き続き立ち上がるが、`follow_planner_mapless` はそれを一切参照しない）:

```bash
ros2 launch th_bringup gazebo.launch.py
```

FOLLOWING_MAPLESS へ切替（CLI から）:

```bash
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 7, requester: 'cli'}"
```

タブレット UI から切替える場合は「軌跡追従(マップ不要)」ボタンを押す。

確認項目:

| 確認項目 | 方法 | 期待動作 |
| --- | --- | --- |
| 軌跡追従 | Inspector（または `person_mover.py --pattern:=patrol`）を移動させる | Nav2 にゴールを送らず、`/cmd_vel_retreat` に直接速度指令が出て試験員の軌跡を遡って追従する |
| 接近時は退避せず停止 | `person_mover.py --ros-args -p pattern:=approach -p approach_dist:=0.6` 等で `stop_distance`（既定 1.0m）以内に接近させる | 後退・旋回せず、その場で停止する（`/cmd_vel_retreat` がゼロ） |
| 停止後の追従再開 | 接近させた後、`resume_distance`（既定 1.3m）以上離す | 自動的に追従を再開する（オペレータ操作不要） |
| 進路上障害物での停止 | ロボットの進行方位上（軌跡ゴールへのベアリング方向）に障害物モデルを置く | 試験員との距離に関わらずその場で停止する |
| フェイルセーフ | `/scan_filtered` を止める（`lidar_filter` ノードを kill 等） | スキャン未受信になった時点で停止する（安全側） |

停止理由はログで確認できる:

```bash
ros2 topic echo /rosout | grep follow_planner_mapless
# "停止中（理由: person_close/obstacle_ahead/no_pose/no_scan）" または "追従再開" が出力される
```

`/cmd_vel_retreat` を直接確認する場合:

```bash
ros2 topic echo /cmd_vel_retreat
```

実機での検証も同じ手順で行える（`bringup.launch.py` にも `follow_planner_mapless` が同様に組み込まれている）。地図・Nav2・AMCL を一切起動していない状態でも `FOLLOWING_MAPLESS` 単体で動作することを確認するとよい。

---

## ワールド環境の構成

```txt
panel_room.world（デフォルト）
  部屋サイズ: 10m × 8m
  配電盤:     北壁沿いに 3 台（青い箱）
  通路:       幅 1.4m の仕切り壁（狭路追従テスト用）
  試験員:     Gazebo Actor（walk.dae アニメーション付き人物）
              → walk.dae がない場合は panel_room_no_actor.world を使用

panel_room_no_actor.world（代替）
  試験員:     cylinder モデル（person_mover.py で自動移動）
```

---

## 試験員（Actor/Cylinder）の制御

### Actor 版（panel_room.world）

Actor はウェイポイントに沿って自動で動きます。経路の変更は world ファイルの
`<trajectory>` セクションを編集してください。

```xml
<!-- 配電盤間を移動するパス例 -->
<waypoint><time>0.0</time><pose>-3.5 2.5 0 0 0 0</pose></waypoint>
<waypoint><time>5.0</time><pose> 3.5 2.5 0 0 0 0</pose></waypoint>
```

### Cylinder 版（panel_room_no_actor.world）

`person_mover.py` でシナリオを制御します。

```bash
# 巡回パターン（配電盤前を往復）
ros2 run th_perception person_mover.py --ros-args -p pattern:=patrol

# 接近パターン（退避ロジックのテスト）
ros2 run th_perception person_mover.py --ros-args \
  -p pattern:=approach -p approach_dist:=0.6

# 静止パターン（静止時再配置のテスト）
ros2 run th_perception person_mover.py --ros-args -p pattern:=static
```

---

## 確認できるシナリオ

| シナリオ | 設定 | RViz2 で見るもの |
| --- | --- | --- |
| 通常追従 | `pattern:=patrol` | ロボットが試験員の後ろ 1.5m を追従 |
| 近接退避 | `pattern:=approach` | 0.8m 以内で `/cmd_vel_retreat` が発行され後退 |
| 狭路真後ろ追従 | 仕切り壁の通路を通る | 角度オフセットが 0° になる |
| 静止再配置 | `pattern:=static` | 2 秒後に試験員の側面〜背面に移動 |
| 配電盤移動 | `ros2 service call /panel_navigator/go_to_panel ...` | Nav2 がパネル前まで誘導 |
| E-Stop | タブレット UI の緊急停止 | ロボットが即時停止し ESTOP モードへ |
| MAP不要軌跡追従 | `FOLLOWING_MAPLESS` へ切替 + `pattern:=approach` | 地図・Nav2 なしで追従、接近時は退避せず停止(詳細は Step 5 参照) |

---

## RViz2 の見方

起動時に表示される `th_sim.rviz` レイアウト:

| 表示 | 説明 |
| --- | --- |
| 赤い点群 | `/scan`（生スキャン、アルミ角柱の死角含む） |
| 緑の点群 | `/scan_filtered`（死角マスク済み、Nav2 が使用するもの） |
| グレーの地図 | SLAM で作成中の `/map` |
| 薄い色のオーバーレイ | ローカル costmap（Nav2 の障害物回避に使用） |
| 青い線 | Nav2 の計画経路 |
| 黄色い球 | 試験員の予測位置（`/person/predicted_position`） |
| ロボットモデル | URDF から生成された 3D モデル |

---

## よくある問題と対処

### Gazebo が起動しない（画面が出ない）

```bash
# X11 転送を確認
echo $DISPLAY           # :0 や :1 が表示されるべき
xhost +local:docker     # ホスト側で実行

# Docker 内で確認
glxinfo | head -5       # OpenGL が使えるか確認
```

### ロボットが Gazebo でスポーンされない

```bash
# spawn を手動で再実行
ros2 run gazebo_ros spawn_entity.py \
  -topic /robot_description -entity th_robot -x -4 -y -3 -Y 0
```

### walk.dae が見つからない（Actor エラー）

`panel_room_no_actor.world` に切り替えてください:

```bash
ros2 launch th_bringup gazebo.launch.py \
  world:=$(ros2 pkg prefix th_bringup)/share/th_bringup/worlds/panel_room_no_actor.world
```

### /scan が来ない（LiDAR プラグインが動かない）

```bash
# Gazebo プラグインのロードを確認
ros2 topic list | grep scan
# → /scan が表示されなければ URDF の gazebo_plugins.xacro を確認

# LiDAR の可視化を ON にして Gazebo で確認
# (gazebo_plugins.xacro の <visualize>false</visualize> を true に変更)
```

### 試験員の位置が /person/status に来ない

```bash
# Gazebo のモデル状態を確認
ros2 topic echo /gazebo/model_states | grep -A5 inspector

# リレーノードのログを確認
ros2 node info /gazebo_person_relay
```

## TBD (今後確定が必要な事項)

- [ ] LiDAR 死角角度の実測値 (`perception_params.yaml`)
- [ ] 配電盤座標の登録 (`panels.yaml`)
- [x] DR-SPAAM 重みファイルの実機への配置・単体動作確認（脚検知パイプライン単体で確認済み。`bringup.launch.py use_stub:=false` を通した end-to-end 確認はまだ）
- [ ] LiDAR 死角と DR-SPAAM 誤検出の相性を実地検証（`leg_tracker_param.yaml` の再取得パラメータ調整、長時間・実運用環境での検証）
- [ ] IMU 調達後の EKF 切替 (`imu_enabled:=true`)
- [ ] カメラ昇降システムとのインターフェース確定
- [ ] ESP32 E-Stop GPIO の極性確認 (`config.h` の `ESTOP_LOW_ACTIVE`)
- [ ] モータードライバ正転方向の確認 (`MOT_RIGHT_FWD`, `MOT_LEFT_FWD`)
- [ ] タブレット機種確定後の UI レイアウト調整

---

---

## テスト

## テスト構成一覧

| ファイル | 分類 | ROS2 要否 | テスト件数 | 設計書対応 |
| --- | --- | --- | --- | --- |
| `test_follow_planner_logic.py` | 純粋単体テスト | 不要 | 43 件 | 10.1 §3 |
| `test_mapless_follow_logic.py` | 純粋単体テスト(MAP不要モード) | 不要 | 20 件 | - |
| `test_mode_transitions.py` | ROS2 統合テスト | 必要 | 19 件 | 10.1 §1 |
| `test_safety_monitor.py` | ROS2 統合テスト | 必要 | 10 件 | 10.1 §5 |
| `test_twist_mux_priority.py` | ROS2 統合テスト | 必要 | 7 件 | 10.1 §4 |
| `test_fault_detection.py` | ROS2 統合テスト | 必要 | 4 件 | 10.1 §5+ |
| `test_simulation_scenarios.py` | シナリオテスト | 必要 | 5 件 | 10.3 |

---

## 純粋単体テストの実行（ROS2 なし）

`follow_planner_core.py` のコアロジックは ROS2 に依存しない純粋な Python モジュールとして実装されており、ROS2 環境がなくてもテストできます。

```bash
# pytest を直接実行
cd th_ws
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py -v

# 特定のテストクラスだけ実行
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py \
  -v -k "TestNextFollowState"

# 失敗時に即座に停止
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py -x
```

`test_follow_planner_logic.py` のテスト対象クラスと確認内容:

```txt
TestFrameConversion         ── base_link相対⇔絶対座標(odom系)変換の往復・回転整合性
TestTrail                   ── 軌跡点の最小移動距離フィルタ・lookback距離での遡り
TestNextFollowState         ── TRACKING/PREPARE/EVADING の境界値・ハンチング防止
TestFindNearestOpenDirection ── 全方向自由/一方向のみ自由/最小自由空間未達での不採用
TestComputeEvadeGoal        ── 退避方向・距離からの退避ゴール点計算
TestOrientToEvadeRoute      ── 許容誤差内での停止・誤差角に比例した旋回
TestPurePursuitControl      ── 停止半径内での停止・距離に応じた速度クランプ
TestFollowPlannerCore       ── 状態遷移・trail追従ゴール・PREPARE/EVADING出力・reset/clear_trail
```

`test_mapless_follow_logic.py` のテスト対象クラスと確認内容:

```txt
TestNextMaplessState  ── stop_distance/resume_distance の境界・ハンチング防止
TestIsPathBlocked     ── 進路上障害物の有無・角度範囲外/inf/nanの無視
TestMaplessFollowCore ── 追従駆動・接近時停止と再開・障害物停止・no_pose/no_scanフェイルセーフ
```

---

## ROS2 統合テストのビルドと実行

```bash
# ROS2 環境をソース（Docker 内では不要）
source /opt/ros/humble/setup.bash

# ビルド（テスト対象パッケージを含めて）
cd th_ws
colcon build --symlink-install \
  --packages-select \
    th_system_msgs th_safety th_mode_manager \
    th_planning th_perception th_testing

source install/setup.bash

# 全統合テストを実行
colcon test --packages-select th_testing \
  --event-handlers console_direct+

# 結果確認
colcon test-result --verbose
```

### 個別テストの単独実行

```bash
# mode_manager 遷移テストのみ
python3 -m pytest src/th_testing/test/test_mode_transitions.py \
  -v --timeout=120

# safety_monitor フォルト検知テストのみ
python3 -m pytest src/th_testing/test/test_safety_monitor.py \
  -v --timeout=90

# twist_mux 優先度テストのみ
python3 -m pytest src/th_testing/test/test_twist_mux_priority.py \
  -v --timeout=60

# フォルト→モード遷移の順序テストのみ
python3 -m pytest src/th_testing/test/test_fault_detection.py \
  -v --timeout=90
```

---

## シナリオテストの実行

シナリオテストは `person_tracker_stub` を使って Gazebo なしで実行できます。
`TH_SKIP_SIM` 環境変数で制御します（デフォルト=スキップ）。

```bash
# シナリオテストを有効化して実行
TH_SKIP_SIM=0 python3 -m pytest \
  src/th_testing/test/test_simulation_scenarios.py -v --timeout=120
```

確認されるシナリオ:

- **Scenario A-1**: 試験員が近接距離以内に接近 → `/cmd_vel_retreat` に退避指令が発行される
- **Scenario A-2**: 壁際（costmap でブロック）での接近 → その場停止
- **Scenario A-3**: 試験員が離れると退避解除 → 通常追従に戻りハンチングしない
- **Scenario C**: ロスト後の予測外挿 → `max_predict_sec` 後に `/person/search_mode=true`
- **Scenario D**: MANUAL 中は退避が作動しない（試験員が近くにいても後退しない）

> **注意**: 退避方向は試験員の位置とは無関係に `find_nearest_open_direction()` による「地図上の最空きスペース方向」で決まる。costmap を配信しない stub 環境では試験員自身が障害物として地図に現れないため、Scenario A-1 の「試験員を避けて後退した」という確認は Gazebo + 実際の costmap がある環境でのみ意味を持つ。詳細は `test_simulation_scenarios.py` 内のコメントを参照。

---

## 一括テスト実行スクリプト

```bash
# 単体テストのみ（デフォルト・高速）
bash scripts/run_tests.sh

# 単体 + ROS2 統合テスト
bash scripts/run_tests.sh --all

# 統合テスト + シナリオテスト
bash scripts/run_tests.sh --all --sim
```

---

## テスト追加のガイドライン

新しいロジックを追加する際は以下の構成に従ってください。

```txt
ロジック追加
  → th_planning/th_planning/follow_planner_core.py に純粋関数/クラスとして実装
  → src/th_testing/test/test_follow_planner_logic.py にテストクラスを追加
  → ROS2 ノード(follow_planner.py)はコアロジックを呼び出すだけにする
```

この構造により、ROS2 環境なしで CI/CD パイプラインの高速テストが可能になります。

---

---

## 保守・拡張 技術詳細

## アーキテクチャ設計方針

本システムは以下の 3 つの原則に基づいて設計されています。

**安全レイヤーの独立性**: `twist_mux` が `/cmd_vel` の最終出力を一元管理し、個々のノードの実装ミスが物理的な動きに影響しないことをアーキテクチャで保証します。`safety_monitor` が `mode_manager` のモード遷移を待たずに `twist_mux` をロックするため、フォルト検知から物理停止までにソフトウェア処理のレイテンシが介在しません。

**テスト可能なコアロジック**: 追従ロジック（`follow_planner_core.py`）を ROS2 非依存の純粋 Python モジュールとして実装しています。これにより ROS2 環境なしでロジックの単体テストが実行でき、アルゴリズムの変更を安全に検証できます。

**パラメータ外部化**: 全ノードの調整値を YAML ファイルで管理し、コード変更なしにチューニングできます。特に追従距離・PID ゲイン・フォルトタイムアウトは現場検証後に頻繁に変更される値であるため、すべてパラメータ化されています。

---

## 速度指令の排他制御（twist_mux）

`/cmd_vel` への速度指令は複数のノードから発行されますが、最終的な出力は常に `twist_mux` が一元管理します。これにより個々のノードが互いの状態を意識する必要がなくなります。

```txt
優先度（高い方が優先）:
  255: /safety/estop      lock   — E-Stop 発動中は全入力を無視してゼロ出力
  254: /safety/fault_lock lock   — フォルト検知時も同様
   20: /cmd_vel_retreat   topic  — follow_planner からの近接退避指令・person_predictor からの捜索旋回指令（Nav2 を迂回）
   10: /cmd_vel_nav       topic  — Nav2 controller_server の通常出力
```

`retreat` が `nav` より高い優先度を持つため、Nav2 がゴールへの経路を計算し続けていても、退避が必要な瞬間に `follow_planner` が直接 `/cmd_vel_retreat` を発行すれば即座に反映されます。`follow_planner` 側で Nav2 のゴールをキャンセルする必要はありません。

退避が終了した際は `/cmd_vel_retreat` の発行を止めるだけで、`twist_mux` のタイムアウト（0.5 秒）が経過すると自動的に `/cmd_vel_nav` に切り替わります。これが「退避解除」の実装です。

各入力トピックのタイムアウト値変更が必要な場合は `th_safety/config/twist_mux.yaml` を編集します。

---

## 状態管理（mode_manager FSM）

### 遷移ルールの実装

`mode_manager` は `isTransitionAllowed()` 関数で遷移の許可・拒否を判定します。新しいモードを追加する場合はこの関数に遷移元/遷移先のペアを追加するだけで対応できます。

```cpp
// mode_manager.cpp の isTransitionAllowed() を参照
// 例: 新モード "AUTO_PATROL" を追加する場合
case RobotMode::IDLE:
    return to == RobotMode::FOLLOWING ||
           to == RobotMode::MANUAL    ||
           to == RobotMode::AUTO_PATROL;  // ← 追加
```

### 安全設計上の制約（変更禁止）

以下の遷移ルールは安全上の要件であり、変更してはいけません。

- `IDLE → FOLLOWING` および `IDLE → MANUAL` は外部からのサービス呼び出し（明示操作）のみで発生し、自動では遷移しない。ロボットが操作者の意図しないタイミングで動き出すことを防ぐための方針。
- `ESTOP` へはどのモードからも即座に遷移できる。`ESTOP` からは `IDLE` にのみ遷移できる。
- フォルト（`/safety/fault`）は `FOLLOWING`・`MOVING_TO_PANEL`・`MANUAL`・`AT_PANEL` のいずれからでも `IDLE` へ強制遷移させる。`IDLE` 中のフォルトはモード変化を起こさない。

### heartbeat によるMANUAL自動解除

`manual_command_handler` はタブレット UI からの `/manual/heartbeat`（`std_msgs/Empty`、2 Hz）を監視します。1 秒間受信がない場合は通信断と判断し、`/mode_manager/set_mode` を呼び出して `IDLE` へ遷移させます。これは想定外の切断（Wi-Fi 瞬断・ブラウザクラッシュ等）に対するフェイルセーフです。タブレットから意図的に `MANUAL` を終了した場合（「追従再開」ボタン押下）は、`FOLLOWING` へ直接遷移します。

---

## 試験員追従ロジック（follow_planner_core / mapless_follow_core）

追従ロジックは2つの独立した実装があり、モードによって使い分けられます。

| | `follow_planner_core.py` | `mapless_follow_core.py` |
| --- | --- | --- |
| 使用モード | `FOLLOWING` / `MOVING_TO_PANEL` | `FOLLOWING_MAPLESS` |
| 目標地点の送出先 | Nav2 (`NavigateToPose`) | 直接 `(v, ω)` を `/cmd_vel_retreat` へ |
| 地図・costmap | 必要（`/local_costmap/costmap` + TF） | 不要（`/odom` TF と `/scan_filtered` のみ） |
| 近接時の挙動 | 地図上の最空きスペース方向へ退避 | 退避せずその場停止・離れたら再開 |

### 内部状態と優先順位（follow_planner_core）

`FollowPlannerCore.update()` は毎制御周期（10 Hz）に呼ばれ、試験員との距離のみに基づいて次の3状態を切り替えます（`followLogic.md` v2 設計）。

```txt
TRACKING（軌跡追従）   d ≥ d_prepare(既定3.0m)
      ↓ d < d_prepare
PREPARE（退避準備）     d_evade(既定2.0m) ≤ d < d_prepare
      ↓ d < d_evade
EVADING（退避）         d < d_evade
```

各状態の詳細は `follow_planner_core.py` の `FollowPlannerCore.update()` を参照してください。

### 状態遷移のヒステリシス（ハンチング防止）

`next_follow_state()` は復帰判定に `distance_hysteresis_m`（既定 0.2m）分の余裕を要求します。例えば `d_prepare=3.0m` の場合、距離が 3.0m を下回ると PREPARE に入りますが、TRACKING へ戻るには `d_prepare + distance_hysteresis_m = 3.2m` 以上必要です（EVADING⇔PREPARE も同様に `d_evade + distance_hysteresis_m` を要求）。現場でハンチングが観測された場合は `distance_hysteresis_m` を大きくしてください。

### 軌跡追従（TRACKING）

試験員の位置履歴（`trail`。ロボットが移動しても意味が変わらないよう絶対座標系=odom系で保持）を `lookback_distance`（既定 1.0m）だけ遡った点をそのまま Nav2 のゴールとして送ります。旧設計にあった通路幅判定・角度オフセット・LiDAR視野角制約は廃止されています。

### 退避方向探索（PREPARE）

退避方向は試験員の歩行方向に依存しません。`find_nearest_open_direction()` が `/local_costmap/costmap` を放射状（`evade_scan_directions`、既定16方向、`evade_scan_max_dist` まで走査）に調べ、`retreat_check_clearance`（既定0.5m）以上の自由空間を確保できる中で最も開けた方向を選びます。この方向は PREPARE 突入時に一度だけ計算され、EVADING に移るまで保持されます。

### 退避走行（EVADING）

PREPARE で決めた方向に `evade_route_length_m`（既定2.0m）先の点をゴールとし、Pure Pursuit（`pure_pursuit_control()`）で `retreat_speed` を上限速度として走行します。

---

## MAP不要軌跡追従ロジック（mapless_follow_core）

`FOLLOWING_MAPLESS` モードでは `MaplessFollowCore.update()` が Nav2 を使わず毎周期直接 `(v, ω)` を計算します。状態は TRACKING/STOPPED の2つのみで、近接時も退避行動は取りません。

```txt
TRACKING（追従中）
      ↓ d < stop_distance(既定1.0m)
STOPPED（停止中・退避せずその場停止）
      ↓ d ≥ resume_distance(既定1.3m)
TRACKING へ復帰
```

- **軌跡追従**: `follow_planner_core.py` の `update_trail`/`get_trail_goal`/`pure_pursuit_control`（絶対座標系での軌跡保持を含む）をそのまま再利用します。
- **進路上障害物チェック**: costmap ではなく `/scan_filtered` の生レンジ値を `is_path_blocked()` で直接走査します。軌跡ゴールへの進行方位を中心に `obstacle_check_half_width_deg`（既定20°）の範囲・`obstacle_check_distance_m`（既定1.0m）未満に有効なレンジ値があれば、試験員との距離に関わらず停止します。
- **フェイルセーフ**: TF（odom→base_link）が未確立、または `/scan_filtered` を一度も受信していない場合は安全側に倒して停止します（costmap 方式の「未受信時は自由とみなす」というフェイルオープンとは逆の設計です。他に障害物安全層が無いためです）。

---

## 安全設計の実装詳細

### フォルト検知のタイムライン

```txt
フォルト発生
  ↓ （check_period_ms 以内、デフォルト 100ms）
safety_monitor が途絶を検知
  ↓ （即時）
/safety/fault を発行
  ↓ （twist_mux がトピックを受信した次の制御周期、通常 10ms 以内）
twist_mux が /cmd_vel をゼロに固定（物理的な動きを停止）
  ↓ （mode_manager が /safety/fault を受信・処理）
mode_manager が IDLE へ遷移
  ↓ （タブレット UI が /robot/mode を受信）
UI にフォルト表示・操作要求
```

`twist_mux` によるモーター停止は `mode_manager` のモード遷移処理を待ちません。これにより、フォルト検知から物理停止までの時間は `check_period_ms`（100ms）+ twist_mux の処理時間（数 ms）のみです。

### ESP32 側の二重フェイルセーフ

ROS2 側の `safety_monitor` に加え、ESP32 ファームウェアにもウォッチドッグが実装されています。

```txt
ROS2 クラッシュ・USB 切断発生
  ↓ （同時に独立して動作）
[ESP32 側] wheel_cmd 受信が WATCHDOG_MS(300ms) 途絶
           → モーター強制停止（ハードウェアレベル）
[ROS2 側]  safety_monitor が wheel_feedback 途絶を esp32_timeout_ms(500ms) で検知
           → /safety/fault 発行 → twist_mux ロック
```

ESP32 ウォッチドッグ（300ms）の方が safety_monitor（500ms）より先に作動する設計です。これにより ROS2 側の処理が間に合わなくても物理停止が保証されます。

### E-Stop の集約

物理ボタンとタブレット UI の両方の E-Stop を `safety_monitor` が集約します。

```txt
[物理スイッチ] → GPIO34 → ESP32 → /safety/estop_hw → safety_monitor
                                                              ↓（OR）
[タブレット UI] → WebSocket → /safety/tablet_estop → safety_monitor
                                                              ↓
                                                    /safety/estop
                                                              ↓
                                              twist_mux lock（ゼロ強制）
                                              mode_manager → ESTOP
```

物理スイッチは電気的にモータードライバ電源を遮断するため、ROS2 の処理に依存しないハードウェア的な停止も同時に発動します。

---

## オドメトリとセンサフュージョン

### 差動駆動の運動学

`esp32_bridge` が実装するオドメトリ計算式です。

```txt
入力:
  v_L, v_R = 左右ホイールの実速度 [m/s]（ESP32 から wheel_feedback で取得）
  dt        = 前回更新からの経過時間 [s]

更新:
  v_center = (v_L + v_R) / 2
  omega    = (v_R - v_L) / wheel_base
  x   += v_center * cos(θ + omega * dt / 2) * dt
  y   += v_center * sin(θ + omega * dt / 2) * dt
  θ   += omega * dt
```

クローラーは超信地旋回時にスリップが発生しやすく、エンコーダ単体のオドメトリは誤差が蓄積します。SLAM Toolbox による自己位置補正（スキャンマッチング）がこのドリフトを抑制します。

### キャリブレーション手順の詳細

```txt
目標精度: 直進 2m で誤差 ±2cm 以内、旋回 360° で誤差 ±5° 以内

1. linear_calib.py を 3 回実行して平均を取る
2. rotation_calib.py を 3 回実行して平均を取る（CCW/CW 両方向で行うと精度が上がる）
3. apply_calib.py で反映（th_bringup/config/calib.yaml に保存）
4. 改めて linear_calib.py で確認 → 精度が目標に達するまで繰り返す

注意:
  wheel_radius と wheel_base は互いに影響するため、
  必ず直進 → 旋回の順でキャリブレーションすること。
```

### IMU 追加時の切替手順（TBD）

IMU（ESP32 の I2C に接続）が調達できた場合の切替手順です。

```bash
# 1. ESP32 ファームウェアを IMU 有効でビルド
#    config.h の IMU_SDA / IMU_SCL のコメントアウトを解除してビルド

# 2. ESP32 が /esp32/imu_data を発行することを確認
ros2 topic echo /esp32/imu_data

# 3. EKF を IMU 入力有効に切替えて起動
ros2 launch th_bringup bringup.launch.py imu_enabled:=true

# 4. EKF のキャリブレーション
#    robot_localization のドキュメントを参照し
#    ekf_params.yaml の process_noise_covariance を調整
```

---

## LiDAR 死角の実測と更新

20mm 角アルミパイプ 4 本による死角は取り付け位置・角度によって変わります。現場環境で正確に実測し、パラメータを更新してください。

```bash
# 1. ロボットを静止させ RViz2 で /scan を表示
ros2 run rviz2 rviz2
# LaserScan を追加し Fixed Frame を "laser_link" に設定

# 2. アルミ角柱が映る角度帯を読み取る
#    RViz2 の "Measurement" ツールで角度を計測
#    または:
ros2 run tf2_ros tf2_echo base_link laser_link  # フレームオフセット確認

# 3. perception_params.yaml を更新
#    例: 角柱が 43°〜47°, 133°〜137°, 223°〜227°, 313°〜317° の場合
```

```yaml
# src/th_bringup/config/perception_params.yaml
lidar_filter:
  ros__parameters:
    blind_angle_ranges:
      - 43.0   # 右前 開始
      - 47.0   # 右前 終了
      - 133.0  # 右後 開始
      - 137.0  # 右後 終了
      - 223.0  # 左後 開始
      - 227.0  # 左後 終了
      - 313.0  # 左前 開始
      - 317.0  # 左前 終了
```

```bash
# 4. 反映（ランタイムで変更も可能）
ros2 param set /lidar_filter blind_angle_ranges "[43.0, 47.0, 133.0, 137.0, 223.0, 227.0, 313.0, 317.0]"
```

---

## 配電盤座標の登録

地図作成後に RViz2 で各配電盤の前面座標（ロボットが停止する位置）を取得し、`panels.yaml` に登録します。

```bash
# RViz2 で "2D Pose Estimate" ツールを使い配電盤前に位置を指定
# → TF ツールで map 座標系の値を読み取る
ros2 topic echo /amcl_pose   # または
ros2 run tf2_ros tf2_echo map base_link
```

```yaml
# src/th_bringup/config/panels.yaml
panels:
  - id: "panel_01"
    name: "第1配電盤"
    x: 3.42        # 実測値を入力
    y: -1.15
    yaw: 1.5708    # 配電盤に向く方向 (rad)
```

`yaw` の値: 配電盤が北壁なら 0、東壁なら -1.5708（-π/2）、南壁なら 3.1416（π）、西壁なら 1.5708（π/2）。

---

## person_tracker 本番実装（human_kenchi ベース）

`person_tracker` の本番実装には
[`TAKA-MEER/human_kenchi`](https://github.com/TAKA-MEER/human_kenchi)（`sobits_follower` から
2D-LiDAR のみの脚検出・追跡部分を抜き出したワークスペース）を採用している。
`use_stub:=false` で以下のパイプラインが起動する。

```txt
/scan_filtered (死角マスク済み)
  → dr_spaam_ros (DR-SPAAM 脚検出, human_kenchi/2d_lidar_person_detection)
  → PersonTracker (leg モード, human_kenchi/multiple_sensor_person_tracking)
  → sobits_follower/multiple_sensor_person_tracking/following_position
  → person_tracker_bridge.py (th_perception)
  → /person/status (th_system_msgs/PersonStatus)
```

`person_tracker_bridge.py` が `following_position.status`
（`0=NO_EXISTS` / `1=EXISTS_LEG`）を `PersonStatus.is_lost` に変換する薄い変換ノード。
`th_ws/src/th_perception/scripts/person_tracker_bridge.py` を参照。

human_kenchi 自体の3パッケージ（`multiple_observation_kalman_filter` /
`multiple_sensor_person_tracking` / `leg_detection_bringup`）は
`th_ws/src/` に直接コミットしている（vendoring）。**human_kenchi はプライベートリポジトリ**
のため Dockerfile から `git clone` できず（認証情報が必要）、`sllidar_ros2` と同じ
外部依存パターンが使えなかったための対応。upstream の更新を取り込む場合は、
アクセス権のあるアカウントで手動 clone し、該当3パッケージのディレクトリを
上書きコミットすること。

DR-SPAAM（`TeamSOBITS/2d_lidar_person_detection`, 公開リポジトリ）は引き続き Dockerfile で
`git clone` して colcon ビルドに含めている（`sllidar_ros2` と同じパターン）。

### DR-SPAAM 重みファイルの配置（初回のみ）

DR-SPAAM の学習済み重み（`ckpt_jrdb_ann_ft_dr_spaam_e20.pth`, 実測 約 30 MB。配布元
フォルダには他モデルの重みも含めて複数ファイルがあるが、`weight_file` パラメータの
既定値であるこのファイルだけあれば動く）は配布元（Google Drive）の都合上 Docker
イメージには含めていない。以下の手順で手動配置する。

1. [重みファイルをダウンロード](https://drive.google.com/drive/folders/1Wl2nC8lJ6s9NI1xtWwmxeAUnuxDiiM4W)
2. リポジトリルートの `th_ws/dr_spaam_weights/` に置く（`.gitignore` 対象、コミット不要）
3. `docker-compose.yml` の bind mount によりコンテナ内
   `/root/th_ws/install/dr_spaam_ros/share/dr_spaam_ros/weights/` に自動反映される
   （`docker compose run --rm` の使い捨てコンテナでも消えない。実機検証で
   このマウント先パスが正しいことを確認済み）

> **既知の問題（対応済み）**: 配布されているチェックポイントは GPU (CUDA) 保存の
> ため、CPU 専用機（`use_gpu:false`）では素の `torch.load` が
> `Attempting to deserialize object on a CUDA device` で失敗する。Dockerfile 内で
> `dr_spaam/detector.py` に `sed` で `map_location` を明示するパッチを当てて回避済み
> （upstream 未対応のため、DR-SPAAM を再取得・再ビルドする際は要再適用）。

### 動作確認

```bash
ros2 launch th_bringup bringup.launch.py \
  map_yaml:=<地図パス> \
  use_stub:=false

# パイプライン各段の確認
ros2 topic hz /dr_spaam/dr_spaam_detections
ros2 topic echo sobits_follower/multiple_sensor_person_tracking/following_position
ros2 topic echo /person/status    # is_lost が適切に切り替わるか
ros2 topic hz /person/status      # 10 Hz 以上発行されているか
```

`use_stub:=true` の場合は上記パイプラインは一切起動せず、`person_tracker_stub.py` のみが
`/person/status` を発行する。

Nav2/ESP32等を含まない脚検知パイプライン単体だけを素早く確認したい場合は
「Windows (WSL2) + Docker Desktop での LiDAR 接続と RViz2 表示」節の
「person_tracker（脚検知）だけを素早く動作確認する」を参照（実機LiDAR単体で動作確認済み）。

### チューニング

障害物の陰に隠れた際の再取得挙動は `th_ws/src/leg_detection_bringup/param/leg_tracker_param.yaml`
で調整する。主要パラメータは human_kenchi の README を参照。LiDAR 死角（アルミ角柱）による誤検出が
出る場合は `perception_params.yaml` の `blind_angle_ranges` を先に見直すこと
（DR-SPAAM の入力は `/scan_filtered` のため死角マスクが有効）。

---

## カメラ昇降システムとの連携（TBD）

現状は `AT_PANEL` 到着通知（`/panel_navigator/arrived`）のみ実装しています。カメラ昇降システム側の設計が確定したら以下を追加実装します。

| 追加が必要な実装 | 対応ノード | 優先度 |
| --- | --- | --- |
| `/panel_navigator/arrived` のメッセージ内容確定 | `panel_navigator.py` | 高 |
| 昇降完了通知の受信 → `complete_inspection` サービス呼び出し | `panel_navigator.py` | 高 |
| `AT_PANEL → MANUAL` 中断時の同期確認（遷移前 OR 非同期通知の選択） | `mode_manager.cpp` + `panel_navigator.py` | 中 |
| `AT_PANEL` 中の近接警告（近接検知のみ、移動なし） | `safety_monitor.cpp` または新規ノード | 中 |

---

## 新しいモードの追加方法

実例: `FOLLOWING_MAPLESS`（MAP不要の軌跡追従モード）を追加した際の手順です。同じ手順で「自動巡回モード（AUTO_PATROL）」等を追加できます。

```txt
1. th_system_msgs/msg/RobotMode.msg に定数を追加
   uint8 AUTO_PATROL = 8   # FOLLOWING_MAPLESS(7) の次の空き番号を使う

2. mode_manager.cpp の isTransitionAllowed() に遷移ルールを追加
   （FOLLOWING_MAPLESS の実装例: IDLE/MANUAL からのみ遷移可、
    MOVING_TO_PANEL 等の地図前提の遷移は含めない）
   case RobotMode::IDLE:
     return to == RobotMode::FOLLOWING ||
            to == RobotMode::AUTO_PATROL;
   sub_fault_ のフォルト強制 IDLE 判定にも新モードを追加すること（安全上必須）。

3. 新ノード auto_patrol.py を th_planning/scripts/ に追加
   /robot/mode を購読し AUTO_PATROL 中のみ動作する
   （既存ノードは無条件起動のまま常時併存させ、RobotMode の排他性で
    衝突を防ぐ。follow_planner.py / follow_planner_mapless.py が同じ
    パターン）

4. th_planning/CMakeLists.txt の install(PROGRAMS ...) に追加
   th_bringup/launch/bringup.launch.py と gazebo.launch.py に
   ノードエントリを追加

5. th_testing/test/test_mode_transitions.py に遷移テストを追加

6. web_ui/src/App.jsx（MODE 定数・ボタン・modeColor）と
   web_ui/src/hooks/useRosbridge.js（MODE_NAMES）に追加
```

新しいモードは常に「ESTOP からは IDLE のみ経由で復帰」「IDLE への安全側遷移を持つ」「フォルト発生時は IDLE へ強制遷移する」という設計方針に従ってください。

---

## パラメータチューニングガイド

### 追従ロジック — FOLLOWING（planning_params.yaml の `follow_planner`）

```txt
lookback_distance: 1.0m
  → 追従がロボットに近づきすぎる場合は大きく（1.5m 等）
  → 追従の反応が鈍い場合は小さく（0.7m 等）

d_prepare: 3.0m
d_evade: 2.0m
distance_hysteresis_m: 0.2m
  → d_prepare/d_evade の差が PREPARE 状態の距離帯の広さ
  → distance_hysteresis_m がハンチング防止幅。ハンチングが観測されたら大きく（0.4m 等）
  → 退避が早すぎる/遅すぎる場合は d_evade を調整

evade_scan_directions: 16
evade_scan_max_dist: 3.0m
retreat_check_clearance: 0.5m
  → 退避方向が不自然な場合は evade_scan_directions を増やして分解能を上げる
  → 狭所で退避不可（stop）が頻発する場合は retreat_check_clearance を小さく

retreat_speed: 0.15 m/s
  → 退避が遅すぎる/速すぎる場合に調整（通常追従速度の50〜70%が目安）
```

### MAP不要追従ロジック — FOLLOWING_MAPLESS（planning_params.yaml の `follow_planner_mapless`）

```txt
lookback_distance: 1.0m
  → follow_planner と同様の目安

stop_distance: 1.0m
resume_distance: 1.3m
  → 差（0.3m）がハンチング防止幅。頻繁に停止/再開を繰り返す場合は差を大きく
  → 停止が遅すぎる場合は stop_distance を大きく

obstacle_check_distance_m: 1.0m
obstacle_check_half_width_deg: 20.0°
  → 障害物での停止が頻発する場合（誤検知）は距離・角度幅を小さく
  → 停止が遅い（間に合わない）場合は obstacle_check_distance_m を大きく

v_max: 0.3 m/s
  → 走行速度の上限。現場の安全要件に応じて調整
```

### フォルト検知タイムアウト（safety_monitor.yaml）

```txt
lidar_timeout_ms: 500ms
esp32_timeout_ms: 500ms
  → 電磁ノイズ・ジッタによる誤検知が多い場合は大きく（800ms 等）
  → 故障への反応が遅い場合は小さく（300ms 等）
  → ESP32 ウォッチドッグ(300ms)よりは必ず大きく設定すること

startup_grace_sec: 3s（シミュレーションでは safety_monitor_sim.yaml で 7s に上書き）
  → 起動直後のタイムアウト誤検知を抑制する猶予時間
  → Gazebo の spawn_delay(4.5s) + 初期化時間を考慮してシミュレーションでは大きく設定する
```

### ESP32 PID ゲイン（config.h）

```txt
初期値: Kp=80-100, Ki=30, Kd=8
チューニング手順:
  1. Ki=0, Kd=0 で Kp のみ調整 → 振動しない最大値を探す
  2. Ki を少しずつ増やして定常偏差を減らす
  3. Kd を増やして過渡応答を改善する
  4. 左右で別々に調整（旋回精度に影響する）
```

---

## トラブルシューティング

### ロボットが動かない

```bash
# 1. E-Stop が解除されているか確認
ros2 topic echo /safety/estop   # false であるべき

# 2. twist_mux の出力を確認
ros2 topic echo /cmd_vel        # ゼロでないか

# 3. ESP32 との通信を確認
ros2 topic hz /esp32/wheel_feedback  # 10 Hz 程度で来ているか

# 4. モードを確認
ros2 topic echo /robot/mode     # FOLLOWING/MANUAL であるべき
```

### 追従がぎこちない（ハンチング）

```bash
# デッドゾーンを広くする（FOLLOWING）
ros2 param set /follow_planner goal_deadzone_m 0.5

# 状態遷移のヒステリシス幅を広くする（FOLLOWING）
ros2 param set /follow_planner distance_hysteresis_m 0.4

# 停止/再開のヒステリシス幅を広くする（FOLLOWING_MAPLESS）
ros2 param set /follow_planner_mapless resume_distance 1.6
```

### LiDAR が誤認識する

```bash
# 死角フィルターのパラメータをランタイムで調整
ros2 param set /lidar_filter blind_angle_ranges \
  "[43.0, 47.0, 133.0, 137.0, 223.0, 227.0, 313.0, 317.0]"

# /scan と /scan_filtered を同時に RViz2 で比較
```

### ESP32 が頻繁に再接続する

```bash
# esp32_bridge の WebSocket 接続ログを確認 (接続/切断イベントが出力される)
ros2 topic echo /rosout | grep esp32_bridge

# ESP32 側のシリアルモニタで WiFi/WS 状態を確認
# (現行構成では ESP32 はラズパイに USB 接続されているため、ラズパイ側で確認する)
pio device monitor    # または ラズパイ上で /dev/ttyUSB1 を 115200 で読む

# ★5 分周期の切断→即再接続は「定期リフレッシュ」で正常動作(仕様)

# AP 構成の場合: PC の固定IP (192.168.4.50) と portproxy (8766→8765) が
# wifi_credentials.h の WS_SERVER_HOST/PORT と一致しているか確認
# STA 構成の場合: ホットスポットの SSID/パスワードが一致しているか確認
# PC 側ファイアウォールが ws_port をブロックしていないか確認

# ウォッチドッグタイムアウトを確認
# config.h: WATCHDOG_MS が通信周期より十分大きいか確認
```

### Nav2 が経路を計画できない

```bash
# costmap を RViz2 で確認
ros2 run rviz2 rviz2
# /local_costmap/costmap と /global_costmap/costmap を表示

# ロボットの位置推定を確認
ros2 topic echo /odom | head -20

# SLAM のマッチング状態確認
ros2 topic echo /slam_toolbox/scan_matched
```
