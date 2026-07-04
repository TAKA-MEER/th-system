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

RPLIDAR をロボット本体ではなく別マシン(ラズパイ等)に接続し、そちらが配信する `/scan` をネットワーク経由で受信する構成にも対応している。

```bash
# ローカルの sllidar_node 起動を止め、外部が配信する /scan を使う
ros2 launch th_bringup bringup.launch.py lidar_source:=network
ros2 launch th_bringup slam.launch.py    lidar_source:=network
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
- **注意**: `config.h` の `ESTOP_BENCH_TEST_BYPASS` は現在無効化されているため、E-Stopスイッチ未配線だと GPIO34 がフローティングになり `ESTOP_HW` の値が不安定になりうる(`WHEEL_FEEDBACK` はE-Stop有効中は送信されないため届かないことがある)。これは既知の挙動で、通信テスト自体には影響しない。動力系を配線せず確実に安定した挙動を見たい場合のみ、一時的に `ESTOP_BENCH_TEST_BYPASS` を再定義して試験し、試験後は必ず戻すこと。

### E-Stop 配線

```tree
物理スイッチ (モーター電断端子)
    └── GPIO 34 (ESP32)
    └── GND
```

`config.h` の `ESTOP_LOW_ACTIVE` をスイッチ OFF 時の論理に合わせて設定すること。

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

## 起動手順

### Step 1: 地図作成 (初回のみ)

```bash
# コンテナ起動
docker compose run --rm th_robot bash

#ビルド

# コンテナ内
cd /root/th_ws
colcon build --symlink-install   # 初回または C++ 変更時
source install/setup.bash

# 駆動系キーボード操作テスト

ros2 launch th_bringup esp32_keyboard_test.launch.py

# コンテナ内
ros2 launch th_bringup slam.launch.py

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
ros2 launch th_bringup bringup.launch.py \
  map_yaml:=/root/th_ws/src/th_bringup/maps/th_map.yaml \
  use_stub:=false

# テスト時 (試験員追従スタブ使用)
ros2 launch th_bringup bringup.launch.py use_stub:=true
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
| FOLLOWING | 試験員自動追従 | 「追従開始」ボタン |
| MOVING_TO_PANEL | 配電盤移動中 | 配電盤ボタン |
| AT_PANEL | 配電盤前作業中 | 自動遷移 |
| MANUAL | 手動操作 | 「手動操作」ボタン |
| ESTOP | 緊急停止 | 緊急停止ボタン / 物理スイッチ |

---

## フォルト対応

| フォルト | 原因 | 対処 |
| --- | --- | --- |
| `LIDAR_LOST` | LiDAR データ途絶 | ケーブル・ドライバ確認 → 再起動 |
| `ESP32_DISCONNECTED` | ESP32 通信途絶 | USB 抜き差し → ファームウェア確認 |
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
│   ├── th_mode_manager/          # FSM（7 状態・全遷移ルール）
│   ├── th_perception/            # lidar_filter・person_predictor・tracker_stub
│   ├── th_planning/              # follow_planner・panel_navigator・manual_handler・crawler_teleop
│   │   └── th_planning/          # Python パッケージ（テスト可能なコアロジック）
│   │       └── follow_planner_core.py
│   ├── th_calibration/           # オドメトリキャリブツール
│   ├── th_bringup/               # 起動設定・地図・パラメータ
│   └── th_testing/               # 単体・統合テスト
│       └── test/
│           ├── conftest.py
│           ├── test_follow_planner_logic.py
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

> **注意 (シミュレーションのみの場合):** `docker-compose.yml` に実機デバイス (`/dev/lidar`) が記載されており、デバイスが存在しないとコンテナ起動に失敗することがある。その場合は `docker-compose.yml` の `devices:` セクションをコメントアウトするか削除すること。

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
- [ ] `person_tracker` 本番実装への切替 (`use_stub:=false`)
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
| `test_follow_planner_logic.py` | 純粋単体テスト | 不要 | 41 件 | 10.1 §3 |
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
  -v -k "TestRetreatHysteresis"

# 失敗時に即座に停止
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py -x
```

テスト対象クラスと確認内容:

```txt
TestRetreatHysteresis   ── trigger/release 境界値・ハンチング防止・速度による予防的退避
TestPositionHistory     ── 速度推定精度・接近速度の正負・履歴不足時のゼロ返却
TestStaticDetector      ── 静止判定タイミング・移動再開によるリセット
TestFovChecker          ── 360° FOV・死角マスク・オフセット縮小ロジック
TestCandidatePoints     ── 作業前方除外・候補点スコアリング
TestComputeRetreatCmd   ── 後退方向・速度・壁際での None 返却
TestComputeFollowGoal   ── 狭路/広空間の角度切替・ゴール姿勢方向
TestFollowPlannerCore   ── 優先順位(退避>静止>通常)・デッドゾーン・状態遷移
TestCorridorWidth       ── 開放空間/狭路の幅推定精度
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

- **Scenario A-1**: 試験員がトリガー距離以内に接近 → `/cmd_vel_retreat` に後退指令が発行される
- **Scenario A-2**: 壁際（costmap でブロック）での接近 → その場停止
- **Scenario A-3**: 試験員が離れると退避解除 → 通常追従に戻りハンチングしない
- **Scenario C**: ロスト後の予測外挿 → `max_predict_sec` 後に `/person/search_mode=true`
- **Scenario D**: MANUAL 中は退避が作動しない（試験員が近くにいても後退しない）

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

## 試験員追従ロジック（follow_planner_core）

### 内部状態と優先順位

`FollowPlannerCore.update()` は毎制御周期（10 Hz）に呼ばれ、以下の優先順位で出力を決定します。

```txt
1. RETREAT（近接退避）   ← 最優先・安全関連
2. STATIC_REPOSITION     ← 静止時の作業スペース配慮
3. NORMAL_FOLLOW         ← 通常追従
```

各状態で何をするかは `follow_planner_core.py` の `FollowPlannerCore.update()` を参照してください。

### 退避ヒステリシス（ハンチング防止）

`RetreatHysteresis` は `trigger_distance`（退避開始）と `release_distance`（退避解除）を分けることでハンチングを防ぎます。`trigger=0.8m`、`release=1.2m` の場合、0.8m 以内に入ると退避を始め、1.2m を超えるまで退避を継続します。この差（0.4m）がヒステリシス幅です。現場でハンチングが観測された場合は `release_distance` を大きくしてください。

### 静止検知と再配置

試験員の速度が `static_speed_threshold`（0.05 m/s）を `static_time_threshold`（2.0 秒）以上下回った場合に「静止」と判定します。静止判定後は試験員を中心に放射状に `candidate_count`（16 個）の候補点を生成し、以下のスコアで最適点を選びます。

```txt
スコア = 移動コスト（現在地からの距離）
       + costmap ブロックペナルティ（障害物方向は大きなペナルティ）
       + FOV 違反ペナルティ（LiDAR 視野角を外れる方向は大きなペナルティ）
```

試験員の正面方向（配電盤側）は `work_front_exclusion_deg`（±60°）で候補から除外されます。これにより作業スペースへの干渉を防ぎます。

### 通路幅判定と角度オフセット

`estimate_corridor_width()` はロボットの横方向に costmap をプローブし、両側の障害物までの距離の和を通路幅とします。`corridor_width_threshold`（1.5m）未満であれば真後ろ追従（オフセット 0°）、それ以上であれば斜め後方追従（最大 `follow_angle_offset_max`=30°）を選択します。斜め後方追従は試験員の視界に入りやすくコミュニケーションを取りやすい位置取りです。

### LiDAR 死角対応

`FovChecker` は `blind_angle_ranges` に設定された角度帯（アルミ角柱による死角）を除外します。追従角度オフセットを適用した結果が死角に入る場合、`clamp_offset()` がオフセットを 0 まで縮小し視野角を確保します。これは 4.3.1 節の「LiDAR 視野角内に試験員を収める（必須制約）」を実装したものです。

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

## person_tracker 本番実装への切替

ML ベースの `person_tracker` 実装が完成したら以下の手順で切替えます。

### 必要なインターフェース

本番の `person_tracker` は以下のトピックを発行してください。

```txt
/person/status  (th_system_msgs/PersonStatus)
  header.stamp    : 現在時刻
  header.frame_id : "base_link"
  position.x      : 試験員の前方距離 [m]（ロボット正面方向）
  position.y      : 試験員の横方向距離 [m]（左が正）
  position.z      : 0.0
  confidence      : 検出信頼度 0.0〜1.0
  is_lost         : 検出できていない場合 true
  lost_reason     : "DETECTION_LOST" / "LOW_CONFIDENCE" / ""
```

`is_lost` フラグは以下のタイミングで `true` にしてください。

- 検出対象が LiDAR スキャン内に見つからない場合
- `confidence` が一定閾値（例: 0.3）を下回った場合

### 切替コマンド

```bash
# bringup launch の use_stub を false に変更
ros2 launch th_bringup bringup.launch.py \
  map_yaml:=<地図パス> \
  use_stub:=false      # ← ここを変更

# 動作確認
ros2 topic echo /person/status    # is_lost が適切に切り替わるか
ros2 topic hz /person/status      # 10 Hz 以上発行されているか
```

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

例として「自動巡回モード（AUTO_PATROL）」を追加する場合の手順です。

```txt
1. th_system_msgs/msg/RobotMode.msg に定数を追加
   uint8 AUTO_PATROL = 7

2. mode_manager.cpp の isTransitionAllowed() に遷移ルールを追加
   case RobotMode::IDLE:
     return to == RobotMode::FOLLOWING ||
            to == RobotMode::AUTO_PATROL;

3. 新ノード auto_patrol.py を th_planning/scripts/ に追加
   /robot/mode を購読し AUTO_PATROL 中のみ動作する

4. th_bringup/launch/bringup.launch.py に auto_patrol ノードを追加

5. th_testing/test/test_mode_transitions.py に遷移テストを追加

6. web_ui/src/App.jsx に操作ボタンを追加
```

新しいモードは常に「ESTOP からは IDLE のみ経由で復帰」「IDLE への安全側遷移を持つ」という設計方針に従ってください。

---

## パラメータチューニングガイド

### 追従ロジック（planning_params.yaml）

```txt
follow_distance_target: 1.5m
  → 近づきすぎる場合は大きく（2.0m 等）
  → 離れすぎる場合は小さく（1.2m 等）

retreat_trigger_distance: 0.8m
retreat_release_distance: 1.2m
  → trigger と release の差（0.4m）がヒステリシス幅
  → ハンチングが観測されたら release を大きく（1.5m 等）
  → 退避が遅すぎる場合は trigger を大きく（1.0m 等）

closing_speed_threshold: 0.3 m/s
  → 急接近を見逃す場合は小さく（0.2 m/s 等）
  → 誤作動が多い場合は大きく（0.4 m/s 等）

static_time_threshold: 2.0s
  → 試験員が立ち止まるとすぐ再配置が発動する場合は大きく（3.0s 等）
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
# デッドゾーンを広くする
ros2 param set /follow_planner goal_deadzone_m 0.5

# ヒステリシス幅を広くする
ros2 param set /follow_planner retreat_release_distance 1.5
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

# ESP32 側のシリアルモニタで WiFi RSSI・WS 接続状態を確認
pio device monitor

# ホットスポットの SSID/パスワードが wifi_credentials.h と一致しているか確認
# PC 側ファイアウォールが ws_port (config/params.yaml の ws_port) を
# ブロックしていないか確認

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
