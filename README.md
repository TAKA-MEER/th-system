# TH システム — 配電盤上部確認ロボット 移動機構

2D-LiDAR による脚検知で試験員を追従するクローラーロボット。
ESP32(モーター制御)+ ラズパイ(LiDAR)+ PC(ROS2 Humble / Docker)の3台構成。

```txt
ESP32 (駆動用, WiFi AP)          192.168.4.1   SSID: th-esp32-ap
  ├── PC (Windows+WSL2, ROS2)   192.168.4.50  (固定IP)
  │     ESP32 → WebSocket: 192.168.4.50:8766 (コンテナ内 esp32_bridge が直接待ち受け)
  └── ラズパイ (LiDAR 配信)      192.168.4.2   (DHCP)
        RPLIDAR S1 → rplidar_ros → /scan (ROS_DOMAIN_ID=10, frame_id=laser_link)
```

> **上図は旧構成 (ESP32 が AP)。2026-09-02 実機の現行構成は「ラズパイが AP」**:
>
> ```txt
> ラズパイ (WiFi AP + LiDAR)     192.168.5.1    SSID: th-rpi-ap
>   ├── PC (ROS2/Docker)         192.168.5.50   esp32_bridge が :8766 で待ち受け
>   └── ESP32 (駆動)             192.168.5.125  STA 子機 (DHCP) → PC:8766 へ接続
> ```
>
> [docs/network.md](docs/network.md) は ESP32-AP 前提のままで**全面的に古い**（要更新）。
> なお構内 AP が ch1 に 10 局あり、この 2.4GHz リンクは平常時でもロス 20〜30% ある
> （ESP32 の断続的な切断の主因。AP を ch6 か 5GHz へ移すのが本命の対処）。


| レイヤー         | 実装                                                         |
| ------------------ | -------------------------------------------------------------- |
| ハードウェア制御 | ESP32 (PlatformIO + WebSocket クライアント, PID+FF 速度制御) |
| ROS2 ブリッジ    | `th_esp32_bridge` (WS サーバー・オドメトリ)                  |
| 安全管理         | `th_safety` (safety_monitor) + `twist_mux`                   |
| 状態管理         | `th_mode_manager` (FSM)                                      |
| 認識             | `th_perception` + DR-SPAAM 脚検知 (human_kenchi)             |
| 計画・追従       | `th_planning` (follow_planner / mapless)                     |
| ナビゲーション   | Nav2 + SLAM Toolbox + robot_localization                     |
| UI               | React + rosbridge WebSocket                                  |

---

## 毎回の起動手順

前提: 初回セットアップ([docs/setup.md](docs/setup.md))済み。詳細は [docs/operation.md](docs/operation.md)。

```powershell
# ① ロボット・ラズパイの電源 ON。PC を SSID "th-rpi-ap" に接続して疎通確認
#    2026-09-02 実機確認の構成 (docs/network.md の ESP32-AP 構成は古い):
#      ラズパイ 192.168.5.1   … AP 本体 + /scan 配信元 (systemd で自動起動)
#      PC       192.168.5.50  … esp32_bridge が :8766 で待ち受け
#      ESP32    192.168.5.125 … STA 子機 (DHCP)。PC:8766 へ繋ぎに来る
ping 192.168.5.1     # ラズパイ (AP 兼 LiDAR)
ping 192.168.5.125   # ESP32 (IP は DHCP。ss -tnp | grep 8766 でも確認できる)
# 繋がらない → netsh wlan disconnect → connect (docs/network.md「復旧手順」)

# Wi-Fiのバックグラウンドスキャンの無効化

netsh wlan set autoconfig enabled=no interface="<アダプタ名>"   # 走行前
# netsh wlan set autoconfig enabled=yes interface="<アダプタ名>"  # 終了後


# ② (推奨) WSL をクリーンに起動
wsl --shutdown
```

```bash
# ③ WSL2 (Ubuntu) ターミナルで th_ws/ にて
docker start th_robot
docker exec -it th_robot bash

# ④ コンテナ内で bringup (地図なし=SLAM モード。地図ありは map_yaml:=... を追加)
cd /root/th_ws
colcon build --symlink-install     # C++ 変更時のみ必須。Python のみの変更は --symlink-install で即反映
source install/setup.bash
ros2 launch th_bringup bringup.launch.py lidar_source:=network use_stub:=false
# IMUあり
ros2 launch th_bringup bringup.launch.py lidar_source:=network use_stub:=false imu_enabled:=true

# 手動教示・教示再生デモ (feat/demo-teach-replay): slam_toolbox を mapping モードで
# 起動し、教示中も再生中も map→odom を連続補正する。教示画面に地図と LiDAR 点群が出る。
ros2 launch th_bringup bringup.launch.py lidar_source:=network use_stub:=false enable_route_slam:=true

# 駆動系だけのキーボード操作テスト (LiDAR・安全監視なしの最小構成)
ros2 launch th_bringup esp32_keyboard_test.launch.py

# ⑤ 健全性確認: 起動直後の [FAULT] は 1〜2 分で全て [FAULT CLEARED] になる。
#    ならないフォルトがあれば docs/network.md の復旧手順へ
ros2 topic echo /robot/mode --once     # mode: 1 (IDLE) = 正常
ros2 topic hz /scan_filtered           # network 時: 10Hz 出ていること。無音なら docs/network.md へ

# ⑥ モード切替 (タブレット UI または CLI)
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 7, requester: 'cli'}"    # 7=FOLLOWING_MAPLESS, 2=FOLLOWING, 1=IDLE
```

うまくいかない時の早見表:


| 症状                                     | 対処                                                                                            |
| ------------------------------------------ | ------------------------------------------------------------------------------------------------- |
| AP に繋がらない / ping 不可              | WiFi 切断→再接続。autoconfig 無効のままなら有効化してから →[docs/network.md](docs/network.md) |
| LIDAR_LOST が消えない                    | ラズパイの rplidar 再起動・時刻ズレ確認 →[docs/network.md](docs/network.md)                    |
| ESP32_DISCONNECTED が消えない            | PC の固定 IP・portproxy 残骸確認 →[docs/network.md](docs/network.md)                           |
| CLI がトピックを見つけない・部分的に不通 | `docker restart th_robot` → だめなら `wsl --shutdown` からやり直し                             |
| 地図が生成されない/ノイズだらけ          | PC の時刻ズレ →[docs/setup.md §4](docs/setup.md)                                              |

---

## ドキュメント目次


| やりたいこと                                             | ドキュメント                                          |
| ---------------------------------------------------------- | ------------------------------------------------------- |
| システムの完成形・実装状況を知る                         | [VISION.md](VISION.md)                                |
| 新しい PC/ラズパイ/ESP32 で環境を作る                    | [docs/setup.md](docs/setup.md)                        |
| ネットワークの仕組み・通信トラブルの復旧                 | [docs/network.md](docs/network.md)                    |
| ESP32 ファームの書き込み・PID チューニング               | [docs/esp32.md](docs/esp32.md)                        |
| 地図作成・キャリブレーション・追従の使い方・トラブル対処 | [docs/operation.md](docs/operation.md)                |
| Gazebo シミュレーションで検証する                        | [docs/simulation.md](docs/simulation.md)              |
| テストを実行する・追加する                               | [docs/testing.md](docs/testing.md)                    |
| 内部設計・パラメータ調整・機能追加                       | [docs/architecture.md](docs/architecture.md)          |
| 音声アナウンスのクレジット・利用条件                     | [docs/voice-credits.md](docs/voice-credits.md)        |
| 音声の話者を聞き比べて選ぶ                               | [docs/voice-audition/](docs/voice-audition/README.md) |

---

## クレジット

音声アナウンスの音声合成に **VOICEVOX Nemo**（[https://voicevox.hiroshiba.jp/nemo/](https://voicevox.hiroshiba.jp/nemo/)）を使用しています。

利用条件・遵守事項の詳細は [docs/voice-credits.md](docs/voice-credits.md) を参照してください。

---

## モード / フォルト早見表


| モード            | 番号 | 状態                           |
| ------------------- | ------ | -------------------------------- |
| IDLE              | 1    | 静止待機(起動時の初期状態)     |
| FOLLOWING         | 2    | 試験員追従(地図・Nav2 使用)    |
| MOVING_TO_PANEL   | 3    | 配電盤へ移動中                 |
| AT_PANEL          | 4    | 配電盤前作業中                 |
| MANUAL            | 5    | 手動操作(タブレット)           |
| ESTOP             | 6    | 緊急停止(復帰は IDLE 経由のみ) |
| FOLLOWING_MAPLESS | 7    | 試験員追従(地図・Nav2 不要)    |


| フォルト              | 意味           | 詳細                                                              |
| ----------------------- | ---------------- | ------------------------------------------------------------------- |
| `LIDAR_LOST`          | /scan 途絶     | [docs/operation.md](docs/operation.md#モード早見表--フォルト対応) |
| `ESP32_DISCONNECTED`  | ESP32 通信途絶 | 同上                                                              |
| `PERSON_TRACKER_LOST` | 追従データ途絶 | 同上                                                              |

フォルト発生時は twist_mux が即座にモーター出力をゼロにし、IDLE へ強制遷移する
(検知〜物理停止は ESP32 ウォッチドッグ 600ms が最終保証)。

---

## ディレクトリ構成

```txt
th-system/
├── README.md                     # 本書 (毎回の起動手順・目次)
├── VISION.md                     # 完成形の記述・実装状況 (方針変更時は必ずここを先に更新)
├── docs/                         # 詳細ドキュメント (上記目次)
└── th_ws/
    ├── Dockerfile / docker-compose.yml
    ├── esp32/                    # ESP32 ファームウェア (PlatformIO)
    ├── udev/ / scripts/ / dr_spaam_weights/
    ├── src/                      # ROS2 ワークスペース
    │   ├── th_system_msgs/       # カスタム型 (RobotMode/PersonStatus/FaultStatus/WheelFeedback)
    │   ├── th_esp32_bridge/      # ESP32 ↔ ROS2 (WebSocket サーバー・オドメトリ)
    │   ├── th_safety/            # safety_monitor + twist_mux 設定
    │   ├── th_mode_manager/      # モード FSM
    │   ├── th_perception/        # lidar_filter・person_predictor・tracker_bridge・stub
    │   ├── th_planning/          # follow_planner(_mapless)・panel_navigator・teleop
    │   │   └── th_planning/      # ROS2 非依存のコアロジック (pytest 対象)
    │   ├── th_calibration/       # オドメトリキャリブツール
    │   ├── th_description/       # URDF (base_link→laser_link 等の TF)
    │   ├── th_bringup/           # launch・地図・全パラメータ YAML
    │   ├── multiple_*_tracking/ leg_detection_bringup/   # vendored (human_kenchi)
    │   └── th_testing/           # 単体・統合テスト
    └── web_ui/                   # React タブレット UI (roslib ローカル同梱)
```
