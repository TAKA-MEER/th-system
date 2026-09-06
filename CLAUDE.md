# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 作業開始前のルール

**実装作業に入る前に必ず `git status` を確認し、作業前から存在する未コミットの変更がないか調べること。**

- 未コミットの変更があれば、着手前にユーザーへ提示して扱いを確認する（先にコミットするか、そのまま残すか）。ユーザーが別のターミナルや別セッションで進行中の作業であることが多い。
- 勝手にコミットも破棄もしない。確認せずに作業を始めると、こちらの変更と混ざって切り分けられなくなる。
- コミット時は自分が変更したファイルだけをステージする。`git add -A` / `git add .` は作業前からあった変更を巻き込むため使わない。
- この環境では `git add -p` が使えない（対話的フラグ非対応）ため、1つのファイルに複数の関心事の変更を混ぜると後からコミットを分割できない。無関係な変更を同じファイルに同時に入れないよう作業順序を組む。

## 方針変更時のルール

このリポジトリでは `VISION.md`(README.md と同じ階層)に、ユーザーが目指す「完成形」(最終的なシステム像・挙動・要件)を記述している。

**追従ロジック・モード遷移・安全設計・アーキテクチャ全般について、ユーザーから方針変更の指示があった場合は、コードを修正する前に必ず `VISION.md` を該当箇所から更新すること。** コード修正はその後に行う。VISION.md とコードの内容に矛盾が生じた場合は、ユーザーに確認しどちらが正か明確にしてから作業を進める。

`docs/architecture.md` は現状実装の保守・拡張ガイド(as-built)であり、`VISION.md` とは役割が異なる(目指す姿 vs 今の実装)。両者が食い違う場合は VISION.md 側を優先して実装を追いつかせる。

`docs/plan/` は**未確定の検討メモ**を置く場所で、VISION.md を上書きしない。書き方のルール（本体は結論と表だけにして一目で読める分量を保ち、根拠・詳細は `<テーマ>-<側面>.md` に分ける）は `docs/plan/README.md` に定義してある。plan 配下を編集する前に必ず読むこと。

## 実機マニュアルの保守ルール

`docs/使い方.md` は**実装を知らない試験担当者が実機を動かすための常設マニュアル**で、
`docs/試験項目.md` は**その都度書き換える「今回何を確かめるか」**。2 つは役割が違う。

- **実機の挙動・操作手順・画面の文言・パラメータの既定値・確認コマンドを変えたら、
  同じコミットで `docs/使い方.md` も直すこと。** 古いマニュアルは無いより悪い。
- `docs/使い方.md` に「今日は何を試すか」を書かない。それは `docs/試験項目.md` の役割。
- `docs/試験項目.md` に動かし方を書かない。`docs/使い方.md` の該当節へリンクする。
- `docs/operation.md` は旧 9 モード体系の記述が残っている別系統の文書。混ぜない。

## このファイル自体の保守ルール

- 作業中に判明したこのプロジェクト固有の環境の癖・落とし穴（コマンドの意外な挙動、ツールの制約など）は、ユーザーに確認せず「環境の癖」セクションに追記してよい。
- **CLAUDE.md を更新するたびに、ファイル全体を読み直し、陳腐化した記述・重複・冗長な説明がないか見直すこと。** コンテキストを圧迫しないよう、価値の下がった記述は削除するか簡潔にまとめる。肥大化を優先して情報を積み増すだけにしない。

## 環境の癖・注意点

- **`pip3 install platformio` をホストの `python3 -m pytest` と同じ環境に入れると、依存の `anyio` が pytest プラグインとして自動登録され、`ModuleNotFoundError: No module named '_pytest.scope'` でテストが全滅する**（この環境の `pytest` は 6.2.5 で `anyio` の新しめのプラグインAPIと非互換）。`python3 -m pytest -p no:anyio ...` で回避できる。ESP32 ファームを `pio run` でビルドしたい場合に踏む（2026-09-05）。
- **ラズパイ (`mirs2602@192.168.5.1`) には `pip3` が無く、`ip route` に default gateway も無いためインターネットに出られない。** Python ライブラリが要る場合は、開発機で `pip3 download --platform manylinux2014_aarch64 --python-version 310 --implementation cp --only-binary=:all: <pkg>` して wheel を展開し、`PYTHONPATH` で読ませる（`docs/network.md`「ラズパイ: pi_serial_relay の導入」に実例）。`sudo` もパスワードが必要で非対話 SSH からは実行できない（systemd unit のインストール等は対話セッションでやる必要がある）。
- **pyserial の `ser.read(size)` は `timeout` の間「`size` バイト溜まるまで」待ち続ける実装**であり、`size > 1` に有限 `timeout` を組み合わせると必ず `timeout` の粒度で足止めされる固定ポーリングになる。ESP32 側が 100ms 周期で送信しているのに `pi_serial_relay.py` が `ser.read(4096)` を `timeout=0.05` で呼んでいたところ、2 つの周期がビート（うなり）を起こして `/esp32/wheel_feedback` が毎周期バースト受信になった（2026-09-05 実機で発覚。教示再生のふらつき増加の原因だった）。低遅延・低ジッタが要る受信は `ser.read(1, timeout=None)`（無期限待ち）→即座に `ser.in_waiting` 分だけノンブロッキングで追い読み、の2段構えにする。
- `ros2 node list` はデーモンキャッシュの影響で新規ノードが反映されないことがある。`ros2 node list --no-daemon`（または `ros2 daemon stop` 後に再実行）で確実に最新状態を取得する。
  **ただし `--no-daemon` でも生きているノードを取りこぼすことがある。**「一覧に出ない」だけで死んだと判断しないこと。launch のログに `process has died` が無いか、当該ノードの起動 INFO が出ているかを併せて見る（`lidar_filter` が正常起動しているのに一覧に出ず、誤って「修正が効いていない」と判断しかけた）。
- **`th_robot` コンテナはユーザーが実機作業中のセッションであることがある。** デバッグ用にノードを起動・停止する前に必ず `docker exec th_robot ps -eo pid,etimes,args` で稼働中のプロセスを確認し、自分が起動したものだけを PID 指定で止めること（実際に `rotation_calib.py` が 50 分間走っている最中に遭遇した）。
- `docker exec th_robot bash -lc '... pkill -f <pattern> ...'` は、パターンがこのシェル自身のコマンドライン（`-lc` の引数文字列全体）にマッチして**自分を殺す**。出力が一切出ず exit 143 になったらこれを疑う。スクリプトをファイルに書いてから実行するか、PID 指定で止める。
- 長時間動くノード（`component_container_mt` 等）を `docker exec` から `&` で起動すると、シェル終了時に道連れになる。`setsid ... > log 2>&1 < /dev/null &` で切り離す。
- **コンテナ内で launch を起動すると `th_ws/data/generated/` が root 所有で書き換わる。**`th_ws/data` は `/root/th_data` にバインドマウントされており、`params_generation` の生成先（`/root/th_data/generated`）がそこに含まれる。生成物は tracked なので `git status` に `M` が並び、しかもホスト側ユーザでは `git checkout --` すら「許可がありません」で失敗する。`docker compose run --rm th_robot bash -lc 'chown -R 1000:1000 /root/th_data'` で所有権を戻してから復元する。
- **`docker-compose.yml` はリポジトリ直下ではなく `th_ws/` にある。** リポジトリルートから `docker compose run ...` を実行すると `no configuration file provided: not found` で即死する。必ず `th_ws/` から実行すること（コンテナ名が `th_ws-th_robot-run-*` になるのはこのため）。
- **`docker compose run --rm th_robot` は毎回新しいコンテナを作り、`build/` と `install/` はバインドマウントされていない**（マウントは `src` / `esp32` / `scripts` / `data` / `dr_spaam_weights` のみ）。そのため `colcon build` と `colcon test` を別々の `docker compose run` で実行すると、テスト側からビルド成果が見えず `colcon test-result` が「0 tests」になる。**ビルドからテストまでを 1 回の `bash -lc` の中で通すこと。**
- **検証スクリプトに `set -u` を書かない。** `source /opt/ros/humble/setup.bash` が `AMENT_TRACE_SETUP_FILES: unbound variable` で即死する。`docker compose run -d`（デタッチ）で踏むと**出力が一切残らないまま `--rm` でコンテナごと消える**ので、原因が分からない。同じスクリプトを非デタッチで走らせるとメッセージが出る。
- **数分かかる Docker 検証は `docker compose run -d`（デタッチ）で回し、結果を `/root/th_data/`（＝`th_ws/data/`）へ書かせる。** `colcon build` ＋ launch は 5 分前後かかり、バックグラウンドのシェルタスクが途中で打ち切られると `--rm` でコンテナごと出力が消える（実際に 3 回取り逃した）。デタッチしてバインドマウント経由で結果を受け取れば、タスクが止まっても残る。待つときは Monitor の until ループを使う。コンテナ側の最後に `chown -R 1000:1000 /root/th_data` を入れておくと、生成物が root 所有で残るのを防げる。
- `test_simulation_scenarios.py` は **Gazebo を起動しない**（`generate_test_description()` が立てるのは `mode_manager` / `safety_monitor` / `person_predictor` / `follow_planner` の 4 ノードだけ）。**既定でスキップする**（`TH_SKIP_SIM=1`）。検証しているのは新設計で廃止済みの as-built 挙動（近接退避・捜索旋回）で、段階 3 の `WP-TRANSIT-01` で `follow_planner` ごと削除される見込み。復活させる前に必ずファイル冒頭の docstring を読むこと（A2 / A3 のアサーションが無力である事実を含む）。
- **テストの大半は Docker 不要でホストの `python3` から直接走る。**`th_ws/src/th_testing/test/` のうち ROS2 環境（`rclpy` / ビルド済み `th_system_msgs`）が要るのは次の 11 ファイルだけで、他は素の pytest で緑赤を判定できる（2026-09-01 時点で 517 passed / 1 skipped）。`colcon build` は数分かかるので、まずホストで回して最後に Docker で 1 回通すのが速い。
  除外する 11 ファイル: `test_connectivity_checker_node.py` / `test_fault_detection.py` / `test_mode_transitions.py` / `test_params_audit_node.py` / `test_safety_monitor.py` / `test_state_manager_node.py` / `test_twist_mux_priority.py` / `test_simulation_scenarios.py` / `test_msg_definitions.py` / `test_esp32_bridge_node.py` / `test_jog_gate_node.py`（後ろ 2 つは `launch_testing`）
- **`launch_testing` を使うテスト（`generate_test_description()` を持つファイル）は、本体が `unittest.TestCase` なので pytest のフィクスチャを一切受け取れない。** `conftest.py` が提供する値（`fault_params` 等）が要るときは、同じ解決ロジックをモジュールレベルで呼ぶこと。CMake 側は `add_launch_test` ではなく `ament_add_pytest_test` でそのまま登録できる（`esp32_bridge_node` / `fault_injection_12` が実例）。pytest の表示は `collected 1 item` になるが、junit には `TestCase` のメソッド数だけ結果が出る。
- `th_ws/esp32/.vscode/extensions.json` は **`.gitignore` に載っているのに tracked** という状態で、内容もモードも index と一致しているのに `git status` に `M` が出続けることがある（index の stat キャッシュが NTFS 時代の古いサイズを持っているため）。`git diff` が空なのに `M` が消えないときはこれ。`git add -f <path>` で解消でき、内容が同じなので差分はステージされない。
- **ノードを `kill -9` で落とすことを繰り返すと、コンテナ内の DDS discovery が壊れる。** 症状は「ノードは起動しログも出ているのに、他プロセスからサービス/トピックが一切見つからない」。`ls /dev/shm | wc -l` で `fastrtps_*` の残骸が溜まっているか確認する（ROS プロセスが 0 なのに大量にあれば該当）。`/dev/shm` の掃除だけでは直らないことがあり、その場合はコンテナ再起動が必要。デバッグ用ノードは `kill -TERM` で落とすこと。
- **`/scan_filtered` が実機で完全に無音になる複合バグ（2026-09-01 修正）。** 症状: 点群表示も slam_toolbox の地図生成も動かない。`ros2 topic hz /scan` は 10Hz 出るのに `lidar_filter` の `_cb` が一度も発火しない。原因は 2 つ:
  1. **`bringup.launch.py` が `lidar_filter`（network 時）に渡していた `FASTRTPS_DEFAULT_PROFILES_FILE`（`config/fastdds_profile.xml`）のユニキャスト初期ピアが `192.168.4.2` 固定だった。** ネットワークが `192.168.5.x` へ移行して**存在しないサブネット**になり、これが逆に discovery を壊した。→ additional_env を外し、マルチキャスト discovery（現行 AP では正常）に戻した。別 AP で不安定なら `fastdds_profile.xml` の `<address>` を現ラズパイ IP に直して再度渡す。
  2. `lidar_filter` の `/scan` 購読が既定 QoS（RELIABLE）だった。センサストリームは必ず `qos_profile_sensor_data`（BEST_EFFORT）で購読する（`safety_monitor` / `obstacle_limiter` / `connectivity_checker` は元から BEST_EFFORT）。
- **実機のネットワークは「ESP32 が AP」から「ラズパイが AP」へ変わっている**（`docs/network.md` / `setup.md` / `esp32.md` は 2026-09-02 に現行構成へ書き直し済み。192.168.4.x や `th-esp32-ap` が出てくる記述を見かけたら古い）。2026-09-02 に実機で確定した構成:

  | 機器 | IP | 役割 |
  |---|---|---|
  | ラズパイ | `192.168.5.1` | **AP 本体**（SSID `th-rpi-ap`）＋ `/scan` 配信元 |
  | PC | `192.168.5.50` | USB WiFi ドングル `wlx6c1ff789d5d4`（AIC8800）。`esp32_bridge` が :8766 で待ち受け |

  **2026-09-01 に CLAUDE.md へ書いた「ラズパイは DHCP で `192.168.5.125`」は誤り。それは（当時の）ESP32。**
  **2026-09-05 追記: ESP32 は無線を廃止しラズパイへシリアル直結した（`docs/network.md`参照）。
  上記表の「ESP32 が `192.168.5.125` の STA 子機」という行はもう実体が無い（歴史的記録として削除した）。
  代わりにラズパイ上の `pi_serial_relay` が PC:8766 へ接続しに来る。**
- **無線が遅い・切れるの真因は PC の USB WiFi ドングル（AIC8800 / `wlx6c1ff789d5d4`）。チャネル混雑ではない。** 2026-09-02 に対照実験で確定した。同じ AP・同じ ch1・同じ部屋・同じ時刻:

  | 条件 | 上り | 下り | ロス | avg RTT | max RTT |
  |---|---|---|---|---|---|
  | ドングル単独 | 3.35 Mbps | 6.71 Mbps | 18% | 151 ms | 970 ms |
  | **内蔵 Intel (`wlo1`/iwlwifi) 単独** | **28.6 Mbps** | **31.9 Mbps** | **0%** | **2.2 ms** | **11 ms** |

  内蔵カードは**混雑した ch1 のまま**ロス 0% / RTT 2.2ms を出す。→ **ch1 の混雑は律速ではない。チャネル移設（ch6 等）は無意味。**

  **【解決済み 2026-09-02】** AIC8800 を撤去し、次の構成にした（NetworkManager に保存済み・autoconnect 有効）:

  | デバイス | 接続先 | IP |
  |---|---|---|
  | 内蔵 Intel `wlo1` (iwlwifi) | `th-rpi-ap-wlo1`（ロボット AP・2.4GHz ch1） | **固定 192.168.5.50/24**（`never-default`。ラズパイの `pi_serial_relay` が決め打ちで接続しに来るため DHCP にしない） |
  | Elecom WDC-433SU2M2 `wlx3897a478b19d` (rtl8821au) | `net5g`（`NCT-WL-ST` 5GHz ch36） | DHCP・**既定経路はこちら** |

  インターネットを 5GHz に逃がしたので**機内の 2.4GHz 共存干渉も消えた**。効果（3 分ソーク）: ESP32 の WS 切断 194 秒ごと → **0 回**、`ESP32_DISCONNECTED` 105 回/7分 → **3 回**（起動時のみ）、`/scan_filtered` 5.4Hz・最大ギャップ 2.94s → **10.07Hz・最大 0.12s・標準偏差 0.006s**。
  - 旧 `th-rpi-ap` プロファイルは撤去した AIC8800 に束縛されたまま残っている（無害・発動しない）。
  - **WDC-433SU2M2 は 5GHz 専用**（実測: 見える AP 24 件すべて 5GHz、2.4GHz は 0 件）。ESP32 が 2.4GHz 専用なので**ロボット回線には使えない**。インターネット専用。
  - ドングルのドライバは統計を信用できない（`rx_drop`/`rx_err`/`tx_err` が全て 0 のまま、`Signal -51dBm` なのに `Link Quality=0/100`）。**このドングルの自己申告値で判断しないこと。**
  - 切り分けの定石: `ping` の「ロス」は RTT が数秒に伸びると測定終了時に飛行中のパケットが計上されるため過大に出る。**TCP スループット（`dd | ssh 'cat >/dev/null'`）で測るのが確実。**
  - ラズパイ→ESP32（PC を通らない経路）は**ロス 0%**。PC が絡む経路にだけロスが出るのが切り分けの決め手だった。
  - Ubuntu は `/etc/NetworkManager/conf.d/*powersave*` で `wifi.powersave = 3`（省電力ON）が既定。ロボット用接続だけ `nmcli connection modify th-rpi-ap 802-11-wireless.powersave 2` で無効化済み。
  - **ESP32 は 2.4GHz 専用**なので AP の 5GHz 化はできない（ESP32 が繋がらなくなる）。
  - PC が 2.4GHz 無線を 2 枚同時に使う状態（内蔵=モバイルホットスポット ch11／ドングル=ロボット ch1）は機内共存干渉を招くので避ける。
- **`pkill -TERM -f "ros2 launch ..."` は launch 親しか殺さず、子ノードは生き残る。** 「止めたはずなのにポートが埋まっている」「修正したのに古い挙動のまま」はこれ。実際に古い `esp32_bridge` が残って新 bringup の 8766 / rosbridge の 9090 を奪い、検証を 1 周無駄にした。`ps -eo pid,args` で ROS 関連を拾って **PID 指定で TERM** すること（`kill -9` は DDS discovery を壊すので使わない）。
- **このリポジトリは `core.fileMode = false`。`chmod +x` しても git の index に反映されない。** 新しく実行するスクリプト（`install(PROGRAMS ...)` に載せるもの）を追加したら `git update-index --chmod=+x <path>` を明示的に叩くこと。忘れると **git 上は 100644 のまま**で、`colcon build` は成功しテストも通るのに、実機の launch だけが `executable '<name>' not found on the libexec directory` で落ちる。`--symlink-install` では install 先がソースへのシンボリックリンクになるため、CMake の `install(PROGRAMS)` が付けるはずの実行権限が効かず、ソース側の権限がそのまま runtime に出るのが理由。**エラー文言が「見つからない」なので権限だと気づけない**（2026-09-03 に `map_downsampler.py` で実際に踏んだ）。`test_installed_scripts_executable.py` が再発を止める。
- **`pkill -f <パターン>` は docker 外（ホスト）でも自分のシェルを殺す。** `pkill -f vite` で exit 144 になり、後続の `rm` が実行されなかった。ホストでも PID 指定で止めること。

## 開発環境

すべての ROS2 コマンドは Docker コンテナ内（または ROS2 Humble がインストールされた環境）で実行する。

Windows では Docker Desktop ではなく **WSL2 内の Docker Engine** でコンテナを起動する。`docker compose` はこの WSL2 側の Docker Engine に接続されるため、コマンドは WSL2 のシェル（または WSL2 統合が有効なターミナル）から実行すること。

```bash
# Linux
xhost +local:docker
docker compose run --rm th_robot bash

# Windows (WSL2)
export DISPLAY=:0
docker compose run --rm th_robot bash
```

コンテナ内では `/root/th_ws` がワークスペースルート。

## ビルドとテスト

```bash
# フルビルド（初回・C++ 変更時）
cd /root/th_ws
colcon build --symlink-install
source install/setup.bash

# 特定パッケージのみ
colcon build --symlink-install --packages-select th_safety th_mode_manager

# 純粋単体テスト（ROS2 不要・最速）
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py -v

# 特定テストクラスのみ
python3 -m pytest src/th_testing/test/test_follow_planner_logic.py -v -k "TestRetreatHysteresis"

# ROS2 統合テスト
colcon test --packages-select th_testing --event-handlers console_direct+
colcon test-result --verbose

# 一括実行スクリプト
bash scripts/run_tests.sh           # 単体テストのみ
bash scripts/run_tests.sh --all     # 単体 + 統合
bash scripts/run_tests.sh --all --sim  # + シナリオテスト
```

## シミュレーション起動

```bash
# SLAM で地図作成（初回）
ros2 launch th_bringup gazebo.launch.py

# 既存地図でナビゲーション
ros2 launch th_bringup gazebo.launch.py \
  slam:=false map_yaml:=/root/th_ws/src/th_bringup/maps/th_map.yaml

# シナリオプリセットで起動（narrow_room / wide_area / cluttered /
# lost_reacquire / panel_shuttle。th_bringup/config/scenarios/ 参照）
ros2 launch th_bringup gazebo.launch.py scenario:=narrow_room

# キーボードテレオペ（別ターミナル）
ros2 launch th_bringup teleop.launch.py           # /cmd_vel_nav 経由（通常。twist_mux → obstacle_limiter を通る）
ros2 launch th_bringup teleop.launch.py direct:=true  # /cmd_vel 直接（SLAM 用。
  # twist_mux も obstacle_limiter も経由しない既存の例外。WP-SAFE-03 の
  # 「/cmd_vel を publish してよいのは obstacle_limiter だけ」という不変ルールに
  # 反する唯一の既知の穴。teleop.launch.py 自体は今回のパケットの範囲外なので
  # 未対応のまま（WP-SAFE-03 完了報告に記載）

# FOLLOWING モードに切替（起動 10 秒後）
ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode \
  "{requested_mode: 2, requester: 'cli'}"
```

## アーキテクチャ

### 速度指令の流れ（最重要）

WP-SAFE-03（2026-08-27）で最終段に `obstacle_limiter` が入った。twist_mux の出力先は
`/cmd_vel` ではなく `/cmd_vel_muxed` になり、`/cmd_vel` を publish してよいのは
`obstacle_limiter` だけになった。

```
挙動系ノード（th_transit/th_onsite/th_route/th_maintenance）
                       ─→ /cmd_vel_behavior (priority 20) ─┐
Nav2 controller_server ─→ /cmd_vel_nav      (priority 10) ─┤ twist_mux ─→ /cmd_vel_muxed ─┐
                                                             │                              │
safety_monitor ─→ /safety/estop      (lock 255) ────────────┤                              │
safety_monitor ─→ /safety/fault_lock (lock 254) ────────────┘                              │
                                                                                             ▼
/scan・/system/state・/cmd_vel_manual・/safety/estop・/safety/fault_lock ──→ [ obstacle_limiter ] ─→ /cmd_vel ─→ ESP32
                                                             （20Hz固定・沈黙禁止。base_link<-laser_link TFを起動時に有界リトライで取得）
```

**不変ルール**: `/cmd_vel` に直接 publish するノードを追加してはいけない。`obstacle_limiter` だけが `/cmd_vel` の publisher。すべての速度指令は twist_mux → `/cmd_vel_muxed` → `obstacle_limiter` 経由。Nav2 経由の移動は `/cmd_vel_nav`（priority 10）、それ以外の挙動系（点検・校正の走行を含む）は `/cmd_vel_behavior`（priority 20）を使う。

`follow_planner.py` / `follow_planner_mapless.py` / `person_predictor.py` は旧設計の挙動ノードで、いまだに廃止済みの `/cmd_vel_retreat` へ publish している（新設計の `/cmd_vel_behavior` ではない）。twist_mux はもうこのトピックを購読していないため、**これらのノードの退避・捜索旋回コマンドは現在誰にも届かず無音のまま捨てられる**（追従自体は Nav2 経由の `/cmd_vel_nav` で動くため気づきにくい）。3ノードとも新設計での廃止対象（`docs/plan/detailed/DetailedDesign-names.md` §1.1・§6.1）なので、書き直すのではなく WP-TRANSIT-01 等での削除を待つこと。

### 追従ロジックの二層構造

追従ロジックは意図的に二層に分けられている:

- `th_planning/th_planning/follow_planner_core.py` — **ROS2 非依存の純粋 Python**。`FollowPlannerCore.update()` がコアアルゴリズム。このファイルは ROS2 を import しないことで `pytest` で直接テスト可能。
- `th_planning/scripts/follow_planner.py` — ROS2 ノード。`follow_planner_core.py` を import して `/person/status` → Nav2 ゴール / `/cmd_vel_retreat` に接続するだけ（`/cmd_vel_retreat` は WP-SAFE-03 以降 twist_mux が購読しておらず無音で捨てられる。上の「速度指令の流れ」の注記参照）。

新しい追従ロジックを追加する際は必ず `follow_planner_core.py` に純粋関数として実装し、`test_follow_planner_logic.py` にテストを追加してから `follow_planner.py` から呼ぶ。

### 安全チェーンの設計

`safety_monitor`（C++）が `/safety/estop` と `/safety/fault_lock` を twist_mux に送る。`mode_manager` の処理を待たずに twist_mux がモーターをゼロにする（フォルト検知 → 物理停止は 100ms 以内）。

ESP32 には独立したウォッチドッグ（600ms、`config.h` の `WATCHDOG_MS`）があり、ROS2 がクラッシュしても停止できる。esp32_bridge は `/cmd_vel` を20Hzキープアライブで再送しており、WiFi ジッタによる誤発動を避けるため2026-08-05に300ms→600msへ緩和した（詳細: `docs/architecture.md`「ESP32側の二重フェイルセーフ」）。

### オドメトリと TF

```
ESP32 (WHEEL_FEEDBACK: 左右速度 + dt) ─→ esp32_bridge ─→ /odom (publish_tf は false)
                                                            ↓
ESP32 (IMU_DATA: BNO055) ─→ /esp32/imu_data ─→ ekf_filter_node ─→ odom→base_link TF
```

**不変ルール**: `odom → base_link` の TF を発行するのは `ekf_filter_node` だけ。`esp32_bridge` の `publish_tf` を true に戻してはいけない（TF ツリーが二重親になる）。

- EKF が融合する IMU 入力は**ジャイロの `vyaw` のみ**。BNO055 は NDOF モードで絶対方位（地磁気参照）を返すため、屋内の磁気擾乱でヨーが飛ぶ。`world_frame: odom` に絶対方位を入れてはいけない。
- オドメトリの積分区間は ESP32 が `WHEEL_FEEDBACK` に載せてくる `dt`。到着時刻から推測してはいけない（WiFi 遅延がそのまま yaw ドリフトになる）。旧形式の 9 byte フレームも受理する。

### モード FSM

`mode_manager.cpp` の `isTransitionAllowed()` で遷移を制御:

```
INIT → IDLE のみ
IDLE → FOLLOWING, FOLLOWING_MAPLESS, SUMMONING, MANUAL
FOLLOWING → MANUAL, MOVING_TO_PANEL, IDLE
FOLLOWING_MAPLESS → MANUAL, IDLE
SUMMONING → MANUAL, IDLE
MOVING_TO_PANEL → AT_PANEL, MANUAL, IDLE
AT_PANEL → FOLLOWING, MANUAL, IDLE
MANUAL → FOLLOWING, FOLLOWING_MAPLESS, IDLE
any → ESTOP
ESTOP → IDLE のみ
```

フォルト発生時は動作系モード（FOLLOWING / FOLLOWING_MAPLESS / SUMMONING / MOVING_TO_PANEL / AT_PANEL / MANUAL）から IDLE へ強制遷移。IDLE 中のフォルトはモード変化なし。

ただし `PERSON_TRACKER_LOST` だけは例外で、試験員データを使うモード（FOLLOWING / FOLLOWING_MAPLESS / SUMMONING）からのみ強制遷移する。MANUAL ジョグや配電盤移動は人物データを使わないため継続できる（VISION.md §5）。

### カスタムメッセージ型（th_system_msgs）

- `RobotMode.msg` — mode フィールド (uint8) と定数 INIT=0, IDLE=1, FOLLOWING=2, MOVING_TO_PANEL=3, AT_PANEL=4, MANUAL=5, ESTOP=6, FOLLOWING_MAPLESS=7, SUMMONING=8
- `PersonStatus.msg` — `position`（ロボット base_link 基準の相対座標 m）, `confidence`, `is_lost`, `lost_reason`
- `FaultStatus.msg` — `active`, `fault_type` ("LIDAR_LOST" / "ESP32_DISCONNECTED" / "PERSON_TRACKER_LOST")
- `WheelFeedback.msg` — ESP32 から届く左右ホイール実速度(`/esp32/wheel_feedback`)。指令値側にも同型を再利用し `/esp32/wheel_cmd_speed`(esp32_bridge が `/cmd_vel` から計算)として発行、WebUI の速度表示カードで指令vs実測を比較する
- 状態 publish 3種（`FollowStatus` = `/follow/status`、`SearchStatus` = `/person/search_status`、`SummonStatus` = `/summon_navigator/status`）— 追従・捜索・呼び寄せの内部状態。音声アナウンスと WebUI 表示のトリガ源。**state / reason / phase の文字列定義は各 .msg のコメントが正**なので、写像を書くときは必ずそちらを見る

### シミュレーション固有のノード

`gazebo.launch.py` が起動するが `bringup.launch.py` には含まれないノード:

- `gazebo_person_relay.py` — Gazebo の Actor/モデル位置を `/person/status` に変換。`GetEntityState` サービス → `/gazebo/model_states` → `/gazebo/link_states` の順にフォールバック。TF は使わずロボット相対座標を直接計算（`robot_name` パラメータで参照）。
- `person_mover.py` — cylinder モデルをシナリオ制御（patrol/approach/static パターン）

### パラメータファイルの場所

| 対象 | ファイル |
|------|---------|
| 追従ロジック全般 | `th_bringup/config/planning_params.yaml` |
| オドメトリ融合（EKF） | `th_bringup/config/ekf_params.yaml`（IMU有効・既定） / `ekf_params_no_imu.yaml` |
| 人物トラッカー | `leg_detection_bringup/param/leg_tracker_param.yaml` |
| 安全タイムアウト（実機） | `th_safety/config/safety_monitor.yaml` |
| 安全タイムアウト（シミュ） | `th_bringup/config/safety_monitor_sim.yaml` |
| twist_mux 優先度 | `th_safety/config/twist_mux.yaml` |
| Nav2（実機） | `th_bringup/config/nav2_params.yaml` |
| Nav2（シミュ） | `th_bringup/config/nav2_params_sim.yaml` |
| LiDAR 死角 | `th_bringup/config/perception_params.yaml` |
| 配電盤座標 | `th_bringup/config/panels.yaml` |
| ESP32 ブリッジ | `th_esp32_bridge/config/params.yaml` |

Python スクリプトは `--symlink-install` によりシンボリックリンクで即時反映される。C++ パッケージを変更した場合は `colcon build` が必要。
