# 全域ローカライズ P0 — オフライン実現性検証の記録

[本体](DetailedDesign-transit-localize.md) §3 の P0 の測定記録（2026-10-02）。要約は [根拠](DetailedDesign-transit-localize-options.md) §0。
測定スクリプトは `tools/w01_p0_*.py`。以下は実装エージェントの報告をそのまま残したもの。
**注**: 本文中の `.briefs/tmp/` は作業時の置き場で、残っていない。スクリプトは `tools/w01_p0_measure.py`・`tools/w01_p0_analyze.py`・`tools/w01_p0_export_maps.sh`（＝`export_maps.sh`）にコミットしてある。地図の pgm と venv は再生成する（手順は §末尾）。

## W-01 P0 報告: 全域ローカライズ粗探索のオフライン実現性検証

ブランチ: `test/w01-p0-feasibility`。製品コードの変更なし（spec・試験に触らず）。
成果物: 測定スクリプト `docs/plan/detailed/tools/w01_p0_{measure,analyze}.py`、
地図変換手順 `.briefs/tmp/export_maps.sh`、本報告 `.briefs/tmp/report.md`
（`.briefs/tmp/` は git 管理外）。設計書への要約追記は
`DetailedDesign-transit-localize-options.md` §0。

## 0. 前提検証（作業前の事実確認。停止級の誤りなし）

| ブリーフの前提 | 確認結果 |
| --- | --- |
| 入力 `th_ws/data/routes/` に `.posegraph`／`.data`・`route_*.json` | あり。ただし `route_1788*.json` は `frame_id: odom` で地図対応なし。地図対応があるのは `id==name`・`frame_id: map` の命名試験ファイルのみ。対象は後者に限定した |
| 長距離0904-2 (20MB/13MB)・0907-1 (28MB/21MB)、短距離0904-2 (8.8MB) | 実測: 0904-2=20M/13M、0907-1=**27M/20M**、短距離0904-2=**8.4M**。丸め誤差級。停止理由にならない |
| `.data` は Boost 直列化で Python 読解困難 | 確認（`serialization::archive`）。読まない方針どおり |
| Docker の slam_toolbox＋map_saver（nav2_map_server 入り） | あり（`localization_slam_toolbox_node`・`map_saver_cli`・`DeserializePoseGraph`・`SaveMap`）。ただしサービス名の注意が2件（§1） |
| LiDAR 仕様は `perception_params.yaml` 等で確認 | **同ファイルに角度・刻み・距離の記載なし**（lidar_filter の topic 名のみ）。実値は別証拠で確定: `/scan_filtered` は実測で **720点・0.499°刻み・全周**（`obstacle_limiter_core.cpp` WS-9P 実測 2026-09-04、C++ 試験も `angle_min=-pi, 720点` 前提）。駆動は RPLIDAR S1・`angle_compensate:=true`・Standard（`docs/setup.md` §rplidar、ラズパイ側 `rplidar_ros`）。`max_range=40.0` は S1 公称の仮定（rosbag が無いため実測なし。**P6 で再確認**） |
| 死角は registry の `blind_angle_ranges` を抜く | 正。値は非空の4セクタ実測値（2026-09-09、上部構造の支柱）。なお `perception_params.yaml` のコメント「現構成は死角なし空配列」は更新漏れ（registry が正）。地図記録日（09-04/09-07）が死角実測（09-09）より前なのは P0 の模擬条件としては問題なし（現行体で再生する前提） |

## 1. 地図の pgm/yaml 化（`.briefs/tmp/maps/`、3本成功）

- `localization_slam_toolbox_node` を起動し `/slam_toolbox/deserialize_map`→
  `/slam_toolbox/save_map`＋`map_saver_cli` で書き出し。入力は `:ro` マウント。
- つまずき2件（いずれも測定スクリプト側で解決。製品設計への影響なし）:
  1. localization モードは `match_type=1/2` を `non-localization deserialization` で拒否する。**`match_type=3 (LOCALIZE_AT_POSE)`＋初期姿勢(0,0,0)で読めた**。P2 の実装者は確定1回ポッキリの方針（options §1）と矛盾しないが、export 用と確定用で match_type が違う点に注意。
  2. `map_saver_cli` は ROS 実行ファイル名（`ros2 run nav2_map_server map_saver_cli`）で、`which` では見つからない。
- `save_map` と `map_saver_cli` は同一内容を出すことを確認し、前者を削除。解像度 0.05 m。
  短距離 287×153、長距離2本 各1.4MB（約70×50m級）。

## 2. 測定方法

- 模擬スキャン: 経路 JSON の姿勢から pgm へ光線追跡（720ビーム・0.5°・angle_min=-π、
  blind 4セクタ除外）。`clean`（σ=0.03m）と `occluded`（σ=0.05m＋人想定の手前遮蔽
  1〜3箇所 8〜25°＋2%欠損）の2条件。サンプリング: 長距離 stride 30（39/44点）、
  短距離 stride 4（39点）。
- 探索（案Aの粗→細）: A=全域 0.5m/10°（間引き180ビーム・tol 0.3）→ NMS（1.5m間隔）上位60
  → B=±1m/0.25m・±15°/5°（tol 0.15）→ C=±0.3m/0.1m・±5°/1°（tol 0.1）。
  `s`=有効点のうち占有セル±0.1m以内一致率、`m`=2m以上離れた2番目（同じくB/C精密化後）との差。
- **実装上の反省2件**（いずれも測定スクリプトのバグで修正済み。設計の前提は崩さない）:
  - 初版の光線追跡が姿勢 yaw を足し忘れ、yaw の付いた姿勢で s が 0.2 台になった。
  - 初版の粗刻み 1m/15° では真値 basin が 351/2700 位に埋もれて半数が失敗。
    0.5m/10°＋間引きで rank 0 になることを確認して採用。**P1/P2 への申送り:
    粗探索は 1m 刻みでは粗すぎる。0.5m/10°級（＋ビーム間引き）が要る。**

## 3. 結果

CPU: 8 x 11th Gen Intel(R) i7-1165G7 @ 2.80GHz（実機 PC と異なる。P6 で実測し直す）。
正解 = 位置誤差≦0.5m かつ角度誤差≦5°。

| 地図 | 条件 | n | 正解率 | t中央値[s] | t最大[s] | s中央値 | m中央値 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 長距離0904-2 | clean | 39 | 87% (34/39) | 4.1 | 6.7 | 0.99 | 0.22 |
| 長距離0904-2 | occluded | 39 | 87% (34/39) | 4.0 | 7.7 | 0.87 | 0.19 |
| 長距離0907-1 | clean | 44 | 98% (43/44) | 2.9 | 3.5 | 0.98 | 0.19 |
| 長距離0907-1 | occluded | 44 | 86% (38/44) | 2.8 | 3.4 | 0.87 | 0.17 |
| 短距離0904-2 | clean | 39 | 100% | 1.7 | 3.6 | 0.99 | 0.48 |
| 短距離0904-2 | occluded | 39 | 100% | 1.6 | 2.8 | 0.86 | 0.35 |

失敗時の s/m（取り違え検出の材料）:

| 地図 | 条件 | 失敗n | s範囲 | m範囲 |
| --- | --- | --- | --- | --- |
| 0904-2 | clean | 5 | 0.86〜0.95 | 0.03〜0.16 |
| 0904-2 | occluded | 5 | 0.76〜0.87 | 0.02〜0.20 |
| 0907-1 | clean | 1 | 0.92 | 0.01 |
| 0907-1 | occluded | 6 | 0.67〜0.87 | 0.00〜0.11 |

分離の評価（§2 の「s だけでは足りない」の裏付け）:

- **s 単独は不可**: 成功の s 最小 0.75〜0.97 と失敗の s 最大 0.87〜0.95 が重なる。
- **m は失敗をほぼ捉えるが成功とも重なる**: 失敗の m 最大 0.20 に対し成功の m 最小は
  0.00〜0.05（0907-1 occ で成功 yet m=0.003 あり）。`m<0.2` を警告にすると失敗は全部拾えるが
  成功にも警告が出る（誤警告）。`m<0.05` を不成立にすると失敗の大半は止まるが
  m=0.16〜0.20 の失敗（廊下沿い 4m ズレ等）がすり抜ける。
- 失敗の内訳: (a) 廊下の沿道方向ズレ（同 yaw、1.7〜12m。0904-2 の南北・東西の長い直線廊下。
  0904-2 idx 60/450/540/570、0907-1 idx 1050/1260/1290）。(b) 対称・類似場所への飛び
  （20〜50m、yaw も90〜180°違う。0907-1 idx 240/420/1020）。(b) は m≦0.11 で検出可。
  (a) は yaw が合っているため replay 再開後に縦方向へ流れる恐れがあり、
  READY の目視確認（設計 §4）が最後の砦になる。
- 成功側の m が極小になる例（0907-1 occ で m=0.003 の成功）: 正解だが別所にほぼ同点の
  alias がある場所。警告は出るが READY 目視では正位置に見える（運用上は許容）。

## 4. 結論: 案Aを続行（案Bへの切替は不要）

判断基準（options §0）に照らして:

1. **時間は収まる**: 最大 7.7s、中央値 1.6〜4.1s。LOCALIZE の待ちとして許せる数十秒に十分収まる。
   実機 CPU は別物だが 1 桁違う余裕があり、P6 で実測すれば足りる水準。
2. **m で取り違え検出の見込みは立つ**: 失敗 17 件の m はすべて 0.20 以下。
   `localize_margin_low ≈ 0.2` の警告で全部拾える。s 単独不可の設計判断（§2）は正しかった。
   ただし成功との完全分離はできないため、警告は誤警告混じりになる前提で
   P1 の閾値設計を行うこと（警告≠不成立の二段構えは維持）。
3. 残課題（P1/P2/P6 へ申送り）:
   - 粗探索の既定刻みは **0.5m/10°級＋ビーム間引き**（1m では真値が埋もれる。§2反省）。
   - `localize_margin_min`（不成立）は m=0.16〜0.20 の廊下沿いズレを止められない。
     不成立単独に頼らず READY 目視＋widen 再探索の組合せで設計すること。
   - **楽観バイアス**: 模擬スキャンは同一地図・静環境（家具移動・人・ガラス・歪みなし）。
     実スキャンでは s/m とも悪化する。実スキャン再確認は P6（実機）に回す。
   - `max_range=40.0` は公称仮定。実 `/scan` の最大距離・点数分布を P6 で実測すること。

## 5. spec との食い違い

なし（spec は読んだ範囲で触らず。P0 はコード変更なし）。

## 6. 再現手順

1. `.briefs/tmp/export_maps.sh` を worktree `th_ws/` から
   `docker compose run` で実行（詳細はファイル頭。match_type=3 が要点）。
2. `.briefs/tmp/venv`（numpy＋scipy）で `w01_p0_measure.py` を実行
   （`--maps-dir`・`--routes-dir`・`--maps`・`--stride`・`--out-dir`）。
3. `w01_p0_analyze.py <maps-dir> <out-dir> [--map <id>...]` で表＋ASCII 失敗地図。
4. `docs/plan/detailed/tools/` の同名ファイルがコミット済みの正本。
