# 全域ローカライズ・途中復帰の根拠

[本体](DetailedDesign-transit-localize.md) の結論に至った根拠。番号はブリーフの
「設計書で答えること」に対応する。

## §0. P0 オフライン実現性検証（P1 の前に行う判断点）

製品コードを書く前に、「自前スキャンマッチの全域粗探索が秒単位で回り、
取り違えないか」を PC だけで測る。裏付けの無い主張（「秒単位」）のまま P1 に進まない。

| 項目 | 内容 |
| --- | --- |
| 使うデータ | **実機の経路地図がある。**`/root/th_data` は `th_ws/data` そのもの（bind mount）で git 管理外のため、この worktree に無いだけ。メイン作業ツリーの `th_ws/data/routes/` に `.posegraph`／`.data` が 12 本ある。**読むだけで書き換えないこと** |
| P0 の入力（具体名） | 長距離: `長距離試験0904-2`（`.posegraph` 20MB／`.data` 13MB）、`長距離試験0907-1`（`.posegraph` 28MB／`.data` 21MB）。短距離: `短距離試験0904-2`（`.posegraph` 8.8MB）等。記録スキャンは `.data`（slam_toolbox の保存形式は各ノードのスキャンを持つ）から取る。`.data` の読み方は P0 の測定スクリプト側で確かめる |
| 測るもの | ①粗→細の段階探索の所要時間（Python＋numpy。地図サイズ・候補刻みごと。まず長距離の 2 本で測る）②対称な部屋・長い廊下での `s`／`m` 分布（取り違えの起きやすさ） |
| 置き場 | 測定スクリプトと結果は `.briefs/tmp/`（製品コードにしない） |
| 判断基準 | 探索が実用時間（LOCALIZE の待ちとして許せる数十秒）に収まり、`m` で取り違えを検出できる見込みが立てば**案 A を続行**。収まらない／`m` が恒常的に小さい地図が相手なら**案 B（AMCL）へ切り替える**（切り替え時は本設計書の案 B 比較表を起点に別設計を起こす） |
| P0 結果（2026-10-02。詳細 [P0 の記録](DetailedDesign-transit-localize-p0.md)） | **案 A を続行**。長距離2本＋短距離1本・各39〜44点・2条件（clean／人想定遮蔽）で中央値 1.6〜4.1s・最大 7.7s（i7-1165G7。実機値は P6）。正解率 86〜100%。失敗 17 件の `m` はすべて 0.20 以下（`s` 単独は成功と重なり不可）。粗刻みは 1m では真値が埋もれるため **0.5m/10°級＋ビーム間引き**が要る（P1/P2 へ申送り）。`m<0.2` 警告で失敗は全部拾えるが成功にも誤警告が出る。不成立閾値だけでは廊下沿いズレ（m 0.16〜0.20）がすり抜けるため READY 目視と併用する。模擬は同一地図・静環境で楽観的なので実スキャン再確認は P6 |

## §1. 全域ローカライズの方式

### 候補

| 案 | 内容 |
| --- | --- |
| A（推奨） | **slam_toolbox の localization モードのまま二段構え。**自前スキャンマッチで候補を粗探索し、上位 1 候補だけ `deserialize_map(match_type=3)` で確定する |
| B | 保存した占有格子を **AMCL に読ませてグローバル初期化**する |
| C | 現在地周辺の拡大だけ（完全グローバルなし） |

### 比較

| 軸 | A | B | C |
| --- | --- | --- | --- |
| 実機 CPU | 自前スコアは `/scan` 1 回分の照合で軽い。確定は現行と同じ `deserialize` 1 回。人検出は REPLAY で動かさない（`Spec-modes.md` §9）ので約 5 コア空きで足りる | AMCL 自体は軽いが、slam との二重起動・切替の間だけ重なる | 最も軽い |
| 所要時間 | 粗探索は秒単位。確定の `respawn＋deserialize` は現行どおり最大 45s＋30s（`replay_runner.py:412-415`）。候補ごとに `respawn` しない（上乗せバグの対策 WS-9S を保つ） | AMCL の収束待ち＋地図変換。初回は A より読めない | 短い |
| 失敗の見分け | スコアが閾値未満なら「見つからない」と明示できる（§2）。確定後は READY で試験員が目視する | AMCL の共分散は使えない（`Spec-safety.md` §3.5.0 で実測済み）。結局自前の見分けが要る | 範囲外のずれを見つけられない |
| 地図形式 | そのまま（`.posegraph`／`.data`。経路ごとに保存済み） | **詰まる。**保存は slam 形式で、`map_server` の `pgm`／`yaml` ではない。`save_map` の pgm 併存か変換が要る | そのまま |
| TF | そのまま（`map→odom` は slam が出し続ける） | **詰まる。**AMCL も `map→odom` を出す。どちらを生かすかの切替設計と、切替瞬間の飛び（B′ の `jump` が鳴る）の扱いが要る | そのまま |

C は `Spec-transit.md` §4.1 手順 4（最初から完全グローバルも選べる）を満たさないので却下。
B は地図形式と TF 競合の解消に別設計が要り、共分散が使えない以上「確度の見分け」を
自前で作る手間は A と同じになる。よって **A を推す**。B は将来の置換候補として記録だけ残す。

### A の手順（LOCALIZE 中）

```
1. 保存 pgm 上で粗探索（§0 の刻み）→ 最良候補で deserialize（match_type=3）1 回。
   候補ごとに respawn／deserialize しない（WS-9S の上乗せ対策を保つ）
2. s ≥ 閾値 かつ m ≥ 閾値 → evt.localize_done（score＋margin 付き）
3. s < 閾値 → evt.localize_low → widen_search:
   最良候補の周辺グリッドで再探索し、更新された最良候補で deserialize し直す
4. それでも不成立、または ui.localize_global → global_localize:
   地図全域の粗探索 → 最良候補で deserialize（＝完全グローバル）
5. READY。m が小さいときは警告を出す（`localize_margin_low` 未満）。
   `m < localize_margin_min` の不成立と `s < localize_match_low` の widen
   でも拾えないものは READY に進めず **LOCALIZE に留まる**
   （`localize_quality='failed'`。試験員は画面の「完全グローバルで探す」か
   選び直しを選ぶ）。`ui.localize_global` は LOCALIZE でしか受け付けない
   （`transitions.yaml` は変えない）ので、この扱いで足りる（P2 決定）。
   ※旧版は「READY で『再生』を非活性」と書いていたが、READY に進めたうえで
   ボタンを殺すより LOCALIZE に留めるほうが FSM の行追加が要らないため改めた。
```

`match_type=1`（`START_AT_FIRST_NODE`。`slam_control_logic.py:28-31`）は
「最初のノードにいる決め打ち」であって全域探索ではない（実機事故 WS-9Y の記録あり）。
全域探索のつもりで使ってはいけない。

| 項目 | 決定 |
| --- | --- |
| 粗探索に使う占有格子 | **保存時に併存させる `pgm`**（`map_dir/<id>.pgm`／`.yaml`。`posegraph` と同じ `base` に `SaveMap` で書き足す）。`/map` トピック（slam_toolbox が出す OccupancyGrid）は使わない |
| `/map` を使わない理由 | 全域探索の前に地図を読まないと `/map` が無い（順序が逆）。初回 `deserialize` で `/map` を得てから粗探索すると `deserialize` が 2 回要り、2 回目は上乗せバグ（WS-9S）の危険域に入る。静的な `pgm` を読めば `deserialize` は確定の 1 回で済む |
| 保存の変更 | `slam_control.py` の `_serialize`（posegraph のみ）に `SaveMap`（pgm＋yaml）呼び出しを足す。VENUE 側の `_cb_save_map` は両方呼ぶ流儀が既にあるのでそれに揃える。`pgm` は地図確定時（教示の保存）のスナップショットで、再生中に変わらない |
| P0 との関係 | 段階探索の刻み・所要時間の実測は §0 で先に取る。刻みの既定値は P0 の結果で決める |

## §2. 確度

**確度＝自前一致度スコア `s`（最良候補）と、マージン `m`（最良と 2 番目の差）の組**
（0〜1。`/scan` の有効点のうち凍結地図の占有セルに一致した割合。
地図に無い物（人・動く物）で悪化するので外れ値を除く。
`O-e2`（ゆっくり間違う検知）と同じ指標で、将来の統合を見込む）。

**`s` だけでは足りない。**廊下や対称な部屋では複数の姿勢で `s` がほぼ同点になり、
1 スキャンでは取り違える。**2 番目は「別の場所」の候補**とする
（最良候補の近傍＝同じ場所の別刻みは除く。近傍を 2 番目にすると `m` が常に小さくなる）。

| 項目 | 決定 |
| --- | --- |
| 「低い」の閾値 | `localize_match_low`（widen の引き金。`s` 用）、`localize_margin_low`（警告用・`m` 用）、`localize_margin_min`（不成立用・`m` 用）。いずれも **registry に placeholder** として新設（`class: c`、`blocking_from_stage: 5`＝起動は止めない。W-15 の教訓） |
| 実測の方法 | 始点マークでの正常値分布と、意図的にずらした位置での値を取る。共分散を捨てた実測（`Spec-safety.md` §3.5.0。`odom` を 1.5 m ずらす rig）を流用し、`s` だけでなく `m` の分布も取る（P0 で先行測定） |
| 発行 | `evt.localize_low`／`evt.localize_done` の `arg_json` に `{score, margin}` を載せる。画面は数値ではなく「高い／低い」の 2 値＋目安文を出す（§4） |

## §3. 途中からの復帰

完全グローバル確定後の姿勢 `p` に対し、経路点列から再開 index を決める。

| 項目 | 決定 |
| --- | --- |
| 点の選び方 | `p` に最も近い点のうち、**進行方向との内積が正**（前向き）のもの。向きで前後関係の曖昧さを解く |
| 向き | 再開点の `yaw` まで既存の超信地旋回（`rotate_toward`）で合わせる。`rotate_to_start_yaw` の実体を再利用し、目標ヨーだけ差し替える |
| 再開の渡し方 | FSM・`load_route` 引数を足さない。`replay_runner` が確定時（探索成功の poll）に確定姿勢から再開 index を計算して保持し、`rotate_to_start_yaw` でその点の向きに合わせる（P4 決定。`transitions.yaml` の effect 転送は明示対応付けであって passthrough ではないことを `state_core._resolve_effect_args`＋`EFFECT_ARG_SPECS` で確認済み。引数渡しも新 effect `seek_route{index}` も要らない） |
| 逆再生との関係 | W-19 の裏側（`reverse_points`）は `from_index` と直交する。反転後の点列に同じ規則を適用する |
| 同値・遠すぎ | 完全同点は手前（index の小さい方）を取る（安全側。終点側へ飛ばない）。最も近い前向き点まで `resume_max_dist_m`（既定 2.0 m。出発点。実測で詰めるのは P6）を超えたら途中復帰せず `failed` で LOCALIZE に留まる（`index 0` から走り出すと離れた始点へ向かうため） |

## §4. READY 確認画面（S-14。spec に無い詳細は設計で決めたと明記）

| 区画 | 内容 | 出どころ |
| --- | --- | --- |
| 地図・経路・現在位置 | 現行どおり | spec §3.7 |
| 確度 | **「高い／低い」の 2 値＋目安文**（例: 「低いときは完全グローバルで探すか経路を選び直す」）。**数値は出さない**（設計で決めた。数値の意味を試験員に説明できないため）。`m` が `localize_margin_low` 未満なら「似た場所が複数あり取り違えの可能性」と明示する | 設計 |
| 不成立 | `s < localize_match_low` の widen でも不成立、または `m` が `localize_margin_min` 未満なら **LOCALIZE に留まる**（`localize_quality='failed'`。`evt.localize_done` を出さない。試験員は「完全グローバルで探す」か経路の選び直しを選ぶ。設計で決めた。取り違えたまま走らせるより止める。※旧版の「READY で『再生』を非活性」から P2 で改めた。理由は §1 手順 5 参照） | 設計 |
| 「完全グローバルで探す」ボタン | spec どおり。押すと `ui.localize_global` | spec §3.7 |
| 「経路を選び直す」 | `PAUSE` からの選び直し（`SM-3.1.2-103`）と同型。`ROUTE_SEL` へ戻す | 設計（既存遷移の流用） |
| 操作カード | 停止／再生／手動（現行どおり） | spec §3.7 |

## §5. FSM の差分

本体 §2 の表が結論。補足：

- `T-REPLAY-02`／`-03` の行自体は既にある（`transitions.yaml:581-604`）。足すのは**発火元と実体**であって行ではない。
- `widen_search`／`global_localize` の実体は `replay_runner.py:377-383` の no-op を置き換える（`WAIVER W-01` タグはそのとき外す）。
- 新規名（`from_index`／`localize_match_low` 等）は実装前に `DetailedDesign-names.md` へ足す（詳細設計の約束 2）。

## §6. 安全

| 局面 | 決定 |
| --- | --- |
| LOCALIZE／READY 中の駆動 | `RUN` でないので `replay_runner` は沈黙する（挙動ノード共通形 B2）。ジョグ介入は可（REPLAY は `jog: 可`） |
| 探索中の `LOCALIZATION_LOST` 誤発火 | 探索は意図的に `map→odom` を飛ばし、`respawn` で `node_down`／`stale` が出る。既存の「計画的再起動の保留」（`Spec-safety.md` §3.5.0）を拡張し、探索中フラグ（`reason=global_localizing`）を `localization_health` へ伝える。**上限付き**（`localization_restart_max_ms` 流用または新設。超えたら本物の故障） |
| READY 中の推定死 | REPLAY 全体が監視対象モードなので、`LOCALIZATION_LOST` で ESTOP に落ちる。**正しい挙動**（壊れた推定で再生させない） |
| B′（`jump`）との関係 | 探索確定の瞬間は計画的不連続なので、B′ の比較は `deserialize` 直後に基準値を捨てる（再起動またぎと同じ扱い） |

## §7. 台帳・他 Way との関係

- **W-01**: 本体 §4 の表が対応。`align_path_to_current` だけは残す（odom フォールバック経路の保険。消すと slam 未起動時に再生不能になる）。
- **W-02**（`replay_drift`・横ずれ表示）: 確度スコア `s` の走行中の連続表示は W-02 の横ずれ表示と統合できる。統合可否は P2 の実装時に判断する。
- **W-19**（逆再生）: §3 のとおり直交する。P4 の試験は順・逆の両方で取る（KPI は分けて計測）。
