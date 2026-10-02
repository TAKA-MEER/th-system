# 全域ローカライズ・途中復帰の根拠

[本体](DetailedDesign-transit-localize.md) の結論に至った根拠。番号はブリーフの
「設計書で答えること」に対応する。

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
1. 始点姿勢で現行どおり deserialize（match_type=3）。スコア s を測る
2. s ≥ 閾値 → evt.localize_done（score 付き）
3. s < 閾値 → evt.localize_low（score 付き）→ widen_search:
   経路点列の周辺グリッドで粗探索し、最良候補で deserialize し直す
4. それでも s < 閾値、または ui.localize_global → global_localize:
   地図全域の粗探索 → 最良候補で deserialize（＝完全グローバル）
5. READY。試験員が地図と位置を目で確かめて「再生」を押す（§4）
```

`match_type=1`（`START_AT_FIRST_NODE`。`slam_control_logic.py:28-31`）は
「最初のノードにいる決め打ち」であって全域探索ではない（実機事故 WS-9Y の記録あり）。
全域探索のつもりで使ってはいけない。

## §2. 確度

**確度＝自前一致度スコア `s`**（0〜1。`/scan` の有効点のうち凍結地図の占有セルに
一致した割合。地図に無い物（人・動く物）で悪化するので外れ値を除く。
`O-e2`（ゆっくり間違う検知）と同じ指標で、将来の統合を見込む）。

| 項目 | 決定 |
| --- | --- |
| 「低い」の閾値 | `localize_match_low`（widen の引き金）、`localize_match_min`（READY で警告）。いずれも **registry に placeholder** として新設（`class: c`、`blocking_from_stage: 5`＝起動は止めない。W-15 の教訓） |
| 実測の方法 | 始点マークでの正常値分布と、意図的にずらした位置での値を取る。共分散を捨てた実測（`Spec-safety.md` §3.5.0。`odom` を 1.5 m ずらす rig）を流用する |
| 発行 | `evt.localize_low`／`evt.localize_done` の `arg_json` に `{score}` を載せる。画面は数値ではなく「高い／低い」の 2 値＋目安文を出す（§4） |

## §3. 途中からの復帰

完全グローバル確定後の姿勢 `p` に対し、経路点列から再開 index を決める。

| 項目 | 決定 |
| --- | --- |
| 点の選び方 | `p` に最も近い点のうち、**進行方向との内積が正**（前向き）のもの。向きで前後関係の曖昧さを解く |
| 向き | 再開点の `yaw` まで既存の超信地旋回（`rotate_toward`）で合わせる。`rotate_to_start_yaw` の実体を再利用し、目標ヨーだけ差し替える |
| 再開の渡し方 | `load_route` に `from_index` 任意引数（`transitions.yaml` の effect 転送が passthrough なら行追加は不要。**実装時に確認**。だめなら新 effect `seek_route{index}` を足す） |
| 逆再生との関係 | W-19 の裏側（`reverse_points`）は `from_index` と直交する。反転後の点列に同じ規則を適用する |

## §4. READY 確認画面（S-14。spec に無い詳細は設計で決めたと明記）

| 区画 | 内容 | 出どころ |
| --- | --- | --- |
| 地図・経路・現在位置 | 現行どおり | spec §3.7 |
| 確度 | **「高い／低い」の 2 値＋目安文**（例: 「低いときは完全グローバルで探すか経路を選び直す」）。**数値は出さない**（設計で決めた。数値の意味を試験員に説明できないため） | 設計 |
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
