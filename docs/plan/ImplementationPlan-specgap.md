# 実装計画 — spec と実装の乖離（2026-10-04 調査）

[ImplementationPlan.md](ImplementationPlan.md) §6「第 1-b 群」（`1b-1`〜`1b-13`）の詳細。
**`docs/plan/spec/` と実際のコードを突き合わせ、計画書（§3・§4・§6）と
[EXCEPTION-LEDGER.md](EXCEPTION-LEDGER.md) のどちらにも載っていない乖離だけを集めた。**

| 項目 | 内容 |
| --- | --- |
| 対象 | `main` = `d26fb39` |
| 方法 | **静的解析のみ。**コードを読む ＋ 状態遷移の純粋コア（`StateCore`）を直接 `step()` する。**実機・ROS ノード・ブラウザでは再現していない** |
| 確度 | **確定**＝呼び出し元・配線まで追った／遷移を実際に回した。**要確認**＝論理は追えたが実行で確かめていない |
| ID | `SG-A*`（安全）／`SG-B*`（運用不能・データ消失）／`SG-C*`（表示）／`SG-D*`（spec 内部の矛盾）。spec の `G-n`（目標）と衝突しないよう `SG-` を付けた |

**着手前に必ず実物を見ること。**静的解析なので、読み違いがありうる。直したら行を取り消し線にしてコミットを書く。

---

## SG-A 安全（G1・SD-8 にかかわる）

| ID | 内容 | 根拠（コード） | 確度 |
| --- | --- | --- | --- |
| **SG-A1** | **盤前移動・呼び寄せ・待機場所への移動で、`PAUSE` に入っても経路追従（FollowPath）を取り消さない。**ジョグを離して約 1 s 後／回復フォルトが消えた瞬間／`BLOCKED` で「停止」した後に経路が空いたとき、**`PAUSE` のまま自律走行が再開する**（`SM-3.1.1-01`〜`-05` 違反）。`REPLAY` は `RUN` でしか走らないので無関係 | `transitions.yaml` の `C-01`・`C-03`・`T-PNAV-06`・`T-SUM-10`・`T-HNAV-06` に `cancel_follow_path` が無い。`venue_navigator.py:320-346`（`PAUSE` では何もしない）、`_recovery_eligible`（:53-67）はモードしか見ない。`test_venue_nav_pause_blocked_node.py` はこの経路を試験していない | 確定 |
| **SG-A2** | **呼び寄せの退避待ちを、非常停止 →「戻る」→「走行」で飛ばせる。**人が立っている地点へ発進する | `C-09c` → `SUMMON/PAUSE` → `T-SUM-07`（ガード無し）→ `resume_follow_path` → `venue_navigator._start_nav`（遷移を実際に回して確認） | 確定 |
| **SG-A3** | **非常停止から「元のモードへ戻る」（`C-09c`）の戻り先が `PAUSE` 固定。**spec `SM-3.1.1-11` は「§6 の `resume_state`」。`REPLAY` の `LOCALIZE`/`READY` から戻ると、確認と始点への向き合わせを飛ばして `RUN` に入れる。`OPCHECK`・`CALIB`・`PREP` では定義外の `X/PAUSE` に入る。`TEACH_MANUAL/ROUTE_SEL` から戻ると `start_record` を出さずに `REC` へ入る。CARRY 側の `C-11` は `$prev_state` なので問題ない（非対称） | `transitions.yaml` `C-09c`、`state_core._validate_names` は組み合わせを検査しない | 確定 |
| **SG-A4** | **ESP32 が、障害物・非常停止・フォルトによる停止指令（目標 0）も 1.5 m/s² のランプに通している。**PWM のスルー制限を外すのは、ランプ後の目標値がちょうど 0 になってからなので、0.55 m/s から約 0.37 s は鈍ったまま。`Spec-safety.md` §3「安全のための停止は鈍らせない」に反する。`brake_accel_mps2`=1.18 は 2026-08-21 の実測で、ランプ導入（09-11）より前 | `esp32/src/main.cpp:155-169`、`pid.h:31-48`（コメントの「障害物停止もすべて目標値 0 として届く」は、ランプ後の値については成り立たない） | 経路は確定。制動距離の伸びは要実測 |
| **SG-A5** | **起動直後（`INIT/CHECK`）に UI 非常停止を押して離すと `IDLE` に入る。物理非常停止が押されたままでも入る。**疎通確認を飛ばせる（`SM-3.1.2-001/-003`、`Spec-safety.md` §4.3 違反）。`C-09d`（「メインメニューへ」）にもガードが無く、重大フォルト中・押下中でも `IDLE` へ抜ける（`SM-3.1.1-12` 違反） | `C-06b` → `C-09b`（遷移を実際に回して確認）、`guards.py:93`、`C-09d` | 確定 |
| **SG-A6** | **走行中でも S-21 の「会場地図を開く」が押せる。**押すと SLAM が再起動し、自己位置が待機場所ピンの位置に置き直される。FollowPath は継続し、自己位置ロストの検知は「計画的な再起動」として保留される | `S21Test.jsx:401-410`（`disabledAll` だけでモードを見ない）、`slam_control.py:582-628`（`/map_session/open` にモード検査なし） | 経路は確定 |
| **SG-A7** | **試験準備の自動帰還（`PREP/RETURN`）で「停止」が効かない。**回復フォルトでも止まらず、フォルト解除後やジョグを離した後に勝手に再開する。`Spec-modes.md` §3.0-②（PREP には止める対象が無い）が、この自律走行を見落としている（→ SG-D4） | `T-PREP-10`、`guards.py` の `_NO_PAUSE_MODES`、`venue_navigator._blocked_recheck` | 確定 |
| **SG-A8** | **フォルトが 2 つ重なっていると、1 つ消えた時点で「解消」と判定される。**「はい」で走行状態へ戻れる。重大と回復が重なって回復だけ消えると `estop_resume_prev` が真になる | `safety_monitor.cpp:535-547`（`active_faults_` は集合で持つのに、解除時は `active=false, NONE` を出す）、`state_manager.py:569-575`（最後のメッセージで上書き） | 確定 |
| **SG-A9** | **「作業中」をサーバ側で確かめていない。**`working` フラグを立てる者がいない。守っているのは画面のボタン無効化だけ（複数端末なら作業中に発進しうる）。台帳 `W-12` のタグ「venue_navigator の working 購読」はコードに実体が無い | `guards.py:124-129`（`flags["working"]`）、S-21 は `ui.working` で状態を変えるだけ | 確定 |
| **SG-A10** | **地図の破棄・切替（`slam_control`）とパラメータ変更（`config_manager`）の「走行中は拒否」が、旧 FSM の `/robot/mode` を見ている。**旧 `mode_manager` は起動後ずっと `IDLE` を出すので、ガードは常に通る。許可条件にも `MANUAL` が入っている（SD-9「停止中だけ」に反する）。**注意: PREP の `discard_map` が動いているのは、このガードが壊れているから。**付け替えるだけでは直らない。観客ビューと音声も `/robot/mode` を見ている | `config_manager.py:86,106-115`、`slam_control.py:222,309-331`、`bringup.launch.py:402-408`（旧 `mode_manager` は今も起動）。`DetailedDesign-open.md:306` は「対応済み」と誤記 | 確定 |
| **SG-A11** | **速度指令の多重化の故障（`MUX_DEAD`）を監視していない。**`Spec-safety.md` §3.5 は重大フォルトに挙げている。`DetailedDesign-open.md` `N-24` が未決のまま | `bringup.launch.py:317` の `SAFETY_ENABLED_TARGETS` に `mux` が無い（`gazebo.launch.py:55` も同じ） | 確定 |
| **SG-A12** | **タブレットが途切れても `PAUSE` にならず、速度上限が 0 になるだけ。**戻ると操作なしで走行を再開する。`Spec-safety.md` §6.1 は「`PAUSE`」、詳細設計（`DetailedDesign-state.md` §6）と `Spec-safety.md` §6.2.1 は「一時停止ではない」（→ **2026-10-04 決定**: 走行中だけ `PAUSE`＋再開確認。下の「ユーザー決定」） | `zones.derive_limits` → `obstacle_limiter` の画面由来の上限。遷移表に該当するイベントが無い | 確定 |
| **SG-A13** | **始業点検 MOTOR は、最初に手を離した時点で判定が確定する。**前進だけで OK になり、左右モーターの入れ替わりを検出できない（入れ替わりが見えるのは旋回だけ。`Spec-checks.md` §2.4 #2 は 4 方向） | `opcheck_runner.py:369-405` | 確定（4 方向を必須とするかは spec の解釈しだい） |
| **SG-A14** | **`v_max` が 1.12 m/s。**`drivetrain_ceiling_mps`=1.4（暫定・データシート未確認）× 0.8 から導出している。spec の計画は 0.7 m/s、天井は約 0.91 m/s（`Spec-params.md` §1）。OUT ゾーンの上限と `obstacle_stop_distance_m` がこれに従う | `registry.yaml` | 確定 |
| **SG-A15** | **試験場内で経路が塞がれると、2 秒ごとに経路を計算し直して自動で迂回する。**`Spec-onsite.md` §6・`SM-3.1.2-060`「自動でルートを変えない」に反する。W-5 の「再検索」も中身は同じ処理。`blocked_lookahead_m` / `blocked_hold_ms` / `unblocked_hold_ms` は読まれていない（WS-9AK で実機の失敗を受けて入れた経緯がある → **2026-10-04 決定**: 迂回しない） | `venue_navigator.py:668-727`（`_blocked_recheck` → `_compute_and_follow`） | 確定 |
| **SG-A16** | `IDLE` から `ui.enter_mode` で `PANEL_NAV`/`SUMMON`/`HOME_NAV` に直接入れる。`goto_allowed`（作業中・ピンの有無）と `begin_two_point` を通らない。画面はこの送り方をしない | `mode_entry.yaml` の IDLE 行、`C-13` | 確定（発進先は要確認） |

## SG-B 運用不能・データ消失・「済み」の裏の欠陥

| ID | 内容 | 根拠（コード） | 確度 |
| --- | --- | --- | --- |
| **SG-B1** | **教示中に非常停止・手押し・重大フォルトが入ると、記録は自動保存されて閉じる。**元の教示へ戻っても記録は再開しない（`resume_record`・`finalize_route` は「記録中でない」として無視）。それでも画面は「記録中」「保存しました」を出す。**後半が無言で失われる。**地図も自動保存の時点で凍結される（WS-9K-D と WS-9O の組み合わせで生じた） | `route_recorder.py:317-325,345-369`、`route_record_core.should_autofinalize` | 確定 |
| **SG-B2** | **機体側から WebUI に宛てた effect を、WebUI が一切購読していない。**対象: `guide`・`ask_save_if_unsaved`・`ask_save`・`open_window`（W-3/W-5）・`show_resume`・`enable_main_menu`・`offer_calib`・`offer_opcheck`。そのため次が出ない: 終了時の保存確認／起動時に非常停止が押されたままの案内（§4.3）／呼び寄せの時間切れ・ロスト／登録中のロスト／記録が途切れたときの保存確認。`guides.js` はどこからも import されていない。キーも食い違う（遷移表は `summon_clear_timeout`・`summon_target_lost`・`register_target_lost`、`guides.js` は `wait_clear_timeout`・`target_lost`） | `state_manager.py:47-91`（`/system/effect` へ出す）、`web_ui/src` に購読 0 件、`Windows.jsx:25-29,227`（W-3 は置き場だけ） | 確定 |
| **SG-B3** | **教示の「終了」は確認なしで自動保存し、同じ名前の正常な経路を `.prev` へ押し出す**（`.prev` は UI から戻せない）。失敗した教示を捨てる手段が無い。`docs/使い方.md:226-228` の「保存せず終了すると一覧に出ない」は実装と逆 | SG-B1 と同じ自動保存。`S13TeachManual.jsx:65-72` | 確定 |
| **SG-B4** | **始業点検の項目 1（非常停止）は、手順どおり「押す → 離す」と操作すると、押した瞬間に `NG(stuck_release)` で確定して `REPAIR` へ移る。**後から来る解除は誰も見ない。OK になるのは「始める前から押していて、始めてから離す」場合だけで、画面の案内と逆。目視の「一致」回答も判定に入っていない。**`WP-MAINT-01` の完了条件「項目 1 が OK」を満たせない** | `opcheck_runner.py:464-476`（押下の時点で `_update_estop_verdict` → `_emit_result`）、`check_core.judge_estop`、`T-OPC-03` | 確定 |
| **SG-B5** | **始業点検の NG から「校正へ」は本番では押せない。**ボタンは `RUNNING_CHECK` のときだけ表示されるが、判定と同時に FSM は `LIST` へ戻る。しかも受理する `T-OPC-07` は `LIST` 限定。IMU の NG は、runner が `repair` に書き換え FSM は `LIST` に戻すので、故障診断にも校正にも行けない。e2e は本番では起きない状態を手で作って試験している。**計画書 §6 #11 は「済み（`71b1424`）」としている** | `S30Opcheck.jsx:83-99,187-192,223,328-364`、`transitions.yaml:1637-1680,1736-1745`、`opcheck_runner.py:604-605`、`e2e/s40-calib-guide.spec.js:20` | 確定 |
| **SG-B6** | **「運用の終了」に実体が無い。**`_unsaved` は常に空、`/shutdown/execute` は何も止めない。それでも画面は「停止が完了しました。電源を切って構いません」と出す。S-20 の手順バーも、一度も保存していないのに「保存 済み」を出す | `state_manager.py:177,612-626`、`S01Main.jsx:12-19`（既知の制限として書いてあるが計画に無い）、`onsiteSteps.js:91-98` | 確定 |
| **SG-B7** | **疎通確認が時間切れになっても、制御系を再起動しない。**`sys.link_timeout` → `restart_control_stack` の effect は出るが、`connectivity_checker` へ届ける経路が無い。S-00 の「再起動しています（n 回目）」表示も無い（`Spec-safety.md` §7、`Spec-webui.md` §3.1） | `transitions.yaml:325-334`、`connectivity_checker.py:54-61,419-442` | 確定 |
| **SG-B8** | **`/params/set` で書いた値が効かない。**`overrides.yaml` に出どころ付きで書かれるが、`params_generation` も `bringup` も読まない（再起動しても反映されない）。`/params/get` は重ねた値を返すので、実際に効いている値と表示がずれる。WebUI からの呼び出しも無い | `params_audit.py:54,331-360` | 確定 |
| **SG-B9** | **S-50 の一般タブが、本番の起動では読み込み全体に失敗する。**`use_stub:=true` でしか起動しない `follow_planner_mapless` の読み込み失敗を、全体の失敗として扱うため。そのうえ一般タブには、廃止予定の旧ノードの 14 項目（`v_max` は 1.5 まで）が並び、registry を経由せずに YAML へ保存できる | `S50Settings.jsx:45-60,175-190,300-313`、`tunable_targets.py:14-39` | 要確認 |
| **SG-B10** | **自動ブレーキが常に ON 固定。**場外の手動走行の既定 OFF（`Spec-safety.md` §2.1）も、試験員による切り替え（`Spec-webui.md` §3.5）も無い。`zones.derive_limits` の値も `attributes.yaml` の `auto_brake_default` も使われていない。S-11 は表示だけ（`DetailedDesign-wp3.md` c5 で「別パケット」とされたまま） | `state_manager.py:167,675`、`S11Manual.jsx:98-105` | 確定 |
| **SG-B11** | **保存したあとに記録を続ける機能（F-32、`SM-3.1.2-017/-018`）が、コードで意図して無効化されている。**spec・コード・`docs/使い方.md:236-239` の 3 つが食い違う（→ **2026-10-04 決定**: 保存＝確定。spec を改定） | `route_recorder.py:322-336,371-394`、`slam_control.py:680-705` | 確定 |
| **SG-B12** | **経路の旧版を 1 世代残すのは JSON だけで、地図は固定名で上書きされる。**`.prev` の経路を戻しても地図と合わない。`generation` は常に 1。**台帳 `W-04` は CLOSED だが、半分しか満たしていない** | `route_record_core.finalize_route_file`（:270-288）、`slam_control._map_session_base`（:572-580） | 確定 |
| **SG-B13** | **走行方式を選ぶ前の前提を判定していない**（`Spec-transit.md` §0.6）。経路が 0 本でも教示再生を押せる。`no_route_recorded` / `device_not_connected` を出す側が無い。「保管場所から開始してください」の案内も無い | `guards.py:115-121,241-256`（「対象外」と明記）、`mainMenuItems.js:56-80`、`reasons.js:17,19` | 確定 |
| **SG-B14** | **S-14 に「地図の更新」トグルと「保存」が無い。**FSM 側も `map_update_available=False` を固定で渡しているので、`T-REPLAY-08` は通らない。台帳 `W-11` は試験場内（S-21）だけを扱う | `state_manager.py:348`、`S14Replay.jsx:278-280`、`slam_control.py:672-674` | 確定 |
| **SG-B15** | **盤前到着の許容差は、実際には 0.15 m。**`venue_navigator` が 20 Hz で回す到着判定（`arrival_xy_tol_m`=0.15）が Nav2 の 0.12 より先に成立する。§6 #6 の実機確認はこの値を見ることになる | `venue_nav_core.arrived`（:47-50）、`venue_navigator.py:101,604-640`、`nav2_params.yaml:46` | 確定 |
| **SG-B16** | **ピンの位置・向きを再登録できない**（`Spec-onsite.md` §2.3）。`EditPin` は改名と削除だけで、登録し直すと別 id のピンになり名前も引き継がれない（`DetailedDesign-onsite.md:170` と不一致） | `EditPin.srv`、`pin_registrar.py:673-695` | 確定 |
| **SG-B17** | **バッテリーの計測・表示・警告がまったく無い**（`Spec-safety.md` §8、`Spec-webui.md` §3.2）。`battery_warn_v` / `battery_critical_v` は誰も読まない。ファームに電圧計測が無く、ADC も未割り当て。§6 0-D はゲートの話しかしていない | `registry.yaml`、`esp32/src` | 確定 |
| **SG-B18** | **`evt.record_broken`（`SM-3.1.2-019`。教示の連続性が切れた）を出すノードがいない。**台帳 `W-02` の「解除時にやること」にあるが、計画書の `W-02` の残り（横ずれ表示・ドリフト実測）から落ちている | `T-TEACH-06` はある。発行するコードが 0 件 | 確定 |
| **SG-B19** | **地図の無い経路を選ぶと `replay_runner` が無言で return する。**60 s 後に、原因と違う「始点マークに置き直す」案内が出る（`Spec-transit.md` §4.2.3）。§4.2.2 の待ち時間の表示も無い | `replay_runner.py:383-391` | 確定 |
| ~~**SG-B20**~~ **（2026-10-04・`9c06759` で解消。LED 表示は 1b-16 ③に残る）** | **`start.sh` が無い**（`Spec-ops.md` §1・§2。端末操作は `setup.sh`／`start.sh` の 2 か所だけのはず）。機体側の起動完了表示（`LED_STATE`）も、ファームにもブリッジにも無い（`Spec-ops.md` §2.5） | リポジトリに `start.sh` 0 件 | 確定 |
| ~~**SG-B25**~~ **（2026-10-04・`9c06759` で解消）** | **`setup.sh` が現行構成と合っていない**（`Spec-ops.md` §1 は導入を `setup.sh` 1 回で済ませると定める）。PC に udev ルールで `/dev/lidar`・`/dev/esp32` を作るが、LiDAR と ESP32 は 2026-09-05 以降ラズパイ側にある。WebUI は `npm install` だけで、本番ビルドが無い（`DetailedDesign-reuse.md:204` は「改修: WebUI の本番ビルドを含める」と定める）。末尾の「次のステップ」は廃止済み・古い手順（`slam.launch.py`・`map_saver`・`linear_calib.py`・`lidar_source` 指定なしの `bringup`・`npm run dev`）。ラズパイ側の導入（`pi_serial_relay` の systemd 登録など。`docs/network.md`）は含まれない。**2026-10-04 の追加調査で判明（初回の突き合わせの漏れ）** | `th_ws/setup.sh` | 確定 |
| **SG-B21** | **registry の行が読まれていない、または数値がノード内に直書きされている**（`W-03`/`W-13` と同じ型だが台帳に無い）。consumers に挙がっているのに読み手がいない: `two_point_spacing_m`・`nav_tolerance_m`/`_deg`・`esp32_ws_port`・`link_quality_regression_ratio`・`link_gap_p99_ms`・`auto_select_hold_s`。直書き: `calib_runner.py:85-95,120-123`、`opcheck_runner.py:80-83`、`check_core.py:27-30`、`venue_navigator.py:101`（`arrival_xy_tol_m`） | 各ファイル | 確定 |
| **SG-B22** | `ui.resume_yes`（`C-04`）が、`run_state` が null のモード（`AT_PANEL`/`AT_HOME`/`OPCHECK`/`CALIB`）で state=None を作る。`C-05` も `PREP` で同じ（SG-A3 の `PREP/PAUSE` から到達できる）。publish の型検査で `state_manager` が落ちうる。画面は通常このイベントを送らない | 遷移を実際に回して確認 | 遷移は確定。落ちるかは要確認 |
| **SG-B23** | 要確認 4 件: ① PREP の地図作成が、slam の今の地図に書き足すだけ（同じ起動中に読んだ経路地図が混入しうる。空にする手段はピンが 1 件以上あるときしか出ない）② 待機場所の宣言による照合は、地図を開く初期姿勢＝HOME ピンなので、補正分しか測れない（§4.0.1 の「地図ずれの測定器」が成り立たない可能性）③ S-14 の再生速度: 未接続の最初の描画で送ったことにしてしまい、表示と実速度がずれうる（`useReplaySpeedPublisher.js:35-48`）④ 導出の `esp32_timeout_ms`=322 が `esp32_watchdog_ms`=600 より小さい（A6 警告に当たる可能性） | — | 要確認 |

## SG-C 表示・文言

| ID | 内容 | 根拠 |
| --- | --- | --- |
| SG-C1 | フォルト 7 種（`DRIVE_RUNAWAY`・`LOCALIZATION_LOST`・`LIMITER_DEAD`・`MUX_DEAD`・`STATE_INCONSISTENT`・`ESTOP_BYPASS_ACTIVE`・`UI_DISCONNECTED`）の日本語が無く、ヘッダと W-1 に「種別不明」と出る。ESP32 を「Wi-Fi 接続」、人物追跡を「カメラの視界」とする古い文言も残っている | `i18n/faults.js:9-14` |
| SG-C2 | 操作カードの色が `U-17`（`Spec-webui.md` §3.3.1）と逆。「走行」が常に青の塗り、「停止」が常に赤、「保存」が枠だけ。「確認」が青のとき青が 2 つ並ぶ。ボタンの幅も変わる | `theme.css:448-449,638-654` |
| SG-C3 | ヘッダにモード別の色が無い。開発モードでもヘッダの色が変わらない | `Header.jsx:68`、`theme.css:95-99` |
| SG-C4 | S-00 にタブレット行が無い。AP 行に状態が無い。総合欄が「必須機器が繋がっていません」を出さず「確認中」のまま。自動点検の結果を OK/NG/WARN の英字で出す | `S00Connect.jsx:66-100` |
| SG-C5 | S-13 の教示タブ: 開始時の向きが一度も表示されない（`RouteStatus` の最上位に `start_yaw` が無い）。「記録中」が画面ローカルのフラグで決まり、赤くない（`PAUSE`/`SAVED` でも出る。再読み込みすると入力フォームに戻る）。既存経路を選び直すリストが無い（同じ名前を入れると黙って上書き）。spec に無い「記録開始」ボタンがある。自動ブレーキの表示が無い | `S13TeachManual.jsx:62,84,208-213`、`route_recorder.py:484-502` |
| SG-C6 | S-14 の地図に始点・終点の印が無い | `RoutePreview.jsx` |
| SG-C7 | 方式 C の地図タップ登録で、ドラッグ中に x,y が出ない（yaw だけ）。数値で補正する入力欄も無い（`Spec-onsite.md` §3.7） | `OnsiteMap.jsx:440-459`、`S20Prep.jsx:670-692` |
| SG-C8 | 待機場所の宣言に成功したとき、ずれ量を出さない。退避待ちのバーは 15 s で計算しているが、実際の時間切れは 30 s。見失っている間も距離が最後の値のまま | `S21Test.jsx:117,123-128,281-287` |
| SG-C9 | 「後方は死角があります」の表示が S-11・S-13 にしか無い（W-6・S-14 に無い）。ジョグ中の自動ブレーキの状態も W-6 に出ない（`Spec-modes.md` §8） | — |
| SG-C10 | 重大フォルトで入った ESTOP でも、押しても何も起きない「非常停止を解除」バーを出す。CARRY 中に UI 非常停止を押した後の停止理由を「障害物」と誤表示する（要確認） | `AppShell.jsx:249`、`limits.js:stopReason` |
| SG-C11 | W-6 が操作カードの停止・走行・保存で閉じない | `OperationCard.jsx:35-38` |
| SG-C12 | 破壊的な操作が二段階アーム式（`Spec-webui.md` §6）になっていない: ピン削除・校正値のロールバック・S-13 の同名上書き | `S20Prep.jsx:821-829`、`S40Calib.jsx:457-462` |
| SG-C13 | 始業点検のモニターが足りない（IMU の sys/gyro/accel/mag のリアルタイム表示、LiDAR の生スキャンのライブ表示と死角マスクの重ね描き。`Spec-checks.md` §2.4 #3・#4） | `S30Opcheck.jsx:22-28,162-195` |
| SG-C14 | S-50 に spec の項目が無い（速度プリセットの割合・自動ブレーキの既定・音声の有無・縦横の固定・地図の表示項目） | `S50Settings.jsx:8-14` |
| SG-C15 | 校正履歴の実施者が固定値 `"webui"`。始業点検の誤差が「悪化」したときの要再校正（`Spec-checks.md` §3.7 の 2 条件目）が無い | `calib_runner.py:166,868` |
| SG-C16 | 開発モードで選んだログが ROS ログ（`/root/.ros/log`）にしか出ず、docker ワークスペースの外にある（`Spec-params.md` §6）。要確認 | `connectivity_checker.py` |

## SG-D spec 内部の矛盾（spec 側の修正が要る）

| ID | 食い違い | コードはどちら |
| --- | --- | --- |
| SG-D1 | 重大フォルト起因の ESTOP の解除先。`Spec-safety.md` §3.5 の表・§4.1、`Spec-modes.md` §4.1・§4.3・§5・§6 は「`IDLE` のみ」。`Spec-safety.md` §3.5.2、`SM-3.1.1-06`/`-11` は「元のモードへ戻れる」 | 後者 |
| SG-D2 | `SM-3.1.1-03`「`PAUSE` を持たない 3 モード」→ `PREP` を加えて 4 モード。`Spec-modes.md` §3 の「`WAIT_CLEAR` に `PAUSE` は無い」と `-03`「任意の状態 → `PAUSE`」が衝突している（SG-A2 の入口） | `PAUSE` に落とす |
| SG-D3 | `Spec-safety.md` §6.1（タブレットが途切れたら `PAUSE`）と §6.2.1（一時停止ではない） | 後者（SG-A12） |
| SG-D4 | `Spec-modes.md` §3.0-②（PREP には止める対象が無い）が、自動帰還（RETURN）を見落としている | SG-A7 |
| SG-D5 | `Spec-onsite.md` §2.1（塞がれた状態かを問わず再試行）と §6（自動でルートを変えない）。到達許容 0.25 m（`Spec-onsite.md` L36・L194、`Spec-params.md` §1・§3）と 0.12 m（`Spec-onsite.md` §6.2）が混在 | 自動再試行（SG-A15） |
| SG-D6 | `Spec-params.md` §6（車輪半径は書き込みで適用）と `Spec-checks.md` §3.6（PC 側のスケール係数）。`Spec-params.md` §1 の「現状」値（2000 ms・0.45 m・0.5 m/s²）が古い | 後者 |
| SG-D7 | W-5 の解除条件（`Spec-webui.md` §4「再検索まで出続ける」と `SM-3.1.2-058`「通れたら自動再開」）。`Spec-webui.md` §1.4 の文字サイズ表と §7 の固定キャンバス。`Spec-modes.md` §8・§4.3 の AT_HOME の書き漏れ。§7 の S-01/S-50 の速度上限（`v_max`。コードは `stop`）。`Spec-onsite.md` §5 と `SM-3.1.2-069` の軽い食い違い | — |

## 計画書・台帳・詳細設計の記述が事実と違うもの

| 記述 | 事実 | 関連 |
| --- | --- | --- |
| ImplementationPlan §6 #11「始業点検の項目 4 から S-40 へ誘導（済み・`71b1424`）」 | 本番の遷移では押せない | SG-B5 |
| ImplementationPlan §6 10-b の残り「項目 1 が OK になることを実機で確かめる」 | 手順どおりに操作すると必ず NG | SG-B4 |
| ImplementationPlan §4 段階 1 #4「W-1〜W-6 実機確認済み」 | W-3 に表示先が無い | SG-B2 |
| ImplementationPlan §4 段階 0 #3 `WP-PARAM-02`「配線済み」 | `/params/set` の値が効かない | SG-B8 |
| 台帳 `W-04` CLOSED | 地図は旧版を残さない | SG-B12 |
| 台帳 `W-12` のタグ「venue_navigator の working 購読」 | コードに実体が無い | SG-A9 |
| `DetailedDesign-open.md:306`「`/robot/mode` → `/system/state` 対応済み」 | 移行していない | SG-A10 |

## ユーザー決定（2026-10-04）

判断が要った 5 件（コードが意図して spec から外れていたもの）の決定。**spec は同日に改定済み。**

| ID | 決定 | spec の改定 | 実装でやること |
| --- | --- | --- | --- |
| `SG-B11` | **保存＝記録の確定。**保存後は記録を続けない（F-32 のうち教示の分を撤回） | `Spec-transit.md` §3.2、`Spec-modes.md` `SM-3.1.2-017/-018`・§3、`Spec-webui.md` §3.4、`Spec-open.md` F-32 | `SAVED` でスティック・「走行」を拒否し、状態だけ `REC` になる誤表示を無くす（1b-7）。`docs/使い方.md` は改定済み |
| `SG-A15` | **自動で迂回しない。**自動の再試行は決めた経路の残りでの走り直しだけ。別ルートは「再検索」のときだけ | `Spec-onsite.md` §2.1・§6、`Spec-modes.md` `SM-3.1.2-058`、`Spec-webui.md` W-5 | `venue_navigator` の再探索を、保持した経路の残りの再送に変える（1b-14）。最初の計画失敗時の経路探しは残す |
| `SG-A12` | **走行中に端末が離れたときだけ `PAUSE` にし、戻ったら再開を問う。**走行中でなければ速度 0 だけで確認は出さない。**30 秒（`tablet_active_window_s`）は実測して見直す** | `Spec-safety.md` §6.1・§6.2.1・§6.2.2（新設）、`Spec-modes.md` `SM-3.1.1-03`、`Spec-params.md` | 在席喪失を走行中だけ回復フォルト相当で `PAUSE` に落とす（1b-1）。実測は実機 |
| `SG-A14` | **天井をファームの出力上限から逆算した 0.71 m/s にする**（`v_max` ≈ 0.57） | `Spec-params.md` §1・§2 | `registry.yaml` の `drivetrain_ceiling_mps` を 0.71 に（1b-10） |
| `SG-B10` | **spec どおり**（場外の手動は既定 OFF・警告のみ、場内は既定 ON、切替可）。**実装時に注意表示の追加を検討する** | 変更なし | 切替の経路と既定値の適用を作る（1b-15） |

**2026-10-05 追加の決定（spec 改定済み）**

| ID | 決定 | spec の改定 | 実装でやること |
| --- | --- | --- | --- |
| `SG-D1`／`SG-A3` | **`ESTOP` からの「戻る」は §6 の復帰先へ**（走行中なら `PAUSE`、止まっている状態ならその入口から、`SUMMON`→`POINT` 等）。`PAUSE` を持たないモードに `PAUSE` を作らない | `Spec-modes.md` `SM-3.1.1-11`・§4.1・§6、`Spec-safety.md` §3.5・§3.5.2・§4.1 | `C-09c` の戻り先（1b-2） |
| `SG-A5` | **押下前が `INIT` なら `INIT/CHECK` へ戻し疎通確認からやり直す。**「メニューへ」（`C-09d`）もフォルト解消・両非常停止解放のときだけ | 同上 | `C-09b`／`C-09d`（1b-2） |
| `SG-D2`／`SG-A2` | **`SUMMON` の戻り先は `POINT`**（退避待ちからやり直す。§6 の既存の右列どおり） | `SG-D1` に含む | `SG-D1` と同じ（1b-2） |
| `SG-D4`／`SG-A7` | **`PREP` の `RETURN` だけ `PAUSE` を持つ。**「はい」で経路の残りから帰還を続け、「いいえ」で `MAPPING` | `Spec-modes.md` §3.0-②・§3・§6 | 1b-1 の残り |
| `SG-A13` | **始業点検の MOTOR は 4 方向すべてで合って OK** | `Spec-checks.md` §2.4 #2 | 1b-8 |
| `SG-A14` の付随 | **`v_leash` を暫定 0.5 m/s にする**（天井 0.71 にすると 1.0 が `v_max` と天井を超え、検査 A13 で生成が止まるため） | `Spec-params.md` §2 | 1b-10 |

これに伴い `SG-D3`（タブレット）・`SG-D5` のうち再試行の中身・`SG-D7` のうち W-5 の解除条件は解消した。

## 確認して spec どおりだったもの（再調査を省くため）

spec の `SM-*` の ID はすべて `transitions.yaml` にある（欠落 0）。`mode_entry.yaml`（SG-A16 を除く）と `attributes.yaml` は spec と一致。ほかに spec どおりだったもの:

- 自己位置監視の範囲（監視するモード・非常停止中の扱い）
- `critical_fault_hold_ms`
- `DRIVE_RUNAWAY` の鮮度による凍結
- 障害物を k=3 番目の距離で判定すること
- 開発モードの `scan_stop`/`lidar_fault` と 3 秒での失効
- UI 非常停止の 2 Hz 継続送信・ラッチ
- W-1 の非表示 → バッジ → 再開確認
- ウォッチドッグ 600 ms
- 非常停止ボタンの寸法と z 順
- その場旋回・比率×上限・リース
- `REPLAY` の `PAUSE`（`RUN` でしか走らない）
- 人物追跡の自動停止と OFF の拒否
- 待機場所到着時の向き合わせ
- costmap のクリア
- 壁からの距離の検査
