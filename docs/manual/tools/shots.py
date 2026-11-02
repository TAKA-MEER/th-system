"""図解マニュアルのショット定義。

1 ショット = 1 枚の PNG。capture.py が 1 枚ごとにモックアップを読み直してから撮る。

  id      PNG の名前。index.html の <figure data-shot="..."> と対応する
  setup   状態を作る JS（モックアップの関数をそのまま呼ぶ）。__tab('r14b') でタブを開く
  clip    切り抜く要素（省略すると画面全体 1280×720）。pad は余白（論理 px）
  marks   (番号, 要素, 番号丸の位置) のならび。位置は tl/tr/bl/br/t/b/l/r/c（既定 tl）
          要素は CSS セレクタ。"css::文字列" でその文字列を含むものに絞る。" || " で矩形を合併

注釈の文言は index.html の凡例に書く。ここには番号と要素だけを置く。
実機の画面で撮り直すときは、setup と要素の指定をここで差し替える。
"""


def S(id, setup="", marks=(), clip=None, pad=12, **kw):
    return dict(id=id, setup=setup, marks=list(marks), clip=clip, pad=pad, **kw)


RUN14 = "go('s14'); act(null,'s14','run');"
HOME21 = "go('s21'); declareHome(true);"

SHOTS = [
    # ---------------- 1 画面の見方 ----------------
    S("layout", "go('s11')", [
        (1, "#screenName", "b"), (2, "#modePill", "bl"), (3, "#faultBox", "b"), (4, "#estopBtn", "bl"),
        (5, "#s11 .top-actions .btn::終了", "bl"), (6, "#s11 .top-actions .tabs", "b"),
        (7, "#s11 .card.ops .opsgrid", "l"), (8, "#st11", "l"),
        (9, "#s11 .stick", "tl"), (10, "#s11 .slider", "l"), (11, "#s11 .spd", "tr"),
    ]),
    S("hdr-normal", "go('s21')", [
        (1, "#screenName", "l"), (2, "#zonePill", "b"), (3, "#modePill", "b"), (4, "#faultBox", "l"), (5, "#estopBtn", "l"),
    ], clip="#hdr", pad=0),
    S("hdr-fault", RUN14 + "raiseFault('LIDAR_LOST');", clip="#hdr", pad=0),
    S("hdr-estop", RUN14 + "engageEstop();", clip="#hdr", pad=0),
    S("hdr-carry", RUN14 + "hwEstop(true);", clip="#hdr", pad=0),
    S("hdr-dev", "go('s01'); toggleDev();", [(1, "#devPill", "bl")], clip="#hdr", pad=0),
    S("ops-pause", "go('s14')", [(1, "#s14 .op-stop", "l"), (2, "#st14", "l")], clip="#s14 .card.ops .opsgrid || #st14"),
    S("ops-run", RUN14, [(1, "#s14 .op-run", "r"), (2, "#st14", "l")], clip="#s14 .card.ops .opsgrid || #st14"),
    S("ops-teach", "go('s13')", [(1, "#s13 .op-save", "l")], clip="#s13 .card.ops .opsgrid || #st13"),

    # ---------------- 2 起動と終了 ----------------
    S("s00-ok", "go('s00')", [
        (1, "#s00 .card::疎通確認", "tl"), (2, "#s00 .lst tr::ESP32（ホイール", "r"),
        (3, "#s00 .lst tr::カメラ（区画線用）", "r"), (4, "#s00ok", "l"), (5, "#s00go", "r"),
        (6, "#s00 .card::始業点検の自動判定", "tl"),
    ]),
    S("s00-ng", "conn(false)", [(1, "#s00ng", "l"), (2, "#s00dev", "r")]),
    S("s01", "go('s01')", [
        (1, "#s01 .card::ネットワーク接続確認", "tl"), (2, "#s01 .card::教示は保管場所から", "tl"),
        (3, "#s01 .btn.dis::ライン誘導", "r"), (4, "#s01 .card::試験準備", "tr"), (5, "#s01 .card::自律測定", "tr"),
        (6, "#s01 .card::始業点検", "tr"), (7, "#s01 .card::駆動用バッテリー", "tr"), (8, "#shutdownBtn", "r"),
    ], allow_scroll=True),
    S("s01-disabled", "go('s01'); whyDisabled('カメラが接続されていません');", clip="#winDisabled"),
    S("shutdown-list", "go('s01'); shutdown();", [
        (1, "#uv-map", "l"), (2, "#uv-map .btn::保存", "t"), (3, "#uv-map .btn::破棄", "tr"),
        (4, "#shutdownBlocked", "l"), (5, "#doShutdownBtn", "r"),
    ], clip="#winShutdown", pad=16),
    S("shutdown-ready", "go('s01'); shutdown(); keepUnsaved('uv-map'); keepUnsaved('uv-route'); resolveUnsaved('uv-calib','破棄しました');",
      [(1, "#unsavedNone", "l"), (2, "#doShutdownBtn", "r")], clip="#winShutdown", pad=16),
    S("shutdown-done", "go('s01'); shutdown(); keepUnsaved('uv-map'); keepUnsaved('uv-route'); keepUnsaved('uv-calib'); doShutdown(); $('body').scrollTop=9999;",
      [(1, "#shutdownDone", "l")], clip="#s01 .card::停止が完了しました", wait=1500),

    # ---------------- 3 移動（試験場の外） ----------------
    S("s11", "go('s11')", [
        (1, "#s11 .note::前方 0.6 m", "l"), (2, "#s11 .brakeOffWarn", "l"), (3, "#s11 .row::自動ブレーキ", "l"),
        (4, "#s11 .brakePill + .btn", "tr"), (5, "#s11 .card::後退", "l"), (6, "#s11 .op-stop", "l"),
        (7, "#s11 .stick", "tl"), (8, "#s11 .slider", "l"), (9, "#s11 .spd", "tr"), (10, "#s11 .kbdHint", "l"),
    ]),
    S("s11-brake-on", "go('s11'); setBrake(true);", [(1, "#s11 .row::自動ブレーキ", "l")],
      clip="#s11 .card::障害物", pad=8),
    S("s11-brake-confirm", "go('s11'); setBrake(true); toggleBrake();", [
        (1, "#winBrakeOff .btn::やめる", "l"), (2, "#winBrakeOff .btn::OFF にする", "r"),
    ], clip="#winBrakeOff", pad=16),
    S("s13-teach", "go('s13'); __tab('t13b');", [
        (1, "#s13 .top-actions .tab::教示", "b"), (2, "#rec13", "r"), (3, "#s13 input.txt", "l"),
        (4, "#s13 .btn::この名前で記録を始める", "l"), (5, "#s13 table.lst::保管場所→試験場 A", "l"),
        (6, "#s13 .card::記録の状況", "tr"),
    ]),
    S("s13-recording", "go('s13'); act(null,'s13','run'); document.querySelector('#s13 .stick').classList.add('act');", [
        (1, "#rec13", "r"), (2, "#s13 .row::自動ブレーキ", "l"), (3, "#st13", "l"), (4, "#s13 .op-save", "r"),
        (5, "#s13 .stick", "tl"),
    ]),
    S("s13-paused", "go('s13')", [(1, "#st13", "l"), (2, "#hint13", "l")], clip="#s13 .card.ops"),
    S("s13-saved", "go('s13'); act(null,'s13','run'); saveRoute('s13');", [(1, "#st13", "l"), (2, "#hint13", "l")],
      clip="#s13 .card.ops"),
    S("s13-save-confirm", "go('s13'); finish13();", [
        (1, "#winSave .btn::破棄する", "l"), (2, "#winSave .btn::保存する", "r"),
    ], clip="#winSave", pad=16),
    S("s14-route", "go('s14')", [
        (1, "#s14 .mapWrap", "tl"), (2, "#s14 .card.ops .opsgrid", "l"), (3, "#s14 .row::地図の更新", "l"),
        (4, "#s14 .subtabs > .tabs", "l"), (5, "#s14 table.lst::保管場所→試験場 A", "l"),
        (6, "#s14 .btnrow::順再生", "l"), (7, "#s14 .btn::この経路で進む", "l"),
    ]),
    S("s14-pose", "go('s14'); __tab('r14b');", [
        (1, "#r14b .note", "l"), (2, "#r14b .row::確度", "l"), (3, "#r14b .btn::完全グローバル", "l"),
    ], clip="#s14 .subtabs", pad=20),
    S("s14-global", "go('s14'); __tab('r14b'); openWin('winConfirmGlobal');", clip="#winConfirmGlobal", pad=16),
    S("s14-run", RUN14 + "__tab('r14c');", [
        (1, "#s14 .op-run", "r"), (2, "#st14", "l"), (3, "#s14 .op-stop", "l"), (4, "#s14 .op-manual", "r"),
        (5, "#r14c .card::再生速度", "l"), (6, "#r14c .card::経路からのずれ", "l"),
    ]),
    S("s14-jog", RUN14 + "openJog('s14');", [
        (1, "#jwMode", "l"), (2, "#jogWin .jw-s", "l"), (3, "#jogWin .stick", "r"),
        (4, "#jogWin .btn::閉じる", "r"), (5, "#s14 .op-manual", "r"),
    ]),
    S("s14-paused", "go('s14')", [(1, "#pause14", "l")], clip="#s14 .card.ops"),
    S("s14-end", "pauseEnd14()", [(1, "#pause14", "l")], clip="#s14 .card.ops"),

    # ---------------- 4 試験場 ----------------
    S("s20", "go('s20')", [
        (1, "#steps20", "tl"), (2, "#s20 .top-actions .btn::終了", "bl"), (3, "#s20 .top-actions .tabs", "b"),
        (4, "#unsaved20", "r"), (5, "#map20 svg", "tl"), (6, "#next20", "r"), (7, "#s20 .card.ops", "r"),
        (8, "#s20 .subtabs > .tabs", "l"),
    ]),
    S("s20-gate", "go('s20'); mapGates(true);", [
        (1, "#gate20 .btn::地図作成開始", "l"), (2, "#gate20 .btn::新しい試験日", "l"),
    ], clip="#gate20", pad=20),
    S("s20-tracker-off", "go('s20'); __tab('t20b'); tracker('off');", [(1, "#s20 .trackerCtl .btn", "l")],
      clip="#s20 .trackerCtl", pad=20),
    S("s20-select", "go('s20'); __tab('t20b'); tracker('found');", [
        (1, "#s20 .trackerCtl", "r"), (2, "#s20 .radarWrap", "bl"), (3, "#s20 .cand.sel", "tr"),
    ], clip="#s20 .trackerCtl || #s20 .radarWrap || #t20b .hint", pad=20),
    S("s20-register", "go('s20'); p2start('配電盤');", [
        (1, "#r20a .row::登録方式", "l"),
        (2, "#r20a .btn::人の位置で待機場所を登録 || #r20a .btn::人の位置で配電盤を登録", "l"),
        (3, "#r20a .btn::いまの姿勢で待機場所を登録 || #r20a .btn::いまの姿勢で配電盤を登録", "l"),
        (4, "#r20a .btn::地図タップで待機場所を登録 || #r20a .btn::地図タップで配電盤を登録", "l"),
        (5, "#p2well", "l"),
    ], clip="#s20 .subtabs", pad=20),
    S("s20-p2-step2", "go('s20'); p2start('配電盤'); p2press();", [(1, "#p2msg", "t")], clip="#p2well", pad=20),
    S("s20-maptap", "mapTap20()", [(1, "#tap20 .note", "l"), (2, "#tap20 .row.mb", "l"), (3, "#tap20 .btn::この位置で確定", "l")],
      clip="#tap20", pad=20),
    S("s20-wall", "wallWarn20()", [
        (1, "#wall20 .btn::このまま登録", "bl"), (2, "#wall20 .btn::壁から離して登録", "b"), (3, "#wall20 .btn::やめる", "br"),
    ], clip="#wall20", pad=20),
    S("s20-pins", "go('s20'); __tab('r20b');", [
        (1, "#r20b table", "l"), (2, "#r20b .btn::編集", "r"), (3, "#r20b .btn::1 ボタンで待機場所に戻す", "l"),
    ], clip="#s20 .subtabs", pad=20),
    S("s20-pin-edit", "go('s20'); editPin('配電盤 A-1');", [
        (1, "#winPin .btn::地図タップで位置と向きを登録し直す", "l"), (2, "#winPin .btn::いまの機体位置で更新", "l"),
        (3, "#winPin .btn::削除", "bl"),
    ], clip="#winPin", pad=16),
    S("s20-save-confirm", "go('s20'); finish20();", [
        (1, "#winSave20 .btn::やめる", "bl"), (2, "#winSave20 .btn::保存せず終了", "b"), (3, "#winSave20 .btn::保存して終了", "br"),
    ], clip="#winSave20", pad=16),
    S("s21", "go('s21')", [
        (1, "#zonePill", "bl"), (2, "#steps21", "tl"), (3, "#modeSub21", "b"), (4, "#map21 svg", "tl"),
        (5, "#s21 .row::地図の更新", "l"), (6, "#s21 .next-action", "r"), (7, "#s21 .card.ops", "r"),
        (8, "#homeBtn", "tr"), (9, "#r21a table", "r"), (10, "#s21 .card::次の行き先", "l"),
    ]),
    S("s21-gate", "go('s21'); mapGates(true);", [(1, "#gate21 .btn", "l")], clip="#gate21", pad=20),
    S("s21-mismatch", "go('s21'); declareHome(false);", [
        (1, "#winHomeMismatch .btn::あとで", "bl"), (2, "#winHomeMismatch .btn::強制的に宣言", "br"),
    ], clip="#winHomeMismatch", pad=16),
    S("s21-declared", HOME21, [(1, "#homePill", "tl"), (2, "#homeOffset", "l")], clip="#r21a .card", pad=20),
    S("s21-moving", HOME21 + "goPanel('配電盤 A-2');", [
        (1, "#modePill", "bl"), (2, "#steps21", "tl"), (3, "#st21", "l"),
    ], allow_scroll=True),
    S("s21-blocked", HOME21 + "goPanel('配電盤 A-2'); openWin('winBlocked');", [
        (1, "#winBlocked .btn::中断", "bl"), (2, "#winBlocked .btn::再検索する", "br"),
    ], clip="#winBlocked", pad=16),
    S("s21-summon", HOME21 + "startSummon(); clearInterval(summonTimer);", [
        (1, "#modePill", "bl"), (2, "#s21 .subtabs .tab::呼び寄せ", "b"), (3, "#clearBox", "l"),
        (4, "#clearBox .btn::中止", "r"), (5, "#s21 .top-actions .tab::対象選択", "b"),
    ]),
    S("s21-panel", HOME21 + "S.mode='AT_PANEL'; paintHeader(); sub21('配電盤前'); __tab('r21c');", [
        (1, "#workBtn", "l"), (2, "#s21 .btn::カメラ", "r"),
    ], clip="#s21 .subtabs", pad=20),
    S("s21-working", HOME21 + "__tab('r21c'); startMeasure();", [
        (1, "#map21b", "l"), (2, "#workRow .pill", "l"), (3, "#workRow .btn::停止を解く", "r"), (4, "#modeSub21", "r"),
    ]),
    S("s21-work-blocked", HOME21 + "__tab('r21c'); startMeasure(); nextDest('待機場所');", clip="#winWorkBlocked", pad=16),
    S("s21-stopbanner", "go('s21'); stopBanner(true);", [(1, "#stopBanner", "r")], allow_scroll=True),

    # ---------------- 5 保守・設定 ----------------
    S("s30", "go('s30')", [
        (1, "#s30 .ocrows", "tl"), (2, "#s30 .ocrow.sel .pill", "t"), (3, "#s30 .ocrows .row > .btn::校正へ", "tr"),
        (4, "#s30 .card::総合", "l"), (5, "#oc4 table.basis", "l"), (6, "#oc4 .well", "l"), (7, "#oc4 .btnrow", "l"),
    ]),
    S("s30-estop", "go('s30'); __ocrow(0);", [(1, "#ocAbort", "l"), (2, "#oc1 table", "l")]),
    S("s30-motor", "go('s30'); __ocrow(1);", [
        (1, "#oc2 .row.mb.sm", "l"), (2, "#oc2 .btnrow", "l"), (3, "#oc2 table", "l"),
    ], clip="#ocAbort || #oc2", pad=20),
    S("s31", "go('s31')", [
        (1, "#s31 .btn::始業点検へ戻る", "bl"), (2, "#s31 .card::症状", "l"), (3, "#s31 .card::チェック手順", "l"),
    ]),
    S("s40", "go('s40')", [
        (1, "#steps40", "tl"), (2, "#s40 .top-actions .btn::中断", "bl"), (3, "#s40 .top-actions .btn.dis::終了", "b"),
        (4, "#s40 .top-actions .tabs", "b"), (5, "#t40a table.lst", "l"), (6, "#s40 .subtabs > .tabs", "l"),
        (7, "#r40b .card", "l"),
    ]),
    S("s40-step1", "go('s40'); cw(1-cwN);", [(1, "#cw1 .btn", "l")], clip="#r40b .card", pad=20),
    S("s40-step2", "go('s40'); cw(2-cwN);", [(1, "#cw2 input", "l"), (2, "#cw2 .btn", "r")], clip="#r40b .card", pad=20),
    S("s40-step3", "go('s40'); cw(3-cwN);", [(1, "#cw3 .kv", "l"), (2, "#cw3 .btn", "l")], clip="#r40b .card", pad=20),
    S("s40-step4", "go('s40'); cw(4-cwN);", [(1, "#cw4 .row", "l"), (2, "#cw4 .note", "l"), (3, "#cw4 .btn::始業点検へ戻る", "l")],
      clip="#r40b .card", pad=20),
    S("s40-history", "go('s40'); __tab('t40b');", [(1, "#t40b .btn::この値に戻す", "r")], clip="#t40b .card", pad=20),
    S("s40-imu", "go('s40'); __tab('r40c');", [(1, "#r40c .card > div.sm[style]", "l"), (2, "#r40c .btn::確定する", "l")],
      clip="#r40c .card", pad=20),
    S("s40-blind", "go('s40'); __tab('r40d');", [
        (1, "#r40d svg", "l"), (2, "#r40d table", "l"), (3, "#r40d .row::総幅", "l"), (4, "#r40d .tone-warn", "l"),
        (5, "#r40d .btnrow", "l"),
    ], clip="#r40d .card", pad=20),
    S("s40-abort", "go('s40'); openWin('winAbortCalib');", clip="#winAbortCalib", pad=16),
    S("s50", "go('s50')", [
        (1, "#s50 .top-actions .btn::戻る", "bl"), (2, "#s50 > .top-actions .tabs", "b"), (3, "#set1 > .tabs", "b"),
        (4, "#g1 .btn::保存", "r"), (5, "#g1 .note", "l"), (6, "#g1 table", "l"), (7, "#g1 .row::変えた理由", "l"),
    ]),
    S("s50-brake", "go('s50'); __tab('g2');", clip="#g2 .card", pad=20),
    S("s50-speaker", "go('s50'); __tab('g3');", [(1, "#g3 .btn::スピーカー ON", "r"), (2, "#g3 input[type=range]", "l"), (3, "#g3 .btn::試しに鳴らす", "l")],
      clip="#g3", pad=20),
    S("s50-loc", "go('s50'); __tab('g4');", clip="#g4 .card", pad=20),
    S("s50-blind", "go('s50'); __tab('g5');", [(1, "#g5 .btn::校正で変更する", "r")], clip="#g5 .card", pad=20),
    S("s50-display", "go('s50'); __tab('set2');", clip="#set2", pad=20),
    S("s50-dev", "go('s50'); __tab('set3');", [
        (1, "#devBtn", "l"), (2, "#set3 .card::無視する警告", "tl"), (3, "#set3 .card::いま無視しているもの", "tl"),
    ]),
    S("s50-history", "go('s50'); __tab('set4');", [(1, "#set4 .fh-row.ongoing", "l")]),

    # ---------------- 6 異常・非常停止 ----------------
    S("estop", RUN14 + "engageEstop();", [
        (1, "#estopBtn", "bl"), (2, "#faultBox", "l"), (3, "#winEstop", "tl"), (4, "#release", "tl"),
    ], allow_scroll=True),
    S("estop-released", RUN14 + "engageEstop(); releaseEstop();", [
        (1, "#weFoot .btn::メインメニューへ", "bl"), (2, "#weFoot .btn::元のモードに戻る", "br"),
    ], clip="#winEstop", pad=16),
    S("carry", RUN14 + "hwEstop(true);", [(1, "#estopBtn", "bl"), (2, "#faultBox", "l"), (3, "#winCarry", "tl")]),
    S("carry-released", RUN14 + "hwEstop(true); hwEstop(false);", [
        (1, "#carryResume .pill", "l"), (2, "#carryResume .btn::再開する", "l"), (3, "#carryResume .btn::再開しない", "l"),
    ], clip="#winCarry", pad=16),
    S("fault", RUN14 + "raiseFault('LIDAR_LOST');", [(1, "#faultBox", "l"), (2, "#winFault", "tl"), (3, "#wfFoot .btn::非表示", "r")]),
    S("fault-hidden", RUN14 + "raiseFault('LIDAR_LOST'); hideFaultWin();", [(1, "#faultBadge", "b")], clip="#hdr", pad=0),
    S("fault-resolved", RUN14 + "raiseFault('LIDAR_LOST'); resolveFault();", [
        (1, "#wfBody .hint", "l"), (2, "#wfFoot .btn::いいえ", "bl"), (3, "#wfFoot .btn::はい", "br"),
    ], clip="#winFault", pad=16),
    S("fault-manual-only", RUN14 + "raiseFault('MANUAL_ONLY'); resolveFault();", [(1, "#wfFoot .btn::確認", "r")],
      clip="#winFault", pad=16),
    S("fault-recent", RUN14 + "raiseFault('LIDAR_LOST'); resolveFault(); resumeAfterFault(false);", [
        (1, "#recentFault", "b"), (2, "#map14b", "l"), (3, "#st14", "l"),
    ]),
    S("fault-history", "go('s14'); openWin('winFaultHistory');", [(1, "#winFaultHistory .fh-row.ongoing", "l")],
      clip="#winFaultHistory", pad=16),
    S("guide", "go('s11'); showGuide('対象を見失いました。立ってほしい位置に立ってください');", [
        (1, "#guideTx", "t"), (2, "#winGuide .btn::閉じる", "r"),
    ]),

    # ---------------- 付録 実機では未実装の画面 ----------------
    S("s10", "go('s10')", [(1, "#s10 .radarWrap", "tl"), (2, "#s10 .card.ops .opsgrid", "l"), (3, "#s10 .stick", "tl")]),
    S("s12", "go('s12'); __tab('t12b');", [(1, "#rec12", "r"), (2, "#s12 .card.ops .opsgrid", "l")]),
    S("s15", "go('s15')"),
    S("s16", "go('s16')", [(1, "#leadState", "l")]),
    S("s60-permit", "go('s01'); openWin('winAutoMeasure');", clip="#winAutoMeasure", pad=16),
    S("s60", "go('s60')", [(1, "#zonePill", "bl"), (2, "#s60 .btn::測定機能の画面へ", "l"), (3, "#s60 .btn::手動", "l"),
                           (4, "#s60 .btn::自律測定を終了", "l")]),
]
