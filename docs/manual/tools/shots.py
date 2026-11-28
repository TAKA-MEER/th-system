"""図解マニュアルのショット定義。

1 ショット = 1 枚の PNG。capture.py が 1 枚ごとにモックアップを読み直してから撮る。

  id      PNG の名前。index.html の <figure data-shot="..."> と対応する
  setup   状態を作る JS。モックアップの外側の切り替え欄をクリックして作る（capture.py の HELPERS_JS）:
            st('replay','run')                 画面 replay の状態 run
            st('follow','entry',{teach:true})  教示（追従）。S-12/S-13 は teach を立てる
            ab('fault')                        異常: none / fault / estop / carry / resolved
            act1('jog')                        画面の中のボタン（data-act）を押す
          S・render()・go 系のモックアップの関数もそのまま呼べる
  clip    切り抜く要素（省略すると画面全体 1280×720）。pad は余白（論理 px）
  marks   (番号, 要素, 番号丸の位置) のならび。位置は tl/tr/bl/br/t/b/l/r/c（既定 tl）
          要素は CSS セレクタ。"css::文字列" でその文字列を含むものに絞る。" || " で矩形を合併

注釈の文言は index.html の凡例に書く。ここには番号と要素だけを置く。
実機の画面で撮り直すときは、setup と要素の指定をここで差し替える。
"""


def S(id, setup="", marks=(), clip=None, pad=12, **kw):
    return dict(id=id, setup=setup, marks=list(marks), clip=clip, pad=pad, **kw)


HDR = "#dev .hdr || #dev .estop-top"      # ヘッダ全体（非常停止は .hdr の外に描かれている）

SHOTS = [
    # ---------------- 1 画面の見方 ----------------
    S("layout", "st('replay','run')", [
        (1, "#dev .hdr .title", "bl"), (2, "#dev .hdr .mst", "bl"), (3, "#dev .estop-top", "bl"),
        (4, "#dev .opbar .exit", "bl"), (5, "#dev .mainv", "tl"), (6, "#dev .rcol .where", "l"),
        (7, "#dev .rcol .tg", "l"), (8, "#dev .rcol .rrow", "l"), (9, "#dev .rcol .rjog", "l"),
    ]),
    S("hdr-normal", "st('day','moving')", [
        (1, "#dev .hdr .title", "b"), (2, "#dev .hdr .mst", "b"), (3, "#dev .estop-top", "l"),
    ], clip=HDR, pad=0),
    S("hdr-fault", "st('day','moving'); ab('fault')", [(1, "#dev .hdr .al", "b")], clip=HDR, pad=0),
    S("hdr-estop", "st('day','moving'); ab('estop')", [(1, "#dev .hdr .al", "b"), (2, "#dev .hdr .mst", "b")], clip=HDR, pad=0),
    S("hdr-carry", "st('day','moving'); ab('carry')", [(1, "#dev .hdr .al", "b")], clip=HDR, pad=0),
    S("hdr-recent", "st('day','moving'); ab('fault'); ab('resolved'); act1('ack')", [(1, "#dev .hdr .al", "b")], clip=HDR, pad=0),
    S("hdr-brakeoff", "st('manual','idle'); act1('bkUnlock','manual'); act1('bkUnlock','manual'); act1('brakeOff','manual')",
      [(1, "#dev .hdr .brkoff", "b"), (2, "#dev .opbar .brk", "b")], clip="#dev .hdr || #dev .opbar || #dev .estop-top", pad=0),
    S("hdr-dev", "st('settings','dev'); S.settings.dev=true; render()", [(1, "#dev .hdr", "b")], clip=HDR, pad=0),

    # ---------------- 1 つづき: 走らせる型の右列・ボタン ----------------
    S("toggle-states", "st('replay','paused')", [
        (1, "#dev .rcol .tg .go-run", "l"), (2, "#dev .rcol .tg .idle", "l"),
    ], clip="#dev .rcol .rops .tg", pad=14),
    S("toggle-run", "st('replay','run')", [(1, "#dev .rcol .tg .go-stop", "l"), (2, "#dev .rcol .tg .idle", "r")],
      clip="#dev .rcol .rops .tg", pad=14),
    S("dis-reason", "st('day','home'); [...document.querySelectorAll('#dev button.dis')].find(b=>b.textContent.includes('測定機能へ')).dispatchEvent(new MouseEvent('click',{bubbles:true}))",
      [(1, "#dev .why-pop", "t"), (2, "#dev .ctl .b.dis::測定機能へ", "b")], clip="#dev .ctl", pad=0),

    # ---------------- 2 起動と終了 ----------------
    S("s00-ok", "st('connect','ok')", [
        (1, "#dev .diag", "tl"), (2, "#dev .side2 .big", "l"), (3, "#dev .side2 .ac", "l"), (4, "#dev .side2 > div:last-child .b", "tl"),
    ]),
    S("s00-ng", "st('connect','lidar')", [
        (1, "#dev .diag .mk.ng", "r"), (2, "#dev .diag .fix", "t"), (3, "#dev .side2 .big", "l"), (4, "#dev .hdr .mst", "bl"),
    ]),
    S("s00-camera", "st('connect','camera')", [(1, "#dev .diag", "tl"), (2, "#dev .side2 .big", "l")]),
    S("s00-restart", "st('connect','restart')", [(1, "#dev .hdr .mst", "bl")], clip="#dev .hdr", pad=0),
    S("s01", "st('menu','top')", [
        (1, "#dev .tile.l", "tl"), (2, "#dev .tile::走行", "tl"), (3, "#dev .tile::点検と校正", "tl"),
        (4, "#dev .tile::自律測定", "tl"), (5, "#dev .tile::設定", "tl"), (6, "#dev .halt", "l"), (7, "#dev .opbar .exit", "bl"),
    ]),
    S("s01-drive", "st('menu','drive')", [
        (1, "#dev .opbar .exit::戻る", "bl"), (2, "#dev .tile::追従走行", "tl"), (3, "#dev .tile::教示再生", "tl"),
    ]),
    S("s01-field-maint", "st('menu','field')", [(1, "#dev .tile::試験準備", "tl"), (2, "#dev .tile::試験", "tr")]),
    S("s01-auto-permit", "st('menu','autoask')", [(1, "#dev .win .b::許可しない", "bl"), (2, "#dev .win .b::許可する", "br")],
      clip="#dev .win", pad=16),
    S("shutdown-list", "st('menu','shutdown')", [
        (1, "#dev .win .b::保存", "t"), (2, "#dev .win .b::破棄", "tr"), (3, "#dev .win .b::止める", "r"),
    ], clip="#dev .win", pad=16),
    S("shutdown-ready", "st('menu','shutdown'); act1('sdSave','map'); act1('sdSave','route')",
      [(1, "#dev .win .b::止める", "r")], clip="#dev .win", pad=16),
    S("shutdown-done", "st('menu','shutdown'); act1('sdSave','map'); act1('sdSave','route'); act1('sdStop')",
      clip="#dev .win", pad=16, wait=1900),

    # ---------------- 3 試験場の外を走る ----------------
    S("s10-pick", "st('follow','pick')", [
        (1, "#dev .opbar .exit", "bl"), (2, "#dev .radar", "tl"), (3, "#dev .rcol .where", "l"), (4, "#dev .rcol .tg", "l"),
        (5, "#dev .rcol .look", "l"), (6, "#dev .rcol .rjog", "l"),
    ]),
    S("s10-ready", "st('follow','ready')", [(1, "#dev .radar", "tl"), (2, "#dev .rcol .look", "l"), (3, "#dev .rcol .tg .go-run", "l")]),
    S("s10-run", "st('follow','run')", [(1, "#dev .hdr .mst", "bl"), (2, "#dev .rcol .where", "l"), (3, "#dev .rcol .tg .go-stop", "l")]),
    S("s11", "st('manual','drive')", [
        (1, "#dev .opbar .brk, #dev .opbar .lockb", "bl"), (2, "#dev .mainv .telem", "tl"), (3, "#dev .mainv .surr, #dev .mainv svg", "tr"),
        (4, "#dev .rcol .stopone", "l"), (5, "#dev .rcol .rjog", "l"),
    ]),
    S("s11-near", "st('manual','near')", [(1, "#dev .mainv .telem", "tl"), (2, "#dev .mainv svg", "tr")]),
    S("brake-lock", "st('manual','idle')", [(1, "#dev .opbar .lockb", "b")], clip="#dev .opbar", pad=0),
    S("brake-armed", "st('manual','idle'); act1('bkUnlock','manual')", [(1, "#dev .opbar .lockb", "b")], clip="#dev .opbar", pad=0),
    S("brake-unlocked", "st('manual','idle'); act1('bkUnlock','manual'); act1('bkUnlock','manual')", [(1, "#dev .opbar .brk", "b")], clip="#dev .opbar", pad=0),
    S("s12-entry", "st('follow','entry',{teach:true})", [(1, "#dev .card.wide .rt:first-child", "tl"), (2, "#dev .card.wide .rt::録り直す", "bl"), (3, "#dev .radar", "tr")]),
    S("s12-rec", "st('follow','run',{teach:true})", [
        (1, "#dev .hdr .mst", "bl"), (2, "#dev .minimap", "tl"), (3, "#dev .rcol .where", "l"), (4, "#dev .rcol .rrow .save, #dev .rcol .rrow .b::保存", "l"),
    ]),
    S("s13-entry", "st('manual','entry',{teach:true})", [(1, "#dev .card.wide .rt:first-child", "tl"), (2, "#dev .card.wide .rt::録り直す", "bl")]),
    S("s13-rec", "st('manual','drive',{teach:true})", [
        (1, "#dev .hdr .mst", "bl"), (2, "#dev .mainv", "tl"), (3, "#dev .rcol .where", "l"), (4, "#dev .rcol .stopone", "l"), (5, "#dev .rcol .b::保存", "l"),
    ]),
    S("s13-saved", "st('manual','saved',{teach:true})", [(1, "#dev .hdr .mst", "bl"), (2, "#dev .rcol .b::保存", "l")]),
    S("s14-select", "st('replay','select')", [
        (1, "#dev .rtl", "tl"), (2, "#dev .rt::地図の画像なし", "r"), (3, "#dev .card.wide .mseg", "l"), (4, "#dev .b.fwd::この経路で進む", "l"),
    ]),
    S("s14-loading", "st('replay','loading')", [(1, "#dev .mainv", "tl")]),
    S("s14-search", "st('replay','search')", [
        (1, "#dev .rcol .where", "l"), (2, "#dev .rcol .tg", "l"), (3, "#dev .rcol .b::完全グローバルで探す", "l"),
    ]),
    S("s14-notfound", "st('replay','notfound')", [(1, "#dev .rcol .where", "l")], clip="#dev .rcol", pad=8),
    S("s14-far", "st('replay','far')", [(1, "#dev .rcol .where", "l")], clip="#dev .rcol", pad=8),
    S("s14-ready", "st('replay','ready')", [(1, "#dev .rcol .where", "l"), (2, "#dev .rcol .tg .go-run", "l")], clip="#dev .rcol", pad=8),
    S("s14-run", "st('replay','run')", [
        (1, "#dev .mainv", "tl"), (2, "#dev .rcol .where", "l"), (3, "#dev .rcol .tg", "l"), (4, "#dev .rcol .rrow .mseg", "l"), (5, "#dev .rcol .rjog", "l"),
    ]),
    S("s14-paused", "st('replay','paused')", [
        (1, "#dev .rcol .tg .go-run", "l"), (2, "#dev .rcol .b::経路を選び直す", "l"),
    ], clip="#dev .rcol", pad=8),
    S("s14-end", "st('replay','end')", [(1, "#dev .rcol .where", "l")], clip="#dev .rcol", pad=8),
    S("s14-jog", "st('replay','run'); act1('jog')", [
        (1, "#dev .w6", "tl"), (2, "#dev .rcol .rjog", "l"),
    ]),

    # ---------------- 4 試験場 ----------------
    S("s20-entry", "st('prep','entry')", [
        (1, "#dev .card .b.fwd", "tl"), (2, "#dev .card .b.caution", "bl"), (3, "#dev .opbar", "bl"),
    ]),
    S("s20-mapping", "st('prep','mapping')", [
        (1, "#dev .opbar", "bl"), (2, "#dev .mainv", "tl"), (3, "#dev .mapctl", "tl"),
        (4, "#dev .b-ops .where", "l"), (5, "#dev .b-ops .tg", "l"), (6, "#dev .b-ops .save", "l"),
        (7, "#dev .b-jog", "l"),
    ]),
    S("s20-register", "st('prep','mapping')", [
        (1, "#dev .mseg", "tl"), (2, "#dev .b::待機場所を登録", "bl"), (3, "#dev .b::配電盤を登録", "br"),
    ], clip="#dev .mseg || #dev .b::待機場所を登録 || #dev .b::配電盤を登録", pad=16),
    S("s20-reg2", "st('prep','reg')", [(1, "#dev .stp", "tl"), (2, "#dev .b::記録", "tr"), (3, "#dev .b::やめる", "tr")],
      clip="#dev .stp || #dev .b::記録 || #dev .b::やめる", pad=16),
    S("s20-pin-edit", "st('prep','mapping'); act1('pin','A-1')", [(1, "#dev .bub", "tl")], clip="#dev .mainv", pad=0),
    S("s20-order", "st('prep','mapping'); act1('ordStart'); act1('pin','B-1'); act1('pin','A-2')", [(1, "#dev .mainv", "tl")], clip="#dev .mainv", pad=0),
    S("s20-tap", "st('prep','mapping'); act1('method','tap'); S.prep.tap={kind:'PANEL',pt:[330,300],dir:[400,280]}; render()",
      [(1, "#dev .mainv", "tl")], clip="#dev .mainv", pad=0),
    S("s20-wall", "st('prep','mapping'); S.prep.warn={kn:'配電盤',kind:'panel',pos:[330,300],ang:0.4}; render()",
      [(1, "#dev .mainv", "tl")], clip="#dev .mainv", pad=0),
    S("s20-return", "st('prep','return')", [(1, "#dev .hdr .mst", "bl"), (2, "#dev .b-ops .tg .go-stop", "l")]),
    S("s21-unloaded", "st('day','unloaded')", [(1, "#dev .card .b.fwd", "tl"), (2, "#dev .ctl .where", "l")]),
    S("s21-home", "st('day','home')", [
        (1, "#dev .opbar .exit", "bl"), (2, "#dev .opbar .tg", "bl"), (3, "#dev .mainv", "tl"), (4, "#dev .ctl .where", "l"),
        (5, "#dev .ctl .tg", "l"), (6, "#dev .ctl .b::手動", "l"), (7, "#dev .ctl .interrupt", "l"),
        (8, "#dev .ctl .next", "l"), (9, "#dev .ctl .b::待機場所", "l"), (10, "#dev .ctl .b::呼び寄せ", "l"),
        (11, "#dev .ctl .b::測定機能へ", "l"), (12, "#dev .ctl .b::カメラ", "l"),
    ]),
    S("s21-pin", "st('day','home'); act1('pin','A-1')", [(1, "#dev .bub", "tl"), (2, "#dev .bub .adv", "bl")], clip="#dev .mainv", pad=0),
    S("s21-moving", "st('day','moving')", [
        (1, "#dev .hdr .mst", "bl"), (2, "#dev .ctl .where", "l"), (3, "#dev .ctl .interrupt", "l"), (4, "#dev .ctl .tg", "l"),
    ]),
    S("s21-blocked", "st('day','blocked')", [(1, "#dev .win .b::再検索", "bl"), (2, "#dev .win .b::中断", "br"), (3, "#dev .hdr .mst", "bl")]),
    S("s21-summon", "st('day','summon')", [(1, "#dev .stp", "tl"), (2, "#dev .b::記録", "tr"), (3, "#dev .ctl .interrupt", "l")]),
    S("s21-waitclear", "st('day','waitclear')", [(1, "#dev .win .b::中止", "br")]),
    S("s21-panel", "st('day','panel')", [(1, "#dev .ctl .b::測定機能へ", "l"), (2, "#dev .ctl .b::カメラ", "l"), (3, "#dev .ctl .next", "l")]),
    S("s21-working", "st('day','working')", [(1, "#dev .hdr .mst", "bl"), (2, "#dev .win .b::停止を解く", "br")]),

    # ---------------- 5 保守・設定 ----------------
    S("s30-list", "st('check','list')", [
        (1, "#dev .lst .lhd", "tl"), (2, "#dev .lrow.on", "tl"), (3, "#dev .csteps", "tl"), (4, "#dev .pfoot .b::やめる", "tl"), (5, "#dev .pfoot .fwd", "tr"),
    ]),
    S("s30-estop", "st('check','estop')", [(1, "#dev .pane", "tl"), (2, "#dev .pfoot .fwd", "tr")]),
    S("s30-motor", "st('check','motor')", [(1, "#dev .arrows", "tl"), (2, "#dev .mgs", "tr"), (3, "#dev .pfoot .b::やめる", "tl")]),
    S("s30-imu", "st('check','imu')", [(1, "#dev .pbody", "tl")]),
    S("s30-lidar-ng", "st('check','lidarNg')", [(1, "#dev .pbody", "tl"), (2, "#dev .b::校正へ", "tl")]),
    S("s31", "st('check','repair')", [(1, "#dev .lst .lrow", "tl"), (2, "#dev .chk", "tl"), (3, "#dev .pfoot .fwd", "tr"), (4, "#dev .opbar .exit", "bl")]),
    S("s40-list", "st('calib','list')", [
        (1, "#dev .lst", "tl"), (2, "#dev .steps", "tl"), (3, "#dev .b::履歴", "bl"), (4, "#dev .pfoot .fwd", "tr"),
    ]),
    S("s40-s1", "st('calib','s1')", [(1, "#dev .steps", "tl"), (2, "#dev .pfoot .b::やめる", "tl"), (3, "#dev .pfoot .adv", "tr")]),
    S("s40-s2", "st('calib','s2')", [(1, "#dev .pbody", "tl")], clip="#dev .pane", pad=0),
    S("s40-s3", "st('calib','s3')", [(1, "#dev .pbody", "tl"), (2, "#dev .pfoot .fwd", "tr")], clip="#dev .pane", pad=0),
    S("s40-s4", "st('calib','s4')", [(1, "#dev .pbody", "tl")], clip="#dev .pane", pad=0),
    S("s40-imu", "st('calib','imu1')", [(1, "#dev .pbody", "tl"), (2, "#dev .pfoot .fwd", "tr")], clip="#dev .pane", pad=0),
    S("s40-blind", "st('calib','blind')", [(1, "#dev .steps", "tl"), (2, "#dev .pbody", "tl"), (3, "#dev .pfoot .fwd", "tr")], clip="#dev .pane", pad=0),
    S("s40-hist", "st('calib','hist')", [(1, "#dev .b::この値に戻す", "r"), (2, "#dev .pfoot .fwd", "tr")], clip="#dev .pane", pad=0),
    S("s50-run", "st('settings','run')", [(1, "#dev .lst.sec7", "tl"), (2, "#dev .pbody", "tl"), (3, "#dev .pfoot .when", "tl"), (4, "#dev .pfoot .b", "tr")]),
    S("s50-loc", "st('settings','loc')", [(1, "#dev .pfoot .when", "tl")], clip="#dev .pane", pad=0),
    S("s50-notify", "st('settings','notify')", [(1, "#dev .pfoot .when", "tl")], clip="#dev .pane", pad=0),
    S("s50-screen", "st('settings','screen')", [(1, "#dev .pbody", "tl")], clip="#dev .pane", pad=0),
    S("s50-link", "st('settings','link')", [(1, "#dev .pbody", "tl")], clip="#dev .pane", pad=0),
    S("s50-log", "st('settings','log')", [(1, "#dev .pbody", "tl")], clip="#dev .pane", pad=0),
    S("s50-blind", "st('settings','blind')", [(1, "#dev .pbody", "tl")], clip="#dev .pane", pad=0),
    S("s50-dev", "st('settings','dev')", [(1, "#dev .mseg", "tl"), (2, "#dev .chk", "tl")], clip="#dev .pane", pad=0),

    # ---------------- 6 異常・非常停止 ----------------
    S("estop", "st('replay','run'); ab('estop')", [
        (1, "#dev .estop-top", "bl"), (2, "#dev .hdr .al", "b"), (3, "#dev .win.em", "tl"), (4, "#dev .relbar", "tl"),
    ]),
    S("estop-released", "st('replay','run'); ab('estop'); act1('release')", [(1, "#dev .win .b", "r")], clip="#dev .win", pad=16),
    S("carry", "st('replay','run'); ab('carry')", [(1, "#dev .hdr .al", "b"), (2, "#dev .win.em", "tl")]),
    S("fault", "st('replay','run'); ab('fault')", [(1, "#dev .hdr .al", "b"), (2, "#dev .win.ab", "tl")]),
    S("fault-resolved", "st('replay','run'); ab('fault'); ab('resolved')", [(1, "#dev .win", "tl"), (2, "#dev .win .b::確認", "r")], clip="#dev .win", pad=16),
    S("fault-recent", "st('replay','run'); ab('fault'); ab('resolved'); act1('ack')", [(1, "#dev .hdr .al.past", "b"), (2, "#dev .hdr .mst", "bl")]),

    # ---------------- 付録 実機では未実装の画面 ----------------
    S("s60-wait", "st('auto','wait')", [
        (1, "#dev .opbar .exit", "bl"), (2, "#dev .rcol .where", "l"), (3, "#dev .rcol .tg", "l"), (4, "#dev .rcol .b.big", "l"), (5, "#dev .rcol .rjog", "l"),
    ]),
    S("s60-turn", "st('auto','turn')", [(1, "#dev .hdr .mst", "bl")]),
]
