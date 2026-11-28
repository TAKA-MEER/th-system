// tools/mockcompare/regions.mjs — 名前を付けた領域（モックアップと実画面で同じ名前）。
//
// mock: モックアップ（#dev の中）のセレクタ。real: 実画面（#app の中）のセレクタ。
// 実画面は先に [data-mc="<name>"] を探し、無ければ real を使う（画面の段で data-mc を足せば
// ここを直さずに対応が取れる）。
// セレクタの書き方（docs/manual/tools/capture.py の __rect と同じ）:
//   "css"            見えている最初の要素
//   "css::文字列"     その文字列を含む要素のうち、いちばん短いもの
//   "a || b"         矩形を合併する
// style: true（または比べる項目の配列 'font' 'color' 'bg' 'radius' 'border'）の領域は rect に加えて計算後の字の大きさ・太さ・色・背景・角の丸み・枠線の幅も比べる。
// group: 同じ形の要素を文書の順に 1, 2, 3… と番号を振って対にする（右列のボタンなど）。

export const REGIONS = [
  // ヘッダ
  { name: 'header', mock: '.hdr', real: '#hdr' },
  { name: 'header-r1', mock: '.hdr .r1', real: '#hdr .row1' },
  { name: 'header-r2', mock: '.hdr .r2', real: '#hdr .row2', style: ['bg'] },
  { name: 'header-logo', mock: '.hdr .logo', real: '#logo' },
  { name: 'header-brand', mock: '.hdr .brand', real: '#hdr .proj', style: true },
  { name: 'header-title', mock: '.hdr .title', real: '#screenName', style: true },
  { name: 'header-zone', mock: '.hdr .zone', real: '#zonePill', style: true },
  { name: 'header-state', mock: '.hdr .mst', real: '#modePill', style: true },
  { name: 'header-alert', mock: '.hdr .r2 .al', real: '#faultTx, #autoBrakeOff, #recentFault', style: true },
  // 非常停止（ヘッダの外に重ねて描く）
  { name: 'estop-plate', mock: '.estop-plate', real: '#hdr .estop-plate', style: ['bg'] },
  { name: 'estop', mock: '.estop-top', real: '#estopBtn', style: true },
  // 本文の枠
  { name: 'body', mock: '.body', real: '#body' },
  { name: 'opbar', mock: '.opbar', real: '[data-pos="opbar"]' },
  { name: 'opbar-exit', mock: '.opbar .b.exit', real: '[data-pos="opbar"] .btn.exit, .btn.exit', style: true },
  { name: 'opbar-item', group: true, mock: '.opbar > :not(.grow)', real: '[data-pos="opbar"] > :not(.grow)' },
  { name: 'content', mock: '.content', real: '.content' },
  // 走らせる型
  { name: 'main', mock: '.mainv', real: '[data-pos="main"]' },
  { name: 'rcol', mock: '.rcol', real: '.rcol' },
  { name: 'rcol-rops', mock: '.rcol .rops', real: '.rcol .rops' },
  { name: 'rcol-where', mock: '[data-pos="where"]', real: '[data-pos="where"]', style: true },
  { name: 'rcol-toggle', mock: '[data-pos="toggle"]', real: '[data-pos="toggle"]' },
  { name: 'rcol-ops', mock: '[data-pos="ops"]', real: '[data-pos="ops"]' },
  { name: 'rcol-jog', mock: '[data-pos="jog"]', real: '[data-pos="jog"]' },
  { name: 'toggle', mock: '.rcol .tg', real: '.rcol .stoprun', style: true },
  { name: 'rcol-btn', group: true, style: true, mock: '.rcol .b', real: '.rcol .btn' },
  { name: 'telem', mock: '.telem', real: '.telem' },
  // 手順を進める型・見る・変える型
  { name: 'list', mock: '.g-list .lst', real: '[data-pos="list"]' },
  { name: 'list-head', mock: '.g-list .lhd', real: '.g-list .lhd', style: true },
  { name: 'list-row', group: true, style: true, mock: '.g-list .lrow', real: '.g-list .lrow' },
  { name: 'pane', mock: '.g-list .pane', real: '[data-pos="body"]' },
  { name: 'pane-head', mock: '.g-list .phd', real: '.g-list .phd', style: true },
  { name: 'pane-body', mock: '.g-list .pbody', real: '.g-list .pbody' },
  { name: 'foot', mock: '[data-pos="foot"]', real: '[data-pos="foot"]' },
  { name: 'foot-btn', group: true, style: true, mock: '[data-pos="foot"] .b', real: '[data-pos="foot"] .btn' },
  // S-00（接続確認）・S-01（メインメニュー）
  { name: 'conn-diag', mock: '.g-conn .diag', real: '.g-conn .diag' },
  { name: 'conn-side', mock: '.g-conn .side2', real: '.g-conn .side2' },
  { name: 'menu-tiles', mock: '.tiles', real: '.tiles' },
  { name: 'menu-tile', group: true, style: true, mock: '.tile:not(.empty)', real: '.tile' },
  { name: 'menu-side', mock: '.side', real: '.side' },
  // 窓
  { name: 'scrim', mock: '.scrim', real: '#overlay.has-modal .backdrop' },
  { name: 'window', mock: '.win', real: '.win.show', style: true },
  { name: 'window-btn', group: true, style: true, mock: '.win .b', real: '.win.show .btn' },
  { name: 'relbar', mock: '.relbar', real: '#release.show #releaseBtn', style: true },
  { name: 'w6', mock: '.w6', real: '#jogWin' },
]

// 3 つの型の骨組み（F0 で ±2 px に揃える領域）。--frame で絞れる
export const FRAME = new Set([
  'header', 'header-r1', 'header-r2', 'estop', 'estop-plate', 'header-title', 'header-state',
  'body', 'opbar', 'opbar-exit', 'content', 'main', 'rcol', 'list', 'pane', 'foot',
])
