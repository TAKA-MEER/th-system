// shell/Layouts.jsx — 画面の3つの型の骨組み（Spec-webui §3.3・§7）。
//
// 「走らせる」（左上の操作バー・主表示・右列①〜④）／
// 「手順を進める」（一覧・手順・下端の左右）／
// 「見る・変える」（区分・中身・保存といつ効くかの印）。
// 横長・縦長（§7の表）の両方。タブの部品は使わない（使わないので描かない）。
// 状態で位置を動かさない（同じ画面の中で状態が変わってもボタンの位置は動かない）。
// data-pos で位置を縛れる形（where/toggle/ops/jog/foot）。使い方は README.md。
// D106（2026-11-28）: DOM の並びをモックアップ（drvView・listView）と同じにする。
//   本文 = 操作バー（.opbar。高さ 68・全幅・下に線）＋ .content（padding 12 14）。
//   操作バーは 3 つの型とも .content の外（以前は .content の中・見る型は手順の中で、型ごとに位置が違った）。
// 寸法は theme.css の「3 つの型の骨組み」。画面ごとの CSS で骨組みの寸法を上書きしない。
export function RunLayout({ opbar, main, where, toggle, ops, jog, id, className = '' }) {
  return (
    <div className={`layout ${className}`.trim()} data-layout="run" id={id}>
      <div className="opbar" data-pos="opbar">{opbar}</div>
      <div className="content g-drv">
        <div className="blk mainv" data-pos="main">{main}</div>
        <div className="rcol">
          <div className="rops">
            <div className="where" data-pos="where">{where}</div>
            <div data-pos="toggle">{toggle}</div>
            <div className="rrow" data-pos="ops">{ops}</div>
          </div>
          <div className="rjog" data-pos="jog">{jog}</div>
        </div>
      </div>
    </div>
  )
}

export function StepsLayout({ opbar, list, body, foot, id }) {
  return (
    <div className="layout" data-layout="steps" id={id}>
      {/* 「戻る」は画面左上の操作バー（走らせる型の「終了」と同じ場所） */}
      <div className="opbar" data-pos="opbar">{opbar}</div>
      <div className="content g-list">
        <div className="lst" data-pos="list">{list}</div>
        <div className="pane" data-pos="body">
          <div className="pbody">{body}</div>
          <div className="pfoot" data-pos="foot">{foot}</div>
        </div>
      </div>
    </div>
  )
}

// 見る・変える型もモックアップの listView と同じ形（操作バーは本文の上）。
export function ViewLayout({ opbar, list, body, foot, id }) {
  return (
    <div className="layout" data-layout="view" id={id}>
      <div className="opbar" data-pos="opbar">{opbar}</div>
      <div className="content g-list">
        <div className="lst" data-pos="list">{list}</div>
        <div className="pane" data-pos="body">
          <div className="pbody">{body}</div>
          <div className="pfoot" data-pos="foot">{foot}</div>
        </div>
      </div>
    </div>
  )
}
