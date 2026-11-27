// shell/Layouts.jsx — 画面の3つの型の骨組み（Spec-webui §3.3・§7）。
//
// 「走らせる」（左上の操作バー・主表示・右列①〜④）／
// 「手順を進める」（一覧・手順・下端の左右）／
// 「見る・変える」（区分・中身・保存といつ効くかの印）。
// 横長・縦長（§7の表）の両方。タブの部品は使わない（使わないので描かない）。
// 状態で位置を動かさない（同じ画面の中で状態が変わってもボタンの位置は動かない）。
// data-pos で位置を縛れる形（where/toggle/ops/jog/foot）。使い方は README.md。
export function RunLayout({ opbar, main, where, toggle, ops, jog, id, className = '' }) {
  return (
    <div className={`content g-drv ${className}`.trim()} data-layout="run" id={id}>
      {/* 操作バーは主表示と右列の上に横 1 列（Spec-webui.md §3.3 の型「走らせる」）。 */}
      <div data-pos="opbar">{opbar}</div>
      <div className="g-drv-row">
        <div className="blk mainv" data-pos="main">{main}</div>
        <div className="rcol">
          <div className="rops">
            <div data-pos="where">{where}</div>
            <div data-pos="toggle">{toggle}</div>
            <div data-pos="ops">{ops}</div>
          </div>
          <div className="rjog" data-pos="jog">{jog}</div>
        </div>
      </div>
    </div>
  )
}

export function StepsLayout({ opbar, list, body, foot }) {
  return (
    <div className="content g-list" data-layout="steps">
      <div className="lst" data-pos="list">{list}</div>
      <div className="pane" data-pos="body">
        <div data-pos="opbar">{opbar}</div>
        <div className="pbody">{body}</div>
        <div className="pfoot" data-pos="foot">{foot}</div>
      </div>
    </div>
  )
}

export function ViewLayout({ opbar, list, body, foot }) {
  return (
    <div className="content g-list" data-layout="view">
      <div className="lst" data-pos="list">{list}</div>
      <div className="pane" data-pos="body">
        <div data-pos="opbar">{opbar}</div>
        <div className="pbody">{body}</div>
        <div className="pfoot" data-pos="foot">{foot}</div>
      </div>
    </div>
  )
}
