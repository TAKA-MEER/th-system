// parts/OnsiteMap.jsx — S-20 / S-21 共用の試験場内地図タブ SVG（brief-onsite-fix C）。
//
// 実地図（/route/map_view、map フレーム）を 1 枚の SVG に載せる。地図が届いていれば
// 占有格子ラスタを背景に、ピン（map フレーム）とロボット（map フレームに揃えた pose）を
// 同じ onsiteMapTransform() 変換で重ねる。地図が未受信（null）なら中央寄せ
// 1m=24px のフォールバック変換に落とす（固定のモックアップ部屋は描かない。
// brief-onsite-fix F-4）。
//
// 座標が食い違わないよう、ピン・ロボット・地図ラスタは必ず onsiteMapTransform() の
// toPx() を通す（別々に書くとずれる。mapGeometry.js）。
//
// ラスタは canvas putImageData → dataURL を <image> に描くため、S-13/S-14 の
// RoutePreview と同じ occupancyGridToPixels() を使う（見た目・行反転規則を一致させる）。
//
// brief-MAP-COSTMAP: 地図タブには Nav2 のグローバル costmap（/global_costmap/costmap）
// と直近の Nav2 経路（/plan）も重ねて載せる。レイヤ順は
// 地図ラスタ → costmap → 経路 → ピン → ロボット → 対象者。costmap は静的地図と
// 解像度・原点・サイズが一致しないため info の四隅を toPx() で変換してから <image> に
// 載せる（costmapPixels.js の赤ヒートマップ）。経路は plannedPath（[{x,y},...]）を
// 水色の <polyline> で描く。両方無ければ何も描かない。
import { useEffect, useRef, useState } from 'react'
import { canvasToWorld, onsiteMapTransform, yawToSvgDeg } from '../mapGeometry.js'
import { occupancyGridToPixels } from '../screens/routePreviewGeom.js'
import { costmapToPixels } from './costmapPixels.js'
import { S20_MAP_RESET_VIEW } from '../i18n/screens.js'

// 論理 viewBox は 340x250（mapGeometry の変換はこの座標系が前提。変更禁止）。
// 表示サイズは CSS（.mapWrap svg の width:100% / max-height）が決める
// （brief-onsite-ux UX-2-c: 固定 px で描かない）。
const MAP_VB_W = 340
const MAP_VB_H = 250

export default function OnsiteMap({
  mapData,
  pins,
  robotPose,
  selectedPinId,
  onSelectPin,
  ariaLabel,
  noPoseLabel,
  robotLabel,
  personPose,
  personLabel,
  scanPoints,
  overlayText,
  costmapData = null,
  plannedPath = [],
  testId,
  // brief-MAPTAP-FRONTEND: 地図タップ登録（方式C）。Spec-onsite.md §3.7。
  // tapMode の間だけ地図背景への press→drag→release をタップ登録ジェスチャー
  // として受け付け、release 時に onTapConfirm(tap1, tap2)（map frame [m]）を
  // 1 回だけ呼ぶ。previewPose は親が確定待ちの候補を描かせるための pose。
  tapMode = false,
  onTapConfirm = null,
  previewPose = null,
}) {
  // 占有格子ラスタ → dataURL。useRouteMap 由来の mapData.info/data。
  const [mapUrl, setMapUrl] = useState(null)
  useEffect(() => {
    if (!mapData?.info) { setMapUrl(null); return undefined }
    const { width, height } = mapData.info
    const data = mapData.data
    if (!(Array.isArray(data) || ArrayBuffer.isView(data)) || width <= 0 || height <= 0) {
      setMapUrl(null)
      return undefined
    }
    const off = document.createElement('canvas')
    off.width = width
    off.height = height
    const ctx = off.getContext('2d')
    const img = ctx.createImageData(width, height)
    img.data.set(occupancyGridToPixels(data, width, height))
    ctx.putImageData(img, 0, 0)
    setMapUrl(off.toDataURL())
    return undefined
  }, [mapData])

  // brief-MAP-COSTMAP: costmap（/global_costmap/costmap）は静的地図と解像度・原点・
  // サイズが一致しないため、info から実世界の四隅を求めて t.toPx() で SVG ピクセルへ
  // 変換してから <image> に載せる（単純に t.view の矩形に重ねると位置がずれる。
  // costmap の origin に回転が付くことは実運用上まず無いので矩形マッピングでよい）。
  // ラスタ化（canvas putImageData → dataURL）は地図ラスタと同じ手法。
  const [costmapUrl, setCostmapUrl] = useState(null)
  useEffect(() => {
    if (!costmapData?.info) { setCostmapUrl(null); return undefined }
    const { width, height } = costmapData.info
    const data = costmapData.data
    if (!(Array.isArray(data) || ArrayBuffer.isView(data)) || width <= 0 || height <= 0) {
      setCostmapUrl(null)
      return undefined
    }
    const off = document.createElement('canvas')
    off.width = width
    off.height = height
    const ctx = off.getContext('2d')
    const img = ctx.createImageData(width, height)
    img.data.set(costmapToPixels(data, width, height))
    ctx.putImageData(img, 0, 0)
    setCostmapUrl(off.toDataURL())
    return undefined
  }, [costmapData])

  // UX-7: 縁のピンのラベル（丸の下 26px）が SVG の外で切れないよう内側に寄せる。
  // brief-MAPTAP-FRONTEND: 内部 zoom/pan state を第6引数に配線する
  // （mapGeometry.js の変換は 340x250 座標系の内部で描画範囲を動かすだけ。
  // MAP_VB_W/MAP_VB_H 定数自体は変えない）。
  const [zoom, setZoom] = useState(1)
  const [pan, setPan] = useState({ x: 0, y: 0 })
  // ジェスチャー中のドラッグ表示（viewBox 座標）。release で消える。
  const [drag, setDrag] = useState(null)
  const svgRef = useRef(null)
  const panRef = useRef(null)
  // 実機確認（2026-09-15）: 同時に押されているポインタ（タッチ）を pointerId ごとに
  // 保持し、2本になったらピンチズームへ切り替える。1本の間はタップジェスチャー／パン。
  const pointersRef = useRef(new Map())
  const pinchRef = useRef(null) // { dist, zoom }（2本目が触れた瞬間の基準値）
  const t = onsiteMapTransform(mapData, MAP_VB_W, MAP_VB_H, 24, 22,
    { zoom, panX: pan.x, panY: pan.y })
  const hasMap = !!(mapUrl && t.view)

  // tapMode を抜けたらジェスチャー途中の表示を捨てる。
  useEffect(() => {
    if (!tapMode) setDrag(null)
  }, [tapMode])

  // ブラウザの clientX/clientY → SVG viewBox 座標。
  //
  // 実機確認（2026-09-15）: 「地図タップの位置が実際にタップした場所とずれる」
  // 不具合の原因。旧実装は getScreenCTM() の逆変換を使っていたが、このアプリは
  // shell/FixedStage.jsx が #app 全体を transform: scale(var(--stage-scale)) で
  // 画面に合わせて拡大する作りで（タブレットでは --stage-scale がほぼ確実に1以外）、
  // getScreenCTM() がこの手の祖先要素の CSS transform を正しく合成するかはブラウザ
  // 実装に依存する（デスクトップ Chrome では偶然正しく見えていた）。
  // getBoundingClientRect() は祖先の transform を含めた最終的な画面上の矩形を
  // 返すので、そこから viewBox の既定レターボックス（xMidYMid meet）を手計算する
  // 方が実装依存が無く確実。
  function clientToViewBox(clientX, clientY) {
    const svg = svgRef.current
    if (!svg) return null
    const rect = svg.getBoundingClientRect()
    if (rect.width <= 0 || rect.height <= 0) return null
    const scale = Math.min(rect.width / MAP_VB_W, rect.height / MAP_VB_H)
    const drawW = MAP_VB_W * scale, drawH = MAP_VB_H * scale
    const offX = rect.left + (rect.width - drawW) / 2
    const offY = rect.top + (rect.height - drawH) / 2
    return [(clientX - offX) / scale, (clientY - offY) / scale]
  }

  function toViewBox(e) {
    return clientToViewBox(e.clientX, e.clientY)
  }

  function toWorld(vb) {
    return canvasToWorld(vb[0], vb[1], mapData.info, t.view)
  }

  function handleWheel(e) {
    if (!t.view) return
    const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15
    setZoom((z) => Math.min(8, Math.max(1, z * factor)))
  }

  function pointerDist(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y)
  }

  function handlePointerDown(e) {
    if (!t.view) return
    try { e.currentTarget.setPointerCapture(e.pointerId) } catch { /* noop */ }
    pointersRef.current.set(e.pointerId, { x: e.clientX, y: e.clientY })
    // 実機確認（2026-09-15）: タブレットでピンチズームができない不具合の対策。
    // 従来 onWheel（マウスホイール）しか無く、タッチのピンチ操作を一切見ていなかった。
    if (pointersRef.current.size === 2) {
      // 2本目が触れた: 進行中のタップジェスチャー／パンは中断してピンチへ切り替える。
      setDrag(null)
      panRef.current = null
      const [a, b] = [...pointersRef.current.values()]
      pinchRef.current = { dist: pointerDist(a, b), zoom }
      return
    }
    if (pointersRef.current.size > 2) return
    const pos = toViewBox(e)
    if (!pos) return
    if (tapMode) {
      setDrag({ start: pos, cur: pos })
    } else {
      panRef.current = { last: pos }
    }
  }

  function handlePointerMove(e) {
    if (!t.view) return
    if (pointersRef.current.has(e.pointerId)) {
      pointersRef.current.set(e.pointerId, { x: e.clientX, y: e.clientY })
    }
    if (pointersRef.current.size === 2 && pinchRef.current) {
      const [a, b] = [...pointersRef.current.values()]
      if (pinchRef.current.dist > 0) {
        const factor = pointerDist(a, b) / pinchRef.current.dist
        setZoom(Math.min(8, Math.max(1, pinchRef.current.zoom * factor)))
      }
      return
    }
    const pos = toViewBox(e)
    if (!pos) return
    if (tapMode) {
      if (drag) setDrag({ start: drag.start, cur: pos })
    } else if (panRef.current) {
      const [lx, ly] = panRef.current.last
      const dx = pos[0] - lx, dy = pos[1] - ly
      panRef.current = { last: pos }
      setPan((p) => ({ x: p.x + dx, y: p.y + dy }))
    }
  }

  function handlePointerUp(e) {
    pointersRef.current.delete(e.pointerId)
    if (pointersRef.current.size < 2) pinchRef.current = null
    panRef.current = null
    if (!tapMode || !drag || !t.view) {
      if (!tapMode) setDrag(null)
      return
    }
    const pos = toViewBox(e) ?? drag.cur
    const [x1, y1] = toWorld(drag.start)
    const [x2, y2] = toWorld(pos)
    setDrag(null)
    if (onTapConfirm) onTapConfirm({ x: x1, y: y1 }, { x: x2, y: y2 })
  }

  function handlePointerCancel(e) {
    pointersRef.current.delete(e.pointerId)
    if (pointersRef.current.size < 2) pinchRef.current = null
    panRef.current = null
    setDrag(null)
  }

  function handleResetView() {
    setZoom(1)
    setPan({ x: 0, y: 0 })
  }

  // brief-MAP-COSTMAP: costmap の world 矩形（origin と origin+width*res /
  // +height*res）を toPx() で対角 2 点に落とし、SVG は y が下向き（worldToCanvas が
  // 上下反転済み）なので min/abs で <image> の矩形にする。
  let costmapRect = null
  if (costmapUrl && costmapData?.info) {
    const info = costmapData.info
    const [x0, y0] = t.toPx(info.origin.position.x, info.origin.position.y)
    const [x1, y1] = t.toPx(
      info.origin.position.x + info.width * info.resolution,
      info.origin.position.y + info.height * info.resolution,
    )
    costmapRect = {
      x: Math.min(x0, x1),
      y: Math.min(y0, y1),
      width: Math.abs(x1 - x0),
      height: Math.abs(y1 - y0),
    }
  }

  // brief-onsite-ux2 F-6/F-7: `${testId}-map` は「地図タブに地図 SVG が
  // マウントされているか」を e2e から直接見るための testid（F-6 の表示ゲート、
  // F-7 の重なり判定の両方が使う）。他の子要素（-map-raster / -map-pin-<id> /
  // -map-robot / -map-no-pose）と同じ `${testId}-map-*` 系列に揃える。
  //
  // brief-MAPTAP-FRONTEND: 戻り値は <svg> 単体でなくフラグメントにし、<svg> の
  // 外にツールバー（全体表示ボタン＋縮尺表示）を置く。呼び出し元は .mapWrap div
  // でラップしているのでレイアウトは崩れない。
  //
  // 縮尺表示: t.view.mPerPx（m/px）から 60px のバーが何 m かを出す（ズームしても
  // 正直な値になるようバー幅は固定・ラベル側を変える）。
  const scaleBarM = t.view ? 60 * t.view.mPerPx : null
  const scaleLabel = scaleBarM == null ? null
    : scaleBarM >= 1 ? `${scaleBarM.toFixed(1)}m` : `${(scaleBarM * 100).toFixed(0)}cm`
  return (
    <>
    <svg
      ref={svgRef}
      viewBox={`0 0 ${MAP_VB_W} ${MAP_VB_H}`}
      aria-label={ariaLabel}
      data-testid={`${testId}-map`}
      // 実機確認（2026-09-15）: パン・ピンチズームは JS 側（pointer イベント）で
      // 自前実装しているので、ブラウザの既定タッチ処理は常に止める
      // （tapMode=false でも pan-y を許すと、ブラウザ既定のスクロールと自前パンが
      // 競合しうる）。
      style={{ cursor: tapMode ? 'crosshair' : 'grab', touchAction: 'none' }}
      onWheel={handleWheel}
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={handlePointerUp}
      onPointerCancel={handlePointerCancel}
    >
      <rect width={MAP_VB_W} height={MAP_VB_H} fill="#20242c" />
      {hasMap ? (
        <image
          x={t.view.offX}
          y={t.view.offY}
          width={t.view.drawW}
          height={t.view.drawH}
          href={mapUrl}
          data-testid={`${testId}-map-raster`}
        />
      ) : null}

      {costmapRect ? (
        // brief-MAP-COSTMAP: Nav2 のグローバル costmap を赤いヒートマップで重ねる
        // （地図ラスタより上・ピンより下のレイヤ）。“高いほど濃い赤”が障害物の濃さ。
        <image
          x={costmapRect.x}
          y={costmapRect.y}
          width={costmapRect.width}
          height={costmapRect.height}
          href={costmapUrl}
          data-testid={`${testId}-map-costmap`}
        />
      ) : null}

      {plannedPath.length > 0 ? (
        // brief-MAP-COSTMAP: 直近の Nav2 グローバル経路（/plan）。地図ラスタ・
        // costmap より上、ピンより下のレイヤ。目立つ水色の折れ線で描く。
        <polyline
          points={plannedPath.map((p) => t.toPx(p.x, p.y).join(',')).join(' ')}
          fill="none"
          stroke="#4fc3f7"
          strokeWidth={2}
          strokeLinecap="round"
          strokeLinejoin="round"
          data-testid={`${testId}-map-path`}
        />
      ) : null}

      {pins.map((pin) => {
        const [px, py] = t.toPx(pin.pose?.position?.x ?? 0, pin.pose?.position?.y ?? 0)
        const sel = pin.id === selectedPinId
        const home = pin.kind === 'HOME'
        // brief-MAPTAP-FRONTEND: tapMode の間はドラッグ＝タップジェスチャーなので
        // ピン選択とは排他にする（tapMode に入る前に見たい範囲へパンしておく運用）。
        // 通常時（tapMode=false）のピンタップ＝選択の挙動は変えない。
        const selectable = !tapMode
        return (
          <g
            key={pin.id}
            className={`${testId}-map-pin${sel ? ' sel' : ''}`}
            transform={`translate(${px} ${py})`}
            role="button"
            tabIndex={selectable ? 0 : -1}
            data-testid={`${testId}-map-pin-${pin.id}`}
            onClick={selectable ? () => onSelectPin(pin.id) : undefined}
            onKeyDown={(e) => {
              if (!selectable) return
              if (e.key === 'Enter' || e.key === ' ') {
                e.preventDefault(); onSelectPin(pin.id)
              }
            }}
          >
            {/* 実機フィードバック（2026-09-14）: 配電盤ピンがコストマップの赤系
                ヒートマップ（costmapPixels.js）と同系色で見分けにくかったため、
                配電盤ピンは黄色系に変更した（コストマップは赤〜橙止まりで
                緑成分が最大180、黄色は235なので明確に区別できる）。選択時の
                縁取りは色相に依らず白にして、青(待機場所)・黄(配電盤)どちらの
                塗りとも常にコントラストが付くようにする。 */}
            <circle r={10} fill={home ? '#2196f3' : '#ffeb3b'} stroke={sel ? '#ffffff' : (home ? '#90caf9' : '#f57f17')} strokeWidth={2} />
            <text y={26} textAnchor="middle" fontSize="10" fill={home ? '#90caf9' : '#fff59d'}>
              {pin.name || pin.id}
            </text>
          </g>
        )
      })}

      {robotPose ? (
        <g
          className={`${testId}-map-robot`}
          transform={`translate(${t.toPx(robotPose.x, robotPose.y)[0]} ${t.toPx(robotPose.x, robotPose.y)[1]}) rotate(${yawToSvgDeg(robotPose.yaw)})`}
          data-testid={`${testId}-map-robot`}
        >
          <title>{robotLabel}</title>
          <circle r={9} fill="#43a047" stroke="#c8e6c9" strokeWidth={2} />
          <line x1={0} y1={0} x2={14} y2={0} stroke="#c8e6c9" strokeWidth={3} strokeLinecap="round" />
        </g>
      ) : (
        <text x={MAP_VB_W / 2} y={MAP_VB_H / 2} textAnchor="middle" fill="#9aa4b2" fontSize="11" data-testid={`${testId}-map-no-pose`}>
          {noPoseLabel}
        </text>
      )}

      {personPose ? (
        // MAP-1: 追従対象者。ロボットの緑丸と区別できるようオレンジの丸＋頭の丸（人型）と
        // ラベルで描く。is_lost のときは呼び出し側が personPose を null にして消す。
        <g
          className={`${testId}-map-person`}
          transform={`translate(${t.toPx(personPose.x, personPose.y)[0]} ${t.toPx(personPose.x, personPose.y)[1]})`}
          data-testid={`${testId}-map-person`}
        >
          {personLabel ? <title>{personLabel}</title> : null}
          <circle cy={-3} r={6} fill="#fb8c00" stroke="#ffe0b2" strokeWidth={2} />
          <circle cy={-10} r={2.5} fill="#fb8c00" stroke="#ffe0b2" strokeWidth={1.5} />
          {personLabel ? (
            <text y={11} textAnchor="middle" fontSize="10" fill="#ffe0b2">{personLabel}</text>
          ) : null}
        </g>
      ) : null}

      {Array.isArray(scanPoints) && scanPoints.length > 0 ? (
        // MAP-SCAN: LiDAR の生スキャン（/scan_filtered、base_link 相対）を
        // baseToWorld() で map 座標に変換した点群。costmap（別ブリーフ、赤系）や
        // 緑のロボット・オレンジの対象者と重ならない明るい水色の小円で描く。
        // 各点を個別 <circle> にする素朴な実装でよい（360 点程度なら SVG でも軽い）。
        <g data-testid={`${testId}-map-scan`}>
          {scanPoints.map((p, i) => {
            const [px, py] = t.toPx(p.x, p.y)
            return <circle key={i} cx={px} cy={py} r={1.2} fill="#00e5ff" />
          })}
        </g>
      ) : null}

      {overlayText ? (
        // WS-9Y: slam_toolbox の再起動待ち（discard_map）／読み直し待ち
        // （reload。最大 45s+30s）の間、地図が無反応に見えるのを防ぐ。
        <g data-testid={`${testId}-map-overlay`}>
          <rect width={MAP_VB_W} height={MAP_VB_H} fill="#000" opacity={0.6} />
          <text x={MAP_VB_W / 2} y={MAP_VB_H / 2} textAnchor="middle" fill="#fff" fontSize="13">
            {overlayText}
          </text>
        </g>
      ) : null}

      {previewPose ? (
        // brief-MAPTAP-FRONTEND: 確定待ちの候補（release 後も確定/キャンセル待ちの
        // 間ずっと表示）。緑=ロボット・橙=対象者・青/赤=ピンと衝突しない紫系。
        <g
          transform={`translate(${t.toPx(previewPose.x, previewPose.y)[0]} ${t.toPx(previewPose.x, previewPose.y)[1]}) rotate(${yawToSvgDeg(previewPose.yaw)})`}
          data-testid={`${testId}-map-preview`}
        >
          <circle r={9} fill="#8e24aa" stroke="#e1bee7" strokeWidth={2} strokeDasharray="4 2" />
          <line x1={0} y1={0} x2={16} y2={0} stroke="#e1bee7" strokeWidth={3} strokeLinecap="round" />
        </g>
      ) : null}

      {drag ? (
        // brief-MAPTAP-FRONTEND: ジェスチャー中の自前プレビュー（表示用）。
        // 保存される yaw はバックエンドが two_point_core.two_point_yaw() で
        // 計算し直すので、ここの角度は目安でよい。
        <g data-testid={`${testId}-map-tapline`} pointerEvents="none">
          <line
            x1={drag.start[0]} y1={drag.start[1]}
            x2={drag.cur[0]} y2={drag.cur[1]}
            stroke="#ce93d8" strokeWidth={2} strokeDasharray="5 3"
          />
          <circle cx={drag.start[0]} cy={drag.start[1]} r={4} fill="#ce93d8" />
          <text
            x={drag.cur[0] + 8} y={drag.cur[1] - 8}
            fontSize="11" fill="#e1bee7"
          >
            {`${Math.round((Math.atan2(
              toWorld(drag.cur)[1] - toWorld(drag.start)[1],
              toWorld(drag.cur)[0] - toWorld(drag.start)[0]) * 180) / Math.PI)}°`}
          </text>
        </g>
      ) : null}
    </svg>
    {t.view ? (
      <div className="row mt" style={{ alignItems: 'center' }}>
        <button
          type="button"
          className="btn sm"
          data-testid={`${testId}-map-reset-view`}
          onClick={handleResetView}
        >
          {S20_MAP_RESET_VIEW}
        </button>
        <span className="sm mut" data-testid={`${testId}-map-scale`}>
          <span
            style={{
              display: 'inline-block', width: 60, height: 0,
              borderTop: '2px solid #9aa4b2', verticalAlign: 'middle', marginRight: 6,
            }}
          />
          {scaleLabel}
        </span>
      </div>
    ) : null}
    </>
  )
}
