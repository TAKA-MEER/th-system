// parts/OnsiteMap.jsx — S-20 / S-21 共用の試験場内地図タブ SVG。
//
// 実地図（/onsite/map_view、map フレーム）を 1 枚の SVG に載せる。地図が届いていれば
// 占有格子ラスタを背景に、ピン（map フレーム）とロボット（map フレームに揃えた pose）を
// 同じ onsiteMapTransform() 変換で重ねる。地図が未受信（null）なら中央寄せ
// 1m=24px のフォールバック変換に落とす（固定のモックアップ部屋は描かない）。
//
// 座標が食い違わないよう、ピン・ロボット・地図ラスタは必ず onsiteMapTransform() の
// toPx() を通す（別々に書くとずれる。mapGeometry.js）。
//
// 見え方（Spec-webui.md §6.1・2026-11-27）: 色相を使わず濃淡だけで描く。
// 未探索＝画面の地と同じ色（地図の外側も）／床＝白／壁＝中間の灰／
// 点群＝壁より濃い灰（減速が始まる距離より近い点は注意の黄）／
// コストマップは「入れない所」（Nav2 の値 99・100 のセル）だけ薄い灰／
// 機体・経路・行き先の輪＝進むの色（緑）／BLOCKED の円＝注意の黄（点線の縁）／
// ピン・追跡中の人・タップの印は中立の色、選んでいるものは反転。
// 色の正本は parts/mapColors.js。
// ラスタは canvas putImageData → dataURL を <image> に描くため、S-13/S-14 の
// RoutePreview と同じ occupancyGridToPixels() を使う（見た目・行反転規則を一致させる）。
//
// brief-MAP-COSTMAP: 地図タブには Nav2 のグローバル costmap（/global_costmap/costmap）
// と直近の Nav2 経路（/plan）も重ねて載せる。costmap は静的地図と解像度・原点・
// サイズが一致しないため info の四隅を toPx() で変換してから <image> に
// 載せる（costmapPixels.js）。経路は plannedPath（[{x,y},...]）を
// 進むの色の <polyline> で描き、終点に行き先の輪（進むの色の輪）を描く。
// 両方無ければ何も描かない。
//
// 地図の端の道具（parts/MapTools.jsx）: 「入れない所」（コストマップの入切）・
// 「凡例」・「左へ回す」「右へ回す」（試験準備だけ）。置き場所は
// toolsClassName / toolsStyle で親から渡せる（既定は右下）。
import { useEffect, useRef, useState } from 'react'
import {
  canvasToWorld, onsiteMapTransform, yawToSvgDeg, quatToYaw,
  MAP_ROTATE_STEP_DEG, unrotateViewPoint, panelOrderOf,
} from '../mapGeometry.js'
import { occupancyGridToPixels } from '../screens/routePreviewGeom.js'
import { costmapToPixels } from './costmapPixels.js'
import {
  MAP_UNKNOWN, MAP_GO, MAP_SCAN, MAP_NEAR, MAP_WARN, MAP_PIN, MAP_PIN_EDGE,
  SLOWDOWN_DISTANCE_M, ROBOT_HALF_M,
} from './mapColors.js'
import { readCostmapVisible, writeCostmapVisible } from './costmapVisibility.js'
import MapTools from './MapTools.jsx'
import './mapTools.css'
import { S20_MAP_RESET_VIEW } from '../i18n/screens.js'

// 論理 viewBox は 340x250（mapGeometry の変換はこの座標系が前提。変更禁止）。
// 表示サイズは CSS（.mapWrap svg の width:100% / max-height）が決める
// （brief-onsite-ux UX-2-c: 固定 px で描かない）。
const MAP_VB_W = 340
const MAP_VB_H = 250
const MAP_CX = MAP_VB_W / 2
const MAP_CY = MAP_VB_H / 2

// ピンの当たり判定は 60 論理 px 以上（Spec-webui.md §1.2・§6.1）。
const PIN_HIT_R = 30

// SG-C7 (1b-11): ドラッグ中の読み値。yaw だけでなく x, y (m) も出す
// （Spec-onsite.md §3.7）。保存される yaw はバックエンドが計算し直すので目安。
function dragReadout(startW, curW) {
  const deg = Math.round((Math.atan2(curW[1] - startW[1], curW[0] - startW[0]) * 180) / Math.PI)
  return `x=${startW[0].toFixed(2)} y=${startW[1].toFixed(2)} ${deg}°`
}

// 机体の四角の半辺（SVG ピクセル）。実寸 0.55 m 角を地図の倍率で描く。
function robotHalfPx(t) {
  const mPerPx = t.view ? t.view.mPerPx : 1 / 24
  return ROBOT_HALF_M / mPerPx
}

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
  // §3.10: 地図の向きを変えられるのは試験準備だけ。「左へ回す」「右へ回す」で
  // 15° ずつ。回すのは表示だけで、ピンや経路の座標・機体へ送る値は変えない。
  rotateEnabled = false,
  // §6.1: 経路が塞がれた（BLOCKED）の間だけ、塞いでいる所を注意の黄の円で示す。
  // 位置は機体から届かないので、経路の先の最初の通れない点（コストマップの
  // 「入れない所」のセル上の最初の経路点）で出せる範囲で描く。コストマップが
  // 無いときは経路の終点に出す（いずれも blocked が立つ間だけ）。
  blocked = false,
  // 地図の端の道具の置き場所（既定は右下。親から渡せる）。
  toolsClassName = '',
  toolsStyle = undefined,
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

  // 「入れない所」の入切（全地図で共通。localStorage。読めないときは既定）。
  const [costmapOn, setCostmapOn] = useState(readCostmapVisible)
  const handleCostmapToggle = () => {
    setCostmapOn((v) => writeCostmapVisible(!v))
  }

  // 地図の向き（回転表示）。画面を開き直したら 0° に戻る（保存は未決）。
  const [rotSteps, setRotSteps] = useState(0)
  const rotDeg = rotSteps * MAP_ROTATE_STEP_DEG
  const handleRotateLeft = () => setRotSteps((s) => (s - 1 + 24) % 24)
  const handleRotateRight = () => setRotSteps((s) => (s + 1) % 24)

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

  // 回した表示の座標を地図の座標へ戻す（逆回転してから canvasToWorld）。
  // これで回した状態でタップしても正しい地図座標が送られる。
  function toWorld(vb) {
    const [ux, uy] = unrotateViewPoint(vb[0], vb[1], MAP_CX, MAP_CY, rotDeg)
    return canvasToWorld(ux, uy, mapData.info, t.view)
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
    // ピンを押したときはポインタキャプチャもパンも始めない。キャプチャを取ると
    // click が svg へ再送出され、ピンの onClick（選択）が届かなくなる
    // （Spec-webui §3.10「ピンを押すと吹き出し」、§3.11「ピンを押して行き先」）。
    // tapMode 中はピン選択をしないので通常どおりドラッグのジェスチャーを始める。
    if (!tapMode && e.target.closest?.('[data-testid*="-map-pin-"]')) return
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
  if (costmapUrl && costmapData?.info && costmapOn) {
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

  // §6.1: 経路が塞がれた（BLOCKED）の間だけ、塞いでいる所を注意の黄の円で示す。
  // 塞いでいる場所の機体値が届かないので、経路の先の最初の通れない点
  // （コストマップの 99・100 セル上の最初の経路点）で出す。コストマップが無い
  // 場合は経路の終点に出す。
  let blockedPoint = null
  if (blocked && plannedPath.length > 0 && costmapData?.info) {
    const info = costmapData.info
    const data = costmapData.data
    if ((Array.isArray(data) || ArrayBuffer.isView(data)) && info.resolution > 0
      && info.origin?.position) {
      const yaw = quatToYaw(info.origin.orientation)
      const cos = Math.cos(-yaw), sin = Math.sin(-yaw)
      for (const p of plannedPath) {
        const dx = p.x - info.origin.position.x
        const dy = p.y - info.origin.position.y
        const gx = Math.floor((dx * cos - dy * sin) / info.resolution)
        const gy = Math.floor((dx * sin + dy * cos) / info.resolution)
        if (gx < 0 || gy < 0 || gx >= info.width || gy >= info.height) continue
        const v = data[gy * info.width + gx]
        if (v === 99 || v === 100) { blockedPoint = p; break }
      }
    }
  }
  if (!blockedPoint && blocked && plannedPath.length > 0) {
    blockedPoint = plannedPath[plannedPath.length - 1]
  }

  // brief-onsite-ux2 F-6/F-7: `${testId}-map` は「地図タブに地図 SVG が
  // マウントされているか」を e2e から直接見るための testid（F-6 の表示ゲート、
  // F-7 の重なり判定の両方が使う）。他の子要素（-map-raster / -map-pin-<id> /
  // -map-robot / -map-no-pose）と同じ `${testId}-map-*` 系列に揃える。
  //
  // brief-MAPTAP-FRONTEND: 戻り値は <svg> 単体でなくフラグメントにし、<svg> の
  // 外にツールバー（全体表示ボタン＋縮尺表示＋地図の端の道具）を置く。呼び出し元は
  // .mapWrap div でラップしているのでレイアウトは崩れない。
  //
  // 縮尺表示: t.view.mPerPx（m/px）から 60px のバーが何 m かを出す（ズームしても
  // 正直な値になるようバー幅は固定・ラベル側を変える）。
  const scaleBarM = t.view ? 60 * t.view.mPerPx : null
  const scaleLabel = scaleBarM == null ? null
    : scaleBarM >= 1 ? `${scaleBarM.toFixed(1)}m` : `${(scaleBarM * 100).toFixed(0)}cm`
  const showDestRing = plannedPath.length > 0
  const destPoint = showDestRing ? plannedPath[plannedPath.length - 1] : null
  const robotHalf = robotHalfPx(t)
  return (
    <div className="mapWrapInner">
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
      <rect width={MAP_VB_W} height={MAP_VB_H} fill={MAP_UNKNOWN} />
      {/* 地図の向き（回転表示）のグループ。回すのは表示だけ。 */}
      <g transform={rotDeg ? `rotate(${rotDeg} ${MAP_CX} ${MAP_CY})` : undefined}>
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
        // Nav2 のグローバル costmap の「入れない所」（99・100 のセル）だけを
        // 薄い灰で重ねる（地図ラスタより上・ピンより下のレイヤ）。
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
        // 直近の Nav2 グローバル経路（/plan）。地図ラスタ・costmap より上、
        // ピンより下のレイヤ。進むの色の折れ線で描く。
        <polyline
          points={plannedPath.map((p) => t.toPx(p.x, p.y).join(',')).join(' ')}
          fill="none"
          stroke={MAP_GO}
          strokeWidth={2}
          strokeLinecap="round"
          strokeLinejoin="round"
          data-testid={`${testId}-map-path`}
        />
      ) : null}

      {destPoint ? (
        // 行き先の輪（進むの色の輪）。経路の終点に描く。
        <circle
          cx={t.toPx(destPoint.x, destPoint.y)[0]}
          cy={t.toPx(destPoint.x, destPoint.y)[1]}
          r={8}
          fill="none"
          stroke={MAP_GO}
          strokeWidth={3}
          data-testid={`${testId}-map-dest`}
        />
      ) : null}

      {blockedPoint ? (
        // §6.1: 経路が塞がれた所（注意の黄の円・点線の縁）。BLOCKED の間だけ。
        <circle
          cx={t.toPx(blockedPoint.x, blockedPoint.y)[0]}
          cy={t.toPx(blockedPoint.x, blockedPoint.y)[1]}
          r={14}
          fill={MAP_WARN}
          fillOpacity={0.45}
          stroke="#a88a00"
          strokeWidth={2.5}
          strokeDasharray="5 4"
          data-testid={`${testId}-map-blocked`}
        />
      ) : null}

      {pins.map((pin) => {
        const [px, py] = t.toPx(pin.pose?.position?.x ?? 0, pin.pose?.position?.y ?? 0)
        const sel = pin.id === selectedPinId
        const home = pin.kind === 'HOME'
        // 向きが分かる形（§6.1・D63）。配電盤は順番の番号を印の中に出す（D67）。
        const order = home ? null : panelOrderOf(pins, pin.id)
        const pinYaw = pin.pose?.orientation
          ? quatToYaw(pin.pose.orientation)
          : 0
        const pinDeg = yawToSvgDeg(pinYaw)
        // 選んでいるものは反転（中立の色同士の反転）。
        const fill = sel ? MAP_PIN_EDGE : MAP_PIN
        const edge = sel ? MAP_PIN : MAP_PIN_EDGE
        const textFill = sel ? MAP_PIN : MAP_PIN_EDGE
        // brief-MAPTAP-FRONTEND: tapMode の間はドラッグ＝タップジェスチャーなので
        // ピン選択とは排他にする（tapMode に入る前に見たい範囲へパンしておく運用）。
        // 通常時（tapMode=false）のピンタップ＝選択の挙動は変えない。
        const selectable = !tapMode
        const r = 12
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
            {/* 当たり判定は 60 論理 px 以上（透明の大きな円で確保する） */}
            <circle r={PIN_HIT_R} fill="#ffffff" fillOpacity="0" />
            {/* 向きが分かる形: 待機場所＝向きの方へ尖った家形、配電盤＝四角に
                向きの三角。回転は機体の向きが分かるよう付ける。 */}
            <g transform={`rotate(${pinDeg})`}>
              {home ? (
                <path
                  d={`M${r + 6} 0 L${r - 2} ${-r} H${-r} V${r} H${r - 2} Z`}
                  fill={fill}
                  stroke={edge}
                  strokeWidth={2}
                  strokeLinejoin="round"
                />
              ) : (
                <>
                  <rect
                    x={-r} y={-r} width={2 * r} height={2 * r} rx={3}
                    fill={fill} stroke={edge} strokeWidth={2}
                    data-testid={`${testId}-map-pin-shape-${pin.id}`}
                  />
                  <path
                    d={`M${r + 1} ${-r + 2} L${r + 12} 0 L${r + 1} ${r - 2} Z`}
                    fill={fill} stroke={edge} strokeWidth={2} strokeLinejoin="round"
                  />
                  {order != null ? (
                    <text
                      y={4} textAnchor="middle" fontSize="13" fontWeight="700" fill={textFill}
                      data-testid={`${testId}-map-pin-order-${pin.id}`}
                    >
                      {order}
                    </text>
                  ) : null}
                </>
              )}
            </g>
            <text
              y={26} textAnchor="middle" fontSize="10" fill={edge}
              data-testid={`${testId}-map-pin-label-${pin.id}`}
            >
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
          {/* 位置と向きの三角＋機体の大きさの四角（進むの色。§6.1） */}
          <rect
            x={-robotHalf} y={-robotHalf} width={2 * robotHalf} height={2 * robotHalf}
            fill="none" stroke={MAP_GO} strokeWidth={2}
            data-testid={`${testId}-map-robot-footprint`}
          />
          <path
            d="M0 -9 L-7 7 L7 7 Z"
            fill={MAP_GO} stroke={MAP_PIN_EDGE} strokeWidth={1.5} strokeLinejoin="round"
          />
        </g>
      ) : (
        <text x={MAP_VB_W / 2} y={MAP_VB_H / 2} textAnchor="middle" fill="#9aa4b2" fontSize="11" data-testid={`${testId}-map-no-pose`}>
          {noPoseLabel}
        </text>
      )}

      {personPose ? (
        // MAP-1: 追従対象者。中立の色（黒い丸に白い縁）。is_lost のときは呼び出し側が
        // personPose を null にして消す。選んでいる人は呼び出し側が反転で描く。
        <g
          className={`${testId}-map-person`}
          transform={`translate(${t.toPx(personPose.x, personPose.y)[0]} ${t.toPx(personPose.x, personPose.y)[1]})`}
          data-testid={`${testId}-map-person`}
        >
          <title>{personLabel}</title>
          <circle cy={-3} r={6} fill={MAP_PIN} stroke={MAP_PIN_EDGE} strokeWidth={2} />
          <circle cy={-10} r={2.5} fill={MAP_PIN} stroke={MAP_PIN_EDGE} strokeWidth={1.5} />
          {personLabel ? (
            <text y={11} textAnchor="middle" fontSize="10" fill={MAP_PIN_EDGE}>{personLabel}</text>
          ) : null}
        </g>
      ) : null}

      {Array.isArray(scanPoints) && scanPoints.length > 0 ? (
        // MAP-SCAN: LiDAR の生スキャン（/scan_filtered、base_link 相対）を
        // baseToWorld() で map 座標に変換した点群。壁より濃い灰で描き、
        // 減速が始まる距離（仮値 0.6 m）より近い点は注意の黄（§6.1）。
        // 各点を個別 <circle> にする素朴な実装でよい（360 点程度なら SVG でも軽い）。
        <g data-testid={`${testId}-map-scan`}>
          {scanPoints.map((p, i) => {
            const [px, py] = t.toPx(p.x, p.y)
            const dist = robotPose
              ? Math.hypot(p.x - robotPose.x, p.y - robotPose.y)
              : Infinity
            const fill = dist <= SLOWDOWN_DISTANCE_M ? MAP_NEAR : MAP_SCAN
            return <circle key={i} cx={px} cy={py} r={1.2} fill={fill} />
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
        // 間ずっと表示）。選んでいるもの＝反転（中立の色同士の反転。§6.1）。
        <g
          transform={`translate(${t.toPx(previewPose.x, previewPose.y)[0]} ${t.toPx(previewPose.x, previewPose.y)[1]}) rotate(${yawToSvgDeg(previewPose.yaw)})`}
          data-testid={`${testId}-map-preview`}
        >
          <circle r={9} fill={MAP_PIN_EDGE} stroke={MAP_PIN} strokeWidth={2} strokeDasharray="4 2" />
          <line x1={0} y1={0} x2={16} y2={0} stroke={MAP_PIN} strokeWidth={3} strokeLinecap="round" />
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
            stroke={MAP_PIN_EDGE} strokeWidth={2} strokeDasharray="5 3"
          />
          <circle cx={drag.start[0]} cy={drag.start[1]} r={4} fill={MAP_PIN_EDGE} />
          <text
            x={drag.cur[0] + 8} y={drag.cur[1] - 8}
            fontSize="11" fill={MAP_PIN_EDGE}
            data-testid={`${testId}-map-tapreadout`}
          >
            {dragReadout(toWorld(drag.start), toWorld(drag.cur))}
          </text>
        </g>
      ) : null}
      </g>
    </svg>
    <MapTools
      testId={testId}
      costmapOn={costmapOn}
      onCostmapToggle={handleCostmapToggle}
      onRotateLeft={rotateEnabled ? handleRotateLeft : null}
      onRotateRight={rotateEnabled ? handleRotateRight : null}
      className={toolsClassName}
      style={toolsStyle}
    />
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
    </div>
  )
}
