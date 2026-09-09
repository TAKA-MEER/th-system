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
import { useEffect, useState } from 'react'
import { onsiteMapTransform, yawToSvgDeg } from '../mapGeometry.js'
import { occupancyGridToPixels } from '../screens/routePreviewGeom.js'
import { costmapToPixels } from './costmapPixels.js'

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
  const t = onsiteMapTransform(mapData, MAP_VB_W, MAP_VB_H, 24, 22)
  const hasMap = !!(mapUrl && t.view)

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
  return (
    <svg viewBox={`0 0 ${MAP_VB_W} ${MAP_VB_H}`} aria-label={ariaLabel} data-testid={`${testId}-map`}>
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
        return (
          <g
            key={pin.id}
            className={`${testId}-map-pin${sel ? ' sel' : ''}`}
            transform={`translate(${px} ${py})`}
            role="button"
            tabIndex={0}
            data-testid={`${testId}-map-pin-${pin.id}`}
            onClick={() => onSelectPin(pin.id)}
            onKeyDown={(e) => {
              if (e.key === 'Enter' || e.key === ' ') {
                e.preventDefault(); onSelectPin(pin.id)
              }
            }}
          >
            <circle r={10} fill={home ? '#2196f3' : '#e91e63'} stroke={sel ? '#ffd54f' : (home ? '#90caf9' : '#f8bbd0')} strokeWidth={2} />
            <text y={26} textAnchor="middle" fontSize="10" fill={home ? '#90caf9' : '#f8bbd0'}>
              {pin.name ?? pin.id}
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
    </svg>
  )
}
