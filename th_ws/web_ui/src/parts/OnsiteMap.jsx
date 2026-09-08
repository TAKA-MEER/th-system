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
import { useEffect, useState } from 'react'
import { onsiteMapTransform, yawToSvgDeg } from '../mapGeometry.js'
import { occupancyGridToPixels } from '../screens/routePreviewGeom.js'

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

  const t = onsiteMapTransform(mapData, MAP_VB_W, MAP_VB_H)
  const hasMap = !!(mapUrl && t.view)

  return (
    <svg viewBox={`0 0 ${MAP_VB_W} ${MAP_VB_H}`} aria-label={ariaLabel} data-testid={testId}>
      <rect width={MAP_VB_W} height={MAP_VB_H} fill="#20242c" />
      {hasMap ? (
        <image
          x={t.view.offX}
          y={t.view.offY}
          width={t.view.drawW}
          height={t.view.drawH}
          href={mapUrl}
          preserveAspectRatio="none"
          data-testid={`${testId}-map-raster`}
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
    </svg>
  )
}
