// parts/icons.jsx — inline SVG for button-kind icons (brief-onsite-ux UX-2-a).
//
// 識別を「色だけ」にしない: 形（塗りつぶし/枠線）とアイコンで区別し、ラベルは
// テキストで読ませる。CDN も絵文字もフォントも使わない。アイコンは全て
// aria-hidden="true" なので、スクリーンリーダーにはテキストラベルだけが届く。
//
// OP_BUTTON_KINDS は parts/opKinds.js が持つ（node --test から import するため
// JSX を含めない。ここではそのまま再エクスポートする）。
export { OP_BUTTON_KINDS } from './opKinds.js'

function Ic({ children }) {
  return (
    <svg
      className="ic"
      viewBox="0 0 16 16"
      width="1em"
      height="1em"
      aria-hidden="true"
      focusable="false"
    >
      {children}
    </svg>
  )
}

const STROKE = {
  fill: 'none',
  stroke: 'currentColor',
  strokeWidth: 2,
  strokeLinecap: 'round',
  strokeLinejoin: 'round',
}

// 進む・移動: 右向き矢印（塗りつぶし・青）。
export function IconArrow() {
  return (
    <Ic>
      <path d="M3 8h9M8 3l5 5-5 5" {...STROKE} />
    </Ic>
  )
}

// 停止: 四角（塗りつぶし・赤）。形は .btn-stop の丸太枠で他と区別する。
export function IconStop() {
  return (
    <Ic>
      <rect x="4" y="4" width="8" height="8" fill="currentColor" />
    </Ic>
  )
}

// 登録・宣言: 地図ピン（枠線・水色）。
export function IconPin() {
  return (
    <Ic>
      <path
        d="M8 2C5.8 2 4 3.8 4 6c0 2.6 3 5 3.6 5.5h.8C9 11 12 8.6 12 6c0-2.2-1.8-4-4-4Z"
        fill="currentColor"
        fillOpacity="0.28"
        stroke="currentColor"
        strokeWidth="1.3"
      />
      <circle cx="8" cy="6" r="1.3" fill="currentColor" />
    </Ic>
  )
}

// 手動: ジョイスティック（丸＋十字。枠線・灰）。
export function IconJoystick() {
  return (
    <Ic>
      <rect
        x="3"
        y="5"
        width="10"
        height="8"
        rx="3"
        fill="currentColor"
        fillOpacity="0.22"
        stroke="currentColor"
        strokeWidth="1.3"
      />
      <circle cx="8" cy="9" r="2" fill="currentColor" />
      <path d="M8 1.8v2M4.2 9H6M10 9h1.8" {...STROKE} strokeWidth="1.6" />
    </Ic>
  )
}

// 保存: フロッピー（枠線・緑）。
export function IconSave() {
  return (
    <Ic>
      <rect
        x="2.5"
        y="2.5"
        width="11"
        height="11"
        rx="1.5"
        fill="currentColor"
        fillOpacity="0.2"
        stroke="currentColor"
        strokeWidth="1.3"
      />
      <path d="M4.5 3v3h4.5V3" {...STROKE} strokeWidth="1.3" />
      <rect x="5.5" y="9" width="5" height="4.5" fill="currentColor" fillOpacity="0.35" />
    </Ic>
  )
}

// 終了: ✕（枠線・灰）。
export function IconClose() {
  return (
    <Ic>
      <path d="M4 4l8 8M12 4l-8 8" {...STROKE} />
    </Ic>
  )
}