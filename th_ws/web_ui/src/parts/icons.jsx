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

// 鍵（自動ブレーキのロック。枠線）。絵文字は使わない（目標 G2）。
export function IconLock() {
  return (
    <Ic>
      <rect x="3.5" y="7.5" width="9" height="6.5" {...STROKE} />
      <path d="M5.5 7.5V5.5a2.5 2.5 0 0 1 5 0v2" {...STROKE} />
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

/* ---- S-01 メインメニューのタイルのアイコン（Spec-webui §3.3.7）。 ----
   アイコンの例外はメインメニューのタイルだけ。新しく足すだけ（既存は変えない）。 */
export function IconField() {
  return (
    <Ic>
      <path d="M3 13V8l5-4 5 4v5" {...STROKE} />
      <path d="M7 13V9.5h2V13" {...STROKE} strokeWidth="1.3" />
    </Ic>
  )
}
export function IconDrive() {
  return (
    <Ic>
      <circle cx="8" cy="8" r="5.5" {...STROKE} />
      <path d="M8 4.5V8l3 2" {...STROKE} />
    </Ic>
  )
}
export function IconMaint() {
  return (
    <Ic>
      <path d="M10.5 2.8a3 3 0 0 0-2.8 3.9L3 11.4a1.8 1.8 0 1 0 2.6 2.6l4.7-4.7a3 3 0 0 0 3.9-2.8l-2.2 2.2-2.1-.5-.5-2.1 2.1-2.2z" fill="currentColor" fillOpacity="0.25" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round" />
    </Ic>
  )
}
export function IconMeasure() {
  return (
    <Ic>
      <rect x="3" y="3" width="10" height="10" rx="2" {...STROKE} />
      <path d="M6 10.5V7M8 10.5V5.5M10 10.5V8.5" {...STROKE} />
    </Ic>
  )
}
export function IconGear() {
  return (
    <Ic>
      <circle cx="8" cy="8" r="2" {...STROKE} />
      <path d="M8 2.5v2M8 11.5v2M2.5 8h2M11.5 8h2M4.2 4.2l1.4 1.4M10.4 10.4l1.4 1.4M11.8 4.2l-1.4 1.4M5.6 10.4l-1.4 1.4" {...STROKE} strokeWidth="1.5" />
    </Ic>
  )
}
export function IconDay() {
  return (
    <Ic>
      <rect x="3" y="4" width="10" height="8" rx="1.5" {...STROKE} />
      <path d="M3 7.5h10" {...STROKE} strokeWidth="1.3" />
      <path d="M5.5 4V2.8M10.5 4V2.8" {...STROKE} strokeWidth="1.3" />
    </Ic>
  )
}
export function IconPrep() {
  return (
    <Ic>
      <path d="M4 13.5V5.5L8 3l4 2.5v8" {...STROKE} strokeWidth="1.5" />
      <path d="M4 13.5h8M6 13.5v-3h4v3" {...STROKE} strokeWidth="1.3" />
    </Ic>
  )
}
export function IconFollow() {
  return (
    <Ic>
      <circle cx="8" cy="5" r="2.2" fill="currentColor" fillOpacity="0.3" stroke="currentColor" strokeWidth="1.3" />
      <path d="M8 7.2v4M8 11.2L5.5 14M8 11.2l2.5 2.8" {...STROKE} />
    </Ic>
  )
}
export function IconManual() {
  return (
    <Ic>
      <circle cx="8" cy="8" r="5" {...STROKE} />
      <circle cx="8" cy="8" r="1.6" fill="currentColor" />
      <path d="M8 3v2M8 11v2M3 8h2M11 8h2" {...STROKE} strokeWidth="1.5" />
    </Ic>
  )
}
export function IconReplay() {
  return (
    <Ic>
      <path d="M4 5.5A4.5 4.5 0 1 1 3.5 9" {...STROKE} />
      <path d="M3 2.5v3h3" {...STROKE} />
    </Ic>
  )
}
export function IconTeach() {
  return (
    <Ic>
      <circle cx="4.5" cy="11.5" r="1.8" fill="currentColor" fillOpacity="0.3" stroke="currentColor" strokeWidth="1.3" />
      <circle cx="11.5" cy="4.5" r="1.8" fill="currentColor" fillOpacity="0.3" stroke="currentColor" strokeWidth="1.3" />
      <path d="M5.8 10.2l4.4-4.4" {...STROKE} strokeDasharray="2 2" />
    </Ic>
  )
}
export function IconCheck() {
  return (
    <Ic>
      <path d="M3 8.5l3 3L13 4" {...STROKE} strokeWidth="2.2" />
    </Ic>
  )
}
export function IconCalib() {
  return (
    <Ic>
      <path d="M8 2.5v3M8 10.5v3M2.5 8h3M10.5 8h3" {...STROKE} strokeWidth="1.5" />
      <circle cx="8" cy="8" r="2" fill="currentColor" fillOpacity="0.3" stroke="currentColor" strokeWidth="1.3" />
    </Ic>
  )
}
export function IconBack() {
  return (
    <Ic>
      <path d="M13 8H4M8 3L3 8l5 5" {...STROKE} strokeWidth="2" />
    </Ic>
  )
}