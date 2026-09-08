// parts/opKinds.js — 運転操作ボタンの「形の種別」クラス名の単一マップ
// （brief-onsite-ux UX-2-a）。JSX を含まないので unit テスト
// （node --test）から直接 import できる。icons.jsx がこの値を .btn に付ける。
//
// 識別は色だけにしない: 塗りつぶし/枠線・角（停止は丸太枠）＋アイコン＋ラベルで
// 区別する。クラス名は theme.css の `.btn-*` と同期する
// （unit テストで「停止は他と異なる」を固定する）。
export const OP_BUTTON_KINDS = {
  advance: 'btn-advance',
  stop: 'btn-stop',
  register: 'btn-register',
  manual: 'btn-manual-op',
  save: 'btn-save-op',
  finish: 'btn-finish',
}