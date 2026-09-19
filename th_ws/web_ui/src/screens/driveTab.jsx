// screens/driveTab.jsx — the single drive tab shared by S-10/S-11/S-12/S-13
// (DetailedDesign-webui.md §8.1 / DetailedDesign-wp3.md WP-UI-03 §4.2).
//
// This packet builds the shared skeleton only:
//   - manual keeping a VirtualStick + SpeedPreset (JogConsole);
//   - follow keeping the same stick stack plus an empty radar insertion
//   - point, where WP-UI-04's CandidateRadar will mount.
//
// The S-11 "manual" extras (obstacle warning / auto-brake toggle / rear
// blind-spot note, §8.1) and the S-10/S-12 radar are future packets (the
// "やらないこと" column): this file only guarantees the slot exists.
//
// It reads ros + stale from useSystemState() itself (screens are expected to
// do that, not have the shell thread state through), and gates the stick on
// `stale` -- a display-level courtesy only (the authority is jog_gate).
import { useSystemState } from '../ros/useSystemState.js'
import JogConsole from '../parts/JogConsole.jsx'
import { FOLLOW_RADAR_SLOT } from '../i18n/screens.js'

export default function DriveTab({ kind }) {
  const { ros, stale } = useSystemState()

  return (
    <div className="driveTab">
      {kind === 'follow' && (
        // WP-UI-04's CandidateRadar mounts here. WP-UI-03 deliberately leaves
        // this empty (c3) -- the follow screen body is a later packet.
        <div className="radarWrap" data-testid="drive-radar-slot" aria-label={FOLLOW_RADAR_SLOT} />
      )}
      {/* keyboard はここ（常設走行タブ）で true。brief-onsite-ux2 F-5 で W-6
          （shell/Windows.jsx）側の JogConsole にも keyboard を渡すよう変えた
          （S-20/S-21 の手動操作パネルで WASD が効かないという実機指摘）。
          window の keydown は全 JogConsole に届くので、両方が同時に画面へ
          出ると指令が二重になる（WS-4）。ただし実際には driveTab（この
          コンポーネント。S-11/S-13/S-14）と W-6（S-20/S-21）は同じ画面に
          同時に出ない（S-11/S-13/S-14 の OperationCard は manual スロットを
          false にしており「手動」ボタン＝W-6 を開く導線が無い）ので、現状は
          二重指令にならない。Windows.jsx 側の詳しい理由もあわせて参照。 */}
      <JogConsole ros={ros} disabled={stale} keyboard />
    </div>
  )
}
