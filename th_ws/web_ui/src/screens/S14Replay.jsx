// screens/S14Replay.jsx — S-14, 教示再生 (SCREEN_NAMES.S14 in i18n/screens.js;
// P5 / demo-teach-replay). Spec-webui.md §3.7 の簡略版。
//
// 教示済み経路を /route/catalog（RouteList）から一覧し、選択して ui.route_select
// {id, reverse} で再生を開始する。FSM（th_state/config/transitions.yaml、P2〜P4 で配線済み）:
//   REPLAY: ROUTE_SEL --ui.route_select {id, reverse}--> LOCALIZE
//           LOCALIZE --(replay_runner が evt.localize_done)--> READY
//           READY --ui.run--> RUN、RUN --ui.stop--> PAUSE、PAUSE --ui.run--> RUN
//           RUN --(replay_runner が evt.arrived)--> PAUSE、* --ui.finish--> IDLE
//
// WAIVER(demo): W-19 — 逆再生はデモ範囲外（ボタンを disabled にしておく）。replay_runner の初期姿勢推定は
// P4 の特例（W-01）で省略済みなので LOCALIZE はほぼ一瞬。
//
// 操作カードは stop（ui.stop）と run（ui.run、ラベル「再生」）だけ。
// onTrigger を空関数にしない（S-11 の既知バグ。この画面では「再生」ボタンが
// 実際に ui.run を送ることを e2e/s14-replay-button-sends-ui-run.spec.js で検証する）。
//
// W-01 P3: 初期姿勢の確度（/route/status の localize_quality を 2 値＋目安文に写す。
// 写像は screens/s14Localize.js）と「完全グローバルで探す」ボタン
// （REPLAY/LOCALIZE のときだけ出し、押すと ui.localize_global。
// FSM は LOCALIZE でだけ受け付ける: transitions.yaml T-REPLAY-03）。
// failed の「経路を選び直す」は既存の「終了」（ui.finish → IDLE → 選び直し）で
// 足りるためボタンは作らない（LOCALIZE から ui.route_select を受ける行は無い）。
import { useEffect, useRef, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useSetFlag, MAP_UPDATE_FLAG } from '../ros/useSetFlag.js'
import { useRouteCatalog } from '../ros/useRouteCatalog.js'
import { useRouteStatus } from '../ros/useRouteStatus.js'
import { useRoutePreview } from '../ros/useRoutePreview.js'
import { useOdomPose } from '../ros/useOdomPose.js'
import { useRoutePose } from '../ros/useRoutePose.js'
import { useRouteMap } from '../ros/useRouteMap.js'
import { useReplaySpeedPublisher } from '../ros/useReplaySpeedPublisher.js'
import RoutePreview from './RoutePreview.jsx'
import OperationCard from '../shell/OperationCard.jsx'
import ReplaySpeedControl from '../parts/ReplaySpeedControl.jsx'
import { REPLAY_SPEED_RATIOS } from '../parts/replaySpeed.js'
import DriveTab from './driveTab.jsx'
import attributes from '../generated/attributes.json'
import {
  S11_MANUAL_TITLE, S11_REAR_TITLE, S11_REAR_NOTE,
  S14_TAB_REPLAY, S14_SELECT_TITLE, S14_EMPTY, S14_FWD, S14_REV, S14_PROCEED,
  S14_POSE_TITLE, S14_POSE_LOCALIZE, S14_POSE_LOCALIZE_TIMEOUT, S14_POSE_READY, S14_POSE_RUN, S14_POSE_PAUSE, S14_POSE_PAUSE_RESUMABLE,
  S14_GLOBAL_BUTTON,
  S14_LENGTH, S14_POINTS, S14_SPEED_TITLE,
  S14_XTRACK_TITLE, S14_XTRACK_EMPTY, S14_XTRACK_VALUE,
  S14_MAP_UPDATE_TITLE, S14_MAP_UPDATE_ON, S14_MAP_UPDATE_OFF, S14_MAP_UPDATE_NOTE,
  S14_QUALITY_SEARCHING_HINT,
} from '../i18n/screens.js'
import { localizeQualityView, localizeQualityText } from './s14Localize.js'
import { stateLabel } from '../i18n/states.js'
import { OP_LABELS } from '../i18n/states.js'

// LOCALIZE が長引く = 地図の読み直しに失敗（保存地図が無い経路、posegraph
// ファイル欠落、始点から離れすぎ 等）。WS-9S 以降は slam_toolbox の respawn +
// deserialize に長距離地図で ~45s かかりうるので、しきい値は余裕を持たせる。
const LOCALIZE_TIMEOUT_MS = 60000

// WS-9P: PAUSE の文言は「なぜ止まったか」で変える。終端に達したのなら終了を促し、
// フォルト・ジョグ介入・停止ボタンで止まったのなら「再生で続きから」を案内する。
// arrived は replay_runner が /route/status に載せてくる（RouteStatus.arrived）。
// 状態名だけでは区別できない（T-REPLAY-06/09 と C-01/C-03 が同じ PAUSE に落ちる）。
function poseText(stateName, arrived) {
  if (stateName === 'LOCALIZE') return S14_POSE_LOCALIZE
  if (stateName === 'READY') return S14_POSE_READY
  if (stateName === 'RUN') return S14_POSE_RUN
  if (stateName === 'PAUSE') return arrived ? S14_POSE_PAUSE : S14_POSE_PAUSE_RESUMABLE
  return null
}

export default function S14Replay({ onFinish }) {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const setFlag = useSetFlag()
  const routes = useRouteCatalog(ros)
  const routeStatus = useRouteStatus(ros)
  const routePreview = useRoutePreview(ros)
  const odomPose = useOdomPose(ros)
  const routePose = useRoutePose(ros)
  const routeMap = useRouteMap(ros)
  const disabledAll = stale || state?.mode == null

  const [selectedId, setSelectedId] = useState(null)
  const [localizeStuck, setLocalizeStuck] = useState(false)
  // WS-9T: 再生速度の比率。既定は中速。走行中に変えても replay_runner が
  // 次ティックで反映する（useReplaySpeedPublisher → /replay/speed_scale）。
  const [speedRatio, setSpeedRatio] = useState(REPLAY_SPEED_RATIOS.mid)
  useReplaySpeedPublisher(ros, speedRatio)

  const stateName = state?.state ?? null
  const pose = poseText(stateName, routeStatus?.arrived === true)
  const empty = routes.length === 0
  const selected = selectedId != null

  // W-01 P3: 初期姿勢の確度（/route/status の localize_quality。ROS 側 P2 が
  // 足す 3 フィールドの 1 つ。未マージの間は undefined → 確度欄を出さない）。
  // 数値は出さず「高い／低い」の 2 値＋目安文（options §4）。
  const qualityView = localizeQualityView(routeStatus?.localize_quality)
  const qualityText = localizeQualityText(qualityView.level)
  const showQuality = qualityView.visible
    && (stateName === 'LOCALIZE' || stateName === 'READY')
  // searching（探索＋地図の読み直し）は 60 秒を超えうるので、既存の
  // 「長引いている」注記は出さない（P3 ブリーフ §やること 1）。
  // 1b-11 SG-B19: failed（原因別の案内が出ている）ときも出さない。
  // 汎用のタイムアウト文言（「始点マークに置き直す」）は原因と違うことがある。
  const showStuckNote = localizeStuck
    && qualityView.level !== 'searching'
    && qualityView.level !== 'failed'
  // 1b-11 SG-B19: 探索・読み直しの待ち時間（Spec-transit.md §4.2.2）。
  const showSearchingHint = qualityView.level === 'searching'
    && (stateName === 'LOCALIZE' || stateName === 'READY')
  // 1b-11 SG-B14: 地図の更新（Spec-webui.md §3.7）。既定 OFF。
  // ON のときだけ操作カードに「保存」が出る。
  const mapUpdate = !!state?.map_update

  // LOCALIZE が長く続いたら（＝地図の読み直し失敗の可能性）手がかりを出す。
  useEffect(() => {
    if (stateName !== 'LOCALIZE') { setLocalizeStuck(false); return undefined }
    setLocalizeStuck(false)
    const t = setTimeout(() => setLocalizeStuck(true), LOCALIZE_TIMEOUT_MS)
    return () => clearTimeout(t)
  }, [stateName])

  // 終了 → ui.finish → 承認で S-01 へ戻る（S11Manual の handleFinish と同じ）。
  async function handleFinish() {
    try {
      const res = await sendTrigger('ui.finish')
      if (res?.accepted && onFinish) onFinish()
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
  }

  // この経路で進む → ui.route_select {id, reverse:false}。
  // WAIVER(demo): W-19 — 逆再生はデモ範囲外（reverse は常に false）。
  async function handleProceed() {
    if (selectedId == null) return
    await sendTrigger('ui.route_select', { id: selectedId, reverse: false })
  }

  // W-01 P3: 完全グローバルで探す → ui.localize_global。
  // FSM は REPLAY/LOCALIZE でだけ受け付ける（transitions.yaml T-REPLAY-03）。
  // ボタン自体を LOCALIZE のときだけ出す。探索中は押せない。
  async function handleGlobalLocalize() {
    try {
      await sendTrigger('ui.localize_global')
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
  }

  // 1b-11 SG-B14: 地図の更新トグル → /system/set_flag map_update。
  // 受理は機体が決める（T-REPLAY-08 のガードが保存の可否を縛る）。
  async function handleMapUpdateToggle() {
    try {
      await setFlag(MAP_UPDATE_FLAG, !mapUpdate, 's14')
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
  }

  return (
    <div className="screen two-col" id="s14">
      <div>
        <div className="top-actions sticky">
          <button
            type="button"
            className="btn sm"
            data-testid="s14-finish"
            disabled={disabledAll}
            onClick={handleFinish}
          >
            {OP_LABELS.finish}
          </button>
          <div className="tabs grow" style={{ margin: 0, border: 'none' }} role="tablist">
            <button type="button" role="tab" aria-selected className="tab on">
              {S14_TAB_REPLAY}
            </button>
          </div>
        </div>

        <div className="tabpane on">
          <div className="card">
            <h3>{S14_SELECT_TITLE}</h3>
            {empty ? (
              <div className="note" data-testid="s14-empty">{S14_EMPTY}</div>
            ) : (
              // 経路が増えても地図を下へ押し出さないよう一覧だけ枠内スクロール
              <div className="lst-scroll">
                <table className="lst">
                  <tbody>
                    {routes.map((r) => (
                      <tr
                        key={r.id}
                        className={selectedId === r.id ? 'sel' : ''}
                        data-testid="s14-route-row"
                      >
                        <td role="radio" aria-checked={selectedId === r.id} className="sm">
                          <button
                            type="button"
                            data-testid={`s14-select-${r.id}`}
                            onClick={() => setSelectedId(r.id)}
                          >
                            {r.name}
                          </button>
                        </td>
                        <td className="r sm">
                          {S14_LENGTH}: {r.length_m != null ? r.length_m.toFixed(2) : '--'}
                          / {S14_POINTS}: {r.point_count}
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            )}
            <div className="row mt">
              <span className="grow sm">{S14_FWD}</span>
              <span className="pill ok">{S14_FWD}</span>
            </div>
            <div className="row mt">
              <span className="grow sm">{S14_REV}</span>
              <button type="button" className="btn sm" disabled>{S14_REV}</button>
            </div>
            <button
              type="button"
              className="btn primary mt"
              data-testid="s14-proceed"
              disabled={disabledAll || empty || !selected}
              onClick={handleProceed}
            >
              {S14_PROCEED}
            </button>
          </div>

          {/* WS-3: 経路プレビュー（再生中の点列＋現在地。targetIndex は /route/status の
              pure-pursuit 目標点。記録中・未走行は -1） */}
          <RoutePreview
            preview={routePreview}
            pose={routePose ?? odomPose}
            mapData={routeMap}
            targetIndex={routeStatus?.target_index ?? -1}
          />

          <div className="card">
            <h3>{S14_POSE_TITLE}</h3>
            <div className="note" data-testid="s14-pose">
              {pose ?? stateLabel(stateName)}
            </div>
            {showQuality && (
              <div className="note" data-testid="s14-localize-quality">{qualityText}</div>
            )}
            {/* 1b-11 SG-B19: 読み直しの待ち時間（Spec-transit.md §4.2.2）。
                原因を断定しない文言にする（§4.2.3）。 */}
            {showSearchingHint && (
              <div className="note" data-testid="s14-localize-wait">{S14_QUALITY_SEARCHING_HINT}</div>
            )}
            {stateName === 'LOCALIZE' && (
              <button
                type="button"
                className="btn sm mt"
                data-testid="s14-localize-global"
                disabled={disabledAll || qualityView.level === 'searching'}
                onClick={handleGlobalLocalize}
              >
                {S14_GLOBAL_BUTTON}
              </button>
            )}
            {showStuckNote && (
              <div className="note" data-testid="s14-localize-stuck">{S14_POSE_LOCALIZE_TIMEOUT}</div>
            )}
          </div>

          {/* 経路からのずれ（Spec-webui.md §3.7）。走行中・一時停止中に
              /route/status の cross_track_m / cross_track_max_m を出す。
              停止しても最大値は残る。小数 2 桁。 */}
          {(stateName === 'RUN' || stateName === 'PAUSE') && (
            <div className="card">
              <h3>{S14_XTRACK_TITLE}</h3>
              <div className="note" data-testid="s14-xtrack">
                {routeStatus?.cross_track_m != null && routeStatus?.cross_track_max_m != null
                  ? S14_XTRACK_VALUE(routeStatus.cross_track_m, routeStatus.cross_track_max_m)
                  : S14_XTRACK_EMPTY}
              </div>
            </div>
          )}

          {/* 1b-11 SG-B14: 地図の更新（Spec-webui.md §3.7）。既定 OFF。
              ON のときだけ操作カードに「保存」が出る。 */}
          <div className="card">
            <h3>{S14_MAP_UPDATE_TITLE}</h3>
            <div className="row mt">
              <button
                type="button"
                className="btn sm"
                data-testid="s14-map-update-toggle"
                aria-pressed={mapUpdate}
                disabled={disabledAll}
                onClick={handleMapUpdateToggle}
              >
                {mapUpdate ? S14_MAP_UPDATE_ON : S14_MAP_UPDATE_OFF}
              </button>
            </div>
            <div className="note">{S14_MAP_UPDATE_NOTE}</div>
          </div>

          {/* WS-9X: 再生速度は経路準備段階の設定なので左列（右列は操作＋手動介入で
              統一する。モックアップ two-col の右列と同じ並び）。 */}
          <div className="card">
            <h3>{S14_SPEED_TITLE}</h3>
            <ReplaySpeedControl
              value={speedRatio}
              onSelect={setSpeedRatio}
              disabled={disabledAll}
            />
          </div>
        </div>
      </div>

      <div>
        {/* 操作カードは右列先頭（S11 と同じ位置）。run→ui.run（ラベル「再生」）、
            stop→ui.stop。save は地図更新 ON のときだけ（1b-11 SG-B14）。
            onTrigger を空関数にしない。 */}
        <OperationCard
          mode={state?.mode}
          stateName={stateName}
          attributes={attributes}
          slots={{ stop: true, check: false, run: true, save: mapUpdate, manual: false }}
          runLabel={S14_TAB_REPLAY}
          disabled={disabledAll}
          onTrigger={(trigger) => sendTrigger(trigger)}
        />
        <div className="card">
          <h3>{S11_MANUAL_TITLE}</h3>
          {/* 手動介入用（常設。Spec-transit §0.4） */}
          <DriveTab kind="manual" />
          {/* 1b-11 SG-C9 の S-14 分：後方は死角（Spec-safety.md §2.3）。
              S-11 / S-13 と同じ文言。W-6 の分は別ブランチが扱う。 */}
          <h3>{S11_REAR_TITLE}</h3>
          <div className="note">{S11_REAR_NOTE}</div>
        </div>
        <div className="state">{stateLabel(stateName)}</div>
      </div>
    </div>
  )
}
