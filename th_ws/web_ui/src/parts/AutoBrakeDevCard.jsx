// parts/AutoBrakeDevCard.jsx — 自律系の画面（S-14 / S-21）の自動ブレーキ切替（A-15）。
//
// 開発モードで項目 auto_brake を選んでいるときだけ出す（Spec-webui §3.5・Spec-safety §10）。
// 選んでいなければ何も描かない。受理は機体（項目が実効のときだけ自律系でも受ける）。
import { useSystemState } from '../ros/useSystemState.js'
import { useDevMode } from '../ros/useDevMode.js'
import { autoBrakeCardVisible, canToggleWithDev, devAutoBrakeEffective } from '../shell/autoBrakeToggle.js'
import AutoBrakeToggle from './AutoBrakeToggle.jsx'
import {
  AB_DEV_CARD_TITLE, W6_AUTO_BRAKE_LABEL, W6_AUTO_BRAKE_ON, W6_AUTO_BRAKE_OFF, W6_AUTO_BRAKE_UNKNOWN,
} from '../i18n/screens.js'

export default function AutoBrakeDevCard({ prefix }) {
  const { ros, state, stale } = useSystemState()
  const { dev } = useDevMode(ros)
  if (!autoBrakeCardVisible(dev)) return null
  const brake = state?.auto_brake ?? null
  const canToggle = canToggleWithDev(false, state, stale, devAutoBrakeEffective(dev))
  return (
    <div className="card" data-testid={`${prefix}-auto-brake-card`}>
      <h3>{AB_DEV_CARD_TITLE}</h3>
      <div className="row">
        <span className="grow sm">{W6_AUTO_BRAKE_LABEL}</span>
        <span className={`pill ${brake ? 'ok' : ''}`} data-testid={`${prefix}-auto-brake`}>
          {brake === true ? W6_AUTO_BRAKE_ON : brake === false ? W6_AUTO_BRAKE_OFF : W6_AUTO_BRAKE_UNKNOWN}
        </span>
        <AutoBrakeToggle brake={brake} canToggle={canToggle} prefix={prefix} />
      </div>
    </div>
  )
}
