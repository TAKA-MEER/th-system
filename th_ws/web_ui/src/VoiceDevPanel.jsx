// ============================================================
// VoiceDevPanel.jsx — 音声アナウンスの開発用テストパネル (?dev=1)
//
// 実機・rosbridge なしでキュー規則を検証するためのもの。
// アナウンス ID を直接発火するだけで、ROS 状態のモックはしない。
//
// 構造は SettingsPanel.jsx に合わせてある (常時マウント + open で早期 return、
// オーバーレイのクリックで閉じる、配列を map で描画)。
// ============================================================

import React from 'react'

import { ANNOUNCEMENTS_BY_LAYER, AUTO, LAYER } from './voice/announcements.js'

const AUTO_BADGE = {
  [AUTO.WIRED]:     { cls: '',           label: '自動' },
  [AUTO.HEURISTIC]: { cls: 'heuristic',  label: '推測' },
  [AUTO.TIER2]:     { cls: 'unwired',    label: '未接続' },
}

function Row({ entry, voice, held }) {
  const badge = AUTO_BADGE[entry.auto]
  return (
    <div className="voicedev-row">
      <button
        className={`voicedev-btn ${badge.cls}`}
        onClick={() => voice.announce(entry.id)}
        title={entry.note ?? entry.trigger}
      >
        <span className="voicedev-id">{entry.id}</span>
        <span className="voicedev-text">{entry.text}</span>
        <span className="voicedev-badge">{badge.label}</span>
      </button>
      {entry.repeatSec && (
        <button
          className={`voicedev-hold ${held ? 'active' : ''}`}
          onClick={() => voice.setCondition(entry.id, !held)}
          title={`継続状態として ${entry.repeatSec} 秒毎に再発話 (上限 ${entry.repeatMax} 回)`}
        >
          {held ? '解除' : `hold ${entry.repeatSec}s`}
        </button>
      )}
    </div>
  )
}

export default function VoiceDevPanel({ open, onClose, voice }) {
  if (!open) return null

  const { snapshot } = voice
  const held = new Set(snapshot.conditionIds)

  const fireLayer = (layer) => {
    for (const e of ANNOUNCEMENTS_BY_LAYER[layer]) voice.announce(e.id)
  }

  const sections = [
    { layer: LAYER.SAFETY, title: '安全通知レイヤ', on: voice.safetyOn },
    { layer: LAYER.DEMO,   title: 'デモ実況レイヤ', on: voice.demoOn },
  ]

  return (
    <div className="voicedev-overlay" onClick={onClose}>
      <div className="voicedev-panel" onClick={(e) => e.stopPropagation()}>
        <div className="voicedev-header">
          <h2>音声テスト (開発用)</h2>
          <button className="settings-close" onClick={onClose}>✕</button>
        </div>

        {/* 状態表示: デバッグ価値の大半はここにある */}
        <div className="voicedev-status">
          <div><b>再生中</b> {snapshot.playingId ?? '—'}</div>
          <div><b>キュー</b> {snapshot.queueIds.length ? snapshot.queueIds.join(' → ') : '—'}</div>
          <div><b>継続中</b> {snapshot.conditionIds.length ? snapshot.conditionIds.join(', ') : '—'}</div>
          <div>
            <b>レイヤ</b> 安全={String(snapshot.layers[LAYER.SAFETY])} デモ={String(snapshot.layers[LAYER.DEMO])}
          </div>
          <div><b>AudioContext</b> {voice.audioState}</div>
          <div>
            <b>直近の破棄</b>{' '}
            {snapshot.lastDropped ? `${snapshot.lastDropped.id} (${snapshot.lastDropped.reason})` : '—'}
          </div>
        </div>

        <div className="voicedev-bulk">
          <button className="voicedev-bulk-btn" onClick={() => fireLayer(LAYER.SAFETY)}>
            安全通知を一括発火
          </button>
          <button className="voicedev-bulk-btn" onClick={() => fireLayer(LAYER.DEMO)}>
            デモ実況を一括発火
          </button>
          <button
            className="voicedev-bulk-btn stop"
            onClick={() => voice.stopAll({ clearConditions: true })}
          >
            停止
          </button>
        </div>

        {sections.map(({ layer, title, on }) => (
          <section className="voicedev-section" key={layer}>
            <h3>
              {title}
              <span className="voicedev-count">{ANNOUNCEMENTS_BY_LAYER[layer].length} 件</span>
              {!on && <span className="voicedev-off">このレイヤは OFF — 発火しても破棄されます</span>}
            </h3>
            {ANNOUNCEMENTS_BY_LAYER[layer].map((e) => (
              <Row key={e.id} entry={e} voice={voice} held={held.has(e.id)} />
            ))}
          </section>
        ))}
      </div>
    </div>
  )
}
