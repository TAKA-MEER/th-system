// ============================================================
// AudienceView.jsx — 観客向けデモ表示 (VISION.md §6.3)
//
// 操作 UI とは別のツリーとしてマウントされる (main.jsx で分岐)。
// ここから ROS2 へ publish は一切しない (readOnly) し、音も鳴らさない
// (captionSink 経由の字幕のみ)。理由は VISION.md §6.3 / §7.2 参照。
// ============================================================
import { useState, useEffect, useCallback, useRef } from 'react'

import { useRosbridge } from '../hooks/useRosbridge'
import { useVoiceTriggers } from '../hooks/useVoiceTriggers'
import { useCaptionSink } from './captionSink.js'
import WorldCanvas, { COLOR } from './WorldCanvas.jsx'
import JudgementPanel from './JudgementPanel.jsx'
import './AudienceView.css'

// /map は 1 通で数百 KB になり得る。観客表示は秒単位の更新で十分
const MAP_THROTTLE_MS = 2000

// 表示レイヤ。キー 1〜6 の並び順もこれに従う
const LAYERS = [
  { id: 'map',        label: '地図',     color: '#5a6472', key: '1' },
  { id: 'scan',       label: '点群',     color: COLOR.scan,      key: '2' },
  { id: 'candidates', label: '検出候補', color: COLOR.candidate, key: '3' },
  { id: 'target',     label: '追跡対象', color: COLOR.target,    key: '4' },
  { id: 'path',       label: '経路',     color: COLOR.path,      key: '5' },
  { id: 'trail',      label: '走行軌跡', color: COLOR.trail,     key: '6' },
]

const LAYER_STORAGE_KEY = 'th_audience_layers'
const ALL_ON = Object.fromEntries(LAYERS.map((l) => [l.id, true]))

const loadLayers = () => {
  try {
    const saved = JSON.parse(localStorage.getItem(LAYER_STORAGE_KEY))
    // 保存後にレイヤが増えても既定 ON で埋まるようにマージする
    return saved && typeof saved === 'object' ? { ...ALL_ON, ...saved } : ALL_ON
  } catch {
    return ALL_ON
  }
}

// 操作子を出しておく時間 (ms)。スクリーンに映るので放っておくと消す
const CONTROL_IDLE_MS = 3000

export default function AudienceView() {
  const ros = useRosbridge(undefined, { readOnly: true, mapThrottleMs: MAP_THROTTLE_MS })
  const {
    connected, modeName, fault,
    personStatus, candidates,
    followStatus, searchStatus,
    mapData, robotPose, scanData, pathData,
  } = ros

  // 音声は鳴らさず、同じトリガ判定をそのまま字幕へ流す
  const captionSink = useCaptionSink()
  useVoiceTriggers(ros, captionSink)

  const [layers, setLayers] = useState(loadLayers)
  useEffect(() => {
    localStorage.setItem(LAYER_STORAGE_KEY, JSON.stringify(layers))
  }, [layers])

  const toggleLayer = useCallback((id) => {
    setLayers((prev) => ({ ...prev, [id]: !prev[id] }))
  }, [])

  // ── 操作子の自動フェード ────────────────────────────────
  const [controlsVisible, setControlsVisible] = useState(true)
  const hideTimerRef = useRef(null)

  const poke = useCallback(() => {
    setControlsVisible(true)
    clearTimeout(hideTimerRef.current)
    hideTimerRef.current = setTimeout(() => setControlsVisible(false), CONTROL_IDLE_MS)
  }, [])

  useEffect(() => {
    poke()
    window.addEventListener('pointermove', poke)
    return () => {
      window.removeEventListener('pointermove', poke)
      clearTimeout(hideTimerRef.current)
    }
  }, [poke])

  // ── キーボード (1〜6)。発表者がカーソルを動かさずに切り替えられるように ──
  useEffect(() => {
    const onKeyDown = (e) => {
      const layer = LAYERS.find((l) => l.key === e.key)
      if (!layer) return
      e.preventDefault()
      toggleLayer(layer.id)
      poke()   // 何が変わったか分かるよう凡例を一時的に出す
    }
    window.addEventListener('keydown', onKeyDown)
    return () => window.removeEventListener('keydown', onKeyDown)
  }, [toggleLayer, poke])

  return (
    <div className="aud">

      <header className="aud-header">
        <h1>TH 追従ロボット</h1>
        <span className="aud-subtitle">LiDAR で人を見つけて追う</span>
        {!connected && <span className="aud-disconnect">● 未接続</span>}
      </header>

      <main className="aud-body">

        <div className="aud-world">
          <WorldCanvas
            mapData={mapData}
            robotPose={robotPose}
            scanData={scanData}
            pathData={pathData}
            personStatus={personStatus}
            candidates={candidates}
            layers={layers}
          />

          {/* 凡例そのものがトグル。色の説明と操作子を兼ねる (VISION.md §6.3) */}
          <div className={`aud-legend ${controlsVisible ? '' : 'faded'}`}>
            {LAYERS.map((l) => (
              <button
                key={l.id}
                className={`aud-legend-item ${layers[l.id] ? '' : 'off'}`}
                onClick={() => toggleLayer(l.id)}
                title={`${l.label} の表示切替 (キー ${l.key})`}
              >
                <span className="aud-swatch" style={{ background: l.color }} />
                {l.label}
                <kbd>{l.key}</kbd>
              </button>
            ))}
          </div>

          {!mapData && (
            <div className={`aud-hint ${controlsVisible ? '' : 'faded'}`}>
              地図なし — ロボット中心表示
            </div>
          )}
        </div>

        <JudgementPanel
          connected={connected}
          modeName={modeName}
          fault={fault}
          personStatus={personStatus}
          candidates={candidates}
          followStatus={followStatus}
          searchStatus={searchStatus}
          captions={captionSink.log}
          conditions={captionSink.conditions}
        />

      </main>
    </div>
  )
}
