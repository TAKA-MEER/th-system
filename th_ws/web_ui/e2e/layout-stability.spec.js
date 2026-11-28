// e2e/layout-stability.spec.js — 内容で枠とボタンが動かないこと（D106・brief-F0 の 2）。
//
// 2026-11-28 ユーザー指摘「少しのことでカードやボタンが動く」。同じ画面の中で、
//   状態・数値の桁（0.0 と 12.4）・ラベルの長さ（短い名前と長い名前）・帯や警告の有無・選択の有無
// を揺らしても、操作バー・右列（①〜④）・手順の下端の「枠」と「押すもの」の rect が 1 px 以内で
// 変わらないことを、横長 1280×720 と縦長 960×1280 の両方で確かめる。
//
// 測るもの: 枠 [data-pos=…]（大きさ 0 の枠も測る。場所を空けていないこと自体が動く原因）と、
//   その中の押すもの（button・role=tab・input・.stoprun）。押すものは data-testid（無ければ枠と順番）で対にし、
//   揺らす前後の両方にあるものだけ比べる（現れる・消える部品は、ほかを動かさなければよい）。
// 今の時点で動いてしまう画面は FIXME に理由を書いて test.fixme にする（画面ごとの段で外す）。
import { test, expect } from '@playwright/test'

const PIN = (id, name, kind, x, y) => ({
  id, name, kind, registered_at: '',
  pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: 0.7071, w: 0.7071 } },
})
const PINS_SHORT = [PIN('p1', 'A-1', 'PANEL', 2, 1), PIN('p2', 'A-2', 'PANEL', 4, 1), PIN('p6', '待機場所', 'HOME', 0, 0)]
const PINS_LONG = [PIN('p1', '第二実験棟 東側 配電盤 A-1 予備系統', 'PANEL', 2, 1), PIN('p2', 'A-2', 'PANEL', 4, 1), PIN('p6', '待機場所', 'HOME', 0, 0)]
const TARGETS = (sel) => ({
  candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }], selected_index: sel, confidence: 0.87, is_lost: false, lost_reason: '',
})
const ROUTES = (name) => [
  { id: 'route_a', name, length_m: 8.2, point_count: 60, start_yaw: 0.1, recorded_at: 0 },
  { id: 'route_b', name: '経路B', length_m: 12.4, point_count: 90, start_yaw: 0.1, recorded_at: 0 },
]
const PREVIEW = Array.from({ length: 40 }, (_, i) => ({ x: i * 0.2, y: Math.sin(i / 5) }))
const ONSITE = {
  __thTestOnsitePins: PINS_SHORT, __thTestPersonTargets: TARGETS(0),
  __thTestOdomPose: { x: 0, y: 0, yaw: 0 }, __thTestRoutePose: { x: 0, y: 0, yaw: 0 }, __thTestHomeDeclared: true,
  __thTestMapSession: { slot: 'VENUE', mode: 'LOADED', dirty: false, instance_id: 'x' },
}
// 帯（W-3 の案内。/system/effect の guide）。揺らしたら帯が本当に出たことも確かめる（出ていなければ何も揺らしていない）
const GUIDE = { name: 'guide', dest: 'WebUI', args_json: JSON.stringify({ key: 'summon_clear_timeout' }) }
const banner = async (page) => {
  await page.evaluate((m) => window.__thSetTestSystemEffect(m), GUIDE)
  await expect(page.getByTestId('w3-guide')).toBeVisible()
}
// 押せないボタンを押して、理由の吹き出しを出す（出たことも確かめる）。
// 捕まえたポインタなどに邪魔されないよう dispatchEvent で押す
const reason = (btn, bubble) => async (page) => {
  await page.getByTestId(btn).dispatchEvent('click')
  await expect(page.locator(bubble)).toBeVisible()
}

// 種の書き換え（ros/use*.js の試験用の縫い目）。fn を配列にすると、在るものを全部呼ぶ（1 つも無ければ落とす）
const set = (fn, v) => async (page) => page.evaluate(({ fs, x }) => {
  const have = fs.filter((f) => typeof window[f] === 'function')
  if (!have.length) throw new Error(`${fs.join('/')} が無い（この画面はその種を購読していない）`)
  for (const f of have) window[f](x)
}, { fs: [].concat(fn), x: v })
const st = (v) => set('__thSetTestState', v)
const click = (id) => async (page) => page.getByTestId(id).click()

// [名前, 画面, 初めの状態, 種, [ [揺らし方の名前, 揺らし方], ... ]]
const CASES = [
  ['S-10 追従走行', 'S10', { mode: 'FOLLOW', state: 'SELECT', tracker_enabled: true }, { ...ONSITE, __thTestPersonTargets: TARGETS(-1) }, [
    ['選択の有無', set('__thSetTestPersonTargets', TARGETS(0))],
    ['状態 CONFIRM', st({ mode: 'FOLLOW', state: 'CONFIRM', tracker_enabled: true })],
    ['状態 RUN', st({ mode: 'FOLLOW', state: 'RUN', tracker_enabled: true })],
    ['状態 PAUSE', st({ mode: 'FOLLOW', state: 'PAUSE', tracker_enabled: true })],
    ['帯あり', banner],
  ]],
  ['S-11 手動走行', 'S11', { mode: 'MANUAL', state: 'PAUSE', tracker_enabled: true }, {}, [
    ['状態 RUN', st({ mode: 'MANUAL', state: 'RUN', tracker_enabled: true })],
    ['数値の桁', set('__thSetTestOdomTwist', { linear: { x: 0.45, y: 0, z: 0 }, angular: { x: 0, y: 0, z: -1.25 } })],
    ['帯あり', banner],
    ['状態 PAUSE', st({ mode: 'MANUAL', state: 'PAUSE', tracker_enabled: true })],
  ]],
  ['S-12 教示（追従）', 'S12', { mode: 'TEACH_FOLLOW', state: 'REC', tracker_enabled: true },
    { __thTestRouteCatalog: ROUTES('経路A'), __thTestRoutePreview: PREVIEW, __thTestPersonTargets: TARGETS(0), __thTestRouteStatus: { recorded_m: 1.3, elapsed_sec: 9, points: 10, saved: false } }, [
    ['数値の桁', set('__thSetTestRouteStatus', { recorded_m: 12.4, elapsed_sec: 912, points: 100, saved: false })],
    ['状態 PAUSE', st({ mode: 'TEACH_FOLLOW', state: 'PAUSE', tracker_enabled: true })],
    ['状態 REC', st({ mode: 'TEACH_FOLLOW', state: 'REC', tracker_enabled: true })],
  ]],
  ['S-13 教示（手動）', 'S13', { mode: 'TEACH_MANUAL', state: 'PAUSE', tracker_enabled: true }, { __thTestRoutePreview: PREVIEW, __thTestRouteStatus: { recorded_m: 1.3, points: 10, saved: false } }, [
    ['状態 REC', st({ mode: 'TEACH_MANUAL', state: 'REC', tracker_enabled: true })],
    ['数値の桁', set('__thSetTestRouteStatus', { recorded_m: 12.4, points: 100, saved: false })],
    ['状態 SAVED', st({ mode: 'TEACH_MANUAL', state: 'SAVED', tracker_enabled: true })],
  ]],
  ['S-14 教示再生', 'S14', { mode: 'REPLAY', state: 'READY', tracker_enabled: true },
    { __thTestRouteCatalog: ROUTES('A'), __thTestRoutePreview: PREVIEW, __thTestRouteStatus: { target_index: 1, cross_track_m: 0.0, cross_track_max_m: 0.0 } }, [
    ['状態 RUN', st({ mode: 'REPLAY', state: 'RUN', tracker_enabled: true })],
    ['数値の桁', set('__thSetTestRouteStatus', { target_index: 5, cross_track_m: 12.4, cross_track_max_m: 12.4 })],
    ['ラベルの長さ', set('__thSetTestRouteCatalog', ROUTES('第二実験棟の東側をぐるりと回る長い名前の経路'))],
    ['押せない理由', reason('s14-reselect', '[data-testid="s14-reselect-reason"]')],
    ['帯あり', banner],
    ['状態 PAUSE', st({ mode: 'REPLAY', state: 'PAUSE', tracker_enabled: true })],
  ]],
  ['S-20 試験準備', 'S20', { mode: 'PREP', state: 'MAPPING', tracker_enabled: true }, { ...ONSITE, __thTestMappingActive: true }, [
    ['状態 REGISTER', st({ mode: 'PREP', state: 'REGISTER', tracker_enabled: true })],
    ['状態 MAPPING・未保存あり', st({ mode: 'PREP', state: 'MAPPING', tracker_enabled: true, unsaved: ['venue_map'] })],
    ['ラベルの長さ', set('__thSetTestOnsitePins', PINS_LONG)],
    ['状態 RETURN', st({ mode: 'PREP', state: 'RETURN', tracker_enabled: true })],
  ]],
  ['S-21 試験（当日）', 'S21', { mode: 'AT_HOME', state: 'IDLE_H', tracker_enabled: true }, ONSITE, [
    ['状態 移動中', st({ mode: 'PANEL_NAV', state: 'NAV', tracker_enabled: true, target_pin: 'p1' })],
    ['数値の桁', set(['__thSetTestOdomPose', '__thSetTestRoutePose'], { x: -10, y: -8, yaw: 0 })],
    ['ラベルの長さ', set('__thSetTestOnsitePins', PINS_LONG)],
    ['状態 一時停止', st({ mode: 'PANEL_NAV', state: 'PAUSE', tracker_enabled: true, target_pin: 'p1' })],
    ['状態 盤の前', st({ mode: 'AT_PANEL', state: 'IDLE_P', tracker_enabled: true })],
  ]],
  ['S-30 始業点検', 'S30', { mode: 'OPCHECK', state: 'LIST' }, {}, [
    ['押せない理由', reason('s30-abort', '[data-pos="foot"] .reason-bubble')],
    ['状態 点検中', async (page) => {
      await st({ mode: 'OPCHECK', state: 'RUNNING_CHECK' })(page)
      await set('__thSetTestOpcheckStatus', { item: 'MOTOR', result: 'UNKNOWN', detail: 'done= hold=NONE', next_screen: '' })(page)
    }],
    ['結果あり', set('__thSetTestOpcheckStatus', { item: 'MOTOR', result: 'OK', detail: '', next_screen: '' })],
  ]],
  ['S-40 校正', 'S40', { mode: 'CALIB', state: 'S1' }, { __thTestCalibStatus: { item: 'LINEAR', step: 'S1', result: 'GUIDE', detail: { commanded: 1.0, progress: 0 } } }, [
    ['状態 S2', async (page) => {
      await st({ mode: 'CALIB', state: 'S2' })(page)
      await set('__thSetTestCalibStatus', { item: 'LINEAR', step: 'S2', result: 'RUNNING', detail: { commanded: 1.0, progress: 0.4 } })(page)
    }],
    ['状態 S3', async (page) => {
      await st({ mode: 'CALIB', state: 'S3' })(page)
      await set('__thSetTestCalibStatus', { item: 'LINEAR', step: 'S3', result: 'WAIT_MEASURED', detail: { commanded: 1.0, progress: 1 } })(page)
    }],
  ]],
  ['S-50 設定', 'S01', { mode: 'IDLE', state: 'NONE', tracker_enabled: true }, {}, [
    ['区分 知らせる', click('s50-sec-notify')],
    ['区分 開発モード', click('s50-sec-dev')],
    ['区分 異常の履歴', click('s50-sec-log')],
  ], async (page) => { await page.getByTestId('s01-open-settings').click(); await page.getByTestId('s50-sec-run').click() }],
]

// 今の時点で動いてしまうもの（画面ごとの段で外す）。キーは `<名前> <向き>`。
const FIXME = {
  // 停止│走行の下に「予約」の説明（s20-reserved-note）が出たり消えたりし、右列の③④が 25 px 上下する（S20Prep.jsx の toggle 枠）
  'S-20 試験準備 横長': 'S-20 の段で直す: 停止│走行の下の予約の説明が出入りして ③④ が 25 px 動く',
  'S-20 試験準備 縦長': 'S-20 の段で直す: 停止│走行の下の予約の説明が出入りして ③ が 25 px 動く',
  // 縦長の一覧（2 列の格子）の下に手順（.csteps）が点検中だけ現れ、一覧が 131 px 伸びて手順の枠が下がる
  'S-30 始業点検 縦長': 'S-30 の段で直す: 縦長で点検中だけ一覧の下に手順が現れ、手順の枠が 131 px 下がる',
}

const SIZES = [
  { name: '横長', width: 1280, height: 720 },
  { name: '縦長', width: 960, height: 1280 },
]
const SLOTS = ['opbar', 'where', 'toggle', 'ops', 'jog', 'main', 'list', 'body', 'foot']
const WATCH = ['opbar', 'where', 'toggle', 'ops', 'jog', 'foot']

async function measure(page) {
  return page.evaluate(({ slots, watch }) => {
    const out = {}
    const app = document.getElementById('app').getBoundingClientRect()
    const r = (e) => { const b = e.getBoundingClientRect(); return [b.left - app.left, b.top - app.top, b.width, b.height].map((v) => Math.round(v * 10) / 10) }
    const shown = (e) => e.getClientRects().length > 0 && getComputedStyle(e).visibility !== 'hidden'
    for (const s of slots) {
      const el = document.querySelector(`#body [data-pos="${s}"]`)
      if (el && shown(el)) out[`[${s}]`] = r(el)
    }
    for (const s of watch) {
      const el = document.querySelector(`#body [data-pos="${s}"]`)
      if (!el) continue
      const items = [...el.querySelectorAll('button, [role="tab"], input, .stoprun')].filter(shown)
      items.forEach((e, i) => {
        const key = e.dataset.testid ? `${s}:${e.dataset.testid}` : `${s}:${e.tagName.toLowerCase()}#${i}`
        out[key] = r(e)
      })
    }
    return out
  }, { slots: SLOTS, watch: WATCH })
}

function diff(a, b) {
  const moved = []
  for (const k of Object.keys(a)) {
    if (!(k in b)) continue
    const d = a[k].map((v, i) => b[k][i] - v)
    if (d.some((x) => Math.abs(x) > 1)) moved.push(`${k} [x,y,w,h]${JSON.stringify(a[k])}→${JSON.stringify(b[k])}`)
  }
  return moved
}

for (const size of SIZES) {
  for (const [name, screen, state, seeds, variants, before] of CASES) {
    test(`${name}（${size.name}）: 状態・桁・ラベル・帯・選択で操作バーと右列が動かない`, async ({ page }) => {
      const fx = FIXME[`${name} ${size.name}`]
      test.fixme(!!fx, fx)
      await page.setViewportSize({ width: size.width, height: size.height })
      await page.addInitScript(({ s, scr, sd }) => {
        window.__thTestState = s
        window.__thTestScreen = scr
        for (const k of Object.keys(sd)) window[k] = sd[k]
      }, { s: state, scr: screen, sd: seeds })
      await page.goto('/')
      await page.locator('#body').waitFor()
      await page.addStyleTag({ content: '*,*::before,*::after{transition:none!important;animation:none!important}' })
      if (before) await before(page)
      await page.waitForTimeout(100)
      const base = await measure(page)
      expect(Object.keys(base).length, '測る枠が無い').toBeGreaterThan(2)
      const moved = []
      for (const [vn, apply] of variants) {
        await apply(page)
        await page.waitForTimeout(100)
        const now = await measure(page)
        for (const m of diff(base, now)) moved.push(`${vn}: ${m}`)
      }
      expect(moved, `${name}（${size.name}）で動いたもの`).toEqual([])
    })
  }
}
