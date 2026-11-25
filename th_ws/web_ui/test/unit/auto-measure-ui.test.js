// 8-g P5a-ui: 自律測定（AUTO_MEASURE）の画面側の台帳が spec と一致すること。
// 詳細設計 ^ DetailedDesign-measure.md §6.2（P5a-ui の決まり）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { menuItems, MENU_GROUPS, MENU_MODES, AUTO_MEASURE_MODE } from '../../src/screens/mainMenuItems.js'
import { resolveScreen, MODE_TO_SCREEN, SCREEN_IDS } from '../../src/screens/screenRouting.js'
import { isInZoneScreen } from '../../src/screens/screenZone.js'
import { REJECT_REASONS } from '../../src/i18n/reasons.js'
import { MODE_NAMES, modeLabel } from '../../src/i18n/modes.js'
import { stateLabel, STATE_NAMES } from '../../src/i18n/states.js'
import modeEntry from '../../src/generated/mode_entry.json' with { type: 'json' }
import attributes from '../../src/generated/attributes.json' with { type: 'json' }

test('mainMenuItems の「測定」群に自律測定（AUTO_MEASURE）が出る', () => {
  const measure = MENU_GROUPS.find((g) => g.key === 'measure')
  assert.ok(measure, 'measure 群が無い')
  assert.deepEqual(measure.modes, ['AUTO_MEASURE'])
  assert.ok(MENU_MODES.includes(AUTO_MEASURE_MODE))
})

test('IDLE から自律測定は押せる（mode_entry.json の IDLE に AUTO_MEASURE）', () => {
  assert.ok(modeEntry.IDLE.includes('AUTO_MEASURE'))
  const items = menuItems({ mode: 'IDLE', tracker_enabled: true }, modeEntry, attributes,
    { routeCount: 1, leashPresent: true })
  const auto = items.find((it) => it.mode === 'AUTO_MEASURE')
  assert.equal(auto.enabled, true)
})

test('AUTO_MEASURE 以外のモードからは自律測定を押せない（IDLE でない）', () => {
  const items = menuItems({ mode: 'MANUAL', tracker_enabled: true }, modeEntry, attributes,
    { routeCount: 1, leashPresent: true })
  const auto = items.find((it) => it.mode === 'AUTO_MEASURE')
  assert.equal(auto.enabled, false)
  assert.equal(auto.reasonKey, 'mode_entry_denied')
})

test('resolveScreen: AUTO_MEASURE -> S60', () => {
  assert.equal(MODE_TO_SCREEN.AUTO_MEASURE, 'S60')
  assert.equal(SCREEN_IDS.S60, 'S-60')
  assert.equal(resolveScreen({
    testScreen: null, passedConnect: true, mode: 'AUTO_MEASURE',
    settingsOpen: false, subState: 'WAIT',
  }), 'S60')
})

test('screenZone: S-60 は場内（IN）', () => {
  assert.equal(isInZoneScreen('S-60'), true)
})

test('i18n: AUTO_MEASURE の表示名は自律測定（Spec-webui.md §8.1）', () => {
  assert.equal(modeLabel('AUTO_MEASURE'), '自律測定')
  assert.equal(MODE_NAMES.AUTO_MEASURE, '自律測定')
})

test('i18n/states: WAIT は指示待ち（Spec-webui.md §3.16 の状態表示）', () => {
  assert.equal(STATE_NAMES.WAIT, '指示待ち')
  assert.equal(stateLabel('WAIT'), '指示待ち')
})

test('i18n/states: ALIGN は向き合わせ中（8-g P5b。PANEL_NAV でも使う）', () => {
  assert.equal(STATE_NAMES.ALIGN, '向き合わせ中')
  assert.equal(stateLabel('ALIGN'), '向き合わせ中')
})

test('attributes: AUTO_MEASURE の run_state は NAV（P5b。AppShell の noRunFallback が効かない）', () => {
  assert.equal(attributes.AUTO_MEASURE.run_state, 'NAV')
})

test('i18n/reasons: measure_link_lost / measure_version_mismatch の文言', () => {
  assert.ok(REJECT_REASONS.measure_link_lost, 'measure_link_lost の文言が無い')
  assert.ok(REJECT_REASONS.measure_version_mismatch, 'measure_version_mismatch の文言が無い')
  assert.notEqual(REJECT_REASONS.measure_link_lost, REJECT_REASONS.measure_version_mismatch)
})
