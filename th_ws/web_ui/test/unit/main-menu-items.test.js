// DetailedDesign-wp1.md WP-UI-02 §7 / §4.1: menuItems() drives S-01's
// buttons purely from mode_entry.json + attributes.json + SystemState,
// never hardcoded per screen.
import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { menuItems, MENU_MODES, MENU_GROUPS } from '../../src/screens/mainMenuItems.js'

const modeEntry = JSON.parse(
  readFileSync(fileURLToPath(new URL('../../src/generated/mode_entry.json', import.meta.url)), 'utf8'),
)
const attributes = JSON.parse(
  readFileSync(fileURLToPath(new URL('../../src/generated/attributes.json', import.meta.url)), 'utf8'),
)

function byMode(items) {
  return Object.fromEntries(items.map((it) => [it.mode, it]))
}

test('MENU_GROUPS is exactly MENU_MODES, partitioned once each', () => {
  const fromGroups = MENU_GROUPS.flatMap((g) => g.modes)
  assert.deepEqual([...fromGroups].sort(), [...MENU_MODES].sort())
  assert.equal(new Set(fromGroups).size, fromGroups.length)
})

test('M-1: mode INIT disables every item, with no reason (nothing to explain yet)', () => {
  const items = menuItems({ mode: 'INIT' }, modeEntry, attributes)
  assert.equal(items.length, MENU_MODES.length)
  for (const it of items) {
    assert.equal(it.enabled, false, `${it.mode} should be disabled while INIT`)
    assert.equal(it.reasonKey, null)
  }
})

test('fail-safe: unknown mode (null) behaves exactly like INIT', () => {
  const items = menuItems({ mode: null }, modeEntry, attributes)
  for (const it of items) assert.equal(it.enabled, false)
  assert.deepEqual(menuItems({ mode: null }, modeEntry, attributes), menuItems({ mode: 'INIT' }, modeEntry, attributes))
})

test('mode IDLE with tracker enabled: every menu mode is enabled (mode_entry.yaml allows all 10 from IDLE)', () => {
  const items = byMode(menuItems({ mode: 'IDLE', tracker_enabled: true }, modeEntry, attributes))
  for (const m of MENU_MODES) {
    assert.equal(items[m].enabled, true, `${m} should be enabled from IDLE`)
    assert.equal(items[m].reasonKey, null)
  }
})

test('mode IDLE with tracker disabled: FOLLOW/TEACH_FOLLOW blocked (needs_tracker: required), PREP exempt', () => {
  const items = byMode(menuItems({ mode: 'IDLE', tracker_enabled: false }, modeEntry, attributes))
  assert.equal(items.FOLLOW.enabled, false)
  assert.equal(items.FOLLOW.reasonKey, 'tracker_disabled')
  assert.equal(items.TEACH_FOLLOW.enabled, false)
  assert.equal(items.TEACH_FOLLOW.reasonKey, 'tracker_disabled')
  // PREP's needs_tracker is 'required' too, but only inside its REGISTER
  // sub-state (DetailedDesign-state.md §8.2 footnote ※※) -- must not be
  // gated at menu level.
  assert.equal(items.PREP.enabled, true)
  // Unrelated modes are unaffected.
  assert.equal(items.MANUAL.enabled, true)
  assert.equal(items.OPCHECK.enabled, true)
})

test('mode FOLLOW: mode_entry.yaml only allows IDLE from FOLLOW, so every menu item is denied', () => {
  const items = menuItems({ mode: 'FOLLOW', tracker_enabled: true }, modeEntry, attributes)
  for (const it of items) {
    assert.equal(it.enabled, false, `${it.mode} should be denied from FOLLOW`)
    assert.equal(it.reasonKey, 'mode_entry_denied')
  }
})

test('mode MANUAL: only OPCHECK/CALIB are allowed (mode_entry.yaml: MANUAL -> [IDLE, OPCHECK, CALIB])', () => {
  const items = byMode(menuItems({ mode: 'MANUAL', tracker_enabled: true }, modeEntry, attributes))
  assert.equal(items.OPCHECK.enabled, true)
  assert.equal(items.CALIB.enabled, true)
  assert.equal(items.FOLLOW.enabled, false)
  assert.equal(items.FOLLOW.reasonKey, 'mode_entry_denied')
})
