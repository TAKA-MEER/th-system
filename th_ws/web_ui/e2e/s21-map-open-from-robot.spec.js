// e2e/s21-map-open-from-robot.spec.js — S-21「会場地図を開いた」は機体が持つ
// （Spec-webui.md §3.11 S-21。実機 2026-10-13: 手押し（CARRY）から戻る・再読み込みで
// 画面ローカルの記憶が消え、機体は地図を開いたままなのに地図が出なくなった）。
//
// 機体の /map_session/status の 1 通は window.__thSetTestMapSession（TEST_MODE は
// rosbridge を張らない。useFaultHistory の __thSetTestFaultHistory と同じ流儀）で再現する。
// 画面を作り直す場合は window.__thTestMapSession（種）を addInitScript で渡して再読み込みする
// （TRANSIENT_LOCAL で最新の 1 通が購読し直しに届くことの再現）。
import { test, expect } from '@playwright/test'
import { gotoScreenWithOnsite, setTestState } from './helpers.js'

const IDLE = { mode: 'IDLE', state: 'NONE' }
const status = (mode, slot = 'VENUE') => ({
  slot, session_id: 'venue', mode, dirty: false, instance_id: 'abc',
})
const setStatus = (page, msg) => page.evaluate((m) => window.__thSetTestMapSession(m), msg)

async function openS21(page, seed) {
  await page.addInitScript((s) => { if (s) window.__thTestMapSession = s }, seed ?? null)
  await gotoScreenWithOnsite(page, 'S21', IDLE, { pins: [], targets: [] })
  await page.locator('#s21').waitFor()
}

const gate = (page) => page.getByTestId('s21-map-gate-msg')
const map = (page) => page.getByTestId('s21-map')

test('(a) 機体が LOCALIZING の VENUE を伝えていれば、画面ローカルが空でも地図タブが出る（再読み込み後も）', async ({ page }) => {
  await openS21(page, status('LOCALIZING'))
  await expect(map(page)).toBeVisible()
  await expect(gate(page)).toHaveCount(0)
  await page.reload()
  await page.locator('#s21').waitFor()
  await expect(map(page)).toBeVisible()
  await expect(gate(page)).toHaveCount(0)
})

test('(b) UNLOADED／MAPPING／ROUTE では「保存した会場地図を開いてください」', async ({ page }) => {
  await openS21(page, status('UNLOADED'))
  await expect(gate(page)).toBeVisible()
  await expect(map(page)).toHaveCount(0)
  await setStatus(page, status('MAPPING'))
  await expect(gate(page)).toBeVisible()
  await setStatus(page, status('LOCALIZING', 'ROUTE'))
  await expect(gate(page)).toBeVisible()
  await setStatus(page, status('LOCALIZING'))
  await expect(map(page)).toBeVisible()
  // 破棄・作り直しで UNLOADED に戻ったら、また閉じる
  await setStatus(page, status('UNLOADED'))
  await expect(gate(page)).toBeVisible()
})

test('(b2) ローカルの「開きました」が残っていても、機体が UNLOADED なら偽（嘘の「開いた」にならない）', async ({ page }) => {
  await page.addInitScript(() => { window.__thTestMapSession = { slot: 'VENUE', mode: 'LOCALIZING' } })
  await gotoScreenWithOnsite(page, 'S21', IDLE, {
    pins: [], targets: [], openVenueMap: { success: true, message: '' },
  })
  await expect(map(page)).toBeVisible()
  await page.getByTestId('s21-open-venue-map').click()
  await expect(page.getByTestId('s21-open-venue-msg')).toBeVisible()
  await setStatus(page, status('UNLOADED'))
  await expect(gate(page)).toBeVisible()
  await expect(map(page)).toHaveCount(0)
})

test('(c) 手押し相当（CARRY → 元のモード）と作り直し（再読み込み）をはさんでも地図は消えない', async ({ page }) => {
  await openS21(page, status('LOCALIZING'))
  await expect(map(page)).toBeVisible()
  await setTestState(page, { mode: 'CARRY', state: 'NONE' })
  await setTestState(page, IDLE)
  await expect(map(page)).toBeVisible()
  // 作り直し: 画面ごと作り直しても、機体の最新の 1 通で地図が出る
  await page.reload()
  await page.locator('#s21').waitFor()
  await expect(map(page)).toBeVisible()
  // 手順バー（段 1）も巻き戻らない
  await expect(page.getByTestId('s21-venue-already-open')).toBeVisible()
})

test('(d) 機体の値が届く前は従来どおり（地図タブは閉じ、ローカルの応答で開く）', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S21', IDLE, {
    pins: [], targets: [], openVenueMap: { success: true, message: '' },
  })
  await expect(gate(page)).toBeVisible()
  await page.getByTestId('s21-map-gate-open').click()
  await expect(map(page)).toBeVisible()
})
