// Spec-safety §11: S-50 一般タブの「機体のスピーカー」区画。
// ON/OFF・音量スライダ・試し鳴らしボタンが speaker_announcer/set_parameters を
// 呼ぶこと（送信は window.__thSpeakerParamCalls に記録。useDevMode.js と同型）。
// 応答なし（rpi_ok=false）の表示も縛る。
import { test, expect } from '@playwright/test'
import { gotoScreenWithSpeaker, setTestSpeaker, speakerParamCalls } from './helpers.js'

const STATE = { mode: 'IDLE', tracker_enabled: true }
const SPEAKER = {
  cue: { cue: 'drive', enabled: true, volume: 60, test: false, rpi_ok: true },
  status: { cue: 'drive', volume: 60, error: '' },
}

async function openSpeakerSection(page, speaker = SPEAKER) {
  await gotoScreenWithSpeaker(page, 'S01', STATE, speaker)
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()
  await page.getByTestId('s50-sec-speaker').click()
  await expect(page.getByTestId('s50-speaker-toggle')).toBeVisible()
}

test('スピーカー区画: ON/OFF が set_parameters を呼ぶ', async ({ page }) => {
  await openSpeakerSection(page)
  await expect(page.getByTestId('s50-speaker-toggle')).toHaveText('スピーカー ON')

  await page.getByTestId('s50-speaker-toggle').click()
  const calls = await speakerParamCalls(page)
  assertHas(calls, { node: 'speaker_announcer', name: 'speaker_enabled', value: false })
})

test('スピーカー区画: 音量スライダが set_parameters を呼ぶ', async ({ page }) => {
  await openSpeakerSection(page)
  const slider = page.getByTestId('s50-speaker-volume')
  await expect(slider).toHaveValue('60')
  await slider.fill('70')
  const calls = await speakerParamCalls(page)
  assertHas(calls, { node: 'speaker_announcer', name: 'speaker_volume', value: 70 })
})

test('スピーカー区画: 試し鳴らしボタンが test_chime を送る', async ({ page }) => {
  await openSpeakerSection(page)
  await page.getByTestId('s50-speaker-test').click()
  const calls = await speakerParamCalls(page)
  assertHas(calls, { node: 'speaker_announcer', name: 'test_chime', value: true })
  await expect(page.getByTestId('s50-speaker-test-sent')).toBeVisible()
})

test('スピーカー区画: 応答なしを表示する', async ({ page }) => {
  await openSpeakerSection(page, {
    cue: { cue: 'none', enabled: true, volume: 60, test: false, rpi_ok: false },
    status: null,
  })
  await expect(page.getByTestId('s50-speaker-state')).toHaveText(/応答なし/)
})

test('スピーカー区画: 再生中の cue を表示する', async ({ page }) => {
  await openSpeakerSection(page)
  await expect(page.getByTestId('s50-speaker-state')).toHaveText(/再生中.*drive/)
  await setTestSpeaker(page, {
    cue: { cue: 'blocked', enabled: true, volume: 60, test: false, rpi_ok: true },
    status: { cue: 'blocked', volume: 60, error: '' },
  })
  await expect(page.getByTestId('s50-speaker-state')).toHaveText(/再生中.*blocked/)
})

function assertHas(calls, want) {
  const hit = calls.some((c) => c.node === want.node && c.name === want.name && c.value === want.value)
  if (!hit) throw new Error(`expected ${JSON.stringify(want)} in ${JSON.stringify(calls)}`)
}
