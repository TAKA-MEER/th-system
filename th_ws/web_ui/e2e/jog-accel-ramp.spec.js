// ジョグの加速度制限（2026-09-02 復活）。
//
// 旧 App.jsx は JOG_LIN_ACCEL=1.0 / JOG_ANG_ACCEL=4.0 を持ち、コメントに
// 「無いとスティック/キー入力の変化が瞬時にそのまま速度指令へ反映され機体が
// 激しく揺れる」という実機検証の記録があった。新 UI へ移す際に rampToward だけが
// stickGeometry.js へ移植され**呼び出しが失われていた**（本番から一度も
// 呼ばれていなかった）。急発進は車輪を滑らせ、/odom は車輪速度フィードバック
// 由来なので教示精度そのものを損なう。
//
// ここが守る性質:
//   1. スティックを一気に倒しても最初の指令が目標値に飛ばない（ランプする）
//   2. 手を離したときのゼロは**ランプを通さない**（通すと離しても進み続ける）
import { test, expect } from '@playwright/test'
import { gotoScreen, downOnStick, jogPublishes } from './helpers.js'

test('スティックを一気に倒しても指令はランプする（急発進しない）', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()

  // 真上＝前進フルスケール。倒した瞬間から最初の 1 指令を見る。
  await downOnStick(page, stick, { offsetX: 0, offsetY: -0.45 })
  const first = (await jogPublishes(page, '/cmd_vel_manual_raw'))[0]
  expect(first, '最初の指令が記録されていない').toBeTruthy()

  // CMD_MS=100ms・JOG_LIN_ACCEL=1.0 m/s^2 なので 1 ティックの上限は 0.1。
  // ランプが無ければ最初から目標値（プリセット倍率ぶん）が出てしまう。
  expect(Math.abs(first.cmd.vx),
    `最初の指令 ${first.cmd.vx} が 1 ティック分(0.1)を超えている＝ランプしていない`)
    .toBeLessThanOrEqual(0.1 + 1e-9)

  // 保持し続ければ目標へ向かって単調に増えること（ランプが進行する）。
  await page.waitForTimeout(600)
  const cmds = (await jogPublishes(page, '/cmd_vel_manual_raw')).filter((c) => c.cmd)
  expect(cmds.length).toBeGreaterThan(2)
  const vxs = cmds.map((c) => c.cmd.vx)
  expect(Math.max(...vxs), '保持しても速度が上がっていない').toBeGreaterThan(Math.abs(first.cmd.vx))
  // 各ステップの増分が加速度上限を超えないこと
  for (let i = 1; i < vxs.length; i++) {
    expect(Math.abs(vxs[i] - vxs[i - 1]),
      `ステップ ${i} の増分が加速度上限を超えた`).toBeLessThanOrEqual(0.1 + 1e-6)
  }

  await page.mouse.up()
})

test('手を離したときのゼロはランプを通さず即座に 0 になる', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()

  // 十分に加速させてから離す（ランプを通すと 0 まで数ティックかかるはず）。
  await downOnStick(page, stick, { offsetX: 0, offsetY: -0.45 })
  await page.waitForTimeout(600)
  const beforeRelease = (await jogPublishes(page, '/cmd_vel_manual_raw')).filter((c) => c.cmd)
  expect(Math.abs(beforeRelease[beforeRelease.length - 1].cmd.vx),
    '離す前に十分加速していない（テストの前提が崩れている）').toBeGreaterThan(0.1)

  await page.mouse.up()

  const cmds = (await jogPublishes(page, '/cmd_vel_manual_raw')).filter((c) => c.cmd)
  const last = cmds[cmds.length - 1]
  expect(last.cmd.vx, '解放時のゼロがランプを通ってしまっている').toBe(0)
  expect(last.cmd.wz).toBe(0)
})
