// e2e/helpers.js — shared setup for shell e2e specs.
//
// The shell must be testable with no rosbridge backend at all (there is
// none in CI, and the offline requirement means the production build must
// never depend on one to render). `window.__thTestState`, read by
// ros/useSystemState.js before it opens any WebSocket, lets a spec seed
// (and later mutate, via `window.__thSetTestState`) the SystemState the
// shell renders against.
export async function gotoWithState(page, state = {}) {
  await page.addInitScript((s) => { window.__thTestState = s }, state)
  await page.goto('/')
}

export async function setTestState(page, patch) {
  await page.evaluate((p) => window.__thSetTestState(p), patch)
}

export async function setTestFault(page, patch) {
  await page.evaluate((p) => window.__thSetTestFault(p), patch)
}

// WP-UI-02: opens a specific screen directly (src/main.jsx's
// window.__thTestScreen hook), instead of the WP-UI-01 OperationCard
// harness gotoWithState() defaults to. Existing specs (estop-clickable etc.)
// must keep using gotoWithState() unchanged -- they depend on the harness's
// [data-testid="opcard-harness"] -- so this is a separate helper, not a
// change to gotoWithState() itself.
export async function gotoScreen(page, screen, state = {}) {
  await page.addInitScript(({ s, scr }) => {
    window.__thTestState = s
    window.__thTestScreen = scr
  }, { s: state, scr: screen })
  await page.goto('/')
}

// Stubs a std_srvs/Trigger-shaped service call (ros/useStdTrigger.js's test
// hook) for /shutdown/prepare / /shutdown/execute. Must be called via
// addInitScript (before the page's first render) since S01Main reads
// window.__thTestServices synchronously on first call.
export async function stubServices(page, services) {
  await page.addInitScript((s) => { window.__thTestServices = s }, services)
}

// Press-and-hold the virtual stick at `stick` (a Locator for `.stick svg`)
// with a real mouse gesture. `offsetX`/`offsetY` は中心からの倒し量を
// **短辺の半分に対する割合**で与える（右／上が正。1.0 = 円の縁）。 Uses page.mouse so the pointer is a genuine active
// pointer and the stick's setPointerCapture() succeeds; the button stays
// down for the caller to release with page.mouse.up(). See
// jog-lease-rate.spec.js / w6-stick-responds.spec.js.
export async function downOnStick(page, stick, { offsetX = 0.45, offsetY = 0 } = {}) {
  const box = await stick.boundingBox()
  if (!box) throw new Error('stick svg has no bounding box (is it visible?)')
  const cx = box.x + box.width / 2
  const cy = box.y + box.height / 2
  // オフセットは **短辺** に対する割合で与える（VirtualStick の update() と
  // 同じ基準）。走行タブでは `.stick` が横に伸びて箱が 1129x230 になるため、
  // 箱の幅を基準にすると円の外側を押すことになる。
  const span = Math.min(box.width, box.height) / 2
  await page.mouse.move(cx + offsetX * span, cy + offsetY * span)
  await page.mouse.down()
}

export async function jogPublishes(page, topic) {
  return (await page.evaluate(() => window.__thJogPublishes ?? []))
    .filter((p) => p.topic === topic)
}
