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
