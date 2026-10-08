import { defineConfig, devices } from '@playwright/test'

// Offline requirement (DetailedDesign-webui.md §9-5 / U-3) can only be
// verified against a production build — `npm run dev` (Vite dev server)
// injects its own HMR client/websocket and behaves differently from what
// ships to the robot. So the webServer here always builds first.
// 複数の worktree で e2e を並行させるときは PW_PORT でポートを分ける
// (既定 4173。CI=1 なら strictPort で、埋まっていれば落ちる)。
const PORT = process.env.PW_PORT || '4173'

export default defineConfig({
  testDir: './e2e',
  fullyParallel: true,
  forbidOnly: !!process.env.CI,
  retries: process.env.CI ? 1 : 0,
  reporter: 'list',
  use: {
    baseURL: `http://localhost:${PORT}`,
    trace: 'retain-on-failure',
  },
  projects: [
    { name: 'chromium', use: { ...devices['Desktop Chrome'] } },
  ],
  webServer: {
    command: `npm run build && npm run preview -- --port ${PORT} --strictPort`,
    url: `http://localhost:${PORT}`,
    reuseExistingServer: !process.env.CI,
    timeout: 120000,
  },
})
