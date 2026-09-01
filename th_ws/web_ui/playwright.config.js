import { defineConfig, devices } from '@playwright/test'

// Offline requirement (DetailedDesign-webui.md §9-5 / U-3) can only be
// verified against a production build — `npm run dev` (Vite dev server)
// injects its own HMR client/websocket and behaves differently from what
// ships to the robot. So the webServer here always builds first.
export default defineConfig({
  testDir: './e2e',
  fullyParallel: true,
  forbidOnly: !!process.env.CI,
  retries: process.env.CI ? 1 : 0,
  reporter: 'list',
  use: {
    baseURL: 'http://localhost:4173',
    trace: 'retain-on-failure',
  },
  projects: [
    { name: 'chromium', use: { ...devices['Desktop Chrome'] } },
  ],
  webServer: {
    command: 'npm run build && npm run preview -- --port 4173 --strictPort',
    url: 'http://localhost:4173',
    reuseExistingServer: !process.env.CI,
    timeout: 120000,
  },
})
