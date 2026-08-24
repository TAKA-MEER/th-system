// ros/clientId.js — persists this browser's ActiveScreen.client_id
// (th_system_msgs/ActiveScreen.msg, DetailedDesign-wp1.md WP-UI-01 §3.1).
//
// Generated once with crypto.randomUUID() (no network call involved) and
// kept in localStorage so a reload of the same tab/device keeps counting
// as the same terminal for derive_limits()'s interacting-client tally
// (DetailedDesign-names.md §4.1). A fresh id on every reload would make one
// tablet look like a churn of terminals to that consumer.
const STORAGE_KEY = 'th_ui_client_id'

export function getClientId() {
  if (typeof window === 'undefined') return ''
  try {
    const existing = window.localStorage.getItem(STORAGE_KEY)
    if (existing) return existing
    const id = window.crypto.randomUUID()
    window.localStorage.setItem(STORAGE_KEY, id)
    return id
  } catch {
    // localStorage unavailable (private mode / disabled) -- fall back to a
    // page-lifetime-only id rather than an empty string.
    return window.crypto?.randomUUID?.() ?? ''
  }
}
