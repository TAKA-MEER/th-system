// ros/paramsPending.js — SG-B8。効いている値と保存済み上書きの差分表示の
// 純粋ヘルパー（React 非依存。node --test から直接読める）。
// useParamsOverrides.js から再 export する。

// 効いている値と保存済みの上書き値の差だけを抜き出す。
// 戻り値は [{name, effective, pending}]（差があるものだけ。順序は names 順）。
export function pendingDiff(names, effective, pending) {
  const out = []
  for (const name of names) {
    if (pending != null && Object.prototype.hasOwnProperty.call(pending, name)) {
      out.push({ name, effective: effective?.[name] ?? null, pending: pending[name] })
    }
  }
  return out
}
