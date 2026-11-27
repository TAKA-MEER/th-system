// W0a レイアウト骨組み・人検出の縛り。
import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'

test('レイアウトは data-pos で位置を固定する（状態で動かさない）', () => {
  const src = readFileSync(new URL('../../src/shell/Layouts.jsx', import.meta.url), 'utf8')
  for (const pos of ['opbar', 'main', 'where', 'toggle', 'ops', 'jog', 'list', 'body', 'foot']) {
    assert.ok(src.includes(`data-pos="${pos}"`), `data-pos=${pos} が無い`)
  }
  assert.ok(!src.includes('role="tab"'), 'タブは使わない')
})

test('人検出トグルは入のときだけ点く（JSX 内の条件）', () => {
  const src = readFileSync(new URL('../../src/parts/PersonDetectToggle.jsx', import.meta.url), 'utf8')
  assert.ok(src.includes('labelOff') && src.includes('labelOn'))
  assert.ok(src.includes('aria-pressed'))
})
