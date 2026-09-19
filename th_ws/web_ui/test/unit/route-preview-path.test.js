// WS-6.4 / demo-teach-replay: useRoutePreview's Path-message handling must
// ignore empty /route/preview messages and keep the last non-empty preview
// (Prevention of flicker #4 at the hook level; mutation 3). previewFromPath is
// a pure, react-free function so this runs with plain node --test.
import test from 'node:test'
import assert from 'node:assert/strict'
import { previewFromPath } from '../../src/screens/routePreviewGeom.js'

const NON_EMPTY = {
  poses: [
    { pose: { position: { x: 1, y: 2 } } },
    { pose: { position: { x: 3, y: 4 } } },
  ],
}

test('keeps the last non-empty preview when an empty Path arrives', () => {
  const prev = [{ x: 1, y: 2 }, { x: 3, y: 4 }]
  assert.deepEqual(previewFromPath({ poses: [] }, prev), prev)
  assert.deepEqual(previewFromPath({}, prev), prev)
  assert.deepEqual(previewFromPath(null, prev), prev)
})

test('adopts a non-empty Path and flattens it to [{x,y}]', () => {
  const prev = [{ x: 9, y: 9 }]
  assert.deepEqual(previewFromPath(NON_EMPTY, prev), [{ x: 1, y: 2 }, { x: 3, y: 4 }])
})

test('an empty initial message stays empty (no previous to fall back to)', () => {
  assert.deepEqual(previewFromPath({ poses: [] }, []), [])
  assert.deepEqual(previewFromPath({ poses: [] }, undefined), undefined)
})