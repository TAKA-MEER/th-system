// DetailedDesign-webui.md §9-10 / §9.1 (U-5): every name in ros/topics.js
// must exist in names.json's `endpoints` (topics ∪ services) -- the
// machine-readable dictionary generated from DetailedDesign-names.md by
// docs/plan/detailed/tools/export_names.py. names.json is a derived file
// and must not be hand-edited; this test only reads it.
import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { TOPICS, SERVICES } from '../../src/ros/topics.js'

const names = JSON.parse(
  readFileSync(fileURLToPath(new URL('../../src/ros/names.json', import.meta.url)), 'utf8'),
)

test('every TOPICS entry exists in names.json endpoints', () => {
  const missing = Object.entries(TOPICS).filter(([, name]) => !names.endpoints.includes(name))
  assert.deepEqual(missing, [], `topics missing from the name dictionary: ${JSON.stringify(missing)}`)
})

test('every SERVICES entry exists in names.json endpoints', () => {
  const missing = Object.entries(SERVICES).filter(([, name]) => !names.endpoints.includes(name))
  assert.deepEqual(missing, [], `services missing from the name dictionary: ${JSON.stringify(missing)}`)
})

test('TOPICS and SERVICES only contain rosbridge-style "/" names', () => {
  for (const name of [...Object.values(TOPICS), ...Object.values(SERVICES)]) {
    assert.match(name, /^\/[a-z0-9_/]+$/, `"${name}" doesn't look like a topic/service name`)
  }
})
