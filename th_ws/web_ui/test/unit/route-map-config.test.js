// WS-9F / WS-9G / demo-teach-replay: useRouteMap's map Topic (now /route/map_view,
// published by map_downsampler — WS-9G) must use rosbridge CBOR compression so the
// huge OccupancyGrid arrives as raw bytes (~1 byte/cell) instead of a JSON integer
// array (~3 bytes/cell), and to avoid the main-thread stall from parsing giant
// JSON. The topic options live in the react-free routeMapTopicConfig module so this
// runs with plain node --test (mutation 3: dropping compression:'cbor' or reverting
// the topic back to /map goes red).
import test from 'node:test'
import assert from 'node:assert/strict'
import { routeMapTopicConfig, ROUTE_MAP_TOPIC, ROUTE_MAP_MSG } from '../../src/ros/routeMapTopicConfig.js'

test('useRouteMap subscribes to the downsampled /route/map_view topic', () => {
  const cfg = routeMapTopicConfig({ ros: 'fake' })
  assert.equal(cfg.name, '/route/map_view')
  assert.equal(ROUTE_MAP_TOPIC, '/route/map_view')
})

test('useRouteMap topic enables cbor compression', () => {
  const cfg = routeMapTopicConfig({ ros: 'fake' })
  assert.equal(cfg.compression, 'cbor')
})

test('topic keeps messageType name and default throttling', () => {
  const cfg = routeMapTopicConfig({ ros: 'fake' })
  assert.equal(cfg.name, ROUTE_MAP_TOPIC)
  assert.equal(cfg.messageType, ROUTE_MAP_MSG)
  // 既存の throttle_rate / queue_length は変えない（WS-9F）
  assert.equal(cfg.throttle_rate, 2000)
  assert.equal(cfg.queue_length, 1)
  assert.ok(cfg.ros, 'ros ハンドルを引き継ぐ')
})
