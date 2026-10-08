// ros/imuCalib.js — BNO055 校正ステータスの分解（機体側 check_core.unpack_calib と同じ bit 配分）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { unpackImuCalib } from '../../src/ros/imuCalib.js'

test('各 2 bit が sys/gyro/accel/mag に分かれる', () => {
  assert.deepEqual(unpackImuCalib(0b11100100), { sys: 3, gyro: 2, accel: 1, mag: 0 })
  assert.deepEqual(unpackImuCalib(0xff), { sys: 3, gyro: 3, accel: 3, mag: 3 })
  assert.deepEqual(unpackImuCalib(0), { sys: 0, gyro: 0, accel: 0, mag: 0 })
})

test('数でない値は null', () => {
  assert.equal(unpackImuCalib(null), null)
  assert.equal(unpackImuCalib(undefined), null)
  assert.equal(unpackImuCalib(NaN), null)
})
