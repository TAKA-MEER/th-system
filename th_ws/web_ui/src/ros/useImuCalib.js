// ros/useImuCalib.js — S-30 IMU の「校正状態（sys/gyro/accel/mag）のリアルタイム表示」
// （Spec-checks.md §2.4 #3-3）。/esp32/imu_calib_status を購読して分解する。
// TEST_MODE: window.__thTestImuCalib（UInt8 の値）でシード、window.__thSetTestImuCalib で更新。
import { useEffect, useState } from 'react'
import { unpackImuCalib } from './imuCalib.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined
const IMU_CALIB_TOPIC = '/esp32/imu_calib_status'
const IMU_CALIB_MSG = 'std_msgs/UInt8'
const THROTTLE_MS = 200

export function useImuCalib(ros) {
  const [raw, setRaw] = useState(TEST_MODE ? (window.__thTestImuCalib ?? null) : null)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestImuCalib = (v) => setRaw(v)
    return () => { delete window.__thSetTestImuCalib }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) return undefined
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    const topic = new ROSLIB.Topic({
      ros, name: IMU_CALIB_TOPIC, messageType: IMU_CALIB_MSG,
      queue_length: 1, throttle_rate: THROTTLE_MS,
    })
    topic.subscribe((msg) => setRaw(typeof msg?.data === 'number' ? msg.data : null))
    return () => topic.unsubscribe()
  }, [ros])

  return unpackImuCalib(raw)
}
