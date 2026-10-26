"""Host test of the real ESP32 PID header and ramp integration gate."""

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
TMP = ROOT / ".briefs" / "tmp"


def test_esp32_pid_ramp_windup_simulation():
    TMP.mkdir(parents=True, exist_ok=True)
    arduino = TMP / "Arduino.h"
    source = TMP / "pid_ramp_windup_harness.cpp"
    binary = TMP / "pid_ramp_windup_harness"
    arduino.write_text(
        """#pragma once
#include <algorithm>
#include <cmath>
using std::max;
using std::min;
template <typename T> T constrain(T value, T low, T high) {
    return min(max(value, low), high);
}
"""
    )
    source.write_text(
        r'''#include <cmath>
#include <cstdio>
#include "pid.h"

struct Result { float peak; float settled; float rampI; float stop; };

Result run(bool gate) {
    constexpr float dt = 0.1f;
    PID pid(120.0f, 80.0f, 1.0f, -200.0f, 200.0f, 100.0f, 280.0f, 300.0f);
    float target = 0.0f, speed = 0.0f, delayedPwm = 0.0f;
    float peak = 0.0f, rampI = 0.0f;
    for (int i = 0; i < 180; ++i) {
        const float before = target;
        if (target < 0.55f) target = std::min(0.55f, target + 0.4f * dt);
        const bool allowI = !gate || PID::targetIsSettled(target - before, dt);
        const float pwm = pid.compute(target, speed, dt, allowI);
        if (target < 0.55f) rampI = std::max(rampI, std::fabs(pid.integralTerm()));
        // One 100 ms command delay followed by a first-order motor response.
        speed += (delayedPwm * 1.08f / 280.0f - speed) * (dt / 0.22f);
        delayedPwm = pwm;
        if (target >= 0.55f && i < 100) peak = std::max(peak, speed);
    }
    const float settled = speed;
    const float stop = pid.compute(0.0f, speed, dt, true);
    return {peak, settled, rampI, stop};
}

int main() {
    const Result old = run(false), fixed = run(true);
    std::printf("old_peak=%.6f new_peak=%.6f settled=%.6f ramp_i_delta=%.6f stop=%.1f\n",
                old.peak, fixed.peak, fixed.settled, fixed.rampI, fixed.stop);
    if (PID::targetIsSettled(0.04f, 0.1f) ||
        !PID::targetIsSettled(0.0f, 0.1f) ||
        std::fabs(fixed.stop) > 0.001f ||
        !(fixed.peak < old.peak - 0.01f) ||
        !(fixed.peak - 0.55f < 0.04f) ||
        std::fabs(fixed.settled - 0.55f) > 0.02f ||
        fixed.rampI > 0.01f) return 1;

    PID saturated(0.0f, 80.0f, 0.0f, 0.0f, 1.0f, 100.0f);
    PID fresh(0.0f, 80.0f, 0.0f, 0.0f, 1.0f, 100.0f);
    for (int i = 0; i < 20; ++i) saturated.compute(1.0f, 0.0f, 0.1f);
    const float afterSaturation = saturated.compute(1.0f, 1.0f, 0.1f);
    const float withoutSaturation = fresh.compute(1.0f, 1.0f, 0.1f);
    if (std::fabs(afterSaturation - withoutSaturation) > 0.001f) return 2;
    return 0;
}
'''
    )
    subprocess.run(
        [
            "g++",
            "-std=c++11",
            "-I",
            str(TMP),
            "-I",
            str(ROOT / "th_ws" / "esp32" / "src"),
            str(source),
            "-o",
            str(binary),
        ],
        check=True,
    )
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    print(result.stdout.strip())
