#include "motor.h"

namespace Motor {

void init() {
    pinMode(MOT_RIGHT_DIR, OUTPUT);
    pinMode(MOT_LEFT_DIR,  OUTPUT);
    digitalWrite(MOT_RIGHT_DIR, LOW);
    digitalWrite(MOT_LEFT_DIR,  LOW);

    ledcSetup(PWM_CH_RIGHT, PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcSetup(PWM_CH_LEFT,  PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcAttachPin(MOT_RIGHT_PWM, PWM_CH_RIGHT);
    ledcAttachPin(MOT_LEFT_PWM,  PWM_CH_LEFT);
    ledcWrite(PWM_CH_RIGHT, 0);
    ledcWrite(PWM_CH_LEFT,  0);
}

static void driveChannel(int dirPin, int pwmCh, bool fwdLevel, float speed) {
    speed = constrain(speed, -1.0f, 1.0f);
    bool fwd = (speed >= 0.0f);
    int  pwm = (int)(fabsf(speed) * 255.0f);
    pwm = constrain(pwm, 0, 255);

    digitalWrite(dirPin, (fwd == fwdLevel) ? HIGH : LOW);
    ledcWrite(pwmCh, pwm);
}

void setRight(float speed) {
    driveChannel(MOT_RIGHT_DIR, PWM_CH_RIGHT, (bool)MOT_RIGHT_FWD, speed);
}

void setLeft(float speed) {
    driveChannel(MOT_LEFT_DIR, PWM_CH_LEFT, (bool)MOT_LEFT_FWD, speed);
}

void stopAll() {
    ledcWrite(PWM_CH_RIGHT, 0);
    ledcWrite(PWM_CH_LEFT,  0);
}

} // namespace Motor
