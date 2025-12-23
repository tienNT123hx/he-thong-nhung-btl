#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// algorithms/pid_controller.h
class PIDController {
    float kp, ki, kd;
    float prev_err = 0, integ = 0;

public:
    PIDController(float p, float i, float d)
        : kp(p), ki(i), kd(d) {}

    float update(float sp, float pv, float dt) {
        float err = sp - pv;
        integ += err * dt;
        float deriv = (err - prev_err) / dt;
        prev_err = err;

        float out = kp*err + ki*integ + kd*deriv;

        // clamp PWM
        if (out > 100) out = 100;
        if (out < -100) out = -100;

        return out;
    }
};
#endif
