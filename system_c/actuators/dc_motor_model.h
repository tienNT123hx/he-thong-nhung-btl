#ifndef DC_MOTOR_MODEL_H
#define DC_MOTOR_MODEL_H

#include <systemc>
using namespace sc_core;

SC_MODULE(DCMotor_Model) {
    sc_in<float>  pwm;
    sc_out<float> speed;   // rad/s

    float omega = 0.0f;

    void dynamics() {
        while (true) {
            float u = pwm.read();
            omega += 0.1f * u - 0.02f * omega;  // quán tính + ma sát
            speed.write(omega);
            wait(10, SC_MS);
        }
    }

    SC_CTOR(DCMotor_Model) {
        SC_THREAD(dynamics);
    }
};

#endif
