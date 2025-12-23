#ifndef L298N_MODEL_H
#define L298N_MODEL_H

#include <systemc>
using namespace sc_core;

SC_MODULE(L298N_Model) {
    sc_in<float>  pwm_in;      // [-1 .. 1]
    sc_out<float> motor_cmd;   // điện áp giả lập

    void process() {
        while (true) {
            motor_cmd.write(pwm_in.read());
            wait(1, SC_MS);
        }
    }

    SC_CTOR(L298N_Model) {
        SC_THREAD(process);
    }
};

#endif
