#ifndef ENCODER_MODEL_H
#define ENCODER_MODEL_H

#include <systemc>
using namespace sc_core;

SC_MODULE(Encoder_Model) {
    sc_in<float>  motor_speed;
    sc_out<float> angle;   // degree

    float theta = 0.0f;

    void integrate() {
        while (true) {
            theta += motor_speed.read() * 0.01f;
            angle.write(theta);
            wait(10, SC_MS);
        }
    }

    SC_CTOR(Encoder_Model) {
        SC_THREAD(integrate);
    }
};

#endif
