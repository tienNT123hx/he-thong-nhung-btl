#ifndef MPU6050_MODEL_H
#define MPU6050_MODEL_H

#include <systemc>
#include <cmath>
#include "communication/data_types.h"

using namespace sc_core;

SC_MODULE(MPU6050_Model) {
    sc_in<float> motor_speed;          // rad/s
    sc_fifo_out<AccelData> out;

    void sense() {
        while (true) {
            AccelData d;

            // Gia tốc rung tỉ lệ với speed (mô hình đơn giản)
            float noise = 0.02f * ((rand() % 100) / 100.0f - 0.5f);
            d.ax = 0.1f * motor_speed.read() + noise;
            d.ay = 0.0f;
            d.az = 1.0f;

            out.write(d);
            wait(10, SC_MS);
        }
    }

    SC_CTOR(MPU6050_Model) {
        SC_THREAD(sense);
    }
};

#endif
