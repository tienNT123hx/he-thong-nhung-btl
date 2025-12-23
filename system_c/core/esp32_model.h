#ifndef ESP32_MODEL_H
#define ESP32_MODEL_H

#include <systemc>
#include <vector>
#include "../communication/i2c_bus.h"
#include "../algorithms/fft_module.h"
#include "../algorithms/pid_controllers.h"
#include "../utils/logger.h"

using namespace sc_core;

SC_MODULE(ESP32_Model) {
    sc_fifo_in<AccelData> in;
    sc_in<float> angle;

    PIDController pid;
    std::vector<float> buffer;

    const int N = 32;
    float setpoint = 30.0f;

    void run() {
        while (true) {
            AccelData d = in.read();  
            buffer.push_back(d.ax);

            if (buffer.size() >= N) {
                float rms  = FFTModule::compute_rms(buffer);
                float peak = FFTModule::compute_peak(buffer);

                float pid_out = pid.update(setpoint, angle.read(), 0.01f);
                Logger::log(pid_out, rms, peak);

                buffer.clear();
            }

            wait(10, SC_MS);
        }
    }

    SC_CTOR(ESP32_Model)
        : pid(5.0f, 0.1f, 1.0f)
    {
        SC_THREAD(run);
    }
};

#endif
