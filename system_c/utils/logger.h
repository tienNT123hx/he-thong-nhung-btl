#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>

class Logger {
public:
    static void log(float pid, float rms, float peak) {
        std::cout
            << "PID: " << pid
            << " | RMS: " << rms
            << " | Peak: " << peak
            << std::endl;
    }
};

#endif
