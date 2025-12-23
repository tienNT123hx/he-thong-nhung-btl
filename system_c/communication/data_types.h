#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <iostream>

struct AccelData {
    float ax;
    float ay;
    float az;
};

// Overload operator<< for SystemC sc_fifo print
inline std::ostream& operator<<(std::ostream& os, const AccelData& d) {
    os << "{ax=" << d.ax
       << ", ay=" << d.ay
       << ", az=" << d.az << "}";
    return os;
}



struct EncoderData {
    float angle;   // degrees
    float speed;   // deg/s
};



struct ControlCmd {
    float pwm;        // -1.0 → 1.0
};

struct FFTData {
    float magnitude;
    float frequency;
};
struct PIDParams {
    float kp;
    float ki;
    float kd;
};

#endif