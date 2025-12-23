#include <systemc>
#include <vector>

#include "sensors/mpu6050_model.h"
#include "communication/i2c_bus.h"
#include "algorithms/fft_module.h"
#include "algorithms/pid_controllers.h"
#include "actuators/l298n_model.h"
#include "actuators/dc_motor_model.h"
#include "actuators/encoder_model.h"
#include "utils/logger.h"

using namespace sc_core;

int sc_main(int argc, char* argv[]) {

    // signals
    sc_signal<float> pwm_sig, motor_cmd, motor_speed, angle_sig;

    // modules
    I2CBus bus("bus");
    MPU6050_Model mpu("mpu");
    L298N_Model l298("l298");
    DCMotor_Model motor("motor");
    Encoder_Model encoder("encoder");

    // wiring
    mpu.out(bus.fifo);

    l298.pwm_in(pwm_sig);
    l298.motor_cmd(motor_cmd);

    motor.pwm(motor_cmd);
    motor.speed(motor_speed);

    encoder.motor_speed(motor_speed);
    encoder.angle(angle_sig);
    mpu.motor_speed(motor_speed);


    PIDController pid(5.0f, 0.1f, 1.0f);

    std::vector<float> buffer;
    const int N = 32;
    float setpoint = 30.0f;

    for (int i = 0; i < 300; i++) {
        sc_start(10, SC_MS);

        AccelData d = bus.fifo.read();
        buffer.push_back(d.ax);

        float pid_out = pid.update(setpoint, angle_sig.read(), 0.01f);
        pwm_sig.write(pid_out * 0.01f);   // scale PWM

        if (buffer.size() >= N) {
            float rms  = FFTModule::compute_rms(buffer);
            float peak = FFTModule::compute_peak(buffer);

            Logger::log(pid_out, rms, peak);
            buffer.clear();
        }
    }

    return 0;
}
