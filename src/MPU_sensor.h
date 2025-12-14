#ifndef MPU_SENSOR_H
#define MPU_SENSOR_H

#include <Wire.h>

// Địa chỉ của MPU6050
#define MPU_ADDR 0x68
#define ACCEL_SCALE 16384.0

// Khởi tạo MPU6050
void MPU6050_Init() {
    // Đánh thức MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // POWER_MGMT_1 register
    Wire.write(0);
    Wire.endTransmission();
}

// Đọc dữ liệu gia tốc từ MPU6050
void readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
}

// Hiển thị dữ liệu gia tốc lên Serial Monitor
void printAccel(int16_t ax, int16_t ay, int16_t az) {
    double ax_g = (double)ax / ACCEL_SCALE;
    double ay_g = (double)ay / ACCEL_SCALE;
    double az_g = (double)az / ACCEL_SCALE;

    Serial.print("AX: "); Serial.print(ax_g, 3);
    Serial.print(" | AY: "); Serial.print(ay_g, 3);
    Serial.print(" | AZ: "); Serial.println(az_g, 3);
}

#endif // MPU_SENSOR_H
