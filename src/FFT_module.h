#ifndef FFT_MODULE_H
#define FFT_MODULE_H

#include <Arduino.h>
#include <arduinoFFT.h>
#include "MPU_sensor.h"

// ================== CẤU HÌNH FFT ==================
#define SAMPLE_SIZE       256
#define SAMPLE_RATE       1000     // Hz
#define SAMPLE_PERIOD_US  (1000000 / SAMPLE_RATE)
#define ACCEL_SCALE       16384.0  // ±2g scale

arduinoFFT FFT = arduinoFFT();

double vReal[SAMPLE_SIZE];
double vImag[SAMPLE_SIZE];

double vRealX[SAMPLE_SIZE];
double vRealY[SAMPLE_SIZE];
double vRealZ[SAMPLE_SIZE];

double vImagX[SAMPLE_SIZE];
double vImagY[SAMPLE_SIZE];
double vImagZ[SAMPLE_SIZE];

// ================== LẤY MẪU FFT ==================
void collectSamples() {
    unsigned long timer;
    int16_t ax, ay, az;

    for (int i = 0; i < SAMPLE_SIZE; i++) {
        timer = micros();

        readAccel(ax, ay, az);

        // Chuyển sang đơn vị g
        double ax_g = (double)ax / ACCEL_SCALE;
        double ay_g = (double)ay / ACCEL_SCALE;
        double az_g = (double)az / ACCEL_SCALE;

        // Lấy độ rung vector: sqrt(ax² + ay² + az²)
        double magnitude = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

        vReal[i] = magnitude;
        vImag[i] = 0;
        
        vRealX[i] = (double)ax / ACCEL_SCALE;
        vRealY[i] = (double)ay / ACCEL_SCALE;
        vRealZ[i] = (double)az / ACCEL_SCALE;
        vImagX[i] = 0;
        vImagY[i] = 0;  
        vImagZ[i] = 0;
        // Chờ đúng chu kỳ lấy mẫu
        while (micros() - timer < SAMPLE_PERIOD_US);
    }
}

// ================== TÍNH RMS ==================
double computeRMS() {
    double sum = 0;
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        sum += vReal[i] * vReal[i];
    }
    return sqrt(sum / SAMPLE_SIZE);
}

// ================== TÍNH FFT ==================
double computeFFT_Frequency() {
    FFT.Windowing(vReal, SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLE_SIZE, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLE_SIZE);

    double peak = 0;
    int peakIndex = 0;

    for (int i = 1; i < SAMPLE_SIZE / 2; i++) {
        if (vReal[i] > peak) {
            peak = vReal[i];
            peakIndex = i;
        }
    }

    return peakIndex * (SAMPLE_RATE / SAMPLE_SIZE);
}

#endif
