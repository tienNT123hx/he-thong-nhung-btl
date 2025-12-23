#ifndef FFT_MODULE_H
#define FFT_MODULE_H

#include <vector>
#include <cmath>

class FFTModule {
public:
    static float compute_rms(const std::vector<float>& x) {
        float sum = 0.0f;
        for (float v : x) sum += v * v;
        return std::sqrt(sum / x.size());
    }

    static float compute_peak(const std::vector<float>& x) {
        float peak = 0.0f;
        for (float v : x)
            if (std::abs(v) > peak) peak = std::abs(v);
        return peak;
    }
};

#endif
