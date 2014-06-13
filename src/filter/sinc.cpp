
#include "sinc.hpp"
#include <complex>

namespace _462 {
    
float sinc1D(float v, float tau) {
    float x = (v > 0) ? v : -v;
    if (x < 1e-5) return 1.f;
    if (x > 1.)   return 0.f;
    x *= PI;
    float sinc = std::sin(x) / x;
    float lanczos = std::sin(x * tau) / (x * tau);
    return sinc * lanczos;
}


float SincFilter::evaluate(float x, float y) {
    float shift_x = (x - width_x) / (width_x);
    float shift_y = (y - width_y) / (width_y);
    float g_x = sinc1D(shift_x, tau);
    float g_y = sinc1D(shift_y, tau);

    printf("%f - %f\n", width_x, width_y);
    printf("%f, %f >> >> %f, %f: %f, %f\n", x, y, shift_x, shift_y, g_x, g_y);

    return g_x * g_y;
}


}
