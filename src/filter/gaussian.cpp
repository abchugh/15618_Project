
#include "gaussian.hpp"

namespace _462 {

float GaussianFilter::evaluate(float x, float y) {
    float shift_x = x - width_x;
    float shift_y = y - width_y;
    float g_x = std::max(0.f, expf(-alpha * shift_x * shift_x) - expX);
    float g_y = std::max(0.f, expf(-alpha * shift_y * shift_y) - expY);

    printf("%f - %f\n", width_x, width_y);
    printf("%f, %f >> %f, %f: %f, %f\n", x, y, shift_x, shift_y, g_x, g_y);

    return g_x * g_y;
}

}
