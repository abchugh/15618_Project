
#include "mitchell.hpp"

namespace _462 {
    
float mitchell1D(float v, float B, float C) {
    float x = 2.f * v;
    x = (x > 0) ? x : -x;

    if (x > 1.f)
	return ((-B - 6*C) * x*x*x + (6*B + 30*C) * x*x +
		(-12*B - 48*C) * x + (8*B + 24*C)) * (1.f/6.f);
    else
	return ((12 - 9*B - 6*C) * x*x*x +
		(-18 + 12*B + 6*C) * x*x +
		(6 - 2*B)) * (1.f/6.f);
}


float MitchellFilter::evaluate(float x, float y) {
    float shift_x = (x - width_x) / (2 * width_x);
    float shift_y = (y - width_y) / (2 * width_y);
    float g_x = mitchell1D(shift_x, b, c);
    float g_y = mitchell1D(shift_y, b, c);

    printf("%f - %f\n", width_x, width_y);
    printf("%f, %f >>> %f, %f: %f, %f\n", x, y, shift_x, shift_y, g_x, g_y);

    return g_x * g_y;
}


}
