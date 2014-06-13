
#ifndef _462_FILM_HPP_
#define _462_FILM_HPP_

#include "sample/sampler.hpp"
#include "math/color.hpp"

namespace _462 {

class Filter;

class Film {
public:
    Film(uint32_t width, uint32_t height,
	 uint32_t x_start, uint32_t x_end,
	 uint32_t y_start, uint32_t y_end);
    ~Film();

    void setFilter(Filter *filter_ptr);

    void addSample(const Sample &sample, const Color3 color);
    void splat(const Sample &sample, const Color3 color);

    void getSampleSize(uint32_t &width, uint32_t &height);
    void getPixelExtent(uint32_t &x_start, uint32_t &x_end,
			uint32_t &y_start, uint32_t &y_end);

    void output(unsigned char* buffer);

    Filter *filter_ptr;

private:
    uint32_t width, height;
    uint32_t x_start, x_end, y_start, y_end;
    Color3 *colors;
    float *sum_weights;
};

}

#endif
