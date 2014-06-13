
#ifndef _462_GAUSSIANFILTER_HPP_
#define _462_GAUSSIANFILTER_HPP_

#include <complex>
#include "filter.hpp"

namespace _462 {

class GaussianFilter : public Filter {
public:
    GaussianFilter(float width_x, float width_y, float alpha) :
	Filter(width_x, width_y), 
	alpha(alpha),
	expX(std::exp(-alpha * (width_x) * (width_x))), 
	expY(std::exp(-alpha * (width_y) * (width_y))) { }
    ~GaussianFilter() {
	delete filter_table;
    }

    float evaluate(float x, float y);

    float alpha;
    float expX, expY;
};

}

#endif
