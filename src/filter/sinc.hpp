
#ifndef _462_SINCFILTER_HPP_
#define _462_SINCFILTER_HPP_

#include <complex>
#include "filter.hpp"

namespace _462 {

class SincFilter : public Filter {
public:
    SincFilter(float width_x, float width_y, float t) :
	Filter(width_x, width_y), 
	tau(t) { }

    ~SincFilter() {
	delete filter_table;
    }

    float evaluate(float x, float y);

    float tau;
};

}

#endif
