
#ifndef _462_MITCHELLFILTER_HPP_
#define _462_MITCHELLFILTER_HPP_

#include <complex>
#include "filter.hpp"

namespace _462 {

class MitchellFilter : public Filter {
public:
    MitchellFilter(float width_x, float width_y, float b, float c) :
	Filter(width_x, width_y), 
	b(b), c(c) { } 

    ~MitchellFilter() {
	delete filter_table;
    }

    float evaluate(float x, float y);

    float b, c;
};

}

#endif
