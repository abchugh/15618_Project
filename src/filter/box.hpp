
#ifndef _462_BOXFILTER_HPP_
#define _462_BOXFILTER_HPP_

#include "filter.hpp"

namespace _462 {

class BoxFilter : public Filter {
public:
    BoxFilter(float width_x, float width_y) :
	Filter(width_x, width_y) { }
    ~BoxFilter() {
	delete filter_table;
    }

    float evaluate(float x, float y);

};

}

#endif
