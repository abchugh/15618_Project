
#include "filter.hpp"

namespace _462 {

void Filter::populate() {
    if (filter_table != NULL)
	return ;
    filter_table = new float[table_edge * table_edge];
    
    float *flt = filter_table;
    for (uint32_t y = 0; y < table_edge; y++) {
	for (uint32_t x = 0; x < table_edge; x++) {
	    float table_x = (x + 0.5) * (2 * width_x) / table_edge;
	    float table_y = (y + 0.5f) * (2 * width_y) / table_edge;
	    *flt++ = evaluate(table_x, table_y);
	}
    }
}

}
