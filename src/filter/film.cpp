
#include <cstring>
#include <omp.h>
#include "film.hpp"
#include "filter.hpp"
#include "scene/bvh.hpp"

namespace _462 {
Film::Film(uint32_t width, uint32_t height,
	   uint32_t x_start, uint32_t x_end,
	   uint32_t y_start, uint32_t y_end) : 
    width(width), height(height), x_start(x_start),
    x_end(x_end), y_start(y_start), y_end(y_end), filter_ptr(NULL) {
    uint32_t pixel_x = x_end - x_start;
    uint32_t pixel_y = y_end - y_start;
    colors = (Color3*)memalign(16, sizeof(real_t) * 3 * pixel_x * pixel_y);
    sum_weights = (float*)memalign(16, sizeof(float) * pixel_x * pixel_y);
    
    memset(colors, 0, sizeof(real_t) * 3 * pixel_x * pixel_y);
    memset(sum_weights, 0, sizeof(float) * pixel_x * pixel_y);
}

Film::~Film() { 
    delete filter_ptr;
    _aligned_free(colors);
    _aligned_free(sum_weights);
}

void Film::setFilter(Filter *filter_ptr) {
    this->filter_ptr = filter_ptr;
    this->filter_ptr->populate();
}

void Film::addSample(const Sample &sample, const Color3 color) {
    int x0 = (int)(sample.x - filter_ptr->width_x + 0.5f);
    int x1 = (int)(sample.x + filter_ptr->width_x - 0.5f);
    int y0 = (int)(sample.y - filter_ptr->width_y + 0.5f);
    int y1 = (int)(sample.y + filter_ptr->width_y - 0.5f);
    
    x0 = std::max((int)x_start, x0);
    x1 = std::min((int)x_end - 1, x1);
    y0 = std::max((int)y_start, y0);
    y1 = std::min((int)y_end - 1, y1);
    
    if (!((x0 <= x1) && (y0 <= y1)))
	return ;

    assert((x0 <= x1) && (y0 <= y1));

    int *ifx = new int[x1 - x0 + 1], *ify = new int[y1 - y0 + 1];

    for (int x = x0; x <= x1; x++) {
	ifx[x - x0] = (int)((sample.x - x + filter_ptr->width_x - 0.5f) / 
			    (2 * filter_ptr->width_x) * (filter_ptr->table_edge - 1));
    }

    for (int y = y0; y <= y1; y++) {
	ify[y - y0] = (int)((sample.y - y + filter_ptr->width_y - 0.5f) / 
			    (2 * filter_ptr->width_y) * (filter_ptr->table_edge - 1));
    }

    for (int y = y0; y <= y1; y++) {
		for (int x = x0; x <= x1; x++) {
			int offset = ify[y - y0] * filter_ptr->table_edge + ifx[x - x0];
			float filter_value = filter_ptr->filter_table[offset];
	    
			int pixel_offset = (y - y_start) * (x_end - x_start) + x - x_start;
			Color3 *des_color = colors + pixel_offset;
			float *sum = sum_weights + pixel_offset;

			// Assume filters all have width >= 1
			#pragma omp atomic
			des_color->r += filter_value * color.r;
			#pragma omp atomic
			des_color->g += filter_value * color.g;
			#pragma omp atomic
			des_color->b += filter_value * color.b;
			#pragma omp atomic
			*sum += filter_value;
	 
			/* known bug for test scene   
			if ((x - x_start) == 421 && (y - y_start) == 140)
			printf("%d, %d (%d) : %f %f - %f\n", x, y, offset, *sum, filter_value, color.r);
			*/
		}
    }

	delete[] ifx;
	delete[] ify;
}

void Film::splat(const Sample &sample, const Color3 color) {
    int x = (int)sample.x;
    int y = (int)sample.y;

    if (x < (int)x_start || x >= (int)x_end ||
	y < (int)y_start || y >= (int)y_end)
	return ;

    int pixel_offset = (y - y_start) * (x_end - x_start) + x - x_start;
    Color3 *des_color = colors + pixel_offset;

#pragma omp atomic
    des_color->r += color.r;
#pragma omp atomic
    des_color->g += color.g;
#pragma omp atomic
    des_color->b += color.b;

}

void Film::getSampleSize(uint32_t &width, uint32_t &height) {
    width = this->width;
    height = this->height;
}

void Film::getPixelExtent(uint32_t &x_start, uint32_t &x_end,
			  uint32_t &y_start, uint32_t &y_end) {
    x_start = this->x_start;
    x_end = this->x_end;
    y_start = this->y_start;
    y_end = this->y_end;
}

void Film::output(unsigned char* buffer) {
    uint32_t pixel_x = x_end - x_start;
    uint32_t pixel_y = y_end - y_start;

    printf("%d, %d\n", pixel_x, pixel_y);

    for (int y = 0; y < pixel_y; y++) {
		for (int x = 0; x < pixel_x; x++) {
			int offset = y * pixel_x + x;
			float dem = sum_weights[offset];
			dem = (dem < 1e-6) ? 1.f : dem;

			Color3 weighted = colors[offset] / dem;
	    
			/*
			if (x < 5 && y < 5)
			printf("#%d: (%d, %d): %f / %f, %f\n", offset, x, y, weighted.r, 
				   colors[offset].r, dem);
			*/

			weighted.to_array_gamma(&buffer[4 * offset], 2.2, 1 / 2.4);

			if (x == 400 & y == 300) {
				printf("r: %f, g: %f, b:%f\n", weighted.r, weighted.g, weighted.b);
				printf("buffer: %d, %d, %d\n", buffer[offset * 4], buffer[offset * 4 + 1], buffer[offset * 4 + 2]);
			}

			/*	
			if (buffer[4 * offset] == 47 &&
			buffer[4 * offset + 1] == 47 &&
			buffer[4 * offset + 2] == 47 )
			printf("#%d: (%d, %d): %f / %f, %f\n", offset, x, y, weighted.r, 
				   colors[offset].r, dem);
			*/
			/*
			if (x > 230 && x < 560 && y == 140 )
			printf("#%d: (%d, %d): %f / %f, %f\n", offset, x, y, weighted.r, 
				   colors[offset].r, dem);
			*/
		}
    }

    //printf("===========\n");
}
   
}
