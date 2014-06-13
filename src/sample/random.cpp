
#include "random.hpp"

namespace _462 {

Sample *RandomSampler::getPacketSamples(uint32_t &x, uint32_t &y, Random462 &rng) {
    uint32_t des_packet;
#ifdef _WINDOWS
#pragma omp critical
	{
		des_packet = current_packet++;
	}
#else
	des_packet = __sync_fetch_and_add(&current, 1);
#endif

    if (des_packet >= wanted_packet_num)
	return NULL;

    uint32_t p_x, p_y;
    p_y = des_packet / (width / p_width_x);
    p_x = des_packet - (p_y * width / p_width_x);

    uint32_t x_start, x_end, y_start, y_end;
    x_start = p_x * p_width_x;
    x_end = x_start + p_width_x;
    y_start = p_y * p_width_y;
    y_end = y_start + p_width_y;

    Sample *result = sampleset.addEmptySamples(p_width_x * p_width_y * pixel_num_sample);

    /*    
    if (des_packet < 2) {
	printf("addr: %u; size: %d: \n", result, sizeof(Sample));
	printf("%d %d %d\n", sampleset.sample_size, sampleset.sample_capacity, sampleset.twoD_num[0]);
	for (int i = 0; i < p_width_x * p_width_y * pixel_num_sample; i++) {
	    printf("%f, %f\n", result[i].x, result[i].y);
	    if ((1 + i) % pixel_num_sample == 0)
		printf("*\n");
	}
	printf("====================\n");
    }
    */

    int count = 0;
    for (int j = y_start; j < y_end; j++) {
		for (int i = x_start; i < x_end; i++) {
			for (int k = 0; k < pixel_num_sample; k++) {
				result[count].x = float(i)+ rng.random();
				result[count++].y = float(j)+ rng.random();
			}
		}
    }

    /*
    if (des_packet < 2) {
	for (int i = 0; i < p_width_x * p_width_y * pixel_num_sample; i++) {
	    printf("%f, %f\n", result[i].x, result[i].y);
	    if ((1 + i) % pixel_num_sample == 0)
		printf("*\n");
	}
	printf("====================\n");
    }
    */
    x = x_start;
    y = y_start;

    return result;
}

}
