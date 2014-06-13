
#include "halton.hpp"

namespace _462 {

<<<<<<< HEAD
Sample *HaltonSampler::getPacketSamples(uint32_t &x, uint32_t &y) {
    uint32_t des_packet = __sync_fetch_and_add(&current_packet, 1);
=======
Sample *HaltonSampler::getPacketSamples(uint32_t &x, uint32_t &y, Random462 &rng) {
    uint32_t des_packet;
#ifdef _WINDOWS
#pragma omp critical
	{
		des_packet = current_packet++;
	}
#else
	des_packet = __sync_fetch_and_add(&current, 1);
#endif

>>>>>>> 9612b61bec1ef47036192ed0a454ddc67da31fc3
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
    float dx = float(1)/width;
    float dy = float(1)/height;

    uint32_t count = 0;

    for (int j = y_start; j < y_end; j++) {
<<<<<<< HEAD
	for (int i = x_start; i < x_end; i++) {
	    int position = j * width + i;
	    for (int k = 0; k < pixel_num_sample; k++) {
		int current_num = position * pixel_num_sample + k;
		float u = radicalInverse(current_num, 2);
		float v = radicalInverse(current_num, 3);
		result[count].x = u + i;
		result[count++].y = v + j;
	    }
	}
=======
		for (int i = x_start; i < x_end; i++) {
			int position = j * width + i;
			for (int k = 0; k < pixel_num_sample; k++) {
				int current_num = position * pixel_num_sample + k;
				float u = radicalInverse(current_num, 2);
				float v = radicalInverse(current_num, 3);
				float* xy_result = (float*)result;
				xy_result[count * sampleset.sample_size] = u + i;
				xy_result[count * sampleset.sample_size + 1] = v + j;

				uint32_t one_offset = 0;
				for (uint32_t i = 0; i < sampleset.oneD_num.size(); i++) {
					latin_hypercube((float*)result + count * sampleset.sample_size + 2 + one_offset, 1, sampleset.oneD_num[i], rng);
					one_offset += sampleset.oneD_num[i];
				}
				uint32_t two_offset = 0;
				for (uint32_t i = 1; i < sampleset.twoD_num.size(); i++) {
					latin_hypercube((float*)result + count * sampleset.sample_size + 2 + one_offset + 
						two_offset, 2, sampleset.twoD_num[i], rng);
					two_offset += 2 * sampleset.twoD_num[i];
				}
				count++;
			}
		}
>>>>>>> 9612b61bec1ef47036192ed0a454ddc67da31fc3
    }

    x = x_start;
    y = y_start;

    return result;
}

}
