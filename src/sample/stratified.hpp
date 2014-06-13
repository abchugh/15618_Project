
#ifndef _462_STRATIFIEDSAMPLER_HPP_
#define _462_STRATIFIEDSAMPLER_HPP_

#include "sampler.hpp"
#include "math/random462.hpp"

namespace _462 {

class StratifiedSampler : public Sampler {
public:
    StratifiedSampler(uint32_t width, uint32_t height, uint32_t p_width_x, uint32_t p_width_y,
		  uint32_t pixel_num_sample) :
	Sampler(width, height, p_width_x, p_width_y, pixel_num_sample) {
<<<<<<< HEAD
	current_packet = 0;
	wanted_packet_num = (width + p_width_x - 1) / p_width_x * 
	    ((height + p_width_y - 1) / p_width_y);
=======
		current_packet = 0;
		wanted_packet_num = (width + p_width_x - 1) / p_width_x * 
			((height + p_width_y - 1) / p_width_y);
		roundSize(pixel_num_sample);
>>>>>>> 9612b61bec1ef47036192ed0a454ddc67da31fc3
    }

    ~StratifiedSampler() { }

<<<<<<< HEAD
    Sample *getPacketSamples(uint32_t &x, uint32_t &y);

private:
    uint32_t current_packet, wanted_packet_num;
=======
    Sample *getPacketSamples(uint32_t &x, uint32_t &y, Random462 &rng);

private:
	void roundSize(uint32_t n);

    uint32_t current_packet, wanted_packet_num;
	uint32_t pixel_num_x, pixel_num_y;
>>>>>>> 9612b61bec1ef47036192ed0a454ddc67da31fc3
    Random462 rng;
};

}

#endif
