
#ifndef _462_HALTONSAMPLER_HPP_
#define _462_HALTONSAMPLER_HPP_

#include "sampler.hpp"

namespace _462 {

class HaltonSampler : public Sampler {
public:
    HaltonSampler(uint32_t width, uint32_t height, uint32_t p_width_x, uint32_t p_width_y,
		  uint32_t pixel_num_sample) :
	Sampler(width, height, p_width_x, p_width_y, pixel_num_sample) {
	current_packet = 0;
	wanted_packet_num = width / p_width_x * height / p_width_y;
    }

    ~HaltonSampler() { }

<<<<<<< HEAD
    Sample *getPacketSamples(uint32_t &x, uint32_t &y);
=======
    Sample *getPacketSamples(uint32_t &x, uint32_t &y, Random462 &rng);
>>>>>>> 9612b61bec1ef47036192ed0a454ddc67da31fc3

private:
    uint32_t current_packet, wanted_packet_num;
};

}

#endif
