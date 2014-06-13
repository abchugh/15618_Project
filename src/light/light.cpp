
#include "light.hpp"
#include "integrator/surface.hpp"

namespace _462 {
void Light::initialize_sampler(Sampler *sampler_ptr, LightOffset &offset) const {
		offset.num = num_samples;
		offset.offset_1d = -1;
		offset.offset_2d = sampler_ptr->add2D(num_samples);
	}
}
