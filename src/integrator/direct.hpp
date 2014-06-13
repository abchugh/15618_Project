
#ifndef _462_INTEGRATOR_DIRECT_HPP_
#define _462_INTEGRATOR_DIRECT_HPP_

#include "surface.hpp"

namespace _462 {

class Scene;

class DirectLightingIntegrator : public SurfaceIntegrator{
public:
	DirectLightingIntegrator(const Scene *scene_ptr, LightSamplingMode mode = SAMPLE_ALL, uint32_t md = 5);
	~DirectLightingIntegrator() {
		delete light_offsets;
		delete bsdf_offsets;
	}

	Color3 li(const Scene *scene_ptr, const Ray &ray, const hitRecord &record, const Sample* sample_ptr,
				Random462 &rng);
	void initialize_sampler(const Scene *scene_ptr, Sampler *sampler_ptr);
private:
	LightSamplingMode sample_mode;
	uint32_t max_depth;
	LightOffset *light_offsets;
	BSDFOffset *bsdf_offsets;

	DirectLightingIntegrator(const DirectLightingIntegrator&);
    DirectLightingIntegrator& operator=(const DirectLightingIntegrator&);
};

} /* _462 */

#endif /* _462_INTEGRATOR_DIRECT_HPP_ */

