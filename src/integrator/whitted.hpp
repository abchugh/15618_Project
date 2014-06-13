
#ifndef _462_INTEGRATOR_WHITTED_HPP_
#define _462_INTEGRATOR_WHITTED_HPP_

#include "surface.hpp"

namespace _462 {

class WhittedIntegrator : public SurfaceIntegrator{
public:
	WhittedIntegrator(uint32_t depth) : max_depth(depth) { }
	Color3 li(const Scene *scene_ptr, const Ray &ray, const hitRecord &record, const Sample* sample_ptr,
				Random462 &rng);
	void initialize_sampler(const Scene *scene_ptr, Sampler *sampler_ptr);
private:
	Color3 SpecularReflect(const Scene *scene_ptr,
											Vector3 &wo, const hitRecord &record,
											const Sample* sample_ptr, Random462 &rng);
	Color3 SpecularTransmit(const Scene *scene_ptr,
											Vector3 &wo, const hitRecord &record,
											const Sample* sample_ptr, Random462 &rng);
	Color3 SpecularTrace(const Scene *scene_ptr, Vector3 &wo, const hitRecord &record,
									const Sample* sample_ptr, Random462 &rng, const BxDFType flags);
	uint32_t max_depth;
};

} /* _462 */

#endif /* _462_INTEGRATOR_WHITTED_HPP_ */

