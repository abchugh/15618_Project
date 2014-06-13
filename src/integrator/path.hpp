
#ifndef _462_INTEGRATOR_PATH_HPP_
#define _462_INTEGRATOR_PATH_HPP_

#include "surface.hpp"

namespace _462 {

class Scene;

class PathIntegrator : public SurfaceIntegrator{
public:
	PathIntegrator(const Scene *scene_ptr, uint32_t sd = 3, uint32_t md = 5, uint32_t npp = 1);
	~PathIntegrator() {
		delete light_offsets;
		delete bsdf_offsets;
	}

	Color3 li(const Scene *scene_ptr, const Ray &ray, const hitRecord &record, const Sample* sample_ptr,
				Random462 &rng);
	void initialize_sampler(const Scene *scene_ptr, Sampler *sampler_ptr);
private:
	uint32_t sample_depth;
	uint32_t max_depth;
	uint32_t num_per_path;
	LightOffset *light_offsets;
	BSDFOffset *bsdf_offsets;
	BSDFOffset *path_offsets;

	PathIntegrator(const PathIntegrator&);
    PathIntegrator& operator=(const PathIntegrator&);
};

} /* _462 */

#endif /* _462_INTEGRATOR_PATH_HPP_ */

