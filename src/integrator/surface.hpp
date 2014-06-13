
#ifndef _462_INTEGRATOR_SURFACE_HPP_
#define _462_INTEGRATOR_SURFACE_HPP_

#include "math/color.hpp"
#include "math/vector.hpp"
#include "math/camera.hpp"
#include "sample/sampler.hpp"
#include "scene/scene.hpp"
#include "math/random462.hpp"
#include "light/light.hpp"

namespace _462 {

class Scene;
class BSDF;

enum BxDFType {
    BSDF_REFLECTION   = 1<<0,
    BSDF_TRANSMISSION = 1<<1,
    BSDF_DIFFUSE      = 1<<2,
    BSDF_GLOSSY       = 1<<3,
    BSDF_SPECULAR     = 1<<4,
    BSDF_ALL_TYPES        = BSDF_DIFFUSE |
                            BSDF_GLOSSY |
                            BSDF_SPECULAR,
    BSDF_ALL_REFLECTION   = BSDF_REFLECTION |
                            BSDF_ALL_TYPES,
    BSDF_ALL_TRANSMISSION = BSDF_TRANSMISSION |
                            BSDF_ALL_TYPES,
    BSDF_ALL              = BSDF_ALL_REFLECTION |
                            BSDF_ALL_TRANSMISSION
};


struct LightSampleSet {
	uint32_t num;
	float *light_1d;
	float *light_2d;
};

struct BSDFSampleSet {
	uint32_t num;
	float *bsdf_1d;
	float *bsdf_2d;
};

struct LightSample {
	LightSample(float r1i, float r2i, float ci) :
		r1(r1i), r2(r2i), c(ci) { }

	float r1;
	float r2;
	float c;
};

struct BSDFSample {
	BSDFSample() : r1(0.f), r2(0.f), c(0.f) { }
	BSDFSample(float r1i, float r2i, float ci) :
		r1(r1i), r2(r2i), c(ci) { }
	float r1;
	float r2;
	float c;
};

struct LightOffset {
	uint32_t num;
	uint32_t offset_1d;
	uint32_t offset_2d;
};

struct BSDFOffset {
	uint32_t num;
	uint32_t offset_1d;
	uint32_t offset_2d;
};

enum LightSamplingMode { SAMPLE_ALL, SAMPLE_ONE };

class SurfaceIntegrator {
public:
	~SurfaceIntegrator() { }
	virtual void preprocess(const Scene *scene_ptr, const Camera *camera_ptr) { }
	virtual Sample* request_samples(const Scene *scene_ptr, const Sampler *sampler_ptr) { return NULL; }

	virtual Color3 li(const Scene *scene_ptr, const Ray &ray, const hitRecord &record, const Sample* sample_ptr,
						Random462 &rng) = 0;
	virtual void initialize_sampler(const Scene *scene_ptr, Sampler *sampler_ptr) = 0;
};

Color3 uniform_sample_all_lights(
	const Scene *scene_ptr, 
	BSDF *bsdf_ptr, 
	const Vector3 &p, const Vector3 &n, const Vector3 &wo, 
								 const LightSampleSet* light_sample_ptr, 
								 const BSDFSampleSet* bsdf_sample_ptr,
								Random462 &rng);
Color3 uniform_sample_one_light(const Scene *scene_ptr, BSDF *bsdf_ptr, const Vector3 &p, const Vector3 &n, const Vector3 &wo, const float light_select,
								const LightSampleSet* light_sample_ptr, const BSDFSampleSet* bsdf_sample_ptr,
								Random462 &rng);

Color3 estimate_direct_light(const Scene *scene_ptr, Light* light, BSDF *bsdf_ptr, const Vector3 &p, const Vector3 &n, const Vector3 &wo, 
								 const LightSample &light_sample, const BSDFSample &bsdf_sample,
								Random462 &rng, BxDFType flags);

Color3 SpecularTrace(const Scene *scene_ptr, SurfaceIntegrator *int_ptr,
										Vector3 &wo, const hitRecord &record,
									const Sample* sample_ptr, Random462 &rng, const BxDFType flags);

Color3 SpecularReflect(const Scene *scene_ptr, SurfaceIntegrator *int_ptr,
											Vector3 &wo, const hitRecord &record,
											const Sample* sample_ptr, Random462 &rng);

Color3 SpecularTransmit(const Scene *scene_ptr, SurfaceIntegrator *int_ptr,
											Vector3 &wo, const hitRecord &record,
											const Sample* sample_ptr, Random462 &rng);

} /* _462 */

#endif /* _462_INTEGRATOR_SURFACE_HPP_ */

