
#include "scene/scene.hpp"
#include "math/vector.hpp"
#include "math/random462.hpp"
#include "light/area.hpp"
#include "direct.hpp"

namespace _462 {
DirectLightingIntegrator::DirectLightingIntegrator(const Scene *scene_ptr, LightSamplingMode mode, uint32_t md)
	: sample_mode(mode), max_depth(md) {
	uint32_t light_sample_num = 1;
	uint32_t bsdf_sample_num = 1;

	if (mode == SAMPLE_ALL) {
		light_sample_num = scene_ptr->num_lights();
		bsdf_sample_num = light_sample_num;
	}
	light_offsets = new LightOffset[light_sample_num];
	bsdf_offsets = new BSDFOffset[bsdf_sample_num];
}

void DirectLightingIntegrator::initialize_sampler(const Scene *scene_ptr, Sampler *sampler_ptr) {
	if (sample_mode == SAMPLE_ONE) {
		// one sample for selecting light, one for area light
		light_offsets[0].num = 1;
		light_offsets[0].offset_1d = sampler_ptr->add1D(2);
		light_offsets[0].offset_2d = sampler_ptr->add2D(1) + 2;
		bsdf_offsets[0].num = 1;
		bsdf_offsets[0].offset_2d = sampler_ptr->add2D(1) + 2;

		sampler_ptr->allocate();

		return ;
	}

	for (uint32_t i = 0; i < scene_ptr->num_lights(); i++) {
		scene_ptr->get_lights()[i]->initialize_sampler(sampler_ptr, light_offsets[i]);
		bsdf_offsets[i].num = scene_ptr->get_lights()[i]->num_samples;
		bsdf_offsets[i].offset_1d = -1;
		bsdf_offsets[i].offset_2d = sampler_ptr->add2D(scene_ptr->get_lights()[i]->num_samples) + light_offsets[i].offset_2d;
		sampler_ptr->allocate();
	}
}

Color3 DirectLightingIntegrator::li(const Scene *scene_ptr, const Ray &ray, const hitRecord &record, const Sample* sample_ptr,
				Random462 &rng) {
	Color3 L = Color3::Black();
    // Compute emitted and reflected light at ray intersection point

	if (record.shape_ptr == NULL) {
		for (uint32_t i = 0; i < scene_ptr->num_lights(); i++) {
			L += scene_ptr->get_lights()[i]->Le(ray);
		}
		return L;
	}

    // Evaluate BSDF at hit point
    BSDF *bsdf_ptr = record.bsdf_ptr;

    // Initialize common variables for Whitted integrator
	Vector3 p = record.p;
    Vector3 n = record.n;
    Vector3 wo = -ray.d;
	LightSampleSet *light_set = NULL;
	BSDFSampleSet *bsdf_set = NULL;

	if (record.shape_ptr->light_ptr != NULL) {
		L += record.shape_ptr->light_ptr->L(n, wo);

		return L;
	}

	if (sample_ptr->x > 400 &&
		sample_ptr->x < 401 &&
		sample_ptr->y > 300 &&
		sample_ptr->y < 301) {
			printf("here\n");
	}

	if (sample_mode == SAMPLE_ALL) {
		light_set = new LightSampleSet[scene_ptr->num_lights()];
		bsdf_set = new BSDFSampleSet[scene_ptr->num_lights()];

		for (uint32_t i = 0; i < scene_ptr->num_lights(); i++) {
			light_set[i].num = light_offsets[i].num;
			light_set[i].light_1d = (float*) sample_ptr + 2 + light_offsets[i].offset_1d;
			light_set[i].light_2d = (float*) sample_ptr + 2 + light_offsets[i].offset_2d;
			bsdf_set[i].num = bsdf_offsets[i].num;
			bsdf_set[i].bsdf_1d = NULL;
			bsdf_set[i].bsdf_2d = (float*) sample_ptr + 2 + bsdf_offsets[i].offset_2d;
		}

		L += uniform_sample_all_lights(scene_ptr, bsdf_ptr, p, n, wo, light_set,
				bsdf_set, rng);
	}
	else {
		light_set = new LightSampleSet[1];
		bsdf_set = new BSDFSampleSet[1];

		light_set[0].num = light_offsets[0].num;
		light_set[0].light_1d = (float*) sample_ptr + 2 + light_offsets[0].offset_1d;
		light_set[0].light_2d = (float*) sample_ptr + 2 + light_offsets[0].offset_2d;
		bsdf_set[0].num = bsdf_offsets[0].num;
		bsdf_set[0].bsdf_1d = NULL;
		bsdf_set[0].bsdf_2d = (float*) sample_ptr + 2 + bsdf_offsets[0].offset_2d;

		float select = *((float*)sample_ptr + 2);

		L += uniform_sample_one_light(scene_ptr, bsdf_ptr, p, n, wo, select, light_set,
				bsdf_set, rng);
	}

    if (record.depth + 1 < max_depth) {
        // Trace rays for specular reflection and refraction
		L += SpecularReflect(scene_ptr, this, wo, record, sample_ptr, rng);
        L += SpecularTransmit(scene_ptr, this, wo, record, sample_ptr, rng);
    }

	delete light_set;
	delete bsdf_set;

    return L;
}


}
