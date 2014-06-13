
#include "scene/scene.hpp"
#include "math/vector.hpp"
#include "math/random462.hpp"
#include "light/area.hpp"
#include "material/bxdf.hpp"
#include "path.hpp"

namespace _462 {
PathIntegrator::PathIntegrator(const Scene *scene_ptr, uint32_t sd, uint32_t md, uint32_t npp)
	: sample_depth(sd), max_depth(md), num_per_path(npp) {
	light_offsets = new LightOffset[sd * npp];
	bsdf_offsets = new BSDFOffset[sd * npp];
	path_offsets = new BSDFOffset[sd * npp];
}

void PathIntegrator::initialize_sampler(const Scene *scene_ptr, Sampler *sampler_ptr) {
	for (uint32_t j = 0; j < num_per_path; j++) {
		for (uint32_t i = 0; i < sample_depth; i++) {
			// one sample for selecting light, one for area light
			light_offsets[j * sample_depth + i].num = 1;
			light_offsets[j * sample_depth + i].offset_1d = sampler_ptr->add1D(2) + 1;
			light_offsets[j * sample_depth + i].offset_2d = sampler_ptr->add2D(1) + 4 * sample_depth * num_per_path;
			bsdf_offsets[j * sample_depth + i].num = 1;
			bsdf_offsets[j * sample_depth + i].offset_1d = sampler_ptr->add1D(1);
			bsdf_offsets[j * sample_depth + i].offset_2d = sampler_ptr->add2D(1) + 4 * sample_depth * num_per_path;
			path_offsets[j * sample_depth + i].num = 1;
			path_offsets[j * sample_depth + i].offset_1d = sampler_ptr->add1D(1);
			path_offsets[j * sample_depth + i].offset_2d = sampler_ptr->add2D(1) + 4 * sample_depth * num_per_path;
		}
	}

	sampler_ptr->allocate();
}

Color3 PathIntegrator::li(const Scene *scene_ptr, const Ray &ray, const hitRecord &record, const Sample* sample_ptr,
				Random462 &rng) {
	Color3 L = Color3::Black();
    if (record.shape_ptr == NULL)
		return L;

	for (uint32_t j = 0; j < num_per_path; j++) {
		bool specular_bounce = false;
		const hitRecord *current_h_ptr = &record;
		hitRecord path_record;
		Ray current_ray(ray);
		Color3 path_weight = Color3::White();
		LightSampleSet light_set;
		BSDFSampleSet bsdf_set;
		BSDFSample path_sample;
		float select;

		light_set.num = 1;
		bsdf_set.num = 1;

		Color3 new_path_l;
		Color3 bsdf_factor;
		Color3 weight_records[7];
		for (uint32_t i = 0; i < max_depth; i++) {
			Vector3 p = current_h_ptr->p;
			Vector3 n = current_h_ptr->n;
			Vector3 wo = -current_ray.d;
			BSDF *bsdf_ptr = current_h_ptr->bsdf_ptr;

			if (i == 0 || specular_bounce) {
				if (current_h_ptr->shape_ptr->light_ptr != NULL) {
					L += path_weight * 
						current_h_ptr->shape_ptr->light_ptr->L(n, wo);
				}
			}		

			// TODO: hit light source?
			if (current_h_ptr->shape_ptr->light_ptr != NULL)
				break;

			if (i < sample_depth) {
				light_set.light_1d = (float*) sample_ptr + 2 + light_offsets[j * sample_depth + i].offset_1d;
				light_set.light_2d = (float*) sample_ptr + 2 + light_offsets[j * sample_depth + i].offset_2d;
				bsdf_set.bsdf_1d = (float*) sample_ptr + 2 + bsdf_offsets[j * sample_depth + i].offset_1d;
				bsdf_set.bsdf_2d = (float*) sample_ptr + 2 + bsdf_offsets[j * sample_depth + i].offset_2d;
				float* path_sample_ptr = (float*) sample_ptr + 2 + path_offsets[j * sample_depth + i].offset_2d;
				path_sample.r1 = path_sample_ptr[0];
				path_sample.r2 = path_sample_ptr[1];
				path_sample.c = *((float*) sample_ptr + 2 + path_offsets[j * sample_depth + i].offset_1d);

				select = *((float*)sample_ptr + 2);
			}
			else {
				light_set.light_1d = NULL;
				light_set.light_2d = NULL;
				bsdf_set.bsdf_1d = NULL;
				bsdf_set.bsdf_2d = NULL;
				path_sample.r1 = rng.random();
				path_sample.r2 = rng.random();
				path_sample.c = rng.random();

				select = -1;
			}

			new_path_l = path_weight * uniform_sample_one_light(scene_ptr, bsdf_ptr, p, n,
				wo, select, &light_set, &bsdf_set, rng);
			L += new_path_l;

			Vector3 wi;
			float path_pdf;
			BxDFType flags;
			Color3 f = bsdf_ptr->sample_f(wo, path_sample.r1,path_sample.r2, path_sample.c, 
				&wi, n, &path_pdf, BSDF_ALL, &flags);
			if (f == Color3::Black() || path_pdf < 1e-3) 
				break;
			specular_bounce = (flags & BSDF_SPECULAR) > 0;
			bsdf_factor = f * std::fabs(dot(n, wi)) / path_pdf;
			weight_records[i] = bsdf_factor;
			path_weight *= bsdf_factor;

			if (i >= sample_depth) {
				// TODO : how to determine the prob?
				float continue_prob = std::min(0.5, path_weight.relative_luminance());
				if (rng.random() > continue_prob)
					break;
				path_weight /= continue_prob;
			}
			else if (i >= max_depth) {
				break;
			}

			Ray path_ray(p, wi);
			if (!scene_ptr->hit(path_ray, 1e-3, BIG_NUMBER, path_record, true)) {
				if (specular_bounce)
					for (uint32_t i = 0; i < scene_ptr->num_lights(); i++)
						L += path_weight * scene_ptr->get_lights()[i]->Le(path_ray);
				break;
			}

			current_ray.e = path_ray.e;
			current_ray.d = path_ray.d;
			current_h_ptr = &path_record;
		}
	}

	L /= num_per_path;

    return L;
}


}
