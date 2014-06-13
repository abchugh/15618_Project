
#include "scene/scene.hpp"
#include "math/vector.hpp"
#include "material/bxdf.hpp"
#include "math/random462.hpp"
#include "light/area.hpp"
#include "surface.hpp"

namespace _462 {

Color3 uniform_sample_all_lights(const Scene *scene_ptr, BSDF *bsdf_ptr, const Vector3 &p, const Vector3 &n, const Vector3 &wo, 
								 const LightSampleSet* light_sampleset_ptr, const BSDFSampleSet* bsdf_sampleset_ptr,
								Random462 &rng) {
	Color3 L = Color3::Black();

	for (uint32_t i = 0; i < scene_ptr->num_lights(); i++) {
		Color3 Ld = Color3::Black();
		uint32_t num = (light_sampleset_ptr) ? light_sampleset_ptr[i].num : 1;

		for (uint32_t j = 0; j < num; j++) {
			float l_r1 = (light_sampleset_ptr && light_sampleset_ptr[i].light_2d) ? light_sampleset_ptr[i].light_2d[j * 2] : rng.random();
			float l_r2 = (light_sampleset_ptr && light_sampleset_ptr[i].light_2d) ? light_sampleset_ptr[i].light_2d[j * 2 + 1] : rng.random();
			float l_c = (light_sampleset_ptr && light_sampleset_ptr[i].light_1d) ? light_sampleset_ptr[i].light_1d[j] : rng.random();
			LightSample light_sample(l_r1, l_r2, l_c);

			float b_r1 = (bsdf_sampleset_ptr && bsdf_sampleset_ptr[i].bsdf_2d) ? bsdf_sampleset_ptr[i].bsdf_2d[j * 2] : rng.random();
			float b_r2 = (bsdf_sampleset_ptr && bsdf_sampleset_ptr[i].bsdf_2d) ? bsdf_sampleset_ptr[i].bsdf_2d[j * 2 + 1] : rng.random();
			float b_c = (bsdf_sampleset_ptr && bsdf_sampleset_ptr[i].bsdf_1d) ? bsdf_sampleset_ptr[i].bsdf_1d[j] : rng.random();
			BSDFSample bsdf_sample(b_r1, b_r2, b_c);

			Ld += estimate_direct_light(scene_ptr, scene_ptr->get_lights()[i], bsdf_ptr, p, n, wo,
				light_sample, bsdf_sample, rng, BxDFType(BSDF_ALL & ~BSDF_SPECULAR));
		}
		L += Ld / num;
	}

	return L;
}

Color3 uniform_sample_one_light(const Scene *scene_ptr, BSDF *bsdf_ptr, const Vector3 &p, const Vector3 &n, const Vector3 &wo, const float light_select,
								const LightSampleSet *light_sampleset_ptr, const BSDFSampleSet *bsdf_sampleset_ptr,
								Random462 &rng) {
	Color3 L = Color3::Black();
	float r_select = (light_select < 0) ? rng.random() : light_select;
	int selected = (int)(scene_ptr->num_lights() * r_select);

	selected = (selected >= scene_ptr->num_lights()) ? scene_ptr->num_lights() -  1 : selected;

	Color3 Ld = Color3::Black();
	uint32_t num = (light_sampleset_ptr) ? light_sampleset_ptr->num : 1;
	
	for (uint32_t j = 0; j < num; j++) {
		float l_r1 = (light_sampleset_ptr && light_sampleset_ptr->light_2d) ? light_sampleset_ptr->light_2d[j * 2] : rng.random();
		float l_r2 = (light_sampleset_ptr && light_sampleset_ptr->light_2d) ? light_sampleset_ptr->light_2d[j * 2 + 1] : rng.random();
		float l_c = (light_sampleset_ptr && light_sampleset_ptr->light_1d) ? light_sampleset_ptr->light_1d[j] : rng.random();
		LightSample light_sample(l_r1, l_r2, l_c);

		float b_r1 = (bsdf_sampleset_ptr && bsdf_sampleset_ptr->bsdf_2d) ? bsdf_sampleset_ptr->bsdf_2d[j * 2] : rng.random();
		float b_r2 = (bsdf_sampleset_ptr && bsdf_sampleset_ptr->bsdf_2d) ? bsdf_sampleset_ptr->bsdf_2d[j * 2 + 1] : rng.random();
		float b_c = (bsdf_sampleset_ptr && bsdf_sampleset_ptr->bsdf_1d) ? bsdf_sampleset_ptr->bsdf_1d[j] : rng.random();
		BSDFSample bsdf_sample(b_r1, b_r2, b_c);
		Ld += estimate_direct_light(scene_ptr, scene_ptr->get_lights()[selected], bsdf_ptr, p, n, wo,
			light_sample, bsdf_sample, rng, BxDFType(BSDF_ALL & ~BSDF_SPECULAR));
	}
	L += Ld / num;
	L *= scene_ptr->num_lights();

	return L;
}

Color3 estimate_direct_light(const Scene *scene_ptr, Light* light, BSDF *bsdf_ptr, const Vector3 &p, const Vector3 &n, const Vector3 &wo, 
								 const LightSample &light_sample, const BSDFSample &bsdf_sample,
								Random462 &rng, BxDFType flags) {
	Color3 L = Color3::Black();

	Vector3 wi;
	float l_pdf;
	float b_pdf;
	VisibilityTest test;
	Color3 li;
	li = light->sample_L(p, light_sample.r1, light_sample.r2, light_sample.c,
						&wi, &l_pdf, &test);

	if (l_pdf > 1e-3 && li != Color3::Black()) {
		Color3 f = bsdf_ptr->f(const_cast<Vector3&>(wo), wi, n, flags);
		b_pdf = bsdf_ptr->pdf(const_cast<Vector3&>(wo), wi, n, flags);
		if(f != Color3::Black() && b_pdf > 1e-3) {
			hitRecord h;
			if (!scene_ptr->hit(test.r, test.t0, test.t1, h, false)) {
				L += f * li * std::fabs(dot(wi, n)) / l_pdf;
				if (!light->IsDeltaLight()) {
					L *= power_heuristic(1, l_pdf, 1, b_pdf);
				}
			}
		}
	}

	BxDFType type;
	Color3 fi = bsdf_ptr->sample_f(const_cast<Vector3&>(wo), bsdf_sample.r1, bsdf_sample.r2, bsdf_sample.c, &wi, n,
		&b_pdf, flags, &type);
	li.r = li.g = li.b = 0.;
	if (fi != Color3::Black() && b_pdf > 1e-3) {
		float weight = 1.f;

		if (!(type & BSDF_SPECULAR)) {
			l_pdf = light->pdf(p, wi);
			// non-specular + shadow
			if (l_pdf < 1e-3)
				return L;
			weight = power_heuristic(1, b_pdf, 1, l_pdf);
		}

		// check along wi to see if theres any light
		Ray ray(p, wi);
		hitRecord h;
		if (scene_ptr->hit(ray, 1e-3, BIG_NUMBER, h, true)) {
			if (h.shape_ptr->light_ptr == (AreaLight*)light) {
				li = h.shape_ptr->light_ptr->L(h.n, -wi);
			}
		}
		// for infinite light
		else
			li = light->Le(ray);
		L += fi * li * std::fabs(dot(wi, n)) * weight / b_pdf;
	}

	return L;
}

Color3 SpecularTrace(const Scene *scene_ptr, SurfaceIntegrator *int_ptr,
										Vector3 &wo, const hitRecord &record,
									const Sample* sample_ptr, Random462 &rng, const BxDFType flags) {
	Color3 L = Color3::Black();
	// Evaluate BSDF at hit point
    BSDF *bsdf_ptr = record.bsdf_ptr;

    // Initialize common variables for Whitted integrator
    Vector3 p = record.p;
    Vector3 n = record.n;

    Vector3 wi;
    float pdf;
	Color3 f = bsdf_ptr->sample_f(wo, rng.random(), rng.random(), rng.random(),
								&wi, n, &pdf,
                                flags);

	if (pdf > 0.f && f != Color3::Black() && std::fabs(dot(n, wi)) > 1e-3) {
		Ray r(p, wi);
		hitRecord h;
		scene_ptr->hit(r, 1e-3, BIG_NUMBER, h, true);
		h.depth = record.depth + 1;
		L += f * std::fabs(dot(n, wi)) / pdf * int_ptr->li(scene_ptr, r, h, sample_ptr, rng);
	}

	return L;
}

Color3 SpecularReflect(const Scene *scene_ptr, SurfaceIntegrator *int_ptr,
											Vector3 &wo, const hitRecord &record,
											const Sample* sample_ptr, Random462 &rng) {
	return SpecularTrace(scene_ptr, int_ptr, wo, record, sample_ptr, rng, BxDFType(BSDF_REFLECTION | BSDF_SPECULAR));
}

Color3 SpecularTransmit(const Scene *scene_ptr, SurfaceIntegrator *int_ptr,
											Vector3 &wo, const hitRecord &record,
											const Sample* sample_ptr, Random462 &rng) {
	return SpecularTrace(scene_ptr, int_ptr, wo, record, sample_ptr, rng, BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR));
}

}
