
#include "scene/scene.hpp"
#include "math/vector.hpp"
#include "material/bxdf.hpp"
#include "math/random462.hpp"
#include "light/area.hpp"
#include "whitted.hpp"

namespace _462 {

void WhittedIntegrator::initialize_sampler(const Scene *scene_ptr, Sampler *sampler_ptr) {
	//sampler_ptr->add1D(1);
	sampler_ptr->add2D(1);
	sampler_ptr->allocate();
}

Color3 WhittedIntegrator::SpecularTrace(const Scene *scene_ptr, Vector3 &wo, const hitRecord &record,
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
		L += f * std::fabs(dot(n, wi)) / pdf * li(scene_ptr, r, h, sample_ptr, rng);
	}

	return L;
}

Color3 WhittedIntegrator::SpecularReflect(const Scene *scene_ptr,
											Vector3 &wo, const hitRecord &record,
											const Sample* sample_ptr, Random462 &rng) {
	return SpecularTrace(scene_ptr, wo, record, sample_ptr, rng, BxDFType(BSDF_REFLECTION | BSDF_SPECULAR));
}

Color3 WhittedIntegrator::SpecularTransmit(const Scene *scene_ptr,
											Vector3 &wo, const hitRecord &record,
											const Sample* sample_ptr, Random462 &rng) {
	return SpecularTrace(scene_ptr, wo, record, sample_ptr, rng, BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR));
}

Color3 WhittedIntegrator::li(const Scene *scene_ptr, const Ray &ray, const hitRecord &record, const Sample* sample_ptr,
			Random462 &rng) {
	Color3 L = Color3::Black();
    // Compute emitted and reflected light at ray intersection point

	if (record.shape_ptr == NULL)
		return L;

    // Evaluate BSDF at hit point
    BSDF *bsdf_ptr = record.bsdf_ptr;

    // Initialize common variables for Whitted integrator
	Vector3 p = record.p;
    Vector3 n = record.n;
    Vector3 wo = -ray.d;

    // Compute emitted light if ray hit an area light source
	if (record.shape_ptr->light_ptr != NULL) {
		L += record.shape_ptr->light_ptr->L(n, wo);

		return L;
	}

    // Add contribution of each light source
    for (uint32_t i = 0; i < scene_ptr->num_lights(); ++i) {
		Color3 Ld = Color3::Black();
		for (uint32_t j = 0; j < scene_ptr->get_lights()[i]->num_samples; j++) {
			Vector3 wi;
			float pdf;
			VisibilityTest visibility;

			Color3 Li = scene_ptr->get_lights()[i]->sample_L(p, 
				rng.random(), rng.random(), rng.random(),
				&wi, &pdf, &visibility);

			if (Li == Color3::Black() ||
				pdf < 1e-4)
				continue;
			Color3 f = Color3::Black();
			if (bsdf_ptr)
				f = bsdf_ptr->f(wo, wi, n);
			hitRecord h;
			if (!(f == Color3::Black()) &&
				!scene_ptr->hit(visibility.r, visibility.t0, visibility.t1, h, false)) {
				float abscos = std::fabs(dot(wi, n));
				Ld += f * Li * abscos / pdf;
			}
		}
		L += Ld / scene_ptr->get_lights()[i]->num_samples;
    }

    if (record.depth + 1 < max_depth) {
        // Trace rays for specular reflection and refraction
		L += _462::SpecularReflect(scene_ptr, this, wo, record, sample_ptr, rng);
        L += _462::SpecularTransmit(scene_ptr, this, wo, record, sample_ptr, rng);
    }
    return L;
}

}
