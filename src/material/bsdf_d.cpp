
#include "math/math.hpp"
#include "bsdf.hpp"
#include "sample/sampler.hpp"


namespace _462 {

Color3 Lambertian::f(Vector3 &wi, Vector3 &wo) const {
	return r / PI;
}

Color3 Lambertian::sample_f(Vector3 &wo, float r1, float r2,
							Vector3 *wi_ptr, float *pdf_ptr) const {
	*wi_ptr = cos_sampled_hemisphere(r1, r2);
	if (wo.z < 0.f) wi_ptr->z = -wi_ptr->z;
	*pdf_ptr = pdf(*wi_ptr, wo);

	return r / PI;
}

float Lambertian::pdf(Vector3 &wi, Vector3 &wo) const {
	return (same_hemisphere(wi, wo)) ? (std::fabs(wo.z) / PI) : 0.f;
}

void Lambertian::initialize_sampler(Sampler *sampler_ptr, BSDFOffset &offset) const {
	offset.num = 1;
	offset.offset_1d = sampler_ptr->add2D(1);
}

//////////////////////////////////

Color3 SpecularReflection::sample_f(Vector3 &wo, float r1, float r2,
									Vector3 *wi_ptr, float *pdf_ptr) const {
	wi_ptr->x = -wo.x;
	wi_ptr->y = -wo.y;
	wi_ptr->z = wo.z;
	*pdf_ptr = 1.f;

	return fresnel_ptr->reflectance(wo.z) * r / std::fabs(wi_ptr->z);
}

///////////////////////////////

int BSDF::num_components(BxDFType flags) const {
	int counter = 0;
	for (uint32_t i = 0; i < nBxDFs; i++) {
		if (bxdfs[i]->isType(flags))
			counter++;
	}

	return counter;
}

Color3 BSDF::f(Vector3 &woW, Vector3 &wiW, const Vector3 n, BxDFType flags) const {
	Vector3 sn, tn;
	coordinate_system(n, &sn, &tn);

	if (dot(wiW, n) * dot(woW, n) > 0)
		flags  = BxDFType(flags & ~BSDF_TRANSMISSION);
	else
		flags  = BxDFType(flags & ~BSDF_REFLECTION);
	Color3 f = Color3::Black();

	Vector3 wi = world_to_shading(wiW, sn, tn, n);
	Vector3 wo = world_to_shading(woW, sn, tn, n);
	for (uint32_t i = 0; i < nBxDFs; i++) {
		f += bxdfs[i]->f(wi, wo);
	}

	return f;
}

Color3 BSDF::sample_f(Vector3 &woW, float r1, float r2, float c,
					Vector3 *wiW_ptr, Vector3 n, float *pdf_ptr,
					BxDFType flags,
                    BxDFType *sampledType) const {
	int matched = num_components(flags);
	if (matched == 0) {
		*pdf_ptr = 0.f;
		return Color3::Black();
	}

	int chosen = (int) matched * c;
	chosen = std::min(chosen, matched - 1);
	int counter = 0;
	BxDF *bxdf = NULL;
	for (uint32_t i = 0; i < nBxDFs; i++) {
		if (bxdfs[i]->isType(flags) && counter++ == chosen) {
			bxdf = bxdfs[i];
			break;
		}
	}

	Vector3 sn, tn;
	coordinate_system(n, &sn, &tn);
	Vector3 wo = world_to_shading(woW, sn, tn, n), wi;
	Color3 f = bxdf->sample_f(wo, r1, r2, &wi, pdf_ptr);

	if (std::fabs(*pdf_ptr) < 1e-4)
		return Color3::Black();

	if (sampledType)
		*sampledType = bxdf->type;
	if (!(bxdf->type & BSDF_SPECULAR) && matched > 1) {
		for (uint32_t i = 0; i < nBxDFs; i++) {
			if (bxdfs[i] != bxdf && bxdfs[i]->isType(flags))
				*pdf_ptr += bxdfs[i]->pdf(wi, wo);
		}
	}
	*pdf_ptr = (matched > 1) ? *pdf_ptr / matched : *pdf_ptr;

	*wiW_ptr = shading_to_world(wi, sn, tn, n);
	if (!(bxdf->type * BSDF_SPECULAR)) {
		f.r = f.g = f.b = 0.f;

		if (dot(*wiW_ptr, n) * dot(woW, n) > 0)
			flags  = BxDFType(flags & ~BSDF_TRANSMISSION);
		else
			flags  = BxDFType(flags & ~BSDF_REFLECTION);

		for (uint32_t i = 0; i < nBxDFs; i++) {
			f += bxdfs[i]->f(wi, wo);
		}
	}
	return f;
}

float BSDF::pdf(Vector3 &woW, Vector3 &wiW, Vector3 n,
          BxDFType flags) const {
	float result = 0.f;
	Vector3 sn, tn;
	coordinate_system(n, &sn, &tn);

	Vector3 wi = world_to_shading(wiW, sn, tn, n);
	Vector3 wo = world_to_shading(woW, sn, tn, n);
	for (uint32_t i = 0; i < nBxDFs; i++) {
		if (bxdfs[i]->isType(flags))
			result += bxdfs[i]->pdf(wi, wo);
	}

	return result;
}

}
