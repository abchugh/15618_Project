
#ifndef _462_MATERIAL_BXDF_HPP_
#define _462_MATERIAL_BXDF_HPP_

#include "math/color.hpp"
#include "math/vector.hpp"
#include "sample/sampler.hpp"
#include "integrator/surface.hpp"
#include <string>

namespace _462 {
#pragma message (__FILE__ ": parent")
struct BSDFOffset;

inline bool same_hemisphere(Vector3 v1, Vector3 v2) {
	return (v1.z * v2.z) > 0;
}


class BxDF {
public:

	BxDF(BxDFType t) : type(t) { }
	virtual ~BxDF() { }

	bool isType(BxDFType t) {
		return (type & t) == type;
	}

	virtual Color3 f(Vector3 &wi, Vector3 &wo) const = 0;
	virtual Color3 sample_f(Vector3 &wo, float r1, float r2,
							Vector3 *wi_ptr, float *pdf_ptr) const = 0;
	virtual float pdf(Vector3 &wi, Vector3 &wo) const = 0;
	virtual void initialize_sampler(Sampler *sampler_ptr, BSDFOffset &offset) const { }

	BxDFType type;
};

class Fresnel {
public:
	virtual Color3 reflectance(float cosi) const = 0;
};

class FresnelNoOp : public Fresnel {
public:
	FresnelNoOp() { }
	Color3 reflectance(float cosi) const { return Color3::White(); }
};

class FresnelConductor : public Fresnel {
public:
	FresnelConductor(float etai, float ki) :
		eta(etai), k(ki) { }
	Color3 reflectance(float cosi) const;

	float eta, k;
};

class FresnelDielectric : public Fresnel {
public:
	FresnelDielectric(float etaii, float etati) :
		etai(etaii), etat(etati) { }
	Color3 reflectance(float cosi) const;

	float etai, etat;
};


class Lambertian : public BxDF {
public:

	Lambertian(Color3 rr) : BxDF(BxDFType(BSDF_DIFFUSE | BSDF_REFLECTION)), r(rr) { }
	~Lambertian() { }

	virtual Color3 f(Vector3 &wi, Vector3 &wo) const;
	virtual Color3 sample_f(Vector3 &wo, float r1, float r2,
							Vector3 *wi_ptr, float *pdf_ptr) const;
	virtual float pdf(Vector3 &wi, Vector3 &wo) const;
	virtual void initialize_sampler(Sampler *sampler_ptr, BSDFOffset &offset) const;

	Color3 r;
};

class SpecularReflection : public BxDF {
public:
	SpecularReflection(Color3 rr, Fresnel *f_ptr) :
		BxDF(BxDFType(BSDF_SPECULAR | BSDF_REFLECTION)), r(rr), fresnel_ptr(f_ptr) { }
	~SpecularReflection() { if (fresnel_ptr) delete fresnel_ptr; }

	virtual Color3 f(Vector3 &wi, Vector3 &wo) const {
		return Color3::Black();
	}
	virtual Color3 sample_f(Vector3 &wo, float r1, float r2,
							Vector3 *wi_ptr, float *pdf_ptr) const;
	virtual float pdf(Vector3 &wi, Vector3 &wo) const {
		return 0.f;
	}

	Color3 r;
	Fresnel *fresnel_ptr;
};

class BSDF {
public:
    #define MAX_BxDFS 8
    BSDF(float eta_i = 1.f) :
		eta(eta_i), nBxDFs(0) { }
	~BSDF() {
		for (int i = 0; i < nBxDFs; i++)
			delete bxdfs[i];
	}
	inline void add(BxDF *bxdf) { if (nBxDFs < MAX_BxDFS) bxdfs[nBxDFs++] = bxdf; }
	int num_components() const { return nBxDFs; }
    int num_components(BxDFType flags) const;
    Color3 f(Vector3 &woW, Vector3 &wiW, const Vector3 n, BxDFType flags = BSDF_ALL) const;
	// BSDF Public Methods
    Color3 sample_f(Vector3 &woW, float r1, float r2, float c,
					Vector3 *wiW_ptr, Vector3 n, float *pdf_ptr,
					BxDFType flags = BSDF_ALL,
                    BxDFType *sampledType = NULL) const;
    float pdf(Vector3 &woW, Vector3 &wiW, Vector3 n,
              BxDFType flags = BSDF_ALL) const;

    // BSDF Public Data
    const float eta;

	Vector3 world_to_shading(const Vector3 &v, const Vector3 &sn, const Vector3 &tn, const Vector3 &nn) const {
		return Vector3(dot(sn, v), dot(tn, v), dot(nn, v));
	}

	Vector3 shading_to_world(const Vector3 &v, const Vector3 &sn, const Vector3 &tn, const Vector3 &nn) const {
		return Vector3(sn.x * v.x + tn.x * v.y + nn.x * v.z,
						sn.y * v.x + tn.y * v.y + nn.y * v.z,
						sn.z * v.x + tn.z * v.y + nn.z * v.z);
	}

private:
	uint32_t nBxDFs;
    BxDF *bxdfs[MAX_BxDFS];
};

} /* _462 */

#endif /* _462_MATERIAL_BXDF_HPP_ */

