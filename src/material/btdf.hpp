
#ifndef _462_MATERIAL_BTDF_HPP_
#define _462_MATERIAL_BTDF_HPP_

#include "math/color.hpp"
#include "math/vector.hpp"
#include "bxdf.hpp"
#include <string>

namespace _462 {

class SpecularTransmission : public BxDF {
public:
	SpecularTransmission(Color3 tt, FresnelDielectric f) :
		BxDF(BxDFType(BSDF_SPECULAR | BSDF_TRANSMISSION)), t(tt), fresnel(f.etai, f.etat) { }
	~SpecularTransmission() { }

	virtual Color3 f(Vector3 &wi, Vector3 &wo) const {
		return Color3::Black();
	}
	virtual Color3 sample_f(Vector3 &wo, float r1, float r2,
							Vector3 *wi_ptr, float *pdf_ptr) const;
	virtual float pdf(Vector3 &wi, Vector3 &wo) const {
		return 0.f;
	}

	Color3 t;
	FresnelDielectric fresnel;
};

} /* _462 */

#endif /* _462_MATERIAL_BTDF_HPP_ */

