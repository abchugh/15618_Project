
#include "math/math.hpp"
#include "btdf.hpp"
#include "sample/sampler.hpp"


namespace _462 {

Color3 SpecularTransmission::sample_f(Vector3 &wo, float r1, float r2,
									  Vector3 *wi_ptr, float *pdf_ptr) const {
	bool entering = wo.z > 0.;
    float ei = fresnel.etai, et = fresnel.etat;
    if (!entering) {
		float temp = ei;
		ei = et;
		et = temp;
	}

	float sin2wo = wo.x * wo.x + wo.y * wo.y;
	float eta2 = et * et / (ei * ei);

	float sin2wi = sin2wo / eta2;

	if (sin2wi > 1.f) return Color3::Black();

	wi_ptr->x = -wo.x * ei / et;
	wi_ptr->y = -wo.y * ei / et;
	wi_ptr->z = std::sqrt(1.f - sin2wi);
	wi_ptr->z = (entering) ? -wi_ptr->z : wi_ptr->z;

	*pdf_ptr = 1.f;

	return (Color3::White() - fresnel.reflectance(wo.z)) * t / std::fabs(wi_ptr->z);// * eta2;
}

}
