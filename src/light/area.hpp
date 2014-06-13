
#ifndef _462_LIGHT_AREA_HPP_
#define _462_LIGHT_AREA_HPP_

#include "light.hpp"

namespace _462 {

struct VisibilityTest;
class Scene;
class Geometry;

class AreaLight : public Light {
public:
    // Light Interface
    AreaLight(const Matrix3 &l2w, Geometry *geo_p, Color3 ll, int ns = 1);
    Color3 sample_L(const Vector3 &p, //float pEpsilon,
							float r1, float r2, float c,
							Vector3 *wi, float *pdf,
							VisibilityTest *vis) const;
    Color3 Power(const Scene *scene) const;
	bool IsDeltaLight() const { return false; }
    float pdf(const Vector3 &p, const Vector3 &wi) const;
	Color3 L(const Vector3 &n, const Vector3 &wi) const;
	void initialize_sampler(Sampler *sampler_ptr, LightOffset &offset) const;
	Color3 l;

protected:
	Geometry *geo_ptr;
	float area;
};

} /* _462 */

#endif /* _462_LIGHT_AREA_HPP_ */

