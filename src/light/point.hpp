
#ifndef _462_LIGHT_POINT_HPP_
#define _462_LIGHT_POINT_HPP_

#include "light.hpp"

namespace _462 {

struct VisibilityTest;
class Scene;
class Geometry;

class PointLight : public Light {
public:
    // Light Interface
	PointLight(const Matrix3 &l2w, const Vector3 posi, const Color3 intensityi)
		: intensity(intensityi), position(posi), Light(l2w) { }
    Color3 sample_L(const Vector3 &p, //float pEpsilon,
							float r1, float r2, float c,
							Vector3 *wi, float *pdf,
							VisibilityTest *vis) const;
    Color3 Power(const Scene *scene) const;
	bool IsDeltaLight() const { return true; }
    float pdf(const Vector3 &p, const Vector3 &wi) const;

protected:
    Color3 intensity;
	Vector3 position;
};

} /* _462 */

#endif /* _462_LIGHT_POINT_HPP_ */

