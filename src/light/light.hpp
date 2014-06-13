
#ifndef _462_LIGHT_LIGHT_HPP_
#define _462_LIGHT_LIGHT_HPP_

#include "math/matrix.hpp"
#include "math/color.hpp"
#include "math/vector.hpp"

namespace _462 {
class Scene;
struct LightOffset;
class Sampler;
class Ray;
struct VisibilityTest;

class Light {
public:
    // Light Interface
	virtual ~Light() { }
    Light(const Matrix3 &l2w, int ns = 1)
        : num_samples(std::max(1, ns)), LightToWorld(l2w) {
        inverse(&WorldToLight, LightToWorld);
    }

	virtual void initialize_sampler(Sampler *sampler_ptr, LightOffset &offset) const;
    virtual Color3 sample_L(const Vector3 &p, //float pEpsilon,
							float r1, float r2, float c,
							Vector3 *wi, float *pdf,
							VisibilityTest *vis) const = 0;
    
    virtual bool IsDeltaLight() const = 0;
	virtual Color3 Power(
		const Scene *scene) const = 0;
    virtual Color3 Le(const Ray &r) const {
		return Color3::Black();
	}

    virtual float pdf(const Vector3 &p, const Vector3 &wi) const = 0;

    // Light Public Data
    int num_samples;
protected:
    // Light Protected Data
    Matrix3 LightToWorld, WorldToLight;
};

} /* _462 */

#endif /* _462_LIGHT_LIGHT_HPP_ */

