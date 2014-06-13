
#ifndef _462_LIGHT_INFINITE_HPP_
#define _462_LIGHT_INFINITE_HPP_

#include "light.hpp"
#include "sample/sampler.hpp"

namespace _462 {

struct VisibilityTest;
class Material;

class InfiniteAreaLight : public Light {
public:
    // Light Interface
    InfiniteAreaLight(const Matrix3 &l2w, Material *m_ptr);
	~InfiniteAreaLight() {
		delete dist_ptr;
		delete material_ptr;
	}

    Color3 sample_L(const Vector3 &p, //float pEpsilon,
							float r1, float r2, float c,
							Vector3 *wi, float *pdf,
							VisibilityTest *vis) const;
    Color3 Power(const Scene *scene) const;
	bool IsDeltaLight() const { return false; }
    float pdf(const Vector3 &p, const Vector3 &wi) const;
	//void initialize_sampler(Sampler *sampler_ptr, LightOffset &offset) const;
	Color3 Le(const Ray &r) const;

private:
	Distribution2D *dist_ptr;
	Material *material_ptr;
};

} /* _462 */

#endif /* _462_LIGHT_INFINITE_HPP_ */

