
#include "point.hpp"
#include "scene/ray.hpp"
#include "light.hpp"
#include "integrator/surface.hpp"
#include "scene/scene.hpp"

namespace _462 {
Color3 PointLight::sample_L(const Vector3 &p, 
							float r1, float r2, float c,
							Vector3 *wi, float *pdf,
							VisibilityTest *vis) const {
	*wi = normalize(position - p);
	*pdf = 1.f;
	vis->r.e = p;
	vis->r.d = position - p;
	vis->t0 = 1e-3f;
	vis->t1 = 1.f - 1e-3f;

	return intensity / (4 * PI * squared_length(position - p));
}

Color3 PointLight::Power(const Scene *scene) const {
	return 4 * PI * intensity;
}

float PointLight::pdf(const Vector3 &p, const Vector3 &wi) const {
	return 0.f;
}

}
