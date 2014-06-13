
#include "area.hpp"
#include "scene/ray.hpp"
#include "light.hpp"
#include "integrator/surface.hpp"
#include "scene/scene.hpp"

namespace _462 {

AreaLight::AreaLight(const Matrix3 &l2w, Geometry *geo_p, Color3 ll, int ns)
	 : Light(l2w, ns), l(ll), geo_ptr(geo_p) {
	area = geo_ptr->get_area();
}

Color3 AreaLight::L(const Vector3 &n, const Vector3 &r) const {
	return (dot(r, n) > 0) ? l : Color3::Black();
}

Color3 AreaLight::sample_L(const Vector3 &p, 
							float r1, float r2, float c,
							Vector3 *wi, float *pdf,
							VisibilityTest *vis) const {
	Vector3 n;
	Vector3 shape_p = geo_ptr->sample(p, r1, r2, c, &n);
	*wi = normalize(shape_p - p);
	*pdf = geo_ptr->pdf(p, *wi);
	vis->r.e = p;
	vis->r.d = shape_p - p;
	vis->t0 = 1e-5;
	vis->t1 = 1.f - 1e-5;

	return L(n, -*wi);
}

Color3 AreaLight::Power(const Scene *) const {
	return l * PI * area;
}

float AreaLight::pdf(const Vector3 &p, const Vector3 &wi) const {
	return geo_ptr->pdf(p, wi);
}

void AreaLight::initialize_sampler(Sampler *sampler_ptr, LightOffset &offset) const {
	offset.num = num_samples;
	offset.offset_1d = sampler_ptr->add1D(num_samples);
	offset.offset_2d = sampler_ptr->add2D(num_samples) + offset.num;
}

}
