
#include "infinite.hpp"
#include "scene/ray.hpp"
#include "light.hpp"
#include "integrator/surface.hpp"
#include "scene/scene.hpp"
#include "material/material.hpp"
#include <complex>

namespace _462 {

InfiniteAreaLight::InfiniteAreaLight(const Matrix3 &l2w, Material *m_ptr) 
	: Light(l2w), material_ptr(m_ptr) {
	int width, height;
	m_ptr->get_texture_size(&width, &height);

	float *tex_buffer = new float[width * height];

	for (uint32_t y = 0; y < height; y++) {
		float sin_theta = std::sin(PI * (y + 0.5f) / height);
		float inv_y = height - 1 - y;
		for (uint32_t x = 0; x < width; x++) {
			tex_buffer[y * width + x] = m_ptr->get_filtered_texture_pixel(x, inv_y).relative_luminance();
			tex_buffer[y * width + x] *= sin_theta;
		}
	}

	dist_ptr = new Distribution2D(tex_buffer, width, height);

	delete[] tex_buffer;
}

Color3 InfiniteAreaLight::sample_L(const Vector3 &p, //float pEpsilon,
							float r1, float r2, float c,
							Vector3 *wi, float *pdf,
							VisibilityTest *vis) const {
	Vector3 n;
	float uv_pdf;
	Vector2 uv = dist_ptr->sample_continuous(r1, r2, &uv_pdf);
	float theta = PI * uv.y, phi = 2 * PI *	uv.x;
	float cos_theta = std::cos(theta), sin_theta = std::sin(theta);
	float sin_phi = std::sin(phi), cos_phi = std::cos(phi);

	Vector3 shape_p(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
	*wi = LightToWorld * shape_p;
	*pdf = (sin_theta < 1e-5) ? 0.f : uv_pdf / (2 * PI * PI * sin_theta);
	vis->r.e = p;
	vis->r.d = *wi;
	vis->t0 = 1e-5;
	vis->t1 = BIG_NUMBER;

	return material_ptr->get_texture_pixel_norm_inv(uv.x, uv.y);
}

Color3 InfiniteAreaLight::Power(const Scene *scene) const {
	Vector3 center;
	float radius;
	scene->bounding_sphere(&center, &radius);

	return PI * radius * radius * material_ptr->get_texture_pixel_norm(0.5f, 0.5f);
}

float InfiniteAreaLight::pdf(const Vector3 &p, const Vector3 &wi) const {
	Vector3 trans_wi = WorldToLight * wi;
	float theta = std::acos(trans_wi.z);
	float sin_theta = std::sin(theta);
	float phi = std::atan2(trans_wi.y, trans_wi.x);
	phi = (phi < 1e-5f) ? phi + 2 * PI : phi;

	float u = phi / (2 * PI), v = theta / PI;
	return dist_ptr->pdf(u, v) / (2 * PI * PI * sin_theta);
}

Color3 InfiniteAreaLight::Le(const Ray &r) const {
	Vector3 trans_d = WorldToLight * r.d;
	float theta = std::acos(trans_d.z);
	float phi = std::atan2(trans_d.y, trans_d.x);
	phi = (phi < 1e-5f) ? phi + 2 * PI : phi;

	float u = phi / (2 * PI), v = theta / PI;
	return material_ptr->get_texture_pixel_norm_inv_gamma(u, v);
}

}
