/**
 * @file sphere.hpp
 * @brief Class defnition for Sphere.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_SPHERE_HPP_
#define _462_SCENE_SPHERE_HPP_

#include "scene/scene.hpp"

namespace _462 {

/**
 * A sphere, centered on its position with a certain radius.
 */
class Sphere : public Geometry
{
public:

    real_t radius;
    const Material* material;

    Sphere();
    virtual ~Sphere();
    virtual void render() const;
    virtual bool hit(const Ray& r, real_t t0, real_t t1, hitRecord& h, bool fullRecord) const;
    virtual void hitPacket(const Packet& packet, int start, int end, real_t t0, real_t *t1Ptr, std::vector<hitRecord>& hs, bool fullRecord) const;
    virtual void InitGeometry();
	virtual float get_area();
	Vector3 sample(float r1, float r2, Vector3 *n_ptr);
	virtual Vector3 sample(const Vector3 &p, float r1, float r2, float c, Vector3 *n_ptr);
	virtual float pdf(const Vector3 &p, const Vector3 &wi);
};

} /* _462 */

#endif /* _462_SCENE_SPHERE_HPP_ */

