/**
 * @file triangle.hpp
 * @brief Class definition for Triangle.
 *
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_TRIANGLE_HPP_
#define _462_SCENE_TRIANGLE_HPP_

#include "scene/scene.hpp"

namespace _462 {

/**
 * a triangle geometry.
 * Triangles consist of 3 vertices. Each vertex has its own position, normal,
 * texture coordinate, and material. These should all be interpolated to get
 * values in the middle of the triangle.
 * These values are all in local space, so it must still be transformed by
 * the Geometry's position, orientation, and scale.
 */
class Triangle : public Geometry
{
public:

    struct Vertex
    {
        // note that position and normal are in local space
        Vector3 position;
        Vector3 normal;
        Vector2 tex_coord;
        const Material* material;
    };

    // the triangle's vertices, in CCW order
    Vertex vertices[3];
    bool simple;
    Triangle();
    virtual ~Triangle();
    virtual void render() const;
	
    virtual bool hit(const Ray& r, real_t t0, real_t t1, hitRecord& h, bool fullRecord) const;
    virtual void hitPacket(const Packet& packet, int start, int end, real_t t0, real_t *t1, std::vector<hitRecord>& hs, bool fullRecord) const;
    virtual void InitGeometry();
    static bool getBarycentricCoordinates(const Ray& r, real_t& t,real_t mult[3], Vector3 position[3]);
    static void getMaterialProperties(MaterialProp& mp, const real_t mult[3],const Vector2& texCoord, const Material* materials[3]);
    static void getMaterialProperties(MaterialProp& mp, const Vector2& texCoord, const Material* materials);

};


} /* _462 */

#endif /* _462_SCENE_TRIANGLE_HPP_ */
