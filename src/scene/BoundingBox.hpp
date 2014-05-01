#ifndef _462_BOUNDING_BOX_HPP_
#define _462_BOUNDING_BOX_HPP_
#include "math/vector.hpp"

#include "partition_ispc.h"
#include "ispc_switch.h"

namespace _462 {
class Ray;
struct Frustum;
class BoundingBox
{
public:
    BoundingBox();

    //Extend the bounding box to accomodate the point
    void AddPoint(Vector3 point);
#ifdef ISPC_SOA
    void AddCentroid(const ispc::BVHPrimitiveInfoList& list, int index);
    void AddBox(const ispc::BVHPrimitiveInfoList& list, int index);
#endif

    //combine with another bounding box to form a bigger bounding volume
    void AddBox(BoundingBox box);

    
    //hit testing on the box
    bool hit(const Ray& r, real_t t0, real_t t1)const;

    //Faster hit testing
    bool hit(const Vector3& invDir,const Vector3& origin, real_t t0, real_t t1, const uint32_t dirIsNeg[3])const;

    //check if a box intersects with another
    bool hit(BoundingBox box)const;

    // Check against a frustum
    bool hit(const Frustum& frustum) const;

    int MaximumExtent()const;
    Vector3 lowCoord, highCoord;
    real_t SurfaceArea()const;
    
    const Vector3& operator[](int v) const
    {
        if(v==0)
            return lowCoord;
        else
            return highCoord;
    }
    inline real_t extent(int dim)const
    {
        return highCoord[dim]-lowCoord[dim];
    }
    inline Vector3 centroid()const
    {
        return 0.5f*lowCoord + 0.5f*highCoord;
    }
};
}
#endif
