
#include "scene/BoundingBox.hpp"
#include "scene/ray.hpp"

namespace _462 {
BoundingBox::BoundingBox():lowCoord(BIG_NUMBER,BIG_NUMBER,BIG_NUMBER), highCoord(-BIG_NUMBER,-BIG_NUMBER,-BIG_NUMBER)
{
}
void BoundingBox::AddPoint(Vector3 point)
{
	for(int i=0;i<3;i++)
	{
        if(lowCoord[i]>point[i])
            lowCoord[i] = point[i];
        if(highCoord[i]<point[i])
            highCoord[i] = point[i];
    }
}
#ifdef ISPC
void BoundingBox::AddCentroid(const ispc::BVHPrimitiveInfoList& list, int index)
{
    if(this->lowCoord.x > list.centroidx[index])
        this->lowCoord.x = list.centroidx[index];
    if(this->highCoord.x < list.centroidx[index])
        this->highCoord.x = list.centroidx[index];

    if(this->lowCoord.y > list.centroidy[index])
        this->lowCoord.y = list.centroidy[index];
    if(this->highCoord.y < list.centroidy[index])
        this->highCoord.y = list.centroidy[index];

    if(this->lowCoord.z > list.centroidz[index])
        this->lowCoord.z = list.centroidz[index];
    if(this->highCoord.z < list.centroidz[index])
        this->highCoord.z = list.centroidz[index];
}

void BoundingBox::AddBox(const ispc::BVHPrimitiveInfoList& list, int index)
{
    if(this->lowCoord.x > list.lowCoordx[index])
        this->lowCoord.x = list.lowCoordx[index];
    if(this->highCoord.x < list.highCoordx[index])
        this->highCoord.x = list.highCoordx[index];

    if(this->lowCoord.y > list.lowCoordy[index])
        this->lowCoord.y = list.lowCoordy[index];
    if(this->highCoord.y < list.highCoordy[index])
        this->highCoord.y = list.highCoordy[index];

    if(this->lowCoord.z > list.lowCoordz[index])
        this->lowCoord.z = list.lowCoordz[index];
    if(this->highCoord.z < list.highCoordz[index])
        this->highCoord.z = list.highCoordz[index];
}
#endif

real_t BoundingBox::SurfaceArea()const
{
    return 2*(extent(0)*extent(1) + extent(0)*extent(2) + extent(1)*extent(2) ); 
}
int BoundingBox::MaximumExtent()const
{
    if( extent(0) > extent(1))
    {
        if(extent(0)>extent(2))
            return 0;
        else
            return 2;
    }
    else
    {
        if(extent(1)>extent(2))
            return 1;
        else
            return 2;
    }
}

void BoundingBox::AddBox(BoundingBox box)
{
    for(int i=0;i<3;i++)
    {
        if(lowCoord[i] > box.lowCoord[i])
            lowCoord[i] = box.lowCoord[i];
        if(highCoord[i]<box.highCoord[i])
            highCoord[i] = box.highCoord[i];
    }
}

bool BoundingBox::hit(BoundingBox box)const
{
    for(int i=0;i<3;i++)
    {
        if(lowCoord[i] > box.highCoord[i] || highCoord[i]< box.lowCoord[i] )
            return false;
    }
    return true;
}

bool BoundingBox::hit(const Vector3& invDir, const Vector3& origin, real_t t0, real_t t1, const uint32_t dirIsNeg[3])const
{
    float tmin =  (operator[](    dirIsNeg[0]).x - origin.x) * invDir.x;
    float tmax =  (operator[](1-dirIsNeg[0]).x - origin.x) * invDir.x;
    float tymin = (operator[](  dirIsNeg[1]).y - origin.y) * invDir.y;
    float tymax = (operator[](1-dirIsNeg[1]).y - origin.y) * invDir.y;
    if ((tmin > tymax) || (tymin > tmax))
        return false;
    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    // Check for ray intersection against $z$ slab
    float tzmin = (operator[](  dirIsNeg[2]).z - origin.z) * invDir.z;
    float tzmax = (operator[](1-dirIsNeg[2]).z - origin.z) * invDir.z;
    if ((tmin > tzmax) || (tzmin > tmax))
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;

    if(tmax<t0 || tmin>t1)
        return false;

    return tmin <= tmax + 1e-5;//SLOP
}

bool BoundingBox::hit(const Ray& r, real_t t0, real_t t1)const
{
    if(r.d.x == 0 || r.d.y == 0 || r.d.z == 0 )
        return true;

    real_t xmin = (lowCoord.x - r.e.x)/r.d.x;
    real_t xmax = (highCoord.x - r.e.x)/r.d.x;

    real_t ymin = (lowCoord.y - r.e.y)/r.d.y;
    real_t ymax = (highCoord.y - r.e.y)/r.d.y;

    real_t zmin = (lowCoord.z - r.e.z)/r.d.z;
    real_t zmax = (highCoord.z - r.e.z)/r.d.z;
    
    if(xmin>xmax)
        std::swap(xmin,xmax);
    if(ymin>ymax)
        std::swap(ymin,ymax);
    if(zmin>zmax)
        std::swap(zmin,zmax);

    real_t maximin = std::max(std::max(xmin,ymin),zmin);
    real_t minimax = std::min(std::min(xmax,ymax),zmax);
    if(minimax<t0 || maximin>t1)
        return false;

    return maximin <= minimax + 1e-5;
}
}
