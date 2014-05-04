
#include "scene/scene.hpp"
#include "scene/BoundingBox.hpp"
#include "scene/ray.hpp"

namespace _462 {
BoundingBox::BoundingBox():lowCoord(BIG_NUMBER,BIG_NUMBER,BIG_NUMBER), highCoord(-BIG_NUMBER,-BIG_NUMBER,-BIG_NUMBER)
{
    //surfaceArea = -1;
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

#ifdef ISPC_SOA
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

bool BoundingBox::hit(const Frustum& frustum) const {
    // n/p vertex
    Vector3 p;
    Vector3 n;
    Vector3 center = 0.5 * (lowCoord + highCoord);
    Vector3 extent = 0.5 * (highCoord - lowCoord);
    
    for (int i = 0; i < 6; i++) {
	
	int x_sign = (-1 + 2 * (frustum.planes[i].norm.x >= 0));
	int y_sign = (-1 + 2 * (frustum.planes[i].norm.y >= 0));
	int z_sign = (-1 + 2 * (frustum.planes[i].norm.z >= 0));

	p.x = center.x + x_sign * extent.x;
	p.y = center.y + y_sign * extent.y;
	p.z = center.z + z_sign * extent.z;

	// p vertex is outside
	if (dot(p - frustum.planes[i].point, frustum.planes[i].norm)
	    < 0) {
	    return false;
	}

	n.x = center.x - x_sign * extent.x;
	n.y = center.y - y_sign * extent.y;
	n.z = center.z - z_sign * extent.z;

	// n vertex is outside, given p vertex is inside
	if (dot(n - frustum.planes[i].point, frustum.planes[i].norm)
	    < 0)
	    return true;

    }
    
    return true;
}

void BoundingBox::hit(const Packet& packet, int start, int end, float *t0, float *t1, char *result) const {
    /*
    __m128 zeros = _mm_setzero_ps();
    __m128 ones = _mm_set1_ps(1);
    __m128i alltrue = _mm_set1_epi32(0xffffffff);
    __m128i local;

    // Handle the boundary condition by allocating result with size divisible
    for (int i = start; i < end; i += LANES) {
	__m128 d_x = _mm_load_ps(packet.d_x + i);
	__m128 d_y = _mm_load_ps(packet.d_y + i);

	__m128 invDir_x = _mm_rcp_ps(d_x);
	__m128 invDir_y = _mm_rcp_ps(d_y);

	__m128 dirIsNeg_x = _mm_and_ps(_mm_cmplt_ps(zeros, invDir_x), ones);
	__m128 dirIsNeg_y = _mm_and_ps(_mm_cmplt_ps(zeros, invDir_y), ones);
	__m128 e_x = _mm_load_ps(packet.e_x + i);
	__m128 e_y = _mm_load_ps(packet.e_y + i);

	__m128 tmin = _mm_mul_ps(_mm_sub_ps(dirIsNeg_x, e_x), invDir_x);
	__m128 tmax = _mm_mul_ps(_mm_sub_ps(_mm_sub_ps(ones, dirIsNeg_x), e_x),
				 invDir_x);

	__m128 tymin = _mm_mul_ps(_mm_sub_ps(dirIsNeg_y, e_y), invDir_y);
	__m128 tymax = _mm_mul_ps(_mm_sub_ps(_mm_sub_ps(ones, dirIsNeg_y), e_y),
				 invDir_y);

	local = _mm_castps_si128(_mm_or_ps(_mm_cmpgt_ps(tmin, tymax), _mm_cmpgt_ps(tymin, tmax)));

	int count_zero = _mm_movemask_epi8(local);
	if (count_zero == 0) {
	    _mm_maskmoveu_si128(local, alltrue, result + i);
	    continue;
	}

	__m128 cmptemp = _mm_cmpgt_ps(tymin, tmin);

	tmin = _mm_add_ps(_mm_and_ps(cmptemp, tymin),
			  _mm_and_ps(_mm_xor_ps(cmptemp, (__m128)alltrue), tmin));
	cmptemp = _mm_cmplt_ps(tymax, tmax);
	tmax = _mm_add_ps(_mm_and_ps(cmptemp, tymax),
			  _mm_and_ps(_mm_xor_ps(cmptemp, (__m128)alltrue), tmax));

	// Check for z-axis
	__m128 d_z = _mm_load_ps(packet.d_z + i);
	__m128 invDir_z = _mm_rcp_ps(d_z);
	__m128 dirIsNeg_z = _mm_and_ps(_mm_cmplt_ps(zeros, invDir_z), ones);
	__m128 e_z = _mm_load_ps(packet.e_z + i);

	__m128 tzmin = _mm_mul_ps(_mm_sub_ps(dirIsNeg_z, e_z), invDir_z);
	__m128 tzmax = _mm_mul_ps(_mm_sub_ps(_mm_sub_ps(ones, dirIsNeg_z), e_z),
				 invDir_z);
	__m128i localz = _mm_castps_si128(_mm_or_ps(_mm_cmpgt_ps(tmin, tzmax), _mm_cmpgt_ps(tzmin, tmax)));
	local = _mm_or_si128(localz, local);

	count_zero = _mm_movemask_epi8(local);
	if (count_zero == 0) {
	    _mm_maskmoveu_si128(local, alltrue, result + i);
	    continue;
	}
	
	cmptemp = _mm_cmpgt_ps(tzmin, tmin);
	tmin = _mm_add_ps(_mm_and_ps(cmptemp, tzmin),
			  _mm_and_ps(_mm_xor_ps(cmptemp, (__m128)alltrue), tmin));
	cmptemp = _mm_cmplt_ps(tzmax, tmax);
	tmax = _mm_add_ps(_mm_and_ps(cmptemp, tzmax),
			  _mm_and_ps(_mm_xor_ps(cmptemp, (__m128)alltrue), tmax));

	__m128 t0_local = _mm_load_ps(t0 + i);
	__m128 t1_local = _mm_load_ps(t1 + i);

	localz = _mm_castps_si128(_mm_or_ps(_mm_cmpgt_ps(tmin, t1_local), _mm_cmpgt_ps(t0_local, tmax)));
	local = _mm_or_si128(localz, local);

	count_zero = _mm_movemask_epi8(local);
	if (count_zero == 0) {
	    _mm_maskmoveu_si128(local, alltrue, result + i);
	    continue;
	}

	__m128 threshold = _mm_set1_ps(1e-5);
	localz = _mm_castps_si128(_mm_cmple_ps(tmin, _mm_add_ps(tmax, threshold)));
	_mm_maskmoveu_si128(local, alltrue, result + i);
    }
    */
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
