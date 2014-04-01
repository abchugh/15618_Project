
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
