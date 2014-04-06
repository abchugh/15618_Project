#ifndef _462_BOUNDING_BOX_HPP_
#define _462_BOUNDING_BOX_HPP_
#include "math/vector.hpp"
namespace _462 {
class Ray;
class BoundingBox
{
public:
	BoundingBox();


	//Extend the bounding box to accomodate the point
	void AddPoint(Vector3 point);
	//combine with another bounding box to form a bigger bounding volume
	void AddBox(BoundingBox box);
	//hit testing on the box
	bool hit(const Ray& r, real_t t0, real_t t1)const;
	//check if a box intersects with another
	bool hit(BoundingBox box)const;

	int MaximumExtent()const;
	Vector3 lowCoord, highCoord;
	real_t SurfaceArea()const;
private:
	inline real_t extent(int dim)const
	{
		return highCoord[dim]-lowCoord[dim];
	}
};
}
#endif