#ifndef _462_BVH_HPP_
#define _462_BVH_HPP_

#include "math/vector.hpp"
#include <vector>
#include "scene/BoundingBox.hpp"
namespace _462 {
	
	class Geometry;
	struct hitRecord;
	
	class bvhNode
	{
	public:
		enum BoundaryType { xBoundary, yBoundary, zBoundary };
		bvhNode(std::vector<Geometry*> geometries);
		~bvhNode();
		Geometry* hit(const Ray& r, const real_t t0, const real_t t1, hitRecord& h, bool fullRecord) const;
	private:

		void GetChildNodeBounds(const std::vector<Geometry*> geometries, BoundingBox& box1, BoundingBox& box2)const;
		BoundingBox bb;
		bvhNode *left, *right;
		std::vector<Geometry*> entries;
		int size;
	};
}/* _462 */

#endif /* _462_BVH_HPP_ */