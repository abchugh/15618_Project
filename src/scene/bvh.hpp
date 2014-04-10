#ifndef _462_BVH_HPP_
#define _462_BVH_HPP_

#include "math/vector.hpp"
#include <vector>
#include <deque>
#include <queue>

#include "scene/BoundingBox.hpp"
namespace _462 {
	
	class Geometry;
	struct hitRecord;
	struct BVHBuildNode
	{
		// BVHBuildNode Public Methods
		BVHBuildNode(BVHBuildNode *p, bool firstChild):parent(p),isFirstChild(firstChild)
		{children[0] = children[1] = NULL; childComplete[0]=childComplete[1]=false;}
		
		void InitLeaf(uint32_t first, uint32_t n, const BoundingBox &b) {
			firstPrimOffset = first;
			nPrimitives = n;
			bounds = b;
		}
		void InitInterior(BVHBuildNode *c0, BVHBuildNode *c1) {
			children[0] = c0;
			children[1] = c1;
			bounds = c0->bounds;
			bounds.AddBox(c1->bounds);
			nPrimitives = 0;
		}
		BoundingBox bounds;
		BVHBuildNode *children[2];
		BVHBuildNode *parent;

		bool isFirstChild; //true if first, false if second child of its parent
		bool childComplete[2];

		uint32_t splitAxis, firstPrimOffset, nPrimitives;
	};
	
	struct queueData
	{
		uint32_t start, end;
		BVHBuildNode* parent;
		bool isFirstChild;
		bool* isValid;
		bool operator<(const queueData& el)const
		{
			return end-start<el.end-el.start;
		}
	};

	// BVHAccel Local Declarations
	struct BVHPrimitiveInfo {
		BVHPrimitiveInfo() { }
		BVHPrimitiveInfo(int pn, const BoundingBox &b)
			: primitiveNumber(pn), bounds(b) {
				centroid = .5f * b.lowCoord + .5f * b.highCoord;
		}
		uint32_t primitiveNumber;
		Vector3 centroid;
		BoundingBox bounds;
	};
	struct LinearBVHNode {
		BoundingBox bounds;
		union {
			uint32_t primitivesOffset;    // leaf
			uint32_t secondChildOffset;   // interior
		};

		uint8_t nPrimitives;  // 0 -> interior node
		uint8_t axis;         // interior node: xyz
		uint8_t pad[2];       // ensure 32 byte total size
	};
	const int MAX_THREADS = 128;
	class BVHAccel
	{
	public:
		BVHAccel(const std::vector<Geometry*>& geometries, uint32_t maxPrims = 1,
             const std::string &sm = "sah");

		//bvhNode();
		~BVHAccel();
		Geometry* hit(const Ray& r, const real_t t0, const real_t t1, hitRecord& h, bool fullRecord) const;
	private:
		BVHBuildNode *recursiveBuild(std::vector<BVHPrimitiveInfo> &buildData, uint32_t start, uint32_t end,
			uint32_t *totalNodes, std::vector<Geometry*> &orderedPrims, BVHBuildNode *parent = NULL, bool firstChild = true);
	    BVHBuildNode *fastRecursiveBuild(std::vector<BVHPrimitiveInfo> &buildData, uint32_t start, uint32_t end,
			uint32_t *totalNodes, std::vector<Geometry*> &orderedPrims, BVHBuildNode *parent = NULL, bool firstChild = true);
	    void buildLeaf(std::vector<BVHPrimitiveInfo> &buildData, uint32_t start,
        uint32_t end, std::vector<Geometry* > &orderedPrims, BVHBuildNode *node, const BoundingBox& bbox);
		uint32_t flattenBVHTree(BVHBuildNode *node, uint32_t *offset);
		
		uint32_t maxPrimsInNode;
		enum SplitMethod { SPLIT_MIDDLE, SPLIT_EQUAL_COUNTS, SPLIT_SAH };
		SplitMethod splitMethod;
		std::vector<Geometry*> primitives;
		LinearBVHNode *nodes;
		std::deque<queueData> q[MAX_THREADS];
		std::priority_queue<queueData> pq;
	};
}/* _462 */

#endif /* _462_BVH_HPP_ */

