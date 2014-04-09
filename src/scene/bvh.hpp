#ifndef _462_BVH_HPP_
#define _462_BVH_HPP_

#include "math/vector.hpp"
#include <vector>
#include <stack>
#include <pthread.h>
#include "scene/BoundingBox.hpp"
namespace _462 {
	
	class Geometry;
	struct hitRecord;
	
	struct BVHBuildNode
	{
		// BVHBuildNode Public Methods
        BVHBuildNode() { children[0] = children[1] = NULL; }
        
        void InitLeaf(uint32_t first, uint32_t n, const BoundingBox &b) {
            firstPrimOffset = first;
            nPrimitives = n;
            bounds = b;
        }
        void InitInterior(uint32_t axis, BVHBuildNode *c0, BVHBuildNode *c1) {
            children[0] = c0;
            children[1] = c1;
            bounds = c0->bounds;
            bounds.AddBox(c1->bounds);
            splitAxis = axis;
            nPrimitives = 0;
        }
	
	void InitChild(BVHBuildNode *child, bool left) {
	    children[(left != true)] = child;
	    bounds.AddBox(child->bounds);
	}

        BoundingBox bounds;
        BVHBuildNode* children[2];
        uint32_t splitAxis, firstPrimOffset, nPrimitives;
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

    struct BucketInfo {
	BucketInfo() { count = 0; }
	int count;
	BoundingBox bounds;
	// TODO: Padding?
    };

    struct BuildArg {
	std::vector<BVHPrimitiveInfo> *buildDataPtr;
	uint32_t *totalNodes;
	std::vector<Geometry*> *orderedPrimsPtr;
	std::vector<Geometry*> *primitivesPtr;
	uint32_t maxPrimsInNode;
	int threadId;
	int threadNum;
	pthread_barrier_t *barrierPtr;
	std::stack<std::pair<int32_t, int32_t>> *infoStackPtr;
	BucketInfo* buckets;
	float* cost;
	int *dimPtr;
	// TODO: Padding
	BoundingBox* bboxes;
	BoundingBox* centroidBounds;
    };

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
                     uint32_t *totalNodes, std::vector<Geometry*> &orderedPrims);
	BVHBuildNode* iterativeBuild(std::vector<BVHPrimitiveInfo> &buildData, 
				     uint32_t *totalNodes, std::vector<Geometry*> &orderedPrims);
        BVHBuildNode *SAHSplit(std::vector<BVHPrimitiveInfo> &buildData, uint32_t start, uint32_t end, 
			       BoundingBox bbox, BoundingBox centroidBounds,
			       int dim, 
			       std::vector<Geometry*> &orderedPrims, BVHBuildNode *node, uint32_t& mid);

        uint32_t flattenBVHTree(BVHBuildNode *node, uint32_t *offset);

	uint32_t maxPrimsInNode;
        enum SplitMethod { SPLIT_MIDDLE, SPLIT_EQUAL_COUNTS, SPLIT_SAH };
        SplitMethod splitMethod;
	std::vector<Geometry*> primitives;
        LinearBVHNode *nodes;
    };
}/* _462 */

#endif /* _462_BVH_HPP_ */
