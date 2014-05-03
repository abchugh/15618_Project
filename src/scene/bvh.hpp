#ifndef _462_BVH_HPP_
#define _462_BVH_HPP_

#include "math/vector.hpp"
#include "partition_ispc.h"
#include "hit_ispc.h"
#include "ispc_switch.h"

#include <cstdlib>
#include <vector>
#include <deque>
#include <queue>

#include "scene/BoundingBox.hpp"
namespace _462 {

    //#undef ISPC
#define NUM_IN_PRE_QUEUE 7
    //#define ENABLED_TIME_LOGS
    const int MAX_THREADS = 128;

    class Geometry;
    struct hitRecord;
    struct Packet;

    struct BVHBuildNode
    {
        // BVHBuildNode Public Methods
        BVHBuildNode(BVHBuildNode *p, bool firstChild):parent(p),isFirstChild(firstChild)
        {
            children[0] = children[1] = NULL; 
            childComplete[0]=childComplete[1]=false; 
            nPrimitives = 0;
        }

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

    const int DONT_USE_AND_DELETE = 0;
    const int USE_AND_DONT_DELETE = 1;
    const int USE_AND_DELETE = 2;

    struct queueData
    {
        uint32_t start, end;
        BVHBuildNode* parent;
        bool isFirstChild;

        char* status;//when in both q and pq-> 1-use and dont delete, 0 don't use and delete,   when only in one of q and pq-> 2-use and delete
        bool operator<(const queueData& el)const
        {
            return end-start<el.end-el.start;
        }
        static bool updateStatus(queueData&);
    };

    // BVHAccel Local Declarations
    struct BVHPrimitiveInfo {
        BVHPrimitiveInfo() { }
        BVHPrimitiveInfo(int pn, const BoundingBox &b)
            : primitiveNumber(pn),bounds(b) {
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



    struct TraversalNode {
        uint32_t node_index;
        uint32_t active;

        TraversalNode() {
            node_index = active = -1;
        }

        TraversalNode(uint32_t index, uint32_t act) {
            node_index = index;
            active = act;
        }
    };

#ifdef ISPC_SOA
    typedef ispc::BVHPrimitiveInfoList PrimitiveInfoList;
#elif defined(ISPC_AOS)
    typedef ispc::BVHPrimitiveInfo PrimitiveInfo;
    typedef std::vector<ispc::BVHPrimitiveInfo> PrimitiveInfoList;
#else
    typedef _462::BVHPrimitiveInfo PrimitiveInfo;
    typedef std::vector<_462::BVHPrimitiveInfo> PrimitiveInfoList;
#endif

    class BVHAccel
    {
    public:
        BVHAccel(const std::vector<Geometry*>& geometries, uint32_t maxPrims = 1,
            const std::string &sm = "sah");

        //bvhNode();
        ~BVHAccel();

        void threadedSubtreeBuild(PrimitiveInfoList &buildData, std::vector< Geometry* > &orderedPrims, uint32_t *totalNodes);
        Geometry* hit(const Ray& r, const real_t t0, const real_t t1, hitRecord& h, bool fullRecord) const;
        void traverse(const Packet& packet, std::vector<hitRecord>& records, const real_t t0, const real_t t1) const;

    private:
        BVHBuildNode *recursiveBuild(PrimitiveInfoList &buildData, uint32_t start, uint32_t end,
            uint32_t *totalNodes, std::vector<Geometry*> &orderedPrims, BVHBuildNode *parent = NULL, bool firstChild = true);
        BVHBuildNode *fastRecursiveBuild(PrimitiveInfoList &buildData, uint32_t start, uint32_t end,
            uint32_t *totalNodes, std::vector<Geometry*> &orderedPrims, BVHBuildNode *parent = NULL, bool firstChild = true);
        void buildLeaf(PrimitiveInfoList &buildData, uint32_t start,
            uint32_t end, std::vector<Geometry* > &orderedPrims, BVHBuildNode *node, const BoundingBox& bbox);
        uint32_t flattenBVHTree(BVHBuildNode *node, uint32_t *offset);

        uint32_t getFirstHit(const Packet& packet, const BoundingBox box, uint32_t active, 
            uint32_t *dirIsNeg, real_t t0, real_t t1) const;

        uint32_t maxPrimsInNode;
        enum SplitMethod { SPLIT_MIDDLE, SPLIT_EQUAL_COUNTS, SPLIT_SAH };
        SplitMethod splitMethod;
        std::vector<Geometry*> primitives;
        LinearBVHNode *nodes;
        BVHBuildNode *root;
        std::priority_queue<queueData> pq;
        std::deque<queueData> q[MAX_THREADS];

    };

}/* _462 */

#endif /* _462_BVH_HPP_ */

