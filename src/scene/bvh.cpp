#include "bvh.hpp"
#include "scene/scene.hpp"
#include <SDL_timer.h>

using namespace std;
namespace _462 {
    
    static real_t BoxArea(BoundingBox b)
    {
        real_t v = 1.0;
        for(int i=0;i<3;i++)
        {
            int j = (i+1)%3;
            v+=2*(b.highCoord[i]-b.lowCoord[i])*(b.highCoord[j]-b.lowCoord[j]);
        }
        return v;
    }
    real_t getValue(BoundingBox leftBox, BoundingBox rightBox, const vector<Geometry*> geometries)
    {
        real_t leftArea = BoxArea( leftBox );
        real_t rightArea = BoxArea( leftBox );
        int leftCount = 0, rightCount = 0;

        for(unsigned int i=0;i<geometries.size();i++)
        {
            if(leftBox.hit(geometries[i]->bb))
                leftCount++;
                
            if(rightBox.hit(geometries[i]->bb))
                rightCount++;
        }

        return ( leftArea*leftCount + rightArea*rightCount);
    }
    
    BVHAccel::BVHAccel(const vector<Geometry*>& geometries, uint32_t mp, const string &sm):nodes(NULL) 
    {
        time_t startTime = SDL_GetTicks();

        printf("Building BVH...\n");
        maxPrimsInNode = min(255u, mp);
        primitives = geometries;
        
        if (sm == "middle") splitMethod = SPLIT_MIDDLE;
        else if (sm == "equal")  splitMethod = SPLIT_EQUAL_COUNTS;
        else/*if (sm == "sah")*/         splitMethod = SPLIT_SAH;
        
        if (primitives.size() == 0)
            return;
        
        // Initialize _buildData_ array for primitives
        vector<BVHPrimitiveInfo> buildData;
        buildData.reserve(primitives.size());
        for (uint32_t i = 0; i < primitives.size(); ++i) {
            buildData.push_back(BVHPrimitiveInfo(i, primitives[i]->bb));
        }
        uint32_t totalNodes = 0;
        vector< Geometry* > orderedPrims;
        orderedPrims.reserve(primitives.size());
        BVHBuildNode *root = recursiveBuild(buildData, 0, primitives.size(), &totalNodes, orderedPrims);
        primitives.swap(orderedPrims);
        
         // Compute representation of depth-first traversal of BVH tree
        nodes = new LinearBVHNode[totalNodes];
        //for (uint32_t i = 0; i < totalNodes; ++i)
            //new (&nodes[i]) LinearBVHNode;
        uint32_t offset = 0;
        flattenBVHTree(root, &offset);
        assert(offset == totalNodes);
        
        time_t endTime = SDL_GetTicks();
        
        printf("Done Building BVH in %d \n", endTime-startTime);
    }

    struct CompareToMid {
        CompareToMid(int d, float m) { dim = d; mid = m; }
        int dim;
        float mid;
        bool operator()(const BVHPrimitiveInfo &a) const {
            return a.centroid[dim] < mid;
        }
    };


    struct ComparePoints {
        ComparePoints(int d) { dim = d; }
        int dim;
        bool operator()(const BVHPrimitiveInfo &a,
                        const BVHPrimitiveInfo &b) const {
            return a.centroid[dim] < b.centroid[dim];
        }
    };


    struct CompareToBucket {
        CompareToBucket(int split, int num, int d, const BoundingBox &b)
            : centroidBounds(b)
        { splitBucket = split; nBuckets = num; dim = d; }
        bool operator()(const BVHPrimitiveInfo &p) const;

        int splitBucket, nBuckets, dim;
        const BoundingBox &centroidBounds;
    };
    
    bool CompareToBucket::operator()(const BVHPrimitiveInfo &p) const {
        int b = nBuckets * ((p.centroid[dim] - centroidBounds.lowCoord[dim]) /
            (centroidBounds.highCoord[dim] - centroidBounds.lowCoord[dim]));
        if (b == nBuckets) b = nBuckets-1;
        assert(b >= 0 && b < nBuckets);
        return b <= splitBucket;
    }

    BVHAccel::~BVHAccel() {
        if(nodes)
        {
            delete []nodes;
            nodes = NULL;
        }
    }
    BVHBuildNode *BVHAccel::recursiveBuild( vector<BVHPrimitiveInfo> &buildData, uint32_t start,
        uint32_t end, uint32_t *totalNodes, vector<Geometry* > &orderedPrims){
        
        assert(start != end);
        (*totalNodes)++;
        BVHBuildNode *node = new BVHBuildNode();
        // Compute bounds of all primitives in BVH node
        BoundingBox bbox;
        for (uint32_t i = start; i < end; ++i)
            bbox.AddBox(buildData[i].bounds);

        uint32_t nPrimitives = end - start;
        if (nPrimitives == 1) {
            // Create leaf _BVHBuildNode_
            uint32_t firstPrimOffset = orderedPrims.size();
            for (uint32_t i = start; i < end; ++i) {
                uint32_t primNum = buildData[i].primitiveNumber;
                orderedPrims.push_back(primitives[primNum]);
            }
            node->InitLeaf(firstPrimOffset, nPrimitives, bbox);
        }
        else {
            // Compute bound of primitive centroids, choose split dimension _dim_
            BoundingBox centroidBounds;
            for (uint32_t i = start; i < end; ++i)
                centroidBounds.AddPoint(buildData[i].centroid);
            int dim = centroidBounds.MaximumExtent();

            // Partition primitives into two sets and build children
            uint32_t mid = (start + end) / 2;
            if (centroidBounds.highCoord[dim] == centroidBounds.lowCoord[dim]) {
                // Create leaf _BVHBuildNode_
                uint32_t firstPrimOffset = orderedPrims.size();
                for (uint32_t i = start; i < end; ++i) {
                    uint32_t primNum = buildData[i].primitiveNumber;
                    orderedPrims.push_back(primitives[primNum]);
                }
                node->InitLeaf(firstPrimOffset, nPrimitives, bbox);
                return node;
            }

            // Partition primitives based on _splitMethod_
            switch (splitMethod) {
            case SPLIT_MIDDLE: {
                // Partition primitives through node's midpoint
                double pmid = .5f * (centroidBounds.lowCoord[dim] + centroidBounds.highCoord[dim]);
                BVHPrimitiveInfo *midPtr = std::partition(&buildData[start],
                                                          &buildData[end-1]+1,
                                                          CompareToMid(dim, pmid));
                mid = midPtr - &buildData[0];
                if (mid != start && mid != end)
                    // for lots of prims with large overlapping bounding boxes, this
                    // may fail to partition; in that case don't break and fall through
                    // to SPLIT_EQUAL_COUNTS
                    break;
            }
            case SPLIT_EQUAL_COUNTS: {
                // Partition primitives into equally-sized subsets
                mid = (start + end) / 2;
                std::nth_element(&buildData[start], &buildData[mid],
                                 &buildData[end-1]+1, ComparePoints(dim));
                break;
            }
            case SPLIT_SAH: default: {
                // Partition primitives using approximate SAH
                if (nPrimitives <= 4) {
                    // Partition primitives into equally-sized subsets
                    mid = (start + end) / 2;
                    std::nth_element(&buildData[start], &buildData[mid],
                                     &buildData[end-1]+1, ComparePoints(dim));
                }
                else {
                    // Allocate _BucketInfo_ for SAH partition buckets
                    const int nBuckets = 12;
                    struct BucketInfo {
                        BucketInfo() { count = 0; }
                        int count;
                        BoundingBox bounds;
                    };
                    BucketInfo buckets[nBuckets];

                    // Initialize _BucketInfo_ for SAH partition buckets
                    for (uint32_t i = start; i < end; ++i) {
                        int b = nBuckets *
                            ((buildData[i].centroid[dim] - centroidBounds.lowCoord[dim]) /
                             (centroidBounds.highCoord[dim] - centroidBounds.lowCoord[dim]));
                        if (b == nBuckets) b = nBuckets-1;
                        assert(b >= 0 && b < nBuckets);
                        buckets[b].count++;
                        buckets[b].bounds.AddBox(buildData[i].bounds);
                    }

                    // Compute costs for splitting after each bucket
                    float cost[nBuckets-1];
                    for (int i = 0; i < nBuckets-1; ++i) {
                        BoundingBox b0, b1;
                        int count0 = 0, count1 = 0;
                        for (int j = 0; j <= i; ++j) {
                            b0.AddBox(buckets[j].bounds);
                            count0 += buckets[j].count;
                        }
                        for (int j = i+1; j < nBuckets; ++j) {
                            b1.AddBox(buckets[j].bounds);
                            count1 += buckets[j].count;
                        }
                        cost[i] = .125f + (count0*b0.SurfaceArea() + count1*b1.SurfaceArea()) /
                                  bbox.SurfaceArea();
                    }

                    // Find bucket to split at that minimizes SAH metric
                    float minCost = cost[0];
                    uint32_t minCostSplit = 0;
                    for (int i = 1; i < nBuckets-1; ++i) {
                        if (cost[i] < minCost) {
                            minCost = cost[i];
                            minCostSplit = i;
                        }
                    }

                    // Either create leaf or split primitives at selected SAH bucket
                    if (nPrimitives > maxPrimsInNode ||
                        minCost < nPrimitives) {
                        BVHPrimitiveInfo *pmid = std::partition(&buildData[start],
                            &buildData[end-1]+1,
                            CompareToBucket(minCostSplit, nBuckets, dim, centroidBounds));
                        mid = pmid - &buildData[0];
                    }
                
                    else {
                        // Create leaf _BVHBuildNode_
                        uint32_t firstPrimOffset = orderedPrims.size();
                        for (uint32_t i = start; i < end; ++i) {
                            uint32_t primNum = buildData[i].primitiveNumber;
                            orderedPrims.push_back(primitives[primNum]);
                        }
                        node->InitLeaf(firstPrimOffset, nPrimitives, bbox);
                        return node;
                    }
                }
                break;
            }
            }
            node->InitInterior(dim,
                               recursiveBuild(buildData, start, mid,
                                              totalNodes, orderedPrims),
                               recursiveBuild(buildData, mid, end,
                                              totalNodes, orderedPrims));
        }
        return node;
    }
    uint32_t BVHAccel::flattenBVHTree(BVHBuildNode *node, uint32_t *offset)
    {
        LinearBVHNode *linearNode = &nodes[*offset];
        linearNode->bounds = node->bounds;
        uint32_t myOffset = (*offset)++;
        if (node->nPrimitives > 0) {
            assert(!node->children[0] && !node->children[1]);
            linearNode->primitivesOffset = node->firstPrimOffset;
            linearNode->nPrimitives = node->nPrimitives;
        }
        else {
            // Creater interior flattened BVH node
            linearNode->axis = node->splitAxis;
            linearNode->nPrimitives = 0;
            flattenBVHTree(node->children[0], offset);
            linearNode->secondChildOffset = flattenBVHTree(node->children[1],
                                                           offset);
        }
        return myOffset;
    }
    
    Geometry* BVHAccel::hit(const Ray& ray, const real_t t0, const real_t t1, hitRecord& h, bool fullRecord) const
    {
        if(!nodes) return NULL;
        Vector3 invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
        uint32_t dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
        // Follow ray through BVH nodes to find primitive intersections
        uint32_t todoOffset = 0, nodeNum = 0;
        uint32_t todo[64];

        real_t minT = t1;
        hitRecord h1;
        Geometry* obj = NULL;
        while (true) {
            const LinearBVHNode *node = &nodes[nodeNum];
            // Check ray against BVH node
            //if (node->bounds.hit(ray, t0, minT)) {
            if (node->bounds.hit(invDir,ray.e, t0, minT,dirIsNeg)) {
                if (node->nPrimitives > 0) {
                    
                    for (uint32_t i = 0; i < node->nPrimitives; ++i)
                    {
                        if (primitives[node->primitivesOffset+i]->hit(ray, t0, minT, h1, fullRecord))
                        {
                            if(minT>h1.t)
                            {
                                obj = primitives[node->primitivesOffset+i];
                                minT = h1.t;
                                h = h1;
                                if(!fullRecord) return obj;
                            }
                        }
                    }
                    if (todoOffset == 0) break;
                    nodeNum = todo[--todoOffset];
                }
                else {
                    // Put far BVH node on _todo_ stack, advance to near node
                    if (dirIsNeg[node->axis]) {
                       todo[todoOffset++] = nodeNum + 1;
                       nodeNum = node->secondChildOffset;
                    }
                    else {
                       todo[todoOffset++] = node->secondChildOffset;
                       nodeNum = nodeNum + 1;
                    }
                }
            }
            else {
                if (todoOffset == 0) break;
                nodeNum = todo[--todoOffset];
            }
        }
        return obj;
    }
}/* _462 */
