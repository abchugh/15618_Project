#include "bvh.hpp"
#ifndef ISPC

#include "scene/scene.hpp"

using namespace std;
namespace _462 {

    struct CompareToVal {
        CompareToVal(int d, float v) { dim = d; val = v; }
        int dim;
        float val;
        bool operator()(const PrimitiveInfo &a) const {
            return a.centroid[dim] < val;
        }
    };
    
    struct ComparePoints {
        ComparePoints(int d) { dim = d; }
        int dim;
        bool operator()(const PrimitiveInfo &a,
            const PrimitiveInfo &b) const {
                return a.centroid[dim] < b.centroid[dim];
        }
    };

    struct CompareToBucket {
        CompareToBucket(int split, int num, int d, const BoundingBox &b)
            : centroidBounds(b)
        { splitBucket = split; nBuckets = num; dim = d; }
        bool operator()(const PrimitiveInfo &p) const;

        int splitBucket, nBuckets, dim;
        const BoundingBox &centroidBounds;
    };
    bool CompareToBucket::operator()(const PrimitiveInfo &p) const {
        int b = nBuckets * ((p.centroid[dim] - centroidBounds.lowCoord[dim]) / centroidBounds.extent(dim));
        if (b == nBuckets) b = nBuckets-1;
        assert(b >= 0 && b < nBuckets);
        return b <= splitBucket;
    }
    unsigned int partition(int start, int end, int dim, float mid, PrimitiveInfoList& buildData, PrimitiveInfoList& buildDataBuffer) {
       PrimitiveInfo* pMid = std::partition(&buildData[start], &buildData[end-1]+1, CompareToVal(dim, mid));
       return pMid - &buildData[0];
    }
    
    void initPrimitiveInfoList(const std::vector<Geometry*>& primitives, PrimitiveInfoList& list, bool allocateOnly)
    {
        list.resize(primitives.size());
        if(!allocateOnly)
	    /*
	    for (uint32_t i = 0; i < primitives.size(); ++i) {
			list.push_back(PrimitiveInfo(i, primitives[i]->bb));
			}*/
	    #pragma omp parallel for
	    for (uint32_t i = 0; i < primitives.size(); ++i) {
		list[i] = PrimitiveInfo(i, primitives[i]->bb);
	    }
		
    }
    void AddBox(const PrimitiveInfoList& buildData, int index, BoundingBox & box)
    {
        box.AddBox(buildData[index].bounds);
    }
    void AddCentroid(const PrimitiveInfoList& buildData, int index, BoundingBox & box)
    {
        box.AddPoint(buildData[index].centroid);
    }
    uint32_t SplitEqually(PrimitiveInfoList& buildData, uint32_t start, uint32_t end, uint32_t dim)
    {
        uint32_t mid = (start + end) / 2;
        std::nth_element(&buildData[start], &buildData[mid],
            &buildData[end-1]+1, ComparePoints(dim));
        return mid;
    }
    
    void clearList(PrimitiveInfoList& buildData)
    {
        buildData.clear();
    }
    float getCentroidDim(const PrimitiveInfoList& buildData, int index, int dim)
    {
        return buildData[index].centroid[dim];
    }
}
#endif
