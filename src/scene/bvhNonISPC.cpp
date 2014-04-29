#include "bvh.hpp"
#ifndef ISPC_SOA

#include <iterator>
#include <cstring>
#include "parallel/partition.h"
#include "scene/scene.hpp"

using namespace std;
namespace _462 {
#define BOX_THRESHOLD 10
#define CENTROID_THRESHOLD 10
    struct CompareToVal {
        CompareToVal(int d, float v) { dim = d; val = v; }
        int dim;
        float val;
        bool operator()(const PrimitiveInfo &a) const {
	    #ifdef ISPC_AOS
            return a.centroid.v[dim] < val;
	    #else
	    return a.centroid[dim] < val;
	    #endif
        }
    };
    
    struct ComparePoints {
        ComparePoints(int d) { dim = d; }
        int dim;
        bool operator()(const PrimitiveInfo &a,
            const PrimitiveInfo &b) const {
            #ifdef ISPC_AOS
	    return a.centroid.v[dim] < b.centroid.v[dim];
	    #else
	    return a.centroid[dim] < b.centroid[dim];
	    #endif
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
	#ifdef ISPC_AOS
        int b = nBuckets * ((p.centroid.v[dim] - centroidBounds.lowCoord[dim]) / centroidBounds.extent(dim));
	#else
	int b = nBuckets * ((p.centroid[dim] - centroidBounds.lowCoord[dim]) / centroidBounds.extent(dim));
	#endif
        if (b == nBuckets) b = nBuckets-1;
        assert(b >= 0 && b < nBuckets);
        return b <= splitBucket;
    }
    unsigned int partition(int start, int end, int dim, float mid, PrimitiveInfoList& buildData, PrimitiveInfoList& buildDataBuffer) {
	int result;
	int num_threads = omp_get_max_threads();

	if (end - start > 2000) {
	    std::iterator_traits<PrimitiveInfo*>::difference_type pMid = 
		__gnu_parallel::__parallel_partition(&buildData[start],
						     &buildData[end], CompareToVal(dim, mid), 12);
	    result = pMid + start;
	}
	else {
	    PrimitiveInfo* pMid = std::partition(&buildData[start], &buildData[end-1]+1, CompareToVal(dim, mid));
	    result = pMid - &buildData[0];
	}

	return result;
    }

    template<typename ISPC_T, typename MATH_T, int Dim = 3>
    void to_ispc_vector(ISPC_T &ispc_v, MATH_T v) {
	for (int i = 0; i < Dim; i++) {
	    ispc_v.v[i] = v[i];
	}
    }

    template<typename ISPC_T, typename MATH_T, int Dim = 3>
    void to_math_vector(MATH_T &v, ISPC_T ispc_v) {
	for (int i = 0; i < Dim; i++) {
	    v[i] = ispc_v.v[i];
	}
    }
    
    void initPrimitiveInfoList(const std::vector<Geometry*>& primitives, PrimitiveInfoList& list, bool allocateOnly)
    {
        list.resize(primitives.size());
        if(!allocateOnly)
            #pragma omp parallel for 
	    for (uint32_t i = 0; i < primitives.size(); ++i) {
		#ifndef ISPC_AOS
		list[i] = PrimitiveInfo(i, primitives[i]->bb);
		#else
		
		float packed_num;
		memcpy(&packed_num, &i, sizeof(int));
		Vector4 low_num(primitives[i]->bb.lowCoord, packed_num);
		Vector4 high(primitives[i]->bb.highCoord, 0.f);
		Vector4 centroid(.5f * primitives[i]->bb.lowCoord +
				 .5f * primitives[i]->bb.highCoord, 0);

		to_ispc_vector<ispc::float4, Vector4, 4>(list[i].lowCoord, low_num);
		to_ispc_vector<ispc::float4, Vector4, 4>(list[i].highCoord, -high);
		to_ispc_vector<ispc::float4, Vector4, 4>(list[i].centroid, centroid);
		to_ispc_vector<ispc::float4, Vector4, 4>(list[i].negCentroid, -centroid);
		
		#endif
	    }
		
    }
    
#ifndef ISPC_AOS
    void AddBox(const PrimitiveInfoList& buildData, uint32_t index, BoundingBox & box)
    {
        box.AddBox(buildData[index].bounds);
    }
    void AddBox(const PrimitiveInfoList& buildData, uint32_t start, uint32_t end, BoundingBox & box)
    {
        for(uint32_t i=start; i<end; i++)
            box.AddBox(buildData[i].bounds);
    }
    void AddCentroid(const PrimitiveInfoList& buildData, uint32_t start, uint32_t end, BoundingBox & box)
    {
        for(uint32_t i=start; i<end; i++)
            box.AddPoint(buildData[i].centroid);
    }
#else    
    
    void AddBox(const PrimitiveInfoList& buildData, uint32_t index, BoundingBox & box)
    {
	BoundingBox bbox;
	to_math_vector<ispc::float4, Vector3, 3>(bbox.lowCoord, buildData[index].lowCoord);
	to_math_vector<ispc::float4, Vector3, 3>(bbox.highCoord, buildData[index].highCoord);
	bbox.highCoord = -bbox.highCoord;
        box.AddBox(bbox);
    }
    void AddBox(const PrimitiveInfoList& buildData, uint32_t start, uint32_t end, BoundingBox & box)
    {
	if (end - start > BOX_THRESHOLD) {
	    ispc::float3 lowCoord;
	    ispc::float3 highCoord;

	    to_ispc_vector<ispc::float3, Vector3, 3>(lowCoord, box.lowCoord);
	    to_ispc_vector<ispc::float3, Vector3, 3>(highCoord, box.highCoord);
	    ispc::AddBox(lowCoord, highCoord, &buildData[0], start, end);
	    to_math_vector<ispc::float3, Vector3, 3>(box.lowCoord, lowCoord);
	    to_math_vector<ispc::float3, Vector3, 3>(box.highCoord, highCoord);
	}
	else {
	    for (int i = start; i < end; i++) {
		BoundingBox bbox;
		to_math_vector<ispc::float4, Vector3, 3>(bbox.lowCoord, buildData[i].lowCoord);
		to_math_vector<ispc::float4, Vector3, 3>(bbox.highCoord, buildData[i].highCoord);
		bbox.highCoord = -bbox.highCoord;
		box.AddBox(bbox);
	    }
	}
    }
    void AddCentroid(const PrimitiveInfoList& buildData, uint32_t start, uint32_t end, BoundingBox & box)
    {
	if (end - start > CENTROID_THRESHOLD) {
	    ispc::float3 lowCoord;
	    ispc::float3 highCoord;

	    to_ispc_vector<ispc::float3, Vector3, 3>(lowCoord, box.lowCoord);
	    to_ispc_vector<ispc::float3, Vector3, 3>(highCoord, box.highCoord);

	    ispc::AddCentroid(lowCoord, highCoord, &buildData[0], start, end);
	    to_math_vector<ispc::float3, Vector3, 3>(box.lowCoord, lowCoord);
	    to_math_vector<ispc::float3, Vector3, 3>(box.highCoord, highCoord);
	}
	else {
	    for(uint32_t i=start; i<end; i++) {
		Vector3 point;
		to_math_vector<ispc::float4, Vector3, 3>(point, buildData[i].centroid);

		box.AddPoint(point);
	    }
	}
    }
#endif
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
	#ifdef ISPC_AOS
        return buildData[index].centroid.v[dim];
	#else
	return buildData[index].centroid[dim];
	#endif
    }
}
#endif
