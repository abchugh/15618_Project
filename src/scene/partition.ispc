
typedef float<3> Vector3;

struct BBox {
    Vector3 lowCoord;
    Vector3 highCoord;
};

struct BVHPrimitiveInfo {
    int primitiveNumber;
    Vector3 centroid;
    Vector3 lowCoord;
    Vector3 highCoord;
};

struct BucketInfo {
    int count;
    BBox bounds;
};

static inline void addPoint(BBox* bound, Vector3 point) {
    bool<3> lowTest = bound->lowCoord < point;
    bool<3> highTest = bound->highCoord > point;

    bound->lowCoord = lowTest ? bound->lowCoord : point;
    bound->highCoord = highTest ? bound->highCoord : point;
}

static inline void addBox(BBox* bound, BBox* add) {
    bool<3> lowTest = bound->lowCoord < add->lowCoord;
    bool<3> highTest = bound->highCoord > add->highCoord;

    bound->lowCoord = lowTest ? bound->lowCoord : add->lowCoord;
    bound->highCoord = highTest ? bound->highCoord : add->highCoord;
}

static inline int compareToMid(BVHPrimitiveInfo buildData, int dim, float m) {
    return buildData.centroid[dim] < m;
}

export void hashBuckets(uniform int start, uniform int end, 
			uniform BVHPrimitiveInfo buildData[], uniform int nBuckets) {
    
}

export uniform int partition_ispc(uniform int start, uniform int end, uniform int dim, uniform float mid,
		      uniform BVHPrimitiveInfo buildData[], uniform BVHPrimitiveInfo result[]) {
    uniform int nPrimitives = end - start;
    uniform int smaller_offset = 0;
    uniform int larger_offset = 0;
    
    foreach (i = 0 ... nPrimitives) {
        int smaller = compareToMid(buildData[start + i], dim, mid);
        int larger = 1 - smaller;

        int gang_smaller_offset = exclusive_scan_add(smaller);
        int gang_larger_offset = exclusive_scan_add(larger);

        int des;

        //print("s: %d;\te: %d\n", start, end);
        //print("%d: %d - %d %d\n", i, smaller, smaller_offset, larger_offset);
        //print("%d - %d\n", gang_smaller_offset, gang_larger_offset);
        if (smaller)
	        des = start + smaller_offset + gang_smaller_offset;
        else
	        des = end - 1 - larger_offset - gang_larger_offset;

        result[des] = buildData[start + i];
	
        //print("des: %d\n", des);
        //print("====\n");

        smaller_offset += reduce_add(smaller);
        larger_offset += reduce_add(larger);
    }
    
    return smaller_offset + start;
}
