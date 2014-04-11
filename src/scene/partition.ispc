
typedef float<3> Vector3;

struct BBox {
    Vector3 lowCoord;
    Vector3 highCoord;
};

struct BuildData {
    int primitiveNumber;
    Vector3 centroid;
    BBox bounds;
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

static inline int compareToMid(BuildData buildData, int dim, float m) {
    return buildData.centroid[dim] < m;
}

export void hashBuckets(uniform int start, uniform int end, 
			uniform BuildData buildData[], uniform int nBuckets) {
    
}

export void partition(uniform int start, uniform int end, uniform int dim, uniform float mid,
		      uniform BuildData buildData[], uniform BuildData result[]) {
    uniform int nPrimitives = end - start;
    uniform int smaller_offset = 0;
    uniform int larger_offset = 0;
    
    foreach (i = 0 ... nPrimitives) {
	int smaller = compareToMid(buildData[start + i], dim, mid);
	int larger = 1 - smaller;

	int gang_smaller_offset = exclusive_scan_add(smaller);
	int gang_larger_offset = exclusive_scan_add(larger);

	if (smaller)
	    result[smaller_offset + gang_smaller_offset] = buildData[start + i];
	else
	    result[end - 1 - larger_offset - gang_larger_offset] = buildData[start + i];

	smaller_offset += reduce_add(smaller);
	larger_offset += reduce_add(larger);
    }

}
