struct BVHPrimitiveInfoList {
    int* primitiveNumber;
    float* centroidx;
    float* centroidy;
    float* centroidz;

    float* lowCoordx;
    float* lowCoordy;
    float* lowCoordz;

    float* highCoordx;
    float* highCoordy;
    float* highCoordz;
    
    int primCount;
};
static inline int copyFromLocation(BVHPrimitiveInfoList* result, int des, const BVHPrimitiveInfoList* buildData, int src) {
    result->primitiveNumber[des] = buildData->primitiveNumber[src];

    result->centroidx[des] = buildData->centroidx[src];
    result->centroidy[des] = buildData->centroidy[src];
    result->centroidz[des] = buildData->centroidz[src];

    result->lowCoordx[des] = buildData->lowCoordx[src];
    result->lowCoordy[des] = buildData->lowCoordy[src];
    result->lowCoordz[des] = buildData->lowCoordz[src];
          
    result->highCoordx[des] = buildData->highCoordx[src];
    result->highCoordy[des] = buildData->highCoordy[src];
    result->highCoordz[des] = buildData->highCoordz[src];
}

export uniform int partition_ispc(uniform int start, uniform int end, uniform int dim, uniform float mid,
		       uniform BVHPrimitiveInfoList buildData[], uniform BVHPrimitiveInfoList result[]) {
                
    uniform int nPrimitives = end - start;
    uniform int smaller_offset = 0;
    uniform int larger_offset = 0;
    uniform float* compareDim = buildData[0].centroidx;
    if(dim==1)
        compareDim = buildData[0].centroidy;
    if(dim==2)
        compareDim = buildData[0].centroidz;
    compareDim = compareDim + start;

    foreach (i = 0 ... nPrimitives) {
        int smaller = compareDim[i]< mid;
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

        copyFromLocation(result, des, buildData, start+i);
        //result[des] = buildData[start + i];
	
        //print("des: %d\n", des);
        //print("====\n");

        smaller_offset += reduce_add(smaller);
        larger_offset += reduce_add(larger);
    }
    
    return smaller_offset + start;
}