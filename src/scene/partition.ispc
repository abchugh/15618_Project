struct BVHPrimitiveInfoList {
    unsigned int* primitiveNumber;
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

export void swap_ispc(uniform int8 a[], uniform int8 b[], uniform int type_size, uniform int size) {
    
    int32 temp_4bytes;
    int8 temp_byte;
    uniform int four_steps = size * type_size / 4;

    foreach (i = 0 ... four_steps) {
	temp_4bytes = *((int32*)(a) + i);
	*((int32*)(a) + i) = *((int32*)(b) + i);
	*((int32*)(b) + i) = temp_4bytes;
    }
    
    foreach (i = four_steps * 4 ... size * type_size) {
	temp_byte = a[i];
	a[i] = b[i];
	b[i] = temp_byte;
    }
    /*
    int64 temp_8bytes;
    int8 temp_byte;
    uniform int eight_steps = size * type_size / 8;

    foreach (i = 0 ... eight_steps) {
	temp_8bytes = *((int64*)(a) + i);
	*((int64*)(a) + i) = *((int64*)(b) + i);
	*((int64*)(b) + i) = temp_8bytes;
    }
    
    foreach (i = eight_steps * 8 ... size * type_size) {
	temp_byte = a[i];
	a[i] = b[i];
	b[i] = temp_byte;
    }
    */
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

    for (uniform int i = 0; i < nPrimitives; i += programCount) {
        int index = i + programIndex;
        float coord = compareDim[index];
        int smaller = coord< mid;
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

        copyFromLocation(result, des, buildData, start+index);
        //result[des] = buildData[start + i];
	
        //print("des: %d\n", des);
        //print("====\n");

        smaller_offset += reduce_add(smaller);
        larger_offset += reduce_add(larger);
    }
    
    return smaller_offset + start;
}
