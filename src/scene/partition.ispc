#include "ispc_switch.h"

typedef float<3> Vector3;
typedef float<4> Vector4;
typedef float<8> Vector8;

#ifdef ISPC_SOA
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

export void AddCentroid(uniform Vector3& lowCoord, uniform Vector3& highCoord, uniform const BVHPrimitiveInfoList& buildData,uniform int start, uniform int end) {
        Vector3 localLow=lowCoord,localHigh=highCoord;
        foreach (i = start ... end){
            if(localLow [0]>buildData.centroidx[i]) localLow [0] = buildData.centroidx[i];
            if(localHigh[0]<buildData.centroidx[i]) localHigh[0] = buildData.centroidx[i];

            if(localLow [1]>buildData.centroidy[i]) localLow [1] = buildData.centroidy[i];
            if(localHigh[1]<buildData.centroidy[i]) localHigh[1] = buildData.centroidy[i];

            if(localLow [2]>buildData.centroidz[i]) localLow [2] = buildData.centroidz[i];
            if(localHigh[2]<buildData.centroidz[i]) localHigh[2] = buildData.centroidz[i];
        }
        lowCoord[0] = reduce_min(localLow[0]);
        lowCoord[1] = reduce_min(localLow[1]);
        lowCoord[2] = reduce_min(localLow[2]);

        highCoord[0] = reduce_max(localHigh[0]);
        highCoord[1] = reduce_max(localHigh[1]);
        highCoord[2] = reduce_max(localHigh[2]);

}

export void AddBox(uniform Vector3& lowCoord, uniform Vector3& highCoord, uniform const BVHPrimitiveInfoList& buildData,uniform int start, uniform int end) {
        Vector3 localLow=lowCoord,localHigh=highCoord;
        foreach (i = start ... end){
            if(localLow [0]>buildData.lowCoordx[i]) localLow [0] = buildData.lowCoordx[i];
            if(localHigh[0]<buildData.highCoordx[i]) localHigh[0] = buildData.highCoordx[i];

            if(localLow [1]>buildData.lowCoordy[i]) localLow [1] = buildData.lowCoordy[i];
            if(localHigh[1]<buildData.highCoordy[i]) localHigh[1] = buildData.highCoordy[i];

            if(localLow [2]>buildData.lowCoordz[i]) localLow [2] = buildData.lowCoordz[i];
            if(localHigh[2]<buildData.highCoordz[i]) localHigh[2] = buildData.highCoordz[i];
        }
        lowCoord[0] = reduce_min(localLow[0]);
        lowCoord[1] = reduce_min(localLow[1]);
        lowCoord[2] = reduce_min(localLow[2]);

        highCoord[0] = reduce_max(localHigh[0]);
        highCoord[1] = reduce_max(localHigh[1]);
        highCoord[2] = reduce_max(localHigh[2]);

}

#elif defined(ISPC_AOS)

// Assume vector lanes are 8-width

struct BVHPrimitiveInfo {
    // use w of lowCoord to store primitiveNumber
    Vector4 lowCoord;
    Vector4 highCoord;
    Vector4 centroid;
    Vector4 negCentroid;
};

export void AddCentroid(uniform Vector3& lowCoord, uniform Vector3& highCoord,
		   const uniform BVHPrimitiveInfo primitiveList[], uniform int start, uniform int end) {
    assert(programCount == 8);

    int coord = programIndex - 4;
    float localCoord;
    uniform float* uniform primitiveCoord;

    if (programIndex < 3)
	localCoord = lowCoord[programIndex];
    else if (programIndex >= 4 && programIndex < 7)
	localCoord = -highCoord[coord];

    for (uniform int i = start; i < end; i++) {
	primitiveCoord = (float*)&(primitiveList[i].centroid);
	localCoord = min(localCoord, primitiveCoord[programIndex]);
    }

    if (programIndex < 3)
	lowCoord[programIndex] = localCoord;
    else if (programIndex >= 4 && programIndex < 7)
	highCoord[coord] = -localCoord;
}

export void AddBox(uniform Vector3& lowCoord, uniform Vector3& highCoord, 
		   const uniform BVHPrimitiveInfo primitiveList[], 
		   uniform int start, uniform int end) {
    // Assume lowCoord and highCoord have been initialized.
    assert(programCount == 8);
    
    int coord = programIndex - 4;
    float localCoord;
    uniform float* uniform primitiveCoord;
    
    if (programIndex < 3)
	localCoord = lowCoord[programIndex];
    else if (programIndex >= 4 && programIndex < 7)
	localCoord = -highCoord[coord];
    
    for (uniform int i = start; i < end; i++) {
	primitiveCoord = (float*)&(primitiveList[i].lowCoord);
	localCoord = min(localCoord, primitiveCoord[programIndex]);
    }
    
    if (programIndex < 3)
	lowCoord[programIndex] = localCoord;
    else if (programIndex >= 4 && programIndex < 7)
	highCoord[coord] = -localCoord;
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
#endif

/*static inline int copyFromLocation(uniform BVHPrimitiveInfoList& result, int des, uniform const BVHPrimitiveInfoList& buildData, int src) {
    result.primitiveNumber[des] = buildData.primitiveNumber[src];
          
    result.centroidx[des] = buildData.centroidx[src];
    result.centroidy[des] = buildData.centroidy[src];
    result.centroidz[des] = buildData.centroidz[src];
                                    
    result.lowCoordx[des] = buildData.lowCoordx[src];
    result.lowCoordy[des] = buildData.lowCoordy[src];
    result.lowCoordz[des] = buildData.lowCoordz[src];
          
    result.highCoordx[des] = buildData.highCoordx[src];
    result.highCoordy[des] = buildData.highCoordy[src];
    result.highCoordz[des] = buildData.highCoordz[src];
}
export uniform int partition_ispc(uniform int start, uniform int end, uniform int dim, uniform float mid,
		       uniform const BVHPrimitiveInfoList& buildData, uniform BVHPrimitiveInfoList& result) {
                
    uniform int nPrimitives = end - start;
    uniform int smaller_offset = 0;
    uniform int larger_offset = 0;
    uniform float* uniform compareDim = buildData.centroidx;
    if(dim==1)
        compareDim = buildData.centroidy;
    if(dim==2)
        compareDim = buildData.centroidz;
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
}*/
