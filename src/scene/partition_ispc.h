//
// /afs/andrew.cmu.edu/usr1/chenxil/Documents/15618_Project/src/scene/partition_ispc.h
// (Header automatically generated by the ispc compiler.)
// DO NOT EDIT THIS FILE.
//

#ifndef ISPC__AFS_ANDREW_CMU_EDU_USR1_CHENXIL_DOCUMENTS_15618_PROJECT_SRC_SCENE_PARTITION_ISPC_H
#define ISPC__AFS_ANDREW_CMU_EDU_USR1_CHENXIL_DOCUMENTS_15618_PROJECT_SRC_SCENE_PARTITION_ISPC_H

#include <stdint.h>



#ifdef __cplusplus
namespace ispc { /* namespace */
#endif // __cplusplus
///////////////////////////////////////////////////////////////////////////
// Vector types with external visibility from ispc code
///////////////////////////////////////////////////////////////////////////

#ifdef _MSC_VER
__declspec( align(16) ) struct float3 { float v[3]; };
#else
struct float3 { float v[3]; } __attribute__ ((aligned(16)));
#endif

struct BVHPrimitiveInfoList {
    uint32_t * primitiveNumber;
    float * centroidx;
    float * centroidy;
    float * centroidz;
    float * lowCoordx;
    float * lowCoordy;
    float * lowCoordz;
    float * highCoordx;
    float * highCoordy;
    float * highCoordz;
    int32_t primCount;
};


///////////////////////////////////////////////////////////////////////////
// Functions exported from ispc code
///////////////////////////////////////////////////////////////////////////
#if defined(__cplusplus) && !defined(__ISPC_NO_EXTERN_C)
extern "C" {
#endif // __cplusplus
    extern void AddBox(float3   &lowCoord, float3   &highCoord, const struct BVHPrimitiveInfoList &buildData, int32_t start, int32_t end);
    extern void AddCentroid(float3   &lowCoord, float3   &highCoord, const struct BVHPrimitiveInfoList &buildData, int32_t start, int32_t end);
#if defined(__cplusplus) && !defined(__ISPC_NO_EXTERN_C)
} /* end extern C */
#endif // __cplusplus


#ifdef __cplusplus
} /* namespace */
#endif // __cplusplus

#endif // ISPC__AFS_ANDREW_CMU_EDU_USR1_CHENXIL_DOCUMENTS_15618_PROJECT_SRC_SCENE_PARTITION_ISPC_H
