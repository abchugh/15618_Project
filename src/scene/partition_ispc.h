//
// W:/Document/15618_Project/src/scene/partition_ispc.h
// (Header automatically generated by the ispc compiler.)
// DO NOT EDIT THIS FILE.
//

#ifndef ISPC_W__DOCUMENT_15618_PROJECT_SRC_SCENE_PARTITION_ISPC_H
#define ISPC_W__DOCUMENT_15618_PROJECT_SRC_SCENE_PARTITION_ISPC_H

#include <stdint.h>



#ifdef __cplusplus
namespace ispc { /* namespace */
#endif // __cplusplus
///////////////////////////////////////////////////////////////////////////
// Vector types with external visibility from ispc code
///////////////////////////////////////////////////////////////////////////

#ifndef __ISPC_VECTOR_float3__
#define __ISPC_VECTOR_float3__
#ifdef _MSC_VER
__declspec( align(16) ) struct float3 { float v[3]; };
#else
struct float3 { float v[3]; } __attribute__ ((aligned(16)));
#endif
#endif


#ifndef __ISPC_STRUCT_BVHPrimitiveInfoList__
#define __ISPC_STRUCT_BVHPrimitiveInfoList__
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
#endif


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

#endif // ISPC_W__DOCUMENT_15618_PROJECT_SRC_SCENE_PARTITION_ISPC_H
