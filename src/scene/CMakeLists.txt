
set(ISPC_PATH /usr/local/ispc/ispc)
set(ISPC_FLAGS -O2 --target=sse4-x2 --arch=x86-64)

set(ISPC_FILES partition.ispc)
string(REPLACE ".ispc" "_ispc.o" ISPC_OBJ ${ISPC_FILES})
string(REPLACE ".ispc" "_ispc.h" ISPC_HEADER ${ISPC_FILES})
add_custom_command(
	OUTPUT ${ISPC_OBJ} ${ISPC_HEADER}
	MAIN_DEPENDENCY ${ISPC_FILES}
	COMMAND ${ISPC_PATH} ${ISPC_FLAGS} ${CMAKE_CURRENT_LIST_DIR}/${ISPC_FILES} -o ${CMAKE_CURRENT_LIST_DIR}/${ISPC_OBJ} -h ${CMAKE_CURRENT_LIST_DIR}/${ISPC_HEADER}
)

# include FindISPC.cmake (doesn't help)
#include(${CMAKE_SOURCE_DIR}/cmake/FindISPC.cmake)
#ispc_compile(${ISPC_FILES} "-O2 --target=sse4-x2 --arch=x86-64" PARTITION PARTITION_H)

add_library(scene material.cpp mesh.cpp model.cpp scene.cpp sphere.cpp
            triangle.cpp ray.cpp bvh.cpp BoundingBox.cpp ${ISPC_FILES})

set_source_files_properties(
	${ISPC_HEADER}
	PROPERTIES
	EXTERNAL_OBJECT TRUE
	GENERATED TRUE
)

set_source_files_properties(
	${ISPC_OBJ}
	PROPERTIES
	EXTERNAL_OBJECT TRUE
	GENERATED TRUE
)

add_library(ispc_obj STATIC ${ISPC_OBJ})
set_target_properties(
  ispc_obj
  PROPERTIES
  LINKER_LANGUAGE C
)

#target_link_libraries(scene ispc_obj)
