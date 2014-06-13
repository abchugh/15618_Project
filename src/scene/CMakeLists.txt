if(UNIX)
	set(ISPC_PATH /usr/local/ispc/ispc)
	set(ISPC_FLAGS -O2 --target=sse4-x2 --arch=x86-64)
endif (UNIX)

if(WIN32)
	set(ISPC_PATH W:/Downloads/ispc-v1.7.0-windows/ispc.exe)
	set(ISPC_FLAGS -O2 --target=sse4-x2 --arch=x86)
endif (WIN32)

set(ISPC_PARTITION_FILE "partition.ispc")
string(REPLACE ".ispc" "_ispc.o" ISPC_PARTITION_OBJ ${ISPC_PARTITION_FILE})
string(REPLACE ".ispc" "_ispc.h" ISPC_PARTITION_HEADER ${ISPC_PARTITION_FILE})
add_custom_command(
	OUTPUT ${ISPC_PARTITION_OBJ} ${ISPC_PARTITION_HEADER}
	MAIN_DEPENDENCY ${ISPC_PARTITION_FILE}
	COMMAND ${ISPC_PATH} ${ISPC_FLAGS} ${CMAKE_CURRENT_LIST_DIR}/${ISPC_PARTITION_FILE} -o ${CMAKE_CURRENT_LIST_DIR}/${ISPC_PARTITION_OBJ} -h ${CMAKE_CURRENT_LIST_DIR}/${ISPC_PARTITION_HEADER}
)

set(ISPC_HIT_FILE "hit.ispc")
string(REPLACE ".ispc" "_ispc.o" ISPC_HIT_OBJ ${ISPC_HIT_FILE})
string(REPLACE ".ispc" "_ispc.h" ISPC_HIT_HEADER ${ISPC_HIT_FILE})
add_custom_command(
	OUTPUT ${ISPC_HIT_OBJ} ${ISPC_HIT_HEADER}
	MAIN_DEPENDENCY ${ISPC_HIT_FILE}
	COMMAND ${ISPC_PATH} ${ISPC_FLAGS} ${CMAKE_CURRENT_LIST_DIR}/${ISPC_HIT_FILE} -o ${CMAKE_CURRENT_LIST_DIR}/${ISPC_HIT_OBJ} -h ${CMAKE_CURRENT_LIST_DIR}/${ISPC_HIT_HEADER}
)

add_library(scene mesh.cpp model.cpp scene.cpp sphere.cpp
            triangle.cpp ray.cpp bvh.cpp bvhISPC.cpp bvhNonISPC.cpp BoundingBox.cpp ${ISPC_PARTITION_OBJ} 
	    ${ISPC_HIT_OBJ} ${ISPC_PARTITION_HEADER}   ${ISPC_HIT_HEADER} )

set_source_files_properties(
	${ISPC_HIT_OBJ}
	PROPERTIES
	EXTERNAL_OBJECT TRUE
	GENERATED TRUE
)
set_source_files_properties(
	${ISPC_PARTITION_OBJ}
	PROPERTIES
	EXTERNAL_OBJECT TRUE
	GENERATED TRUE
)
set_source_files_properties(
	${ISPC_HEADER_PATH}
	PROPERTIES
	HEADER_FILE_ONLY TRUE
	GENERATED TRUE
)

#target_link_libraries(scene ispc_obj)
