if(UNIX)
	set(ISPC_PATH /usr/local/ispc/ispc)
	set(ISPC_FLAGS -O2 --target=sse4-x2 --arch=x86-64)

	set(ISPC_FILES ../scene/partition.ispc)
	string(REPLACE ".ispc" "_ispc.o" ISPC_OBJ ${ISPC_FILES})
	string(REPLACE ".ispc" "_ispc.h" ISPC_HEADER ${ISPC_FILES})
	add_custom_command(
		OUTPUT ${ISPC_OBJ} ${ISPC_HEADER}
		MAIN_DEPENDENCY ${ISPC_FILES}
		COMMAND ${ISPC_PATH} ${ISPC_FLAGS} ${CMAKE_CURRENT_LIST_DIR}/${ISPC_FILES} -o ${ISPC_OBJ} -h ${CMAKE_CURRENT_LIST_DIR}/${ISPC_HEADER}
	)
endif (UNIX)

if(WIN32)
	set(ISPC_PATH C:/Users/Abhi/Downloads/ispc-v1.6.0-windows/ispc.exe)
	set(ISPC_FLAGS -O2 --target=sse4-x2 --arch=x86)

	set(ISPC_FILES ../scene/partition.ispc)
	string(REPLACE ".ispc" "_ispc.obj" ISPC_OBJ ${ISPC_FILES})
	string(REPLACE ".ispc" "_ispc.h" ISPC_HEADER ${ISPC_FILES})
	add_custom_command(
		OUTPUT ${ISPC_OBJ} ${ISPC_HEADER}
		MAIN_DEPENDENCY ${ISPC_FILES}
		COMMAND ${ISPC_PATH} ${ISPC_FLAGS} ${CMAKE_CURRENT_LIST_DIR}/${ISPC_FILES} -o ${ISPC_OBJ} -h ${CMAKE_CURRENT_LIST_DIR}/${ISPC_HEADER}
	)
endif (WIN32)


add_executable(p3 main.cpp raytracer.cpp)
target_link_libraries(p3 application math scene tinyxml ${SDL_LIBRARY}
                      ${PNG_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES}
                      ${GLEW_LIBRARIES})

if(APPLE)
    target_link_libraries(p3 SDLmain)
endif()

install(TARGETS p3 DESTINATION ${PROJECT_SOURCE_DIR}/..)
