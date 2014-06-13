add_executable(p3 main.cpp raytracer.cpp)
target_link_libraries(p3 application math integrator light material scene filter sample tinyxml ${SDL_LIBRARY}
                      ${PNG_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES}
                      ${GLEW_LIBRARIES})

if(APPLE)
    target_link_libraries(p3 SDLmain)
endif()

install(TARGETS p3 DESTINATION ${PROJECT_SOURCE_DIR}/..)

add_dependencies(p3 scene)
