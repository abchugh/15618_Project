cmake_minimum_required(VERSION 2.8)
project(P3)

include(build/CMakeLists.txt)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${SDL_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIRS}
    ${PNG_INCLUDE_DIRS}
)

add_subdirectory(application)
add_subdirectory(math)
add_subdirectory(p3)
add_subdirectory(scene)
add_subdirectory(filter)
add_subdirectory(sample)
add_subdirectory(material)
add_subdirectory(integrator)
add_subdirectory(light)
add_subdirectory(tinyxml)

if(APPLE)
    add_subdirectory(SDLmain)
endif()
