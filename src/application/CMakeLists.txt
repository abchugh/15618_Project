add_library(application application.cpp camera_roam.cpp imageio.cpp
            scene_loader.cpp)

add_dependencies(application scene)