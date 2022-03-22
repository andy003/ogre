
add_library(urdf STATIC
        src/model.cpp
        src/link.cpp
        src/pose.cpp
        src/joint.cpp
        )
target_include_directories(urdf PUBLIC include)
target_link_libraries(urdf tinyxml)