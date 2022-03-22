cmake_minimum_required(VERSION 3.10.0)

project(ccrobot)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "D:/Qt/5.15.2/msvc2019_64/lib/cmake")

add_subdirectory(ccrobot)
add_subdirectory(third_lib)
