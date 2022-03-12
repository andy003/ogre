cmake_minimum_required(VERSION 3.10.0)


set(OGRE_VERSION_MAJOR 13)
set(OGRE_VERSION_MINOR 3)
set(OGRE_VERSION_PATCH 0)

set(OGRE_VERSION_SUFFIX "ccors")
set(OGRE_VERSION_NAME "Tsathoggua")
set(OGRE_TEMPLATES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/CMake/Templates)

include(GenerateExportHeader)
include(CMake/ConfigureBuild.cmake)

add_subdirectory(OgreMain)
add_subdirectory(Components)
