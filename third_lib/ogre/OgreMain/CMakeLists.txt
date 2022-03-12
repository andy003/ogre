#-------------------------------------------------------------------
# This file is part of the CMake build system for OGRE
#     (Object-oriented Graphics Rendering Engine)
# For the latest info, see http://www.ogre3d.org/
#
# The contents of this file are placed in the public domain. Feel
# free to make use of it in any way you like.
#-------------------------------------------------------------------

############################################################
# OgreMain core library
############################################################

# define header and source files for the library
file(GLOB HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
#list(APPEND HEADER_FILES
#    ${PROJECT_BINARY_DIR}/include/OgreBuildSettings.h
#    ${PROJECT_BINARY_DIR}/include/OgreComponents.h
#    ${PROJECT_BINARY_DIR}/include/OgreExports.h
#    ${CMAKE_CURRENT_SOURCE_DIR}/include/Ogre.i)

file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
MESSAGE(STATUS "sources ${SOURCE_FILES}")

# Remove optional header files
list(REMOVE_ITEM HEADER_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/include/OgreDDSCodec.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/OgrePVRTCCodec.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/OgreETCCodec.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/OgreZip.h"
        )

# Remove optional source files
list(REMOVE_ITEM SOURCE_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreFileSystemLayerNoOp.cpp"
#        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreDDSCodec.cpp"
#        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgrePVRTCCodec.cpp"
#        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreETCCodec.cpp"
#        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreZip.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreSearchOps.cpp"
        )

file(GLOB PLATFORM_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/WIN32/*.cpp")
list(APPEND PLATFORM_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/Threading/OgreDefaultWorkQueueStandard.cpp")
list(APPEND PLATFORM_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/zip/zip.c")
add_library(OgreMain STATIC ${SOURCE_FILES} ${HEADER_FILES} ${PLATFORM_SOURCE_FILES})
target_include_directories(OgreMain
        PUBLIC
        include
        include/Threading
        src/zip
        "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>"
        PRIVATE
        src
        )

generate_export_header(OgreMain
        EXPORT_MACRO_NAME _OgreExport
        NO_EXPORT_MACRO_NAME _OgrePrivate
        DEPRECATED_MACRO_NAME OGRE_DEPRECATED
        EXPORT_FILE_NAME ${PROJECT_BINARY_DIR}/include/OgreExports.h)

if (MSVC)
    target_compile_definitions(OgreMain PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(OgreMain PUBLIC /wd4305 /wd4819 /wd4244 /wd4267)
endif ()


