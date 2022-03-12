#-------------------------------------------------------------------
# This file is part of the CMake build system for OGRE
#     (Object-oriented Graphics Rendering Engine)
# For the latest info, see http://www.ogre3d.org/
#
# The contents of this file are placed in the public domain. Feel
# free to make use of it in any way you like.
#-------------------------------------------------------------------

############################################################
# Paging optional component
############################################################

# define header and source files for the library
file(GLOB HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
list(APPEND HEADER_FILES
#        ${PROJECT_BINARY_DIR}/include/OgreBitesPrerequisites.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/OgreBites.i)
set(SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreAdvancedRenderControls.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreApplicationContextBase.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreBitesConfigDialog.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreCameraMan.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreSGTechniqueResolverListener.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreStaticPluginLoader.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreTrays.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreWindowEventUtilities.cpp")

include_directories("src")
list(APPEND SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreWIN32ConfigDialog.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/resource.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/misc/OgreWin32Resources.rc")
# Need to include resource files so that icons are linked
set(RESOURCE_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/resource.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/misc/OgreWin32Resources.rc"
        )
source_group(Resources FILES ${RESOURCE_FILES})

# setup target
add_library(OgreBites ${OGRE_COMP_LIB_TYPE} ${HEADER_FILES} ${SOURCES})
set_target_properties(OgreBites PROPERTIES VERSION ${OGRE_SOVERSION} SOVERSION ${OGRE_SOVERSION})
target_link_libraries(OgreBites PUBLIC OgreMain OgreOverlay PRIVATE OgreRTShaderSystem)
if (OGRE_BUILD_COMPONENT_RTSHADERSYSTEM)
    target_link_libraries(OgreBites PUBLIC OgreRTShaderSystem)
endif ()
target_include_directories(OgreBites PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${NATIVE_INCLUDES}>"
        $<INSTALL_INTERFACE:include/OGRE/Bites>)

if (OGRE_STATIC OR OGRE_BITES_STATIC_PLUGINS)
    target_compile_definitions(OgreBites PRIVATE OGRE_BITES_STATIC_PLUGINS)
endif ()

generate_export_header(OgreBites
        EXPORT_MACRO_NAME _OgreBitesExport
        EXPORT_FILE_NAME ${PROJECT_BINARY_DIR}/include/OgreBitesPrerequisites.h)

#if(Qt6_FOUND OR Qt5_FOUND)
#  if(Qt6_FOUND)
#    qt6_wrap_cpp(MOC_SRC "${CMAKE_CURRENT_SOURCE_DIR}/include/OgreApplicationContextQt.h")
#  else()
#    qt5_wrap_cpp(MOC_SRC "${CMAKE_CURRENT_SOURCE_DIR}/include/OgreApplicationContextQt.h")
#  endif()
#
#  add_library(OgreBitesQt ${OGRE_COMP_LIB_TYPE} ${MOC_SRC} "${CMAKE_CURRENT_SOURCE_DIR}/src/OgreApplicationContextQt.cpp")
#  set_target_properties(OgreBitesQt PROPERTIES VERSION ${OGRE_SOVERSION} SOVERSION ${OGRE_SOVERSION})
#  target_link_libraries(OgreBitesQt PUBLIC Qt${QT_VERSION_MAJOR}::Gui OgreBites)
#  ogre_config_component(OgreBitesQt)
#endif()
