#-------------------------------------------------------------------
# This file is part of the CMake build system for OGRE
#     (Object-oriented Graphics Rendering Engine)
# For the latest info, see http://www.ogre3d.org/
#
# The contents of this file are placed in the public domain. Feel
# free to make use of it in any way you like.
#-------------------------------------------------------------------

#######################################################################
# This file takes care of configuring Ogre to build with the settings
# given in CMake. It creates the necessary config.h file and will
# also prepare package files for pkg-config and CMake.
#######################################################################

set(OGRE_ASSERT_MODE 2)
set(OGRE_SET_ASSERT_MODE ${OGRE_ASSERT_MODE})
set(OGRE_SET_THREADS 2)
set(OGRE_SET_THREAD_PROVIDER 4)
set(OGRE_RESOURCEMANAGER_STRICT 2)
set(OGRE_NO_VIEWPORT_ORIENTATIONMODE 1)

set(RTSHADER_SYSTEM_BUILD_CORE_SHADERS TRUE)

# generate OgreBuildSettings.h
configure_file(${OGRE_TEMPLATES_DIR}/OgreComponents.h.in ${PROJECT_BINARY_DIR}/include/OgreComponents.h @ONLY)
configure_file(${OGRE_TEMPLATES_DIR}/OgreBuildSettings.h.in ${PROJECT_BINARY_DIR}/include/OgreBuildSettings.h @ONLY)
configure_file(${OGRE_TEMPLATES_DIR}/OgreRTShaderConfig.h.in ${PROJECT_BINARY_DIR}/include/OgreRTShaderConfig.h @ONLY)
#configure_file(${OGRE_TEMPLATES_DIR}/OgreGLES2Config.h.in ${PROJECT_BINARY_DIR}/include/OgreGLES2Config.h @ONLY)

configure_file(${OGRE_TEMPLATES_DIR}/OgreConfigPaths.h.in ${PROJECT_BINARY_DIR}/include/OgreConfigPaths.h @ONLY)
