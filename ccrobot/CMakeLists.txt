cmake_minimum_required(VERSION 3.10.0)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)

add_executable(ccrobot
        main.cpp
        main_menu.cpp

        ogre_helper/shape.cpp
        ogre_helper/object.cpp
        ogre_helper/axes.cpp
        ogre_helper/arrow.cpp
#        ogre_helper/movable_text.cpp
        ogre_helper/grid.cpp
        ogre_helper/billboard_line.cpp

        robot/robot.cpp
        robot/robot_link.cpp
        robot/robot_joint.cpp
        )

target_link_libraries(ccrobot PRIVATE OgreMain OgreBitesQt OgreOverlay urdf)
target_include_directories(ccrobot PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if (WIN32)
    set(DEBUG_SUFFIX)
    if (CMAKE_BUILD_TYPE MATCHES "Debug")
        set(DEBUG_SUFFIX "d")
    endif ()
    set(QT_INSTALL_PATH "${CMAKE_PREFIX_PATH}")
    if (NOT EXISTS "${QT_INSTALL_PATH}/bin")
        set(QT_INSTALL_PATH "${QT_INSTALL_PATH}/..")
        if (NOT EXISTS "${QT_INSTALL_PATH}/bin")
            set(QT_INSTALL_PATH "${QT_INSTALL_PATH}/..")
        endif ()
    endif ()
    if (EXISTS "${QT_INSTALL_PATH}/plugins/platforms/qwindows${DEBUG_SUFFIX}.dll")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory
                "${EXECUTABLE_OUTPUT_PATH}/plugins/platforms//plugins/platforms/")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                "${QT_INSTALL_PATH}/plugins/platforms/qwindows${DEBUG_SUFFIX}.dll"
                "${EXECUTABLE_OUTPUT_PATH}/plugins/platforms/")
    endif ()
    foreach (QT_LIB Core Gui Widgets SerialPort)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                "${QT_INSTALL_PATH}/bin/Qt5${QT_LIB}${DEBUG_SUFFIX}.dll"
                "${EXECUTABLE_OUTPUT_PATH}")
    endforeach (QT_LIB)
endif ()
