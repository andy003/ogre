cmake_minimum_required(VERSION 3.10.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)

add_executable(ccrobot
        main.cpp
        )
target_link_libraries(ccrobot PRIVATE OgreMain OgreBitesQt)
