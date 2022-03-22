cmake_minimum_required(VERSION 3.21)

add_library(tinyxml STATIC
        src/tinyxml2.cpp
        )

target_include_directories(tinyxml PUBLIC include)