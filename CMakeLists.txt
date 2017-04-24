cmake_minimum_required(VERSION 3.7)
project(roomrecog)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES src/amde/roomrecog/roomrecog.cpp include/amde/roomrecog/roomrecog.h include/amde/roomrecog/map.h src/amde/roomrecog/brushpalette.h src/amde/roomrecog/brush.h src/amde/roomrecog/brushpalette.cpp src/amde/roomrecog/brush.cpp src/amde/roomrecog/checkpoint.h src/amde/roomrecog/roompoint.h src/amde/roomrecog/checkpoint.cpp src/amde/roomrecog/map.cpp include/amde/roomrecog/rectangle.h src/amde/roomrecog/roompoint.cpp src/amde/roomrecog/mappoint.h)
add_library(roomrecog SHARED ${SOURCE_FILES})