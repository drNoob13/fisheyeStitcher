cmake_minimum_required(VERSION 2.8)
project( fisheyeStitcher )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( fisheyeStitcher src/fisheyeStitcher.cpp )
target_link_libraries( fisheyeStitcher ${OpenCV_LIBS} )

