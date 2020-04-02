cmake_minimum_required(VERSION 3.5.1)

project( fisheyeStitcher )

include(CheckCXXCompilerFlag) 
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    add_compile_options(-std=c++11 -O3)
else()
    message(FATAL_ERROR "ERROR: The compiler ${CMAKE_CXX_COMPILER} has no C++11 
                         support. Please use a different C++ compiler.")
endif()

find_package( OpenCV REQUIRED )

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    include
)

add_definitions( 
    -DMY_DEBUG      # Print debug info and dump intermediate images 
    # -DPROFILING     # Profiling performance function-wise
    # -DGEAR360_C200  # Setting for GEAR 360 C200 camera 
)
add_library( stitcher SHARED
    src/fisheye_stitcher.cpp
)

add_executable( fisheyeStitcher 
    src/fisheye_stitcher.cpp 
    app/stitch.cpp
)

target_link_libraries( fisheyeStitcher 
    stitcher
    ${OpenCV_LIBS} 
)
set_target_properties( fisheyeStitcher
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)