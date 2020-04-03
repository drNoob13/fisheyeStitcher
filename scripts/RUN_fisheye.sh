#!/bin/bash
#----------------------------------------------------------------------------#
#                                                                            #
# This file is part of the fisheye stitcher project.                         #
# Copyright (c) 2018-2020 Tuan Phan Minh Ho <drnoob2013@gmail.com>           #
# https://github.com/drNoob13/fisheyeStitcher                                #
#                                                                            #
#----------------------------------------------------------------------------#
# Function: RUN fisheye stitcher

BUILD_DIR='../build'
BINARY='fisheyeStitcher'

CURR_DIR=`pwd`

if [ ! -d $BUILD_DIR ]; then
    echo " You need to build the code first"
    echo " Help:"
    echo "     mkdir build && cd build" 
    echo "     cmake .." 
    echo "     make" 
else
    cd $BUILD_DIR
    rm ./bin/${BINARY}
    cmake ..
    make -j 4
    cd $CURR_DIR
fi

########################################################################################################3

image_width=3840   # dual-fisheye image
image_height=1920  # dual-fisheye image
in_dir='../input'
video_path='../input/input_video.mp4'
mls_map_path='../utils/grid_xd_yd_3840x1920.yml.gz'
img_nm='test'
out_dir='../stitched'
enb_lc='false'  # light compensation
enb_ra='false'  # refine alignment

########################################################################################################3

if [ ! -d $out_dir ]; then
    mkdir -p $out_dir
fi

echo ""
echo ""

${BUILD_DIR}/bin/${BINARY}          \
   --out_dir         $out_dir       \
   --image_width     $image_width   \
   --image_height    $image_height  \
   --img_nm          $img_nm        \
   --video_path      $video_path    \
   --mls_map_path    $mls_map_path  \
   --enb_lc          $enb_lc        \
   --enb_ra          $enb_ra        \
   --mode            "video"