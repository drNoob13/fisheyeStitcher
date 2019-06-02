#!/bin/sh
#-----------------------------------------------------------------------
#
# Author  : Tuan Ho
# Function: RUN fisheye stitcher and combine stitched frames
# Source  : https://github.com/drNoob13/fisheyeStitcher
#
#-----------------------------------------------------------------------

# Change these numbers according to your actual data
#========================================================================
FR_START=1085
FR_END=1145
playback_enb=1
#========================================================================

# Directory setup
IN_DIR='./Frames'
compressed_='sample.tgz'
OUT_DIR='./Stitched_Frames'
RESO='3840x1920'
VIDEO_DIR='./Stitched_Video'
video_in='./input/input_video.mp4'

# Uncompress test images
if [ -f "${IN_DIR}/${compressed_}" ]; then
    echo "Uncompressing files.."
    cd $IN_DIR
    tar -xzf $compressed_
    cd ..
fi

if [ ! -d "$IN_DIR" ]; then
    mkdir $IN_DIR
fi

if [ ! -d "$OUT_DIR" ]; then
    mkdir $OUT_DIR
fi

if [ ! -d "$VIDEO_DIR" ]; then
    mkdir $VIDEO_DIR
fi

# Filename setup
filein_prefix='360_0546_SRA_'               # input filename prefix
fileout_prefix='pano_360_0546_SRA'          # output filename prefix
fileout_="stitched_${fileout_prefix}.mp4"   # stitched videos


#---- stitch frames
echo "-------------------------------------------------"
echo "Stitching: $video_in "
echo "-------------------------------------------------"
./build/fisheyeStitcher --video_in $video_in \
                        --in_dir $IN_DIR     \
                        --in $filein_prefix  \
                        --out_dir $OUT_DIR   \
                        --fr $FR_START $FR_END

echo "finish stitching!"
echo "" 