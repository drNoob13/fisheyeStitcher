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
echo "Stitching: $IN_DIR/${filein_prefix}${FR_START}:${FR_END}.jpg "
echo "-------------------------------------------------"
./fisheyeStitcher --in_dir $IN_DIR --in $filein_prefix --out_dir $OUT_DIR --fr $FR_START $FR_END
echo "finish stitching!"
echo ""

#---- backup previous stitched video
if [ -f "${VIDEO_DIR}/${fileout_}" ]; then
    mv ${VIDEO_DIR}/${fileout_}  ${VIDEO_DIR}/bk_${fileout_}
fi

echo "-------------------------------------------------"
echo "Generating video: ${VIDEO_DIR}/${fileout_}"
echo "-------------------------------------------------"
#---- combine frames
ffmpeg -loglevel error -r 29 -f image2 -s $RESO -start_number $FR_START  -i ${OUT_DIR}/pano_360_0546_SRA_%04d.jpg -vcodec libx264 -crf 25 -pix_fmt yuv420p  ${VIDEO_DIR}/${fileout_} 


filein_='360_0546_SRA.mp4'
if [ ! -f ${VIDEO_DIR}/${filein_} ]; then
    echo "-------------------------------------------------"
    echo "Generating video: ${VIDEO_DIR}/${filein_} (original video)"
    echo "-------------------------------------------------"
    # Generate video from original images
    ffmpeg -loglevel error -y -r 29 -f image2 -s $RESO -start_number $FR_START  -i ${IN_DIR}/360_0546_SRA_%04d.jpg -vcodec libx264 -crf 25 -pix_fmt yuv420p  ${VIDEO_DIR}/${filein_} 
else
    echo "Skip generating original video (already exists!)"
fi


echo "-------------------------------------------------"
echo "Generating vert-stacked video"
echo "-------------------------------------------------"

ffmpeg \
  -y \
  -i ${VIDEO_DIR}/${filein_} \
  -i ${VIDEO_DIR}/${fileout_} \
  -filter_complex vstack=inputs=2 \
  -loglevel warning \
  ${VIDEO_DIR}/output_vstack.mp4

echo "-------------------------------------------------"
echo "Playing: original vs stitched"
echo "-------------------------------------------------"
if [ $playback_enb -eq 1 ]; then
    ffplay -loglevel error -loop 10  ${VIDEO_DIR}/output_vstack.mp4
fi
