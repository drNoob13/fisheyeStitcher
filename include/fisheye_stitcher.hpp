//----------------------------------------------------------------------------//
// This file is part of the fisheye stitcher project.                         //
// Author  : Tuan Phan Minh Ho (drnoob2013@gmail.com)                         //
// Function: 360-degree Video Stitcher for Dual-fisheye Lens Camera           //
// Support : Samsung Gear360 2016-model (C200)                                //
// Source  : https://github.com/drNoob13/fisheyeStitcher                      //
//----------------------------------------------------------------------------//
#ifndef FISHEYE_STITCHER_HPP
#define FISHEYE_STITCHER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <vector> 
#include <tuple>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> // for imshow
#include <opencv2/calib3d.hpp> // for findHomography
#include "opencv2/stitching/detail/seam_finders.hpp" // seam_finder
#include <opencv2/core/utility.hpp>

#define     MAX_FOVD      195.0f

// Polynomial Coefficients
#define    P1_    -7.5625e-17
#define    P2_     1.9589e-13
#define    P3_    -1.8547e-10
#define    P4_     6.1997e-08
#define    P5_    -6.9432e-05
#define    P6_     0.9976

namespace stitcher
{

class FisheyeStitcher
{
public:
    FisheyeStitcher(int width, int height, int in_fovd, 
                    bool enb_light_compen, bool enb_refine_align);
    ~FisheyeStitcher();
    cv::Mat stitch(const cv::Mat& image1, const cv::Mat& image2);

private:
    cv::Mat fishUnwarp( const cv::Mat &map_x, const cv::Mat &map_y, 
                         const cv::Mat &src );
    std::tuple<double, double> fish2Eqt( double x_dest, double  y_dest, 
                                         double W_rad );
    std::tuple<cv::Mat, cv::Mat> fish2Map();
    std::tuple<cv::Mat, cv::Mat> createMask();
    cv::Mat deform( const cv::Mat &map_x, const cv::Mat &map_y, 
                    const cv::Mat &src );
    cv::Mat compenMap( const cv::Mat &R_pf );

    //
    int m_hs;
    int m_ws;
    int m_hd;
    int m_wd;
    float m_in_fovd;
    bool m_enb_light_compen;
    bool m_enb_refine_align;
};

}   // namespace

#endif  // FISHEYE_STITCHER_HPP