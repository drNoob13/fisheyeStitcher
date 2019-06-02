/***************************************************************************************************
*
* Author  : Tuan Ho (drnoob2013@gmail.com)
* Function: 360-degree Video Stitcher for Dual-fisheye Lens Camera
* Support : Samsung Gear360 2016-model (C200)
* Source  : https://github.com/drNoob13/fisheyeStitcher
*
***************************************************************************************************/

#include <iostream>
#include <fstream>
#include <string>
#include <vector> 

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> // for imshow
#include <opencv2/calib3d.hpp> // for findHomography
#include "opencv2/stitching/detail/seam_finders.hpp" // seam_finder
#include <opencv2/core/utility.hpp>

using namespace cv;
using namespace std;

#define     MAX_FOVD      195.0f

// Polynomial Coefficients
#define    P1_    -7.5625e-17
#define    P2_     1.9589e-13
#define    P3_    -1.8547e-10
#define    P4_     6.1997e-08
#define    P5_    -6.9432e-05
#define    P6_     0.9976


/***************************************************************************************************
*
* Fisheye Unwarping
*   Unwarp source fisheye -> 360x180 equirectangular
*
***************************************************************************************************/
void 
fish_unwarp( const cv::Mat &map_x, const cv::Mat &map_y, const cv::Mat &src, cv::Mat &dst )
{
    remap(src, dst, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
} /* fish_unwarp() */


/***************************************************************************************************
*
* Function: convert fisheye-vertical to equirectangular 
*   (reference: Panotool)
*
***************************************************************************************************/
void 
fish2Eqt( double x_dest, double  y_dest, double* x_src, double* y_src, double W_rad )
{
    double phi, theta, r, s;
    double v[3];
    phi   = x_dest / W_rad;
    theta = -y_dest / W_rad + CV_PI / 2;

    if (theta < 0)
    {
        theta = -theta;
        phi += CV_PI;
    }
    if (theta > CV_PI)
    {
        theta = CV_PI - (theta - CV_PI);
        phi += CV_PI;
    }

    s = sin(theta);
    v[0] = s * sin(phi);
    v[1] = cos(theta);
    r = sqrt(v[1] * v[1] + v[0] * v[0]);
    theta = W_rad * atan2(r, s * cos(phi));
    //
    *x_src = theta * v[0] / r;
    *y_src = theta * v[1] / r;

} /* fish2Eqt() */


/***************************************************************************************************
*
* Map 2D fisheye image to 2D projected sphere
*
***************************************************************************************************/
void 
fish_2D_map( cv::Mat &map_x, cv::Mat &map_y, int Hs, int Ws, int Hd, int Wd, float in_fovd )
{
    static double W_rad = Wd / (2 * CV_PI);
    double x_d, y_d; // dest
    double x_s, y_s; // source
    double w2  = (double)Wd / 2.0 - 0.5;
    double h2  = (double)Hd / 2.0 - 0.5;
    double ws2 = (double)Ws / 2.0 - 0.5;
    double hs2 = (double)Hs / 2.0 - 0.5;

    for (int y = 0; y < Hd; ++y)
    {
        // y-coordinate in dest image relative to center
        y_d = (double)y - h2;
        for (int x = 0; x < Wd; ++x)
        {
            x_d = (double)x - w2;
            // Convert fisheye coordinate to cartesian coordinate (equirectangular)
            fish2Eqt(x_d, y_d, &x_s, &y_s, W_rad);
            // Convert source cartesian coordinate to screen coordinate
            x_s += ws2;
            y_s += hs2;
            // Create map
            map_x.at<float>(y, x) = float(x_s);
            map_y.at<float>(y, x) = float(y_s);
        }
    }

} /* fish_2D_map() */


/***************************************************************************************************
*
* Mask creation for cropping image data inside the FOVD circle
*
***************************************************************************************************/
void 
fish_mask_erect( const int Ws, const int Hs, const float in_fovd, 
                 cv::Mat &cir_mask, cv::Mat &inner_cir_mask )
{
    Mat cir_mask_ = Mat(Hs, Ws, CV_8UC3);
    Mat inner_cir_mask_ = Mat(Hs, Ws, CV_8UC3);
    // Mat cir_mask_       = Mat(Hs, Ws, CV_8UC1);
    // Mat inner_cir_mask_ = Mat(Hs, Ws, CV_8UC1);

    int wShift = static_cast<int>(floor(((Ws * (MAX_FOVD - in_fovd) / MAX_FOVD) / 2.0f)));

    cout << "wshift = " << wShift << "\n";
    // Create Circular mask to crop the input W.R.T. FOVD
    int r1 = static_cast<int>(floor((double(Ws) / 2.0)));
    int r2 = static_cast<int>(floor((double(Ws) / 2.0 - wShift * 2.0)));
    circle(cir_mask_,       Point(int(Hs / 2), int(Ws / 2)), r1, Scalar(255, 255, 255), -1, 8, 0); // fill circle with 0xFF
    circle(inner_cir_mask_, Point(int(Hs / 2), int(Ws / 2)), r2, Scalar(255, 255, 255), -1, 8, 0); // fill circle with 0xFF

    cir_mask_.convertTo(cir_mask, CV_8UC3);
    inner_cir_mask_.convertTo(inner_cir_mask, CV_8UC3);
    // cir_mask_.convertTo(cir_mask, CV_8UC1);
    // inner_cir_mask_.convertTo(inner_cir_mask, CV_8UC1);
} /* fish_mask_erect() */


/***************************************************************************************************
*
* Rigid Moving Least Squares Interpolation
*
***************************************************************************************************/
void 
fish_rmls_deform( const cv::Mat &map_x, const cv::Mat &map_y, const cv::Mat &src, cv::Mat &dst )
{
    remap(src, dst, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
} /* fish_rmls_interp2() */


/***************************************************************************************************
*
* Fisheye Light Fall-off Compensation: Scale_Map Construction
*
***************************************************************************************************/
void 
fish_compen_map( const int H, const int W, const cv::Mat &R_pf, cv::Mat &scale_map )
{
    int W_ = floor(W / 2);
    int H_ = floor(H / 2);

    // Create IV quadrant map
    Mat scale_map_quad_4 = Mat::zeros(H_, W_, R_pf.type());
    float da = R_pf.at<float>(0, W_ - 1);
    int x, y;
    float r, a, b;

    for (x = 0; x < W_; ++x)
    {
        for (y = 0; y < H_; ++y)
        {
            r = floor(sqrt(std::pow(x, 2) + std::pow(y, 2)));
            if (r >= (W_ - 1))
            {
                scale_map_quad_4.at<float>(y, x) = da;
            }
            else
            {
                a = R_pf.at<float>(0, r);
                if ((x < W_) && (y < H_)) // within boundaries
                    b = R_pf.at<float>(0, r + 1);
                else // on boundaries
                    b = R_pf.at<float>(0, r);
                scale_map_quad_4.at<float>(y, x) = (a + b) / 2.0f;
            }
        } // x()
    } // y()

    // Assume Optical Symmetry & Flip
    Mat scale_map_quad_1(scale_map_quad_4.size(), scale_map_quad_4.type());
    Mat scale_map_quad_2(scale_map_quad_4.size(), scale_map_quad_4.type());
    Mat scale_map_quad_3(scale_map_quad_4.size(), scale_map_quad_4.type());
    // 
    cv::flip(scale_map_quad_4, scale_map_quad_1, 0); // quad I, up-down or around x-axis
    cv::flip(scale_map_quad_4, scale_map_quad_3, 1); // quad III, left-right or around y-axis
    cv::flip(scale_map_quad_1, scale_map_quad_2, 1); // quad II, up-down or around x-axis
    // 
    Mat quad_21, quad_34;
    cv::hconcat(scale_map_quad_2, scale_map_quad_1, quad_21);
    cv::hconcat(scale_map_quad_3, scale_map_quad_4, quad_34);
    // 
    cv::vconcat(quad_21, quad_34, scale_map);

} /* fisheye_compen_map() */


/***************************************************************************************************
*
* Fisheye Light Fall-off Compensation
*
***************************************************************************************************/
void 
fish_lighFO_compen( cv::Mat out_img, cv::Mat &in_img, const cv::Mat &scale_map )
{
    Mat rgb_ch[3];
    Mat rgb_ch_double[3];
    Mat out_img_double(in_img.size(), in_img.type());
    cv::split(in_img, rgb_ch);
    rgb_ch[0].convertTo(rgb_ch_double[0], scale_map.type());
    rgb_ch[1].convertTo(rgb_ch_double[1], scale_map.type());
    rgb_ch[2].convertTo(rgb_ch_double[2], scale_map.type());
    //
    rgb_ch_double[0] = rgb_ch_double[0].mul(scale_map); // element-wise multiplication
    rgb_ch_double[1] = rgb_ch_double[1].mul(scale_map);
    rgb_ch_double[2] = rgb_ch_double[2].mul(scale_map);
    cv::merge(rgb_ch_double, 3, out_img_double);

    out_img_double.convertTo(out_img, CV_8U);

} /* fish_lighFO_compen() */


void
fish_create_blend_mask( const cv::Mat &cir_mask, const cv::Mat &inner_cir_mask,
                  const int Ws, const int Hs, const int Wd, const int Hd,
                  const Mat &map_x, const Mat &map_y,
                  cv::Mat &binary_mask, std::vector<int> &blend_post )
{
    Mat inner_cir_mask_n;
    Mat ring_mask, ring_mask_unwarped;

    int Ws2 = static_cast<int>(Ws / 2);
    int Hs2 = static_cast<int>(Hs / 2);
    int Wd2 = static_cast<int>(Wd / 2);
    bitwise_not(inner_cir_mask, inner_cir_mask_n);

    cir_mask.copyTo(ring_mask, inner_cir_mask_n);
    
    remap(ring_mask, ring_mask_unwarped, map_x, map_y, INTER_LINEAR, 
          BORDER_CONSTANT, Scalar(0, 0, 0));
    
    Mat mask_ = ring_mask_unwarped(Rect(Wd2-Ws2, 0, Ws, Hd)); 
    mask_.convertTo(mask_, CV_8UC3);

    int H_ = mask_.size().height;
    int W_ = mask_.size().width;

    int ridx, cidx;

    const int first_zero_col = 120; // first cidx that mask value is zero
    const int first_zero_row =  45; // first ridx that mask value is zero

    // Clean up
    for( ridx=0; ridx < H_; ++ridx)
    {
        if( cidx < first_zero_col || cidx > W_-first_zero_col )
        {
            mask_.at<Vec3b>(Point(cidx,ridx)) = Vec3b(255,255,255);
        }
    }

    for( ridx=0; ridx < H_; ++ridx )
    {
        for( cidx=0; cidx < W_; ++cidx )
        {
            if( (ridx < (static_cast<int>(H_/2)) ) &&
                (cidx > first_zero_col-1) && 
                (cidx < W_-first_zero_col+1) )
            { 
                mask_.at<Vec3b>(Point(cidx,ridx)) = Vec3b(0, 0, 0);
            }
        }
    }

    int offset = 30;
    for( ridx=0; ridx < H_; ++ridx )
    {
        if( ridx > H_-first_zero_row )
        {
            blend_post.push_back( 0 ); 
            continue;
        }
        for( cidx=first_zero_col-10; cidx < W_/2+10; ++cidx )
        {
            Vec3b color = mask_.at<Vec3b>(Point(cidx,ridx));
            if( color == Vec3b(0,0,0))
            {
                blend_post.push_back( cidx - offset );
                break; 
            }
        } 
    }

    mask_.convertTo( binary_mask, CV_8UC3 );

#if MY_DEBUG
    cout << "size mask_ = " << mask_.size() << ", type = " << mask_.type() << ", ch = " << mask_.channels() << "\n";
    imwrite("mask.jpg", mask_);
#endif
}


/***************************************************************************************************
*
* Common Preparation
*    Output:   map_x, map_y(unwarping)
*              cir_mask(circular crop)
*
***************************************************************************************************/
void 
fish_common_init( cv::Mat &map_x, cv::Mat &map_y, cv::Mat &cir_mask, 
                  cv::Mat &scale_map, cv::Mat &mls_map_x, cv::Mat &mls_map_y,
                  cv::Mat &binary_mask, std::vector<int> &blend_post,
                  int Hs, const int Ws, const int Hd, 
                  const int Wd, const float fovd )
{
    // Unwarp map_x, map_y, cir_mask
    fish_2D_map(map_x, map_y, Hs, Ws, Hd, Wd, fovd);

    CV_Assert( (Hs % 2 == 0) && (Ws % 2 == 0) );

    // Create Circular mask to Crop the input W.R.T FOVD 
    // (mask all data outside the FOVD circle)
    Mat inner_cir_mask;
    double inner_fovd = 183.0;
    fish_mask_erect(Ws, Hs, inner_fovd, cir_mask, inner_cir_mask);

    fish_create_blend_mask( cir_mask, inner_cir_mask, Ws, Hs, Wd, Hd, 
                      map_x, map_y, binary_mask, blend_post );

    // Scale_Map Reconstruction for Fisheye Light Fall-off Compensation
    int H = Hs;
    int W = Ws;
    int W_ = W / 2;
    Mat x_coor = Mat::zeros(1, W_, CV_32F);
    Mat temp(x_coor.size(), x_coor.type());

    for (int i = 0; i < W_; ++i)
    {
        x_coor.at<float>(0, i) = i;
    }

    //  R_pf = P1_ * (x_coor.^5.0) + P2_ * (x_coor.^4.0) + P3_ * (x_coor.^3.0) + 
    //         P4_ * (x_coor.^2.0) + P5_ * x_coor + P6_;
    Mat R_pf = Mat::zeros(x_coor.size(), x_coor.type());
    cv::pow(x_coor, 5.0, temp);
    R_pf = R_pf + P1_ * temp;
    cv::pow(x_coor, 4.0, temp);
    R_pf = R_pf + P2_ * temp;
    cv::pow(x_coor, 3.0, temp);
    R_pf = R_pf + P3_ * temp;
    cv::pow(x_coor, 2.0, temp);
    R_pf = R_pf + P4_ * temp;
    R_pf = R_pf + P5_ * x_coor + P6_;
    //
    // PF_LUT
    cv::divide(1, R_pf, R_pf); //element-wise inverse

    // Generate Scale Map
    fish_compen_map(H, W, R_pf, scale_map);

    // Rigid MLS Interp2 grids
    //3840x1920 resolution (C200 video)
    cv::FileStorage fs("./utils/grid_xd_yd_3840x1920.yml.gz", FileStorage::READ);
    fs["Xd"] >> mls_map_x;
    fs["Yd"] >> mls_map_y;
    fs.release();

} /* fish_common_init() */


/***************************************************************************************************
*
* Adaptive Alignment: Norm XCorr
*    Input: Ref and Template images
*    Output: matchLoc (matching location)
*
***************************************************************************************************/
void 
fish_norm_xcorr( cv::Point2f &matchLoc, const cv::Mat &Ref, cv::Mat &Tmpl, const char* img_window, 
                bool disableDisplay )
{
    double tickStart, tickEnd, runTime;
    Mat img = Ref;
    Mat templ = Tmpl;
    Mat img_display, result;
    img.copyTo(img_display);
    int result_cols = img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;
    result.create(result_rows, result_cols, CV_32FC1);

    // Select Normalized Cross-Correlation as Template Matching Method
    int match_method = CV_TM_CCORR_NORMED;

    // Match template
    matchTemplate(img, templ, result, match_method);
    normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

    // Check for peak cross-correlation
    double minVal; double maxVal; Point minLoc; Point maxLoc;

    //Point matchLoc;
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

    if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED)
    {
        matchLoc = minLoc;
    }
    else // CV_TM_CCORR_NORMED
    {
        matchLoc = maxLoc;
    }

    if (!disableDisplay)
    {
        rectangle(img_display, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), 
                  Scalar(0, 255, 0), 2, 8, 0);
        Mat RefTemplCat;
        cv::hconcat(img_display, Tmpl, RefTemplCat);
        imshow(img_window, RefTemplCat);
    }

} /* fish_norm_xcorr() */


/***************************************************************************************************
*
* Construct Control Points for Affine2D estimation
*    Input: Matched Points in the template & ref
*    Output: movingPoints & fixedPoints
*
***************************************************************************************************/
void 
fish_ctrl_points_construct( vector<Point2f> &movingPoints, vector<Point2f> &fixedPoints, 
            const cv::Point2f &matchLocLeft, const cv::Point2f &matchLocRight, const int row_start, 
            const int row_end, const int p_wid, const int p_x1, const int p_x2, const int p_x2_ref )
{
    float x1 = matchLocLeft.x;
    float y1 = matchLocLeft.y;
    float x2 = matchLocRight.x;
    float y2 = matchLocRight.y;

    //-------------------------------------------------------------------------------
    // Construct MovingPoints pRef (matched points of template on reference)
    //-------------------------------------------------------------------------------
    // Left Boundary
    movingPoints.push_back(cv::Point2f(x1, y1 + row_start));                    // pRef_11
    movingPoints.push_back(cv::Point2f(x1 + p_wid, y1 + row_start));            // PRef_12
    movingPoints.push_back(cv::Point2f(x1, y1 + row_end));                      // pRef_13
    movingPoints.push_back(cv::Point2f(x1 + p_wid, y1 + row_end));              // pRef_14
    // Right Boundary
    movingPoints.push_back(cv::Point2f(x2 + p_x2_ref, y2 + row_start));         // pRef_21
    movingPoints.push_back(cv::Point2f(x2 + p_x2_ref + p_wid, y2 + row_start)); // pRef_22
    movingPoints.push_back(cv::Point2f(x2 + p_x2_ref, y2 + row_end));           // pRef_23
    movingPoints.push_back(cv::Point2f(x2 + p_x2_ref + p_wid, y2 + row_end));   // pRef_24

    //-------------------------------------------------------------------------------
    // Construct fixedPoint pTmpl (matched points of template on template)
    //-------------------------------------------------------------------------------
    // Left Boundary
    fixedPoints.push_back(cv::Point2f(p_x1, row_start));          // pTmpl_11
    fixedPoints.push_back(cv::Point2f(p_x1 + p_wid, row_start));  // pTmpl_12
    fixedPoints.push_back(cv::Point2f(p_x1, row_end));            // pTmpl_13
    fixedPoints.push_back(cv::Point2f(p_x1 + p_wid, row_end));    // pTmpl_14
    // Right boundary
    fixedPoints.push_back(cv::Point2f(p_x2, row_start));          // pTmpl_21
    fixedPoints.push_back(cv::Point2f(p_x2 + p_wid, row_start));  // pTmpl_22
    fixedPoints.push_back(cv::Point2f(p_x2, row_end));            // pTmpl_23
    fixedPoints.push_back(cv::Point2f(p_x2 + p_wid, row_end));    // pTmpl_24

} /* fish_ref_point_construct() */


/***************************************************************************************************
*
* Ramp Blending on the right patch
*
***************************************************************************************************/
void 
fish_blend_right( cv::Mat &bg, const cv::Mat &bg1, const cv::Mat &bg2 )
{
    int h = bg1.size().height;
    int w = bg1.size().width;
    Mat bg_ = Mat::zeros(bg1.size(), CV_32F);
    double alpha1, alpha2;
    Mat bg1_, bg2_;
    bg1.convertTo(bg1_, CV_32F);
    bg2.convertTo(bg2_, CV_32F);
    //
    Mat bgr_bg[3], bgr_bg1[3], bgr_bg2[3];   //destination array
    split(bg1_, bgr_bg1); //split source  
    split(bg2_, bgr_bg2); //split source  
    // 
    bgr_bg[0] = Mat::zeros(bgr_bg1[1].size(), CV_32F);
    bgr_bg[1] = Mat::zeros(bgr_bg1[1].size(), CV_32F);
    bgr_bg[2] = Mat::zeros(bgr_bg1[1].size(), CV_32F);

    for (int r = 0; r < h; ++r)
    {
        for (int c = 0; c < w; ++c)
        {
            alpha1 = double(c) / double(w);
            alpha2 = 1 - alpha1;
            bgr_bg[0].at<float>(r, c) = alpha1*bgr_bg1[0].at<float>(r, c) + alpha2*bgr_bg2[0].at<float>(r, c);
            bgr_bg[1].at<float>(r, c) = alpha1*bgr_bg1[1].at<float>(r, c) + alpha2*bgr_bg2[1].at<float>(r, c);
            bgr_bg[2].at<float>(r, c) = alpha1*bgr_bg1[2].at<float>(r, c) + alpha2*bgr_bg2[2].at<float>(r, c);
        }
    }
    cv::merge(bgr_bg, 3, bg);
    bg.convertTo(bg, CV_8U);

} /* fish_blend_right() */


/***************************************************************************************************
*
* Ramp Blending on the left patch
*
***************************************************************************************************/
void 
fish_blend_left( cv::Mat &bg, const cv::Mat &bg1, const cv::Mat &bg2 )
{
    int h = bg1.size().height;
    int w = bg1.size().width;
    Mat bg_ = Mat::zeros(bg1.size(), CV_32F);
    double alpha1, alpha2;
    Mat bg1_, bg2_;
    bg1.convertTo(bg1_, CV_32F);
    bg2.convertTo(bg2_, CV_32F);
    // 
    Mat bgr_bg[3], bgr_bg1[3], bgr_bg2[3];   //destination array
    split(bg1_, bgr_bg1); //split source  
    split(bg2_, bgr_bg2); //split source  
    // 
    bgr_bg[0] = Mat::zeros(bgr_bg1[1].size(), CV_32F);
    bgr_bg[1] = Mat::zeros(bgr_bg1[1].size(), CV_32F);
    bgr_bg[2] = Mat::zeros(bgr_bg1[1].size(), CV_32F);

    for (int r = 0; r < h; ++r)
    {
        for (int c = 0; c < w; ++c)
        {
            alpha1 = (double(w) - c + 1) / double(w);
            alpha2 = 1 - alpha1;
            bgr_bg[0].at<float>(r, c) = alpha1*bgr_bg1[0].at<float>(r, c) + alpha2*bgr_bg2[0].at<float>(r, c);
            bgr_bg[1].at<float>(r, c) = alpha1*bgr_bg1[1].at<float>(r, c) + alpha2*bgr_bg2[1].at<float>(r, c);
            bgr_bg[2].at<float>(r, c) = alpha1*bgr_bg1[2].at<float>(r, c) + alpha2*bgr_bg2[2].at<float>(r, c);
        }
    }
    cv::merge(bgr_bg, 3, bg);
    bg.convertTo(bg, CV_8U);

} /* fish_blend_left() */


/***************************************************************************************************
*
* Blending aligned images
*    Input: aligned images
*    Output: blended pano
*
***************************************************************************************************/
void 
fish_blend( const cv::Mat &left_img, const cv::Mat &right_img_aligned, cv::Mat &pano,
            const cv::Mat &binary_mask, const std::vector<int> blend_post )
{ 
#if GEAR360_C200
    Mat post;
    // Read YML
    cv::FileStorage fs("./utils/post_find.yml", FileStorage::READ);
    fs["post_ret"] >> post; // 1772 x 1
    fs.release();

    // Mask
    Mat mask = imread("./utils/mask_1920x1920_fovd_187.jpg", IMREAD_COLOR);
#else   // use the internally created
    // use `blend_post` instead of `post`
    // use `binary_mask` instead of `mask` from file
    Mat mask = binary_mask;
#endif
    int H = mask.size().height;
    int W = mask.size().width;

    int Worg = 1920;
    int imH = left_img.size().height;
    int imW = left_img.size().width;
    Mat left_img_cr = left_img(Rect(imW / 2 + 1 - Worg / 2, 0, Worg, imH));
    int  sideW = 45;
    Mat left_blend, right_blend;

    for (int r = 0; r < H; ++r)
    {
#if GEAR360_C200 
        int p = post.at<float>(r, 0);
#else
        int p = blend_post[r];
#endif
        if (p == 0)
        {
            continue;
        }
        // Left boundary
        Mat lf_win_1 = left_img_cr(Rect(p - sideW, r, 2 * sideW, 1));
        Mat rt_win_1 = right_img_aligned(Rect(p - sideW, r, 2 * sideW, 1));
        // Right boundary
        Mat lf_win_2 = left_img_cr(Rect((W - p - sideW), r, 2 * sideW, 1));
        Mat rt_win_2 = right_img_aligned(Rect((W - p - sideW), r, 2 * sideW, 1));
        // Blend(ramp)
        Mat bleft, bright;
        fish_blend_left(bleft, lf_win_1, rt_win_1);
        fish_blend_right(bright, lf_win_2, rt_win_2);
        // Update left boundary
        bleft.copyTo(lf_win_1);
        bleft.copyTo(rt_win_1);
        // Update right boundary
        bright.copyTo(lf_win_2);
        bright.copyTo(rt_win_2);
    }

#if MY_DEBUG
    imwrite("left_crop_blend.jpg", left_img_cr);
    imwrite("right_blend.jpg", right_img_aligned);
#endif

    // BLENDING
    Mat mask_ = mask(Rect(0, 0, mask.size().width, mask.size().height - 2));
    Mat mask_n;
    bitwise_not(mask_, mask_n);
    bitwise_and(left_img_cr, mask_, left_img_cr); // Left image
    // 
    Mat temp1 = left_img(Rect(0, 0, (imW / 2 - Worg / 2), imH));
    Mat temp2 = left_img(Rect((imW / 2 + Worg / 2), 0, (imW / 2 - Worg / 2), imH));
    Mat t;
    cv::hconcat(temp1, left_img_cr, t);
    cv::hconcat(t, temp2, left_img);
    // 
    bitwise_and(right_img_aligned, mask_n, right_img_aligned); // Right image
    //
    pano = left_img;
    Mat temp = pano(Rect((imW / 2 - Worg / 2), 0, Worg, imH));
    Mat t2;
    bitwise_or(temp, right_img_aligned, t2);
    t2.copyTo(temp); // updated pano

} /* fish_blend() */


/***************************************************************************************************
*
* Single frame stitch
*
****************************************************************************************************/
void 
fish_stitch_one( cv::Mat &pano, cv::Mat &in_img_L, cv::Mat &in_img_R, const cv::Mat &cir_mask, 
        cv::Mat &scale_map, const cv::Mat &map_x, const cv::Mat &map_y, const cv::Mat &mls_map_x, 
        const cv::Mat &mls_map_y, const int Hs, const int Ws, const int Hd, const int Wd, 
        const int W_in, const bool disable_light_compen, const bool disable_refine_align,
        const Mat &binary_mask, const std::vector<int> blend_post )
{
    Mat left_unwarped, right_unwarped;
    double tickStart, tickEnd, runTime;

#if PROFILING
    tickStart = double(getTickCount());
#endif

    //--------------------------------------------------------------------------
    // Circular Crop
    //--------------------------------------------------------------------------
    bitwise_and(in_img_L, cir_mask, in_img_L); // Left image
    bitwise_and(in_img_R, cir_mask, in_img_R); // Right image

#if PROFILING
    tickEnd = double(getTickCount());
    runTime = (tickEnd - tickStart) / getTickFrequency();
    tickStart = tickEnd;
    std::cout << "run-time (Crop) = " << runTime << " (sec)" << "\n";
#endif

    //--------------------------------------------------------------------------
    // Light Fall-off Compensation
    //--------------------------------------------------------------------------
    Mat left_img_compensated(in_img_L.size(), in_img_L.type());
    Mat right_img_compensated(in_img_R.size(), in_img_R.type());
    if (disable_light_compen)
    {
        left_img_compensated = in_img_L;
        right_img_compensated = in_img_R;
    }
    else
    {
        fish_lighFO_compen(left_img_compensated, in_img_L, scale_map);
        fish_lighFO_compen(right_img_compensated, in_img_R, scale_map);
    }

#if PROFILING
    tickEnd = double(getTickCount());
    runTime = (tickEnd - tickStart) / getTickFrequency();
    tickStart = tickEnd;
    std::cout << "run-time (LightCompen) = " << runTime << " (sec)" << "\n";
#endif

    //--------------------------------------------------------------------------
    // Fisheye Unwarping
    //--------------------------------------------------------------------------
    fish_unwarp(map_x, map_y, left_img_compensated, left_unwarped);
    fish_unwarp(map_x, map_y, right_img_compensated, right_unwarped);

#if PROFILING
    tickEnd = double(getTickCount());
    runTime = (tickEnd - tickStart) / getTickFrequency();
    tickStart = tickEnd;
    std::cout << "run-time (Unwarp) = " << runTime << " (sec)" << "\n";
#endif

#if MY_DEBUG
    imwrite("l.jpg", left_unwarped);
    imwrite("r.jpg", right_unwarped);
#endif

#if PROFILING
    tickStart = double(getTickCount());
#endif

    //--------------------------------------------------------------------------
    // Rigid Moving Least Squares Deformation
    //--------------------------------------------------------------------------
    Mat rightImg_crop, rightImg_mls_deformed;
    rightImg_crop = right_unwarped(Rect(int(Wd / 2) - (W_in / 2), 0, W_in, Hd - 2)); // notice on (Hd-2) --> become: (Hd)
    fish_rmls_deform(mls_map_x, mls_map_y, rightImg_crop, rightImg_mls_deformed);

#if PROFILING
    tickEnd = double(getTickCount());
    runTime = (tickEnd - tickStart) / getTickFrequency();
    tickStart = tickEnd;
    std::cout << "run-time (MLS Deform) = " << runTime << " (sec)" << "\n";
#endif

    //--------------------------------------------------------------------------
    // Rearrange Image for Adaptive Alignment
    //--------------------------------------------------------------------------
    Mat temp1 = left_unwarped(Rect(0, 0, int(Wd / 2), Hd - 2));
    Mat temp2 = left_unwarped(Rect(int(Wd / 2), 0, int(Wd / 2), Hd - 2));
    Mat left_unwarped_arr; // re-arranged left unwarped
    cv::hconcat(temp2, temp1, left_unwarped_arr);
    Mat leftImg_crop;
    leftImg_crop = left_unwarped_arr(Rect(int(Wd / 2) - (W_in / 2), 0, W_in, Hd - 2)); 
    uint16_t crop = uint16_t(0.5 * Ws * (195.0 - 180.0) / 195.0); // half overlap region

    //--------------------------------------------------------------------------
    // PARAMETERS (hard-coded) for C200 videos 
    //--------------------------------------------------------------------------
    uint16_t p_wid = 55;
    uint16_t p_x1 = 90 - 15;
    uint16_t p_x2 = 1780 - 5;
    uint16_t p_x1_ref = 2 * crop;
    uint16_t row_start = 590;
    uint16_t row_end = 1320;
    uint16_t p_x2_ref = Ws - 2 * crop + 1;
    // 
    Mat Ref_1, Ref_2, Tmpl_1, Tmpl_2;
    Ref_1  = leftImg_crop(Rect(0, row_start, p_x1_ref, row_end - row_start));
    Ref_2  = leftImg_crop(Rect(p_x2_ref, row_start, Ws - p_x2_ref, row_end - row_start));
    Tmpl_1 = rightImg_mls_deformed(Rect(p_x1, row_start, p_wid, row_end - row_start));
    Tmpl_2 = rightImg_mls_deformed(Rect(p_x2, row_start, p_wid, row_end - row_start));

#if MY_DEBUG
    imwrite("./output/Ref_1.jpg", Ref_1);
    imwrite("./output/Ref_2.jpg", Ref_2);
    imwrite("./output/Tmpl_1.jpg", Tmpl_1);
    imwrite("./output/Tmpl_2.jpg", Tmpl_2);
#endif


    //--------------------------------------------------------------------------
    // Adaptive Alignment (Norm XCorr)
    //--------------------------------------------------------------------------
    bool disableDisplay = 1;   // 1: display off, 0: display on
    const char* name_bound_1 = "Matching On Left Boundary";
    const char* name_bound_2 = "Matching On Right Boundary";

#if PROFILING
    tickStart = double(getTickCount());
#endif

    cv::Mat warpedRightImg;
    if (disable_refine_align)
    {
        warpedRightImg = rightImg_mls_deformed;
    }
    else
    {
        //--------------------------------------------------------------
        // Normalized XCorr
        //--------------------------------------------------------------
        cv::Point2f matchLocLeft, matchLocRight;
        fish_norm_xcorr(matchLocLeft, Ref_1, Tmpl_1, name_bound_1, disableDisplay); // Left boundary
        fish_norm_xcorr(matchLocRight, Ref_2, Tmpl_2, name_bound_2, disableDisplay); // Right boundary

#if MY_DEBUG
        cout << "matchLocLeft(x=" << matchLocLeft.x << ", y=" << matchLocLeft.y << "), matchLocRight(x=" << matchLocRight.x << ", y=" << matchLocRight.y << ")\n";
#endif

        vector<Point2f> movingPoints;  // matched points in Refs  
        vector<Point2f> fixedPoints;   // matched points in Templates

        //--------------------------------------------------------------
        // Construct control points for estimation
        //--------------------------------------------------------------
        fish_ctrl_points_construct(movingPoints, fixedPoints, matchLocLeft, matchLocRight, row_start, row_end, p_wid, p_x1, p_x2, p_x2_ref);

#if PROFILING
        tickEnd = double(getTickCount());
        runTime = (tickEnd - tickStart) / getTickFrequency();
        tickStart = tickEnd;
        std::cout << "run-time (Xcorr & fitGeoTrans) = " << runTime << " (sec)" << "\n";
#endif

        cv::Mat tform_refine_mat;
        //--------------------------------------------------------------
        // Estimate affine matrix
        //--------------------------------------------------------------
        tform_refine_mat = cv::findHomography(fixedPoints, movingPoints, 0);

        //--------------------------------------------------------------
        // Warp Image
        //--------------------------------------------------------------
        cv::warpPerspective(rightImg_mls_deformed, warpedRightImg, tform_refine_mat, 
                            rightImg_mls_deformed.size(), INTER_LINEAR);

#if PROFILING
        tickEnd = double(getTickCount());
        runTime = (tickEnd - tickStart) / getTickFrequency();
        tickStart = tickEnd;
        std::cout << "run-time (estimate tform_mat & warping) = " << runTime << " (sec)" << "\n";
#endif

    } // Normalized xcorr

    //--------------------------------------------------------------
    // Blend Images
    //--------------------------------------------------------------
    fish_blend(left_unwarped_arr, warpedRightImg, pano, binary_mask, blend_post);

#if PROFILING
    tickEnd = double(getTickCount());
    runTime = (tickEnd - tickStart) / getTickFrequency();
    tickStart = tickEnd;
    std::cout << "run-time (Blending) = " << runTime << " (sec)" << "\n";
#endif

} /* fish_stitch_one() */


/***************************************************************************************************
*
* Main()
*
***************************************************************************************************/
int 
main( int argc, char** argv )
{
    // default values
    string img_name, in_dir_name, in_img_name, in_name_L, in_name_R, out_dir_name;
    float fovd = 195.0f;
    bool  disable_light_compen = 1;
    bool  disable_refine_align = 1;
    int fr_from = 1;
    int fr_to = 1;
    int W_in = 1920; // default: video 3840x1920

    if (argc == 1)
    {
        cout << "Wrong arguments\n";
        return -1;
    }

    for (int i = 1; i < argc; ++i)
    {
        if (string(argv[i]) == "--out_dir")
        {
            out_dir_name = argv[i + 1];
            i++;
        }
        else if (string(argv[i]) == "--in")
        {
            in_img_name = argv[i + 1];
            i++;
        }
        else if (string(argv[i]) == "--in_dir")
        {
            in_dir_name = argv[i + 1];
            i++;
        }
        else if (string(argv[i]) == "--fr")
        {
            fr_from = (int)atof(argv[i + 1]);
            i++;
            fr_to = (int)atof(argv[i + 1]);
            i++;
        }
        else if (string(argv[i]) == "--fovd")
        {
            fovd = (float)atof(argv[i + 1]);
            i++;
        }
        else if (string(argv[i]) == "--enable_light_compen")
        {
            disable_light_compen = 0;
        }
        else if (string(argv[i]) == "--enable_refine_align")
        {
            disable_refine_align = 0;
        }
        else
            ;
    }

    string out_name_L, out_name_R;
    Mat in_img, in_img_L, in_img_R;
    double endTime;
    double totalTime;

    //**************************************************************************
    // PreComputation
    //**************************************************************************
    char buf[4 + 1]; //+1 extra char for NULL. Exception thrown for buf[4]
    int n = 0;
    n = snprintf(buf, sizeof(buf), "%04d", fr_from);
    img_name = in_dir_name + "/" + in_img_name + buf + ".jpg";
     
    cout << "FishEye  --in_dir " << in_dir_name.c_str() << "  --in_img " << in_img_name.c_str() 
         << "  --out " << out_dir_name.c_str() << "  --fovd " << fovd << "  --fr_from " << fr_from 
         << "  --fr_to " << fr_to 
         << "  --enable_light_compen " << ~disable_light_compen 
         << "  --enable_refine_align " << ~disable_refine_align
         << "\n";

    in_img = imread(img_name.c_str(), IMREAD_COLOR);
    if (in_img.empty())
    {
        cout << "!! Could not open reference fisheye image: " << img_name.c_str() << " !!\n";
        return -1;
    }
    int Worg = in_img.size().width;
    int Horg = in_img.size().height;
    // Source image
    int Ws = int(Worg / 2);
    int Hs = Horg;
    // Destination pano
    int Wd = int(Ws * 360.0 / MAX_FOVD);
    int Hd = int(Wd / 2);

    //--------------------------------------------------------------------------
    // Initialization of all precomputed data
    //--------------------------------------------------------------------------
    cout << "Fisheye Stitcher Initialization..\n";
    //
    Mat map_x(Hd, Wd, CV_32FC1);
    Mat map_y(Hd, Wd, CV_32FC1);
    Mat cir_mask;
    Mat scale_map(Hs, Ws, CV_32F);
    Mat mls_map_x, mls_map_y;
    Mat binary_mask;             // blending
    std::vector<int> blend_post; // blending
    //
    fish_common_init(map_x, map_y, cir_mask, scale_map, mls_map_x, mls_map_y, 
                     binary_mask, blend_post, Hs, Ws, Hd, Wd, 195);
    // 
    in_img.release(); 
    double startTime = double(getTickCount()); // frame stitching starts 
    cv::Mat pano;

    //--------------------------------------------------------------------------
    // Read frames and process
    //--------------------------------------------------------------------------
    for (int fr = fr_from; fr < fr_to + 1; ++fr)
    { 
        n = snprintf(buf, sizeof(buf), "%04d", fr);
        img_name = in_dir_name + "/" + in_img_name + buf + ".jpg";
        // Read input fisheye images
        in_img = imread(img_name.c_str(), IMREAD_COLOR);
        in_img_L = in_img(Rect(0, 0, Worg / 2, Horg));        // left fisheye
        in_img_R = in_img(Rect(Worg / 2, 0, Worg / 2, Horg)); // right fisheye
        double startOneFrTime = double(getTickCount()); // frame stitching starts

        //--------------------------------------------------------------
        // Stitch
        //--------------------------------------------------------------
        fish_stitch_one(pano, in_img_L, in_img_R, cir_mask, scale_map, 
                        map_x, map_y, mls_map_x, mls_map_y, Hs, Ws, Hd, Wd, 
                        W_in, disable_light_compen, disable_refine_align,
                        binary_mask, blend_post );

        // RunTime
        endTime = double(getTickCount());
        totalTime = (endTime - startOneFrTime) / getTickFrequency();
        cout << "Stitching frame: " << fr << " (" << totalTime << " sec)\n";

        //--------------------------------------------------------------
        // Output pano
        //--------------------------------------------------------------
        string out_name = out_dir_name + "/pano_" + in_img_name + buf + ".jpg";

#if PROFILING
        double tickStart = endTime; // previous count
#endif

        Mat resize_pano(1920, 3840, pano.type());
        resize(pano, resize_pano, resize_pano.size(), 0, 0, INTER_LINEAR);

#if PROFILING
        double tickEnd = double(getTickCount());
        double runTime = (tickEnd - tickStart) / getTickFrequency();
        tickStart = tickEnd;
        std::cout << "run-time (resize) = " << runTime << " (sec)" << "\n";
#endif

        imwrite(out_name.c_str(), resize_pano);
    } /* for( all frames ) */

    // RunTime
    endTime = double(getTickCount());
    totalTime = (endTime - startTime) / getTickFrequency();
    std::cout << "run-time (sequence) = " << totalTime / 60 << " min" << " (" << totalTime << " sec)\n";

    return 0;

} /* main() */
