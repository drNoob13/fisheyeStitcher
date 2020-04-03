//----------------------------------------------------------------------------//
//                                                                            //
// This file is part of the fisheye stitcher project.                         //
// Copyright (c) 2018-2020 Tuan Phan Minh Ho <drnoob2013@gmail.com>           //
// https://github.com/drNoob13/fisheyeStitcher                                //
//                                                                            //
//----------------------------------------------------------------------------//
#include "fisheye_stitcher.hpp"

namespace stitcher
{

FisheyeStitcher::FisheyeStitcher(int width, int height, float in_fovd, 
                                 bool enb_light_compen, bool enb_refine_align,
                                 std::string map_path ) 
:   m_ws_org(width), m_hs_org(height), m_in_fovd(195.0f), m_inner_fovd(183.0f),
    m_enb_light_compen(enb_light_compen), m_enb_refine_align(enb_refine_align),
    m_map_path(map_path)
{ 
    CV_Assert( (width % 2 == 0) && (height % 2 == 0) );

    // Source images
    m_ws = static_cast<int>(width / 2); // e.g. 1920 
    m_hs = height; // e.g. 1920
    CV_Assert( (m_ws % 2 == 0) && (m_hs % 2 == 0) );

    // Destination pano
    m_wd = static_cast<int>(m_ws * 360.0 / MAX_FOVD);
    m_hd = static_cast<int>(m_wd / 2);
    m_wd2 = static_cast<int>(m_wd / 2);

    std::cout << "Initializing necessary parameters..\n";
    init();
}

FisheyeStitcher::~FisheyeStitcher()
{
}

//! 
//! @brief: Fisheye Unwarping
//!
//!   Unwarp source fisheye -> 360x180 equirectangular
//!
cv::Mat
FisheyeStitcher::unwarp( const cv::Mat &src )
{
    cv::Mat dst(src.size(), src.type());
    remap(src, dst, m_map_x, m_map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 
          cv::Scalar(0, 0, 0));
    return dst;

}   // unwarp()

//! 
//! @brief Convert fisheye-vertical to equirectangular (reference: Panotool)
//! 
std::tuple<double, double>  
FisheyeStitcher::fish2Eqt( const double x_dest, const double y_dest, 
                           const double W_rad )
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
    double x_src = theta * v[0] / r;
    double y_src = theta * v[1] / r;

    return std::make_tuple(x_src, y_src);

}   // fish2Eqt()

//! 
//! @brief Map 2D fisheye image to 2D projected sphere
//! @param  map_x output map for x element.
//! @param  map_y output map for y element.
//! 
void
FisheyeStitcher::fish2Map()
{
    cv::Mat map_x(m_hd, m_wd, CV_32FC1);
    cv::Mat map_y(m_hd, m_wd, CV_32FC1);
    double w_rad = m_wd / (2.0 * CV_PI);
    double x_d, y_d; // dest
    double x_s, y_s; // source
    double w2  = static_cast<double>(m_wd) / 2.0 - 0.5;
    double h2  = static_cast<double>(m_hd) / 2.0 - 0.5;
    double ws2 = static_cast<double>(m_ws) / 2.0 - 0.5;
    double hs2 = static_cast<double>(m_hs) / 2.0 - 0.5;

    for (int y = 0; y < m_hd; ++y)
    {
        // y-coordinate in dest image relative to center
        y_d = static_cast<double>(y) - h2;
        for (int x = 0; x < m_wd; ++x)
        {
            x_d = static_cast<double>(x) - w2;

            // Convert fisheye coordinate to cartesian coordinate (equirectangular)
            std::tie(x_s, y_s) = fish2Eqt(x_d, y_d, w_rad);

            // Convert source cartesian coordinate to screen coordinate
            x_s += ws2;
            y_s += hs2;
            // Create map
            map_x.at<float>(y, x) = static_cast<float>(x_s);
            map_y.at<float>(y, x) = static_cast<float>(y_s);
        }
    }
    // return std::make_tuple(map_x, map_y);
    map_x.copyTo(m_map_x);
    map_y.copyTo(m_map_y);

}   // fish2Map()

//! 
//! @brief Mask creation for cropping image data inside the FOVD circle
//! @param  cir_mask  output circular mask
//! @param  inner_cir_mask output circular mask for the inner circle
//! 
void
FisheyeStitcher::createMask() 
{
    cv::Mat cir_mask_ = cv::Mat(m_hs, m_ws, CV_8UC3);
    cv::Mat inner_cir_mask_ = cv::Mat(m_hs, m_ws, CV_8UC3);

    int wShift = static_cast<int>(std::floor(
                    ((m_ws * (MAX_FOVD - m_inner_fovd) / MAX_FOVD) / 2.0f)));

    // Create Circular mask to crop the input W.R.T. FOVD
    int r1 = static_cast<int>(std::floor((double(m_ws) / 2.0)));
    int r2 = static_cast<int>(std::floor((double(m_ws) / 2.0 - wShift * 2.0)));
    cv::circle(cir_mask_,       cv::Point(int(m_hs / 2), int(m_ws / 2)), r1, 
               cv::Scalar(255, 255, 255), -1, 8, 0); // fill circle with 0xFF
    cv::circle(inner_cir_mask_, cv::Point(int(m_hs / 2), int(m_ws / 2)), r2, 
               cv::Scalar(255, 255, 255), -1, 8, 0); // fill circle with 0xFF

    cv::Mat cir_mask;
    cv::Mat inner_cir_mask;
    cir_mask_.convertTo(cir_mask, CV_8UC3);
    inner_cir_mask_.convertTo(inner_cir_mask, CV_8UC3);
    cir_mask.copyTo(m_cir_mask);
    inner_cir_mask.copyTo(m_inner_cir_mask);

}   // createMask() 

//! 
//! @brief  Rigid Moving Least Squares Interpolation
//! @param  src  input source image
//! @param  dst  output deformed image
//!
cv::Mat
FisheyeStitcher::deform( const cv::Mat &src )
{
    cv::Mat dst(src.size(), src.type());
    cv::remap(src, dst, m_mls_map_x, m_mls_map_y, CV_INTER_LINEAR, 
          cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    return dst;

}   // deform() 

//! 
//! @brief Fisheye Light Fall-off Compensation: Scale_Map Construction
//! @param R_pf       input reverse profile model
//! 
void
FisheyeStitcher::genScaleMap()
{
    // TODO: remove duplicate params

    //------------------------------------------------------------------------//
    // Generate R_pf (reverse light fall-off profile)                         //
    //------------------------------------------------------------------------//
    int H = m_hs;
    int W = m_ws;
    int W_ = std::floor(m_ws / 2);
    int H_ = std::floor(m_hs / 2);
    cv::Mat x_coor = cv::Mat::zeros(1, W_, CV_32F);
    cv::Mat temp(x_coor.size(), x_coor.type());

    for (int i = 0; i < W_; ++i)
    {
        x_coor.at<float>(0, i) = i;
    }

    //-----------------------------------------------------------------------//
    //  R_pf = P1_ * (x_coor.^5.0) + P2_ * (x_coor.^4.0) +                   //
    //         P3_ * (x_coor.^3.0) + P4_ * (x_coor.^2.0) +                   //
    //         P5_ * x_coor        + P6_;                                    //
    //-----------------------------------------------------------------------//
    cv::Mat R_pf = cv::Mat::zeros(x_coor.size(), x_coor.type());
    cv::pow(x_coor, 5.0, temp);
    R_pf = R_pf + P1_ * temp;
    cv::pow(x_coor, 4.0, temp);
    R_pf = R_pf + P2_ * temp;
    cv::pow(x_coor, 3.0, temp);
    R_pf = R_pf + P3_ * temp;
    cv::pow(x_coor, 2.0, temp);
    R_pf = R_pf + P4_ * temp;
    R_pf = R_pf + P5_ * x_coor + P6_;
    
    // PF_LUT
    cv::divide(1, R_pf, R_pf); //element-wise inverse

    //------------------------------------------------------------------------//
    // Generate scale map                                                     //
    //------------------------------------------------------------------------//
    // Create IV quadrant map
    cv::Mat scale_map_quad_4 = cv::Mat::zeros(H_, W_, R_pf.type());
    float da = R_pf.at<float>(0, W_ - 1);
    int x, y;
    float r, a, b;

    for (x = 0; x < W_; ++x)
    {
        for (y = 0; y < H_; ++y)
        {
            r = std::floor(sqrt(std::pow(x, 2) + std::pow(y, 2)));
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
    cv::Mat scale_map_quad_1(scale_map_quad_4.size(), scale_map_quad_4.type());
    cv::Mat scale_map_quad_2(scale_map_quad_4.size(), scale_map_quad_4.type());
    cv::Mat scale_map_quad_3(scale_map_quad_4.size(), scale_map_quad_4.type());
    // 
    cv::flip(scale_map_quad_4, scale_map_quad_1, 0); // quad I, up-down or around x-axis
    cv::flip(scale_map_quad_4, scale_map_quad_3, 1); // quad III, left-right or around y-axis
    cv::flip(scale_map_quad_1, scale_map_quad_2, 1); // quad II, up-down or around x-axis
    // 
    cv::Mat quad_21, quad_34;
    cv::hconcat(scale_map_quad_2, scale_map_quad_1, quad_21);
    cv::hconcat(scale_map_quad_3, scale_map_quad_4, quad_34);
    // 
    cv::Mat scale_map;
    cv::vconcat(quad_21, quad_34, scale_map);

    scale_map.copyTo(m_scale_map);

}   // genScaleMap() 

//! 
//! @brief  Fisheye Light Fall-off Compensation
//! @param  in_img  LFO-uncompensated image
//! @param  out_img  LFO-compensated image
//!
cv::Mat
FisheyeStitcher::compenLightFO( const cv::Mat &in_img ) 
{
    cv::Mat rgb_ch[3];
    cv::Mat rgb_ch_double[3];
    cv::Mat out_img_double(in_img.size(), in_img.type());
    cv::split(in_img, rgb_ch);
    rgb_ch[0].convertTo(rgb_ch_double[0], m_scale_map.type());
    rgb_ch[1].convertTo(rgb_ch_double[1], m_scale_map.type());
    rgb_ch[2].convertTo(rgb_ch_double[2], m_scale_map.type());
    //
    rgb_ch_double[0] = rgb_ch_double[0].mul(m_scale_map); // element-wise multiplication
    rgb_ch_double[1] = rgb_ch_double[1].mul(m_scale_map);
    rgb_ch_double[2] = rgb_ch_double[2].mul(m_scale_map);
    cv::merge(rgb_ch_double, 3, out_img_double);

    cv::Mat out_img;
    out_img_double.convertTo(out_img, CV_8U);
    return out_img;

}   // compenLightFO() 

//! 
//! @brief Create binary mask for blending
//!
void
FisheyeStitcher::createBlendMask() 
{
    cv::Mat inner_cir_mask_n;
    cv::Mat ring_mask, ring_mask_unwarped;

    int Ws2 = static_cast<int>(m_ws / 2);
    int Hs2 = static_cast<int>(m_hs / 2);
    int Wd2 = static_cast<int>(m_wd / 2);
    cv::bitwise_not(m_inner_cir_mask, inner_cir_mask_n);

    m_cir_mask.copyTo(ring_mask, inner_cir_mask_n); // masking

#if MY_DEBUG
    std::cout << "Ws = " << m_ws << ", Hs = " << m_hs << "\n";
    std::cout << "Wd = " << m_wd << ", Hd = " << m_hd << "\n"; 
    cv::imwrite("m_cir_mask.jpg", m_cir_mask);
    cv::imwrite("ring_mask.jpg", ring_mask);
#endif
    
    cv::remap(ring_mask, ring_mask_unwarped, m_map_x, m_map_y, CV_INTER_LINEAR,
              cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    
    cv::Mat mask_ = ring_mask_unwarped(cv::Rect(Wd2-Ws2, 0, m_ws, m_hd)); 
    mask_.convertTo(mask_, CV_8UC3);

#if MY_DEBUG
    cv::imwrite("mask_.jpg", mask_);
#endif

    int H_ = mask_.size().height;
    int W_ = mask_.size().width;

    int ridx, cidx;

    // Hard-coded for dual-fisheye image of size 3840x1920
    const int first_zero_col = 120; // first cidx that mask value is zero
    const int first_zero_row =  45; // first ridx that mask value is zero

    // Clean up
    for( ridx=0; ridx < H_; ++ridx)
    {
        if( cidx < first_zero_col || cidx > W_-first_zero_col )
        {
            mask_.at<cv::Vec3b>(cv::Point(cidx,ridx)) = cv::Vec3b(255,255,255);
        }
    }
    for( ridx=0; ridx < H_; ++ridx )
    {
        for( cidx=0; cidx < W_; ++cidx )
        {
            if( (ridx < (static_cast<int>(H_/2)) ) &&
                (cidx > first_zero_col - 1) && 
                (cidx < W_ - first_zero_col + 1) )
            { 
                mask_.at<cv::Vec3b>(cv::Point(cidx,ridx)) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    // Create m_blend_post
    int offset = 15;
    for( ridx=0; ridx < H_; ++ridx )
    {
        if( ridx > H_ - first_zero_row )
        {
            m_blend_post.push_back( 0 ); 
            continue;
        }
        for( cidx=first_zero_col-10; cidx < W_/2+10; ++cidx )
        {
            cv::Vec3b color = mask_.at<cv::Vec3b>(cv::Point(cidx,ridx));
            if( color == cv::Vec3b(0,0,0))
            {
                m_blend_post.push_back( cidx - offset );
                break; 
            }
        } 
    }

    // generate binary mask
    cv::Mat binary_mask;
    mask_.convertTo( binary_mask, CV_8UC3 );
    binary_mask.copyTo( m_binary_mask );

#if MY_DEBUG
    std::cout << "size mask_ = " << mask_.size() << ", type = " << mask_.type()
              << ", ch = " << mask_.channels() << "\n";
    cv::imwrite("binary_mask.jpg", binary_mask);
#endif

}   // createBlendMask()

// 
// @brief Initialize common parameters for stitching
// 
void 
FisheyeStitcher::init() 
{
    //------------------------------------------------------------------------//
    // Create deformation maps                                                //
    //------------------------------------------------------------------------//
    fish2Map(); // update m_map_x and m_map_y

    //------------------------------------------------------------------------//
    // Create Circular mask to Crop the input W.R.T FOVD                      //
    //------------------------------------------------------------------------//
    // (mask all data outside the FOVD circle)
    createMask(); // update m_cir_mask, m_inner_cir_mask 

    //------------------------------------------------------------------------//
    // Creat masks that used in blending the deformed images                  //
    //------------------------------------------------------------------------//
    createBlendMask();  // update m_blend_post, m_binary_mask

    //------------------------------------------------------------------------//
    // Create scale_map for fisheye light fall-off compensation               //
    //------------------------------------------------------------------------//
    genScaleMap(); // update m_scale_map

    //------------------------------------------------------------------------//
    // Read rigid MLS interp grids from file                                  //
    //------------------------------------------------------------------------//
    cv::Mat mls_map_x, mls_map_y;
    // 3840x1920 resolution (C200 video)
    cv::FileStorage fs(m_map_path, cv::FileStorage::READ);
    if( fs.isOpened())
    {
        fs["Xd"] >> mls_map_x;
        fs["Yd"] >> mls_map_y;
        fs.release();
    }
    else
    {
        CV_Error_(cv::Error::StsBadArg, 
            ("Cannot open map file1: %s", m_map_path.c_str()));
    }
    mls_map_x.copyTo(m_mls_map_x);
    mls_map_y.copyTo(m_mls_map_y);

}   // init()


// 
// @brief Adaptive Alignment: Norm XCorr
// @param  Ref input reference image
// @param  Tmpl input template image
// @param  matchLoc output matching location 
//
cv::Point2f
FisheyeStitcher::findMatchLoc( const cv::Mat &Ref, 
                               const cv::Mat &Tmpl, 
                               const std::string &img_window, 
                               const bool disable_display )
{
    cv::Point2f matchLoc;
    double tickStart, tickEnd, runTime;
    cv::Mat img = Ref;
    cv::Mat templ = Tmpl;
    cv::Mat img_display, result;
    img.copyTo(img_display);
    int result_cols = img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;
    result.create(result_rows, result_cols, CV_32FC1);

    // Select Normalized Cross-Correlation as Template Matching Method
    int match_method = CV_TM_CCORR_NORMED;

    // Match template
    cv::matchTemplate(img, templ, result, match_method);
    cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    // Check for peak cross-correlation
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;

    // Point matchLoc
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    if (match_method == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED)
    {
        matchLoc = minLoc;
    }
    else // CV_TM_CCORR_NORMED
    {
        matchLoc = maxLoc;
    }

    if (!disable_display)
    {
        cv::rectangle(img_display, matchLoc, 
                      cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), 
                      cv::Scalar(0, 255, 0), 2, 8, 0);
        cv::Mat RefTemplCat;
        cv::hconcat(img_display, Tmpl, RefTemplCat);
        cv::imshow(img_window, RefTemplCat);
    }
    return matchLoc;

}   // findMatchLoc()

//! 
//! @brief  Construct control points for affine2D
//! @param  movingPoints  output match points of template on reference 
//! @param  fixedPoints   output match points of template on template
//! 
std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f> > 
FisheyeStitcher::createControlPoints( const cv::Point2f &matchLocLeft, 
            const cv::Point2f &matchLocRight, const int row_start, 
            const int row_end, const int p_wid, const int p_x1, 
            const int p_x2, const int p_x2_ref )
{
    std::vector<cv::Point2f> movingPoints; 
    std::vector<cv::Point2f> fixedPoints; 

    float x1 = matchLocLeft.x;
    float y1 = matchLocLeft.y;
    float x2 = matchLocRight.x;
    float y2 = matchLocRight.y;

    //------------------------------------------------------------------------//
    // Construct MovingPoints pRef (matched points of template on reference)  //
    //------------------------------------------------------------------------//
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

    //------------------------------------------------------------------------//
    // Construct fixedPoint pTmpl (matched points of template on template)    //
    //------------------------------------------------------------------------//
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

    return std::make_tuple(movingPoints, fixedPoints);

}   // createControlPoints()


//! 
//! @brief Ramp blending on the right patch
//! 
cv::Mat 
FisheyeStitcher::blendRight( const cv::Mat &bg1, const cv::Mat &bg2 )
{
    int h = bg1.size().height;
    int w = bg1.size().width;
    cv::Mat bg_ = cv::Mat::zeros(bg1.size(), CV_32F);
    double alpha1, alpha2;
    cv::Mat bg1_, bg2_;
    bg1.convertTo(bg1_, CV_32F);
    bg2.convertTo(bg2_, CV_32F);
    //
    cv::Mat bgr_bg[3], bgr_bg1[3], bgr_bg2[3];   //destination array
    split(bg1_, bgr_bg1); //split source  
    split(bg2_, bgr_bg2); //split source  
    // 
    bgr_bg[0] = cv::Mat::zeros(bgr_bg1[1].size(), CV_32F);
    bgr_bg[1] = cv::Mat::zeros(bgr_bg1[1].size(), CV_32F);
    bgr_bg[2] = cv::Mat::zeros(bgr_bg1[1].size(), CV_32F);

    for (int r = 0; r < h; ++r)
    {
        for (int c = 0; c < w; ++c)
        {
            alpha1 = double(c) / double(w);
            alpha2 = 1.0 - alpha1;
            bgr_bg[0].at<float>(r, c) = alpha1*bgr_bg1[0].at<float>(r, c) + 
                                            alpha2*bgr_bg2[0].at<float>(r, c);
            bgr_bg[1].at<float>(r, c) = alpha1*bgr_bg1[1].at<float>(r, c) + 
                                            alpha2*bgr_bg2[1].at<float>(r, c);
            bgr_bg[2].at<float>(r, c) = alpha1*bgr_bg1[2].at<float>(r, c) + 
                                            alpha2*bgr_bg2[2].at<float>(r, c);
        }
    }
    cv::Mat bg;
    cv::merge(bgr_bg, 3, bg);
    bg.convertTo(bg, CV_8U);
    return bg;

}   // blendRight()

//! 
//!  @brief Ramp blending on the left patch
//! 
cv::Mat 
FisheyeStitcher::blendLeft( const cv::Mat &bg1, const cv::Mat &bg2 )
{
    int h = bg1.size().height;
    int w = bg1.size().width;
    cv::Mat bg_ = cv::Mat::zeros(bg1.size(), CV_32F);
    double alpha1, alpha2;
    cv::Mat bg1_, bg2_;
    bg1.convertTo(bg1_, CV_32F);
    bg2.convertTo(bg2_, CV_32F);
    // 
    cv::Mat bgr_bg[3], bgr_bg1[3], bgr_bg2[3]; //destination array
    split(bg1_, bgr_bg1); //split source  
    split(bg2_, bgr_bg2); //split source  
    // 
    bgr_bg[0] = cv::Mat::zeros(bgr_bg1[1].size(), CV_32F);
    bgr_bg[1] = cv::Mat::zeros(bgr_bg1[1].size(), CV_32F);
    bgr_bg[2] = cv::Mat::zeros(bgr_bg1[1].size(), CV_32F);

    for (int r = 0; r < h; ++r)
    {
        for (int c = 0; c < w; ++c)
        {
            alpha1 = (double(w) - c + 1) / double(w);
            alpha2 = 1.0 - alpha1;
            bgr_bg[0].at<float>(r, c) = alpha1*bgr_bg1[0].at<float>(r, c) + 
                                            alpha2*bgr_bg2[0].at<float>(r, c);
            bgr_bg[1].at<float>(r, c) = alpha1*bgr_bg1[1].at<float>(r, c) + 
                                            alpha2*bgr_bg2[1].at<float>(r, c);
            bgr_bg[2].at<float>(r, c) = alpha1*bgr_bg1[2].at<float>(r, c) + 
                                            alpha2*bgr_bg2[2].at<float>(r, c);
        }
    }
    cv::Mat bg;
    cv::merge(bgr_bg, 3, bg);
    bg.convertTo(bg, CV_8U);
    return bg;

}   // blendLeft()


//! 
//! @brief  Blending aligned images
//! @param  left_img  left unwarped image
//! @param  right_img_aligned  aligned right image 
//! @param  return   blended image
//! 
cv::Mat
FisheyeStitcher::blend( const cv::Mat &left_img, 
                        const cv::Mat &right_img_aligned )
{
#if GEAR360_C200
    cv::Mat post;
    // Read YML
    cv::FileStorage fs("./utils/post_find.yml", cv::FileStorage::READ);
    fs["post_ret"] >> post; // 1772 x 1
    fs.release();
    // Mask
    cv::Mat mask = imread("./utils/mask_1920x1920_fovd_187.jpg", cv::IMREAD_COLOR);
#else 
    // use `m_blend_post` instead of `post`
    // use `m_binary_mask` instead of `mask` from file
    cv::Mat mask = m_binary_mask;
#endif
    int H = mask.size().height;
    int W = mask.size().width;

    //-----------------------------------------------------------------------//
    // Prepare 2 blending patches                                            //
    //-----------------------------------------------------------------------//
    // int Worg = 1920;
    int Worg = m_ws;
    int imH = left_img.size().height;
    int imW = left_img.size().width;
    cv::Mat left_img_cr = left_img(cv::Rect(imW / 2 + 1 - Worg / 2, 0, Worg, imH));

    int sideW = 45; // width in pixels
    cv::Mat left_blend, right_blend;

    for (int r = 0; r < H; ++r)
    {
#if GEAR360_C200 
        int p = post.at<float>(r, 0);
#else
        int p = m_blend_post[r];
#endif
        if (p == 0)
        {
            continue;
        }
        // Left boundary
        cv::Mat lf_win_1 = left_img_cr(cv::Rect(p - sideW, r, 2 * sideW, 1));
        cv::Mat rt_win_1 = right_img_aligned(cv::Rect(p - sideW, r, 2 * sideW, 1));
        // Right boundary
        cv::Mat lf_win_2 = left_img_cr(cv::Rect((W - p - sideW), r, 2 * sideW, 1));
        cv::Mat rt_win_2 = right_img_aligned(cv::Rect((W - p - sideW), r, 2 * sideW, 1));
        // Blend(ramp)
        cv::Mat bleft, bright;
        bleft  = blendLeft(lf_win_1, rt_win_1);
        bright = blendRight(lf_win_2, rt_win_2);
        // Update left boundary
        bleft.copyTo(lf_win_1);
        bleft.copyTo(rt_win_1);
        // Update right boundary
        bright.copyTo(lf_win_2);
        bright.copyTo(rt_win_2);
    }

#if MY_DEBUG
    cv::imwrite("left_crop_blend.jpg", left_img_cr);
    cv::imwrite("right_blend.jpg", right_img_aligned);
#endif

    //-----------------------------------------------------------------------//
    // Blending                                                              //
    //-----------------------------------------------------------------------//
    cv::Mat mask_ = mask(cv::Rect(0, 0, mask.size().width, 
                                        mask.size().height - 2));
    cv::Mat mask_n;
    bitwise_not(mask_, mask_n);
    bitwise_and(left_img_cr, mask_, left_img_cr); // Left image
    // 
    cv::Mat temp1 = left_img(cv::Rect(0, 0, (imW / 2 - Worg / 2), imH));
    cv::Mat temp2 = left_img(cv::Rect((imW / 2 + Worg / 2), 0, 
                                      (imW / 2 - Worg / 2), imH));
    cv::Mat t;
    cv::hconcat(temp1, left_img_cr, t);
    cv::hconcat(t, temp2, left_img);
    // 
    bitwise_and(right_img_aligned, mask_n, right_img_aligned); // Right image
    //
    cv::Mat pano;
    pano = left_img;
    cv::Mat temp = pano(cv::Rect((imW / 2 - Worg / 2), 0, Worg, imH));
    cv::Mat t2;
    cv::bitwise_or(temp, right_img_aligned, t2);
    t2.copyTo(temp); // updated pano

    return pano;

}   // blend()


//! 
//! @brief single frame stitching
//! @param  in_img_L  left image
//! @param  in_img_R  right image
//! @param  return    stitched image 
//!
cv::Mat
FisheyeStitcher::stitch(const cv::Mat& in_img_L, const cv::Mat& in_img_R)
{
    // int W_in = 1920;
    int W_in = m_ws; // default: video 3840 x 1920
    cv::Mat left_unwarped, right_unwarped;
    double tickStart, tickEnd, runTime;

#if PROFILING
    tickStart = double(cv::getTickCount());
#endif

    //------------------------------------------------------------------------//
    // Circular Crop                                                          //
    //------------------------------------------------------------------------// 
    cv::bitwise_and(in_img_L, m_cir_mask, in_img_L); // Left image
    cv::bitwise_and(in_img_R, m_cir_mask, in_img_R); // Right image

#if PROFILING
    tickEnd = double(cv::getTickCount());
    runTime = (tickEnd - tickStart) / cv::getTickFrequency();
    tickStart = tickEnd;
    std::cout << "run-time (Crop) = " << runTime << " (sec)" << "\n";
#endif

    //------------------------------------------------------------------------//
    // Light Fall-off Compensation                                            //
    //------------------------------------------------------------------------//
    cv::Mat left_img_compensated(in_img_L.size(), in_img_L.type());
    cv::Mat right_img_compensated(in_img_R.size(), in_img_R.type());
    if (!m_enb_light_compen) // skip LFOC
    {
        left_img_compensated  = in_img_L;
        right_img_compensated = in_img_R;
    }
    else
    {
       left_img_compensated  = compenLightFO(in_img_L);
       right_img_compensated = compenLightFO(in_img_R);
    }

#if PROFILING
    tickEnd = double(cv::getTickCount());
    runTime = (tickEnd - tickStart) / cv::getTickFrequency();
    tickStart = tickEnd;
    std::cout << "run-time (LightCompen) = " << runTime << " (sec)" << "\n";
#endif

    //------------------------------------------------------------------------//
    // Fisheye Unwarping                                                      //
    //------------------------------------------------------------------------//
    left_unwarped  = unwarp(left_img_compensated);
    right_unwarped = unwarp(right_img_compensated);

#if PROFILING
    tickEnd = double(cv::getTickCount());
    runTime = (tickEnd - tickStart) / cv::getTickFrequency();
    tickStart = tickEnd;
    std::cout << "run-time (Unwarp) = " << runTime << " (sec)" << "\n";
#endif

#if MY_DEBUG
    cv::imwrite("l.jpg", left_unwarped);
    cv::imwrite("r.jpg", right_unwarped);
#endif

#if PROFILING
    tickStart = double(cv::getTickCount());
#endif

    //------------------------------------------------------------------------//
    // Rigid Moving Least Squares Deformation                                 //
    //------------------------------------------------------------------------//
    cv::Mat rightImg_crop, rightImg_mls_deformed;
    rightImg_crop = right_unwarped(cv::Rect(int(m_wd / 2) - (W_in / 2), 0, 
                                   W_in, m_hd - 2)); // notice on (Hd-2) --> become: (Hd)
    rightImg_mls_deformed = deform(rightImg_crop);

#if MY_DEBUG
    cv::imwrite("r_img_crop.jpg", rightImg_crop);
    cv::imwrite("r_mls_deformed.jpg",rightImg_mls_deformed);
#endif

#if PROFILING
    tickEnd = double(cv::getTickCount());
    runTime = (tickEnd - tickStart) / cv::getTickFrequency();
    tickStart = tickEnd;
    std::cout << "run-time (MLS Deform) = " << runTime << " (sec)" << "\n";
#endif

    //------------------------------------------------------------------------//
    // Rearrange Image for Adaptive Alignment                                 //
    //------------------------------------------------------------------------//
    cv::Mat temp1 = left_unwarped(cv::Rect(0, 0, m_wd2, m_hd - 2));
    cv::Mat temp2 = left_unwarped(cv::Rect(m_wd2, 0, m_wd2, 
                                  m_hd - 2));
    cv::Mat left_unwarped_arr; // re-arranged left unwarped
    cv::hconcat(temp2, temp1, left_unwarped_arr);
    cv::Mat leftImg_crop;
    leftImg_crop = left_unwarped_arr(cv::Rect(m_wd2 - (W_in / 2), 0, 
                                     W_in, m_hd - 2)); 
    uint16_t crop = static_cast<uint16_t>(0.5f * m_ws * (MAX_FOVD - 180.0) / MAX_FOVD); // half overlap region

    //------------------------------------------------------------------------//
    // PARAMETERS (hard-coded) for C200 videos                                //
    //------------------------------------------------------------------------//
    // (empirical experiment for dual-fisheye image of size 3840 x 1920)
    uint16_t p_wid     = 55;
    uint16_t p_x1      = 90 - 15;
    uint16_t p_x2      = 1780 - 5;
    uint16_t p_x1_ref  = 2 * crop;
    uint16_t row_start = 590;
    uint16_t row_end   = 1320;
    uint16_t p_x2_ref  = m_ws - 2 * crop + 1;
    // 
    cv::Mat Ref_1, Ref_2, Tmpl_1, Tmpl_2;
    Ref_1  = leftImg_crop(cv::Rect(0, row_start, p_x1_ref, row_end - row_start));
    Ref_2  = leftImg_crop(cv::Rect(p_x2_ref, row_start, m_ws - p_x2_ref, row_end - row_start));
    Tmpl_1 = rightImg_mls_deformed(cv::Rect(p_x1, row_start, p_wid, row_end - row_start));
    Tmpl_2 = rightImg_mls_deformed(cv::Rect(p_x2, row_start, p_wid, row_end - row_start));

#if MY_DEBUG
    cv::imwrite("./output/Ref_1.jpg", Ref_1);
    cv::imwrite("./output/Ref_2.jpg", Ref_2);
    cv::imwrite("./output/Tmpl_1.jpg", Tmpl_1);
    cv::imwrite("./output/Tmpl_2.jpg", Tmpl_2);
#endif 

    //------------------------------------------------------------------------//
    // Adaptive Alignment (Norm XCorr)                                        //
    //------------------------------------------------------------------------//
    bool disable_display = 1;   // 1: display off, 0: display on
    std::string wname1 = "Matching On Left Boundary";
    std::string wname2 = "Matching On Right Boundary";

#if PROFILING
    tickStart = double(cv::getTickCount());
#endif

    cv::Mat warpedRightImg;
    if (!m_enb_refine_align) // skip
    {
        warpedRightImg = rightImg_mls_deformed;
    }
    else
    {
        //--------------------------------------------------------------------//
        // Find matching location (normalized XCorr)                          //
        //--------------------------------------------------------------------//
        cv::Point2f matchLocLeft, matchLocRight;
        matchLocLeft  = findMatchLoc(Ref_1, Tmpl_1, wname1, disable_display); // Left boundary
        matchLocRight = findMatchLoc(Ref_2, Tmpl_2, wname2, disable_display); // Right boundary

#if MY_DEBUG
        std::cout << "matchLocLeft(x=" << matchLocLeft.x 
                  << ", y=" << matchLocLeft.y 
                  << "), matchLocRight(x=" << matchLocRight.x 
                  << ", y=" << matchLocRight.y << ")\n";
#endif

        //--------------------------------------------------------------------//
        // Construct control points                                           //
        //--------------------------------------------------------------------//
        std::vector<cv::Point2f> movingPoints;  // matched points in Refs  
        std::vector<cv::Point2f> fixedPoints;   // matched points in Templates

        std::tie(movingPoints, fixedPoints) = 
            createControlPoints(matchLocLeft, matchLocRight, row_start, row_end, 
                                p_wid, p_x1, p_x2, p_x2_ref);

#if PROFILING
        tickEnd = double(cv::getTickCount());
        runTime = (tickEnd - tickStart) / cv::getTickFrequency();
        tickStart = tickEnd;
        std::cout << "run-time (Xcorr & fitGeoTrans) = " << runTime << " (sec)" << "\n";
#endif

        //--------------------------------------------------------------------//
        // Estimate affine matrix                                             //
        //--------------------------------------------------------------------//
        cv::Mat tform_refine_mat;
        tform_refine_mat = cv::findHomography(fixedPoints, movingPoints, 0);

        //--------------------------------------------------------------------//
        // Warp Image                                                         //
        //--------------------------------------------------------------------//
        cv::warpPerspective(rightImg_mls_deformed, warpedRightImg, 
                            tform_refine_mat, rightImg_mls_deformed.size(), 
                            CV_INTER_LINEAR);

#if PROFILING
        tickEnd = double(cv::getTickCount());
        runTime = (tickEnd - tickStart) / cv::getTickFrequency();
        tickStart = tickEnd;
        std::cout << "run-time (estimate tform_mat & warping) = " << runTime << " (sec)" << "\n";
#endif

    } // Normalized xcorr

    //-----------------------------------------------------------------------//
    // Blend Images                                                          //
    //-----------------------------------------------------------------------//
    cv::Mat pano;
    pano = blend(left_unwarped_arr, warpedRightImg);

#if PROFILING
    tickEnd = double(cv::getTickCount());
    runTime = (tickEnd - tickStart) / cv::getTickFrequency();
    tickStart = tickEnd;
    std::cout << "run-time (Blending) = " << runTime << " (sec)" << "\n";
#endif

    return pano;

}   // stitch() 

}   // namespace