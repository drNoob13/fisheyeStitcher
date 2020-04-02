//----------------------------------------------------------------------------//
//                                                                            //
// This file is part of the fisheye stitcher project.                         //
// Copyright (c) 2018-2020 Tuan Phan Minh Ho <drnoob2013@gmail.com>           //
// https://github.com/drNoob13/fisheyeStitcher                                //
//                                                                            //
//----------------------------------------------------------------------------//
#include "fisheye_stitcher.hpp"
#include "input_parser.hpp"

int
main(int argc, char **argv)
{
    // Parse input 
    ffstitcher::InputParser Parser(argc, argv);

    int Ws = Parser.getImageSize().width;
    int Hs = Parser.getImageSize().height;

    // Video input 
    cv::VideoCapture VCap( Parser.getVideoPath() );
    if( !VCap.isOpened() )
    {
        CV_Error_(cv::Error::StsBadArg, ("Error opening video: %s",
                    Parser.getVideoPath().c_str()));
    } 

    // Video output 
    // OpenCV 3.4.2
    // int    frame_fps      = VCap.get(CV_CAP_PROP_FPS);
    // int    frame_width    = VCap.get(CV_CAP_PROP_FRAME_WIDTH);
    // int    frame_height   = VCap.get(CV_CAP_PROP_FRAME_HEIGHT);
    // OpenCV 4.1.0
    int    frame_fps      = VCap.get(CV_CAP_PROP_FPS);
    int    frame_width    = VCap.get(CV_CAP_PROP_FRAME_WIDTH);
    int    frame_height   = VCap.get(CV_CAP_PROP_FRAME_HEIGHT);
    std::string video_out_name = Parser.getOutDir() + "/" + 
                        Parser.getImageName() + "_blend_video.avi";

    cv::VideoWriter VOut;
    VOut.open( video_out_name, cv::VideoWriter::fourcc('X','2','6','4'), // OpenCV 4.1.0
               frame_fps, cv::Size(frame_width, frame_height) );
    if( !VOut.isOpened() )
    {
        CV_Error_(cv::Error::StsBadArg, 
                  ("Error opening video: %s", video_out_name.c_str()));
    } 
    std::cout << "Writing to video [" 
         << frame_width << "x" << frame_height << "] @" 
         << frame_fps << "fps  --> " << video_out_name << "\n";

    // CV_Assert( frame_width/2 == Ws && frame_height  == Hs );

    // Dual-fisheye stitcher
    stitcher::FisheyeStitcher Stitcher(
        Ws, Hs,                        // image size
        195.0f,                        // tonemapping method
        Parser.getFlagLightCompen(),
        Parser.getFlagRefineAlign()
    );

    // Main video loop
    int count = 0;
    cv::Mat img, frame;
    while( 1 )
    { 
        VCap >> frame;

        if( frame.empty())
        {
            std::cout << "end of video\n";
            break;
        }
        frame.copyTo(img);

        // Testing
        // if( count == 430 ) break;
        // if( count == 390 ) break;
        // if( count == 40 ) break;
        // if( count == 10 ) break;
 
        cv::Mat img_l, img_r;
        img_l = img(cv::Rect(0,  0, Ws, Hs)); // left fisheye
        img_r = img(cv::Rect(Ws, 0, Ws, Hs)); // right fisheye 

        std::cout << "Stitch frame: " << count << "\n";

        // Stitch video frames
        cv::Mat pano;
        pano = Stitcher.stitch(img_l, img_r);

        VOut << pano;

        count++;

    }   // while(1)

    VCap.release();
    VOut.release();
    std::cout << "Finished! Video write to: " << video_out_name << "\n";
    std::cout << "\nSee you again.\n";

    return 0;
}