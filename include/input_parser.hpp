//----------------------------------------------------------------------------//
//                                                                            //
// This file is part of the fisheye stitcher project.                         //
// Copyright (c) 2018-2020 Tuan Phan Minh Ho <drnoob2013@gmail.com>           //
// https://github.com/drNoob13/fisheyeStitcher                                //
//                                                                            //
//----------------------------------------------------------------------------//
#ifndef INPUT_PARSER_HPP
#define INPUT_PARSER_HPP

#include <iostream>
#include <opencv2/core/core.hpp>
#include <variant>    // std::variant

namespace ffstitcher 
{
enum class Options { 
    OPT_IMAGE_PATH       = 0,
    OPT_IMAGE_NAME       = 1,
    OPT_OUTPUT_DIR       = 2,
    OPT_VIDEO_PATH       = 3,
    OPT_MLSMAP_PATH      = 4,
    OPT_MODE             = 5,
    OPT_IMAGE_SIZE       = 6,
    OPT_ENB_LIGHT_COMPEN = 7,
    OPT_ENB_REFINE_ALIGN = 8
};

class InputParser
{
public: 

    InputParser(int argc, char **argv) 
      : m_img_path(std::string("nofile")), m_out_dir(std::string(" ")),
        m_enb_lc(false), m_enb_ra(false),
        m_mode(std::string("image")),
        m_video_path("nofile"),
        m_mls_map_path("nofile")
    {
        if ((argc < 2) ||
            (std::string(argv[1]) == "--help") || 
            (std::string(argv[1]) == "--h"   ) ||
            (std::string(argv[1]) == "-help" ) || 
            (std::string(argv[1]) == "-h"    ))
        {
            print_help();
            std::cout << "\nexit\n";
            exit(EXIT_FAILURE);
        }

        for (int i = 1; i < argc; ++i)
        {
            if (std::string(argv[i]) == "--img_path")
            {
                m_img_path = argv[i + 1];
                i++;
            }
            else if (std::string(argv[i]) == "--out_dir")
            {
                m_out_dir = argv[i + 1];
                i++;
            }
            else if (std::string(argv[i]) == "--img_nm")
            {
                m_img_nm = argv[i + 1];
                i++;
            }
            else if (std::string(argv[i]) == "--enb_light_compen")
            {
                m_enb_lc = static_cast<bool>(atoi(argv[i + 1]));
            }
            else if (std::string(argv[i]) == "--enb_refine_align")
            {
                m_enb_ra = static_cast<bool>(atoi(argv[i + 1]));
            }
            else if (std::string(argv[i]) == "--video_path")
            {
                m_video_path = argv[i + 1];
                i++;
            }
            else if (std::string(argv[i]) == "--mls_map_path")
            {
                m_mls_map_path = argv[i + 1];
                i++;
            }
            else if (std::string(argv[i]) == "--mode")
            {
                m_mode = argv[i + 1];
                i++;
            }
        }
        print_input();
        check_input();
    }   // InputParser


    void check_input()
    {
        if (m_mode == "image")
        {
            if (m_img_path == "nofile")
            {
                CV_Error_(cv::Error::StsBadArg, 
                    ("FFStitcher: Please specify image path. %s", 
                      m_helper.c_str()));
            }
        }
        else if (m_mode == "video")
        {
            if (m_video_path == "nofile")
            {
                CV_Error_(cv::Error::StsBadArg, 
                    ("FFStitcher: Please specifiy video path. %s", 
                      m_helper.c_str()));
            }
        }
        else
        {
            CV_Error_(cv::Error::StsBadArg, 
                ("FFStitcher: Cannot recognize input mode. %s", 
                  m_helper.c_str()));
        }

        if (m_mls_map_path == "nofile")
        {
            CV_Error_(cv::Error::StsBadArg, 
                ("FFStitcher: Cannot find MLS grids. %s", 
                  m_helper.c_str()));
        }

        if (m_img_nm == "empty")
        {
            CV_Error_(cv::Error::StsBadArg, 
                ("FFStitcher: Image name cannot be empty. %s", 
                  m_helper.c_str()));
        }

        if (m_out_dir == " ")
        {
            CV_Error_(cv::Error::StsBadArg, 
                ("FFStitcher: Please specify output directory. %s", 
                  m_helper.c_str()));
        } 
    }


    void print_input()
    {
        std::cout << "\n------------------------ Input Arguments --------------"
             << "--------------\n"
             << "img_nm             : " << m_img_nm        << "\n"
             << "out_dir            : " << m_out_dir       << "\n"
             << "video_path         : " << m_video_path    << "\n"
             << "mls_map_path       : " << m_mls_map_path  << "\n"
             << "\n";
    }


    void print_help()
    {
        std::cout << "\n------------------------------ Help -------------------"
           << "----------- \n"
           << "ffStitcher_video                                           \\ \n"
           << "  --input mode      video                                  \\ \n"
           << "  --img_nm          <your_filename>                        \\ \n"
           << "  --out_dir         ../output/                             \\ \n"
           << "  --video_path      <your_path>/in_video.mp4               \\ \n"
           << "  --mls_map_path    ../utils/grid_xd_yd_3840x1920.yml.gz   \\ \n"
           << "\n";
    }

    // A getter function that returns data type for each input type
    std::variant<bool, std::string> get(Options Opt) 
    {
        switch(Opt)
        {
        case Options::OPT_IMAGE_PATH       : return m_img_path;
        case Options::OPT_IMAGE_NAME       : return m_img_nm;
        case Options::OPT_OUTPUT_DIR       : return m_out_dir;
        case Options::OPT_VIDEO_PATH       : return m_video_path;
        case Options::OPT_MLSMAP_PATH      : return m_mls_map_path;
        case Options::OPT_MODE             : return m_mode;
        case Options::OPT_ENB_LIGHT_COMPEN : return m_enb_lc;
        case Options::OPT_ENB_REFINE_ALIGN : return m_enb_ra;
        default: ;
        }
    }

private:
    bool          m_use_store_map;
    bool          m_enb_lc;  // enb light compen (0: off)
    bool          m_enb_ra;  // enb refine alignment (0: off)
    std::string   m_img_nm;
    std::string   m_out_dir;
    std::string   m_img_path;
    std::string   m_video_path;
    std::string   m_mls_map_path;
    std::string   m_mode;
    const std::string m_helper{"| Hint: Run with no arg or --h for help |"};
};  // InputParser

}   // ffstitcher 

#endif  // INPUT_PARSER_HPP