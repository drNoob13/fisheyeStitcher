/**
 * Tuan Ho
 * Function: Input parser 
 * 
*/
#ifndef INPUT_PARSER_HPP
#define INPUT_PARSER_HPP

#include <iostream>
#include <opencv2/core/core.hpp>

namespace ffstitcher 
{

class InputParser
{
public:
    InputParser(int argc, char **argv) 
      : m_img_path(std::string("nofile")), m_out_dir(std::string(" ")),
        m_corr_file(std::string("empty")), m_img_nm(std::string("empty")),
        m_ccalib_ed_l(std::string("nofile")),
        m_ccalib_ed_r(std::string("nofile")),
        m_use_store_map(0), m_mode(std::string("image")),
        m_ed_map_l(std::string("empty")), m_ed_map_r(std::string("empty")),
        m_image_width(0), m_image_height(0), m_video_path("nofile")
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
                m_enb_light_compen = static_cast<bool>(atoi(argv[i + 1]));
            }
            else if (std::string(argv[i]) == "--enb_refine_align")
            {
                m_enb_refine_align = static_cast<bool>(atoi(argv[i + 1]));
            }
            else if (std::string(argv[i]) == "--ccalib_ed_l")
            {
                m_ccalib_ed_l = argv[i + 1];
                i++;
            }
            else if (std::string(argv[i]) == "--ccalib_ed_r")
            {
                m_ccalib_ed_r = argv[i + 1];
                i++;
            }
            else if (std::string(argv[i]) == "--ed_map_l")
            {
                m_ed_map_l = argv[i + 1];
                i++;
            }
            else if (std::string(argv[i]) == "--ed_map_r")
            {
                m_ed_map_r = argv[i + 1];
                i++;
            }
            else if (std::string(argv[i]) == "--use_store_map")
            {
                m_use_store_map = static_cast<bool>(atoi(argv[i + 1]));
            }
            else if (std::string(argv[i]) == "--image_width")
            {
                m_image_width = (atoi(argv[i + 1]));
                i++;
            }
            else if (std::string(argv[i]) == "--image_height")
            {
                m_image_height = (atoi(argv[i + 1]));
                i++;
            }
            else if (std::string(argv[i]) == "--video_path")
            {
                m_video_path = argv[i + 1];
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

        if ((m_image_width == 0) || (m_image_height == 0) ||
            (m_image_width % 2 != 0) || (m_image_height % 2 != 0))
        {
            CV_Error_(cv::Error::StsBadSize,
                ("FFStitcher: Image size (width or height) has to be a non-zero\
                  multiple of two. Input: [%d x %d] %s", 
                  m_image_width, m_image_height, m_helper.c_str()));
        }
    }


    void print_input()
    {
        std::cout << "\n------------------------ Input Arguments --------------"
             << "--------------\n"
             << "input mode         : " << m_mode     << "\n"
             << "use_store_map      : " << m_use_store_map << "\n"
             << "img_nm             : " << m_img_nm        << "\n"
             << "out_dir            : " << m_out_dir       << "\n"
             << "ed_map_l           : " << m_ed_map_l      << "\n"
             << "ed_map_r           : " << m_ed_map_r      << "\n"
             << "image_width        : " << m_image_width   << "\n"
             << "image_height       : " << m_image_height  << "\n"
             << "video_path         : " << m_video_path    << "\n"
             << "img_path           : " << m_img_path      << "\n"
             << "ccalib_ed_l        : " << m_ccalib_ed_l   << "\n"
             << "ccalib_ed_r        : " << m_ccalib_ed_r   << "\n"
             << "\n";
    }


    void print_help()
    {
        std::cout << "\n------------------------------ Help -------------------"
           << "----------- \n"
           << "ffStitcher_video                                           \\ \n"
           << "  --input mode      video | default:image                  \\ \n"
           << "  --use_store_map   1                                      \\ \n"
           << "  --img_nm          <your_filename>                        \\ \n"
           << "  --out_dir         ../output/                             \\ \n"
           << "  --ed_map_l        ../data/ed_map_left.yaml.gz            \\ \n"
           << "  --ed_map_r        ../data/ed_map_right.yaml.gz           \\ \n"
           << "  --image_width     1920 <single fisheye width>            \\ \n"
           << "  --image_height    1920 <single fisheye height>           \\ \n"
           << "  --video_path      <your_path>/in_video.mp4               \\ \n"
           << "  --img_path        <your_path>/Mall_Nhung_0059.png        \\ \n"
           << "  --ccalib_ed_l     ../data/<calib_param_left_camera>.yaml \\ \n"
           << "  --ccalib_ed_r     ../data/<calib_param_rght_camera>.yaml    \n"
           << "\n";
    }

    std::string   getImagePath()         const { return m_img_path;            }
    std::string   getImageName()         const { return m_img_nm;              }
    std::string   getOutDir()            const { return m_out_dir;             }
    std::string   getEdMapFileL()        const { return m_ed_map_l;            }
    std::string   getEdMapFileR()        const { return m_ed_map_r;            }
    std::string   getVideoPath()         const { return m_video_path;          }
    std::string   getEdCalibParamFileL() const { return m_ccalib_ed_l;         }
    std::string   getEdCalibParamFileR() const { return m_ccalib_ed_r;         }
    std::string   getMode()              const { return m_mode;                }     
    bool          getUseStoredMap()      const { return m_use_store_map;       }
    cv::Size      getImageSize()         const { return cv::Size(m_image_width,
                                                           m_image_height);    }
    bool          getFlagLightCompen()   const { return m_enb_light_compen;    }                                                       
    bool          getFlagRefineAlign()   const { return m_enb_refine_align;    }


private:
    bool          m_use_store_map;
    bool          m_enb_light_compen;
    bool          m_enb_refine_align;
    int           m_image_width;
    int           m_image_height;
    std::string   m_img_nm;
    std::string   m_out_dir;
    std::string   m_corr_file;
    std::string   m_img_path;
    std::string   m_ccalib_ed_l;
    std::string   m_ccalib_ed_r;
    std::string   m_ed_map_l;
    std::string   m_ed_map_r;
    std::string   m_video_path;
    std::string   m_mode;
    const std::string m_helper{"| Hint: Run with no arg or --h for help |"};
};  // InputParser

}   // ffstitcher 

#endif  // INPUT_PARSER_HPP
