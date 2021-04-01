/*** USER-DEFINED PARAMS ***/
#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/ccalib/omnidir.hpp"
#include <opencv2/core/utils/filesystem.hpp>
#include <filesystem>
#include <stdexcept> // To throw errors when checking filepaths
using namespace std;
using namespace cv;
namespace fs = std::filesystem;


const int REG_INIT_SECOND = 0;
const double APERTURE_DISTANCE_PX = 283;
const double OUTPUT_FPS = 10;
const double OUTPUT_RESOLUTION_PX = 512;
const int MEDIAN_BLUR_RADIUS = 23;

const std::string filepath = "/home/echai/Documents/NIFA/Repos/Recon/SRC/Modules/Shadow-Detection";
// File Paths
const std::string CAPTURE_MODE = "live_fisheye";
//const string LIVE_STREAM_PATH = "rtmp://10.1.1.1/live/drone";

//const string LIVE_IMAGES_PATH = filepath + "/calibration/" + CAPTURE_MODE + "/image_";
//const string CALIB_IMAGES_PATH = filepath + "/calibration/" + CAPTURE_MODE + "/";
//const std::string CALIB_IMAGES_LOCATIONS_PATH = filepath + "/calibration/params/" + CAPTURE_MODE + "_paths.xml";

const std::string CAMERA_MODEL_PATH = filepath + "/calib_results.txt";

// Calibration
const Size BOARD_SIZE = Size(7, 5);
const Size FINAL_SIZE = Size(1280, 720);

// Helper function: Bilinear interpolation for image pixels
inline cv::Vec3b getColorSubpixHelper(const cv::Mat& img, cv::Point2d pt)
{
    cv::Mat patch;
    cv::getRectSubPix(img, cv::Size(1, 1), pt, patch);
    return patch.at<cv::Vec3b>(0, 0);
}

