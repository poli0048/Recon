/*** USER-DEFINED PARAMS ***/
#pragma once

//System Includes
#include <stdexcept> // To throw errors when checking filepaths

//External Includes
#include "../../../handycpp/Handy.hpp" //Provides std::filesystem for old and new compilers/standards
#include <opencv2/opencv.hpp>

const int    REG_INIT_SECOND = 0;
const double APERTURE_DISTANCE_PX = 283.0;
const double OUTPUT_FPS = 10.0;
const double OUTPUT_RESOLUTION_PX = 512.0;
const int    MEDIAN_BLUR_RADIUS = 23;

const std::string CAMERA_MODEL_PATH = (Handy::Paths::ThisExecutableDirectory() / "FisheyCamModel.txt").string();

// Calibration
const cv::Size BOARD_SIZE = cv::Size(7, 5);
const cv::Size FINAL_SIZE = cv::Size(1280, 720);

// Helper function: Bilinear interpolation for image pixels
inline uint8_t getColorSubpixHelper_UC1(cv::Mat const & Img, cv::Point2d Pt) {
    cv::Mat patch;
    cv::getRectSubPix(Img, cv::Size(1, 1), Pt, patch);
    return patch.at<uint8_t>(0, 0);
}

inline cv::Vec3b getColorSubpixHelper_UC3(cv::Mat const & Img, cv::Point2d Pt) {
    cv::Mat patch;
    cv::getRectSubPix(Img, cv::Size(1, 1), Pt, patch);
    return patch.at<cv::Vec3b>(0, 0);
}

