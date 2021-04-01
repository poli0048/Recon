#pragma once

#include "opencv2/xfeatures2d.hpp"
#include "FRF.h"
using namespace cv::xfeatures2d;
inline int minHessian = 400;

// Input ref_color, output ref_descriptors, keypoints_ref

inline void getRefDescriptors(const Mat& ref_color, Mat& descriptors_ref, vector<KeyPoint>& keypoints_ref){
	cv::Mat img_ref;

	cv::cvtColor(ref_color, img_ref, COLOR_BGR2GRAY);

	cv::Ptr<cv::ORB> detector = cv::ORB::create();

	detector->detectAndCompute(img_ref, noArray(), keypoints_ref, descriptors_ref); //remove. save descriptors_ref
}


inline void getOrbRotation(const cv::Mat& descriptors_ref, const std::vector<KeyPoint>& keypoints_ref, const cv::Mat& rot_color, cv::Mat& H) {
	cv::Mat img_rot;
	std::vector<KeyPoint> keypoints_rot;
	cv::Mat descriptors_rot;
	std::vector<DMatch> good_matches;
	std::vector<Point2f> ref;
	std::vector<Point2f> rot;

	cv::cvtColor(rot_color, img_rot, COLOR_BGR2GRAY);
	cv::Ptr<cv::ORB> detector = cv::ORB::create();
	
	detector->detectAndCompute(img_rot, noArray(), keypoints_rot, descriptors_rot);


	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
	std::vector< std::vector<cv::DMatch> > knn_matches;
	matcher->knnMatch(descriptors_ref, descriptors_rot, knn_matches, 2);

	const float ratio_thresh = 0.75f;
	
	for (size_t i = 0; i < knn_matches.size(); i++)
	{
		if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
		{
			ref.push_back(keypoints_ref[knn_matches[i][0].queryIdx].pt);
			rot.push_back(keypoints_rot[knn_matches[i][0].trainIdx].pt);
			//good_matches.push_back(knn_matches[i][0]);
		}
	}

	H = cv::estimateAffinePartial2D(rot, ref);
}

inline void getBinaryCloudMask(const cv::Mat& img, cv::Mat& bright, cv::Mat& binary) {
	cv::Mat hsv, v, diff, very_dark;
	std::vector<cv::Mat> hsv_channels;

	cv::cvtColor(img, hsv, COLOR_BGR2HSV);
	split(hsv, hsv_channels);
	v = hsv_channels[2];

	// Running maximum brightness for each pixel
	max(bright, v, bright);

	// Difference between brightest observed and current
	subtract(bright, v, diff);

	// Anything with too small of a difference is floored to zero (Avoids segmentation when there is no segmentation to be done). 
	threshold(diff, diff, 20, 255, THRESH_TOZERO);

	// Dynamic thresholding to binary by minimizing within-class variance 
	threshold(diff, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
}

inline void getApertureMask(const Mat& img, Mat& mask) {
	Mat img_thresh, img_binary, img_filled, img_filled_inv, img_final;

	cvtColor(img, img_binary, COLOR_RGB2GRAY);
	threshold(img_binary, img_thresh, 30, 255, THRESH_BINARY);
	img_thresh.copyTo(img_filled);
	
	floodFill(img_filled, Point(0, 0), Scalar(255));
	floodFill(img_filled, Point(0, OUTPUT_RESOLUTION_PX - 1), Scalar(255));
	floodFill(img_filled, Point(OUTPUT_RESOLUTION_PX - 1, 0), Scalar(255));
	floodFill(img_filled, Point(OUTPUT_RESOLUTION_PX - 1, OUTPUT_RESOLUTION_PX - 1), Scalar(255));
	
	bitwise_not(img_filled, img_filled_inv);
	mask = img_thresh | img_filled_inv;

	floodFill(mask, Point(0, 0), Scalar(0));
	floodFill(mask, Point(0, OUTPUT_RESOLUTION_PX - 1), Scalar(0));
	floodFill(mask, Point(OUTPUT_RESOLUTION_PX - 1, 0), Scalar(0));
	floodFill(mask, Point(OUTPUT_RESOLUTION_PX - 1, OUTPUT_RESOLUTION_PX - 1), Scalar(0));
}

inline void create_masked_binary(const int mapRows, const int mapCols, const Mat mask, Mat binary_masked, Mat binary_sampled){

    for (uint32_t row = 0U; row < mapRows; row++) {
        for (uint32_t col = 0U; col < mapCols; col++) {
            if ((int) mask.at<uchar>(row, col) == 0) {
                binary_masked.at<uchar>(row,col) = NAN;
                        //newLayer->SetValue(row, col, (int)binary_sampled.at<uchar>(row, col));
            } 
        }
    }
}

inline void set_NewValue(const int mapRows, const int mapCols, FRFLayer* newLayer, Mat binary_masked){
    for (uint32_t row = 0U; row < mapRows; row++) {
        for (uint32_t col = 0U; col < mapCols; col++) {
		newLayer->SetValue(row, col, (int) binary_masked.at<uchar>(row, col));
        }
    }
}
