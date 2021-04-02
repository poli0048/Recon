#pragma once

//External Includes
#include <opencv2/features2d.hpp>
#include "FRF.h"

inline int minHessian = 400;

// Input ref_color, output ref_descriptors, keypoints_ref

inline void getRefDescriptors(const cv::Mat& ref_color, cv::Mat& descriptors_ref, std::vector<cv::KeyPoint>& keypoints_ref){
	cv::Mat img_ref;

	cv::cvtColor(ref_color, img_ref, cv::COLOR_BGR2GRAY);

	cv::Ptr<cv::ORB> detector = cv::ORB::create();

	detector->detectAndCompute(img_ref, cv::noArray(), keypoints_ref, descriptors_ref); //remove. save descriptors_ref
}


inline void getOrbRotation(const cv::Mat& descriptors_ref, const std::vector<cv::KeyPoint>& keypoints_ref, const cv::Mat& rot_color, cv::Mat& H) {
	cv::Mat img_rot;
	std::vector<cv::KeyPoint> keypoints_rot;
	cv::Mat descriptors_rot;
	std::vector<cv::DMatch> good_matches;
	std::vector<cv::Point2f> ref;
	std::vector<cv::Point2f> rot;

	cv::cvtColor(rot_color, img_rot, cv::COLOR_BGR2GRAY);
	cv::Ptr<cv::ORB> detector = cv::ORB::create();
	
	detector->detectAndCompute(img_rot, cv::noArray(), keypoints_rot, descriptors_rot);


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

	cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
	split(hsv, hsv_channels);
	v = hsv_channels[2];

	// Running maximum brightness for each pixel
	max(bright, v, bright);

	// Difference between brightest observed and current
	subtract(bright, v, diff);

	// Anything with too small of a difference is floored to zero (Avoids segmentation when there is no segmentation to be done). 
	threshold(diff, diff, 20, 255, cv::THRESH_TOZERO);

	// Dynamic thresholding to binary by minimizing within-class variance 
	threshold(diff, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
}

inline void getApertureMask(const cv::Mat& img, cv::Mat& mask) {
	cv::Mat img_thresh, img_binary, img_filled, img_filled_inv, img_final;

	cvtColor(img, img_binary, cv::COLOR_RGB2GRAY);
	threshold(img_binary, img_thresh, 30, 255, cv::THRESH_BINARY);
	img_thresh.copyTo(img_filled);
	
	floodFill(img_filled, cv::Point(0, 0), cv::Scalar(255));
	floodFill(img_filled, cv::Point(0, OUTPUT_RESOLUTION_PX - 1), cv::Scalar(255));
	floodFill(img_filled, cv::Point(OUTPUT_RESOLUTION_PX - 1, 0), cv::Scalar(255));
	floodFill(img_filled, cv::Point(OUTPUT_RESOLUTION_PX - 1, OUTPUT_RESOLUTION_PX - 1), cv::Scalar(255));
	
	bitwise_not(img_filled, img_filled_inv);
	mask = img_thresh | img_filled_inv;

	floodFill(mask, cv::Point(0, 0), cv::Scalar(0));
	floodFill(mask, cv::Point(0, OUTPUT_RESOLUTION_PX - 1), cv::Scalar(0));
	floodFill(mask, cv::Point(OUTPUT_RESOLUTION_PX - 1, 0), cv::Scalar(0));
	floodFill(mask, cv::Point(OUTPUT_RESOLUTION_PX - 1, OUTPUT_RESOLUTION_PX - 1), cv::Scalar(0));
}

inline void create_masked_binary(const int mapRows, const int mapCols, const cv::Mat mask, cv::Mat binary_masked, cv::Mat binary_sampled){

    for (uint32_t row = 0U; row < (uint32_t) mapRows; row++) {
        for (uint32_t col = 0U; col < (uint32_t) mapCols; col++) {
            if ((int) mask.at<uchar>(row, col) == 0) {
                binary_masked.at<uchar>(row,col) = NAN; //TODO: This is a problem - there is no NAN for uchar types
                        //newLayer->SetValue(row, col, (int)binary_sampled.at<uchar>(row, col));
            } 
        }
    }
}

inline void set_NewValue(const int mapRows, const int mapCols, FRFLayer* newLayer, cv::Mat binary_masked){
    for (uint32_t row = 0U; row < (uint32_t) mapRows; row++) {
        for (uint32_t col = 0U; col < (uint32_t) mapCols; col++) {
		newLayer->SetValue(row, col, (int) binary_masked.at<uchar>(row, col));
        }
    }
}
