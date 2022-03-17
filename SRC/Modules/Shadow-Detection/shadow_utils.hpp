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

//Note: This function is misleadingly named. estimateAffinePartial2D() estimates a 4DOF affine transformation that allows rotation, translation,
//and uniform scaling. This is likily too general for image stabilization... rotation and translation only would be better.
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
	
	//This next function appears to raise an exception sometimes. This is probably because there aren't enough matches to run the algorithm.
	//It looks like we just put as many items in these vectors as we have ORB keypoint matches that pass our inclusion test. There is no guarentee that there
	//will be at least 2 of them. This is especially likily if we get a corrupted frame from the drone... there may be no matches.
	try {
		H = cv::estimateAffinePartial2D(rot, ref);
	}
	catch (...) {
		std::cerr << "cv::estimateAffinePartial2D failed. Defaulting stabilization homography.\r\n";
		H = cv::Mat::eye(2, 3, CV_64F);
	}
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
// Any value set to 255 is set to 254
// Reserve 255 as the sentinel value for NAN
// Value is supposed to be CV_8UC3 but only the first value is used
inline void create_masked_binary(const int mapRows, const int mapCols, const cv::Mat mask, cv::Mat binary_sampled){

    for (uint32_t row = 0U; row < (uint32_t) mapRows; row++) {
        for (uint32_t col = 0U; col < (uint32_t) mapCols; col++) {
	    // Set any pixel with value of 254 to 255
	    if (binary_sampled.at<uchar>(row,col) == 255){
		//std::cout << "Settting value at " << row << "," << col << " to 254" << std::endl;
		binary_sampled.at<uchar>(row,col) = 254;
	    }
	    // Set the sentinel pixel
            if ((int) mask.at<uchar>(row, col) == 0) {
		//std::cout << "Settting value at " << row << "," << col << " to sentinel for NAN" << std::endl;
                binary_sampled.at<uchar>(row,col) = 255;
            } 
        }
    }
}

inline void set_NewValue(const int mapRows, const int mapCols, FRFLayer* newLayer, cv::Mat binary_sampled){
    for (uint32_t row = 0U; row < (uint32_t) mapRows; row++) {
        for (uint32_t col = 0U; col < (uint32_t) mapCols; col++) {
		if (binary_sampled.at<uchar>(row,col) == 255){
			newLayer->SetValue(row, col, NAN);
		}
		else{
        		newLayer->SetValue(row, col, (int) binary_sampled.at<uchar>(row, col));
		}
        }
    }
}
