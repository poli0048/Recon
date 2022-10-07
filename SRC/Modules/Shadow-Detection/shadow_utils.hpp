#pragma once

//External Includes
#include <opencv2/features2d.hpp>
#include "FRF.h"

//inline int minHessian = 400;

//Forward Declarations
inline void getApertureMask(const cv::Mat & img, cv::Mat & mask);

// Input ref_color, output ref_descriptors, keypoints_ref

/*inline void getRefDescriptors(const cv::Mat& ref_color, cv::Mat& descriptors_ref, std::vector<cv::KeyPoint>& keypoints_ref){
	cv::Mat img_ref;

	cv::cvtColor(ref_color, img_ref, cv::COLOR_BGR2GRAY);

	cv::Ptr<cv::ORB> detector = cv::ORB::create();

	detector->detectAndCompute(img_ref, cv::noArray(), keypoints_ref, descriptors_ref); //remove. save descriptors_ref
}*/

//Alternative to cv::estimateAffinePartial2D but which only computes rotation and translation, but not scale (locked at 1).
//Computes the affine transformation H that best maps points from the A frame to the B frame using only rotation and translation.
//Computes a 2x3 matrix H.
inline cv::Mat EstimateRTTransformation(std::vector<cv::Point2f> const & PointsA, std::vector<cv::Point2f> const & PointsB) {
	double PI = 3.14159265358979;

	//Apply a transformation to PointsA and use that for PointsB for testing - need to make PointsB a non-const & arg for this test
	/*{
		std::vector<cv::Point2f> PointsBPrime(PointsA.size());
		cv::Mat HTrue = cv::Mat::eye(2, 3, CV_64F);
		double theta = 40*PI/180.0;
		double tx = 35.0;
		double ty = -17.0;
		HTrue.at<double>(0,0) = cos(theta); HTrue.at<double>(0,1) = -1.0*sin(theta); HTrue.at<double>(0,2) = tx;
		HTrue.at<double>(1,0) = sin(theta); HTrue.at<double>(1,1) =      cos(theta); HTrue.at<double>(1,2) = ty;
		cv::transform(PointsA, PointsBPrime, HTrue);
		PointsB.swap(PointsBPrime);
	}*/

	if ((PointsA.size() < 2U) || (PointsA.size() != PointsB.size()))
		return cv::Mat::eye(2, 3, CV_64F); //Default to identity

	//Grab pairs of matches that aren't especially close to one another in either frame. If the points are too close
	//then the rotation component becomes less observable. From each pair, estimate the rotation and translation between the two frames.
	//This gives rise to a candidate homography
	cv::Point2f centerA(0.0, 0.0);
	cv::Point2f centerB(0.0, 0.0);
	for (size_t n = 0U; n < PointsA.size(); n++) {
		centerA += PointsA[n];
		centerB += PointsB[n];
	}
	centerA = centerA / double(PointsA.size());
	centerB = centerB / double(PointsB.size());

	double meanDistFromCenterA = 0.0;
	double meanDistFromCenterB = 0.0;
	for (size_t n = 0U; n < PointsA.size(); n++) {
		meanDistFromCenterA += cv::norm(PointsA[n] - centerA);
		meanDistFromCenterB += cv::norm(PointsB[n] - centerB);	
	}
	meanDistFromCenterA /= double(PointsA.size());
	meanDistFromCenterB /= double(PointsB.size());

	int numCandidates = 1000;
	std::vector<double> thetaCandidates;
	thetaCandidates.reserve(numCandidates);
	cv::RNG rng;
	for (int n = 0; n < numCandidates; n++) {
		int index1 = rng(PointsA.size());
		int index2 = rng(PointsA.size());
		if (index1 == index2)
			continue;

		cv::Point2f const & point1A(PointsA[index1]);
		cv::Point2f const & point1B(PointsB[index1]);

		cv::Point2f const & point2A(PointsA[index2]);
		cv::Point2f const & point2B(PointsB[index2]);		

		double distA = cv::norm(point2A - point1A);
		double distB = cv::norm(point2B - point1B);

		if ((distA < 0.5*meanDistFromCenterA) || (distB < 0.5*meanDistFromCenterB))
			continue;

		//Use this pair of matches to estimate rotation
		Eigen::Vector2d V_A(point2A.x - point1A.x, point2A.y - point1A.y);
		Eigen::Vector2d V_B(point2B.x - point1B.x, point2B.y - point1B.y);
		V_A.normalize();
		V_B.normalize();
		double dot = V_A.dot(V_B);
		double det = V_A(0)*V_B(1) - V_A(1)*V_B(0);
		double theta = std::atan2(det, dot);

		if (theta > 45.0*PI/180.0)
			continue;

		thetaCandidates.push_back(theta);
	}
	double theta = getMedian(thetaCandidates, true);

	//Now estimate the translation component
	double cosTheta = cos(theta);
	double sinTheta = sin(theta);
	std::vector<double> txCandidates; txCandidates.reserve(PointsA.size());
	std::vector<double> tyCandidates; tyCandidates.reserve(PointsA.size());
	for (size_t n = 0U; n < PointsA.size(); n++) {
		cv::Point2f const & pointA(PointsA[n]);
		cv::Point2f const & pointB(PointsB[n]);

		double tx = pointB.x - pointA.x*cosTheta + pointA.y*sinTheta;
		double ty = pointB.y - pointA.x*sinTheta - pointA.y*cosTheta;

		if ((std::abs(tx) <= 200.0) && (std::abs(ty) <= 200.0)) {
			txCandidates.push_back(tx);
			tyCandidates.push_back(ty);
		}
	}
	double tx = getMedian(txCandidates, true);
	double ty = getMedian(tyCandidates, true);

	//std::cerr << "theta: " << theta * 180.0/PI << " deg. tx: " << tx << ". ty: " << ty << ".\r\n";

	//Assemble the full transformation matrix
	cv::Mat H(2, 3, CV_64F);
	H.at<double>(0,0) = cosTheta; H.at<double>(0,1) = -1.0*sinTheta; H.at<double>(0,2) = tx;
	H.at<double>(1,0) = sinTheta; H.at<double>(1,1) =      cosTheta; H.at<double>(1,2) = ty;
	return H;
}

//Custom implementation of equalizeHist that supports a mask
//Any pixel for which mask is 0 is left untouched (and won't impact the histogram)
//It is ok if Src and Dst reference the same image
inline void EqualizeHistWithMask(cv::Mat const & Src, cv::Mat & Dst, cv::Mat const & Mask) {
	if ((Src.rows == 0) || (Src.cols == 0))
		return;
	if ((Src.channels() != 1) || (Mask.channels() != 1)) {
		std::cerr << "Warning: EqualizeHistWithMask() only supports grayscale imagery. Skipping.\r\n";
		return;
	}
	if ((Src.depth() != CV_8U) || (Mask.depth() != CV_8U)) {
		std::cerr << "Warning: EqualizeHistWithMask() only supports uint8 imagery. Skipping.\r\n";
		return;
	}
	if ((Mask.rows != Src.rows) || (Mask.cols != Src.cols)) {
		std::cerr << "Warning: EqualizeHistWithMask() Mask is not the same size as Src Mat. Skipping.\r\n";
		return;
	}

	if ((Dst.rows != Src.rows) || (Dst.cols != Src.cols) || (Dst.channels() != 1) || (Dst.depth() != CV_8U)) {
		//std::cerr << "Reinitializing Dst.\r\n";
		Dst = cv::Mat(Src.rows, Src.cols, CV_8UC1);
	}

	//Build the histogram of the masked source image
	std::vector<int> hist(256, 0);
	for (int row = 0; row < Src.rows; row++) {
		for (int col = 0; col < Src.cols; col++) {
			uint8_t p = Src.at<uint8_t>(row,col);
			uint8_t m = Mask.at<uint8_t>(row,col);
			if (m > 0U)
				hist[p]++;
		}
	}

	//Build normalized histogram (probability distribution)
	int totalPixels = 0;
	for (int x : hist)
		totalPixels += x;
	if (totalPixels <= 0) {
		std::cerr << "Warning in EqualizeHistWithMask(): Entire image is masked off. Skipping.\r\n";
		return;
	}
	std::vector<float> pdist(256);
	for (size_t n = 0U; n < hist.size(); n++)
		pdist[n] = float(hist[n]) / float(totalPixels);

	//Build look-up-table for intensity transformation
	std::vector<uint8_t> LUT(256);
	for (size_t k = 0U; k < hist.size(); k++) {
		float cumProb = 0.0;
		for (size_t n = 0U; n <= k; n++)
			cumProb += pdist[n];
		LUT[k] = (uint8_t) std::min(std::floor(255.0f*cumProb), 255.0f);
	}

	//Transform all unmasked pixels
	for (int row = 0; row < Src.rows; row++) {
		for (int col = 0; col < Src.cols; col++) {
			uint8_t p = Src.at<uint8_t>(row,col);
			uint8_t m = Mask.at<uint8_t>(row,col);
			if (m > 0U)
				Dst.at<uint8_t>(row,col) = LUT[p];
			else
				Dst.at<uint8_t>(row,col) = p;
		}
	}
}

//Take a color BGR frame and perform any pre-processing then detect keypoints and build descriptors
inline void GetKeypointsAndDescriptors(cv::Mat const & Frame_BGR, std::vector<cv::KeyPoint> & Keypoints, cv::Mat & Descriptors,
	                                   cv::Mat const & ApertureMask) {
	cv::Mat Frame_Gray;
	cv::cvtColor(Frame_BGR, Frame_Gray, cv::COLOR_BGR2GRAY);

	cv::Mat Frame_EQ;
	//cv::equalizeHist(Frame_Gray, Frame_EQ);
	EqualizeHistWithMask(Frame_Gray, Frame_EQ, ApertureMask);

	cv::Ptr<cv::AKAZE> AKAZEDetector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.002f);
	AKAZEDetector->detectAndCompute(Frame_EQ, ApertureMask, Keypoints, Descriptors);
}

//Get the matrix representing the stabilization transform mapping points in the current frame to their corresponding
//locations in the reference frame. If the output is a 2x3 matrix it should be interpreted as an affine transformation,
//and should be applied using warpAffine(). If it is 3x3 it should be interpreted as a homography and should be applied
//using warpPerspective().
//RefFrame is only provided for visualization purposes (if enabled). It doesn't impact the transformation.
//Update: This function uses custom estimators and only supports an affine 2x3 matrix alignment model.
inline void GetStabilizationMatrix(cv::Mat const & Descriptors_Ref, std::vector<cv::KeyPoint> const & Keypoints_Ref,
	                               cv::Mat const & Frame_BGR, cv::Mat const & ApertureMask, cv::Mat & H, cv::Mat const & RefFrame) {
	std::vector<cv::KeyPoint> Keypoints_Frame;
	cv::Mat Descriptors_Frame;
	GetKeypointsAndDescriptors(Frame_BGR, Keypoints_Frame, Descriptors_Frame, ApertureMask);

	//Match keypoints with cross-check screening. With this method, for each descriptor in each collection we find the closest
	//descriptor in the other collection. A match is admitted if both keypoints choose each other as the best match in the other set.
	//This is a bit more expensive than distance ratio screening, but is often times less fidly.
	double maxKeypointMovement = 200.0;
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
	std::vector<cv::DMatch> RefSetMatches;
	std::vector<cv::DMatch> FrameSetMatches;
	matcher->match(Descriptors_Ref,   Descriptors_Frame, RefSetMatches);
	matcher->match(Descriptors_Frame, Descriptors_Ref,   FrameSetMatches);
	std::unordered_map<int, int> refMatches; //Ref index -> frame index
	for (cv::DMatch const & match : RefSetMatches) {
		int refKeypointIndex   = match.queryIdx;
		int frameKeypointIndex = match.trainIdx;
		refMatches[refKeypointIndex] = frameKeypointIndex;
	}
	//Build two vectors of cross-checked matches. Element n of RefKeypoints corresponds with element n of FrameKeypoints.
	//RefKeypoints[n] is the location of the matched keypoint in the reference frame.
	//FrameKeypoints[n] is the location of the matched keypoint in the current frame.
	std::vector<cv::Point2f> RefKeypoints;
	std::vector<cv::Point2f> FrameKeypoints;
	for (cv::DMatch const & match : FrameSetMatches) {
		int refKeypointIndex   = match.trainIdx;
		int frameKeypointIndex = match.queryIdx;
		if ((refMatches.count(refKeypointIndex) > 0U) && (refMatches.at(refKeypointIndex) == frameKeypointIndex) &&
			(cv::norm(Keypoints_Ref[refKeypointIndex].pt - Keypoints_Frame[frameKeypointIndex].pt) <= maxKeypointMovement)) {
			RefKeypoints.push_back(Keypoints_Ref[refKeypointIndex].pt);
			FrameKeypoints.push_back(Keypoints_Frame[frameKeypointIndex].pt);
		}
	}

	bool show_matches = false;
	if (show_matches) {
		cv::Mat vis;

		std::vector<cv::DMatch> validatedMatches;
		for (cv::DMatch const & match : FrameSetMatches) {
			int refKeypointIndex   = match.trainIdx;
			int frameKeypointIndex = match.queryIdx;
			if ((refMatches.count(refKeypointIndex) > 0U) && (refMatches.at(refKeypointIndex) == frameKeypointIndex) &&
				(cv::norm(Keypoints_Ref[refKeypointIndex].pt - Keypoints_Frame[frameKeypointIndex].pt) <= maxKeypointMovement))
				validatedMatches.emplace_back(refKeypointIndex, frameKeypointIndex, match.distance);
		}

		cv::Scalar matchCol = cv::Scalar::all(-1);
		cv::Scalar singlePointCol = cv::Scalar::all(-1);
		std::vector<char> matchMask = std::vector<char>();
		cv::DrawMatchesFlags flags = cv::DrawMatchesFlags::DEFAULT;
		cv::drawMatches(RefFrame, Keypoints_Ref, Frame_BGR, Keypoints_Frame, validatedMatches, vis, matchCol, singlePointCol, matchMask, flags);
		cv::imshow("Matches", vis);
		cv::waitKey(1);
	}
	
	//Estimate a transformation that maps the points in the current frame to the corresponding points in the reference frame
	//There is something subtle here. Since we are using a fisheye lens, which is not rectilinear, there won't generally exist
	//a homography that properly maps points in the image plane of the current frame to the image plane of the reference frame.
	//Any homography fit in one portion of the images will not fit well in other parts of the images. This makes RANSAC a poor
	//choice for finding the transformation. RANSAC will fit transformations to random collections of point matches from all over
	//the images: depending on the matches chosen in a given "run" some candidates will be good fits in the central region and others
	//will be good fits in portions of the higher-distortion periphery (and some will be outliers, of course). The one we end up
	//selecting will come down to the distribution of keypoints throughout the images and the inlier threshold. Sometimes we might
	//get one that does a good job in the central region and other times we'll get one that only fits well in some portion of the
	//periphery, but the one we select is pretty random. What we want is a method that is robust but uses all of the point matches
	//to ensure that we get a transformation that is a decent "all-over" fit and not a great fit in one portion of the images and a
	//terrible one elsewhere. This gives generally better stabilization and much better temporal stability.
	//
	//The robust all-over methods, like LMEDS, don't typically handle outlier ratios above 0.5. Thus, to make this work, we need to
	//be careful to try and keep our outlier ratio down. We do this by using cross-check screening in the keypoint matching process
	//and by throwing out matches where the distance between the points exceeds an a-priori threshold (which could only correspond to
	//a transformation pretty far from the identity). It's important to note the limitation that this imposes: this function is
	//only designed to work with video that is already nominally stable... that is, where the transformations required to align
	//new frames with the reference frame are all relatively close to the identity.
	//
	//We have also experimentally observed that all of this works a bit better with simpler alignment models. Thus, rather than
	//using a homography, or even RTS affine transformation, we use a custom robust solver to find a simple RT transformation.
	try {
		//Simplest alignment model... RT transformation
		H = EstimateRTTransformation(FrameKeypoints, RefKeypoints);

		//Next-simplest alignment model... RTS transformation
		//H = cv::estimateAffinePartial2D(FrameKeypoints, RefKeypoints, cv::noArray(), cv::LMEDS, 15.0);
		//Force the scale factor to exactly 1
		//This is not actually a good idea - not the same as projecting H to the manifold of scale-1 transformations because
		//we aren't adjusting the translation component properly. Just something to try since it is simple to do.
		//double H11 = H.at<double>(0,0);
		//double H21 = H.at<double>(1,0);
		//double s = std::sqrt(H11*H11 + H21*H21);
		//H.at<double>(0,0) /= s;
		//H.at<double>(0,1) /= s;
		//H.at<double>(1,0) /= s;
		//H.at<double>(1,1) /= s;

		//Most general of the "reasonable" alignment models... a homography
		//H = cv::findHomography(FrameKeypoints, RefKeypoints, cv::LMEDS, 3.0, cv::noArray(), 2000, 0.995);
	}
	catch (...) {
		//Some sovers can throw exceptions in some cases... we default to the identity transformation in such cases.
		std::cerr << "cv::estimateAffinePartial2D failed. Defaulting stabilization homography.\r\n";
		H = cv::Mat::eye(2, 3, CV_64F);
	}
}

//Note: This function is misleadingly named. estimateAffinePartial2D() estimates a 4DOF affine transformation that allows rotation, translation,
//and uniform scaling. This is likily too general for image stabilization... rotation and translation only would be better.
/*inline void getOrbRotation(const cv::Mat& descriptors_ref, const std::vector<cv::KeyPoint>& keypoints_ref, const cv::Mat& rot_color, cv::Mat& H) {
	cv::Mat img_rot;
	std::vector<cv::KeyPoint> keypoints_rot;
	cv::Mat descriptors_rot;
	//std::vector<cv::DMatch> good_matches;
	std::vector<cv::Point2f> ref;
	std::vector<cv::Point2f> rot;

	cv::cvtColor(rot_color, img_rot, cv::COLOR_BGR2GRAY);
	cv::Ptr<cv::ORB> detector = cv::ORB::create();
	
	detector->detectAndCompute(img_rot, cv::noArray(), keypoints_rot, descriptors_rot);

	cv::Mat frameWithKeypoints;
	cv::drawKeypoints(img_rot, keypoints_rot, frameWithKeypoints, cv::Scalar(255,0,0));
	cv::imshow("Non Stabilized Frame", frameWithKeypoints);
	cv::waitKey(1);

	{
		//cv::Mat frame_HSV;
		//cv::cvtColor(rot_color, frame_HSV, cv::COLOR_BGR2HSV);
		//std::vector<cv::Mat> HSVChannels;
		//cv::split(frame_HSV, HSVChannels);
		//cv::Mat Hue = HSVChannels[0];
		//cv::Mat Sat = HSVChannels[1];

		cv::Mat apertureMask;
		getApertureMask(rot_color, apertureMask);

		//cv::Mat HueEQ;
		//cv::equalizeHist(Hue, HueEQ);
		//EqualizeHistWithMask(Hue, HueEQ, apertureMask);
		
		//cv::Mat SatEQ;
		//cv::equalizeHist(Sat, SatEQ);
		//EqualizeHistWithMask(Sat, SatEQ, apertureMask);

		cv::Mat imgEQ;
		//cv::equalizeHist(img_rot, imgEQ);
		EqualizeHistWithMask(img_rot, imgEQ, apertureMask);

		//cv::Mat HS;
		//cv::addWeighted(Hue, 0.5, Sat, 0.5, 0.0, HS);

		cv::Ptr<cv::AKAZE> AKAZEDetector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.002f);
		std::vector<cv::KeyPoint> keypoints_AKAZE;
		cv::Mat descriptors_AKAZE;
		AKAZEDetector->detectAndCompute(imgEQ, apertureMask, keypoints_AKAZE, descriptors_AKAZE);

		std::vector<cv::KeyPoint> keypoints_AKAZE_Screened;
		for (auto const & kp : keypoints_AKAZE) {
			if (kp.octave > 0)
				keypoints_AKAZE_Screened.push_back(kp);
		}


		cv::Mat frameWithAKAZEKeypoints;
		cv::drawKeypoints(imgEQ, keypoints_AKAZE, frameWithAKAZEKeypoints, cv::Scalar(0,0,255));
		cv::drawKeypoints(frameWithAKAZEKeypoints, keypoints_AKAZE_Screened, frameWithAKAZEKeypoints, cv::Scalar(255,0,0));
		cv::imshow("AKAZE Keypoints", frameWithAKAZEKeypoints);
		cv::waitKey(1);
	}

	//Match keypoints with distance ratio screening. With this method, for each descriptor in a "query" set we find the 2 closest
	//descriptors in a "train" set. The match is admitted if the best match is substantially closer to the query descriptor than
	//the second closest match.
	//cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
	//std::vector< std::vector<cv::DMatch> > knn_matches;
	//matcher->knnMatch(descriptors_ref, descriptors_rot, knn_matches, 2);

	//const float ratio_thresh = 0.75f;
	
	//for (size_t i = 0; i < knn_matches.size(); i++)
	//{
	//	if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
	//	{
	//		ref.push_back(keypoints_ref[knn_matches[i][0].queryIdx].pt);
	//		rot.push_back(keypoints_rot[knn_matches[i][0].trainIdx].pt);
	//		//good_matches.push_back(knn_matches[i][0]);
	//	}
	//}

	//Match keypoints with cross-check screening. With this method, for each descriptor in each collection we find the closest
	//descriptor in the other collection. A match is admitted if both keypoints choose each other as the best match in the other set.
	//This is a bit more expensive than distance ratio screening, but is often times less fidly.
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
	std::vector<cv::DMatch> RefSetMatches;
	std::vector<cv::DMatch> RotSetMatches;
	matcher->match(descriptors_ref, descriptors_rot, RefSetMatches);
	matcher->match(descriptors_rot, descriptors_ref, RotSetMatches);
	std::unordered_map<int, int> refMatches; //Ref index -> rot index
	for (cv::DMatch const & match : RefSetMatches) {
		int refKeypointIndex = match.queryIdx;
		int rotKeypointIndex = match.trainIdx;
		refMatches[refKeypointIndex] = rotKeypointIndex;
	}
	for (cv::DMatch const & match : RotSetMatches) {
		int refKeypointIndex = match.trainIdx;
		int rotKeypointIndex = match.queryIdx;
		if ((refMatches.count(refKeypointIndex) > 0U) && (refMatches.at(refKeypointIndex) == rotKeypointIndex)) {
			ref.push_back(keypoints_ref[refKeypointIndex].pt);
			rot.push_back(keypoints_rot[rotKeypointIndex].pt);
		}
	}

	
	//This next function appears to raise an exception sometimes. This is probably because there aren't enough matches to run the algorithm.
	//It looks like we just put as many items in these vectors as we have ORB keypoint matches that pass our inclusion test. There is no guarentee that there
	//will be at least 2 of them. This is especially likily if we get a corrupted frame from the drone... there may be no matches.
	try {
		H = cv::estimateAffinePartial2D(rot, ref);

		//Force the scale factor to exactly 1
		double H11 = H.at<double>(0,0);
		double H21 = H.at<double>(1,0);
		double s = std::sqrt(H11*H11 + H21*H21);
		H.at<double>(0,0) /= s;
		H.at<double>(0,1) /= s;
		H.at<double>(1,0) /= s;
		H.at<double>(1,1) /= s;

		//H = EstimateRTTransformation(rot, ref);
		//cv::Mat HPrime = EstimateRTTransformation(rot, ref);
		//std::cerr << H << "\r\n";
		//std::cerr << HPrime << "\r\n\r\n";
	}
	catch (...) {
		std::cerr << "cv::estimateAffinePartial2D failed. Defaulting stabilization homography.\r\n";
		H = cv::Mat::eye(2, 3, CV_64F);
	}
}*/

/*inline void getBinaryCloudMask(const cv::Mat& img, cv::Mat& bright, cv::Mat& binary) {
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
}*/

//Get a mask that is 0 for invalid pixels (light is not focused on these pixels by the lens) and 255 for valid pixels.
inline void getApertureMask(const cv::Mat & Img, cv::Mat & Mask) {
	//The original implementation appeared to do a lot of copying and flood filling and I can't quite figure out
	//what the intended logic was since intermediate images always seem trivial (e.g. all white). This has been re-written
	
	//Re-allocate Mask if it has the wrong dims, channels, or type
	if ((Mask.rows != Img.rows) || (Mask.cols != Img.cols) || (Mask.channels() != 1) || (Mask.depth() != CV_8U))
		Mask = cv::Mat(Img.rows, Img.cols, CV_8UC1);

	cv::Mat img_gray, img_thresh;
	cvtColor(Img, img_gray, cv::COLOR_BGR2GRAY);
	threshold(img_gray, img_thresh, 50, 255, cv::THRESH_BINARY);

	//Find outermost contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(img_thresh, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	//The largest outer contour should be our aperture contour (very similar to karthik's code)
	Mask.setTo(0);
	if (contours.size() > 0) {
		size_t largestContourIndex = 0U;
		double largestContourArea  = -1.0;
		for (size_t n = 0U; n < contours.size(); n++) {
			double contourArea = cv::contourArea(contours[n]);
			if ((n == 0U) || (contourArea > largestContourArea)) {
				largestContourIndex = n;
				largestContourArea = contourArea;
			}
		}

		//Clean up the mask by fitting an ellipse to the contour
		cv::RotatedRect rect = cv::fitEllipse(contours[largestContourIndex]);
		rect.size.width  -= 4.0f; //Erode slightly: 2 pixels on all sides
		rect.size.height -= 4.0f; //Erode slightly: 2 pixels on all sides
		cv::ellipse(Mask, rect, cv::Scalar(255), -1);

		//Fill the contour directly, without any clean up
		//cv::drawContours(Mask, contours, largestContourIndex, cv::Scalar(255), -1);
	}

	/*cv::Mat img_thresh, img_binary, img_filled, img_filled_inv, img_final;

	cvtColor(img, img_binary, cv::COLOR_RGB2GRAY);
	threshold(img_binary, img_thresh, 30, 255, cv::THRESH_BINARY);
	img_thresh.copyTo(img_filled);

	if (img.rows == 512) {
		cv::imshow("A", img_thresh);
		cv::waitKey(1);
	}
	
	floodFill(img_filled, cv::Point(0, 0), cv::Scalar(255));
	floodFill(img_filled, cv::Point(0, img.rows - 1), cv::Scalar(255));
	floodFill(img_filled, cv::Point(img.cols - 1, 0), cv::Scalar(255));
	floodFill(img_filled, cv::Point(img.cols - 1, img.rows - 1), cv::Scalar(255));
	
	if (img.rows == 512) {
		cv::imshow("B", img_filled);
		cv::waitKey(1);
	}

	bitwise_not(img_filled, img_filled_inv);
	mask = img_thresh | img_filled_inv;

	if (img.rows == 512) {
		cv::imshow("C", mask);
		cv::waitKey(1);
	}

	floodFill(mask, cv::Point(0, 0), cv::Scalar(0));
	floodFill(mask, cv::Point(0, img.rows - 1), cv::Scalar(0));
	floodFill(mask, cv::Point(img.cols - 1, 0), cv::Scalar(0));
	floodFill(mask, cv::Point(img.cols - 1, img.rows - 1), cv::Scalar(0));

	if (img.rows == 512) {
		cv::imshow("D", mask);
		cv::waitKey(1);
	}*/
	//std::cerr << mask.rows << " x " << mask.cols << ", " << mask.channels() << ", " << mask.depth() << "\r\n";
}

// Any value set to 255 is set to 254
// Reserve 255 as the sentinel value for NAN
// Value is supposed to be CV_8UC3 but only the first value is used
/*inline void create_masked_binary(const int mapRows, const int mapCols, cv::Mat const & mask, cv::Mat & binary_sampled){
    for (uint32_t row = 0U; row < (uint32_t) mapRows; row++) {
        for (uint32_t col = 0U; col < (uint32_t) mapCols; col++) {
		    // Set any pixel with value of 255 to 254
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
}*/

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

//Perform a masked 2D moving average filter. Done as 2 1D passes. K should be an odd integer and represents the width of
//the filter window. Currently only supports single channel matrices of type uint8_t
//pixels where Mask is 0 are excluded from the average. This means each pixel in Out is approximately the average of
//all unmasked pixels in a K-by-K box centered at the pixel in question. Note that this isn't exactly correct since we
//do the filtering in two 1D passes, so some pixels may get more weight than others but it is close enough in most cases.
inline void MAFilter(cv::Mat const & In, cv::Mat const & Mask, cv::Mat & Out, int K) {
	if ((Out.rows != In.rows) || (Out.cols != In.cols) || (Out.channels() != 1) || (Out.depth() != CV_8U))
		Out = cv::Mat(In.rows, In.cols, CV_8UC1);
	int radius = K/2;
	
	cv::Mat Filtered1D(In.rows, In.cols, In.type());
	for (int row = 0; row < In.rows; row++) {
		for (int col = 0; col < In.cols; col++) {
			//Set Filtered1D(row, col) to the average of In over a horizontal window centered at (row, col)
			unsigned int sum = 0U;
			unsigned int tally = 0U;
			for (int sampleCol = col - radius; sampleCol <= col + radius; sampleCol++) {
				if ((sampleCol < 0) || (sampleCol >= In.cols))
					continue;
				if (Mask.at<uint8_t>(row, sampleCol) > uint8_t(0)) {
					sum += In.at<uint8_t>(row, sampleCol);
					tally++;
				}
			}
			if (tally == 0U)
				Filtered1D.at<uint8_t>(row, col) = (uint8_t) 0U;
			else
				Filtered1D.at<uint8_t>(row, col) = (uint8_t) std::min(sum/tally, 255U);
		}
	}

	for (int row = 0; row < In.rows; row++) {
		for (int col = 0; col < In.cols; col++) {
			//Set Out(row, col) to the average of Filtered1D over a vertical window centered at (row, col)
			unsigned int sum = 0U;
			unsigned int tally = 0U;
			for (int sampleRow = row - radius; sampleRow <= row + radius; sampleRow++) {
				if ((sampleRow < 0) || (sampleRow >= In.rows))
					continue;
				if (Mask.at<uint8_t>(sampleRow, col) > uint8_t(0)) {
					sum += Filtered1D.at<uint8_t>(sampleRow, col);
					tally++;
				}
			}
			if (tally == 0U)
				Out.at<uint8_t>(row, col) = (uint8_t) 0U;
			else
				Out.at<uint8_t>(row, col) = (uint8_t) std::min(sum/tally, 255U);
		}
	}
}