//Utilities to support coordinate frame transformations and cam projection/unprojection, and pose finding for the shadow
//detection module.
//
//Original Author: Ben Choi (Stanford)
//Additional Authors: Elaina Chai (Stanford), Bryan Poling (Sentek Systems, LLC)
#pragma once

//System Includes
#include <random>
#include <chrono>
#include <numeric>
#include <algorithm>

//Project Includes
#include "../../Maps/MapUtils.hpp"

//#include "../../Utilities.hpp" //Only needed for benchmarking

//Back-project a collection of points in the image plane - find unit bearing vectors in the camera frame for each image point to all the
//world points that project onto it.
inline void multCam2World(std::Evector<Eigen::Vector2d> const & Pixel_coords, std::vector<cv::Point3d> & Backprojected, ocam_model const & o) {
	for (int i = 0; i < (int) Pixel_coords.size(); i++) {
		double point3D[3];
		// y,x => row, col => 1, 0
		double point2D[2] = { Pixel_coords[i](1), Pixel_coords[i](0) };
		cam2world(point3D, point2D, &o);
		Backprojected.push_back(cv::Point3d(point3D[0], point3D[1], point3D[2]));
	}
}

//Sample 3 world vectors and back-projections to pack into the output matrices world_vec and bearing_vec
inline void getPoseInputMatrices(std::Evector<Eigen::Vector3d> const & world, std::vector<cv::Point3d> const & backproj,
	                            Eigen::Matrix3d & world_vec, Eigen::Matrix3d & bearing_vec, unsigned int seed) {
	std::vector<int> allIndices(world.size());
	std::iota(allIndices.begin(), allIndices.end(), 0);
	std::vector<int> sampleIndices;
	std::sample(allIndices.begin(), allIndices.end(), std::back_inserter(sampleIndices), 3, std::default_random_engine(seed));

	for (int n = 0; n < 3; n++) {
		int index = sampleIndices[n];
		bearing_vec.col(n) << backproj[index].x, backproj[index].y, backproj[index].z;
		world_vec.col(n)   << world[index](0),   world[index](1),   world[index](2);
	}
}

//Project multiple points in a world frame to the camera image plane. Output camera-frame points are passed as the rows of matrix "Pos_Cam"
inline void multWorld2CAM(std::Evector<Eigen::Vector3d> const & Pos_World, cv::Mat & Pos_Cam, Eigen::Matrix3d const & R_Cam_World,
	                   Eigen::Vector3d const & CamCenter_World, ocam_model const & o) {
	Pos_Cam = cv::Mat(Pos_World.size(), 2, CV_64F);
	Eigen::Matrix3d R_World_Cam = R_Cam_World.transpose();
	for (int row = 0; row < (int) Pos_World.size(); row++) {
		Eigen::Vector3d v_cam = R_World_Cam * (Pos_World[row] - CamCenter_World);
		double point3D[3] = { v_cam(0), v_cam(1), v_cam(2) };
		double point2D[2];

		world2cam(point2D, point3D, &o);

		Pos_Cam.at<double>(row, 0) = point2D[1];
		Pos_Cam.at<double>(row, 1) = point2D[0];
	}
}

//Compute reprojection error for a given collection of world points in a fixed world frame, their projections to the image plane,
//and a given camera pose relative to the world frame. This function computes the mean error of the lowest "FractionIncluded"
//reprojection errors. Returned error is in units of pixels.
inline double reprojectionError(cv::Mat const & orig_PX, std::Evector<Eigen::Vector3d> const & orig_World, Eigen::Matrix3d const & R_Cam_World,
	                           Eigen::Vector3d const & CamCenter_World, ocam_model const & o, double FractionIncluded) {
	//Handle degenerate case and sanitize inputs
	if (orig_PX.rows == 0)
		return 0.0;
	FractionIncluded = std::clamp(FractionIncluded, 0.0, 1.0);

	cv::Mat reprojected;
	multWorld2CAM(orig_World, reprojected, R_Cam_World, CamCenter_World, o);

	//Here is a relatively clean implementation using standard STL containers and STL sort instead of OpenCV vectorized matrix operations
	//and reduction. This gives exactly the same results as the OpenCV version, but is 2X - 5X slower. Just here to check output.
	/*std::vector<double> reprojectionErrors(orig_PX.rows);
	for (int n = 0; n < orig_PX.rows; n++)
		reprojectionErrors[n] = cv::norm(reprojected.row(n) - orig_PX.row(n));
	std::sort(reprojectionErrors.begin(), reprojectionErrors.end());
	size_t lastIndexToInclude = (size_t) std::round(FractionIncluded * double(reprojectionErrors.size() - 1U));
	double totalError = 0.0;
	for (size_t n = 0U; n <= lastIndexToInclude; n++)
		totalError += reprojectionErrors[n];
	double meanError = totalError / double(lastIndexToInclude + 1U);*/
	
	//Faster solution using OpenCV vectorized matrix operations and reduction.
	cv::Mat norm;
	cv::Mat diff = orig_PX - reprojected;
	cv::reduce(diff.mul(diff), norm, 1, cv::REDUCE_SUM, CV_64F);
	cv::sqrt(norm, norm);
	cv::sort(norm, norm, cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);
	int end = FractionIncluded * norm.rows;
	cv::Mat subMatrix = cv::Mat(norm, cv::Range(0, end), cv::Range::all());
	double meanError = cv::sum(subMatrix)[0]/double(end);

	return meanError;
}

//Use RANSAC and LambdaTwist to find the camera pose relative to a fixed Euclidean world-frame (e.g. LEA, ENU, or NED)
//Pose is passed back through reference arguments R_Cam_World and CamCenter_World.
//Points in the world frame and the camera frame are related as follows:
//X_Cam = R_Cam_World.transpose() * (X_World - CamCenter_World)
//where X_World is a point in the world frame and X_Cam is the same point in the (3D) camera frame.
inline void findPose(std::Evector<Eigen::Vector2d> const & fiducials_PX, std::Evector<Eigen::Vector3d> const & fiducials_World,
	                ocam_model const & o, Eigen::Matrix3d & R_Cam_World, Eigen::Vector3d & CamCenter_World) {
	std::vector<cv::Point3d> fiducials_BACKPROJ;
	Eigen::Matrix3d input_world, input_bearing;
	std::Evector<std::tuple<Eigen::Vector3d, Eigen::Matrix3d>> possiblePoses;
	//unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	unsigned int seed = 0; //Detirministic but pseudo-random behavior
	double best_error = -1;
	std::tuple<Eigen::Vector3d, Eigen::Matrix3d> best_pose;

	multCam2World(fiducials_PX, fiducials_BACKPROJ, o);

	cv::Mat f_px(fiducials_PX.size(), 2, CV_64F);
	for (size_t n = 0U; n < fiducials_PX.size(); n++) {
		f_px.at<double>(n,0) = fiducials_PX[n](0);
		f_px.at<double>(n,1) = fiducials_PX[n](1);
	}
	
	for (int i = 0; i < 5000; i++) {
		getPoseInputMatrices(fiducials_World, fiducials_BACKPROJ, input_world, input_bearing, seed + i);
		
		LambdaTwistSolve(input_bearing, input_world, possiblePoses, true);
		
		for (int poseIndex = 0; poseIndex < (int) possiblePoses.size(); poseIndex++) {
			std::tuple<Eigen::Vector3d, Eigen::Matrix3d> pose = possiblePoses[poseIndex];
			R_Cam_World = std::get<1>(pose);
			CamCenter_World = std::get<0>(pose);
			
			double reproj_error = reprojectionError(f_px, fiducials_World, R_Cam_World, CamCenter_World, o, 1);
			if ((best_error == -1) || (reproj_error < best_error)) {
				best_error = reproj_error;
				best_pose = pose;
			}
		}
		
	}
	R_Cam_World = std::get<1>(best_pose);
	CamCenter_World = std::get<0>(best_pose);

	//cout << "\nBest error: " << best_error << endl;
	//std::cout << "Pose estimated." << std::endl;
}

//Wrapper for ECEF2LLA() that takes a cv::Point3d input and returns a cv::Point3d
//Compute latitude (radians), longitude (radians), and altitude (height above WGS84 ref. elipsoid, in meters) from ECEF position (in meters)
//Latitude and longitude are also both given with respect to the WGS84 reference elipsoid.
inline cv::Point3d positionECEF2LLA(cv::Point3d const & Position_ECEF) {
	Eigen::Vector3d Position_LLA = ECEF2LLA(Eigen::Vector3d(Position_ECEF.x, Position_ECEF.y, Position_ECEF.z));
	return cv::Point3d(Position_LLA(0), Position_LLA(1), Position_LLA(2));
}

//(col, row)-pixel coords to reference coords
//NorthBounds and EastBounds hold the EN limits of the reference plane, in meters
//GSD is the ground sampling distance, in meters.
//N looks like the number of pixels in the reference plane in the North-South direction
inline Eigen::Vector2d PixCoordsToRefCoords(Eigen::Vector2d const & PixCoords, double GSD, double N, Eigen::Vector2d const & NorthBounds,
	                                       Eigen::Vector2d const & EastBounds) {
	return(Eigen::Vector2d(GSD * PixCoords(0) + EastBounds(0), GSD * (double(N - 1) - PixCoords(1)) + NorthBounds(0)));
}

//Compute the camera pose relative to the local level ENU frame from the camera pose relative to the Local ECEF-Aligned frame (LEA).
//Inputs:  LEAOrigin_ECEF - Origin of LEA frame in ECEF
//         R_Cam_LEA      - Rotation between Cam and LEA frame
//         CamCenter_LEA  - Position of camera center in LEA frame
//Outputs: R_Cam_ENU      - Rotation between Cam and ENU frame
//         CamCenter_ENU  - Position of camera center in ENU frame
//         ENUOrigin_ECEF - Origin of the ENU frame in ECEF
inline void poseLEA2ENU(Eigen::Vector3d const & LEAOrigin_ECEF, Eigen::Matrix3d const & R_Cam_LEA, Eigen::Vector3d const & CamCenter_LEA,
	                   Eigen::Matrix3d & R_Cam_ENU, Eigen::Vector3d & CamCenter_ENU, Eigen::Vector3d & ENUOrigin_ECEF) {
	Eigen::Vector3d CamCenter_ECEF = CamCenter_LEA + LEAOrigin_ECEF;
	
	Eigen::Vector3d cam_center_LLA = ECEF2LLA(CamCenter_ECEF);
	Eigen::Vector3d LEAOrigin_LLA  = ECEF2LLA(LEAOrigin_ECEF);
	Eigen::Vector3d ENUOrigin_LLA(cam_center_LLA(0) , cam_center_LLA(1), LEAOrigin_LLA(2));

	Eigen::Matrix3d C_ECEF_ENU = latLon_2_C_ECEF_ENU(ENUOrigin_LLA(0), ENUOrigin_LLA(1));
	ENUOrigin_ECEF = LLA2ECEF(ENUOrigin_LLA);

	R_Cam_ENU = C_ECEF_ENU * R_Cam_LEA;
	CamCenter_ENU = C_ECEF_ENU * (CamCenter_ECEF - ENUOrigin_ECEF);
}

//Compute the center of the visible portion of the EN plane and the farthest we can see in that plane.
//Outputs: center - EN coordinates of center of visible area in the EN plane
//         max_extent - distance between the east-most visible point and the west-most visible point (in meters), or
//                      the distance between the north-most visible point and the south-most visible point (in meters),
//                      whichever is greater.
inline void get_centered_extent(Eigen::Vector3d const & LEAOrigin_ECEF, double const & aperture_distance_px, cv::Size const & FINAL_SIZE,
	                           Eigen::Matrix3d const & R_Cam_ENU, Eigen::Vector3d const & CamCenter_ENU,
	                           Eigen::Vector3d const & CamCenter_LEA, ocam_model & o, Eigen::Vector2d & center, double & max_extent) {
	double PI = 3.14159265358979;

	double aperture3D[3];
	double aperture2D[2] = { (double)FINAL_SIZE.height / 2 + aperture_distance_px,  (double)FINAL_SIZE.width / 2 };
	cam2world(aperture3D, aperture2D, &o);
	double theta = acos(std::abs(aperture3D[2])); //The maximum chief ray angle

	std::cerr << "Max chief ray angle: " << theta*180.0/PI << "\r\n";

	//Compute the optical axis in the ENU frame - note that the omni camera model we use has an unusual definition (hence the -1)
	Eigen::Vector3d OA_ENU = R_Cam_ENU * Eigen::Vector3d(0, 0, -1);

	//std::cerr << "OA_ENU:\r\n" << OA_ENU << "\r\n\r\n";

	//Rotate the optical axis north, south, east, and west by theta
	Eigen::AngleAxisd AA1(theta, Eigen::Vector3d(1, 0, 0));
	Eigen::AngleAxisd AA2(theta, Eigen::Vector3d(-1, 0, 0));
	Eigen::AngleAxisd AA3(theta, Eigen::Vector3d(0, 1, 0));
	Eigen::AngleAxisd AA4(theta, Eigen::Vector3d(0, -1, 0));
	Eigen::Vector3d v1 = AA1.toRotationMatrix() * OA_ENU;
	Eigen::Vector3d v2 = AA2.toRotationMatrix() * OA_ENU;
	Eigen::Vector3d v3 = AA3.toRotationMatrix() * OA_ENU;
	Eigen::Vector3d v4 = AA4.toRotationMatrix() * OA_ENU;

	//If any of the rotated vectors are almost horizontal or pointing up, rotate them down toward the Earth
	v1(2) = std::min(v1(2), -0.01);
	v2(2) = std::min(v2(2), -0.01);
	v3(2) = std::min(v3(2), -0.01);
	v4(2) = std::min(v4(2), -0.01);

	//Find the intersections of the rotated vectors and the EN plane
	Eigen::Vector3d X1_ENU = CamCenter_ENU - v1 * CamCenter_ENU(2) / v1(2);
	Eigen::Vector3d X2_ENU = CamCenter_ENU - v2 * CamCenter_ENU(2) / v2(2);
	Eigen::Vector3d X3_ENU = CamCenter_ENU - v3 * CamCenter_ENU(2) / v3(2);
	Eigen::Vector3d X4_ENU = CamCenter_ENU - v4 * CamCenter_ENU(2) / v4(2);

	double eastMin  = cv::min({ X1_ENU(0), X2_ENU(0), X3_ENU(0), X4_ENU(0) });
	double eastMax  = cv::max({ X1_ENU(0), X2_ENU(0), X3_ENU(0), X4_ENU(0) });
	double northMin = cv::min({ X1_ENU(1), X2_ENU(1), X3_ENU(1), X4_ENU(1) });
	double northMax = cv::max({ X1_ENU(1), X2_ENU(1), X3_ENU(1), X4_ENU(1) });

	max_extent = cv::max({ eastMax - eastMin, northMax - northMin });
	center = Eigen::Vector2d(0.5 * (eastMin + eastMax), 0.5 * (northMin + northMax));
}

inline void positionPX2LLA(cv::Mat & frame, Eigen::Vector2d const & pixel_coords,
	                      Eigen::Vector2d const & center, const double & max_extent, const double & num_pixels,
	                      Eigen::Vector3d & point_LLA, Eigen::Matrix3d & R_Cam_ENU,
	                      Eigen::Vector3d & CamCenter_ENU, ocam_model const & o,
	                      Eigen::Vector3d const & ENUOrigin_ECEF, Eigen::Matrix3d const & C_ECEF_ENU) {
	Eigen::Vector2d NorthBounds(center(1) - max_extent / 2, center(1) + max_extent / 2);
	Eigen::Vector2d EastBounds(center(0) - max_extent / 2, center(0) + max_extent / 2);
	double GSD = max_extent / num_pixels;

	Eigen::Vector2d ref_coords = PixCoordsToRefCoords(pixel_coords, GSD, num_pixels, NorthBounds, EastBounds);

	Eigen::Vector3d pixel_enu(ref_coords(0), ref_coords(1), 0);
	Eigen::Vector3d pixel_ECEF = C_ECEF_ENU.transpose()*pixel_enu + ENUOrigin_ECEF;
	point_LLA = ECEF2LLA(pixel_ECEF);

	Eigen::Vector3d pixel_cam = R_Cam_ENU.transpose() * (pixel_enu - CamCenter_ENU);
	double point_bearing[3] = { pixel_cam(0), pixel_cam(1), pixel_cam(2) };
	double point_pixel[2];

	world2cam(point_pixel, point_bearing, &o);

	circle(frame, cv::Point2d(point_pixel[1], point_pixel[0]), 3, cv::Scalar(255, 0, 0), -1);
}

inline void sampleENUSquare(cv::Mat & inputFrame, ocam_model const & o, Eigen::Matrix3d & R_Cam_ENU, Eigen::Vector3d & CamCenter_ENU,
	                       const Eigen::Vector2d & center, const double & max_extent, const double & num_pixels,
	                       bool showFrames, cv::Mat & outputFrame) {
	if ((inputFrame.channels() != 1) && (inputFrame.channels() != 3)) {
		std::cerr << "Error in sampleENUSquare: Unsupported number of image channels.\r\n";
		return;
	}

	Eigen::Vector2d NorthBounds(center(1) - max_extent / 2, center(1) + max_extent / 2);
	Eigen::Vector2d EastBounds(center(0) - max_extent / 2, center(0) + max_extent / 2);
	double GSD = max_extent / num_pixels;

	outputFrame = cv::Mat(num_pixels, num_pixels, inputFrame.type());

	cv::Mat frameCopyBGR;
	if (showFrames) {
		if (inputFrame.channels() == 3)
			inputFrame.copyTo(frameCopyBGR);
		else
			cv::cvtColor(inputFrame, frameCopyBGR, cv::COLOR_GRAY2BGR);
	}

	for (int row = 0; row < num_pixels; row++) {
		for (int col = 0; col < num_pixels; col++) {
			Eigen::Vector2d pixel_coords(col, row);
			Eigen::Vector2d ref_coords = PixCoordsToRefCoords(pixel_coords, GSD, num_pixels, NorthBounds, EastBounds);

			Eigen::Vector3d pixel_enu(ref_coords(0), ref_coords(1), 0);

			Eigen::Vector3d pixel_cam = R_Cam_ENU.inverse() * (pixel_enu - CamCenter_ENU);
			double point_bearing[3] = { pixel_cam(0), pixel_cam(1), pixel_cam(2) };
			double point_pixel[2];

			world2cam(point_pixel, point_bearing, &o);

			if (showFrames)
				circle(frameCopyBGR, cv::Point2d(point_pixel[1], point_pixel[0]), 3, cv::Scalar(255, 0, 0), -1);

			cv::Point2d sample_point(point_pixel[1], point_pixel[0]);
			if (inputFrame.channels() == 1)
				outputFrame.at<uint8_t>(row, col) = getColorSubpixHelper_UC1(inputFrame, sample_point);
			else if (inputFrame.channels() == 3)
				outputFrame.at<cv::Vec3b>(row, col) = getColorSubpixHelper_UC3(inputFrame, sample_point);
		}
	}

	if (showFrames) {
		cv::imshow("Sample Locations", frameCopyBGR);
		cv::waitKey(1);
	}
}

//Composes a sequence of transformations to go from pixel coordinates in the EN-plane image to raw image coordinates
//and then for each pixel in the EN-plane image, we sample the raw image in the corresponding location.
inline void RawImageToENImage(cv::Mat const & ImageRaw, ocam_model const & o, Eigen::Matrix3d const & R_Cam_ENU,
	                         Eigen::Vector3d const & CamCenter_ENU, Eigen::Vector2d const & Center_EN, double max_extent, double num_pixels,
	                         bool showFrames, cv::Mat & ENImage, cv::Mat StabilizationMatrix = cv::Mat::eye(2, 3, CV_64F)) {
	if ((ImageRaw.channels() != 1) && (ImageRaw.channels() != 3)) {
		std::cerr << "Error in RawImageToENImage: Unsupported number of image channels.\r\n";
		return;
	}
	if ((StabilizationMatrix.rows != 2) || (StabilizationMatrix.cols != 3)) {
		std::cerr << "Error in RawImageToENImage: Stabilization matrix has unsupported dimensions.\r\n";
		return;
	}

	//Invert the stabilization transformation. Inverse transformation will be applied: T(x) = A*x + b
	Eigen::Matrix2d A;
	Eigen::Vector2d b;
	{
		double h11 = StabilizationMatrix.at<double>(0,0);
		double h12 = StabilizationMatrix.at<double>(0,1);
		double h13 = StabilizationMatrix.at<double>(0,2);
		double h21 = StabilizationMatrix.at<double>(1,0);
		double h22 = StabilizationMatrix.at<double>(1,1);
		double h23 = StabilizationMatrix.at<double>(1,2);
		Eigen::Matrix2d H_2X2;
		H_2X2 << h11, h12,
		         h21, h22;
		A = H_2X2.inverse();
		b = -1.0 * A * Eigen::Vector2d(h13, h23);
	}

	Eigen::Vector2d NorthBounds(Center_EN(1) - max_extent / 2, Center_EN(1) + max_extent / 2);
	Eigen::Vector2d EastBounds(Center_EN(0) - max_extent / 2, Center_EN(0) + max_extent / 2);
	double GSD = max_extent / num_pixels;

	ENImage = cv::Mat(num_pixels, num_pixels, ImageRaw.type());

	cv::Mat frameCopyBGR;
	if (showFrames) {
		if (ImageRaw.channels() == 3)
			ImageRaw.copyTo(frameCopyBGR);
		else
			cv::cvtColor(ImageRaw, frameCopyBGR, cv::COLOR_GRAY2BGR);
	}

	Eigen::Matrix3d R_ENU_Cam = R_Cam_ENU.transpose();
	for (int row = 0; row < num_pixels; row++) {
		for (int col = 0; col < num_pixels; col++) {
			Eigen::Vector2d X_ENImageCoords(col, row);
			Eigen::Vector2d X_EN = PixCoordsToRefCoords(X_ENImageCoords, GSD, num_pixels, NorthBounds, EastBounds);
			Eigen::Vector3d X_ENU(X_EN(0), X_EN(1), 0);

			Eigen::Vector3d X_Cam = R_ENU_Cam * (X_ENU - CamCenter_ENU);
			double point_bearing[3] = { X_Cam(0), X_Cam(1), X_Cam(2) };
			double point_pixel[2];
			world2cam(point_pixel, point_bearing, &o);
			Eigen::Vector2d X_RefImageCoords(point_pixel[1], point_pixel[0]);

			Eigen::Vector2d X_ImageCoords = A*X_RefImageCoords + b;

			if (showFrames)
				circle(frameCopyBGR, cv::Point2d(X_ImageCoords(0), X_ImageCoords(1)), 3, cv::Scalar(255, 0, 0), -1);

			cv::Point2d sample_point(X_ImageCoords(0), X_ImageCoords(1));
			if (ImageRaw.channels() == 1)
				ENImage.at<uint8_t>(row, col) = getColorSubpixHelper_UC1(ImageRaw, sample_point);
			else if (ImageRaw.channels() == 3)
				ENImage.at<cv::Vec3b>(row, col) = getColorSubpixHelper_UC3(ImageRaw, sample_point);
		}
	}

	if (showFrames) {
		cv::imshow("Sample Locations", frameCopyBGR);
		cv::waitKey(1);
	}
}
