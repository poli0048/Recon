#pragma once

//System Includes
#include <random>
#include <chrono>

//Project Includes
#include "../../Maps/MapUtils.hpp"

inline void multCam2World(std::Evector<Eigen::Vector2d>& pixel_coords, std::vector<cv::Point3d>& backprojected, ocam_model& o) {
	for (int i = 0; i < (int) pixel_coords.size(); i++) {
		double point3D[3];
		// y,x => row, col => 1, 0
		double point2D[2] = { pixel_coords[i](1), pixel_coords[i](0) };
		cam2world(point3D, point2D, &o);
		backprojected.push_back(cv::Point3d(point3D[0], point3D[1], point3D[2]));
	}
}

inline void sampleMatrix(const std::Evector<Eigen::Vector3d>& src, const int pool_size, const unsigned seed, Eigen::Matrix3d& dst) {
	std::vector<int> sampleIndices;
	for (int i = 0; i < pool_size; i++) {
		sampleIndices.push_back(i);
	}

	std::shuffle(sampleIndices.begin(), sampleIndices.end(), std::default_random_engine(seed));
	
	int rowA = sampleIndices[0];
	int rowB = sampleIndices[1];
	int rowC = sampleIndices[2];
	//dst.col(0) << src.at<double>(rowA, 0), src.at<double>(rowA, 1), src.at<double>(rowA, 2);
	//dst.col(1) << src.at<double>(rowB, 0), src.at<double>(rowB, 1), src.at<double>(rowB, 2);
	//dst.col(2) << src.at<double>(rowC, 0), src.at<double>(rowC, 1), src.at<double>(rowC, 2);

	dst.col(0) << src[rowA](0), src[rowA](1), src[rowA](2);
	dst.col(1) << src[rowB](0), src[rowB](1), src[rowB](2);
	dst.col(2) << src[rowC](0), src[rowC](1), src[rowC](2);
}


inline void getPoseInputMatrices(std::Evector<Eigen::Vector3d>& world, std::vector<cv::Point3d>& backproj, Eigen::Matrix3d& world_vec, Eigen::Matrix3d& bearing_vec, const unsigned seed) {
	//cv::Mat bp_matrix = cv::Mat(backproj);
	// backproj has cv::Point3d objects. Need to convert to Eigen::Vector3d
	std::Evector<Eigen::Vector3d> bp_matrix;
	for (int i = 0; i < (int) backproj.size(); i++){
		Eigen::Vector3d temp_row;
		temp_row << backproj[i].x, backproj[i].y, backproj[i].z;
		bp_matrix.push_back(temp_row);
	}
	sampleMatrix(bp_matrix, bp_matrix.size(), seed, bearing_vec); // ECHAI: remove rows, move to vector<Point3d to keep consistent>
	sampleMatrix(world, world.size(), seed, world_vec); 
}

inline void multENU2CAM(std::Evector<Eigen::Vector3d>& enu, cv::Mat& cam, Eigen::Matrix3d R, Eigen::Vector3d t, ocam_model& o) {
	cam = cv::Mat(enu.size(), 2, CV_64F);
	for (int row = 0; row < (int) enu.size(); row++) {
		Eigen::Vector3d v_cam = R.inverse() * (enu[row] - t);
		double point3D[3] = { v_cam(0), v_cam(1), v_cam(2) };
		double point2D[2];

		world2cam(point2D, point3D, &o);

		cam.at<double>(row, 0) = point2D[1];
		cam.at<double>(row, 1) = point2D[0];
	}
}

inline double reprojectionError(cv::Mat &orig_PX, std::Evector<Eigen::Vector3d>&orig_LEA, Eigen::Matrix3d& R_cam_LEA, Eigen::Vector3d& t_cam_LEA, ocam_model& o, double percent_include) {
	cv::Mat reprojected, norm;
	multENU2CAM(orig_LEA, reprojected, R_cam_LEA, t_cam_LEA, o);
	cv::Mat diff = orig_PX - reprojected;
	reduce(diff.mul(diff), norm, 1, cv::REDUCE_SUM, CV_64F);
	cv::sort(norm, norm, cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);
	int end = percent_include * norm.rows;
	cv::Mat subMatrix = cv::Mat(norm, cv::Range(0, end), cv::Range::all());
	return cv::sum(subMatrix)[0];
}

inline void findPose(std::Evector<Eigen::Vector2d>& fiducials_PX, std::Evector<Eigen::Vector3d>&fiducials_LEA, ocam_model &o, Eigen::Matrix3d &R_cam_LEA, Eigen::Vector3d &t_cam_LEA) {
	std::vector<cv::Point3d> fiducials_BACKPROJ;
	Eigen::Matrix3d input_world, input_bearing;
	std::Evector<std::tuple<Eigen::Vector3d, Eigen::Matrix3d>> possiblePoses;
	cv::Mat f_px;
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	double best_error = -1;
	std::tuple<Eigen::Vector3d, Eigen::Matrix3d> best_pose;

	multCam2World(fiducials_PX, fiducials_BACKPROJ, o); 
	f_px = cv::Mat(fiducials_PX.size(), 2, CV_64F, fiducials_PX.data());
	
	
	for (int i = 0; i < 5000; i++) {
		getPoseInputMatrices(fiducials_LEA, fiducials_BACKPROJ, input_world, input_bearing, seed + i);//ECHAI: I have edited up to here
		
		LambdaTwistSolve(input_bearing, input_world, possiblePoses, true);
		
		for (int poseIndex = 0; poseIndex < (int) possiblePoses.size(); poseIndex++) {
			std::tuple<Eigen::Vector3d, Eigen::Matrix3d> pose = possiblePoses[poseIndex];
			R_cam_LEA = std::get<1>(pose);
			t_cam_LEA = std::get<0>(pose);
			
			double reproj_error = reprojectionError(f_px, fiducials_LEA, R_cam_LEA, t_cam_LEA, o, 1);
			if (reproj_error < best_error || best_error == -1) {
				best_error = reproj_error;
				best_pose = pose;

				//cout << i << ", " << best_error << endl;
			}
		}
		
	}
	R_cam_LEA = std::get<1>(best_pose);
	t_cam_LEA = std::get<0>(best_pose);

	//cout << "\nBest error: " << best_error << endl;
	//std::cout << "Pose estimated." << std::endl;
}

//Compute latitude (radians), longitude (radians), and altitude (height above WGS84 ref. elipsoid, in meters) from ECEF position (in meters)
//Latitude and longitude are also both given with respect to the WGS84 reference elipsoid.
//This is a wrapper for the function in MapUtils, but accepting openCV structures for input
inline void positionECEF2LLA(cv::Point3d const & Position, double & lat, double & lon, double & alt) {
	Eigen::Vector3d Position_ECEF(Position.x, Position.y, Position.z);
	Eigen::Vector3d Position_LLA = ECEF2LLA(Position_ECEF);
	lat = Position_LLA(0);
	lon = Position_LLA(1);
	alt = Position_LLA(2);
}

inline cv::Point3d positionECEF2LLA_construct(cv::Point3d const& PositionECEF) {
	double lat = 0.0;
	double lon = 0.0;
	double alt = 0.0;
	positionECEF2LLA(PositionECEF, lat, lon, alt);
	return cv::Point3d(lat, lon, alt);
}

//(col, row)-pixel coords to reference coords
inline Eigen::Vector2d PixCoordsToRefCoords(Eigen::Vector2d const& PixCoords, double GSD, double N, const Eigen::Vector2d& NorthBounds, const Eigen::Vector2d& EastBounds) {
	return(Eigen::Vector2d(GSD * PixCoords(0) + EastBounds(0), GSD * (double(N - 1) - PixCoords(1)) + NorthBounds(0)));
}

inline void poseLEA2ENU(cv::Point3d &centroid_ECEF, Eigen::Matrix3d& R_cam_LEA, Eigen::Vector3d &t_cam_LEA, Eigen::Matrix3d& R_cam_ENU, Eigen::Vector3d& t_cam_ENU) {
	Eigen::Vector3d centroid_vec_ECEF(centroid_ECEF.x, centroid_ECEF.y, centroid_ECEF.z);
	Eigen::Vector3d cam_center_ECEF = t_cam_LEA + centroid_vec_ECEF;
	cv::Point3d cam_center_pt_ECEF(cam_center_ECEF(0), cam_center_ECEF(1), cam_center_ECEF(2));
	cv::Point3d cam_center_LLA = positionECEF2LLA_construct(cam_center_pt_ECEF);
	cv::Point3d centroid_LLA = positionECEF2LLA_construct(centroid_ECEF);
	cv::Point3d origin_enu_LLA = cv::Point3d(cam_center_LLA.x, cam_center_LLA.y, centroid_LLA.z);

	Eigen::Matrix3d C_ECEF_ENU = latLon_2_C_ECEF_ENU(origin_enu_LLA.x, origin_enu_LLA.y);
	Eigen::Vector3d origin_enu_ECEF = LLA2ECEF(Eigen::Vector3d(origin_enu_LLA.x, origin_enu_LLA.y, origin_enu_LLA.z));

	R_cam_ENU = C_ECEF_ENU * R_cam_LEA;
	t_cam_ENU = C_ECEF_ENU * (cam_center_ECEF - origin_enu_ECEF);
}


inline void get_centered_extent(const cv::Point3d &centroid_ECEF, const double &aperture_distance_px, const cv::Size &FINAL_SIZE, const Eigen::Matrix3d &R_cam_ENU, const Eigen::Vector3d &t_cam_ENU, const Eigen::Vector3d& t_cam_LEA, ocam_model &o, Eigen::Vector2d& center, double &max_extent) {
	double lat, lon, alt, theta;
	positionECEF2LLA(centroid_ECEF, lat, lon, alt);

	Eigen::Vector3d centroid_vec_ECEF(centroid_ECEF.x, centroid_ECEF.y, centroid_ECEF.z);
	Eigen::Vector3d cam_center_ECEF = t_cam_LEA + centroid_vec_ECEF;
	cv::Point3d cam_center_pt_ECEF(cam_center_ECEF(0), cam_center_ECEF(1), cam_center_ECEF(2));
	cv::Point3d cam_center_LLA = positionECEF2LLA_construct(cam_center_pt_ECEF);

	double aperture3D[3];
	double aperture2D[2] = { (double)FINAL_SIZE.height / 2 + aperture_distance_px,  (double)FINAL_SIZE.width / 2 };
	cam2world(aperture3D, aperture2D, &o);
	theta = acos(-aperture3D[2]);

	Eigen::Vector3d OA_ENU = R_cam_ENU * Eigen::Vector3d(0, 0, 1); // Optical Axis in the Camera Frame

	Eigen::AngleAxisd AA1(theta, Eigen::Vector3d(1, 0, 0));
	Eigen::AngleAxisd AA2(theta, Eigen::Vector3d(-1, 0, 0));
	Eigen::AngleAxisd AA3(theta, Eigen::Vector3d(0, 1, 0));
	Eigen::AngleAxisd AA4(theta, Eigen::Vector3d(0, -1, 0));
	Eigen::Vector3d v1 = AA1.toRotationMatrix() * OA_ENU;
	Eigen::Vector3d v2 = AA2.toRotationMatrix() * OA_ENU;
	Eigen::Vector3d v3 = AA3.toRotationMatrix() * OA_ENU;
	Eigen::Vector3d v4 = AA4.toRotationMatrix() * OA_ENU;

	double t1 = -t_cam_ENU(2) / v1(2);
	double t2 = -t_cam_ENU(2) / v2(2);
	double t3 = -t_cam_ENU(2) / v3(2);
	double t4 = -t_cam_ENU(2) / v4(2);

	Eigen::Vector3d X1_ENU = t_cam_ENU + v1 * t1;
	Eigen::Vector3d X2_ENU = t_cam_ENU + v2 * t2;
	Eigen::Vector3d X3_ENU = t_cam_ENU + v3 * t3;
	Eigen::Vector3d X4_ENU = t_cam_ENU + v4 * t4;

	double eastMin = cv::min({ X1_ENU(0), X2_ENU(0), X3_ENU(0), X4_ENU(0) });
	double eastMax = cv::max({ X1_ENU(0), X2_ENU(0), X3_ENU(0), X4_ENU(0) });
	double northMin = cv::min({ X1_ENU(1), X2_ENU(1), X3_ENU(1), X4_ENU(1) });
	double northMax = cv::max({ X1_ENU(1), X2_ENU(1), X3_ENU(1), X4_ENU(1) });

	max_extent = cv::max({ eastMax - eastMin, northMax - northMin });
	center = Eigen::Vector2d(0.5 * (eastMin + eastMax), 0.5 * (northMin + northMax));

}

inline void positionPX2LLA(cv::Mat& frame, Eigen::Vector2d& pixel_coords, const cv::Point3d& ECEF_origin, const Eigen::Vector2d &center, const double &max_extent, const double &num_pixels, Eigen::Vector3d& point_LLA, Eigen::Matrix3d& R, Eigen::Vector3d& t, ocam_model& o) {
	double lat, lon, alt;

	Eigen::Vector2d NorthBounds(center(1) - max_extent / 2, center(1) + max_extent / 2);
	Eigen::Vector2d EastBounds(center(0) - max_extent / 2, center(0) + max_extent / 2);
	double GSD = max_extent / num_pixels;

	Eigen::Vector2d ref_coords = PixCoordsToRefCoords(pixel_coords, GSD, num_pixels, NorthBounds, EastBounds);

	Eigen::Vector3d pixel_enu(ref_coords(0), ref_coords(1), 0);
	Eigen::Vector3d ECEF_vec_origin(ECEF_origin.x, ECEF_origin.y, ECEF_origin.z);
	Eigen::Vector3d pixel_ECEF = pixel_enu + ECEF_vec_origin;
	cv::Point3d pixel_point_ECEF(pixel_ECEF(0), pixel_ECEF(1), pixel_ECEF(2));
	positionECEF2LLA(pixel_point_ECEF, lat, lon, alt);

	point_LLA = Eigen::Vector3d(lat, lon, alt);

	Eigen::Vector3d pixel_cam = R.inverse() * (pixel_enu - t);
	double point_bearing[3] = { pixel_cam(0), pixel_cam(1), pixel_cam(2) };
	double point_pixel[2];

	world2cam(point_pixel, point_bearing, &o);

	circle(frame, cv::Point2d(point_pixel[1], point_pixel[0]), 3, cv::Scalar(255, 0, 0), -1);
}

inline void sampleENUSquare(cv::Mat& inputFrame, ocam_model& o, Eigen::Matrix3d& R, Eigen::Vector3d& t, const Eigen::Vector2d& center, const double& max_extent, const double &num_pixels, bool showFrames, cv::Mat& outputFrame) {
	Eigen::Vector2d NorthBounds(center(1) - max_extent / 2, center(1) + max_extent / 2);
	Eigen::Vector2d EastBounds(center(0) - max_extent / 2, center(0) + max_extent / 2);
	double GSD = max_extent / num_pixels;

	outputFrame = cv::Mat(num_pixels, num_pixels, CV_8UC3);

	cv::Mat frameCopy;
	if (showFrames) {
		inputFrame.copyTo(frameCopy);
	}

	for (int row = 0; row < num_pixels; row++) {
		for (int col = 0; col < num_pixels; col++) {
			Eigen::Vector2d pixel_coords(col, row);
			Eigen::Vector2d ref_coords = PixCoordsToRefCoords(pixel_coords, GSD, num_pixels, NorthBounds, EastBounds);

			Eigen::Vector3d pixel_enu(ref_coords(0), ref_coords(1), 0);

			Eigen::Vector3d pixel_cam = R.inverse() * (pixel_enu - t);
			double point_bearing[3] = { pixel_cam(0), pixel_cam(1), pixel_cam(2) };
			double point_pixel[2];

			world2cam(point_pixel, point_bearing, &o);

			if (showFrames) {
				circle(frameCopy, cv::Point2d(point_pixel[1], point_pixel[0]), 3, cv::Scalar(255, 0, 0), -1);
			}

			cv::Point2d sample_point(point_pixel[1], point_pixel[0]);
			outputFrame.at<cv::Vec3b>(row, col) = getColorSubpixHelper(inputFrame, sample_point);
		}
	}

}
