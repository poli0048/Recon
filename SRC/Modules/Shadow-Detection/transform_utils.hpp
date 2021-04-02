#pragma once

//System Includes
#include <random>
#include <chrono>

//Return ECEF position corresponding to given lat (rad), lon (rad), and alt (m)
inline cv::Point3d positionLLA2ECEF(double lat, double lon, double alt) {
	double a = 6378137.0;           //Semi-major axis of reference ellipsoid
	double ecc = 0.081819190842621; //First eccentricity of the reference ellipsoid
	double eccSquared = ecc * ecc;
	double N = a / sqrt(1.0 - eccSquared * sin(lat) * sin(lat));
	double X = (N + alt) * cos(lat) * cos(lon);
	double Y = (N + alt) * cos(lat) * sin(lon);
	double Z = (N * (1 - eccSquared) + alt) * sin(lat);
	return(cv::Point3d(X, Y, Z));
}

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
/*
inline void multENU2CAM(Mat& enu, Mat& cam, Eigen::Matrix3d R, Eigen::Vector3d t, ocam_model& o) {
	cam = Mat(enu.rows, 2, CV_64F);
	for (int row = 0; row < enu.rows; row++) {
		Eigen::Vector3d v(enu.at<double>(row, 0), enu.at<double>(row, 1), enu.at<double>(row, 2));
		Eigen::Vector3d v_cam = R.inverse() * (v - t);
		double point3D[3] = { v_cam(0), v_cam(1), v_cam(2) };
		double point2D[2];

		world2cam(point2D, point3D, &o);

		cam.at<double>(row, 0) = point2D[1];
		cam.at<double>(row, 1) = point2D[0];
	}
}
*/

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

	std::cout << "Pose estimated." << std::endl;
}

inline cv::Mat latLon_2_C_ECEF_NED(double lat, double lon) {
	//Compute matrix components
	double C11 = -sin(lat) * cos(lon);
	double C12 = -sin(lat) * sin(lon);
	double C13 = cos(lat);
	double C21 = -sin(lon);
	double C22 = cos(lon);
	double C23 = 0.0;
	double C31 = -cos(lat) * cos(lon);
	double C32 = -cos(lat) * sin(lon);
	double C33 = -sin(lat);

	cv::Mat C_ECEF_NED = (cv::Mat_<double>(3, 3) << C11, C12, C13, C21, C22, C23, C31, C32, C33); // every three is one row

	////Populate C_ECEF_NED
	//Eigen::Matrix3d C_ECEF_NED;
	//C_ECEF_NED(0, 0) = C11; C_ECEF_NED(0, 1) = C12; C_ECEF_NED(0, 2) = C13;
	//C_ECEF_NED(1, 0) = C21; C_ECEF_NED(1, 1) = C22; C_ECEF_NED(1, 2) = C23;
	//C_ECEF_NED(2, 0) = C31; C_ECEF_NED(2, 1) = C32; C_ECEF_NED(2, 2) = C33;

	return(C_ECEF_NED);
}

//Compute latitude (radians), longitude (radians), and altitude (height above WGS84 ref. elipsoid, in meters) from ECEF position (in meters)
//Latitude and longitude are also both given with respect to the WGS84 reference elipsoid.
inline void positionECEF2LLA(cv::Point3d const& Position, double& lat, double& lon, double& alt) {
	double x = Position.x;
	double y = Position.y;
	double z = Position.z;

	//Set constants
	double R_0 = 6378137.0;
	double R_P = 6356752.314;
	double ecc = 0.081819190842621;

	//Calculate longitude (radians)
	lon = atan2(y, x);

	//Compute intermediate values needed for lat and alt
	double eccSquared = ecc * ecc;
	double p = sqrt(x * x + y * y);
	double E = sqrt(R_0 * R_0 - R_P * R_P);
	double F = 54.0 * (R_P * z) * (R_P * z);
	double G = p * p + (1.0 - eccSquared) * z * z - eccSquared * E * E;
	double c = pow(ecc, 4.0) * F * p * p / pow(G, 3.0);
	double s = pow(1.0 + c + sqrt(c * c + 2.0 * c), 1.0 / 3.0);
	double P = (F / (3.0 * G * G)) / ((s + (1.0 / s) + 1.0) * (s + (1.0 / s) + 1.0));
	double Q = sqrt(1.0 + 2.0 * pow(ecc, 4.0) * P);
	double k_1 = -1.0 * P * eccSquared * p / (1.0 + Q);
	double k_2 = 0.5 * R_0 * R_0 * (1.0 + 1.0 / Q);
	double k_3 = -1.0 * P * (1.0 - eccSquared) * z * z / (Q * (1.0 + Q));
	double k_4 = -0.5 * P * p * p;
	double r_0 = k_1 + sqrt(k_2 + k_3 + k_4);
	double k_5 = (p - eccSquared * r_0);
	double U = sqrt(k_5 * k_5 + z * z);
	double V = sqrt(k_5 * k_5 + (1.0 - eccSquared) * z * z);

	double z_0 = (R_P * R_P * z) / (R_0 * V);
	double e_p = (R_0 / R_P) * ecc;

	//Calculate latitude (radians)
	lat = atan((z + z_0 * e_p * e_p) / p);

	//Calculate Altitude (m)
	alt = U * (1.0 - (R_P * R_P / (R_0 * V)));
}

inline cv::Mat latLon_2_C_ECEF_ENU(double lat, double lon) {
	cv::Mat C_ECEF_NED = latLon_2_C_ECEF_NED(lat, lon);
	cv::Mat C_NED_ENU = (cv::Mat_<double>(3, 3) << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0);
	cv::Mat C_ECEF_ENU = C_NED_ENU * C_ECEF_NED;
	return(C_ECEF_ENU);
}

inline void matToMatrix3d(cv::Mat &in, Eigen::Matrix3d &out) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out(i, j) = in.at<double>(i, j);
		}
	}
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


//Convert an LLA vector: <Latitude (radians), Longitude (radians), Altitude (m)> to an ECEF position
inline cv::Point3d positionLLA2ECEF_construct(cv::Point3d const& PositionLLA) {
	return positionLLA2ECEF(PositionLLA.x, PositionLLA.y, PositionLLA.z);
}
inline void poseLEA2ENU(cv::Point3d &centroid_ECEF, Eigen::Matrix3d& R_cam_LEA, Eigen::Vector3d &t_cam_LEA, Eigen::Matrix3d& R_cam_ENU, Eigen::Vector3d& t_cam_ENU) {
	Eigen::Vector3d centroid_vec_ECEF(centroid_ECEF.x, centroid_ECEF.y, centroid_ECEF.z);
	Eigen::Vector3d cam_center_ECEF = t_cam_LEA + centroid_vec_ECEF;
	cv::Point3d cam_center_pt_ECEF(cam_center_ECEF(0), cam_center_ECEF(1), cam_center_ECEF(2));
	cv::Point3d cam_center_LLA = positionECEF2LLA_construct(cam_center_pt_ECEF);
	cv::Point3d centroid_LLA = positionECEF2LLA_construct(centroid_ECEF);
	cv::Point3d origin_enu_LLA = cv::Point3d(cam_center_LLA.x, cam_center_LLA.y, centroid_LLA.z);

	cv::Mat C_ECEF_ENU = latLon_2_C_ECEF_ENU(origin_enu_LLA.x, origin_enu_LLA.y);
	Eigen::Matrix3d C_Matrix3d_ECEF_ENU;
	matToMatrix3d(C_ECEF_ENU, C_Matrix3d_ECEF_ENU);

	cv::Point3d origin_enu_pt_ECEF = positionLLA2ECEF(origin_enu_LLA.x, origin_enu_LLA.y, origin_enu_LLA.z);
	Eigen::Vector3d origin_enu_ECEF = Eigen::Vector3d(origin_enu_pt_ECEF.x, origin_enu_pt_ECEF.y, origin_enu_pt_ECEF.z);

	R_cam_ENU = C_Matrix3d_ECEF_ENU * R_cam_LEA;
	t_cam_ENU = C_Matrix3d_ECEF_ENU * (cam_center_ECEF - origin_enu_ECEF);
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
