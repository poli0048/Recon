//This module provides the main interface for the shadow detection system

//Project Includes
#include "ShadowDetection.hpp"
#include "calib.h"
#include "lambda_twist.h"
#include "transform_utils.hpp"
#include "shadow_utils.hpp"
#include "FRF.h"
#include "ShadowMapIO.hpp"

namespace ShadowDetection {
	//Save the shadow map history to an FRF file with the given path. Return true on success and false on failure
	bool ShadowMapHistory::SaveFRFFile(std::filesystem::path const & Filepath) {
		// Save entire ShadowMapHistory entry, which is a vector maps
		// Generate a Layer with all the correct settings
		FRFImage shadowMap;

		//Set Image dimensions - this must be done now, when there are no layers in the image yet.
		if (!shadowMap.SetWidth(512U))
		std::cerr << "Failed to set image width.\r\n";
		if (!shadowMap.SetHeight(512U))
		std::cerr << "Failed to set image height.\r\n";
		
		ShadowMapInfoBlock myShadowMapInfoBlock;
		myShadowMapInfoBlock.FileTimeEpoch_Week = 0U;
		myShadowMapInfoBlock.FileTimeEpoch_TOW = std::nan("");
		
		// Iterate through the Maps
		for (int i = 0; i < (int) Maps.size(); i++){
			//Add a new layer and set it up
			FRFLayer* newLayer = shadowMap.AddLayer();
			newLayer->Name = std::string("Shadow Map Layer");
			newLayer->Description = std::string("0 = Unshadowed, 1 = Fully shadowed");
			newLayer->UnitsCode = -1; //No units
			newLayer->SetTypeCode(8U); //See Table 1 in the spec. We are going to use 8-bit unsigned integers for each pixel in this layer
			newLayer->HasValidityMask = true; //Add validity info for each pixel in this layer
			newLayer->SetAlphaAndBetaForGivenRange(0.0, 1.0); //Let the FRF lib set coefficients so values are in range [0,1]
			newLayer->AllocateStorage(); //This needs to be called before the layer can be accessed

			set_NewValue((uint) shadowMap.Rows(), (uint) shadowMap.Rows(), newLayer, Maps[i]);

			myShadowMapInfoBlock.LayerTimeTags.push_back(i);
		}

		FRFVisualizationColormap* viz = shadowMap.AddVisualizationColormap();
		viz->LayerIndex = 0U; //Base the visualization on the first layer of the shadow map
		viz->SetPoints.push_back(std::make_tuple(0.0, 1.0, 1.0, 1.0)); //Map value 0 (unshadowed) to white (RGB all set to 1)
		viz->SetPoints.push_back(std::make_tuple(1.0, 0.0, 0.0, 0.0)); //Map value 1 (fully shadowed) to black (RGB all set to 0)

		FRFGeoRegistration GeoRegistrationTag;
		GeoRegistrationTag.Altitude = std::nan(""); 
		GeoRegistrationTag.RegisterFromCornerLocations(UL_LL, UR_LL, LL_LL, LR_LL);
		shadowMap.SetGeoRegistration(GeoRegistrationTag);
		//TODO: When we have the ability, set the GPST field of GeoRegistrationTag

		if (!myShadowMapInfoBlock.AttachToFRFFile(shadowMap))
			std::cerr << "Error adding shadow map info block... do we have the right number of time tags? There should be 1 per layer.\r\n";
		
		if (!shadowMap.SaveToDisk(Filepath)) {
			std::cerr << "ShadowMapHistory::SaveFRFFile(). Error saving ShadowMapHistory\r\n";
			return false;
		}
		else {
			return true;
		}
	}
	
	//A lock is already held on the object when this function is called so don't lock it here
	//On entry we also already know that we have at least 3 fiducials and a non-empty reference frame
	void ShadowDetectionEngine::ProcessFrame(cv::Mat const & Frame, TimePoint const & Timestamp) {
		//Compute new shadow map based on the newly received frame
		cv::Mat H_out, img_rot, img_rot_sampled, binary, binary_sampled, mask, img_masked, binary_masked, frame_sampled;
		cv::Mat frame_orig;
		frame_orig = Frame;

		getOrbRotation(ref_descriptors, keypoints_ref, Frame, H_out);
		
		cv::warpAffine(Frame, img_rot, H_out, m_ReferenceFrame.size());
		getBinaryCloudMask(img_rot, brightest, binary);
		cv::medianBlur(binary, binary, MEDIAN_BLUR_RADIUS);
		
		cv::cvtColor(binary, binary, cv::COLOR_GRAY2RGB);
            
            
		sampleENUSquare(binary, o, R_cam_ENU, t_cam_ENU, center, max_extent, OUTPUT_RESOLUTION_PX, false, binary_sampled);
		sampleENUSquare(img_rot, o, R_cam_ENU, t_cam_ENU, center, max_extent, OUTPUT_RESOLUTION_PX, false, img_rot_sampled);
		sampleENUSquare(frame_orig, o, R_cam_ENU, t_cam_ENU, center, max_extent, OUTPUT_RESOLUTION_PX, false, frame_sampled);
		
		getApertureMask(img_rot_sampled, mask);
		img_rot_sampled.copyTo(img_masked, mask);
		
		cv::cvtColor(binary_sampled, binary_sampled, cv::COLOR_RGB2GRAY);
		binary_masked = binary_sampled;
		imshow("Frame Original", frame_orig);
		imshow("Frame Stabilized", img_rot_sampled);
		create_masked_binary((uint) binary_sampled.cols, (uint) binary_sampled.rows, mask, binary_masked, binary_sampled);

		//Update m_ShadowMap
		m_ShadowMap.Map = binary_masked; //Shadow map based on most recently processed frame
		m_ShadowMap.Timestamp = Timestamp;
			
		//Update m_History (Essentially a vector of ShadowMap Histories)
		m_History.back().Maps.push_back(binary_masked); //Record of all computed shadow maps - add new element when ref frame changes (since registration changes)
		m_History.back().Timestamps.push_back(Timestamp);
		
		//Call any registered callbacks
		for (auto const & kv : m_callbacks)
			kv.second(m_ShadowMap);
	}
	
	//Set the reference frame to be used for registration and stabilization (all other frames are aligned to the reference frame)
	//While we are setting up the reference frame, we will generate brightest, since we will need that for the binary cloud mask
	void ShadowDetectionEngine::SetReferenceFrame(cv::Mat const & RefFrame) {
		std::scoped_lock lock(m_mutex);
		RefFrame.copyTo(m_ReferenceFrame);
		
		std::vector<cv::Mat> hsv_channels;
		cv::Mat hsv;

		cv::cvtColor(m_ReferenceFrame, hsv, cv::COLOR_BGR2HSV);
		cv::split(hsv, hsv_channels);
		Eigen::Vector2d UL_LL, UR_LL, LL_LL, LR_LL;
		brightest = hsv_channels[2];

		getRefDescriptors(m_ReferenceFrame, ref_descriptors, keypoints_ref);	
		
		//set up new element of shadowMapHistory by adding an empty shadowMap to m_History
		ShadowMapHistory new_History;
		m_History.push_back(new_History);

		// Set up points
		Eigen::Vector2d UL(0, 0);
		Eigen::Vector2d UR(OUTPUT_RESOLUTION_PX - 1, 0);
		Eigen::Vector2d LR(OUTPUT_RESOLUTION_PX - 1, OUTPUT_RESOLUTION_PX - 1);
		Eigen::Vector2d LL(0, OUTPUT_RESOLUTION_PX - 1);
		Eigen::Vector3d UL_LLA, LR_LLA, UR_LLA, LL_LLA;

		positionPX2LLA(m_ReferenceFrame, UL, centroid_ECEF, center, max_extent, OUTPUT_RESOLUTION_PX, UL_LLA, R_cam_ENU, t_cam_ENU, o);
		positionPX2LLA(m_ReferenceFrame, LL, centroid_ECEF, center, max_extent, OUTPUT_RESOLUTION_PX, LL_LLA, R_cam_ENU, t_cam_ENU, o);
		positionPX2LLA(m_ReferenceFrame, LR, centroid_ECEF, center, max_extent, OUTPUT_RESOLUTION_PX, LR_LLA, R_cam_ENU, t_cam_ENU, o);
		positionPX2LLA(m_ReferenceFrame, UR, centroid_ECEF, center, max_extent, OUTPUT_RESOLUTION_PX, UR_LLA, R_cam_ENU, t_cam_ENU, o);

		UL_LL << UL_LLA(0), UL_LLA(1);
		UR_LL << UR_LLA(0), UR_LLA(1);
		LL_LL << LL_LLA(0), LL_LLA(1);
		LR_LL << LR_LLA(0), LR_LLA(1);

		m_History.back().UL_LL = UL_LL;
		m_History.back().UR_LL = UR_LL;
		m_History.back().LL_LL = LL_LL;
		m_History.back().LR_LL = LL_LL;
		
		m_ShadowMap.UL_LL = UL_LL;
		m_ShadowMap.UR_LL = UR_LL;
		m_ShadowMap.LL_LL = LL_LL;
		m_ShadowMap.LR_LL = LL_LL;
	}
	
	//Set the fiducials used for registration. Each fiducial is a tuple of the form <PixCoords, LLA> where:
	//PixCoords are the coordinates (in pixels) in the reference image of a fiducial: (col, row) with upper-left corner origin
	//LLA are the WGS84 latitude (radians), longitude (radians), and altitude (m) of the fiducial, in that order.
	//No need to convert to radians since testbench already does so
	void ShadowDetectionEngine::SetFiducials(std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> const & Fiducials) {
		std::scoped_lock lock(m_mutex);
		m_Fiducials = Fiducials;
		int rows = m_Fiducials.size();
		
		std::Evector<Eigen::Vector3d> Fiducials_LEA(rows);
		std::Evector<Eigen::Vector2d> Fiducials_PX(rows);

    		Eigen::Matrix3d R_cam_LEA;
    		Eigen::Vector3d t_cam_LEA;
		// centroid_ECEF, center, max_extent, t_cam_ENU and R_cam_ENU already defined as private variables
		
    		const char* file = CAMERA_MODEL_PATH.c_str();
		
		//getECEFCentroid(LLA, centroid_ECEF)
		Eigen::MatrixXd ECEF_points(rows, 3);
		for (int row = 0; row < rows; row++) {
			//Eigen::Vector3d  LLA = std::get<1>Fiducials[row];
			cv::Point3d ECEF_point = positionLLA2ECEF(std::get<1>(m_Fiducials[row])(0), std::get<1>(m_Fiducials[row])(1), std::get<1>(m_Fiducials[row])(2));
			ECEF_points.row(row) = Eigen::Vector3d(ECEF_point.x, ECEF_point.y, ECEF_point.z);
		}
		Eigen::Vector3d centroid_temp = ECEF_points.colwise().mean();
		centroid_ECEF = cv::Point3d(centroid_temp[0], centroid_temp[1], centroid_temp[2]);
		
		//multLLA2LEA(LLA, LEA, ECEF)
		for (int row = 0; row < rows; row++) {
			cv::Point3d ECEF_point = positionLLA2ECEF(std::get<1>(m_Fiducials[row])(0), std::get<1>(m_Fiducials[row])(1), std::get<1>(m_Fiducials[row])(2));
			cv::Mat LEA_point = cv::Mat(ECEF_point - centroid_ECEF).t();
			//LEA_point.copyTo(Fiducials_LEA[row]);
			Fiducials_LEA[row] << LEA_point.at<double>(0),  LEA_point.at<double>(1), LEA_point.at<double>(2);
			Fiducials_PX[row] << (double)std::get<0>(m_Fiducials[row])(0), (double)std::get<0>(m_Fiducials[row])(1);
			}
		get_ocam_model(&o, file);
		findPose(Fiducials_PX, Fiducials_LEA, o, R_cam_LEA, t_cam_LEA); //ECHAI: Still editing findPose

		poseLEA2ENU(centroid_ECEF, R_cam_LEA, t_cam_LEA, R_cam_ENU, t_cam_ENU);
		std::cout << "The matrix t_cam_ENU  " << t_cam_ENU << std::endl;
		std::cout << "The matrix t_cam_LEA  " << t_cam_LEA << std::endl;
		
    		get_centered_extent(centroid_ECEF, APERTURE_DISTANCE_PX, FINAL_SIZE, R_cam_ENU, t_cam_ENU, t_cam_LEA, o, center, max_extent);
		std::cout << "Max Extent  " << max_extent << std::endl;
		std::cout << "The matrix center  " << center << std::endl;
	}
}




