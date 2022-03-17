//This module provides the main interface for the shadow detection system

//Project Includes
#include "ShadowDetection.hpp"
#include "calib.h"
#include "lambda_twist.h"
#include "transform_utils.hpp"
#include "shadow_utils.hpp"
#include "FRF.h"
#include "ShadowMapIO.hpp"

#define PI 3.14159265358979

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
			//set_NewValue((uint) shadowMap.Rows(), (uint) shadowMap.Rows(), newLayer, Maps[i]);

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
		std::cout << "Geo Reg Finished, Saving FRF" << std::endl;
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
		/*cv::Mat PreWarp(2, 3, CV_64F);
		PreWarp.at<double>(0,0) = std::cos(25.0*PI/180.0);
		PreWarp.at<double>(0,1) = -1.0*std::sin(25.0*PI/180.0);
		PreWarp.at<double>(0,2) = 0.0;
		PreWarp.at<double>(1,0) = std::sin(25.0*PI/180.0);
		PreWarp.at<double>(1,1) = std::cos(25.0*PI/180.0);
		PreWarp.at<double>(1,2) = -250;
		cv::warpAffine(Frame, Frame, PreWarp, m_ReferenceFrame.size());*/


		//Note: It seems that a significant factor in the relatively poor stabilisation quality is that since we are
		//using a fisheye camera, both affine transformations and homographies are not sufficient to properly characterize
		//the mapping between keypoints in the uncorrected images. Both assume perspective cameras, which a fisheye isn't.
		//I had hoped that over small changes in camera pose it might still work OK, but I don't think so at this point.

		//A traditional approach would be to "undistort" the imagery and then stabilize it, but this is only practical
		//for a modest amount of fisheye distortion. This is because "undistorting" gives the image that would have been
		//observed by a true perspective camera seeing roughly the same scene. For a hemispherical camera, there is no 
		//corresponding perspective camera that could possible see the same content.

		//A new approach may be needed here. One option is to compute rays to world points associated with each keypoint
		//and then use RBA to estimate the relative pose of the camera in the current frame to the reference frame camera.
		//This can be used to transfer the camera pose for the reference camera to get the current camera pose.
		//We could use this to register the current frame to the EN plane. This, in theory, does not rely on any assumption
		//of recti-linearity and should work with virtually any camera model.

		//For now, probably the best thing to do is use the simplest RT model (and restrict to small translations)
		//and pair this with image blurring to hide the remaining instability

		//We also need to do something smarter than keeping track of the all-time brightest value at each ground
		//point since this is sensitive to any (even sinlge-frame) stabilization problems. Maybe we should keep a
		//running history of the values at each pixel over the past N epochs and use the 90'th percentile value
		//as the "unshadowed" value for that pixel. It seems this wouldn't be super efficient.

		//It would be better perhaps to do shadow detection in the EN plane instead of in the image plane.
		//The resolution is much lower so this would save a lot of time (and save us from wasting processing time
		//on parts of the image that are outside the active area)

		//Steps:
		//In SetReferenceFrame(), we need to map the reference frame to the EN plane along with the reference frame aperture mask
		//Initialize an EN-frame data structure for holding per-pixel value history
		//
		//1 - Get "stabilization matrix" that mostly aligns the frame with the reference frame
		//2 - Go directly to the EN plane by composing transformations. Resample the frame and the aperture mask
		//3 - Compute an EN mask by taking the pixelwise min of the EN ref frame aperture mask and the EN current frame aperture mask.
		//4 - Build a reference value EN image from the value history data structure
		//5 - Smooth both the reference value image and the current-frame value image
		//6 - Compare the current EN-frame value image to the reference EN-frame value image to get a diff image
		//7 - Threshold the diff image to get an EN-frame instantaneous shadow map

		//TODO: The current implementation uses a "diff image" and identifies any point where the absolute intensity
		//drop exceeds a set threshold as shadowed. This isn't quite right... we should be looking at the ratio of
		//the current intensity to the reference intensity and setting threshold there. This is since shadowing
		//doesn't drop all intensities equally... it drops the illumination, which still gets multiplied by surface
		//reflectance to get intensity. The current scheme is more sensitive in areas of high albedo.

		bool newMethod = true;
		cv::Mat shadowMap_EN;
		if (newMethod) {
			cv::Mat H;
			GetStabilizationMatrix(ref_descriptors, keypoints_ref, Frame, m_apertureMask, H, m_ReferenceFrame);

			//Using the pose of the reference camera and the stabilization matrix, map to the EN plane
			cv::Mat frame_EN, frameApertureMask_EN;
			RawImageToENImage(Frame, o, R_Cam_ENU, CamCenter_ENU, center, max_extent, OUTPUT_RESOLUTION_PX, false, frame_EN, H);
			RawImageToENImage(m_apertureMask, o, R_Cam_ENU, CamCenter_ENU, center, max_extent, OUTPUT_RESOLUTION_PX, false, frameApertureMask_EN, H);
			cv::threshold(frameApertureMask_EN, frameApertureMask_EN, 220.0, 255.0, cv::THRESH_BINARY);

			//Compute the current "brightness" image in the EN plane
			cv::Mat frameGray_EN, currentBrightness_EN;
			cv::cvtColor(frame_EN, frameGray_EN, cv::COLOR_BGR2GRAY);
			//int kernelWidth = 15;
			//double sigma = 0.3*((double(kernelWidth) - 1.0)*0.5 - 1) + 0.8;
			//sigma *= 4.0;
			//cv::GaussianBlur(frameGray_EN, currentBrightness_EN, cv::Size(kernelWidth,kernelWidth), sigma, sigma, cv::BORDER_REPLICATE);
			MAFilter(frameGray_EN, frameApertureMask_EN, currentBrightness_EN, 15);
			//currentBrightness_EN.setTo(0, frameApertureMask_EN < 128);
			
			//Compute a mask that is >0 only for pixels that are visible in both the reference frame and the current frame
			cv::Mat mask_EN;
			cv::min(frameApertureMask_EN, m_refFrameApertureMask_EN, mask_EN);
			//cv::threshold(mask_EN, mask_EN, 220.0, 255.0, cv::THRESH_BINARY);



			//Update the brightness history for all unmasked pixels
			auto T0 = std::chrono::steady_clock::now();
			size_t maxHistoryLength = 120U;
			for (int row = 0; row < currentBrightness_EN.rows; row++) {
				for (int col = 0; col < currentBrightness_EN.cols; col++) {
					if (mask_EN.at<uint8_t>(row, col) >= (uint8_t) 128) {
						m_brightnessHist_EN[row][col].push_back(currentBrightness_EN.at<uint8_t>(row, col));
						if (m_brightnessHist_EN[row][col].size() > maxHistoryLength)
							m_brightnessHist_EN[row][col].pop_front();
					}
				}
			}
			auto T1 = std::chrono::steady_clock::now();

			//Compute the reference brightness image
			cv::Mat refBrightness_EN(currentBrightness_EN.rows, currentBrightness_EN.cols, CV_8UC1, cv::Scalar(0));
			for (int row = 0; row < refBrightness_EN.rows; row++) {
				for (int col = 0; col < refBrightness_EN.cols; col++) {
					if (mask_EN.at<uint8_t>(row, col) >= (uint8_t) 128) {
						std::deque<uint8_t> hist = m_brightnessHist_EN[row][col]; //Get a copy of the history to partially sort
						if (hist.empty())
							std::cerr << "Internal error: Shouldn't have empty history for any unmasked pixels.\r\n";

						//Get the 90'th percentile brightness value for this pixel
						int index = (int) std::round(0.9*double(hist.size() - 1U));
						index = std::clamp(index, 0, int(hist.size() - 1U));
						std::nth_element(hist.begin(), hist.begin() + index, hist.end());
						refBrightness_EN.at<uint8_t>(row,col) = hist[index];
					}
				}
			}
			auto T2 = std::chrono::steady_clock::now();

			bool printProcessingTimes = false;
			if (printProcessingTimes) {
				double structUpdateTime   = SecondsElapsed(T0, T1);
				double refImageEvalTime   = SecondsElapsed(T1, T2);
				double fullProcessingTime = SecondsElapsed(T0, T2);
				std::cerr << "Hist struct update time -: " << 1000.0*structUpdateTime << " ms.\r\n";
				std::cerr << "Ref image eval time -----: " << 1000.0*refImageEvalTime << " ms.\r\n";
				std::cerr << "Full time ---------------: " << 1000.0*fullProcessingTime << " ms.\r\n\r\n";
			}

			bool showImages = false;
			if (showImages) {
				imshow("Registered Frame", frame_EN);
				cv::waitKey(1);
				//imshow("Registered Mask", frameApertureMask_EN);
				//cv::waitKey(1);
				
				//imshow("Registered Ref Brightness Image", m_refBrightness_EN);
				//cv::waitKey(1);
				//imshow("Registered Ref Frame Mask", m_refFrameApertureMask_EN);
				//cv::waitKey(1);

				cv::Mat currentBrightness_vis;
				cv::cvtColor(currentBrightness_EN, currentBrightness_vis, cv::COLOR_GRAY2BGR);
				currentBrightness_vis.setTo(0, mask_EN < 128);

				std::vector<std::vector<cv::Point>> contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::findContours(frameApertureMask_EN, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
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
					cv::drawContours(currentBrightness_vis, contours, largestContourIndex, cv::Scalar(255,0,0), 1);
				}
				contours.clear();
				hierarchy.clear();
				cv::findContours(m_refFrameApertureMask_EN, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
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
					cv::drawContours(currentBrightness_vis, contours, largestContourIndex, cv::Scalar(0,0,255), 1);
				}

				imshow("Current Brightness", currentBrightness_vis);
				cv::waitKey(1);

				imshow("Ref Brightness", refBrightness_EN);
				cv::waitKey(1);
			}

			//Update reference brightness image
			/*for (int row = 0; row < m_refBrightness_EN.rows; row++) {
				for (int col = 0; col < m_refBrightness_EN.cols; col++) {
					if (mask_EN.at<uint8_t>(row,col) > 0) {
						uint8_t refBrightness = m_refBrightness_EN.at<uint8_t>(row,col);
						uint8_t currentBrightness = currentBrightness_EN.at<uint8_t>(row,col);
						m_refBrightness_EN.at<uint8_t>(row,col) = std::max(refBrightness, currentBrightness);
					}
				}
			}*/

			//Find the pixelwise brightness over the reference brightness, saturating at 1
			cv::Mat relBrightness_EN;
			cv::divide(currentBrightness_EN, refBrightness_EN, relBrightness_EN, 255);
			//cv::addWeighted(refBrightness_EN, -1.0, currentBrightness_EN, 1.0, 255.0, relBrightness_EN);
			

			//Fade in from the periphery of the visible area
			cv::Mat erodedMask1_EN, erodedMask2_EN, erodedMask3_EN;
			cv::erode(mask_EN, erodedMask1_EN, cv::Mat(), cv::Point(-1,-1), 2);
			cv::erode(erodedMask1_EN, erodedMask2_EN, cv::Mat(), cv::Point(-1,-1), 3);
			cv::erode(erodedMask2_EN, erodedMask3_EN, cv::Mat(), cv::Point(-1,-1), 3);
			cv::Mat ring1, ring2, ring3;
			cv::subtract(mask_EN, erodedMask1_EN, ring1);
			cv::subtract(erodedMask1_EN, erodedMask2_EN, ring2);
			cv::subtract(erodedMask2_EN, erodedMask3_EN, ring3);

			cv::Mat multiplierMat;
			cv::addWeighted(ring1, 1.0, ring2, 0.9, 0.0, multiplierMat);
			cv::addWeighted(multiplierMat, 1.0, ring3, 0.8, 0.0, multiplierMat);
			cv::addWeighted(multiplierMat, 1.0, erodedMask3_EN, 0.7, 0.0, multiplierMat);

			cv::multiply(relBrightness_EN, multiplierMat, relBrightness_EN, 1.0/178.0);
			relBrightness_EN.setTo(255, mask_EN < 128);

			/*cv::Mat ringVis(mask_EN.rows, mask_EN.cols, CV_8UC3, cv::Scalar(0,0,0));
			ringVis.setTo(cv::Scalar(255,0,0), ring1 > 128);
			ringVis.setTo(cv::Scalar(0,255,0), ring2 > 128);
			ringVis.setTo(cv::Scalar(0,0,255), ring3 > 128);
			cv::imshow("Rings", multiplierMat);
			cv::waitKey(1);*/





			//Look for dark areas in the rel brightness image using multiple thresholds.
			//We allow an small area to be detected if the brightness reduction is substantial,
			//but require larger continuous area of reduced brightness for smaller reductions in brightness
			{
				std::vector<std::vector<cv::Point>> contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::Mat threshImage;
				
				double threshold = 178.0;  //30% reduction in brightness
				double minArea   = 200.0; //Approximately 0.1% of visible pixels
				cv::threshold(relBrightness_EN, threshImage, threshold, 255.0, cv::THRESH_BINARY_INV);
				cv::Mat shadowImage1(relBrightness_EN.rows, relBrightness_EN.cols, CV_8UC1, cv::Scalar(0));
				cv::findContours(threshImage, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
				for (int n = 0; n < (int) contours.size(); n++) {
					if ((hierarchy[n][3] < 0) && (cv::contourArea(contours[n]) > minArea)) {
						cv::drawContours(shadowImage1, contours, n, cv::Scalar(254), -1);
						int childIndex = hierarchy[n][2];
						while (childIndex >= 0) {
							if (cv::contourArea(contours[childIndex]) > minArea/5.0)
								cv::drawContours(shadowImage1, contours, childIndex, cv::Scalar(0), -1);
							childIndex = hierarchy[childIndex][0];
						}
					}
				}

				threshold = 204.0;  //20% reduction in brightness
				//minArea   = 2071.0; //Approximately 1% of visible pixels
				minArea   = 200.0; //Approximately 0.1% of visible pixels
				contours.clear();
				hierarchy.clear();
				cv::threshold(relBrightness_EN, threshImage, threshold, 255.0, cv::THRESH_BINARY_INV);
				cv::Mat shadowImage2(relBrightness_EN.rows, relBrightness_EN.cols, CV_8UC1, cv::Scalar(0));
				cv::findContours(threshImage, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
				for (int n = 0; n < (int) contours.size(); n++) {
					if ((hierarchy[n][3] < 0) && (cv::contourArea(contours[n]) > minArea)) {
						cv::drawContours(shadowImage2, contours, n, cv::Scalar(254), -1);
						int childIndex = hierarchy[n][2];
						while (childIndex >= 0) {
							if (cv::contourArea(contours[childIndex]) > minArea/5.0)
								cv::drawContours(shadowImage2, contours, childIndex, cv::Scalar(0), -1);
							childIndex = hierarchy[childIndex][0];
						}
					}
				}

				threshold = 217.0;  //15% reduction in brightness
				//minArea   = 4142.0; //Approximately 2% of visible pixels
				minArea   = 200.0; //Approximately 0.1% of visible pixels
				contours.clear();
				hierarchy.clear();
				cv::threshold(relBrightness_EN, threshImage, threshold, 255.0, cv::THRESH_BINARY_INV);
				cv::Mat shadowImage3(relBrightness_EN.rows, relBrightness_EN.cols, CV_8UC1, cv::Scalar(0));
				cv::findContours(threshImage, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
				for (int n = 0; n < (int) contours.size(); n++) {
					if ((hierarchy[n][3] < 0) && (cv::contourArea(contours[n]) > minArea)) {
						cv::drawContours(shadowImage3, contours, n, cv::Scalar(254), -1);
						int childIndex = hierarchy[n][2];
						while (childIndex >= 0) {
							if (cv::contourArea(contours[childIndex]) > minArea/5.0)
								cv::drawContours(shadowImage3, contours, childIndex, cv::Scalar(0), -1);
							childIndex = hierarchy[childIndex][0];
						}
					}
				}				

				
				//imshow("Temp", threshImage);
				//cv::waitKey(1);

				/*cv::Mat vis(relBrightness_EN.rows, relBrightness_EN.cols, CV_8UC3, cv::Scalar(0,0,0));
				vis.setTo(cv::Scalar(255,0,0), shadowImage3 > 0);
				vis.setTo(cv::Scalar(0,255,0), shadowImage2 > 0);
				vis.setTo(cv::Scalar(0,0,255), shadowImage1 > 0);
				imshow("Shadows", vis);
				cv::waitKey(1);*/
				

				cv::Mat temp(relBrightness_EN.rows, relBrightness_EN.cols, CV_8UC1);
				cv::max(shadowImage1, shadowImage2, temp);
				cv::max(temp, shadowImage3, shadowMap_EN);
				shadowMap_EN.setTo(255, mask_EN < 128);
			}


			//Threshold the relative brightness image
			//cv::threshold(relBrightness_EN, shadowMap_EN, 200.0, 254.0, cv::THRESH_BINARY_INV);
			//shadowMap_EN.setTo(255, mask_EN < 128);

			
			//imshow("Rel Brightness", relBrightness_EN);
			//cv::waitKey(1);




		}
		else {
			//Compute new shadow map based on the newly received frame
			cv::Mat H;
			//H = cv::Mat::eye(2, 3, CV_64F);
			GetStabilizationMatrix(ref_descriptors, keypoints_ref, Frame, m_apertureMask, H, m_ReferenceFrame);

			cv::Mat FrameStabilized, ApertureMaskStabilized;
			if (H.rows == 2) {
				cv::warpAffine(Frame, FrameStabilized, H, m_ReferenceFrame.size());
				cv::warpAffine(m_apertureMask, ApertureMaskStabilized, H, m_apertureMask.size());
			}
			else if (H.rows == 3) {
				cv::warpPerspective(Frame, FrameStabilized, H, m_ReferenceFrame.size());
				cv::warpPerspective(m_apertureMask, ApertureMaskStabilized, H, m_apertureMask.size());
			}
			else
				std::cerr << "Internal Error: Stabilization matrix has invalid dims.\r\n";

			//Build a mask of pixels in the ref image plane that are valid in both the ref image and the current frame
			cv::Mat mask;
			cv::min(m_apertureMask, ApertureMaskStabilized, mask);

			//Update "brightest" to keep a running record of the brightest value associated with each point on the ground (only unmasked pixels)
			cv::Mat hsv, valueImage, diffImage;
			std::vector<cv::Mat> hsv_channels;
			cv::cvtColor(Frame, hsv, cv::COLOR_BGR2HSV);
			cv::split(hsv, hsv_channels);
			int kernelWidth = 15;
			double sigma = 0.3*((double(kernelWidth) - 1.0)*0.5 - 1) + 0.8;
			sigma *= 4.0;
			cv::GaussianBlur(hsv_channels[2], valueImage, cv::Size(kernelWidth,kernelWidth), sigma, sigma, cv::BORDER_REPLICATE);
			//valueImage = hsv_channels[2];
			for (int row = 0; row < brightest.rows; row++) {
				for (int col = 0; col < brightest.cols; col++) {
					if (mask.at<uint8_t>(row,col) > 0)
						brightest.at<uint8_t>(row,col) = std::max(brightest.at<uint8_t>(row,col), valueImage.at<uint8_t>(row,col));
				}
			}

			//Compute difference between "brightest" and current value image (defaulting to 0 for masked pixels)
			diffImage = cv::Scalar(0);
			cv::subtract(brightest, valueImage, diffImage, mask);

			// Anything with too small of a difference is floored to zero (Avoids segmentation when there is no segmentation to be done). 
			cv::Mat binaryShadowMapImagePlane;
			cv::threshold(diffImage, binaryShadowMapImagePlane, 20, 255, cv::THRESH_BINARY);

			// Dynamic thresholding to binary by minimizing within-class variance 
			//cv::threshold(binaryShadowMapImagePlane, binaryShadowMapImagePlane, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

			//Clean up output with median blur
			cv::medianBlur(binaryShadowMapImagePlane, binaryShadowMapImagePlane, MEDIAN_BLUR_RADIUS);

			//Build registered shadow map and convert mask image to sentinal value
			cv::Mat mask_EN;
			sampleENUSquare(binaryShadowMapImagePlane, o, R_Cam_ENU, CamCenter_ENU, center, max_extent, OUTPUT_RESOLUTION_PX, false, shadowMap_EN);
			sampleENUSquare(mask, o, R_Cam_ENU, CamCenter_ENU, center, max_extent, OUTPUT_RESOLUTION_PX, false, mask_EN);

			//Apply the mask as a sentinal value in the shadow map image
			for (int row = 0; row < shadowMap_EN.rows; row++) {
				for (int col = 0; col < shadowMap_EN.cols; col++) {
					if (shadowMap_EN.at<uint8_t>(row,col) == 255U)
						shadowMap_EN.at<uint8_t>(row,col) = 254;
					if (mask_EN.at<uint8_t>(row,col) == 0)
						shadowMap_EN.at<uint8_t>(row,col) = 255;
				}
			}

			bool show_StabilizedImagery = false;
			if (show_StabilizedImagery) {
				imshow("Stabilized Frame", FrameStabilized);
				cv::waitKey(1);
			}

			bool show_RegisteredImagery = true;
			if (show_RegisteredImagery) {
				cv::Mat frame_EN;
				sampleENUSquare(FrameStabilized, o, R_Cam_ENU, CamCenter_ENU, center, max_extent, OUTPUT_RESOLUTION_PX, false, frame_EN);
				imshow("Registered Frame", frame_EN);
				cv::waitKey(1);
			}

			bool show_DiffImage = false;
			if (show_DiffImage) {
				imshow("Diff Image", diffImage);
				cv::waitKey(1);
			}

			bool show_ImPlaneShadowMap = false;
			if (show_ImPlaneShadowMap) {
				imshow("binaryShadowMapImagePlane", binaryShadowMapImagePlane);
				cv::waitKey(1);
			}
		}

		//Update m_ShadowMap
		std::scoped_lock lock(m_mutex); //Lock now after processing but before saving and executing callbacks
		m_ShadowMap.Map = shadowMap_EN; //Shadow map based on most recently processed frame
		m_ShadowMap.Timestamp = Timestamp;

		//Update m_History (Essentially a vector of ShadowMap Histories)
		m_History.back().Maps.push_back(shadowMap_EN); //Record of all computed shadow maps - add new element when ref frame changes (since registration changes)
		m_History.back().Timestamps.push_back(Timestamp);

		//Call any registered callbacks
		for (auto const & kv : m_callbacks)
			kv.second(m_ShadowMap);
	}
	
	//Set the reference frame to be used for registration and stabilization (all other frames are aligned to the reference frame)
	//While we are setting up the reference frame, we will generate brightest, since we will need that for the binary cloud mask
	void ShadowDetectionEngine::SetReferenceFrame(cv::Mat const & RefFrame) {
		if ((RefFrame.rows == 0) || (RefFrame.cols == 0)) {
			std::cerr << "Error in ShadowDetectionEngine::SetReferenceFrame(): Given ref frame is empty.\r\n";
			return;
		}
		
		std::scoped_lock lock(m_mutex);
		if (m_running) {
			std::cerr << "Error in ShadowDetectionEngine::SetReferenceFrame(): Module is currently running.\r\n";
			return;
		}
		
		//Save the reference frame and compute the aperture mask
		RefFrame.copyTo(m_ReferenceFrame);
		getApertureMask(RefFrame, m_apertureMask);

		//Compute reference keypoints and descriptors
		GetKeypointsAndDescriptors(RefFrame, keypoints_ref, ref_descriptors, m_apertureMask);

		//Initialize "brightest" to the value channel of the reference frame
		cv::Mat hsv;
		cv::cvtColor(m_ReferenceFrame, hsv, cv::COLOR_BGR2HSV);
		std::vector<cv::Mat> hsv_channels;
		cv::split(hsv, hsv_channels);
		int kernelWidth = 15;
		double sigma = 0.3*((double(kernelWidth) - 1.0)*0.5 - 1) + 0.8;
		sigma *= 4.0;
		cv::GaussianBlur(hsv_channels[2], brightest, cv::Size(kernelWidth,kernelWidth), sigma, sigma, cv::BORDER_REPLICATE);

		//getRefDescriptors(m_ReferenceFrame, ref_descriptors, keypoints_ref);
		
		TryInitShadowMapAndHistory();
	}
	
	//Set the fiducials used for registration. Each fiducial is a tuple of the form <PixCoords, LLA> where:
	//PixCoords are the coordinates (in pixels) in the reference image of a fiducial: (col, row) with upper-left corner origin
	//LLA are the WGS84 latitude (radians), longitude (radians), and altitude (m) of the fiducial, in that order.
	void ShadowDetectionEngine::SetFiducials(std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> const & Fiducials) {
		if (Fiducials.size() < 3U) {
			std::cerr << "Error in ShadowDetectionEngine::SetFiducials(): Fewer than 3 fiducials provided.\r\n";
			return;
		}
		
		std::scoped_lock lock(m_mutex);
		if (m_running) {
			std::cerr << "Error in ShadowDetectionEngine::SetFiducials(): Module is currently running.\r\n";
			return;
		}
		
		m_Fiducials = Fiducials;
		int rows = m_Fiducials.size();
		
		std::Evector<Eigen::Vector3d> Fiducials_LEA(rows);
		std::Evector<Eigen::Vector2d> Fiducials_PX(rows);

    		Eigen::Matrix3d R_Cam_LEA;
    		Eigen::Vector3d CamCenter_LEA;
		// LEAOrigin_ECEF, center, max_extent, CamCenter_ENU and R_Cam_ENU already defined as private variables
		
    		const char* file = CAMERA_MODEL_PATH.c_str();
		
		//Compute the centroid of the fiducials in ECEF - this is a point somewhere near the middle of the scene and
		//it serves as the origin for the LEA frame, which is aligned with ECEF.
		Eigen::MatrixXd ECEF_points(rows, 3);
		for (int row = 0; row < rows; row++)
			ECEF_points.row(row) = LLA2ECEF(std::get<1>(m_Fiducials[row]));
		LEAOrigin_ECEF = ECEF_points.colwise().mean();
		
		for (int row = 0; row < rows; row++) {
			Eigen::Vector3d fiducial_ECEF = ECEF_points.row(row);
			Fiducials_LEA[row] = fiducial_ECEF - LEAOrigin_ECEF;
			Fiducials_PX[row]  = std::get<0>(m_Fiducials[row]);
		}
		get_ocam_model(&o, file);

		findPose(Fiducials_PX, Fiducials_LEA, o, R_Cam_LEA, CamCenter_LEA);

		poseLEA2ENU(LEAOrigin_ECEF, R_Cam_LEA, CamCenter_LEA, R_Cam_ENU, CamCenter_ENU, ENUOrigin_ECEF);

    		get_centered_extent(LEAOrigin_ECEF, APERTURE_DISTANCE_PX, FINAL_SIZE, R_Cam_ENU, CamCenter_ENU, CamCenter_LEA, o, center, max_extent);
    		
    		TryInitShadowMapAndHistory();
	}
	
	//Sets the corner coords in both m_ShadowMap and m_History if GCPs and a ref frame are set
	//Also performs additional initialization that can only happen once fiducials and the reference frame have been set
	void ShadowDetectionEngine::TryInitShadowMapAndHistory(void) {
		if ((! m_ReferenceFrame.empty()) && (! m_Fiducials.empty())) {
			cv::Mat refFrame_EN, refFrameGray_EN;
			RawImageToENImage(m_ReferenceFrame, o, R_Cam_ENU, CamCenter_ENU, center, max_extent, OUTPUT_RESOLUTION_PX, false, refFrame_EN);
			RawImageToENImage(m_apertureMask, o, R_Cam_ENU, CamCenter_ENU, center, max_extent, OUTPUT_RESOLUTION_PX, false, m_refFrameApertureMask_EN);
			cv::threshold(m_refFrameApertureMask_EN, m_refFrameApertureMask_EN, 220.0, 255.0, cv::THRESH_BINARY);
			
			cv::cvtColor(refFrame_EN, refFrameGray_EN, cv::COLOR_BGR2GRAY);
			//int kernelWidth = 15;
			//double sigma = 0.3*((double(kernelWidth) - 1.0)*0.5 - 1) + 0.8;
			//sigma *= 4.0;
			//cv::GaussianBlur(refFrameGray_EN, m_refBrightness_EN, cv::Size(kernelWidth,kernelWidth), sigma, sigma, cv::BORDER_REPLICATE);
			
			//cv::imshow("Unmasked Ref Brightness", m_refBrightness_EN);
			//cv::waitKey(1);

			MAFilter(refFrameGray_EN, m_refFrameApertureMask_EN, m_refBrightness_EN, 15);
			m_refBrightness_EN.setTo(0, m_refFrameApertureMask_EN < 128);

			m_brightnessHist_EN.resize(m_refBrightness_EN.rows, std::vector<std::deque<uint8_t>>(m_refBrightness_EN.cols));
			for (int row = 0; row < m_refBrightness_EN.rows; row++) {
				for (int col = 0; col < m_refBrightness_EN.cols; col++) {
					if (m_refFrameApertureMask_EN.at<uint8_t>(row, col) > (uint8_t) 0)
						m_brightnessHist_EN[row][col].push_back(m_refBrightness_EN.at<uint8_t>(row, col));
				}
			}

			Eigen::Vector2d UL(0, 0);
			Eigen::Vector2d UR(OUTPUT_RESOLUTION_PX - 1, 0);
			Eigen::Vector2d LR(OUTPUT_RESOLUTION_PX - 1, OUTPUT_RESOLUTION_PX - 1);
			Eigen::Vector2d LL(0, OUTPUT_RESOLUTION_PX - 1);
			Eigen::Vector3d UL_LLA, LR_LLA, UR_LLA, LL_LLA;
			
			Eigen::Vector3d ENUOrigin_LLA = ECEF2LLA(ENUOrigin_ECEF);
			Eigen::Matrix3d C_ECEF_ENU = latLon_2_C_ECEF_ENU(ENUOrigin_LLA(0), ENUOrigin_LLA(1));

			positionPX2LLA(m_ReferenceFrame, UL, center, max_extent, OUTPUT_RESOLUTION_PX, UL_LLA, R_Cam_ENU, CamCenter_ENU, o, ENUOrigin_ECEF, C_ECEF_ENU);
			positionPX2LLA(m_ReferenceFrame, LL, center, max_extent, OUTPUT_RESOLUTION_PX, LL_LLA, R_Cam_ENU, CamCenter_ENU, o, ENUOrigin_ECEF, C_ECEF_ENU);
			positionPX2LLA(m_ReferenceFrame, LR, center, max_extent, OUTPUT_RESOLUTION_PX, LR_LLA, R_Cam_ENU, CamCenter_ENU, o, ENUOrigin_ECEF, C_ECEF_ENU);
			positionPX2LLA(m_ReferenceFrame, UR, center, max_extent, OUTPUT_RESOLUTION_PX, UR_LLA, R_Cam_ENU, CamCenter_ENU, o, ENUOrigin_ECEF, C_ECEF_ENU);
			
			m_ShadowMap.UL_LL << UL_LLA(0), UL_LLA(1);
			m_ShadowMap.UR_LL << UR_LLA(0), UR_LLA(1);
			m_ShadowMap.LL_LL << LL_LLA(0), LL_LLA(1);
			m_ShadowMap.LR_LL << LR_LLA(0), LR_LLA(1);
			
			//set up new element of shadowMapHistory and set the corners
			m_History.emplace_back();
			m_History.back().UL_LL << UL_LLA(0), UL_LLA(1);
			m_History.back().UR_LL << UR_LLA(0), UR_LLA(1);
			m_History.back().LL_LL << LL_LLA(0), LL_LLA(1);
			m_History.back().LR_LL << LR_LLA(0), LR_LLA(1);
		}
	}
}




