//Shadow propagation module implementation. The shadow propagation module takes instantaneous shadow maps and
//predicts their evolution into the future to compute time available maps that indicate how long each point
//visible in a shadow map will be available for imaging before being impacted by cloud shadows.
//
//Authors: karthik Dharmarajan, Bryan Poling

//External Includes
#include "torch/torch.h"
#include "torch/script.h"
#include "torch/cuda.h"
#include "torch/version.h"
#include <opencv2/imgproc.hpp>

//Project Includes
#include "ShadowPropagation.hpp"
#include "../../Utilities.hpp"
#include "../../Polygon.hpp"

static void CheckForBadTensorValues(torch::Tensor const & T) {
	int NaNCount = 0;
	int negativeCount = 0;
	int over1Count = 0;
	
	for (int n = 0; n < 64; n++) {
		for (int m = 0; m < 64; m++) {
			float val = T[0][0][n][m].item<float>();
			if (std::isnan(val)) {
				//std::cerr << "Element (0,0," << n << "," << m << ") is NaN.\r\n";
				NaNCount++;
			}
			else if (val < 0.0f) {
				//std::cerr << "Element (0,0," << n << "," << m << ") is negative.\r\n";
				negativeCount++;
			}
			else if (val > 1.0f) {
				//std::cerr << "Element (0,0," << n << "," << m << ") exceeds 1.0\r\n";
				over1Count++;
			}
		}
	}
	if ((NaNCount == 0) && (negativeCount == 0) && (over1Count == 0))
		std::cerr << "All elements are OK.\r\n";
	else
		std::cerr << "Tensor contains " << NaNCount << " NaNs, " << negativeCount << " negative vals, " << over1Count << " vals over 1.\r\n";
}

static torch::Tensor EigenMatrixToTensor(Eigen::MatrixXf const & M, torch::Device const & Dev) {
	auto options = torch::TensorOptions().dtype(torch::kFloat32).layout(torch::kStrided).device(torch::kCPU);
	torch::Tensor T = torch::empty({1, 1, 64, 64}, options);
	
	auto accessor = T.accessor<float,4>();
	for (int n = 0; n < 64; n++) {
		for (int m = 0; m < 64; m++)
			accessor[0][0][n][m] = M(n,m);
	}
	return T.to(Dev);
}

//Create a 64x64 floating point shadow map matrix where each pixel is in the range 0-1 from an instantanious shadow map object.
//A value of 0 corresponds to fully unshadowed land and a value of 1 corresponds to fully shadowed land.
//Areas that are masked off are treated as unshadowed and get value 0.
//This function uses Karthik's solution where the mask is re-computed using contours. Technically this shouldn't be necessary
//since the shadow map contains the mask data (using a sentinal value), but this can mitigate the fringe artifacts that sometimes
//appear right along the aperture boundary due to imperfect video stabilization.
//Typical runtime is about 0.1 - 0.2 ms on i7-1165G7 CPU @ 2.80GHz
static Eigen::MatrixXf ShadowMapIntToFloat_UsingContours(ShadowDetection::InstantaneousShadowMap const & ShadowMap) {
	cv::Mat downsizedMap;
	cv::resize(ShadowMap.Map, downsizedMap, cv::Size(64, 64), cv::INTER_AREA);
	// Model was trained using 0-1 float values instead of 0-255, so scale and convert to floating point Mat
	cv::Mat cvInputWithExcess;
	downsizedMap.convertTo(cvInputWithExcess, CV_32FC1, 1.f/255.0);
	// Given ShadowMap has a white area on the outside of what the fisheye lens sees
	// The LTSM will interpret this white as a large cloud on the border of the image (which is incorrect)
	// Solution is to use a mask to "black off" the outside area

	// Creating mask
	cv::Mat mask = cv::Mat::zeros(cv::Size(64, 64), CV_8UC1);

	// Creates inverted shadow map
	cv::Mat invertedShadowMap;
	cv::threshold(downsizedMap, invertedShadowMap, 127, 255, cv::THRESH_BINARY_INV);
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	// Get contours on inverted shadow map 
	cv::findContours(invertedShadowMap, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	cv::Point2f center;
	float radius;
	// Pick out the largest contour if one is available, and select a minimum enclosing circle around it
	// This enclosing circle represents the area on the shadow map that was seen by the fisheye lens
	if (contours.size() > 0) {
		std::vector<cv::Point>& largestContour = contours[0];
		double largestContourArea = cv::contourArea(largestContour);
		for (int i = 0; i < (int) contours.size(); i++) {
			if (cv::contourArea(contours[i]) > largestContourArea) {
				largestContour = contours[i];
				largestContourArea = cv::contourArea(largestContour);
			}
		}
		cv::minEnclosingCircle(largestContour, center, radius);
	}
	// In the case that there are no contours or the estimated area that the fisheye lens sees is too small, use some hardcoded default value
	// This case generally occurs when the shadow covers all of the area the fisheye sees, so the shadow map is completely white (inverse is completely black) 
	if (contours.size() <= 0 || radius < 29) {
		// Default circle + radius estimation so things don't fall apart
		radius = 32;
		center = cv::Point2f(33.7, 33.7);
	}
	// Add estimated circle to mask, and then mask the 64x64 shadow map
	cv::circle(mask, center, radius - 3, cv::Scalar(255), -1, 8, 0);
	cv::Mat cvInput;
	cvInputWithExcess.copyTo(cvInput, mask);

	// Convert cv::Mat to Eigen::MatrixXf for storage in m_inputHist
	Eigen::MatrixXf shadowMapMatrix(64, 64);
	for (int n = 0; n < 64; n++) {
		for (int m = 0; m < 64; m++) {
			shadowMapMatrix(n,m) = cvInput.at<float>(n,m);
		}
	}
	return shadowMapMatrix;
}

//Create a 64x64 floating point shadow map matrix where each pixel is in the range 0-1 from an instantanious shadow map object.
//A value of 0 corresponds to fully unshadowed land and a value of 1 corresponds to fully shadowed land.
//Areas that are masked off are treated as unshadowed and get value 0.
//This function uses block averaging over the central 4x4 pixel block in the source image corresponding to each pixel in the target image.
//This is a mix of full block averaging and sub-sampling, which gives some anti-aliasing but also very good performance.
//Typical runtime is about 0.03 - 0.05 ms on i7-1165G7 CPU @ 2.80GHz
static Eigen::MatrixXf ShadowMapIntToFloat_UsingMask(ShadowDetection::InstantaneousShadowMap const & ShadowMap) {
	Eigen::MatrixXf shadowMapMatrix = Eigen::MatrixXf::Zero(64, 64);
	for (int targetRow = 0; targetRow < 64; targetRow++) {
		for (int targetCol = 0; targetCol < 64; targetCol++) {
			//Average middle block of pixels (fast but with some amount of anti-aliasing)
			float sum = 0.0f;
			int count = 0;
			for (int sourceRow = 8*targetRow+3; sourceRow <= 8*targetRow+4; sourceRow++) {
				for (int sourceCol = 8*targetCol+3; sourceCol <= 8*targetCol+4; sourceCol++) {
					uint8_t intVal = ShadowMap.Map.at<uint8_t>(sourceRow, sourceCol);
					float floatVal = float(intVal) / 254.0;
					if (intVal < uint8_t(255)) {
						//Non-masked pixel
						sum += floatVal;
						count++;
					}
				}
			}
			if (count > 0)
				shadowMapMatrix(targetRow,targetCol) = sum / float(count);

			//Full block-averaging... slower but very good anti-aliasing
			/*float sum = 0.0f;
			int count = 0;
			for (int sourceRow = 8*targetRow; sourceRow < 8*(targetRow+1); sourceRow++) {
				for (int sourceCol = 8*targetCol; sourceCol < 8*(targetCol+1); sourceCol++) {
					uint8_t intVal = ShadowMap.Map.at<uint8_t>(sourceRow, sourceCol);
					float floatVal = float(intVal) / 254.0;
					if (intVal < uint8_t(255)) {
						//Non-masked pixel
						sum += floatVal;
						count++;
					}
				}
			}
			if (count > 0)
				shadowMapMatrix(targetRow,targetCol) = sum / float(count);*/
		}
	}
	return shadowMapMatrix;
}

//Show a shadow map matrix after it has been converted to a 64x64 float matrix. This is a development function and should
//not normally be called. This is really to compare different conversion methods.
static void ShowShadowMapFloatMatrix(Eigen::MatrixXf const & ShadowMapMatrix, std::string const & WindowName) {
	if ((ShadowMapMatrix.rows() != 64) || (ShadowMapMatrix.cols() != 64)) {
		std::cerr << "Error in ShowShadowMapFloatMatrix: Input matrix is not 64x64.\r\n";;
		return;
	}

	//cv::Mat SM_8UC1(64, 64, CV_8UC1);
	cv::Mat SM_2X_8UC1(128, 128, CV_8UC1);
	for (int row = 0; row < ShadowMapMatrix.rows(); row++) {
		for (int col = 0; col < ShadowMapMatrix.cols(); col++) {
			uint8_t intval = (uint8_t) std::round(ShadowMapMatrix(row,col)*255.0f);
			//SM_8UC1.at<uint8_t>(row, col) = intval;
			SM_2X_8UC1.at<uint8_t>(2*row,   2*col  ) = intval;
			SM_2X_8UC1.at<uint8_t>(2*row,   2*col+1) = intval;
			SM_2X_8UC1.at<uint8_t>(2*row+1, 2*col  ) = intval;
			SM_2X_8UC1.at<uint8_t>(2*row+1, 2*col+1) = intval;
		}
	}
	//cv::imshow(WindowName, SM_8UC1);
	cv::imshow(WindowName, SM_2X_8UC1);
	cv::waitKey(1);
}

//Take a predicted shadow map, a given number of epochs in the future and update an epochs available map
static void UpdateEAMap(cv::Mat & EA, torch::Tensor const & CurrentPredictionTensor, uint16_t EpochsInFuture, float DetThreshold) {
	if ((EA.rows != 64) || (EA.cols != 64)) {
		std::cerr << "Error in UpdateEAMap: EA (Epochs Available) map has wrong dimensions.\r\n";
		return;
	}
	for (int i = 0; i < 64; i++) {
		for (int j = 0; j < 64; j++) {
			float predictedVal = CurrentPredictionTensor[0][0][i][j].item<float>();
			uint16_t currentEA = EA.at<uint16_t>(i, j);
			if ((predictedVal > DetThreshold) && (EpochsInFuture < currentEA))
				EA.at<uint16_t>(i, j) = EpochsInFuture;
		}
	}
}

/*static cv::Mat EAMapToTAMap(cv::Mat const & EA, double SecondsPerEpoch) {
	uint16_t maxUint16Val = std::numeric_limits<uint16_t>::max();
	cv::Mat TA(EA.size(), CV_16UC1, cv::Scalar(maxUint16Val));
	for (int row = 0; row < EA.rows; row++) {
		for (int col = 0; col < EA.cols; col++) {
			uint16_t epochs = EA.at<uint16_t>(row,col);
			if (epochs < maxUint16Val)
				TA.at<uint16_t>(row,col) = (uint16_t) std::min(std::round(double(epochs)*SecondsPerEpoch), double(maxUint16Val));
		}
	}
	return TA;
}*/

//Take an Epochs available map as input and output a Time available map, scaled up by 8X in each dimension
static cv::Mat EAMapToTAMap(cv::Mat const & EA, double SecondsPerEpoch) {
	uint16_t maxUint16Val = std::numeric_limits<uint16_t>::max();

	//Initially build an unmasked TA image and a mask image at the base resolution level
	cv::Mat TA_BaseRes_unmasked(EA.size(), CV_16UC1, cv::Scalar(0));
	cv::Mat Mask_BaseRes(EA.size(), CV_8UC1, cv::Scalar(0)); //0 for unmasked, 255 for masked
	uint16_t maxUnmaskedTAVal = 0;
	for (int row = 0; row < EA.rows; row++) {
		for (int col = 0; col < EA.cols; col++) {
			uint16_t epochs = EA.at<uint16_t>(row,col);
			if (epochs < maxUint16Val) {
				TA_BaseRes_unmasked.at<uint16_t>(row,col) = (uint16_t) std::min(std::round(double(epochs)*SecondsPerEpoch), double(maxUint16Val));
				maxUnmaskedTAVal = std::max(maxUnmaskedTAVal, TA_BaseRes_unmasked.at<uint16_t>(row,col));
			}
			else
				Mask_BaseRes.at<uint8_t>(row,col) = 255;
		}
	}
	for (int row = 0; row < EA.rows; row++) {
		for (int col = 0; col < EA.cols; col++) {
			uint16_t epochs = EA.at<uint16_t>(row,col);
			if (epochs == maxUint16Val)
				TA_BaseRes_unmasked.at<uint16_t>(row,col) = maxUnmaskedTAVal;
		}
	}

	cv::Mat TA_Upscaled;
	//cv::resize(TA_BaseRes_unmasked, TA_Upscaled, cv::Size(8*EA.cols, 8*EA.rows), cv::INTER_LINEAR);
	cv::resize(TA_BaseRes_unmasked, TA_Upscaled, cv::Size(8*EA.cols, 8*EA.rows), cv::INTER_CUBIC);
	//TA_Upscaled is unmasked at this point

	cv::Mat Mask_Upscaled;
	//cv::resize(Mask_BaseRes, Mask_Upscaled, cv::Size(8*EA.cols, 8*EA.rows), cv::INTER_LINEAR);
	cv::resize(Mask_BaseRes, Mask_Upscaled, cv::Size(8*EA.cols, 8*EA.rows), cv::INTER_CUBIC);

	for (int row = 0; row < TA_Upscaled.rows; row++) {
		for (int col = 0; col < TA_Upscaled.cols; col++) {
			if (Mask_Upscaled.at<uint8_t>(row,col) >= 128)
				TA_Upscaled.at<uint16_t>(row,col) = maxUint16Val;
		}
	}

	return TA_Upscaled;
}

namespace ShadowPropagation {
	void ShadowPropagationEngine::ModuleMain_LSTM(void) {
		const     int   TARGET_INPUT_LENGTH = 10;   //Number of history epochs for bootstrapping LSTM
		const     int   TIME_HORIZON        = 10;   //Number of epochs (not necessarily seconds) to predict into future
		constexpr float OUTPUT_THRESHOLD    = 0.4f; //Min float value in prediction to be interpreted as shadowing

		//LibTorch Setup - if CUDA is available we will use CUDA... otherwise fallback to CPU evaluation
		torch::Device torchDevice(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);

		torch::jit::script::Module torchModule; // TorchScript Model
		if (torchDevice.is_cuda()) {
			std::cerr << "LibTorch device: CUDA\r\n";
			torchModule = torch::jit::load(Handy::Paths::ThisExecutableDirectory().parent_path().string().append(
				                          "/SRC/Modules/Shadow-Propagation/model_cuda.pt"), torchDevice);
		}
		else {
			std::cerr << "LibTorch device: CPU\r\n";
			torchModule = torch::jit::load(Handy::Paths::ThisExecutableDirectory().parent_path().string().append(
				                          "/SRC/Modules/Shadow-Propagation/model.pt"), torchDevice);
			at::set_num_interop_threads(1); //As far as I can tell, both inter and intra-op parallelism actually slow down
			at::set_num_threads(1);         //model evaluation while using more CPU. Turn off all LibTorch SMP
		}
		torchModule.eval();

		std::Edeque<Eigen::MatrixXf> inputHist_maps; //Stores previous shadow maps in the form of Eigen::MatrixXf
		std::deque<TimePoint> inputHist_timestamps;  //History of timestamps for recently received shadow

		bool initNeeded = true; //When true, we need to clear our history and re-initialize internal state
		while (! m_abort) {
			m_mutex.lock();
			if (! m_running) {
				m_mutex.unlock();
				initNeeded = true; //Next time through the loop we need to re-initialize
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}

			if (m_unprocessedShadowMaps.empty()) {
				//There are no unprocessed shadow maps todeal with
				m_mutex.unlock();
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}

			//If we get here, we have unprocessed shadow maps waiting for us
			
			//If this is the first received shadow map since (re)-starting, clear internal state data
			if (initNeeded) {
				inputHist_maps.clear();
				inputHist_timestamps.clear();
				initNeeded = false;
			}
			
			//Grab the first unprocessed shadow map and remove it from the dequeue
			ShadowDetection::InstantaneousShadowMap map = m_unprocessedShadowMaps.front();
			m_unprocessedShadowMaps.pop_front();

			bool behindRealtime = !m_unprocessedShadowMaps.empty();
			if (behindRealtime) {
				std::cerr << "Warning: Shadow Propagation module falling behind real-time. ";
				std::cerr << (unsigned int) m_unprocessedShadowMaps.size() << " frames buffered.\r\n"; 
			}

			//Now unlock m_mutex and proceed with model evaluation - lock m_mutex later when updating protected fields
			m_mutex.unlock();

			//Convert the instantaneous shadow map from a 512x512 integer image with sentinal mask value to a 64x64 float
			//matrix with "masked" pixels treated as unshadowed. 0 corresponds to no shadow and 1 corresponds to full shadow.
			//This is the input format that the NN was trained on and knows how to handle.
			//auto t1 = std::chrono::steady_clock::now();
			Eigen::MatrixXf shadowMapMatrix = ShadowMapIntToFloat_UsingMask(map);
			//Eigen::MatrixXf shadowMapMatrix = ShadowMapIntToFloat_UsingContours(map);
			//auto t2 = std::chrono::steady_clock::now();
			//std::cerr << "Shadow map conversion time: " << 1000.0*SecondsElapsed(t1, t2) << " ms\r\n";
			//ShowShadowMapFloatMatrix(shadowMapMatrix, "ShadowMapMatrix"s);

			//Save the converted shadow map to our history buffer
			inputHist_maps.push_back(shadowMapMatrix);
			inputHist_timestamps.push_back(map.Timestamp);
			if (inputHist_maps.size() > TARGET_INPUT_LENGTH) {
				inputHist_maps.pop_front();
				inputHist_timestamps.pop_front();
			}

			if (behindRealtime) {
				std::cerr << "Warning: Shadow Propagation Module skipping update because we are lagging real-time.\r\n";
				continue;
			}

			if (inputHist_maps.size() < TARGET_INPUT_LENGTH) {
				//Not enough shadow maps to bootstrap the LSTM yet
				continue;
			}
			
			//If we get here, we want to evaluate the LSTM and compute a new TA function
			
			//Bootstrap the LSTM by feeding in our history of shadow maps - all but the most recent map
			for (int i = 0; i+1 < (int) inputHist_maps.size(); i++) {
				// The LSTM takes in 3 values for inputs: torch::Tensor tensor, bool firstTimestep, bool decoding
				// "tensor" is the input tensor to use for bootstrapping or prediction
				// "firstTimestep" is a bool indicating whether an input is the first in a sequence (inits the models internal states)
				// "decoding" is not used for anything in the model code (always set to false)
				std::vector<torch::jit::IValue> inputs;
				torch::Tensor histInput = EigenMatrixToTensor(inputHist_maps[i], torchDevice);
				inputs.push_back(histInput);
				inputs.push_back((i == 0));
				inputs.push_back(false);

				torchModule.forward(inputs);
			}

			//Grab the most recent shadow map and initialize a local (64x64) Epochs Available (EA) function
			//The epochs available function counts epochs (instead of seconds) before expected shadowing
			cv::Mat EpochsAvailable(cv::Size(64, 64), CV_16UC1, cv::Scalar(std::numeric_limits<uint16_t>::max()));
			torch::Tensor currentPredictionTensor = EigenMatrixToTensor(inputHist_maps.back(), torchDevice);
			UpdateEAMap(EpochsAvailable, currentPredictionTensor, uint16_t(0), OUTPUT_THRESHOLD);

			for (int epoch = 1; epoch <= TIME_HORIZON; epoch++) {
				//Propogate the current prediction a single epoch into the future and update EA function
				std::vector<torch::jit::IValue> inputs;
				inputs.push_back(currentPredictionTensor);
				inputs.push_back(false);
				inputs.push_back(false);

				// The output from torchModule.forward() is really a tuple with multiple tensors:
				// [decoder_input, decoder_hidden, output_image, _, _]
				// For this, the only tensor that matters is the output_image, as it is the prediction generated by the LSTM
				currentPredictionTensor = torchModule.forward(inputs).toTensor().clone().detach();
				UpdateEAMap(EpochsAvailable, currentPredictionTensor, uint16_t(epoch), OUTPUT_THRESHOLD);
			}

			//Build TA map from EA map
			double historyDuration = SecondsElapsed(inputHist_timestamps.front(), inputHist_timestamps.back());
			double secondsPerEpoch = historyDuration / double(inputHist_timestamps.size() - 1);
			//cv::Mat TimeAvailable64x64 = EAMapToTAMap(EpochsAvailable, secondsPerEpoch);
			cv::Mat TimeAvailable512x512 = EAMapToTAMap(EpochsAvailable, secondsPerEpoch);

			{
				std::scoped_lock lock(m_mutex);
				//Scale up localTimeAvailable from 64x64 to 512x512 and copy registration and timestamp from map
				//TODO - This won't cut it... using bilinear interp on this map is problematic since we use a sentinal
				//value to represent locations that are free for the full time horizon. Using interpolation here
				//results in pixels near sentinal-valued pixels getting interpolated to very large but non-sentinal values.
				//This isn't right... we should probably do sentinal-aware interpolation.
				//cv::resize(TimeAvailable64x64, m_TimeAvail.TimeAvailable, cv::Size(512, 512), cv::INTER_LINEAR);

				m_TimeAvail.TimeAvailable = TimeAvailable512x512;
				m_TimeAvail.UL_LL = map.UL_LL;
				m_TimeAvail.UR_LL = map.UR_LL;
				m_TimeAvail.LL_LL = map.LL_LL;
				m_TimeAvail.LR_LL = map.LR_LL;
				m_TimeAvail.Timestamp = map.Timestamp;

				for (auto const & kv : m_callbacks)
					kv.second(m_TimeAvail);
			}
		}
	}

	//Take a vector of cv::Point objects and populate the given simple polygon using it. 
	//N controls thinning behavior... we take every N'th point when making the simple polygon
	//Returns true on success and false if the source contour is degenerate (fewer than 3 points) or
	//if it is very small and should be ignored.
	static bool CVContourToSimplePolygon(std::vector<cv::Point> const & Contour, SimplePolygon & SPoly, int N = 1) {
		if (Contour.size() < 3U)
			return false;
		N = std::clamp(N, 1, int(Contour.size())/3);
		
		std::Evector<Eigen::Vector2d> points;
		points.reserve(int(Contour.size())/N + 3);
		for (int index = 0; index < (int) Contour.size(); index += N)
			points.push_back(Eigen::Vector2d(Contour[index].x, Contour[index].y));

		SPoly.SetBoundary(points);

		//Return success if the simple ploygon has significant area - this ignores tiny shadows
		//We just use a hard-coded limit of 10 pixels... anything smaller than this gets culled.
		//if (SPoly.GetArea() <= 10.0)
		//	std::cerr << "Culling small contour. Area: " << SPoly.GetArea() << "\r\n";
		return (SPoly.GetArea() > 10.0);
	}

	//For a given point X find the closest boundary point in the collection of provided shadows. If shadows is empty, we return false.
	//Otherwise we will find a closest boundary point and return true. When successful we pass back the location of the
	//point we found, along with it's location in the input shadow data structure: <polyIndex, simplePolyIndex, vertexIndex>
	//There is an important subtlety here: the output index will refer to an existing vertex in the input Shadows data structure,
	//but the output ClosestBdryPoint may not coincide with an actual vertex of any simple polygon... this is because we
	//find the closest point on the boundary of a shadow... which may be on a segment between vertices.
	static bool FindClosestBoundaryPoint(Eigen::Vector2d const & X, std::Evector<Polygon> const & Shadows, int & PolyIndex,
		                                int & SimplePolyIndex, int & VertexIndex, Eigen::Vector2d & ClosestBdryPoint) {
		double closestPointDist = std::nan("");
		for (int polyIndex = 0; polyIndex < (int) Shadows.size(); polyIndex++) {
			Polygon const & poly(Shadows[polyIndex]);
			for (int simplePolyIndex = -1; simplePolyIndex < (int) poly.m_holes.size(); simplePolyIndex++) {
				SimplePolygon const & simplePoly(simplePolyIndex < 0 ? poly.m_boundary : poly.m_holes[simplePolyIndex]);
				
				size_t closestVertexIndex = 0U;
				Eigen::Vector2d projection = simplePoly.ProjectPointToBoundary(X, closestVertexIndex);
				double projectionDist = (X - projection).norm();

				if (std::isnan(closestPointDist) || (projectionDist < closestPointDist)) {
					PolyIndex        = polyIndex;
					SimplePolyIndex  = simplePolyIndex;
					VertexIndex      = (int) closestVertexIndex;
					ClosestBdryPoint = projection;
					closestPointDist = projectionDist;
				}
			}
		}
		return (! std::isnan(closestPointDist));
	}

	//Render the given shadows to an image - leaves untouched pixels that are not in shadow
	static void PaintShadows_UC16(cv::Mat & TargetImage, std::Eunordered_map<std::tuple<int,int>, SimplePolygon> const & ShadowSimplePolys,
		                         uint16_t ShadowValue) {
		//Get a unique value to use for unshadowed regions
		//uint16_t noShadowValue = (ShadowValue == uint16_t(0)) ? uint16_t(1) : uint16_t(0);

		cv::Mat myShadows(TargetImage.rows, TargetImage.cols, CV_8UC1);
		myShadows.setTo(0);

		//First paint all boundary polygons
		for (auto const & kv : ShadowSimplePolys) {
			//int polyIndex = std::get<0>(kv.first);
			int simplePolyIndex = std::get<1>(kv.first);
			if (simplePolyIndex < 0) {
				//This is a boundary polygon
				std::vector<std::vector<cv::Point>> tempContours;
				tempContours.emplace_back();
				std::Evector<Eigen::Vector2d> const & vertices(kv.second.GetVertices());
				tempContours.back().reserve(vertices.size());
				for (Eigen::Vector2d const & v : vertices)
					tempContours.back().push_back(cv::Point(v(0), v(1)));
				cv::drawContours(myShadows, tempContours, 0, cv::Scalar(1), -1);
			}
		}

		//Now paint all hole polygons
		for (auto const & kv : ShadowSimplePolys) {
			//int polyIndex = std::get<0>(kv.first);
			int simplePolyIndex = std::get<1>(kv.first);
			if (simplePolyIndex >= 0) {
				//This is a hole polygon
				std::vector<std::vector<cv::Point>> tempContours;
				tempContours.emplace_back();
				std::Evector<Eigen::Vector2d> const & vertices(kv.second.GetVertices());
				tempContours.back().reserve(vertices.size());
				for (Eigen::Vector2d const & v : vertices)
					tempContours.back().push_back(cv::Point(v(0), v(1)));
				cv::drawContours(myShadows, tempContours, 0, cv::Scalar(0), -1);
			}
		}

		//Paint our shadows onto the target image
		TargetImage.setTo(ShadowValue, myShadows == 1);
	}

	//If we have an array of length N and we want the n'th element using circular (wrap-around) indexing, this function
	//returns the actual index in the array that corresponds to the n'th element. For example, if n = -1 and N = 7, this function
	//returns 6, since that is the last element, which is seen as 1 left of the first element. If n = 7 and N = 7, this function
	//returns 0, since that is one element past then end, which wraps back to 0.
	static int circularIndex(int n, int N) {
		bool wasNegative = false;
		if (n < 0) {
			wasNegative = true;
			n = -n;
		}
		int offset = n % N;
		return (wasNegative) ? (N - offset) : (offset);
	}

	static void DisplayInstantaneousShadowsAndFlows(std::Evector<Polygon> const & Shadows, cv::Mat & UnmaskedShadowMap,
		                         std::Eunordered_map<std::tuple<int,int,int>, Eigen::Vector2d> const & CurrentBestEstimateFlow) {
		cv::Mat visBGR(UnmaskedShadowMap.rows, UnmaskedShadowMap.cols, CV_8UC3, cv::Scalar(55,55,55));
		for (auto const & shadow : Shadows) {
			//Draw the boundary
			std::vector<std::vector<cv::Point>> tempContoursA;
			tempContoursA.emplace_back();
			std::Evector<Eigen::Vector2d> const & verticesA(shadow.m_boundary.GetVertices());
			tempContoursA.back().reserve(verticesA.size());
			for (Eigen::Vector2d const & v : verticesA)
				tempContoursA.back().push_back(cv::Point(v(0), v(1)));
			cv::drawContours(visBGR, tempContoursA, 0, cv::Scalar(255,255,255), -1);

			//Draw the holes
			for (auto const & hole : shadow.m_holes) {
				std::vector<std::vector<cv::Point>> tempContoursB;
				tempContoursB.emplace_back();
				std::Evector<Eigen::Vector2d> const & verticesB(hole.GetVertices());
				tempContoursB.back().reserve(verticesB.size());
				for (Eigen::Vector2d const & v : verticesB)
					tempContoursB.back().push_back(cv::Point(v(0), v(1)));
				cv::drawContours(visBGR, tempContoursB, 0, cv::Scalar(55,55,55), -1);
			}
		}

		for (auto const & shadow : Shadows) {
			//Draw the boundary
			std::vector<std::vector<cv::Point>> tempContoursA;
			tempContoursA.emplace_back();
			std::Evector<Eigen::Vector2d> const & verticesA(shadow.m_boundary.GetVertices());
			tempContoursA.back().reserve(verticesA.size());
			for (Eigen::Vector2d const & v : verticesA)
				tempContoursA.back().push_back(cv::Point(v(0), v(1)));
			cv::drawContours(visBGR, tempContoursA, 0, cv::Scalar(128,128,128), 2);

			//Draw the holes
			for (auto const & hole : shadow.m_holes) {
				std::vector<std::vector<cv::Point>> tempContoursB;
				tempContoursB.emplace_back();
				std::Evector<Eigen::Vector2d> const & verticesB(hole.GetVertices());
				tempContoursB.back().reserve(verticesB.size());
				for (Eigen::Vector2d const & v : verticesB)
					tempContoursB.back().push_back(cv::Point(v(0), v(1)));
				cv::drawContours(visBGR, tempContoursB, 0, cv::Scalar(128,128,128), 2);
			}
		}

		for (int polyIndex = 0; polyIndex < (int) Shadows.size(); polyIndex++) {
			Polygon const & poly(Shadows[polyIndex]);
			for (int simplePolyIndex = -1; simplePolyIndex < (int) poly.m_holes.size(); simplePolyIndex++) {
				SimplePolygon const & simplePoly(simplePolyIndex < 0 ? poly.m_boundary : poly.m_holes[simplePolyIndex]);
				std::Evector<Eigen::Vector2d> const & vertices(simplePoly.GetVertices());
				for (int vertexIndex = 0; vertexIndex < (int) vertices.size(); vertexIndex++) {
					std::tuple<int,int,int> currentPointIndex(polyIndex, simplePolyIndex, vertexIndex);
					Eigen::Vector2d const & boundaryPoint(vertices[vertexIndex]);

					Eigen::Vector2d flow = Eigen::Vector2d::Zero();
					if (CurrentBestEstimateFlow.count(currentPointIndex) > 0U)
						flow = CurrentBestEstimateFlow.at(currentPointIndex);

					flow = 2.0*flow; //Optionally magnify the flow to make the vectors more visible
					cv::Point pt1(boundaryPoint(0), boundaryPoint(1));
					cv::Point pt2(boundaryPoint(0) + flow(0), boundaryPoint(1) + flow(1));
					cv::arrowedLine(visBGR, pt1, pt2, cv::Scalar(0,0,255), 1, 8, 0, 0.1);
				}
			}
		}

		cv::imshow("Shadow Flows"s, visBGR);
		cv::waitKey(1);
	}

	void ShadowPropagationEngine::ModuleMain_ContourFlow(void) {
		const int HISTORY_LENGTH = 10; //Number of epochs into the past to "smooth" flow vectors over
		const int PREDICTION_HORIZON = 20; //Number of epochs into the future to predict shadow evolution
		const double MAX_SHADOW_SPEED_MPS = 44.7; //Flows above this speed are thrown out immediately (m/s)

		std::deque<TimePoint> inputHist_timestamps; //History of timestamps for recently received shadow

		//We keep a vector of shadows found in the most recent instantaneous shadow map
		//We store each shadow as a polygon, each of which may have arbitrarily many holes. It seems that this may be
		//overkill given how the rest of this algorithm has shaken out, but it isn't especially expensive to do and is a
		//representation that contains the most information, so we'll stay with this so we may try other approaches later, if needed.
		std::Evector<Polygon> shadows;

		//The contour flow map has structure boundaryPointIndex -> FlowHistory
		//boundaryPointIndex has structure: <polyIndex, simplePolyIndex, vertexIndex>
		//simplePolyIndex follows the convention that -1 refers to the boundary and any value >= 0 refers to a hole
		//FlowHistory is a queue that holds the flow history for the corresponding boundary point.
		std::Eunordered_map<std::tuple<int,int,int>, std::Edeque<Eigen::Vector2d>> contourFlowMap;

		bool initNeeded = true; //When true, we need to clear our history and re-initialize internal state
		while (! m_abort) {
			m_mutex.lock();
			if (! m_running) {
				m_mutex.unlock();
				initNeeded = true; //Next time through the loop we need to re-initialize
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}

			if (m_unprocessedShadowMaps.empty()) {
				//There are no unprocessed shadow maps to deal with
				m_mutex.unlock();
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}

			//If we get here, we have unprocessed shadow maps waiting for us
			
			//If this is the first received shadow map since (re)-starting, clear internal state data
			if (initNeeded) {
				inputHist_timestamps.clear();
				initNeeded = false;
			}
			
			//Grab the first unprocessed shadow map and remove it from the dequeue
			ShadowDetection::InstantaneousShadowMap map = m_unprocessedShadowMaps.front();
			m_unprocessedShadowMaps.pop_front();

			bool behindRealtime = !m_unprocessedShadowMaps.empty();
			if (behindRealtime) {
				std::cerr << "Warning: Shadow Propagation module falling behind real-time. ";
				std::cerr << (unsigned int) m_unprocessedShadowMaps.size() << " frames buffered.\r\n"; 
			}

			//Now unlock m_mutex and proceed with model evaluation - lock m_mutex later when updating protected fields
			m_mutex.unlock();

			//Save the shadow map to our history buffer
			inputHist_timestamps.push_back(map.Timestamp);
			if (inputHist_timestamps.size() > HISTORY_LENGTH)
				inputHist_timestamps.pop_front();

			if (behindRealtime) {
				std::cerr << "Warning: Shadow Propagation Module skipping update because we are lagging real-time.\r\n";
				continue;
			}

			//Here is where we do the work
			cv::Mat unmaskedShadowMap;
			map.Map.copyTo(unmaskedShadowMap);
			unmaskedShadowMap.setTo(0, unmaskedShadowMap == 255);
			unmaskedShadowMap.setTo(0, unmaskedShadowMap  < 128);
			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::findContours(unmaskedShadowMap, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

			//Build Polygons for each shadow
			TimePoint T0 = std::chrono::steady_clock::now();
			std::Evector<Polygon> newShadows;
			for (int index = 0; index < (int) contours.size(); index++) {
				if (hierarchy[index][3] < 0) {
					//This is a shadow outer boundary
					newShadows.emplace_back();
					if (! CVContourToSimplePolygon(contours[index], newShadows.back().m_boundary, 4))
						newShadows.pop_back();
					else {
						//Add any child contours as holes in the shadow
						int childIndex = hierarchy[index][2];
						while (childIndex >= 0) {
							newShadows.back().m_holes.emplace_back();
							if (! CVContourToSimplePolygon(contours[childIndex], newShadows.back().m_holes.back(), 4))
								newShadows.back().m_holes.pop_back();
							childIndex = hierarchy[childIndex][0];
						}
					}
				}
			}
			//if (newShadows.empty())
			//	std::cerr << "No shadows.\r\n";

			//Compute the seconds per epoch
			double secondsPerEpoch = 1.0; //Default when we only have one timestamp
			if (inputHist_timestamps.size() > 1U) {
				double historyDuration = SecondsElapsed(inputHist_timestamps.front(), inputHist_timestamps.back());
				secondsPerEpoch = historyDuration / double(inputHist_timestamps.size() - 1);
			}

			//Convert the upper limit on contour movement speed from meters/second to pixels per epoch
			Eigen::Vector2d map_LL_NM = LatLonToNM(map.LL_LL);
			Eigen::Vector2d map_UR_NM = LatLonToNM(map.UR_LL);
			Eigen::Vector2d mapCenter_NM = 0.5*map_LL_NM + 0.5*map_UR_NM;
			double metersPerPixel_X = NMUnitsToMeters(map_UR_NM(0) - map_LL_NM(0), mapCenter_NM(1)) / double(map.Map.cols);
			double metersPerPixel_Y = NMUnitsToMeters(map_UR_NM(1) - map_LL_NM(1), mapCenter_NM(1)) / double(map.Map.rows);
			double metersPerPixel = 0.5*metersPerPixel_X + 0.5*metersPerPixel_Y;
			double MaxFlow_PixelsPerEpoch = MAX_SHADOW_SPEED_MPS * secondsPerEpoch / metersPerPixel;


			//We want to avoid building shadow tracks if possible. Such an approach requires us to assiciate shadows
			//in a new map with shadows in previous maps and build trajectories. This requires us to deal with track
			//initiation, termination, fusion, and separation events. Instead, we can work directly with points on
			//the boundaries of shadows. For each boundary point, find the nearest boundary point in the previous map.
			//The displacement between these points gives us a flow value for the boundary point. We can also inherit
			//the flow value that was previously computed for the point we found in the previous map, and thus an entire
			//history of flow values. We can use a median over some number of epochs as our current best estimate of
			//contour flow at this boundary point at the present time. New shadows will have abberant instantaneous
			//flow values, but only for one epoch, so the median operator will screen these out. The result is that
			//a new shadow will essentially inherit the contour flows from the nearest already existing shadow, which
			//is a very reasonable behavior.


			//Update contourFlowMap and shadows - Also compute current best estimate of flow for each boundary point
			TimePoint T1 = std::chrono::steady_clock::now();
			std::Eunordered_map<std::tuple<int,int,int>, std::Edeque<Eigen::Vector2d>> newContourFlowMap;
			std::Eunordered_map<std::tuple<int,int,int>, Eigen::Vector2d> currentBestEstimateFlow;
			for (int polyIndex = 0; polyIndex < (int) newShadows.size(); polyIndex++) {
				Polygon & poly(newShadows[polyIndex]);
				for (int simplePolyIndex = -1; simplePolyIndex < (int) poly.m_holes.size(); simplePolyIndex++) {
					SimplePolygon & simplePoly(simplePolyIndex < 0 ? poly.m_boundary : poly.m_holes[simplePolyIndex]);
					std::Evector<Eigen::Vector2d> const & vertices(simplePoly.GetVertices());
					for (int vertexIndex = 0; vertexIndex < (int) vertices.size(); vertexIndex++) {
						std::tuple<int,int,int> currentPointIndex(polyIndex, simplePolyIndex, vertexIndex);
						Eigen::Vector2d const & boundaryPoint(vertices[vertexIndex]);

						int polyIndexPrevMap = 0;
						int simplePolyIndexPrevMap = 0;
						int vertexIndexPrevMap = 0;
						Eigen::Vector2d ClosestBdryPointInPrevMap;
						bool pointFound = FindClosestBoundaryPoint(boundaryPoint, shadows, polyIndexPrevMap, simplePolyIndexPrevMap,
						                                           vertexIndexPrevMap, ClosestBdryPointInPrevMap);

						if (pointFound) {
							std::tuple<int,int,int> prevPointIndex(polyIndexPrevMap, simplePolyIndexPrevMap, vertexIndexPrevMap);

							//We compute the instantaneous flow vector as the displacement between the closest boundary point
							//in the previous map and the current boundary point;
							Eigen::Vector2d flowVec = boundaryPoint - ClosestBdryPointInPrevMap;

							//Only copy the flow history from the previous point if the current flow vector is
							//plausible - this prevents bootstrapping new shadow motion from very distant shadows
							if ((contourFlowMap.count(prevPointIndex) > 0U) && (flowVec.norm() <= MaxFlow_PixelsPerEpoch))
								newContourFlowMap[currentPointIndex] = contourFlowMap[prevPointIndex];
							if (flowVec.norm() <= MaxFlow_PixelsPerEpoch) {
								newContourFlowMap[currentPointIndex].push_back(flowVec);
								if (newContourFlowMap.at(currentPointIndex).size() > HISTORY_LENGTH)
									newContourFlowMap.at(currentPointIndex).pop_front();
							}

							if ((newContourFlowMap.count(currentPointIndex) > 0U) &&
							    (newContourFlowMap.at(currentPointIndex).size() >= 5U)) {
								//A 2D geometric median is probably the "correct" thing to do here,
								//but it's probably overkill and an iterative scheme like Weiszfeld's algorithm
								//in an inner loop like this will add significant computational expense. It should
								//suffice to use a cheaper solution like element-wise median or vector of median magnitude.

								//Grab the flow vector of median magnitude
								/*std::Evector<Eigen::Vector2d> flows;
								std::vector<double> flowNorms;
								flows.reserve(HISTORY_LENGTH);
								flowNorms.reserve(HISTORY_LENGTH);
								for (auto const & flow : newContourFlowMap.at(currentPointIndex)) {
									flows.push_back(flow);
									flowNorms.push_back(flow.norm());
								}
								std::vector<int> indices(flowNorms.size());
								std::iota(indices.begin(), indices.end(), 0);
								std::sort(indices.begin(), indices.end(), [&](int A, int B) -> bool {
									return flowNorms[A] < flowNorms[B];
								});
								int middleIndex = flowNorms.size() / 2U;
								currentBestEstimateFlow[currentPointIndex] = flows[indices[middleIndex]];*/

								//Get the element-wise median flow - this seems to work better most of the time
								std::vector<double> flows_x;   flows_x.reserve(HISTORY_LENGTH);
								std::vector<double> flows_y;   flows_y.reserve(HISTORY_LENGTH);
								for (auto const & flow : newContourFlowMap.at(currentPointIndex)) {
									flows_x.push_back(flow(0));
									flows_y.push_back(flow(1));
								}
								double medianFlow_x = getMedian(flows_x, true);
								double medianFlow_y = getMedian(flows_y, true);
								currentBestEstimateFlow[currentPointIndex] << medianFlow_x, medianFlow_y;
							}
						}
					}
				}
			}
			shadows.swap(newShadows);
			contourFlowMap.swap(newContourFlowMap);
			TimePoint T2 = std::chrono::steady_clock::now();

			//Apply a moving average filter to the flows along each contour
			for (int polyIndex = 0; polyIndex < (int) shadows.size(); polyIndex++) {
				Polygon & poly(shadows[polyIndex]);
				for (int simplePolyIndex = -1; simplePolyIndex < (int) poly.m_holes.size(); simplePolyIndex++) {
					SimplePolygon & simplePoly(simplePolyIndex < 0 ? poly.m_boundary : poly.m_holes[simplePolyIndex]);
					
					//Pack the flow vectors along this boundary into a vector
					std::Evector<Eigen::Vector2d> flows(simplePoly.GetVertices().size());
					for (int vertexIndex = 0; vertexIndex < (int) simplePoly.GetVertices().size(); vertexIndex++) {
						std::tuple<int,int,int> vertexAddress(polyIndex, simplePolyIndex, vertexIndex);
						if (currentBestEstimateFlow.count(vertexAddress) > 0U)
							flows[vertexIndex] = currentBestEstimateFlow.at(vertexAddress);
						else
							flows[vertexIndex] << std::nan(""), std::nan("");
					}

					//Apply NaN-aware MA filter - write results directly back to best-estimate flow map
					int N = 7;
					for (int vertexIndex = 0; vertexIndex < (int) flows.size(); vertexIndex++) {
						Eigen::Vector2d cumulativeFlow = Eigen::Vector2d::Zero();
						int count = 0;
						for (int n = vertexIndex - N/2; n < vertexIndex + N/2; n++) {
							int sampleIndex = circularIndex(n, int(flows.size()));
							if (! std::isnan((flows[sampleIndex])(0))) {
								cumulativeFlow += flows[sampleIndex];
								count++;
							}
						}

						std::tuple<int,int,int> vertexAddress(polyIndex, simplePolyIndex, vertexIndex);
						if (count == 0)
							currentBestEstimateFlow.erase(vertexAddress);
						else
							currentBestEstimateFlow[vertexAddress] = cumulativeFlow / double(count);
					}
				}
			}
			TimePoint T3 = std::chrono::steady_clock::now();

			bool showCurrentShadowsAndFlows = false;
			if (showCurrentShadowsAndFlows)
				DisplayInstantaneousShadowsAndFlows(shadows, unmaskedShadowMap, currentBestEstimateFlow);

			//Now we propagate the vertices of all simple polygons forward in time.
			//We build a data structure to hold the vertices of each simple polygon over some future time horizon
			//We need to do this so we can propagate boundaries into the future without having to constantly sanitize
			//simple polygons and re-compute flows by associating new boundary points with old ones (which isn't cheap)
			//Structure: <polyIndex,simplePolyIndex,PredictionNum> -> vector of vertex locations
			//PredictionNum corresponds to PredictionNum*deltaE epochs into the future.
			//Thus, Epoch 0 corresponds to the present epoch, not the first prediction.
			//We also track which simple polygons have flow data... that is, which contain at least 1 vertex with a flow estimate
			double deltaE = 0.5; //Time resolution of forward predictions, in epochs (1 means step forward 1 epoch at a time)
			int numPredictions = (int) std::round(double(PREDICTION_HORIZON)/deltaE);
			std::Eunordered_map<std::tuple<int,int,int>, std::Evector<Eigen::Vector2d>> simplePolyVertices;
			std::Eunordered_map<std::tuple<int,int>, bool> simplePolyFlowKnown; //<polyIndex,simplePolyIndex> -> valid
			for (int polyIndex = 0; polyIndex < (int) shadows.size(); polyIndex++) {
				Polygon & poly(shadows[polyIndex]);
				for (int simplePolyIndex = -1; simplePolyIndex < (int) poly.m_holes.size(); simplePolyIndex++) {
					SimplePolygon & simplePoly(simplePolyIndex < 0 ? poly.m_boundary : poly.m_holes[simplePolyIndex]);
					std::tuple<int,int> simplePolyAddress(polyIndex,simplePolyIndex);
					simplePolyFlowKnown[simplePolyAddress] = false; //initialize flowKnown flag to false

					std::Evector<Eigen::Vector2d> const & vertices_CurrentPos(simplePoly.GetVertices());

					//Copy the current vertex locations to all future epochs for initialization
					for (int predNum = 0; predNum < numPredictions + 1; predNum++) {
						std::tuple<int,int,int> address(polyIndex, simplePolyIndex, predNum);
						simplePolyVertices[address] = vertices_CurrentPos;
					}

					for (int vertexIndex = 0; vertexIndex < (int) vertices_CurrentPos.size(); vertexIndex++) {
						std::tuple<int,int,int> currentPointIndex(polyIndex, simplePolyIndex, vertexIndex);

						//Right now we use a flow of 0 if there is no flow estimate. This has the effect of not ignoring
						//new or transient shadows, but just assuming that they are stationary. We may want to remove such
						//vertices instead and ignore such shadows.
						Eigen::Vector2d flow = Eigen::Vector2d::Zero();
						if (currentBestEstimateFlow.count(currentPointIndex) > 0U) {
							flow = currentBestEstimateFlow.at(currentPointIndex);
							simplePolyFlowKnown[simplePolyAddress] = true;
						}

						for (int predNum = 0; predNum < numPredictions + 1; predNum++) {
							std::tuple<int,int,int> address(polyIndex, simplePolyIndex, predNum);
							double epochsIntoFuture = double(predNum)*deltaE;
							simplePolyVertices.at(address)[vertexIndex] = vertices_CurrentPos[vertexIndex] + flow*epochsIntoFuture;
						}
					}
				}
			}
			TimePoint T4 = std::chrono::steady_clock::now();

			//The next step is to build a TA function by initializing a canvas to the sentinal value and iteratively
			//painting our predictions to the canvas, starting with the farthest prediction into the future, and working
			//backwards to the current instantaneous shadow map. This ensures that predictions that are closer to the present
			//take precedent over those farther into the future, which is what we want in the time available function.
			//We have two implementations for this. The first uses the raw vertex vectors that we computed in the previous step.
			//The second sanitizes these vertex vectors before painting. The sanitizing ensures that the countours that we
			//propagated into the future still, in fact, represent valid simple polygons, without things like self-intersections.
			//This is technically safer, but comes with added computational expense. It seems that the vast majority of the time
			//the resulting TA functions are nearly indistinguishable. Skipping sanitization speeds up this step by about 2-4X
			//and since this step is by far the most expensive step in the module, this is significant.

			//One thing we don't want to do here is convert all future predictions to polygons and do an inclusion test
			//for each pixel in each prediction. This uses a winding number test which requires us to iterate over all vertices
			//for each simple polygon in each prediction, and doing this for every pixel is too expensive. We did an initial
			//implementation this way for simplicity and it takes about 2 seconds per TA evaluation in a typical case... which is
			//about 400X slower than Implementation 1 below, even when running with reduced temporal resolution. Since this is
			//completely non-viable there is no need to include this as an option.

			//Implementation 1   *******************************************************************************
			//Compute the time available map by iteratively "painting" the shadow predictions on the same canvas
			//We paint to a vector of canvases... one per prediction where 0 means no shadow and 1 means shadow
			//We first paint all boundary polygons and then come back and paint all holes. This implementation does
			//not sanitize our predictions and so is a little bit unsafe but faster (in practice artifacts seem to
			//be both rare and minor).
			std::vector<cv::Mat> futurePredictionRasters(numPredictions + 1);
			for (cv::Mat & raster : futurePredictionRasters)
				raster = cv::Mat(map.Map.rows, map.Map.cols, CV_8UC1, cv::Scalar(0));
			for (auto const & kv : simplePolyVertices) {
				int polyIndex       = std::get<0>(kv.first);
				int simplePolyIndex = std::get<1>(kv.first);
				int predictionNum   = std::get<2>(kv.first);
				std::tuple<int,int> simplePolyAddress(polyIndex, simplePolyIndex);

				//If this is a boundary polygon with known flow somewhere along it's contour, paint it now
				if ((simplePolyIndex < 0) && (simplePolyFlowKnown.at(simplePolyAddress))) {
					std::Evector<Eigen::Vector2d> const & vertices(kv.second);
					std::vector<std::vector<cv::Point>> tempContours;
					tempContours.emplace_back();
					tempContours.back().reserve(vertices.size());
					for (Eigen::Vector2d const & v : vertices)
						tempContours.back().push_back(cv::Point(v(0), v(1)));
					cv::drawContours(futurePredictionRasters[predictionNum], tempContours, 0, cv::Scalar(1), -1);
				}
			}
			for (auto const & kv : simplePolyVertices) {
				int polyIndex       = std::get<0>(kv.first);
				int simplePolyIndex = std::get<1>(kv.first);
				int predictionNum   = std::get<2>(kv.first);
				std::tuple<int,int> simplePolyAddress(polyIndex, simplePolyIndex);

				//If this is a hole polygon with known flow somewhere along it's contour, paint it now
				if ((simplePolyIndex >= 0) && (simplePolyFlowKnown.at(simplePolyAddress))) {
					std::Evector<Eigen::Vector2d> const & vertices(kv.second);
					std::vector<std::vector<cv::Point>> tempContours;
					tempContours.emplace_back();
					tempContours.back().reserve(vertices.size());
					for (Eigen::Vector2d const & v : vertices)
						tempContours.back().push_back(cv::Point(v(0), v(1)));
					cv::drawContours(futurePredictionRasters[predictionNum], tempContours, 0, cv::Scalar(0), -1);
				}
			}
			//Now combine all of the raster predictions into a TA function
			cv::Mat TA(map.Map.rows, map.Map.cols, CV_16UC1, cv::Scalar(std::numeric_limits<uint16_t>::max()));
			for (int predictionNum = numPredictions; predictionNum >= 0; predictionNum--) {
				double epochsIntoFuture = double(predictionNum)*deltaE;
				uint16_t secondsIntoFuture = uint16_t(std::round(epochsIntoFuture * secondsPerEpoch));
				//std::cerr << secondsIntoFuture << "\r\n";
				TA.setTo(secondsIntoFuture, futurePredictionRasters[predictionNum] == 1);
			}
			cv::medianBlur(TA, TA, 5); //Clean up tiny holes and imperfections in TA function
			TimePoint T5 = std::chrono::steady_clock::now();

			//Implementation 2   *******************************************************************************
			//Start by sanitizing all propagated simple polygon boundaries
			//This is the safer but slower implementation.
			//One map per prediction: <polyIndex,simplePolyIndex> -> simplePoly
			/*std::Evector<std::Eunordered_map<std::tuple<int,int>, SimplePolygon>> futureSimplePolys(numPredictions + 1);
			for (auto const & kv : simplePolyVertices) {
				int predictionNum = std::get<2>(kv.first);
				std::tuple<int,int> simplePolyAddress(std::get<0>(kv.first), std::get<1>(kv.first));
				
				//We skip any simple polygon with unknown flow
				if (simplePolyFlowKnown.at(simplePolyAddress))
					(futureSimplePolys[predictionNum])[simplePolyAddress] = SimplePolygon(kv.second);
			}
			//Compute a time available map by iteratively "painting" the shadow predictions on the same canvas
			cv::Mat TA(map.Map.rows, map.Map.cols, CV_16UC1);
			uint16_t maxUint16Val = std::numeric_limits<uint16_t>::max();
			TA.setTo(maxUint16Val);
			for (int predNum = numPredictions; predNum >= 0; predNum--) {
				double epochsIntoFuture = double(predNum)*deltaE;
				uint16_t secondsIntoFuture = uint16_t(std::round(epochsIntoFuture * secondsPerEpoch));
				PaintShadows_UC16(TA, futureSimplePolys[predNum], secondsIntoFuture);
			}
			cv::medianBlur(TA, TA, 5); //Clean up tiny holes and imperfections in TA function
			TimePoint T5 = std::chrono::steady_clock::now();*/
			
			//Hole In-painting   *******************************************************************************
			//In-paint holes in the TA function, which can happen when the propogated contours deform over time or when a shadow is
			//very small relative to it's speed and there are gaps between predicted locations. This is a bit expensive and doesn't
			//always give great results. It seems these holes can be avoided by increasing the temporal resolution (dropping deltaE),
			//resulting in better TA functions and without this more expensive step.
			bool inpaintHoles = false;
			if (inpaintHoles) {
				cv::Mat inpaintMask(map.Map.rows, map.Map.cols, CV_8UC1, cv::Scalar(0));
				inpaintMask.setTo(255, TA == std::numeric_limits<uint16_t>::max());
				cv::Mat inpaintMaskLabels, componentStats, centroids;
				int numLabels = cv::connectedComponentsWithStats(inpaintMask, inpaintMaskLabels, componentStats, centroids, 4, CV_16U);
				//std::cerr << "Num components: " << numLabels << ". Areas:\r\n";
				for (int n = 1; n < numLabels; n++) {
					double area = componentStats.at<int32_t>(n, cv::CC_STAT_AREA);
					//std::cerr << area << " pixels\r\n";
					if (area > 100.0)
						inpaintMask.setTo(0, inpaintMaskLabels == n);
				}
				cv::inpaint(TA, inpaintMask, TA, 15.0, cv::INPAINT_NS);
				//std::cerr << "\r\n";
				//cv::imshow("mask", inpaintMask);
				//cv::waitKey(1);
			}

			bool printProcessingRuntimes = false;
			if (printProcessingRuntimes) {
				double processingTime_A   = SecondsElapsed(T0, T1); //Build polygon objects for shadows
				double processingTime_B   = SecondsElapsed(T1, T2); //Updating contour flows
				double processingTime_C   = SecondsElapsed(T2, T3); //Filtering contour flows
				double processingTime_D   = SecondsElapsed(T3, T4); //Propagating vertices into future
				double processingTime_E   = SecondsElapsed(T4, T5); //TA evaluation
				double fullProcessingTime = SecondsElapsed(T0, T5);

				std::cerr << "Raster --> Polygons ----: " << processingTime_A*1000.0 << " ms\r\n";
				std::cerr << "Updating contour flows -: " << processingTime_B*1000.0 << " ms\r\n";
				std::cerr << "Filtering contour flows : " << processingTime_C*1000.0 << " ms\r\n";
				std::cerr << "Propagating vertices ---: " << processingTime_D*1000.0 << " ms\r\n";
				std::cerr << "TA evaluation ----------: " << processingTime_E*1000.0 << " ms\r\n";
				std::cerr << "Full processing time ---: " << fullProcessingTime*1000.0 << " ms\r\n\r\n";
			}

			{
				//Lock our mutex and update our public TA function and call all registered callbacks.
				std::scoped_lock lock(m_mutex);

				m_TimeAvail.TimeAvailable = TA;
				m_TimeAvail.UL_LL = map.UL_LL;
				m_TimeAvail.UR_LL = map.UR_LL;
				m_TimeAvail.LL_LL = map.LL_LL;
				m_TimeAvail.LR_LL = map.LR_LL;
				m_TimeAvail.Timestamp = map.Timestamp;

				for (auto const & kv : m_callbacks)
					kv.second(m_TimeAvail);
			}
		}
	}
}
