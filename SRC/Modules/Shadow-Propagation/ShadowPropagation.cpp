//
//Authors: karthik Dharmarajan, Bryan Poling

//External Includes
#include "torch/cuda.h"
#include "torch/version.h"

//Project Includes
#include "ShadowPropagation.hpp"
#include "../../Utilities.hpp"

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

static cv::Mat EAMapToTAMap(cv::Mat const & EA, double SecondsPerEpoch) {
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
}

namespace ShadowPropagation {
	void ShadowPropagationEngine::ModuleMain(void) {
		while (! m_abort) {
			m_mutex.lock();
			if (! m_running) {
				m_mutex.unlock();
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
			if (m_Starting) {
				m_inputHist.clear();
				m_inputHistTimestamps.clear();
				m_Starting = false;
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
			m_inputHist.push_back(shadowMapMatrix);
			m_inputHistTimestamps.push_back(map.Timestamp);
			if (m_inputHist.size() > TARGET_INPUT_LENGTH) {
				m_inputHist.pop_front();
				m_inputHistTimestamps.pop_front();
			}

			if (behindRealtime) {
				std::cerr << "Warning: Shadow Propagation Module skipping update because we are lagging real-time.\r\n";
				continue;
			}

			if (m_inputHist.size() < TARGET_INPUT_LENGTH) {
				//Not enough shadow maps to bootstrap the LSTM yet
				continue;
			}
			
			//If we get here, we want to evaluate the LSTM and compute a new TA function
			
			//Bootstrap the LSTM by feeding in our history of shadow maps - all but the most recent map
			for (int i = 0; i+1 < (int) m_inputHist.size(); i++) {
				// The LSTM takes in 3 values for inputs: torch::Tensor tensor, bool firstTimestep, bool decoding
				// "tensor" is the input tensor to use for bootstrapping or prediction
				// "firstTimestep" is a bool indicating whether an input is the first in a sequence (inits the models internal states)
				// "decoding" is not used for anything in the model code (always set to false)
				std::vector<torch::jit::IValue> inputs;
				torch::Tensor histInput = EigenMatrixToTensor(m_inputHist[i], m_device);
				inputs.push_back(histInput);
				inputs.push_back((i == 0));
				inputs.push_back(false);

				m_module.forward(inputs);
			}

			//Grab the most recent shadow map and initialize a local (64x64) Epochs Available (EA) function
			//The epochs available function counts epochs (instead of seconds) before expected shadowing
			cv::Mat EpochsAvailable(cv::Size(64, 64), CV_16UC1, cv::Scalar(std::numeric_limits<uint16_t>::max()));
			torch::Tensor currentPredictionTensor = EigenMatrixToTensor(m_inputHist.back(), m_device);
			UpdateEAMap(EpochsAvailable, currentPredictionTensor, uint16_t(0), OUTPUT_THRESHOLD);

			for (int epoch = 1; epoch <= TIME_HORIZON; epoch++) {
				//Propogate the current prediction a single epoch into the future and update EA function
				std::vector<torch::jit::IValue> inputs;
				inputs.push_back(currentPredictionTensor);
				inputs.push_back(false);
				inputs.push_back(false);

				// The output from m_module.forward() is really a tuple with multiple tensors:
				// [decoder_input, decoder_hidden, output_image, _, _]
				// For this, the only tensor that matters is the output_image, as it is the prediction generated by the LSTM
				currentPredictionTensor = m_module.forward(inputs).toTensor().clone().detach();
				UpdateEAMap(EpochsAvailable, currentPredictionTensor, uint16_t(epoch), OUTPUT_THRESHOLD);
			}

			//Build TA map from EA map
			double historyDuration = SecondsElapsed(m_inputHistTimestamps.front(), m_inputHistTimestamps.back());
			double secondsPerEpoch = historyDuration / double(m_inputHistTimestamps.size() - 1);
			cv::Mat TimeAvailable64x64 = EAMapToTAMap(EpochsAvailable, secondsPerEpoch);

			{
				std::scoped_lock lock(m_mutex);
				//Scale up localTimeAvailable from 64x64 to 512x512 and copy registration and timestamp from map
				cv::resize(TimeAvailable64x64, m_TimeAvail.TimeAvailable, cv::Size(512, 512), cv::INTER_LINEAR);
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
