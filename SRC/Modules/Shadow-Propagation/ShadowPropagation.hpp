//This module provides the main interface for the shadow propagation system
//Authors: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <iostream>
#include <deque>

//External Includes
#include <opencv2/opencv.hpp>

//Project Includes
#include "../../EigenAliases.h"
#include "../Shadow-Detection/ShadowDetection.hpp"
#include <torch/script.h>

namespace ShadowPropagation {
	//Class for a time-stamped, geo-registered time available function - each pixel corresponds to a patch of ground. We use type uint16_t and
	//interperet the value for a pixel as the number of seconds that patch of ground is expected to be free of shadows (measured from the timestamp).
	//If we don't see anything in our forcast that is expected to hit a given pixel then the pixel is predicted to be clear for the full prediction
	//horizon. That horizon is different for different pixels though (e.g. a pixel on our periphery may have almost no prediction horizon), and it may not
	//be easy to estimate the horizon for a given pixel (it depends on cloud speed and direction). It is likily also not very actionable information so
	//to avoid the added complexity of trying to estimate this we will just use a sentinal value to indicate this condition (that nothing currently visible
	//is expected to hit a given pixel). We will use std::numeric_limits<uint16_t>::max() to indicate this.
	class TimeAvailableFunction {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
			
			cv::Mat TimeAvailable; //Raster data. Type: uint16_t, and values represents time available (in seconds).
			Eigen::Vector2d UL_LL; //(Latitude, Longitude) of center of upper-left pixel, in radians
			Eigen::Vector2d UR_LL; //(Latitude, Longitude) of center of upper-right pixel, in radians
			Eigen::Vector2d LL_LL; //(Latitude, Longitude) of center of lower-left pixel, in radians
			Eigen::Vector2d LR_LL; //(Latitude, Longitude) of center of lower-right pixel, in radians
			TimePoint Timestamp;
	};
	
	//Singleton class for the shadow propagation system - We use a callback system to ensure that every new shadow map that is computed is received by
	//this function (even if we are falling behind real-time in processing). We use a similar mechanism in the shadow detection module to ensure that
	//it does not miss frames from the drone feed. This is all because the shadow propagation module may use models that don't handle missing data well,
	//such as LSTM networks. Note, however, that the shadow propagation module does not offer callbacks of its own because downstream users (i.e. the
	//Guidance module) do not have the same limitation and can just poll for the latest TA function whenever they need it.
	class ShadowPropagationEngine {
		private:
			std::thread       m_engineThread;
			bool              m_running;
			std::atomic<bool> m_abort;
			std::mutex        m_mutex;
<<<<<<< HEAD
			int               m_callbackHandle;
			static const int  TARGET_INPUT_LENGTH = 10;
			static const int  TIME_HORIZON = 10;
			static constexpr double OUTPUT_THRESHOLD = 0.4;

=======
			int               m_callbackHandle; //Handle for this objects shadow detection engine callback
			std::unordered_map<int, std::function<void(TimeAvailableFunction const & TA)>> m_callbacks;
			
>>>>>>> origin/master
			std::Evector<ShadowDetection::InstantaneousShadowMap> m_unprocessedShadowMaps;
			TimeAvailableFunction m_TimeAvail; //Most recent time available function
            torch::jit::script::Module m_module; // TorchScript Model
            std::deque<torch::Tensor> m_prevInputs;
			
			inline void ModuleMain(void);
			
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;

        static ShadowPropagationEngine & Instance() { static ShadowPropagationEngine Obj; return Obj; }
			
			//Constructors and Destructors
			ShadowPropagationEngine() : m_running(false), m_abort(false), m_prevInputs() {
				m_engineThread = std::thread(&ShadowPropagationEngine::ModuleMain, this);
				m_module = torch::jit::load(Handy::Paths::ThisExecutableDirectory().parent_path().string().append("/SRC/Modules/Shadow-Propagation/model.pt"));
			}
			~ShadowPropagationEngine() {
				m_abort = true;
				if (m_engineThread.joinable())
					m_engineThread.join();
			}
			
			inline void Start(void);     //Start or restart continuous processing of new shadow maps
			inline void Stop(void);      //Stop processing
			inline bool IsRunning(void); //Returns true if running, false if stopped
			
			//Callback access
			inline int  RegisterCallback(std::function<void(TimeAvailableFunction const & TA)> Callback); //Regester callback for new TA functions
			inline void UnRegisterCallback(int Handle); //Unregister callback for new TA functions (input is token returned by RegisterCallback()
			
			//Accessors - Each returns false if no Time Available functions have been computed yet
			inline bool GetTimestampOfMostRecentTimeAvailFun(TimePoint & Timestamp);
			inline bool GetMostRecentTimeAvailFun(TimeAvailableFunction & TimeAvailFun);
	};

	inline void ShadowPropagationEngine::Start() {
		std::scoped_lock lock(m_mutex);
		m_unprocessedShadowMaps.clear(); //Ditch any old unprocessed data in the buffer
		//Note: Also reset any state data relating to internal models
		if (m_running)
			return;
		//Register a callback for handling new shadow maps. Note that our callback just copies the data to a buffer - we don't do any actual
		//processing here or it would hold up the shadow detection module. The heavy lifting is done in ModuleMain()
		m_callbackHandle = ShadowDetection::ShadowDetectionEngine::Instance().RegisterCallback([this](ShadowDetection::InstantaneousShadowMap const & NewMap) {
			std::scoped_lock lock(m_mutex);
			m_unprocessedShadowMaps.push_back(NewMap);
		});
		m_running = true;
	}

	inline void ShadowPropagationEngine::Stop(void) {
		std::scoped_lock lock(m_mutex);
		if (! m_running)
			return;
		ShadowDetection::ShadowDetectionEngine::Instance().UnRegisterCallback(m_callbackHandle);
		m_running = false;
	}

	inline bool ShadowPropagationEngine::IsRunning(void) {
		std::scoped_lock lock(m_mutex);
		return m_running;
	}
	
	//Regester callback for new TA functions (returns handle)
	inline int ShadowPropagationEngine::RegisterCallback(std::function<void(TimeAvailableFunction const & TA)> Callback) {
		std::scoped_lock lock(m_mutex);
		int token = 0;
		while (m_callbacks.count(token) > 0U)
			token++;
		m_callbacks[token] = Callback;
		return token;
	}
	
	//Unregister callback for new TA functions (input is token returned by RegisterCallback()
	inline void ShadowPropagationEngine::UnRegisterCallback(int Handle) {
		std::scoped_lock lock(m_mutex);
		m_callbacks.erase(Handle);
	}
	
	//If this function gets large (and it easily could), drop the "inline" specifier and move it to a CPP file.
	//This will likily be needed anyways because some external libraries may be used in here and we don't want
	//to include their headers in a header file.
	inline void ShadowPropagationEngine::ModuleMain(void) {
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
			
			//If we get here, we have unprocessed shadow maps to deal with
			//TODO: *********************************************************************************
			//TODO: ***************************** Magic sauce goes here *****************************
			//TODO: *********************************************************************************
			//Compute new Time Available function by some means
			ShadowDetection::InstantaneousShadowMap & map(m_unprocessedShadowMaps[0]);
			
			
			//Update m_TimeAvail
			//Note: It might be a good idea to check if the buffer m_unprocessedShadowMaps is getting too large.
			//This is an indicator that we are falling behind real time. Even if we can't handle missing data gracefully we should
			//try to have a plan for this since if we fall behind real time our TA functions become worthless. Can we reset the model
			//and only process every other shadow map? This doubling the time step would be like shadows are moving twice as fast, but
			//a model trained for one time step may fork for another... just an idea.
            torch::NoGradGuard no_grad;
            m_TimeAvail.UL_LL = map.UL_LL;
			m_TimeAvail.UR_LL = map.UR_LL;
            m_TimeAvail.LL_LL = map.LL_LL;
            m_TimeAvail.LR_LL = map.LR_LL;

            // Resize 512 x 512 Mat from InstantaneousShadowMap into 64 x 64 for module input
            cv::Mat downsizedMap;
            cv::resize(map.Map, downsizedMap, cv::Size(64, 64), cv::INTER_AREA);
            // Model was trained using 0-1 float values instead of 0-255
            cv::Mat cvInputWithExcess;
            downsizedMap.convertTo(cvInputWithExcess, CV_32FC1, 1.f/255.0);

            // Need to remove the area outside of the fisheye lens
            // Using a mask to "black off" the outside area

            // Creating mask
            cv::Mat mask = cv::Mat::zeros(cv::Size(64, 64), CV_8UC1);

            // Creates inverted shadow map, then finds enclosing circle of largest contour to create mask
            cv::Mat invertedShadowMap;
            cv::threshold(downsizedMap, invertedShadowMap, 127, 255, cv::THRESH_BINARY_INV);
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(invertedShadowMap, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            cv::Point2f center;
            float radius;
            if (contours.size() > 0) {
                std::vector<cv::Point>& largestContour = contours[0];
                double largestContourArea = cv::contourArea(largestContour);
                for (int i = 0; i < contours.size(); i++) {
                    if (cv::contourArea(contours[i]) > largestContourArea) {
                        largestContour = contours[i];
                        largestContourArea = cv::contourArea(largestContour);
                    }
                }
                cv::minEnclosingCircle(largestContour, center, radius);
            }
            if (contours.size() <= 0 || radius < 29) {
                // Default circle + radius estimation so things don't fall apart
                radius = 32;
                center = cv::Point2f(33.7, 33.7);
            }
            cv::circle(mask, center, radius - 3, cv::Scalar(255), -1, 8, 0);
            cv::Mat cvInput;
            cvInputWithExcess.copyTo(cvInput, mask);

            torch::Tensor inputTensorCompressed = torch::from_blob(cvInput.data, {64, 64}, torch::kFloat);
            torch::Tensor inputTensor = inputTensorCompressed.unsqueeze(0).unsqueeze(0);
            m_prevInputs.push_back(inputTensor);
            if (m_prevInputs.size() > TARGET_INPUT_LENGTH) {
                m_prevInputs.pop_front();
            }
            for (int i = 0; i < m_prevInputs.size(); i++) {
                std::vector<torch::jit::IValue> inputs;
                inputs.push_back(m_prevInputs[i]);
                inputs.push_back((i == 0));
                m_module.forward(inputs);
            }
            cv::Mat localTimeAvailable(cv::Size(64, 64), CV_16UC1, cv::Scalar(std::numeric_limits<uint16_t>::max()));
            for (int t = 1; t <= TIME_HORIZON; t++) {
                std::vector<torch::jit::IValue> inputs;
                inputs.push_back(inputTensor);
                auto result = m_module.forward(inputs).toTuple();
                // decoder_input, decoder_hidden, output_image, _, _
                torch::Tensor outputTensor = result->elements()[2].toTensor();
                // Iterates over output tensor and then updates localTimeAvailable accordingly
                for (int i = 0; i < 64; i++) {
                    for (int j = 0; j < 64; j++) {
                        if (outputTensor[0][0][i][j].item<float>() > OUTPUT_THRESHOLD && localTimeAvailable.at<uint16_t>(i, j) > t) {
                            localTimeAvailable.at<uint16_t>(i, j) = t;
                        }
                    }
                }
                inputTensor = outputTensor;
            }
            // Scales up localTimeAvailable
            cv::resize(localTimeAvailable, m_TimeAvail.TimeAvailable, cv::Size(512, 512), cv::INTER_LINEAR);
            m_TimeAvail.Timestamp = map.Timestamp;

			//Call any registered callbacks
			for (auto const & kv : m_callbacks)
				kv.second(m_TimeAvail);
			
			m_unprocessedShadowMaps.erase(m_unprocessedShadowMaps.begin()); //Will cause re-allocation but the buffer is small so don't worry about it
			//TODO: *********************************************************************************
			//TODO: *********************************************************************************
			//TODO: *********************************************************************************
			
			//Unlock but don't snooze if we actually did work
			m_mutex.unlock();
		}
	}
	
	inline bool ShadowPropagationEngine::GetTimestampOfMostRecentTimeAvailFun(TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		if ((m_TimeAvail.TimeAvailable.rows == 0) || (m_TimeAvail.TimeAvailable.cols == 0))
			return false;
		Timestamp = m_TimeAvail.Timestamp;
		return true;
	}

	inline bool ShadowPropagationEngine::GetMostRecentTimeAvailFun(TimeAvailableFunction & TimeAvailFun) {
		std::scoped_lock lock(m_mutex);
		if ((m_TimeAvail.TimeAvailable.rows == 0) || (m_TimeAvail.TimeAvailable.cols == 0))
			return false;
		TimeAvailFun = m_TimeAvail;
		return true;
	}
}




