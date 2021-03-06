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

//External Includes
#include <opencv2/opencv.hpp>

//Project Includes
#include "../../EigenAliases.h"
#include "../Shadow-Detection/ShadowDetection.hpp"

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
			int               m_callbackHandle;
			
			std::Evector<ShadowDetection::InstantaneousShadowMap> m_unprocessedShadowMaps;
			TimeAvailableFunction m_TimeAvail; //Most recent time available function
			
			inline void ModuleMain(void);
			
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
			
			static ShadowPropagationEngine & Instance() { static ShadowPropagationEngine Obj; return Obj; }
			
			//Constructors and Destructors
			ShadowPropagationEngine() : m_engineThread(&ShadowPropagationEngine::ModuleMain, this), m_running(false), m_abort(false) { }
			~ShadowPropagationEngine() {
				m_abort = true;
				if (m_engineThread.joinable())
					m_engineThread.join();
			}
			
			inline void Start(void);     //Start or restart continuous processing of new shadow maps
			inline void Stop(void);      //Stop processing
			inline bool IsRunning(void); //Returns true if running, false if stopped
			
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




