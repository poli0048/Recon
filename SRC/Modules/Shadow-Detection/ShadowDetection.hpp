//This module provides the main interface for the shadow detection system
//Authors: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <system_error>
#include <unordered_map>

//External Includes
//OpenCV basic headers for use of cv::Mat

//Project Includes
#include "../../EigenAliases.h"
#include "../DJI-Drone-Interface/DroneManager.hpp"

namespace ShadowDetection {
	class InstantaneousShadowMap {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
			
			cv::Mat Map;           //fixed-size (512 x 512) shadow map image. Use uint8_t type with 0-127 = unshadowed, and 128-256 = shadowed
			Eigen::Vector2d UL_LL; //(Latitude, Longitude) of center of upper-left pixel, in radians
			Eigen::Vector2d UR_LL; //(Latitude, Longitude) of center of upper-right pixel, in radians
			Eigen::Vector2d LL_LL; //(Latitude, Longitude) of center of lower-left pixel, in radians
			Eigen::Vector2d LR_LL; //(Latitude, Longitude) of center of lower-right pixel, in radians
			TimePoint Timestamp;
	};
	
	//This class holds a sequence of instantaneous shadow maps corresponding to different instants in time, but with the same
	//registration data (pixel (n,m) corresponds to the same point on the ground in all maps).
	class ShadowMapHistory {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
			
			std::vector<cv::Mat> Maps;         //Same definition as in InstantaneousShadowMap
			std::vector<TimePoint> Timestamps; //Item n is the timestamp for Maps[n]
			Eigen::Vector2d UL_LL; //(Latitude, Longitude) of center of upper-left pixel, in radians
			Eigen::Vector2d UR_LL; //(Latitude, Longitude) of center of upper-right pixel, in radians
			Eigen::Vector2d LL_LL; //(Latitude, Longitude) of center of lower-left pixel, in radians
			Eigen::Vector2d LR_LL; //(Latitude, Longitude) of center of lower-right pixel, in radians
			
			inline bool SaveFRFFile(std::filesystem::path const & Filepath);
	};
	
	//Singleton class for the shadow detection system
	class ShadowDetectionEngine {
		private:
			std::thread       m_engineThread;
			bool              m_running;
			std::atomic<bool> m_abort;
			std::mutex        m_mutex;
			std::unordered_map<int, std::function<void(InstantaneousShadowMap const & ShadowMap)>> m_callbacks;
			
			std::string m_ImageProviderDroneSerial; //Empty if none
			Drone * m_ImageProviderDrone = nullptr; //Pointer to image provider drone
			int m_DroneImageCallbackHandle = -1;    //Handle for image callback (if registered). -1 if none registered.
			
			cv::Mat m_ReferenceFrame;
			std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> m_Fiducials; //See SetFiducials() for structure
			
			std::vector<std::tuple<cv::Mat, TimePoint>> m_unprocessedFrames;
			InstantaneousShadowMap m_ShadowMap; //Shadow map based on most recently processed frame
			std::Evector<ShadowMapHistory> m_History; //Record of all computed shadow maps - add new element when ref frame changes (since registration changes)
			
			inline void ModuleMain(void);
			inline void ProcessFrame(cv::Mat const & Frame, TimePoint const & Timestamp);
			
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
			
			static ShadowDetectionEngine & Instance() { static ShadowDetectionEngine Obj; return Obj; }
			
			//Constructors and Destructors
			ShadowDetectionEngine() : m_engineThread(&ShadowDetectionEngine::ModuleMain, this), m_running(false), m_abort(false) { }
			~ShadowDetectionEngine() {
				Stop();
				m_abort = true;
				if (m_engineThread.joinable())
					m_engineThread.join();
			}
			
			inline void Start(std::string const & DroneSerial); //Start or restart continuous processing of fisheye imagery from the given drone
			inline void Stop(void);                             //Stop processing
			inline bool IsRunning(void);                        //Returns true if running, false if stopped
			inline int  RegisterCallback(std::function<void(InstantaneousShadowMap const & ShadowMap)> Callback); //Regester callback for new shadow maps
			inline void UnRegisterCallback(int Handle); //Unregister callback for new shadow maps (input is token returned by RegisterCallback()
			
			//Set the reference frame to be used for registration and stabilization (all other frames are aligned to the reference frame)
			inline bool SetReferenceFrame(cv::Mat const & RefFrame);
			inline bool IsReferenceFrameSet(void);
			
			//Set the fiducials used for registration. Each fiducial is a tuple of the form <PixCoords, LLA> where:
			//PixCoords are the coordinates (in pixels) in the reference image of a fiducial: (col, row) with upper-left corner origin
			//LLA are the WGS84 latitude (radians), longitude (radians), and altitude (m) of the fiducial, in that order.
			inline void SetFiducials(std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> const & Fiducials);
			inline size_t GetNumberOfFiducials(void);
			
			//Accessors - Each returns false if no shadow maps have been computed yet.
			inline bool GetTimestampOfMostRecentShadowMap(TimePoint & Timestamp);
			inline bool GetMostRecentShadowMap(InstantaneousShadowMap & ShadowMap);
			
			//Save accumulated shadow map history to a mission folder on disk and clear the in-memory history. Should be called after a mission.
			//Note: During a mission we just accumulate shadow maps in memory and delay the FRF file generation until the mission is over. This
			//results in about 15 MB / minute of memory demand if processing imagery ad 1 FPS. A 30 minute mission would amount to about 450 MB,
			//which is an acceptable hit in exchange for delaying the processing burden until a mission is over.
			inline void SaveAndFlushShadowMapHistory(void);
	};

	//Save the shadow map history to an FRF file with the given path. Return true on success and false on failure
	inline bool ShadowMapHistory::SaveFRFFile(std::filesystem::path const & Filepath) {
		//TODO - Implement Me
		std::cerr << "ShadowMapHistory::SaveFRFFile() Not implemented yet.\r\n";
		return false;
	}

	inline void ShadowDetectionEngine::Start(std::string const & DroneSerial) {
		std::scoped_lock lock(m_mutex);
		//If we already have a drone object pointer with a registered callback - unregister it and clear our pointer
		if (m_ImageProviderDrone != nullptr) {
			if (m_DroneImageCallbackHandle >= 0) {
				m_ImageProviderDrone->UnRegisterCallback(m_DroneImageCallbackHandle);
				m_DroneImageCallbackHandle = -1;
			}
			m_ImageProviderDrone = nullptr;
		}
		m_ImageProviderDroneSerial = DroneSerial; //Actual connection will occur in ModuleMain()
		m_unprocessedFrames.clear(); //Clear any unprocessed imagery
		m_running = true;
	}

	inline void ShadowDetectionEngine::Stop(void) {
		std::scoped_lock lock(m_mutex);
		m_ImageProviderDroneSerial.clear();
		if (m_ImageProviderDrone != nullptr) {
			if (m_DroneImageCallbackHandle >= 0) {
				m_ImageProviderDrone->UnRegisterCallback(m_DroneImageCallbackHandle);
				m_DroneImageCallbackHandle = -1;
			}
			m_ImageProviderDrone = nullptr;
		}
		m_running = false;
	}

	inline bool ShadowDetectionEngine::IsRunning(void) {
		std::scoped_lock lock(m_mutex);
		return m_running;
	}
	
	//Regester callback for new shadow maps (returns handle)
	inline int ShadowDetectionEngine::RegisterCallback(std::function<void(InstantaneousShadowMap & ShadowMap)> Callback) {
		std::scoped_lock lock(m_mutex);
		int token = 0;
		while (m_callbacks.count(token) > 0U)
			token++;
		m_callbacks[token] = Callback;
		return token;
	}
	
	//Unregister callback for new shadow maps (input is token returned by RegisterCallback()
	inline void ShadowDetectionEngine::UnRegisterCallback(int Handle) {
		std::scoped_lock lock(m_mutex);
		m_callbacks.erase(Handle);
	}
	
	//When we stop or restart processing we immediately unregister any image callback and trash our pointer to the drone. However, we don't
	//immediately connect to a drone when told to start processing because that drone may not be available yet. We periodically check and try
	//to connect in the main loop. We also periodically check to see if we have unprocessed imagery waiting and process a frame when we do.
	//Note: We don't chain callbacks - the callback copies data and returns and our own processing happens in our private thread (and any
	//extra threads that it might want to create). Doing the heavy lifting in the callback itself is simpler but could slow down the drone
	//objects internal thread, which is not good practice.
	inline void ShadowDetectionEngine::ModuleMain(void) {
		while (! m_abort) {
			m_mutex.lock();
			if (m_running) {
				if ((! m_ImageProviderDroneSerial.empty()) && (m_ImageProviderDrone == nullptr)) {
					//We haven't been able to get a pointer to the desired drone yet and register a callback. Try now
					m_ImageProviderDrone = DroneInterface::DroneManager::Instance().GetDrone(m_ImageProviderDroneSerial);
					if (m_ImageProviderDrone != nullptr) {
						//We connected to the drone - register an imagery callback
						m_DroneImageCallbackHandle = m_ImageProviderDrone->RegisterCallback([this](cv::Mat const & Frame, TimePoint const & Timestamp) {
							std::scoped_lock lock(m_mutex);
							//Only queue frames when we have what we need to process them or the buffer could grow without end
							if ((! m_ReferenceFrame.empty()) && (m_Fiducials.size() >= 3U)) {
								cv::Mat frameCopy;
								Frame.copyTo(frameCopy);
								m_unprocessedFrames.push_back(std::make_tuple(frameCopy, Timestamp));
							}
						});
					}
				}
				
				if ((! m_unprocessedFrames.empty()) && (! m_ReferenceFrame.empty()) && (m_Fiducials.size() >= 3U)) {
					//Process the first unprocessed frame and pop it from m_unprocessedFrames.
					ProcessFrame(std::get<0>(m_unprocessedFrames[0]), std::get<1>(m_unprocessedFrames[0]));
					m_unprocessedFrames.erase(m_unprocessedFrames.begin()); //Will cause vector re-allocation but it's actually not a big deal
					
					//We did useful work so unlock the mutex but dont snooze
					m_mutex.unlock();
				}
				else {
					m_mutex.unlock();
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
			}
			else {
				m_mutex.unlock();
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
		}
	}
	
	//A lock is already held on the object when this function is called so don't lock it here
	//On entry we also already know that we have at least 3 fiducials and a non-empty reference frame
	//TODO: Ideally, this function will lose the "inline" specifier and be moved to a CPP file.
	inline void ShadowDetectionEngine::ProcessFrame(cv::Mat const & Frame, TimePoint const & Timestamp) {
		//TODO: *********************************************************************************
		//TODO: ***************************** Magic sauce goes here *****************************
		//TODO: *********************************************************************************
		//Compute new shadow map based on the newly received frame
		
		//Update m_ShadowMap
		//Update m_History
		
		//TODO: *********************************************************************************
		//TODO: *********************************************************************************
		//TODO: *********************************************************************************
		
		//Call any registered callbacks
		for (auto const & kv : m_callbacks)
			kv.second(m_ShadowMap);
	}

	//Set the reference frame to be used for registration and stabilization (all other frames are aligned to the reference frame)
	inline bool ShadowDetectionEngine::SetReferenceFrame(cv::Mat const & RefFrame) {
		std::scoped_lock lock(m_mutex);
		RefFrame.copyTo(m_ReferenceFrame);
	}

	inline bool ShadowDetectionEngine::IsReferenceFrameSet(void) {
		std::scoped_lock lock(m_mutex);
		return (! m_ReferenceFrame.empty());
	}

	//Set the fiducials used for registration. Each fiducial is a tuple of the form <PixCoords, LLA> where:
	//PixCoords are the coordinates (in pixels) in the reference image of a fiducial: (col, row) with upper-left corner origin
	//LLA are the WGS84 latitude (radians), longitude (radians), and altitude (m) of the fiducial, in that order.
	inline void ShadowDetectionEngine::SetFiducials(std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> const & Fiducials) {
		std::scoped_lock lock(m_mutex);
		m_Fiducials = Fiducials;
	}

	inline size_t ShadowDetectionEngine::GetNumberOfFiducials(void) {
		std::scoped_lock lock(m_mutex);
		return m_Fiducials.size();
	}

	inline bool ShadowDetectionEngine::GetTimestampOfMostRecentShadowMap(TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		if ((m_ShadowMap.Map.rows == 0) || (m_ShadowMap.Map.cols == 0))
			return false;
		Timestamp = m_ShadowMap.Timestamp;
		return true;
	}

	inline bool ShadowDetectionEngine::GetMostRecentShadowMap(InstantaneousShadowMap & ShadowMap) {
		std::scoped_lock lock(m_mutex);
		if ((m_ShadowMap.Map.rows == 0) || (m_ShadowMap.Map.cols == 0))
			return false;
		ShadowMap = m_ShadowMap;
		return true;
	}
	
	inline void ShadowDetectionEngine::SaveAndFlushShadowMapHistory(void) {
		//Save each item of m_History to disk (each is a new FRF file)
		//We will save all FRF files for the mission to a subdirectory of "Shadow Map Files" in the executable directory
		//The subdirectories will be named "Mission N (Date)"
		std::scoped_lock lock(m_mutex);
		
		//Get the first free mission number 
		std::filesystem::path TargetDir = Handy::Paths::ThisExecutableDirectory() / "Shadow Map Files";
		std::error_code ec;
		if (! std::filesystem::exists(TargetDir))
			std::filesystem::create_directory(TargetDir, ec)
		std::vector<std::filesystem::path> subDirs = SubDirectories(TargetDir);
		std::unordered_set<int> existingMissionNumbers;
		for (std::filesystem::path const & p : subDirs) {
			std::string folderName = p.filename().string();
			if (folderName.substr(0, 8) == "Mission "s) {
				std::string tailStr = folderName.substr(8);
				std::string MissionNumStr = tailStr;
				size_t index = tailStr.find_first_of(" ");
				if (index != std::string::npos)
					MissionNumStr = MissionNumStr.substr(0, index);
				
				int missionNum = 0;
				if (Handy::TryConvert<int>(MissionNumStr, missionNum))
					existingMissionNumbers.insert(missionNum);
			}
		}
		int missionNum = 1;
		while (existingMissionNumbers.count(missionNum) > 0U)
			missionNum++;
		
		//Get the date string
		auto t = std::time(nullptr);
		auto tm = *std::localtime(&t);
		std::ostringstream oss;
		oss << std::put_time(&tm, "%m-%d-%Y");
		std::string dateString = oss.str();
		
		//Create the target folder
		std::filesystem::path MissionFolderPath = TargetDir / ("Mission "s + std::to_string(missionNum) + " (" + dateString + ")");
		if (! std::filesystem::create_directory(MissionFolderPath, ec)) {
			std::cerr << "Internal Error in ShadowDetectionEngine::SaveShadowMapHistory. Could not create target directory.\r\n";
			std::cerr << "Path: " << MissionFolderPath.string() << "\r\n";
			return;
		}
		
		//Save each shadow map sequence
		bool success = true;
		for (size_t n = 0U; n < m_History.size(); n++) {
			std::filesystem::path Filepath = MissionFolderPath / ("Shadow Map Sequence "s + std::to_string(n + 1U) + ".frf"s);
			if (! m_History[n].SaveFRFFile(Filepath)) {
				std::cerr << "Warning: Saving Shadow Map History to FRF file failed. File: " << Filepath.string() << "\r\n";
				success = false;
			}
		}
		if (success)
			m_History.clear();
		else
			std::cerr << "Warning: Not clearing shadow map history because saving to disk was not successful for all files.\r\n";
	}
}




