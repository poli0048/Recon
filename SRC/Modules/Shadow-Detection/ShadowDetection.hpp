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
#include "../../../../handycpp/Handy.hpp"
#include <opencv2/opencv.hpp>
#include "../../../../eigen/Eigen/Core" //Seems like this must be included before opencv/core/eigen.hpp
#include <opencv2/core/eigen.hpp>

//Project Includes
#include "../../EigenAliases.h"
#include "../DJI-Drone-Interface/DroneManager.hpp"
#include "ocam_utils.h"

namespace ShadowDetection {
	using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
	
	class InstantaneousShadowMap {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			
			cv::Mat Map;           //fixed-size (512 x 512) shadow map image. Use uint8_t type with 0-127 = unshadowed, and 128-254 = shadowed, 255 = Masked
			Eigen::Vector2d UL_LL; //(Latitude, Longitude) of center of upper-left pixel, in radians
			Eigen::Vector2d UR_LL; //(Latitude, Longitude) of center of upper-right pixel, in radians
			Eigen::Vector2d LL_LL; //(Latitude, Longitude) of center of lower-left pixel, in radians
			Eigen::Vector2d LR_LL; //(Latitude, Longitude) of center of lower-right pixel, in radians
			TimePoint Timestamp;

			InstantaneousShadowMap() = default;
			InstantaneousShadowMap(InstantaneousShadowMap const & Other) {
				Other.Map.copyTo(this->Map); //Deep copy the map
				this->UL_LL     = Other.UL_LL;
				this->UR_LL     = Other.UR_LL;
				this->LL_LL     = Other.LL_LL;
				this->LR_LL     = Other.LR_LL;
				this->Timestamp = Other.Timestamp;
			}
	};
	
	//This class holds a sequence of instantaneous shadow maps corresponding to different instants in time, but with the same
	//registration data (pixel (n,m) corresponds to the same point on the ground in all maps).
	class ShadowMapHistory {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			
			std::vector<cv::Mat> Maps;         //Same definition as in InstantaneousShadowMap
			std::vector<TimePoint> Timestamps; //Item n is the timestamp for Maps[n]
			Eigen::Vector2d UL_LL; //(Latitude, Longitude) of center of upper-left pixel, in radians
			Eigen::Vector2d UR_LL; //(Latitude, Longitude) of center of upper-right pixel, in radians
			Eigen::Vector2d LL_LL; //(Latitude, Longitude) of center of lower-left pixel, in radians
			Eigen::Vector2d LR_LL; //(Latitude, Longitude) of center of lower-right pixel, in radians
			
			bool SaveFRFFile(std::filesystem::path const & Filepath);
	};
	
	//Singleton class for the shadow detection system
	class ShadowDetectionEngine {
		private:
			std::thread       m_engineThread;
			std::atomic<bool> m_running;
			std::atomic<bool> m_abort;
			std::atomic<bool> m_autosaveOnStop;
			
			//We use a staging area to register and unregister callbacks. In ProcessFrame(), we check to see if the staging area has
			//been modified and update our "official" callbacks structure if necessary. This allows us to invoke our callbacks
			//without holding a lock on our callback mutex, but it also only triggers a copy when a callback is registered or unregistered.
			//m_callbacks and m_callbacks_staging are kept synchronized.
			std::mutex m_callbacks_stagingMutex;
			bool m_callback_stagingModified = false;
			std::unordered_map<int, std::function<void(InstantaneousShadowMap const & ShadowMap)>> m_callbacks_staging;
			
			std::unordered_map<int, std::function<void(InstantaneousShadowMap const & ShadowMap)>> m_callbacks; //Only used in ProcessFrame()
			
			//These are not directly accessed in ProcessFrame()
			std::mutex m_ImageProviderMutex;
			std::string m_ImageProviderDroneSerial; //Empty if none
			DroneInterface::Drone * m_ImageProviderDrone = nullptr; //Pointer to image provider drone
			int m_DroneImageCallbackHandle = -1;    //Handle for image callback (if registered). -1 if none registered.
			std::vector<std::tuple<cv::Mat, TimePoint>> m_unprocessedFrames;
			
			//These variables are modified in ProcessFrame()
			std::mutex m_shadowMapMutex; //Protects the fields in this block
			InstantaneousShadowMap m_ShadowMap; //Shadow map based on most recently processed frame
			std::Evector<ShadowMapHistory> m_History; //Record of all computed shadow maps - add new element when ref frame changes (since registration changes)
			cv::Mat m_ReferenceFrame; //Computed in SetReferenceFrame()
			std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> m_Fiducials; //Set in SetFiducials() - see method for structure
			
			//These variables can safely be accessed in ProcessFrame without locking since the other methods that modify them fail if module is running
			//Some of these are only used in one or the two implementations in ProcessFrame().
			Eigen::Vector3d LEAOrigin_ECEF; //Computed in SetFiducials()
			Eigen::Vector3d ENUOrigin_ECEF; //Computed in SetFiducials()
			Eigen::Vector2d center;         //Computed in SetFiducials()
    			double max_extent;              //Computed in SetFiducials()
			Eigen::Matrix3d R_Cam_ENU;      //Computed in SetFiducials()
    			Eigen::Vector3d CamCenter_ENU;  //Computed in SetFiducials()
			struct ocam_model o;            //Set in SetFiducials()
			cv::Mat ref_descriptors;        //Computed in SetReferenceFrame()
			std::vector<cv::KeyPoint> keypoints_ref; //Computed in SetReferenceFrame()
			cv::Mat brightest;              //Initialized in SetReferenceFrame(), updated in ProcessFrame()
			cv::Mat m_apertureMask;         //Mask of which pixels are valid (in raw image space - no distortion correction, etc.)
			cv::Mat m_refFrameApertureMask_EN;
			std::vector<std::vector<std::deque<uint8_t>>> m_brightnessHist_EN; //[row][col] -> deque of recent values for the given pixel
			
			inline void ModuleMain(void);
			void ProcessFrame(cv::Mat const & Frame, TimePoint const & Timestamp);
			
			void TryInitShadowMapAndHistory(void); //Sets the corner coords in both m_ShadowMap and m_History if GCPs and a ref frame are provided
			
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			
			static ShadowDetectionEngine & Instance() { static ShadowDetectionEngine Obj; return Obj; }
			
			//Constructors and Destructors
			ShadowDetectionEngine() : m_running(false), m_abort(false), m_autosaveOnStop(false) {
				m_engineThread = std::thread(&ShadowDetectionEngine::ModuleMain, this);
			}
			~ShadowDetectionEngine() { Shutdown(); }
			inline void Shutdown(void);  //Stop processing and terminate engine thread
			
			inline void        Start(std::string const & DroneSerial); //Start or restart continuous processing of fisheye imagery from the given drone
			inline void        Stop(void);                             //Stop processing
			inline bool        IsRunning(void);                        //Returns true if running, false if stopped
			inline std::string GetProviderDroneSerial(void);           //Returns serial of drone we are getting imagery from (empty string if not running)
			inline int         RegisterCallback(std::function<void(InstantaneousShadowMap const & ShadowMap)> Callback); //Regester callback for new shadow maps
			inline void        UnRegisterCallback(int Handle); //Unregister callback for new shadow maps (input is token returned by RegisterCallback()
			
			//Set the reference frame to be used for registration and stabilization (all other frames are aligned to the reference frame)
			void SetReferenceFrame(cv::Mat const & RefFrame);
			inline bool IsReferenceFrameSet(void);
			
			//Set the fiducials used for registration. Each fiducial is a tuple of the form <PixCoords, LLA> where:
			//PixCoords are the coordinates (in pixels) in the reference image of a fiducial: (col, row) with upper-left corner origin
			//LLA are the WGS84 latitude (radians), longitude (radians), and altitude (m) of the fiducial, in that order.
			void SetFiducials(std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> const & Fiducials);
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

	inline void ShadowDetectionEngine::Shutdown(void) {
		if (m_running)
			Stop(); //Gracefully stop processing - will also trigger save if still running on real drone
		
		m_abort = true;
		if (m_engineThread.joinable())
			m_engineThread.join();
	}

	inline void ShadowDetectionEngine::Start(std::string const & DroneSerial) {
		std::scoped_lock lock(m_ImageProviderMutex);
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
		m_autosaveOnStop = false; //Default to false - will be set to true on drone connection if drone is real
		m_running = true;
	}

	inline void ShadowDetectionEngine::Stop(void) {
		m_ImageProviderMutex.lock();
		m_ImageProviderDroneSerial.clear();
		if (m_ImageProviderDrone != nullptr) {
			if (m_DroneImageCallbackHandle >= 0) {
				m_ImageProviderDrone->UnRegisterCallback(m_DroneImageCallbackHandle);
				m_DroneImageCallbackHandle = -1;
			}
			m_ImageProviderDrone = nullptr;
		}
		m_running = false;
		m_ImageProviderMutex.unlock();

		if (m_autosaveOnStop)
			SaveAndFlushShadowMapHistory();
		m_autosaveOnStop = false; //Reset to false - will be set back to true next time it's started on a real drone
	}

	inline bool ShadowDetectionEngine::IsRunning(void) {
		return m_running;
	}
	
	inline std::string ShadowDetectionEngine::GetProviderDroneSerial(void) {
		std::scoped_lock lock(m_ImageProviderMutex);
		return m_ImageProviderDroneSerial;
	}
	
	//Regester callback for new shadow maps (returns handle)
	inline int ShadowDetectionEngine::RegisterCallback(std::function<void(InstantaneousShadowMap const & ShadowMap)> Callback) {
		std::scoped_lock lock(m_callbacks_stagingMutex);
		int token = 0;
		while (m_callbacks_staging.count(token) > 0U)
			token++;
		m_callbacks_staging[token] = Callback;
		m_callback_stagingModified = true;
		return token;
	}
	
	//Unregister callback for new shadow maps (input is token returned by RegisterCallback())
	inline void ShadowDetectionEngine::UnRegisterCallback(int Handle) {
		std::scoped_lock lock(m_callbacks_stagingMutex);
		m_callbacks_staging.erase(Handle);
		m_callback_stagingModified = true;
	}
	
	//When we stop or restart processing we immediately unregister any image callback and trash our pointer to the drone. However, we don't
	//immediately connect to a drone when told to start processing because that drone may not be available yet. We periodically check and try
	//to connect in the main loop. We also periodically check to see if we have unprocessed imagery waiting and process a frame when we do.
	//Note: We don't chain callbacks - the callback copies data and returns and our own processing happens in our private thread (and any
	//extra threads that it might want to create). Doing the heavy lifting in the callback itself is simpler but could slow down the drone
	//objects internal thread, which is not good practice.
	inline void ShadowDetectionEngine::ModuleMain(void) {
		while (! m_abort) {
			if (m_running) {
				m_ImageProviderMutex.lock();
				if ((! m_ImageProviderDroneSerial.empty()) && (m_ImageProviderDrone == nullptr)) {
					//We haven't been able to get a pointer to the desired drone yet and register a callback. Try now
					m_ImageProviderDrone = DroneInterface::DroneManager::Instance().GetDrone(m_ImageProviderDroneSerial);
					if (m_ImageProviderDrone != nullptr) {
						//We connected to the drone - register an imagery callback
						m_DroneImageCallbackHandle = m_ImageProviderDrone->RegisterCallback([this](cv::Mat const & Frame, TimePoint const & Timestamp) {
							std::scoped_lock lock(m_ImageProviderMutex, m_shadowMapMutex);
							//Only queue frames when we have what we need to process them or the buffer could grow without end
							if ((! m_ReferenceFrame.empty()) && (m_Fiducials.size() >= 3U)) {
								cv::Mat frameCopy;
								Frame.copyTo(frameCopy);
								m_unprocessedFrames.push_back(std::make_tuple(frameCopy, Timestamp));
							}
						});

						//If the module was started on a real drone, enable auto-saving of shadow map histories
						DroneInterface::RealDrone * realDronePtr = dynamic_cast<DroneInterface::RealDrone *>(m_ImageProviderDrone);
						if (realDronePtr != nullptr)
							m_autosaveOnStop = true;
					}
				}
				
				m_shadowMapMutex.lock();
				bool refFrameAndFiducialsSet = (! m_ReferenceFrame.empty()) && (m_Fiducials.size() >= 3U);
				m_shadowMapMutex.unlock();

				if ((! m_unprocessedFrames.empty()) && refFrameAndFiducialsSet) {
					//Get the first unprocessed frame and timestamp - remove from queue.
					cv::Mat frame = std::get<0>(m_unprocessedFrames[0]);
					TimePoint timestamp = std::get<1>(m_unprocessedFrames[0]);
					m_unprocessedFrames.erase(m_unprocessedFrames.begin()); //Will cause vector re-allocation but it's actually not a big deal
					//Due to OpenCV ref counting this doesn't copy the image... it takes ownership of it
					
					bool realtime = true;
					if (m_unprocessedFrames.size() > 0U) {
						realtime = false;
						std::cerr << "Warning: Shadow Detection module falling behind real-time. ";
						std::cerr << (unsigned int) m_unprocessedFrames.size() << " frames buffered.\r\n"; 
					}
					
					m_ImageProviderMutex.unlock(); //Done modifying m_unprocessedFrames - release lock
					
					//Process the first unprocessed frame.
					ProcessFrame(frame, timestamp);
					
					if (! realtime)
						std::cerr << "Frame processed.\r\n";
				}
				else {
					m_ImageProviderMutex.unlock();
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
			}
			else
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	
	inline bool ShadowDetectionEngine::IsReferenceFrameSet(void) {
		std::scoped_lock lock(m_shadowMapMutex);
		return (! m_ReferenceFrame.empty());
	}
	
	inline size_t ShadowDetectionEngine::GetNumberOfFiducials(void) {
		std::scoped_lock lock(m_shadowMapMutex);
		return m_Fiducials.size();
	}
	
	inline bool ShadowDetectionEngine::GetTimestampOfMostRecentShadowMap(TimePoint & Timestamp) {
		std::scoped_lock lock(m_shadowMapMutex);
		if ((m_ShadowMap.Map.rows == 0) || (m_ShadowMap.Map.cols == 0))
			return false;
		Timestamp = m_ShadowMap.Timestamp;
		return true;
	}
	
	inline bool ShadowDetectionEngine::GetMostRecentShadowMap(InstantaneousShadowMap & ShadowMap) {
		std::scoped_lock lock(m_shadowMapMutex);
		if ((m_ShadowMap.Map.rows == 0) || (m_ShadowMap.Map.cols == 0))
			return false;
		ShadowMap = m_ShadowMap;
		return true;
	}
	
	inline void ShadowDetectionEngine::SaveAndFlushShadowMapHistory(void) {
		//Save each item of m_History to disk (each is a new FRF file)
		//We will save all FRF files for the mission to a subdirectory of "Shadow Map Files" in the executable directory
		//The subdirectories will be named "Mission N (Date)"
		
		//Get the first free mission number 
		std::filesystem::path TargetDir = Handy::Paths::ThisExecutableDirectory() / "Shadow Map Files";
		std::error_code ec;
		if (! std::filesystem::exists(TargetDir))
			std::filesystem::create_directory(TargetDir, ec);
		std::vector<std::filesystem::path> subDirs = Handy::SubDirectories(TargetDir);
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
		std::scoped_lock lock(m_shadowMapMutex);
		for (size_t n = 0U; n < m_History.size(); n++) {
			std::filesystem::path Filepath = MissionFolderPath / ("Shadow Map Sequence "s + std::to_string(n + 1U) + ".frf"s);
			std::cout<< "Saving to " << Filepath << std::endl;
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




