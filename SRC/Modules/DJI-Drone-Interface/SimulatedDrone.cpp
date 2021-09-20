//The drone interface module provides the software interface to DJI drones, connected over network sockets
//This particular source file defines a simulated drone for testing and development purposes
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//System Includes

//External Includes

//Project Includes
#include "Drone.hpp"
#include "../../Utilities.hpp"
#include "../../Maps/MapUtils.hpp"
#include "../../UI/VehicleControlWidget.hpp"
#include "../Guidance/Guidance.hpp"
#include "DroneUtils.hpp"

#define PI 3.14159265358979

//DroneInterface::Drone::TimePoint InitTimepoint = std::chrono::steady_clock::now(); //Used for testing message age warnings

static double sgn(double val) {
    if (val < 0.0) return -1.0;
    if (val > 0.0) return  1.0;
    return 0.0;
}

namespace DroneInterface {
	SimulatedDrone::SimulatedDrone() : m_abort(false) {
		m_MainThread = std::thread(&SimulatedDrone::DroneMain, this); //Launch private thread
	}
	
	SimulatedDrone::SimulatedDrone(std::string Serial, Eigen::Vector3d const & Position_LLA) : SimulatedDrone() {
		std::scoped_lock lock(m_mutex);
		
		m_serial = Serial; //Save Serial String
		
		//Set position
		m_Lat = Position_LLA(0);
		m_Lon = Position_LLA(1);
		m_Alt = Position_LLA(2);
		
		//Set velocity
		m_V_North = 0.0;
		m_V_East  = 0.0;
		m_V_Down  = 0.0;
		
		//Set orientation
		m_yaw = 0.0;
		m_pitch = 0.0;
		m_roll = 0.0;
		
		//Set home location
		m_HomeLat = Position_LLA(0);
		m_HomeLon = Position_LLA(1);
		
		//Start with a full battery
		m_battLevel = 1.0;
		
		//Drone initialized with position will initially be on the ground
		m_flightMode = 0;
		
		m_groundAlt = Position_LLA(2);
		
		//Initialize timestamps
		m_LastVSCommand_ModeA_Timestamp = std::chrono::steady_clock::now();
		m_LastVSCommand_ModeB_Timestamp = std::chrono::steady_clock::now();
		m_LastPoseUpdate = std::chrono::steady_clock::now();
	}
	
	SimulatedDrone::~SimulatedDrone() {
		m_abort = true;
		m_VideoProcessingThreadAbort = true;
		
		if (m_VideoProcessingThread.joinable())
			m_VideoProcessingThread.join();
		if (m_MainThread.joinable())
			m_MainThread.join();
	}
	
	//Simulated drones can do all there initialization in the constructor, so they are ready immediately on construction
	bool SimulatedDrone::Ready(void) {
		return true;
	}
	
	//Get drone serial number as a string (should be available on construction)
	std::string SimulatedDrone::GetDroneSerial(void) {
		std::scoped_lock lock(m_mutex);
		return m_serial;
	}
	
	//Lat & Lon (radians) and WGS84 Altitude (m)
	bool SimulatedDrone::GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Latitude  = m_Lat;
		Longitude = m_Lon;
		Altitude  = m_Alt;
		Timestamp = std::chrono::steady_clock::now();
		//Timestamp = InitTimepoint;
		return true;
	}
	
	//NED velocity vector (m/s)
	bool SimulatedDrone::GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		V_North   = m_V_North;
		V_East    = m_V_East;
		V_Down    = m_V_Down;
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Yaw, Pitch, Roll (radians) using DJI definitions
	bool SimulatedDrone::GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Yaw   = m_yaw;
		Pitch = m_pitch;
		Roll  = m_roll;
		Timestamp = std::chrono::steady_clock::now();
		//Timestamp = InitTimepoint;
		return true;
	}
	
	//Barometric height above ground (m) - Drone altitude minus takeoff altitude
	bool SimulatedDrone::GetHAG(double & HAG, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		HAG = m_Alt - m_groundAlt;
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Drone Battery level (0 = Empty, 1 = Full)
	bool SimulatedDrone::GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		BattLevel = m_battLevel;
		//BattLevel = 0.19;
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Whether the drone has hit height or radius limits
	bool SimulatedDrone::GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		MaxHAG = false;
		MaxDistFromHome = false;
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Wind & other vehicle warnings as strings
	bool SimulatedDrone::GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		ActiveWarnings.clear();
		//ActiveWarnings.push_back("Warning 1"s);
		//ActiveWarnings.push_back("Warning 2"s);
		//ActiveWarnings.push_back("Warning 3"s);
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//GNSS status (-1 for none, 0-5: DJI definitions)
	bool SimulatedDrone::GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		SatCount    = 13U;
		SignalLevel = 5;
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Returns true if recognized DJI camera is present - Should be available on construction
	bool SimulatedDrone::IsDJICamConnected(void) {
		std::scoped_lock lock(m_mutex);
		return true;
	}
	
	//True if receiving imagery from drone, false otherwise (valid on construction... initially returns false)
	bool SimulatedDrone::IsCamImageFeedOn(void) {
		std::scoped_lock lock(m_mutex);
		return m_imageFeedActive;
	}
	
	//Start sending frames of live video (as close as possible to the given framerate (frame / s))
	void SimulatedDrone::StartDJICamImageFeed(double TargetFPS) {
		std::scoped_lock lock(m_mutex);
		//If there is already a video processing thread, kill it
		if (m_VideoProcessingThread.joinable()) {
			m_VideoProcessingThreadAbort = true;
			m_VideoProcessingThread.join();
		}
		
		//Initialize protected fields
		m_NextFrameMutex.lock();
		m_NextFrame = cv::Mat();
		m_NextFrameReady = false;
		m_VideoFileReadFinished = false;
		m_NextFrameMutex.unlock();
		
		//If the source video hasn't been set or doesn't exist on disk, abort here with a message
		if ((m_videoPath.empty()) || (! std::filesystem::exists(m_videoPath))) {
			std::cerr << "Error in SimulatedDrone::StartDJICamImageFeed() - Source video path empty or file doesn't exist.\r\n";
			return;
		}
		
		//Tell main thread to start receiving imagery and initialize other fields
		m_VideoProcessingThreadAbort = false;
		m_targetFPS = TargetFPS;
		m_Frame = cv::Mat();
		m_FrameNumber = 0U;
		m_VideoFeedStartTimestamp = std::chrono::steady_clock::now();
		m_FrameTimestamp = std::chrono::steady_clock::now();
		m_imageFeedActive = true;
		
		std::string videoPathStr = m_videoPath.string();
		m_VideoProcessingThread = std::thread([videoPathStr, TargetFPS, this](){
			std::cerr << "Video Processing thread started.\r\n";
			
			cv::VideoCapture myCap(videoPathStr);
			if (! myCap.isOpened()) {
				std::cerr << "Error in StartDJICamImageFeed(): Unable to open video file. Not starting.\r\n";
				return;
			}
			double videoFileFPS = myCap.get(cv::CAP_PROP_FPS);
			
			//Get first frame
			unsigned int fileFrameNum = 0U;
			unsigned int outputFrameNum = 0U;
			cv::Mat NextFrame;
			if (myCap.grab() && (myCap.retrieve(NextFrame)) && (! NextFrame.empty()) && ResizeTo720p(NextFrame)) {
				std::scoped_lock lock(m_NextFrameMutex);
				m_NextFrame = NextFrame;
				m_NextFrameReady = true;
			}
			else {
				//We have reached the end of the video file (or for some other reason we can't read any farther)
				std::cerr << "Warning in StartDJICamImageFeed(): Failed to retrieve first frame.\r\n";
				std::scoped_lock lock(m_NextFrameMutex);
				m_VideoFileReadFinished = true;
				return;
			}
			
			//When m_NextFrameReady goes false, get the next frame. Do this until the video file has been fully read.
			while (true) {
				if (m_VideoProcessingThreadAbort)
					return;
				bool go = false;
				m_NextFrameMutex.lock();
				go = ! m_NextFrameReady;
				m_NextFrameMutex.unlock();
				if (! go) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					continue;
				}
				
				//Time to get the next frame
				double targetNextFrameTime = double(outputFrameNum + 1U)/TargetFPS;
				//int skipCounter = 0;
				while (double(fileFrameNum)/videoFileFPS < targetNextFrameTime) {
					//skipCounter++;
					if (myCap.grab())
						fileFrameNum++;
					else {
						//We have reached the end of the video file (or for some other reason we can't read any farther)
						std::scoped_lock lock(m_NextFrameMutex);
						m_VideoFileReadFinished = true;
						return;
					}
				}
				//std::cerr << "Skipped " << skipCounter << " frames.\r\n";
				if (myCap.grab() && (myCap.retrieve(NextFrame)) && (! NextFrame.empty()) && ResizeTo720p(NextFrame)) {
					fileFrameNum++;
					outputFrameNum++;
					std::scoped_lock lock(m_NextFrameMutex);
					m_NextFrame = NextFrame;
					m_NextFrameReady = true;
					//std::cerr << "Frame decoded.\r\n";
				}
				else {
					//We have reached the end of the video file (or for some other reason we can't read any farther)
					std::scoped_lock lock(m_NextFrameMutex);
					m_VideoFileReadFinished = true;
					return;
				}
			}
		});
	}
	
	//Stop sending frames of live video
	void SimulatedDrone::StopDJICamImageFeed(void) {
		std::scoped_lock lock(m_mutex);
		if (m_VideoProcessingThread.joinable()) {
			m_VideoProcessingThreadAbort = true;
			m_VideoProcessingThread.join();
		}
		m_imageFeedActive = false;
	}
	
	bool SimulatedDrone::GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		if ((m_Frame.rows <= 0) || (m_Frame.cols <= 0))
			return false;
		else {
			Frame = m_Frame;
			FrameNumber = m_FrameNumber;
			Timestamp = m_FrameTimestamp;
			return true;
		}
	}
	
	//Regester callback for new frames
	int SimulatedDrone::RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) {
		std::scoped_lock lock(m_mutex);
		int token = 0;
		while (m_ImageryCallbacks.count(token) > 0U)
			token++;
		m_ImageryCallbacks[token] = Callback;
		return token;
	}

	//Unregister callback for new frames (input is token returned by RegisterCallback()
	void SimulatedDrone::UnRegisterCallback(int Handle) {
		std::scoped_lock lock(m_mutex);
		m_ImageryCallbacks.erase(Handle);
	}
	
	//Populate Result with whether or not the drone is currently flying (in any mode)
	bool SimulatedDrone::IsCurrentlyFlying(bool & Result, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		
		Result = (m_flightMode > 0);
		//double t = SecondsSinceT0Epoch(std::chrono::steady_clock::now());
		//Result = (fmod(t, 10.0) < 5.0);
		
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Get flight mode as a human-readable string
	bool SimulatedDrone::GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		switch (m_flightMode) {
			case 0:  FlightModeStr = "On Ground"s;         break;
			case 1:  FlightModeStr = "Hover (P mode)"s;    break;
			case 2:  FlightModeStr = "Waypoint Mission"s;  break;
			case 3:  FlightModeStr = "Virtual Stick (A)"s; break;
			case 4:  FlightModeStr = "Virtual Stick (B)"s; break;
			case 5:  FlightModeStr = "Taking Off"s;        break;
			case 6:  FlightModeStr = "Landing"s;           break;
			case 7:  FlightModeStr = "Returning Home"s;    break;
			default: FlightModeStr = "Unknown"s;           break;
		}
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Stop current mission, if running. Then load, verify, and start new waypoint mission.
	void SimulatedDrone::ExecuteWaypointMission(WaypointMission & Mission) {
		std::scoped_lock lock(m_mutex);
		m_LastMission = Mission;
		
		//Set state to takeoff if necessary. Just go to first waypoint if already in the air
		if (m_flightMode == 0)
			m_waypointMissionState = 0;
		else {
			if (m_LastMission.CurvedTrajectory)
				m_waypointMissionState = 4;
			else
				m_waypointMissionState = 1;
		}
		m_targetWaypoint = 0;
		m_flightMode = 2;
	}
	
	//Populate Result with whether or not a waypoint mission is currently being executed
	bool SimulatedDrone::IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Result = (m_flightMode == 2);
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Populate arg with current mission (returns false if not flying waypoint mission)
	bool SimulatedDrone::GetCurrentWaypointMission(WaypointMission & Mission) {
		std::scoped_lock lock(m_mutex);
		if (m_flightMode == 2) {
			Mission = m_LastMission;
			return true;
		}
		else
			return false;
	}
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void SimulatedDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command) {
		std::scoped_lock lock(m_mutex);
		m_flightMode = 3;
		m_LastVSCommand_ModeA = Command;
		m_LastVSCommand_ModeA_Timestamp = std::chrono::steady_clock::now();
	}
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void SimulatedDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command) {
		std::scoped_lock lock(m_mutex);
		m_flightMode = 4;
		m_LastVSCommand_ModeB = Command;
		m_LastVSCommand_ModeB_Timestamp = std::chrono::steady_clock::now();
	}
	
	//Stop any running missions an leave virtualStick mode (if in it) and hover in place (P mode)
	void SimulatedDrone::Hover(void) {
		std::scoped_lock lock(m_mutex);
		if (m_flightMode == 0)
			m_flightMode = 5; //Takeoff
		else
			m_flightMode = 1; //Hover
	}
	
	//Initiate landing sequence immediately at current vehicle location
	void SimulatedDrone::LandNow(void) {
		std::scoped_lock lock(m_mutex);
		m_flightMode = 6;
	}
	
	//Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location
	void SimulatedDrone::GoHomeAndLand(void) {
		std::scoped_lock lock(m_mutex);
		m_flightMode = 7;
	}
	
	//True: Imagery will be provided at close-to-real-time rate. False: Imagery is provided as fast as possible
	void SimulatedDrone::SetRealTime(bool Realtime) {
		std::scoped_lock lock(m_mutex);
		m_realtime = Realtime;
	}
	
	//Should be set before calling StartDJICamImageFeed()
	void SimulatedDrone::SetSourceVideoFile(std::filesystem::path const & VideoPath) {
		std::scoped_lock lock(m_mutex);
		m_videoPath = VideoPath;
	}
	
	std::filesystem::path SimulatedDrone::GetSourceVideoFile(void) {
		std::scoped_lock lock(m_mutex);
		return m_videoPath;
	}
	
	//Get a single frame - will fail if the video feed is running
	bool SimulatedDrone::GetReferenceFrame(double SecondsIntoVideo, cv::Mat & Frame) {
		std::scoped_lock lock(m_mutex);
		
		if (m_imageFeedActive)
			return false;
		
		cv::VideoCapture myCap(m_videoPath.string());
		if (! myCap.isOpened())
			return false;
		
		double refFrameNum = std::round(SecondsIntoVideo * myCap.get(cv::CAP_PROP_FPS));
		if (refFrameNum >= myCap.get(cv::CAP_PROP_FRAME_COUNT) - 1.0)
			return false;
		
		myCap.set(cv::CAP_PROP_POS_FRAMES, refFrameNum);
		return (myCap.grab() && (myCap.retrieve(Frame)) && (! Frame.empty()) && ResizeTo720p(Frame));
	}
	
	//Function for internal thread managing drone object
	void SimulatedDrone::DroneMain(void) {
		while (! m_abort) {
			m_mutex.lock();
			
			//Update the drones position and orientation based on current mode, state, and objective
			UpdateDronePose();
			
			if (! m_imageFeedActive) {
				m_mutex.unlock();
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}
			
			if (m_realtime) {
				//If not enough time has passed since our last dispatch, keep waiting
				TimePoint FrameDispatchTime = m_VideoFeedStartTimestamp + std::chrono::milliseconds(uint64_t(1000.0*(double(m_FrameNumber)/m_targetFPS)));
				if (std::chrono::steady_clock::now() < FrameDispatchTime) {
					m_mutex.unlock();
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					continue;
				}
			}
			
			{
				//See if there is a frame ready for us
				m_NextFrameMutex.lock();
				if (m_NextFrameReady) {
					m_Frame = m_NextFrame;
					m_NextFrameReady = false;
					m_NextFrameMutex.unlock();
					m_FrameNumber++;
					m_FrameTimestamp = std::chrono::steady_clock::now();
					for (auto const & kv : m_ImageryCallbacks)
						kv.second(m_Frame, m_FrameTimestamp);
				}
				else if (m_VideoFileReadFinished) {
					m_NextFrameMutex.unlock();
					m_imageFeedActive = false;
				}
				else {
					m_NextFrameMutex.unlock();
					m_mutex.unlock();
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					continue;
				}
			}
			m_mutex.unlock();
		}
	}
	
	//Get the value that should be added to Yaw to get TargetYaw. Returns a positive or negative number... whichever has lesser
	//magnitude and results in the correct result, modulo 2 PI.
	static double GetYawDelta(double Yaw, double TargetYaw) {
		double delta1 = TargetYaw - Yaw;
		
		//Map to range [0, 2 PI]
		delta1 = fmod(delta1, 2*PI);
		if (delta1 < 0.0)
			delta1 += 2*PI;
		
		double delta2 = delta1 - 2.0*PI;
		if (fabs(delta1) < fabs(delta2))
			return delta1;
		else
			return delta2;
	}
	
	//Updates m_Lat, m_Lon
	void SimulatedDrone::UpdateDrone2DPositionBasedOnVelocity(double deltaT, Eigen::Matrix3d const & C_ENU_ECEF) {
		//Update 2D Position
		Eigen::Vector3d P_ECEF = LLA2ECEF(Eigen::Vector3d(m_Lat, m_Lon, m_Alt));
		Eigen::Vector3d V_ECEF = C_ENU_ECEF * Eigen::Vector3d(m_V_East, m_V_North, 0.0);
		P_ECEF += deltaT*V_ECEF;
		Eigen::Vector3d LLA = ECEF2LLA(P_ECEF);
		m_Lat = LLA(0);
		m_Lon = LLA(1);
	}
	
	//Updates m_Alt, m_V_Down
	void SimulatedDrone::UpdateDroneVertChannelBasedOnTargetHAG(double deltaT, double TargetHAG, double climbRate, double descentRate) {
		double HAG = m_Alt - m_groundAlt;
		double HAGDelta = TargetHAG - HAG; //Target - Current HAG
		double targetHAGChange = 0.5*HAGDelta;
		double maxHAGChange = (targetHAGChange > 0.0) ? deltaT*climbRate : deltaT*descentRate;
		double HAGChange = targetHAGChange;
		if (fabs(HAGChange) > maxHAGChange)
			HAGChange = sgn(HAGChange)*maxHAGChange;
		m_Alt += HAGChange;
		m_V_Down = -1.0*HAGChange/deltaT;
	}
	
	//Updates m_yaw, m_pitch, m_roll. Pitch/Roll are just kept at 0 to mimic level, steady flight (even though this is not accurate when moving)
	//Returns the absolute value of the deviation between current and target yaw (in radians).
	double SimulatedDrone::UpdateDroneOrientationBasedOnYawTarget(double deltaT, double TargetYaw, double turnRate) {
		double yawDelta = GetYawDelta(m_yaw, TargetYaw);
		double targetYawChange = 0.5*yawDelta;
		double maxYawChange = deltaT*turnRate;
		double YawChange = targetYawChange;
		if (fabs(YawChange) > maxYawChange)
			YawChange = sgn(YawChange)*maxYawChange;
		m_yaw += YawChange;
		m_pitch = 0.0;
		m_roll = 0.0;
		
		//Map Yaw to range [0, 2 PI]
		m_yaw = fmod(m_yaw, 2*PI);
		if (m_yaw < 0.0)
			m_yaw += 2*PI;
		
		return fabs(yawDelta);
	}
	
	//Updates m_V_East, m_V_North
	void SimulatedDrone::Update2DVelocityBasedOnTarget(double deltaT, Eigen::Vector2d const & V_Target_EN, double max2DAcc, double max2DDec, double max2DSpeed) {
		Eigen::Vector2d V_EN(m_V_East, m_V_North);
		double maxVChange = (V_Target_EN.norm() > V_EN.norm()) ? deltaT*max2DAcc : deltaT*max2DDec;
		Eigen::Vector2d V_Change_EN = 0.5*(V_Target_EN - V_EN);
		if (V_Change_EN.norm() > maxVChange)
			V_Change_EN = V_Change_EN * maxVChange / V_Change_EN.norm();
		V_EN += V_Change_EN;
		if (V_EN.norm() > max2DSpeed)
			V_EN = V_EN * max2DSpeed / V_EN.norm();
		m_V_East  = V_EN(0);
		m_V_North = V_EN(1);
	}
	
	//Compute the desired EN velocity vector needed to move to and/or hold a given position
	//TargetLat and TargetLon are in radians.
	//TargetMoveSpeed is in m/s.
	//V_Target_EN is in m/s
	void SimulatedDrone::ComputeTarget2DVelocityBasedOnTargetPosAndSpeed(double TargetLat, double TargetLon, double TargetMoveSpeed,
	                                                                     Eigen::Vector2d & V_Target_EN, Eigen::Matrix3d const & C_ECEF_ENU) {
		Eigen::Vector3d curentPos_ECEF = LLA2ECEF(Eigen::Vector3d(m_Lat, m_Lon, m_Alt));
		Eigen::Vector3d targetPos_ECEF = LLA2ECEF(Eigen::Vector3d(TargetLat, TargetLon, m_Alt));
		Eigen::Vector3d delta_ECEF     = targetPos_ECEF - curentPos_ECEF;
		Eigen::Vector3d delta_ENU      = C_ECEF_ENU * delta_ECEF;
		Eigen::Vector2d V_EN(delta_ENU(0), delta_ENU(1));
		double distFromTarget          = V_EN.norm();
		V_EN.normalize();
		
		//We will set our East-North velocity to a multiple of V_EN... we just need to scale it appropriately
		V_Target_EN = V_EN;
		double maxDecel = 2.5; //m/s/s - measured on Inspire 1
		double stoppingDist = TargetMoveSpeed*TargetMoveSpeed / (2.0 * maxDecel);
		if (distFromTarget > stoppingDist)
			V_Target_EN *= TargetMoveSpeed;
		else {
			//On final approach - slowing down (we command < the speed corresponding to a perfect stop to get ahead of system latency)
			V_Target_EN *= 0.9*std::sqrt(2.0 * maxDecel * distFromTarget);
			
			//Re-shape and soften commanded V near 0 to avoid oscillations
			if (V_Target_EN.norm() < 1.0)
				V_Target_EN = V_Target_EN * V_Target_EN.norm();
		}
	}
	
	//Update the drones position and orientation based on current mode, state, and objective (Called in loop in DroneMain())
	void SimulatedDrone::UpdateDronePose(void) {
		double max2DAcc = 2.5;    //Positive number (m/s/s)
		double max2DDec = 2.5;    //Positive number (m/s/s)
		double max2DSpeed = 15.1; //Positive number (m/s)
		double climbRate = 2.6;   //Positive number (m/s)
		double descentRate = 3.2; //Positive number (m/s)
		double turnRate = 1.2;    //Positive number (rad/s)
		
		double HAG = m_Alt - m_groundAlt;
		Eigen::Matrix3d C_ECEF_ENU = latLon_2_C_ECEF_ENU(m_Lat, m_Lon);
		Eigen::Matrix3d C_ENU_ECEF = C_ECEF_ENU.transpose();
		TimePoint now = std::chrono::steady_clock::now();
		double deltaT = SecondsElapsed(m_LastPoseUpdate, now); 
		m_LastPoseUpdate = now;
		
		//Update drone battery level
		double dischargeRate = (m_flightMode > 0) ? 7.2464e-04 : 1.5e-04; //Units per second
		m_battLevel = std::max(m_battLevel - deltaT * dischargeRate, 0.0);
		
		//First update 2D position (all flying modes)
		if (m_flightMode > 0)
			UpdateDrone2DPositionBasedOnVelocity(deltaT, C_ENU_ECEF);
		
		if (m_flightMode == 1) {
			//P (Hover) mode
			Update2DVelocityBasedOnTarget(deltaT, Eigen::Vector2d(0.0, 0.0), max2DAcc, max2DDec, max2DSpeed);
		}
		if (m_flightMode == 2) {
			//Currently flying a waypoint mission - this is a big state machine that depends on m_targetWaypoint and m_waypointMissionState
			if ((m_targetWaypoint < 0) || (m_targetWaypoint >= (int) m_LastMission.Waypoints.size())) {
				//The waypoint is invalid or the mission is over
				if (m_LastMission.LandAtLastWaypoint)
					m_flightMode = 6;
				else
					m_flightMode = 1;
			}
			else if (m_waypointMissionState == 0) {
				//Taking off and reaching altitude of first waypoint
				double targetHAG = m_LastMission.Waypoints[0].Altitude - m_groundAlt;
				UpdateDroneVertChannelBasedOnTargetHAG(deltaT, targetHAG, climbRate, descentRate);
				Update2DVelocityBasedOnTarget(deltaT, Eigen::Vector2d(0.0, 0.0), max2DAcc, max2DDec, max2DSpeed);
				if (fabs(m_LastMission.Waypoints[0].Altitude - m_Alt) < 0.5) {
					if (m_LastMission.CurvedTrajectory)
						m_waypointMissionState = 4;
					else
						m_waypointMissionState = 1;
				}
			}
			else if (m_waypointMissionState == 1) {
				//Goto waypoint (P2P)
				Waypoint const & waypoint(m_LastMission.Waypoints[m_targetWaypoint]);
				UpdateDroneVertChannelBasedOnTargetHAG(deltaT, waypoint.Altitude - m_groundAlt, climbRate, descentRate);
				
				double speed = 0.0; //m/s
				if (m_targetWaypoint == 0)
					speed = m_LastMission.Waypoints[0].Speed;
				else
					speed = m_LastMission.Waypoints[m_targetWaypoint - 1].Speed;
				Eigen::Vector2d V_Target_EN;
				ComputeTarget2DVelocityBasedOnTargetPosAndSpeed(waypoint.Latitude, waypoint.Longitude, speed, V_Target_EN, C_ECEF_ENU);
				Update2DVelocityBasedOnTarget(deltaT, V_Target_EN, max2DAcc, max2DDec, max2DSpeed);
				
				if (V_Target_EN.norm() > 0.1) {
					double targetYaw = std::atan2(V_Target_EN(0), V_Target_EN(1));
					UpdateDroneOrientationBasedOnYawTarget(deltaT, targetYaw, turnRate);
				}
				
				//Detirmine when we have reached the waypoint and update state accordingly
				Eigen::Vector3d P1 = LLA2ECEF(Eigen::Vector3d(m_Lat, m_Lon, m_Alt));
				Eigen::Vector3d P2 = LLA2ECEF(Eigen::Vector3d(waypoint.Latitude, waypoint.Longitude, m_Alt));
				if ((P2 - P1).norm() < 0.25) {
					m_waypointMissionState = 2;
					m_arrivalAtWaypoint_Timestamp = std::chrono::steady_clock::now();
				}
			}
			else if (m_waypointMissionState == 2) {
				//Pause at waypoint (P2P)
				Update2DVelocityBasedOnTarget(deltaT, Eigen::Vector2d(0.0, 0.0), max2DAcc, max2DDec, max2DSpeed);
				if (SecondsElapsed(m_arrivalAtWaypoint_Timestamp) > 2.0)
					m_waypointMissionState = 3;
			}
			else if (m_waypointMissionState == 3) {
				//turning at waypoint (P2P)
				if (m_targetWaypoint + 1 >= (int) m_LastMission.Waypoints.size()) {
					//We are at the last waypoint
					if (m_LastMission.LandAtLastWaypoint)
						m_flightMode = 6;
					else
						m_flightMode = 1;
				}
				else {
					//We are not at the last waypoint - turn to the next one
					//Get EN vector to next waypoint (cheat by using our Compute2DVelocity function)
					Waypoint const & nextWaypoint(m_LastMission.Waypoints[m_targetWaypoint + 1]);
					Eigen::Vector2d V_Target_EN;
					ComputeTarget2DVelocityBasedOnTargetPosAndSpeed(nextWaypoint.Latitude, nextWaypoint.Longitude, 10.0, V_Target_EN, C_ECEF_ENU);
					
					if (V_Target_EN.norm() > 0.1) {
						double targetYaw = std::atan2(V_Target_EN(0), V_Target_EN(1));
						double delta = UpdateDroneOrientationBasedOnYawTarget(deltaT, targetYaw, turnRate);
						if (delta < 2.0 * PI/180.0) {
							m_targetWaypoint++;
							m_waypointMissionState = 1;
						}
					}
					else {
						m_targetWaypoint++;
						m_waypointMissionState = 1;
					}
				}
				
			}
			else if (m_waypointMissionState == 4) {
				//Goto waypoint (curved)
				Waypoint const & waypoint(m_LastMission.Waypoints[m_targetWaypoint]);
				UpdateDroneVertChannelBasedOnTargetHAG(deltaT, waypoint.Altitude - m_groundAlt, climbRate, descentRate);
				
				double speed = 0.0; //m/s
				if (m_targetWaypoint == 0)
					speed = m_LastMission.Waypoints[0].Speed;
				else
					speed = m_LastMission.Waypoints[m_targetWaypoint - 1].Speed;
				Eigen::Vector2d V_Target_EN;
				ComputeTarget2DVelocityBasedOnTargetPosAndSpeed(waypoint.Latitude, waypoint.Longitude, speed, V_Target_EN, C_ECEF_ENU);
				Update2DVelocityBasedOnTarget(deltaT, V_Target_EN, max2DAcc, max2DDec, max2DSpeed);
				
				if (V_Target_EN.norm() > 0.1) {
					double targetYaw = std::atan2(V_Target_EN(0), V_Target_EN(1));
					UpdateDroneOrientationBasedOnYawTarget(deltaT, targetYaw, turnRate);
				}
				
				//When we get close enough to the waypoint change modes
				Eigen::Vector3d P1 = LLA2ECEF(Eigen::Vector3d(m_Lat, m_Lon, m_Alt));
				Eigen::Vector3d P2 = LLA2ECEF(Eigen::Vector3d(waypoint.Latitude, waypoint.Longitude, m_Alt));
				if (m_targetWaypoint + 1 >= (int) m_LastMission.Waypoints.size()) {
					//We are at the last waypoint
					if ((P2 - P1).norm() < 0.25)
						m_targetWaypoint++; //This will trigger the mission ending on next pass through loop
				}
				else {
					if ((P2 - P1).norm() < std::max((double) waypoint.CornerRadius, 0.25)) {
						m_waypointMissionState = 5;
						m_turningSpeedThroughWaypoint = std::sqrt(m_V_North*m_V_North + m_V_East*m_V_East);
					}
				}
			}
			else if (m_waypointMissionState == 5) {
				//turn through waypoint (curved)
				Waypoint const & waypoint(m_LastMission.Waypoints[m_targetWaypoint]);
				Waypoint const & nextWaypoint(m_LastMission.Waypoints[m_targetWaypoint + 1]); //We should never be in this state for the last waypoint
				
				//Computing the actual ideal arc to follow is overkill here. Instead, we aim for a point between the current waypoint
				//and the next one (a distance of CornerRadius away from the current waypoint) and update our orientation to point
				//between the waypoints. We allow for above-limit accel/decel to reduce overshoot and when we transition states to
				//head to the next waypoint we kill off any velocity that is orthogonal to the line between the waypoints. This
				//fully eliminates overshoot. Technically we are breaking the laws of physics here (which we already do with the vertical
				//channel and orientation) but the violation is relatively minor and gives very realistic looking trajectories.
				
				Eigen::Vector3d P1_ECEF = LLA2ECEF(Eigen::Vector3d(waypoint.Latitude, waypoint.Longitude, m_Alt));
				Eigen::Vector3d P2_ECEF = LLA2ECEF(Eigen::Vector3d(nextWaypoint.Latitude, nextWaypoint.Longitude, m_Alt));
				Eigen::Vector3d V_ECEF = P2_ECEF - P1_ECEF;
				V_ECEF.normalize();
				Eigen::Vector3d V_ENU = C_ECEF_ENU * V_ECEF;
				Eigen::Vector3d P3_ECEF = P1_ECEF + waypoint.CornerRadius * V_ECEF;
				Eigen::Vector3d Target_LLA = ECEF2LLA(P3_ECEF);
				
				Eigen::Vector2d V_Target_EN;
				ComputeTarget2DVelocityBasedOnTargetPosAndSpeed(Target_LLA(0), Target_LLA(1), m_turningSpeedThroughWaypoint, V_Target_EN, C_ECEF_ENU);
				Update2DVelocityBasedOnTarget(deltaT, V_Target_EN, 2.0*max2DAcc, 2.0*max2DDec, max2DSpeed);
				
				if (V_ENU.norm() > 0.1) {
					double targetYaw = std::atan2(V_ENU(0), V_ENU(1));
					UpdateDroneOrientationBasedOnYawTarget(deltaT, targetYaw, turnRate);
				}
				
				Eigen::Vector3d currentVel_ENU(m_V_East, m_V_North, -1.0*m_V_Down);
				Eigen::Vector3d orthVel = currentVel_ENU - currentVel_ENU.dot(V_ENU)*V_ENU;
				if (orthVel.norm() < 0.25 * m_turningSpeedThroughWaypoint) {
					currentVel_ENU = currentVel_ENU.dot(V_ENU)*V_ENU;
					m_V_North = currentVel_ENU(1);
					m_V_East  = currentVel_ENU(0);
					m_V_Down  = -1.0*currentVel_ENU(2);
					m_waypointMissionState = 4;
					m_targetWaypoint++;
				}
			}
			
		}
		else if (m_flightMode == 3) {
			//Currently in Virtual Stick Mode A
			
			//Update altitude and vertical velocity
			UpdateDroneVertChannelBasedOnTargetHAG(deltaT, m_LastVSCommand_ModeA.HAG, climbRate, descentRate);
			
			//Update orientation
			UpdateDroneOrientationBasedOnYawTarget(deltaT, m_LastVSCommand_ModeA.Yaw, turnRate);
			
			//Update 2D Velocity
			Update2DVelocityBasedOnTarget(deltaT, Eigen::Vector2d(m_LastVSCommand_ModeA.V_East, m_LastVSCommand_ModeA.V_North), max2DAcc, max2DDec, max2DSpeed);
			
			//Zero out velocity components if last command is too old
			if (SecondsElapsed(m_LastVSCommand_ModeA_Timestamp, std::chrono::steady_clock::now()) > m_LastVSCommand_ModeA.timeout) {
				m_LastVSCommand_ModeA.V_East = 0.0;
				m_LastVSCommand_ModeA.V_North = 0.0;
			}
		}
		else if (m_flightMode == 4) {
			//Currently in Virtual Stick Mode B. Note: This mode is not tested since it isn't really used in Recon
			
			//Update altitude and vertical velocity
			UpdateDroneVertChannelBasedOnTargetHAG(deltaT, m_LastVSCommand_ModeB.HAG, climbRate, descentRate);
			
			//Update orientation
			UpdateDroneOrientationBasedOnYawTarget(deltaT, m_LastVSCommand_ModeB.Yaw, turnRate);
			
			//Compute target EN velocity from Vehicle-frame velocity. We define the vehicle frame to have X axis through the right wing,
			//Y axis through the vehicle nose, and Z axis up. Note that this is not the typical body frame.
			Eigen::Matrix2d C_Vehicle_EN;
			C_Vehicle_EN <<      cos(m_yaw), sin(m_yaw),
			                -1.0*sin(m_yaw), cos(m_yaw);
			Eigen::Vector2d V_Target_Vehicle(m_LastVSCommand_ModeB.V_Right, m_LastVSCommand_ModeB.V_Forward);
			Eigen::Vector2d V_Target_EN = C_Vehicle_EN * V_Target_Vehicle;
			
			//Update 2D Velocity
			Update2DVelocityBasedOnTarget(deltaT, V_Target_EN, max2DAcc, max2DDec, max2DSpeed);
			
			//Zero out velocity components if last command is too old
			if (SecondsElapsed(m_LastVSCommand_ModeB_Timestamp, std::chrono::steady_clock::now()) > m_LastVSCommand_ModeB.timeout) {
				m_LastVSCommand_ModeB.V_Right = 0.0;
				m_LastVSCommand_ModeB.V_Forward = 0.0;
			}
		}
		else if (m_flightMode == 5) {
			//Taking off
			UpdateDroneVertChannelBasedOnTargetHAG(deltaT, 5.0, climbRate, descentRate);
			Update2DVelocityBasedOnTarget(deltaT, Eigen::Vector2d(0.0, 0.0), max2DAcc, max2DDec, max2DSpeed);
			
			//Once we have reached approximately the target takeoff height, switch to mode P
			if (HAG > 4.9)
				m_flightMode = 1;
		}
		else if (m_flightMode == 6) {
			//Landing now
			UpdateDroneVertChannelBasedOnTargetHAG(deltaT, 0.0, climbRate, descentRate);
			Update2DVelocityBasedOnTarget(deltaT, Eigen::Vector2d(0.0, 0.0), max2DAcc, max2DDec, max2DSpeed);
			
			//Once we have reached approximately the ground height we are done.
			if (HAG < 0.1) {
				m_flightMode = 0;
				m_V_East  = 0.0;
				m_V_North = 0.0;
			}
			
		}
		else if (m_flightMode == 7) {
			//Returning to home
			double RTHSpeed = 5.5; //m/s
			Eigen::Vector2d V_Target_EN;
			ComputeTarget2DVelocityBasedOnTargetPosAndSpeed(m_HomeLat, m_HomeLon, RTHSpeed, V_Target_EN, C_ECEF_ENU);
			Update2DVelocityBasedOnTarget(deltaT, V_Target_EN, max2DAcc, max2DDec, max2DSpeed);
			
			if (V_Target_EN.norm() > 0.1) {
				double targetYaw = std::atan2(V_Target_EN(0), V_Target_EN(1));
				UpdateDroneOrientationBasedOnYawTarget(deltaT, targetYaw, turnRate);
			}
			
			//When we get close enough to the home position, land
			Eigen::Vector3d P1 = LLA2ECEF(Eigen::Vector3d(m_Lat, m_Lon, m_Alt));
			Eigen::Vector3d P2 = LLA2ECEF(Eigen::Vector3d(m_HomeLat, m_HomeLon, m_Alt));
			if ((P2 - P1).norm() < 0.25)
				m_flightMode = 6;
		}
		
		
	}
	
	bool SimulatedDrone::ResizeTo720p(cv::Mat & Frame) {
		if ((Frame.rows == 720) && (Frame.cols == 1280))
			return true;
		if ((Frame.rows == 2160) && (Frame.cols == 3840))
			return Resize_4K_to_720p(Frame);
		return false;
	}
	
	//Drop a 4K m_frame down to 720p - copying Ben's resizing strategy
	bool SimulatedDrone::Resize_4K_to_720p(cv::Mat & Frame) {
		if ((Frame.rows != 2160) && (Frame.cols != 3840))
			return false;
		//Ben's method
		cv::Mat BufA, BufB;
		pyrDown(Frame, BufA);
		pyrDown(BufA,    BufB);
		resize(BufB, Frame, cv::Size(1280, 720));
		return true;
		
		//Bryan's method - this is a bit faster than above, and retains the full sharpness of the imagery. Using PyrDown twice
		//overshoots the target resolution so requires an up-scaling afterwards. PyrDown also tends to filter more aggresively than
		//is necessary. Combined, these result in softer, oversmoothed imagery. Just doing 3x3 block averages avoids this.
		/*if (Frame.type() != CV_8UC3)
			return false;
		cv::Mat Buf(720, 1280, CV_8UC3);
		for (int out_row = 0; out_row < Buf.rows; out_row++) {
			for (int out_col = 0; out_col < Buf.cols; out_col++) {
				cv::Vec3b x1 = Frame.at<cv::Vec3b>(3*out_row, 3*out_col);
				cv::Vec3b x2 = Frame.at<cv::Vec3b>(3*out_row, 3*out_col + 1);
				cv::Vec3b x3 = Frame.at<cv::Vec3b>(3*out_row, 3*out_col + 2);
				
				cv::Vec3b y1 = Frame.at<cv::Vec3b>(3*out_row + 1, 3*out_col);
				cv::Vec3b y2 = Frame.at<cv::Vec3b>(3*out_row + 1, 3*out_col + 1);
				cv::Vec3b y3 = Frame.at<cv::Vec3b>(3*out_row + 1, 3*out_col + 2);
				
				cv::Vec3b z1 = Frame.at<cv::Vec3b>(3*out_row + 2, 3*out_col);
				cv::Vec3b z2 = Frame.at<cv::Vec3b>(3*out_row + 2, 3*out_col + 1);
				cv::Vec3b z3 = Frame.at<cv::Vec3b>(3*out_row + 2, 3*out_col + 2);
				
				cv::Vec3i newVal(0,0,0);
				newVal += cv::Vec3i(x1(0), x1(1), x1(2));
				newVal += cv::Vec3i(x2(0), x2(1), x2(2));
				newVal += cv::Vec3i(x3(0), x3(1), x3(2));
				newVal += cv::Vec3i(y1(0), y1(1), y1(2));
				newVal += cv::Vec3i(y2(0), y2(1), y2(2));
				newVal += cv::Vec3i(y3(0), y3(1), y3(2));
				newVal += cv::Vec3i(z1(0), z1(1), z1(2));
				newVal += cv::Vec3i(z2(0), z2(1), z2(2));
				newVal += cv::Vec3i(z3(0), z3(1), z3(2));
				
				(Buf.at<cv::Vec3b>(out_row, out_col))(0) = uint8_t(newVal(0) / 9);
				(Buf.at<cv::Vec3b>(out_row, out_col))(1) = uint8_t(newVal(1) / 9);
				(Buf.at<cv::Vec3b>(out_row, out_col))(2) = uint8_t(newVal(2) / 9);
			}
		}
		Frame = Buf;
		return true;*/
	}
	
	void SimulatedDrone::StartSampleWaypointMission(int NumWaypoints, bool CurvedTrajectories, bool LandAtEnd,
	                                                Eigen::Vector2d const & StartOffset_EN, double HAG) {
		std::cerr << "Starting sample waypoint mission.\r\n";
		
		//Tell the vehicle control widget and the guidance module to stop commanding this drone.
		std::string droneSerial = GetDroneSerial();
		VehicleControlWidget::Instance().StopCommandingDrone(droneSerial);
		Guidance::GuidanceEngine::Instance().RemoveLowFlier(droneSerial);
		
		//Get drone's current position and the ground altitude
		DroneInterface::Drone::TimePoint Timestamp;
		double Latitude, Longitude, Altitude, currentHAG;
		if (! this->GetPosition(Latitude, Longitude, Altitude, Timestamp)) {
			std::cerr << "Error in StartSampleWaypointMission(): Failed to get drone's current position. Aborting.\r\n";
			return;
		}
		if (! this->GetHAG(currentHAG, Timestamp)) {
			std::cerr << "Error in StartSampleWaypointMission(): Failed to get drone's HAG. Aborting.\r\n";
			return;
		}
		double groundAlt = Altitude - currentHAG;
		std::cerr << "groundAlt: " << groundAlt << "\r\n";
		
		Eigen::Matrix3d C_ECEF_ENU = latLon_2_C_ECEF_ENU(Latitude, Longitude);
		Eigen::Vector3d currentPos_ECEF = LLA2ECEF(Eigen::Vector3d(Latitude, Longitude, Altitude));
		Eigen::Vector3d StartOffset_ENU(StartOffset_EN(0), StartOffset_EN(1), 0.0);
		Eigen::Vector3d FirstWaypoint_ECEF = currentPos_ECEF + C_ECEF_ENU.transpose()*StartOffset_ENU;
		Eigen::Vector3d FirstWaypoint_LLA = ECEF2LLA(FirstWaypoint_ECEF);
		Eigen::Vector2d FirstWaypoint_LL(FirstWaypoint_LLA(0), FirstWaypoint_LLA(1));
		
		DroneInterface::WaypointMission mission = CreateSampleWaypointMission(NumWaypoints, CurvedTrajectories, LandAtEnd,
		                                                                      FirstWaypoint_LL, groundAlt, HAG);
		
		this->ExecuteWaypointMission(mission);
	}
}








