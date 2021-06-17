//The drone interface module provides the software interface to DJI drones, connected over network sockets
//This particular source file defines a simulated drone for testing and development purposes
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//System Includes

//External Includes

//Project Includes
#include "Drone.hpp"

#define PI 3.14159265358979

//DroneInterface::Drone::TimePoint InitTimepoint = std::chrono::steady_clock::now(); //Used for testing message age warnings

namespace DroneInterface {
	SimulatedDrone::SimulatedDrone() : m_abort(false) {
		//Set up a waypoint mission that we can pretend to be flying
		m_LastMission.Waypoints.clear();
		m_LastMission.Waypoints.emplace_back();
		m_LastMission.Waypoints.back().Latitude     =  44.237308 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Longitude    = -95.307433 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Altitude     =  466.0; // m
		m_LastMission.Waypoints.back().CornerRadius =  0.0f;  // m (Not used since we aren't using curved trajectories)
		m_LastMission.Waypoints.back().Speed        =  9.5f;  // m/s
		m_LastMission.Waypoints.back().LoiterTime   = std::nanf("");
		m_LastMission.Waypoints.back().GimbalPitch  = std::nanf("");
		
		m_LastMission.Waypoints.emplace_back();
		m_LastMission.Waypoints.back().Latitude     =  44.237308 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Longitude    = -95.309309 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Altitude     =  466.0; // m
		m_LastMission.Waypoints.back().CornerRadius = 0.0f;   // m (Not used since we aren't using curved trajectories)
		m_LastMission.Waypoints.back().Speed        = 9.5f;   // m/s
		m_LastMission.Waypoints.back().LoiterTime   = std::nanf("");
		m_LastMission.Waypoints.back().GimbalPitch  = std::nanf("");
		
		m_LastMission.Waypoints.emplace_back();
		m_LastMission.Waypoints.back().Latitude     =  44.237819 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Longitude    = -95.309309 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Altitude     =  466.0; // m
		m_LastMission.Waypoints.back().CornerRadius = 0.0f;   // m (Not used since we aren't using curved trajectories)
		m_LastMission.Waypoints.back().Speed        = 9.5f;   // m/s
		m_LastMission.Waypoints.back().LoiterTime   = std::nanf("");
		m_LastMission.Waypoints.back().GimbalPitch  = std::nanf("");
		
		m_LastMission.Waypoints.emplace_back();
		m_LastMission.Waypoints.back().Latitude     =  44.237819 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Longitude    = -95.307433 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Altitude     =  466.0; // m
		m_LastMission.Waypoints.back().CornerRadius = 0.0f;   // m (Not used since we aren't using curved trajectories)
		m_LastMission.Waypoints.back().Speed        = 9.5f;   // m/s
		m_LastMission.Waypoints.back().LoiterTime   = std::nanf("");
		m_LastMission.Waypoints.back().GimbalPitch  = std::nanf("");
		
		m_LastMission.Waypoints.emplace_back();
		m_LastMission.Waypoints.back().Latitude     =  44.238344 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Longitude    = -95.307433 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Altitude     =  466.0; // m
		m_LastMission.Waypoints.back().CornerRadius = 0.0f;   // m (Not used since we aren't using curved trajectories)
		m_LastMission.Waypoints.back().Speed        = 9.5f;   // m/s
		m_LastMission.Waypoints.back().LoiterTime   = std::nanf("");
		m_LastMission.Waypoints.back().GimbalPitch  = std::nanf("");
		
		m_LastMission.Waypoints.emplace_back();
		m_LastMission.Waypoints.back().Latitude     =  44.238344 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Longitude    = -95.309309 * PI/180.0; // radians
		m_LastMission.Waypoints.back().Altitude     =  466.0; // m
		m_LastMission.Waypoints.back().CornerRadius = 0.0f; // m (Not used since we aren't using curved trajectories)
		m_LastMission.Waypoints.back().Speed        = 9.5f; // m/s
		m_LastMission.Waypoints.back().LoiterTime   = std::nanf("");
		m_LastMission.Waypoints.back().GimbalPitch  = std::nanf("");
		
		m_LastMission.LandAtLastWaypoint = false;
		m_LastMission.CurvedTrajectory = false;
		
		m_flightMode = 1;
		
		m_MainThread = std::thread(&SimulatedDrone::DroneMain, this);
	}
	
	SimulatedDrone::~SimulatedDrone() {
		m_abort = true;
		m_VideoProcessingThreadAbort = true;
		
		if (m_VideoProcessingThread.joinable())
			m_VideoProcessingThread.join();
		if (m_MainThread.joinable())
			m_MainThread.join();
	}
	
	//Get drone serial number as a string (should be available on construction)
	std::string SimulatedDrone::GetDroneSerial(void) {
		std::scoped_lock lock(m_mutex);
		return "Simulation"s;
	}
	
	//Lat & Lon (radians) and WGS84 Altitude (m)
	bool SimulatedDrone::GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		//Return a fixed position about 400 feet above ground in Lamberton, MN
		Latitude  = 0.7720877065630863;
		Longitude = -1.663401472323783;
		Altitude  = 466.0;
		Timestamp = std::chrono::steady_clock::now();
		//Timestamp = InitTimepoint;
		return true;
	}
	
	//NED velocity vector (m/s)
	bool SimulatedDrone::GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		//Return 0 (Vehicle is stationary)
		V_North   = 0.0;
		V_East    = 0.0;
		V_Down    = 0.0;
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Yaw, Pitch, Roll (radians) using DJI definitions
	bool SimulatedDrone::GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Yaw   = 30.0*PI/180.0; //close to nose facing north-northeast
		Pitch = 0.0; //Should be level-hover
		Roll  = 0.0; //Should be level-hover
		Timestamp = std::chrono::steady_clock::now();
		//Timestamp = InitTimepoint;
		return true;
	}
	
	//Barometric height above ground (m) - Drone altitude minus takeoff altitude
	bool SimulatedDrone::GetHAG(double & HAG, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		HAG = 121.1;
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Drone Battery level (0 = Empty, 1 = Full)
	bool SimulatedDrone::GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		BattLevel = 0.76;
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
		ActiveWarnings.push_back("Warning 1"s);
		ActiveWarnings.push_back("Warning 2"s);
		ActiveWarnings.push_back("Warning 3"s);
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
		Result = true;
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Get flight mode as a human-readable string
	bool SimulatedDrone::GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		switch (m_flightMode) {
			case 0:  FlightModeStr = "Hover (P mode)"s;   break;
			case 1:  FlightModeStr = "Waypoint Mission"s; break;
			case 2:  FlightModeStr = "Virtual Stick"s;    break;
			default: FlightModeStr = "Unknown"s;          break;
		}
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Stop current mission, if running. Then load, verify, and start new waypoint mission.
	void SimulatedDrone::ExecuteWaypointMission(WaypointMission & Mission) {
		std::scoped_lock lock(m_mutex);
		m_LastMission = Mission;
		m_flightMode = 1;
		std::cerr << "Warning: SimulatedDrone doesn't currently support simulated flight. Can't execute waypoint missions.\r\n";
	}
	
	//Populate Result with whether or not a waypoint mission is currently being executed
	bool SimulatedDrone::IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Result = (m_flightMode == 1);
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Populate arg with current mission (returns false if not flying waypoint mission)
	bool SimulatedDrone::GetCurrentWaypointMission(WaypointMission & Mission) {
		std::scoped_lock lock(m_mutex);
		if (m_flightMode == 1) {
			Mission = m_LastMission;
			return true;
		}
		else
			return false;
	}
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void SimulatedDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command) {
		std::scoped_lock lock(m_mutex);
		m_flightMode = 2;
		std::cerr << "Warning: SimulatedDrone doesn't currently support simulated flight. Can't execute VirtualStick command.\r\n";
	}
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void SimulatedDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command) {
		std::scoped_lock lock(m_mutex);
		m_flightMode = 2;
		std::cerr << "Warning: SimulatedDrone doesn't currently support simulated flight. Can't execute VirtualStick command.\r\n";
	}
	
	//Stop any running missions an leave virtualStick mode (if in it) and hover in place (P mode)
	void SimulatedDrone::Hover(void) {
		std::scoped_lock lock(m_mutex);
		m_flightMode = 0;
		std::cerr << "Warning: SimulatedDrone doesn't currently support simulated flight. Can't execute Hover() command.\r\n";
	}
	
	//Initiate landing sequence immediately at current vehicle location
	void SimulatedDrone::LandNow(void) {
		std::scoped_lock lock(m_mutex);
		m_flightMode = -1;
		std::cerr << "Warning: SimulatedDrone doesn't currently support simulated flight. Can't execute LandNow() command.\r\n";
	}
	
	//Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location
	void SimulatedDrone::GoHomeAndLand(void) {
		std::scoped_lock lock(m_mutex);
		m_flightMode = -1;
		std::cerr << "Warning: SimulatedDrone doesn't currently support simulated flight. Can't execute GoHomeAndLand() command.\r\n";
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
					//m_NextFrame.copyTo(m_Frame);
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
}








