//System Includes

//External Includes

//Project Includes
#include "Drone.hpp"

namespace DroneInterface {
	SimulatedDrone::SimulatedDrone() : m_thread(&SimulatedDrone::DroneMain, this), m_abort(false) { }
	
	SimulatedDrone::~SimulatedDrone() {
		m_abort = true;
		if (m_thread.joinable())
			m_thread.join();
	}
	
	//Get drone serial number as a string (should be available on construction)
	std::string SimulatedDrone::GetDroneSerial(void) {
		std::scoped_lock lock(m_mutex);
		return "Simulation";
	}
	
	//Lat & Lon (radians) and WGS84 Altitude (m)
	bool SimulatedDrone::GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		//Return a fixed position about 400 feet above ground in Lamberton, MN
		Latitude  = 0.7720877065630863;
		Longitude = -1.663401472323783;
		Altitude  = 466.0;
		Timestamp = std::chrono::steady_clock::now();
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
		Yaw   = 0.0; //Should be vehicle nose pointing north
		Pitch = 0.0; //Should be level-hover
		Roll  = 0.0; //Should be level-hover
		Timestamp = std::chrono::steady_clock::now();
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
	
	//Start sending frames of live video (as close as possible to the given framerate (frame / s))
	void SimulatedDrone::StartDJICamImageFeed(double TargetFPS) {
		std::scoped_lock lock(m_mutex);
		//Open the source video file here
		m_videoCap.open(m_videoPath.string());
		if (m_videoCap.isOpened()) {
			m_targetFPS = TargetFPS;
			m_imageFeedActive = true;
			m_videoCap_NextFrameIndex = 0;
			m_videoCap_NumFrames = static_cast<int>(m_videoCap.get(cv::CAP_PROP_FRAME_COUNT));
			m_videoCap_FPS = static_cast<int>(round(m_videoCap.get(cv::CAP_PROP_FPS)));
			m_VideoFeedStartTimestamp = std::chrono::steady_clock::now();
			m_videoFeedStarted = true;
			if (! GetNextVideoFrame()) {
				std::cerr << "Error in SimulatedDrone::StartDJICamImageFeed(): Failed to get first frame from video file. Stopping.\r\n";
				m_imageFeedActive = false;
				m_videoCap.release();
				m_videoCap_NumFrames = -1;
				m_videoCap_FPS = -1;
			}
		}
		else
			std::cerr << "Error in SimulatedDrone::StartDJICamImageFeed(): Unable to open video file. Not starting.\r\n";
	}
	
	//Stop sending frames of live video
	void SimulatedDrone::StopDJICamImageFeed(void) {
		std::scoped_lock lock(m_mutex);
		m_imageFeedActive = false;
		//Close the source video file here
		m_videoCap.release();
		m_videoCap_NextFrameIndex = 0;
		m_videoCap_NumFrames = -1;
		m_videoCap_FPS = -1;
	}
	
	bool SimulatedDrone::GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		if ((Frame.rows <= 0) || (Frame.cols <= 0))
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
	
	//Get flight mode as a human-readable string
	bool SimulatedDrone::GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		FlightModeStr = "Hover (P mode)";
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Stop current mission, if running. Then load, verify, and start new waypoint mission.
	void SimulatedDrone::ExecuteWaypointMission(WaypointMission & Mission) {
		std::scoped_lock lock(m_mutex);
		std::cerr << "Warning: SimulatedDrone doesn't currently support simulated flight. Can't execute waypoint missions.\r\n";
	}
	
	//Populate Result with whether or not a waypoint mission is currently being executed
	bool SimulatedDrone::IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Result = false;
		Timestamp = std::chrono::steady_clock::now();
		return true;
	}
	
	//Retrieve the ID of the currently running waypoint mission (if running).
	bool SimulatedDrone::GetCurrentWaypointMissionID(uint16_t & MissionID, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void SimulatedDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command) {
		std::scoped_lock lock(m_mutex);
		std::cerr << "Warning: SimulatedDrone doesn't currently support simulated flight. Can't execute VirtualStick command.\r\n";
	}
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void SimulatedDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command) {
		std::scoped_lock lock(m_mutex);
		std::cerr << "Warning: SimulatedDrone doesn't currently support simulated flight. Can't execute VirtualStick command.\r\n";
	}
	
	//Stop any running missions an leave virtualStick mode (if in it) and hover in place (P mode)
	void SimulatedDrone::Hover(void) {
		std::scoped_lock lock(m_mutex);
		//We only mimic hovering flight right now, so we don't need to do anything here
	}
	
	//Initiate landing sequence immediately at current vehicle location
	void SimulatedDrone::LandNow(void) {
		std::scoped_lock lock(m_mutex);
		std::cerr << "Warning: SimulatedDrone doesn't currently support simulated flight. Can't execute LandNow() command.\r\n";
	}
	
	//Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location
	void SimulatedDrone::GoHomeAndLand(void) {
		std::scoped_lock lock(m_mutex);
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
	
	//Returns true if end of video file reached and sim is done
	bool SimulatedDrone::IsSimVideoFinished(void) {
		std::scoped_lock lock(m_mutex);
		return (m_videoFeedStarted && (! m_imageFeedActive));
	}
	
	//Function for internal thread managing drone object
	void SimulatedDrone::DroneMain(void) {
		while (! m_abort) {
			m_mutex.lock();
			
			if (! m_imageFeedActive) {
				m_mutex.unlock();
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			
			//If we get here, our image feed is active - we don't have to worry about opening/closing the video file... that's handled in the start()/stop() functions
			if (m_realtime) {
				//In realtime mode, wait until we have reached the necessary time point to release the next frame
				TimePoint NextFrameDispatchTime = m_VideoFeedStartTimestamp + std::chrono::milliseconds(uint64_t(1000.0*(double(m_FrameNumber + 1U)/m_targetFPS)));
				if (std::chrono::steady_clock::now() < NextFrameDispatchTime) {
					//We need to wait - we haven't reached the dispatch time yet
					m_mutex.unlock();
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
				else {
					//We have reached the dispatch time - get the next frame and call callbacks
					if (GetNextVideoFrame()) {
						for (auto const & kv : m_ImageryCallbacks)
							kv.second(m_Frame, m_FrameTimestamp);
					}
					else
						m_imageFeedActive = false;
					m_mutex.unlock();
				}
			}
			else {
				//When we aren't running in real-time mode, don't wait for anything... just shovel frames as fast as we can
				if (GetNextVideoFrame()) {
					for (auto const & kv : m_ImageryCallbacks)
						kv.second(m_Frame, m_FrameTimestamp);
				}
				else
					m_imageFeedActive = false;
				m_mutex.unlock();
			}
		}
	}
	
	//Advance to and decode the next video from that needs to be used
	bool SimulatedDrone::GetNextVideoFrame(void) {
		if (m_videoCap_NextFrameIndex == 0) {
			//First frame
			if (m_videoCap.grab()) {
				m_videoCap_NextFrameIndex++;
				if ((m_videoCap.retrieve(m_Frame)) && (! m_Frame.empty())) {
					m_FrameNumber = 0U;
					m_FrameTimestamp = std::chrono::steady_clock::now();
					if (ResizeTo720p())
						return true;
				}
			}
			return false;
		}
		else {
			//Not first frame
			double targetNextFrameTime = double(m_FrameNumber + 1U)/m_targetFPS;
			//Advance in video until the next frame index corresponds to a time equal to or after the target time
			while (FrameNumToTimeIntoVid(m_videoCap_NextFrameIndex) < targetNextFrameTime) {
				if (m_videoCap.grab())
					m_videoCap_NextFrameIndex++;
				else
					return false;
			}
			//Grab and decode the next frame
			if (m_videoCap.grab()) {
				m_videoCap_NextFrameIndex++;
				if ((m_videoCap.retrieve(m_Frame)) && (! m_Frame.empty())) {
					m_FrameNumber++;
					m_FrameTimestamp = std::chrono::steady_clock::now();
					if (ResizeTo720p())
						return true;
				}
			}
			return false;
		}
	}
	
	bool SimulatedDrone::ResizeTo720p(void) {
		if ((m_Frame.rows == 720) && (m_Frame.cols == 1280))
			return true;
		if ((m_Frame.rows == 2160) && (m_Frame.cols == 3840))
			return Resize_4K_to_720p();
		return false;
	}
	
	//Drop a 4K m_frame down to 720p - copying Ben's resizing strategy
	bool SimulatedDrone::Resize_4K_to_720p(void) {
		if ((m_Frame.rows != 2160) && (m_Frame.cols != 3840))
			return false;
		//Ben's method
		/*cv::Mat BufA, BufB;
		pyrDown(m_Frame, BufA);
		pyrDown(BufA,    BufB);
		resize(BufB, m_Frame, cv::Size(1280, 720));
		return true;*/
		
		//Bryan's method - this is a bit faster than above, and retains the full sharpness of the imagery. Using PyrDown twice
		//overshoots the target resolution so requires an up-scaling afterwards. PyrDown also tends to filter more aggresively than
		//is necessary. Combined, these result in softer, oversmoothed imagery. Just doing 3x3 block averages avoids this.
		if (m_Frame.type() != CV_8UC3)
			return false;
		cv::Mat Buf(720, 1280, CV_8UC3);
		for (int out_row = 0; out_row < Buf.rows; out_row++) {
			for (int out_col = 0; out_col < Buf.cols; out_col++) {
				cv::Vec3b x1 = m_Frame.at<cv::Vec3b>(3*out_row, 3*out_col);
				cv::Vec3b x2 = m_Frame.at<cv::Vec3b>(3*out_row, 3*out_col + 1);
				cv::Vec3b x3 = m_Frame.at<cv::Vec3b>(3*out_row, 3*out_col + 2);
				
				cv::Vec3b y1 = m_Frame.at<cv::Vec3b>(3*out_row + 1, 3*out_col);
				cv::Vec3b y2 = m_Frame.at<cv::Vec3b>(3*out_row + 1, 3*out_col + 1);
				cv::Vec3b y3 = m_Frame.at<cv::Vec3b>(3*out_row + 1, 3*out_col + 2);
				
				cv::Vec3b z1 = m_Frame.at<cv::Vec3b>(3*out_row + 2, 3*out_col);
				cv::Vec3b z2 = m_Frame.at<cv::Vec3b>(3*out_row + 2, 3*out_col + 1);
				cv::Vec3b z3 = m_Frame.at<cv::Vec3b>(3*out_row + 2, 3*out_col + 2);
				
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
		m_Frame = Buf;
		return true;
	} 
	
	
}








