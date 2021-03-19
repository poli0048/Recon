//The drone interface module provides the software interface to DJI drones, connected over network sockets
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//System Includes

//External Includes

//Project Includes
#include "Drone.hpp"

namespace DroneInterface {
	RealDrone::RealDrone() : m_thread(&RealDrone::DroneMain, this), m_abort(false) {
		//Initialization Here (Retrieve serial number, etc.)
	}
	
	RealDrone::~RealDrone() {
		m_abort = true;
		if (m_thread.joinable())
			m_thread.join();
	}
	
	//Get drone serial number as a string (should be available on construction)
	std::string RealDrone::GetDroneSerial(void) { return std::string(); }
	
	//Lat & Lon (radians) and WGS84 Altitude (m)
	bool RealDrone::GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) { return false; }
	
	//NED velocity vector (m/s)
	bool RealDrone::GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp) { return false; }
	
	//Yaw, Pitch, Roll (radians) using DJI definitions
	bool RealDrone::GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp) { return false; }
	
	//Barometric height above ground (m) - Drone altitude minus takeoff altitude
	bool RealDrone::GetHAG(double & HAG, TimePoint & Timestamp) { return false; }
	
	//Drone Battery level (0 = Empty, 1 = Full)
	bool RealDrone::GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp) { return false; }
	
	//Whether the drone has hit height or radius limits
	bool RealDrone::GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp) { return false; }
	
	//Wind & other vehicle warnings as strings
	bool RealDrone::GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) { return false; }
	
	//GNSS status (-1 for none, 0-5: DJI definitions)
	bool RealDrone::GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp) { return false; }
	
	//Returns true if recognized DJI camera is present - Should be available on construction
	bool RealDrone::IsDJICamConnected(void) { return false; }
	
	//Start sending frames of live video (as close as possible to the given framerate (frame / s))
	void RealDrone::StartDJICamImageFeed(double TargetFPS) { }
	
	//Stop sending frames of live video
	void RealDrone::StopDJICamImageFeed(void) { }
	
	bool RealDrone::GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp) { return false; }
	
	//Regester callback for new frames
	int RealDrone::RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) {
		std::scoped_lock lock(m_mutex);
		int token = 0;
		while (m_ImageryCallbacks.count(token) > 0U)
			token++;
		m_ImageryCallbacks[token] = Callback;
		return token;
	}

	//Unregister callback for new frames (input is token returned by RegisterCallback()
	void RealDrone::UnRegisterCallback(int Handle) {
		std::scoped_lock lock(m_mutex);
		m_ImageryCallbacks.erase(Handle);
	}
	
	//Get flight mode as a human-readable string
	bool RealDrone::GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp) { return false; }
	
	//Stop current mission, if running. Then load, verify, and start new waypoint mission.
	void RealDrone::ExecuteWaypointMission(WaypointMission & Mission) { }
	
	//Populate Result with whether or not a waypoint mission is currently being executed
	bool RealDrone::IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) { return false; }
	
	//Retrieve the ID of the currently running waypoint mission (if running).
	bool RealDrone::GetCurrentWaypointMissionID(uint16_t & MissionID, TimePoint & Timestamp) { return false; }
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void RealDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command) { }
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void RealDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command) { }
	
	//Stop any running missions an leave virtualStick mode (if in it) and hover in place (P mode)
	void RealDrone::Hover(void) { }
	
	//Initiate landing sequence immediately at current vehicle location
	void RealDrone::LandNow(void) { }
	
	//Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location
	void RealDrone::GoHomeAndLand(void) { }
	
	//Function for internal thread managing drone object
	void RealDrone::DroneMain(void) {
		while (! m_abort) {
			m_mutex.lock();
			
			//Check to see if there is new data on the socket to process and process it.
			//Decode received packets and update private state
			
			//If a new image was received and decoded, call the callbacks:
			/*if (newImageProcessed) {
				for (auto const & kv : m_ImageryCallbacks)
					kv.second(NewImage, NewImageTimestamp);
			}*/
			
			//At the end of the loop, if we did useful work do:
			//m_mutex.unlock();
			//If we did not do useful work do:
			//m_mutex.unlock();
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
}




