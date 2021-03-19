//The drone interface module provides the software interface to DJI drones, connected over network sockets
//This particular source file defines a simulated drone for testing and development purposes
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//System Includes

//External Includes

//Project Includes
#include "Drone.hpp"

namespace DroneInterface {
	SimulatedDrone::SimulatedDrone() : m_thread(&SimulatedDrone::DroneMain, this), m_abort(false) {
		//Initialization Here (Retrieve serial number, etc.)
	}
	
	SimulatedDrone::~SimulatedDrone() {
		m_abort = true;
		if (m_thread.joinable())
			m_thread.join();
	}
	
	//Get drone serial number as a string (should be available on construction)
	std::string SimulatedDrone::GetDroneSerial(void) {
		std::scoped_lock lock(m_mutex);
		return "Simulation"s;
	}
	
	//Lat & Lon (radians) and WGS84 Altitude (m)
	bool SimulatedDrone::GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//NED velocity vector (m/s)
	bool SimulatedDrone::GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//Yaw, Pitch, Roll (radians) using DJI definitions
	bool SimulatedDrone::GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//Barometric height above ground (m) - Drone altitude minus takeoff altitude
	bool SimulatedDrone::GetHAG(double & HAG, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//Drone Battery level (0 = Empty, 1 = Full)
	bool SimulatedDrone::GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//Whether the drone has hit height or radius limits
	bool SimulatedDrone::GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//Wind & other vehicle warnings as strings
	bool SimulatedDrone::GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//GNSS status (-1 for none, 0-5: DJI definitions)
	bool SimulatedDrone::GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//Returns true if recognized DJI camera is present - Should be available on construction
	bool SimulatedDrone::IsDJICamConnected(void) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//Start sending frames of live video (as close as possible to the given framerate (frame / s))
	void SimulatedDrone::StartDJICamImageFeed(double TargetFPS) {
		std::scoped_lock lock(m_mutex);
	}
	
	//Stop sending frames of live video
	void SimulatedDrone::StopDJICamImageFeed(void) {
		std::scoped_lock lock(m_mutex);
	}
	
	bool SimulatedDrone::GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
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
		return false;
	}
	
	//Stop current mission, if running. Then load, verify, and start new waypoint mission.
	void SimulatedDrone::ExecuteWaypointMission(WaypointMission & Mission) {
		std::scoped_lock lock(m_mutex);
	}
	
	//Populate Result with whether or not a waypoint mission is currently being executed
	bool SimulatedDrone::IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//Retrieve the ID of the currently running waypoint mission (if running).
	bool SimulatedDrone::GetCurrentWaypointMissionID(uint16_t & MissionID, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void SimulatedDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command) {
		std::scoped_lock lock(m_mutex);
	}
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void SimulatedDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command) {
		std::scoped_lock lock(m_mutex);
	}
	
	//Stop any running missions an leave virtualStick mode (if in it) and hover in place (P mode)
	void SimulatedDrone::Hover(void) {
		std::scoped_lock lock(m_mutex);
	}
	
	//Initiate landing sequence immediately at current vehicle location
	void SimulatedDrone::LandNow(void) {
		std::scoped_lock lock(m_mutex);
	}
	
	//Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location
	void SimulatedDrone::GoHomeAndLand(void) {
		std::scoped_lock lock(m_mutex);
	}
	
	//Function for internal thread managing drone object
	void SimulatedDrone::DroneMain(void) {
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




