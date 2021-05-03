//The drone interface module provides the software interface to DJI drones, connected over network sockets
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <string>
#include <chrono>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>

//External Includes
#include "../../../handycpp/Handy.hpp" //Provides std::filesystem and Handy::File
#include <opencv2/opencv.hpp>

//Project Includes

namespace DroneInterface {
	//VirtualStickMode has several different configuration settings that impact how each control is interpreted. We
	//only implement two combinations of settings, which we call Mode A and Mode B. Both of these modes attempt to
	//command the vehicle state in an absolute sense as much as possible (e.g. specifying height instead of vertical velocity)
	//but it is not possible to specify absolute 2D position in VirtualStickMode, so we specify 2D velocity in either the
	//vehicle body frame or in East-North. Because we are commanding velocity, we need to worry about what happens if our software
	//crashes - we certainly don't want the drones to keep moving with their last-commanded velocity. Thus, we include a timout
	//field for each virtual stick command. If another virtual stick command isn't received by the client App within the timeout
	//window, it should issue it's own new virtual stick command with the same values as the most recent received command except
	//with the 2D velocity fields set to 0. This way, commands also serve as a heartbeat signal from the GCS and if they stop coming,
	//the drones hover (without changing modes though).
	
	//To use a ModeA virtual stick command, the drone should be configured as follows:
	//DJIVirtualStickVerticalControlMode    = DJIVirtualStickVerticalControlModePosition
	//DJIVirtualStickRollPitchControlMode   = DJIVirtualStickRollPitchControlModeVelocity
	//DJIVirtualStickYawControlMode         = DJIVirtualStickYawControlModeAngle
	//DJIVirtualStickFlightCoordinateSystem = DJIVirtualStickFlightCoordinateSystemGround
	//In this mode, yaw is specified in an absolute sense, relative to North. Height is commanded in an absolute sense, relative to ground,
	//2D position is controlled by commanding vehicle velocity in the North and East directions
	struct VirtualStickCommand_ModeA {
		float Yaw;       //Radians: 0 corresponds to North, positive is clockwise rotation
		float V_North;   //m/s: North component of vehicle velocity (Acceptable range -15 to 15)
		float V_East;    //m/s: East component of vehicle velocity (Acceptable range -15 to 15)
		float HAG;       //m: Height above ground (vehicle altitude - takeoff altitude)
		float timeout;   //s: If a new command isn't received within this time, the drone should hover
		
		//If switching to C++20, default this
		bool operator==(VirtualStickCommand_ModeA const & Other) const {
			return (this->Yaw       == Other.Yaw)       &&
			       (this->V_North   == Other.V_North)   &&
			       (this->V_East    == Other.V_East)    &&
			       (this->HAG       == Other.HAG)       &&
			       (this->timeout   == Other.timeout);
		}
	};
	
	//To use a ModeB virtual stick command, the drone should be configured as follows:
	//DJIVirtualStickVerticalControlMode    = DJIVirtualStickVerticalControlModePosition
	//DJIVirtualStickRollPitchControlMode   = DJIVirtualStickRollPitchControlModeVelocity
	//DJIVirtualStickYawControlMode         = DJIVirtualStickYawControlModeAngle
	//DJIVirtualStickFlightCoordinateSystem = DJIVirtualStickFlightCoordinateSystemBody
	//In this mode, yaw is specified in an absolute sense, relative to North. Height is commanded in an absolute sense, relative to ground,
	//2D position is controlled by commanding vehicle velocity in vehicle body frame (forward and vehicle right).
	struct VirtualStickCommand_ModeB {
		float Yaw;       //Radians: 0 corresponds to North, positive is clockwise rotation
		float V_Forward; //m/s: Forward component of vehicle velocity (Acceptable range -15 to 15)
		float V_Right;   //m/s: Vehicle-Right component of vehicle velocity (Acceptable range -15 to 15)
		float HAG;       //m: Height above ground (vehicle altitude - takeoff altitude)
		float timeout;   //s: If a new command isn't received within this time, the drone should hover
		
		//If switching to C++20, default this
		bool operator==(VirtualStickCommand_ModeB const & Other) const {
			return (this->Yaw       == Other.Yaw)       &&
			       (this->V_Forward == Other.V_Forward) &&
			       (this->V_Right   == Other.V_Right)   &&
			       (this->HAG       == Other.HAG)       &&
			       (this->timeout   == Other.timeout);
		}
	};
	
	//Waypoint objects are used as components of WaypointMission objects. Note that the speed field should be checked before putting it in a DJIWaypoint. If it is 0,
	//it needs to be adjusted upwards to a default min value. If 0 is put in a DJIWaypoint speed field, the behavior changes and the speed gets overwritten by
	//another value set at the mission level. We don't want this ridiculous behavior so we should make sure this is never actually 0.
	struct Waypoint {
		double Latitude;  //WGS84 Latitude of waypoint (Radians)
		double Longitude; //WGS84 Longitude of waypoint (Radians)
		double Altitude;  //WGS84 Altitude of waypoint (meters) - Note that this is actual altitude and not height above ground
		
		float CornerRadius; //Radius of arc (m) to make when cutting corner at this waypoint. Only used when CurvedTrajectory = true in the parent mission.
		float Speed;        //Vehicle speed (m/s) between this waypoint and the next waypoint (0 < Speed <= 15)
		
		//Waypoint Actions: The drone can be told to execute certain actions once reaching a waypoint. Actions are not mutually exclusive (according to the
		//docs anyways) so none, one, or multiple can be used in a waypoint. Important Note: Actions are only executed if CurvedTrajectory = false in the parent mission.
		//For each action field, the value NaN indicates that the action should not be included. A non-NaN value generally indicates that an action should be added
		//to the waypoint to accomplish the given goal. These fields correspond to the following waypoint actions:
		//LoiterTime:  DJIWaypointActionTypeStay
		//GimbalPitch: DJIWaypointActionTypeRotateGimbalPitch
		float LoiterTime;   //Time (s) to hover at this waypoint (0 is equivilent to NaN and should result in the action not being included).
		float GimbalPitch;  //Pitch of Gimbal, if connected (DJI Definition) in radians at waypoint.
		
		//If switching to C++20, default this
		bool operator==(Waypoint const & Other) const {
			return (this->Latitude     == Other.Latitude)     &&
			       (this->Longitude    == Other.Longitude)    &&
			       (this->Altitude     == Other.Altitude)     &&
			       (this->CornerRadius == Other.CornerRadius) &&
			       (this->Speed        == Other.Speed)        &&
			       (this->LoiterTime   == Other.LoiterTime)   &&
			       (this->GimbalPitch  == Other.GimbalPitch);
		}
	};
	
	inline std::ostream & operator<<(std::ostream & Str, Waypoint const & v) { 
		double PI = 3.14159265358979;
		Str << "Latitude ----: " << 180.0/PI*v.Latitude     << " degrees\r\n";
		Str << "Longitude ---: " << 180.0/PI*v.Longitude    << " degrees\r\n";
		Str << "Altitude ----: " <<          v.Altitude     << " m\r\n";
		Str << "CornerRadius : " <<          v.CornerRadius << " m\r\n";
		Str << "Speed -------: " <<          v.Speed        << " m/s\r\n";
		Str << "LoiterTime --: " <<          v.LoiterTime   << " s\r\n";
		Str << "GimbalPitch -: " << 180.0/PI*v.GimbalPitch  << " degrees\r\n";
		return Str;
	}
	
	//This struct holds a waypoint mission for a single drone. The full DJI waypoint mission interface is relatively complex - we only implement the
	//subset of it's functionality that we expect to be useful for our purposes. Note that for all these missions, the vehicle Heading Mode should be set to
	//DJIWaypointMissionHeadingAuto, which orients the aircraft so the front is always pointed in the direction of motion.
	//DJIWaypointMissionGotoWaypointMode should be set to DJIWaypointMissionGotoWaypointPointToPoint. This means the vehicle goes directly from it's
	//current location to the first waypoint, rather than changing 2D position and altitude separately. If you want the vehicle to change position vertically,
	//add a waypoint above or below another.
	struct WaypointMission {
		std::vector<Waypoint> Waypoints; //Waypoints to fly to, in order from the vehicle starting position (which is not included as a waypoint)
		bool LandAtLastWaypoint; //If true, the vehicle lands after hitting the final waypoint. If false, the mission ends with the vehicle hovering in P flight mode.
		bool CurvedTrajectory; //If true, cut corners when hitting waypoints, resulting in curved trajectory. If false, fly point-to-point, stopping at each waypoint.
		
		//If switching to C++20, default this
		bool operator==(WaypointMission const & Other) const {
			if ((this->LandAtLastWaypoint != Other.LandAtLastWaypoint) || (this->CurvedTrajectory != Other.CurvedTrajectory))
				return false;
			if (this->Waypoints.size() != Other.Waypoints.size())
				return false;
			for (size_t n = 0U; n < Waypoints.size(); n++) {
				if (! (this->Waypoints[n] == Other.Waypoints[n]))
					return false;
			}
			return true;
		}
	};
	
	//Abstract class for drones - This means that an object of this type cannot actually exist... only objects of a derived type. We have two derived types:
	//RealDrone and SimulatedDrone
	class Drone {
		public:
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
			Drone() = default;
			virtual ~Drone() = default;
			
			//Basic hardware info (should be available on construction)
			virtual std::string GetDroneSerial(void) = 0;
			
			//Telemetry - all methods return true if at least one valid reading has been received (regardless of age) and false otherwise
			virtual bool GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) = 0; //Lat & Lon (radians) and WGS84 Altitude (m)
			virtual bool GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp) = 0; //NED velocity vector (m/s)
			virtual bool GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp) = 0; //Yaw, Pitch, Roll (radians) using DJI definitions
			virtual bool GetHAG(double & HAG, TimePoint & Timestamp) = 0; //Barometric height above ground (m) - Drone altitude minus takeoff altitude
			
			//Drone state and warnings
			virtual bool GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp) = 0; //Drone Battery level (0 = Empty, 1 = Full)
			virtual bool GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp) = 0; //Whether the drone has hit height or radius limits
			virtual bool GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) = 0; //Wind & other vehicle warnings as strings
			virtual bool GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp) = 0; //GNSS status (-1 for none, 0-5: DJI definitions)
			
			//Drone live video access - The mobile client should decode the live video feed and extract every N'th frame and send it to the server.
			//N should be detirmined when StartDJICamImageFeed() is called in order to achieve the given frame rate as closely as possible.
			//There is a frame counter that starts at 0 and increments each time a new frame is received by the server.
			virtual bool IsDJICamConnected(void) = 0; //Should be available on construction
			virtual bool IsCamImageFeedOn(void) = 0; //True if receiving imagery from drone, false otherwise (valid on construction... initially returns false)
			virtual void StartDJICamImageFeed(double TargetFPS) = 0; //Start sending frames of live video (as close as possible to the given framerate (frame / s))
			virtual void StopDJICamImageFeed(void) = 0; //Stop sending frames of live video
			virtual bool GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp) = 0;
			virtual int  RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) = 0; //Regester callback for new frames
			virtual void UnRegisterCallback(int Handle) = 0; //Unregister callback for new frames (input is token returned by RegisterCallback()
			
			//Drone Command & Control. If a method returns bool, it should return True if the requested info is known (even if very old) and set the
			//Timestamp argument to the instant of validity of the most recently received value. If the requested info is unknown, these return False.
			//There is a method provided to get an ID for the currently running waypoint mission (if one is running). An unfortunate quirk of the DJI
			//SDK design though is that it doesn't look like you can choose your own IDs when creating a mission, so to use this to see if a new mission
			//is running you need to get the mission ID, start your new mission, and then check the ID again after some time to see if the ID has changed.
			virtual bool GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp) = 0; //Get flight mode as a human-readable string
			virtual void ExecuteWaypointMission(WaypointMission & Mission) = 0; //Stop current mission, if running. Then load, verify, and start new waypoint mission.
			virtual bool IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) = 0;
			virtual bool IsCurrentlyFlying(bool & Result, TimePoint & Timestamp) = 0;
			virtual bool GetCurrentWaypointMissionID(uint16_t & MissionID, TimePoint & Timestamp) = 0;
			virtual void IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command) = 0; //Put in virtualStick Mode and send command (stop mission if running)
			virtual void IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command) = 0; //Put in virtualStick Mode and send command (stop mission if running)
			
			//If a Hover command is issued when the drone is not flying (on ground), it should take off and hover at a safe height above ground.
			virtual void Hover(void) = 0; //Stop any running missions and leave virtualStick mode (if in it) and hover in place (P mode)
			virtual void LandNow(void) = 0; //Initiate landing sequence immediately at current vehicle location
			virtual void GoHomeAndLand(void) = 0; //Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location
	};
	
	//The RealDrone class provides an interface to interact with a single real drone
	class RealDrone : public Drone {
		public:
			RealDrone();
			~RealDrone();
			
			std::string GetDroneSerial(void) override;
			
			bool GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) override;
			bool GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp)       override;
			bool GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp)           override;
			bool GetHAG(double & HAG, TimePoint & Timestamp)                                                  override;
			
			bool GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp)                   override;
			bool GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp)  override;
			bool GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) override;
			bool GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp)    override;
			
			bool IsDJICamConnected(void)                                                                            override;
			bool IsCamImageFeedOn(void)                                                                             override;
			void StartDJICamImageFeed(double TargetFPS)                                                             override;
			void StopDJICamImageFeed(void)                                                                          override;
			bool GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp)             override;
			int  RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) override;
			void UnRegisterCallback(int Handle)                                                                     override;
			
			bool GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp)         override;
			void ExecuteWaypointMission(WaypointMission & Mission)                         override;
			bool IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) override;
			bool IsCurrentlyFlying(bool & Result, TimePoint & Timestamp)                   override;
			bool GetCurrentWaypointMissionID(uint16_t & MissionID, TimePoint & Timestamp)  override;
			void IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command)       override;
			void IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command)       override;
			
			void Hover(void)         override;
			void LandNow(void)       override;
			void GoHomeAndLand(void) override;
		
		private:
			//Some modules that use imagery can't handle missing frames gracefully. Thus, we use provide a callback mechanism to ensure that such a module
			//can have a guarantee that each frame received by the drone interface module will be provided downstream.
			std::unordered_map<int, std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)>> m_ImageryCallbacks;
			
			std::thread       m_thread;
			std::atomic<bool> m_abort;
			std::mutex        m_mutex; //Lock in each public method for thread safety
			
			void DroneMain(void);
	};
	
	//The SimulatedDrone class provides an interface to interact with a single virtual/simulated drone
	//Note: Right now the simulated drone is very dumb... we don't pretend to fly missions or send warnings or any of the other
	//standard things one might expect of a simulated drone. We just mimic a drone in a stationary hover (P mode) over a fixed
	//point in Lamberton, MN. We do provide meaningful implementations of all image-related functions however, so the simulated
	//drone can be used to test anything involving live drone imagery. Imagery is pulled from a video file and dispatched either
	//at real-time speed or as fast as possible, depending on configuration.
	class SimulatedDrone : public Drone {
		public:
			SimulatedDrone();
			~SimulatedDrone();
			
			std::string GetDroneSerial(void) override;
			
			bool GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) override;
			bool GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp)       override;
			bool GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp)           override;
			bool GetHAG(double & HAG, TimePoint & Timestamp)                                                  override;
			
			bool GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp)                   override;
			bool GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp)  override;
			bool GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) override;
			bool GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp)    override;
			
			bool IsDJICamConnected(void)                                                                            override;
			bool IsCamImageFeedOn(void)                                                                             override;
			void StartDJICamImageFeed(double TargetFPS)                                                             override;
			void StopDJICamImageFeed(void)                                                                          override;
			bool GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp)             override;
			int  RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) override;
			void UnRegisterCallback(int Handle)                                                                     override;
			
			bool GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp)         override;
			void ExecuteWaypointMission(WaypointMission & Mission)                         override;
			bool IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) override;
			bool IsCurrentlyFlying(bool & Result, TimePoint & Timestamp)                   override;
			bool GetCurrentWaypointMissionID(uint16_t & MissionID, TimePoint & Timestamp)  override;
			void IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command)       override;
			void IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command)       override;
			
			void Hover(void)         override;
			void LandNow(void)       override;
			void GoHomeAndLand(void) override;
			
			//SimulatedDrone-specific methods
			void SetRealTime(bool Realtime); //True: Imagery will be provided at close-to-real-time rate. False: Imagery is provided as fast as possible
			void SetSourceVideoFile(std::filesystem::path const & VideoPath); //Should be set before calling StartDJICamImageFeed()
			bool GetReferenceFrame(double SecondsIntoVideo, cv::Mat & Frame); //Get a single frame - will fail if the video feed is running
			
			static bool ResizeTo720p(cv::Mat & Frame); //Make sure the frame is 720p... resize if needed.
			static bool Resize_4K_to_720p(cv::Mat & Frame); //Drop a 4K m_frame down to 720p
		private:
			//Some modules that use imagery can't handle missing frames gracefully. Thus, we use provide a callback mechanism to ensure that such a module
			//can have a guarantee that each frame received by the drone interface module will be provided downstream.
			std::unordered_map<int, std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)>> m_ImageryCallbacks;
			
			std::thread       m_MainThread;
			std::atomic<bool> m_abort;
			std::mutex        m_mutex; //Lock in each public method for thread safety
			
			//When the video feed is running, a thread is launched to do nothing but read and decode the necessary frames.
			std::thread       m_VideoProcessingThread;
			std::atomic<bool> m_VideoProcessingThreadAbort;
			std::mutex        m_NextFrameMutex;
			cv::Mat           m_NextFrame;                     //Protected by m_NextFrameMutex
			bool              m_NextFrameReady = false;        //Protected by m_NextFrameMutex
			bool              m_VideoFileReadFinished = false; //Protected by m_NextFrameMutex
			
			//Additional State Data
			bool m_realtime = false;
			std::filesystem::path m_videoPath;
			double m_targetFPS = -1.0;
			bool m_imageFeedActive = false;
			cv::Mat m_Frame;                     //Most recent frame
			unsigned int m_FrameNumber = 0U;     //Frame number of most recent frame (increments on each *used* frame)
			TimePoint m_FrameTimestamp;          //Timestamp of most recent frame
			TimePoint m_VideoFeedStartTimestamp; //Timestamp of start of video feed
			
			void DroneMain(void);
	};
	
}



