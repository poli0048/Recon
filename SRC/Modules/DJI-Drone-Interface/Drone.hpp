//The drone interface module provides the software interface to DJI drones, connected over network sockets
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <string>

//External Includes
//OpenCV basic headers for use of cv::Mat

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
	};
	
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
	};
	
	//The Drone class provides an interface to interact with a single drone
	class Drone {
		public:
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
			
			//TODO: There needs to be some thought given to thread safety. Best option:
			//Add a private mutex field and scope-lock it in each public method
			
			DroneManager();
			~DroneManager() = default;
			
			//Basic hardware info (should be available on construction)
			std::string GetDroneSerial(void);
			
			//Telemetry - all methods return true if at least one valid reading has been received (regardless of age) and false otherwise
			bool GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp); //Lat and Lon (radians) and WGS84 Altitude (m)
			bool GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp); //NED velocity vector (m/s)
			bool GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp); //Yaw, Pitch, Roll (radians) using DJI definitions
			bool GetHAG(double & HAG, TimePoint & Timestamp); //Barometric height above ground (m) - Drone altitude minus takeoff altitude
			
			//Drone state and warnings
			bool GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp); //Drone Battery level (0 = Empty, 1 = Full)
			bool GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp); //Whether the drone has hit height or radius limits
			bool GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp); //Wind and other vehicle warnings as human-readable strings
			bool GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp); //GNSS status (-1 for none, 0-5 following DJI definitions)
			
			//Drone live video access - The mobile client should decode the live video feed and extract every N'th frame and send it to the server.
			//N should be detirmined when StartDJICamImageFeed() is called in order to achieve the given frame rate as closely as possible.
			//There is a frame counter that starts at 0 and increments each time a new frame is received by the server.
			bool IsDJICamConnected(void); //Should be available on construction
			void StartDJICamImageFeed(double TargetFPS); //Start sending frames of live video (as close as possible to the given framerate (frame / s))
			void StopDJICamImageFeed(void); //Stop sending frames of live video
			bool GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp);
			
			//Drone Command & Control. If a method returns bool, it should return True if the requested info is known (even if very old) and set the
			//Timestamp argument to the instant of validity of the most recently received value. If the requested info is unknown, these return False.
			//There is a method provided to get an ID for the currently running waypoint mission (if one is running). An unfortunate quirk of the DJI
			//SDK design though is that it doesn't look like you can choose your own IDs when creating a mission, so to use this to see if a new mission
			//is running you need to get the mission ID, start your new mission, and then check the ID again after some time to see if the ID has changed.
			bool GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp); //Get flight mode as a human-readable string
			void ExecuteWaypointMission(WaypointMission & Mission); //Stop current mission, if running. Then load, verify, and start new waypoint mission.
			bool IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp);
			bool GetCurrentWaypointMissionID(uint16_t & MissionID, TimePoint & Timestamp);
			void IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command); //Put in virtualStick Mode and issue command (stop mission if running)
			void IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command); //Put in virtualStick Mode and issue command (stop mission if running)
			
			void Hover(void); //Stop any running missions an leave virtualStick mode (if in it) and hover in place (P mode)
			void LandNow(void); //Initiate landing sequence immediately at current vehicle location
			void GoHomeAndLand(void); //Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location
	};
}



