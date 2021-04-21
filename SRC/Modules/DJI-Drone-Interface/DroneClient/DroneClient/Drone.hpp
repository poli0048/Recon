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
}
