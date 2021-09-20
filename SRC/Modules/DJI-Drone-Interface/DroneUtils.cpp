//This sub-module includes utilities that are useful for real and simulated drones
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.

//Project Includes
#include "DroneUtils.hpp"
#include "../../Maps/MapUtils.hpp"

// *************************************************************************************************************************
// **************************************************   Local Utilities   **************************************************
// *************************************************************************************************************************


// *************************************************************************************************************************
// **************************************************   Public Utilities   *************************************************
// *************************************************************************************************************************

namespace DroneInterface {
	WaypointMission CreateSampleWaypointMission(int NumWaypoints, bool CurvedTrajectories, bool LandAtEnd, Eigen::Vector2d const & StartPos_LL,
	                                            double GroundAlt, double HAG) {
		DroneInterface::WaypointMission mission;
		mission.LandAtLastWaypoint = LandAtEnd;
		mission.CurvedTrajectory   = CurvedTrajectories;
		
		//Set up ENU frame under first waypoint
		Eigen::Vector3d Pos_ECEF = LLA2ECEF(Eigen::Vector3d(StartPos_LL(0), StartPos_LL(1), GroundAlt));
		Eigen::Matrix3d C_ECEF_ENU = latLon_2_C_ECEF_ENU(StartPos_LL(0), StartPos_LL(1));
		Eigen::Matrix3d C_ENU_ECEF = C_ECEF_ENU.transpose();
		
		//Set first waypoint
		Eigen::Vector3d WaypointPos_LLA = ECEF2LLA(Pos_ECEF);
		mission.Waypoints.emplace_back();
		mission.Waypoints.back().Latitude  = WaypointPos_LLA(0);
		mission.Waypoints.back().Longitude = WaypointPos_LLA(1);
		mission.Waypoints.back().Altitude  = GroundAlt + HAG;
		mission.Waypoints.back().CornerRadius = 5.0;
		mission.Waypoints.back().Speed = 15.0;
		
		//std::cerr << "Waypoint Alt: " << mission.Waypoints.back().Altitude << "\r\n";
		
		//Add waypoints
		for (int n = 0; n < NumWaypoints; n++) {
			//Get last waypoint position
			DroneInterface::Waypoint const & lastWaypoint(mission.Waypoints.back());
			Eigen::Vector3d LastPosition_ECEF = LLA2ECEF(Eigen::Vector3d(lastWaypoint.Latitude, lastWaypoint.Longitude, lastWaypoint.Altitude));
			
			Eigen::Vector3d V_EN(0.0, 0.0, 0.0); //Vector between last waypoint and this waypoint
			switch (n % 4) {
				case 0: V_EN <<    0.0, 15.0, 0.0; break;
				case 1: V_EN << -100.0,  0.0, 0.0; break;
				case 2: V_EN <<    0.0, 15.0, 0.0; break;
				case 3: V_EN <<  100.0,  0.0, 0.0; break;
			}
			
			Eigen::Vector3d NewPosition_ECEF = LastPosition_ECEF + C_ENU_ECEF*V_EN;
			Eigen::Vector3d NewPosition_LLA  = ECEF2LLA(NewPosition_ECEF);
			
			mission.Waypoints.emplace_back();
			mission.Waypoints.back().Latitude  = NewPosition_LLA(0);
			mission.Waypoints.back().Longitude = NewPosition_LLA(1);
			mission.Waypoints.back().Altitude  = NewPosition_LLA(2);
			mission.Waypoints.back().CornerRadius = 5.0;
			mission.Waypoints.back().Speed = 15.0;
		}
		
		return mission;
	}
}
