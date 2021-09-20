//This sub-module includes utilities that are useful for real and simulated drones
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.
#pragma once

//System Includes
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>

//External Includes
#include "../../../handycpp/Handy.hpp" //Provides std::filesystem and Handy::File
#include <opencv2/opencv.hpp>

//Project Includes
#include "../../EigenAliases.h"
#include "Drone.hpp"

namespace DroneInterface {
	//StartPos_LL holds the lat and lon of the first waypoint (radians)
	//GroundAlt and HAG are in meters.
	WaypointMission CreateSampleWaypointMission(int NumWaypoints, bool CurvedTrajectories, bool LandAtEnd, Eigen::Vector2d const & StartPos_LL,
	                                            double GroundAlt, double HAG);

}
