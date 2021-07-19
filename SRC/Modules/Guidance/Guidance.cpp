//This module provides the main interface for the guidance system
//Authors: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//System Includes
//#include <vector>
//#include <string>
//#include <thread>
//#include <mutex>
//#include <iostream>
//#include <iomanip>
//#include <ctime>
//#include <sstream>
//#include <system_error>

//External Includes

//Project Includes
#include "Guidance.hpp"
#include "../../UI/VehicleControlWidget.hpp"
//#include "../../EigenAliases.h"
//#include "../../SurveyRegionManager.hpp"
//#include "../../Polygon.hpp"
//#include "../Shadow-Propagation/ShadowPropagation.hpp"
//#include "../DJI-Drone-Interface/DroneManager.hpp"


namespace Guidance {
	// *********************************************************************************************************************************
	// **************************************   GuidanceEngine Non-Inline Functions Definitions   **************************************
	// *********************************************************************************************************************************
	//Start a survey mission (currently active region) using the given drones
	bool GuidanceEngine::StartSurvey(std::vector<std::string> const & LowFlierSerials) {
		std::scoped_lock lock(m_mutex);
		if (m_running)
			return false; //Require stopping the previous mission first
		if (LowFlierSerials.empty())
			return false; //Require at least 1 drone to start a mission
		if (! SurveyRegionManager::Instance().GetCopyOfActiveRegionData(nullptr, &m_surveyRegion, nullptr))
			return false; //No active survey region
		m_dronesUnderCommand.clear();
		for (std::string serial : LowFlierSerials) {
			DroneInterface::Drone * ptr = DroneInterface::DroneManager::Instance().GetDrone(serial);
			if (ptr != nullptr) {
				m_dronesUnderCommand.push_back(ptr);
				VehicleControlWidget::Instance().StopCommandingDrone(serial);
			}
		}
		if (m_dronesUnderCommand.size() == LowFlierSerials.size()) {
			m_running = true;
			m_missionPrepDone = false; //This will trigger the pre-planning work that needs to happen for a new mission
			return true;
		}
		else
			return false;
	}
	
	//Add a drone to the collection of low fliers and start commanding it
	bool GuidanceEngine::AddLowFlier(std::string const & Serial) {
		std::scoped_lock lock(m_mutex);
		if (! m_running)
			return false;
		else {
			for (auto drone : m_dronesUnderCommand) {
				if (drone->GetDroneSerial() == Serial) {
					VehicleControlWidget::Instance().StopCommandingDrone(Serial);
					return true;
				}
			}
			DroneInterface::Drone * ptr = DroneInterface::DroneManager::Instance().GetDrone(Serial);
			if (ptr != nullptr) {
				m_dronesUnderCommand.push_back(ptr);
				VehicleControlWidget::Instance().StopCommandingDrone(Serial);
				return true;
			}
			else
				return false;
		}
	}
	
	//Stop commanding the drone with the given serial
	bool GuidanceEngine::RemoveLowFlier(std::string const & Serial) {
		std::scoped_lock lock(m_mutex);
		if (! m_running)
			return false;
		for (size_t n = 0U; n < m_dronesUnderCommand.size(); n++) {
			if (m_dronesUnderCommand[n]->GetDroneSerial() == Serial) {
				m_currentDroneMissions.erase(Serial);
				m_dronesUnderCommand.erase(m_dronesUnderCommand.begin() + n);
				
				//If we just removed the last drone, abort the mission - this makes it unnecessary to have an extra UI control
				//to cancel a mission that has effectively already been canceled through the removal of all drones.
				if (m_dronesUnderCommand.empty()) {
					m_running = false;
					m_currentDroneMissions.clear();
					m_surveyRegion.Clear();
				}
				
				return true;
			}
		}
		return false;
	}
	
	//Returns true if currently commanding a mission, false otherwise
	bool GuidanceEngine::IsRunning(void) {
		std::scoped_lock lock(m_mutex);
		return m_running;
	}
	
	
	// *********************************************************************************************************************************
	// *******************************************   Guidance Algorithm Function Definitions   *****************************************
	// *********************************************************************************************************************************
	
	//1 - Take two points and estimate the time (s) it would take a drone to fly from one to the other (stopped at start and end), assuming a max flight speed (m/s)
	double EstimateMissionTime(DroneInterface::Waypoint const & A, DroneInterface::Waypoint const & B, double TargetSpeed) {
		//TODO
		return 0.0;
	}
	
	//2 - Take a waypoint mission and estimate the time (s) it will take a drone to fly it (not including take-off and landing, or movement to the region).
	//    Support both the mode where we come to a stop at each waypoint and the mode where we do not stop at waypoints (CurvedTrajectory field of Mission)
	double EstimateMissionTime(DroneInterface::WaypointMission const & Mission) {
		//TODO
		return 0.0;
	}
	
	//3 - Take a survey region, and break it into sub-regions of similar size that can all be flown in approximately the same flight time (argument).
	//    A good partition uses as few components as possible for a given target execution time. Hueristically, this generally means simple shapes.
	//    This function needs the target drone speed and imaging requirements because they impact what sized region can be flown in a given amount of time.
	//Arguments:
	//Region           - Input  - The input survey region to cover (polygon collection in NM coords)
	//Partition        - Output - A vector of sub-regions, each one a polygon collection in NM coords (typical case will have a single poly in each sub-region)
	//TargetFlightTime - Input  - The approx time (in seconds) a drone should be able to fly each sub-region in, given the max vehicle speed and sidelap
	//ImagingReqs      - Input  - Parameters specifying speed and row spacing (see definitions in struct declaration)
	void PartitionSurveyRegion(PolygonCollection const & Region, std::Evector<PolygonCollection> & Partition, double TargetFlightTime,
	                           ImagingRequirements const & ImagingReqs) {
		//TODO
		Partition.clear();
	}
	
	//4 - Take a region or sub-region and plan a trajectory to cover it at a given height that meets the specified imaging requirements. In this case we specify
	//    the imaging requirements using a maximum speed and sidelap fraction.
	//Arguments:
	//Region      - Input  - The input survey region or sub-region to cover (polygon collection in NM coords)
	//Mission     - Output - The planned mission that covers the input region
	//ImagingReqs - Input  - Parameters specifying speed and row spacing (see definitions in struct declaration)
	void PlanMission(PolygonCollection const & Region, DroneInterface::WaypointMission & Mission, ImagingRequirements const & ImagingReqs) {
		//TODO
		Mission.Waypoints.clear();
		Mission.LandAtLastWaypoint = false;
		Mission.CurvedTrajectory = false;
	}
	
	//5 - Take a Time Available function, a waypoint mission, and a progress indicator (where in the mission you are) and detirmine whether or not the drone
	//    will be able to complete the mission in the time remaining (i.e. at no point will the time available within a radius of the drone hit 0).
	//    Additionally, we compute some notion of confidence as follows... we find the lowest that the TA function will be under the drone throughout the mission.
	//    If this gets closer to 0 it means we are cutting it close and if the predicted TA function is off we could be in trouble.
	//Arguments:
	//TA                 - Input  - Time Available function
	//Mission            - Input  - Drone Mission
	//DroneStartWaypoint - Input  - The index of the waypoint the drone starts at. Can be non-integer - e.g. 2.4 means 40% of way between waypoint 2 and 3.
	//DroneStartTime     - Input  - The time when the drone starts the mission. This is used to compare with the timestamped TA function
	//Margin             - Output - The lowest the TA ever gets under the drone during the mission
	//
	//Returns: True if expected to finish without shadows and false otherwise
	bool IsPredictedToFinishWithoutShadows(ShadowPropagation::TimeAvailableFunction const & TA, DroneInterface::WaypointMission const & Mission,
	                                       double DroneStartWaypoint, std::chrono::time_point<std::chrono::steady_clock> DroneStartTime, double & Margin) {
		//TODO
		return false;
	}
	
	//6 - Given a Time Available function, a collection of sub-regions (with their pre-planned missions), and a start position for a drone, select a sub-region
	//    to task the drone to. We are balancing 2 things here - trying to do useful work while avoiding shadows, but also avoiding non-sensical jumping to a
	//    distant region. At a minimum we should ensure that we don't task a drone to a region that we don't expect it to be able to finish without getting hit
	//    with shadows.
	//Arguments:
	//TA                - Input - Time Available function
	//SubregionMissions - Input - A vector of drone Missions - Element n is the mission for sub-region n.
	//StartPos          - Input - The starting position of the drone (to tell us how far away from each sub-region mission it is)
	//
	//Returns: The index of the drone mission (and sub-region) to task the drone to. Returns -1 if none are plausable
	int SelectSubRegion(ShadowPropagation::TimeAvailableFunction const & TA, std::vector<DroneInterface::WaypointMission> const & SubregionMissions,
	                    DroneInterface::Waypoint const & StartPos) {
		//TODO
		return -1;
	}
	
	//7 - Given a Time Available function, a collection of sub-regions (with their pre-planned missions), and a collection of drone start positions, choose
	//    sequences (of a given length) of sub-regions for each drone to fly, in order. When the mission time exceeds our prediction horizon the time available
	//    function is no longer useful in chosing sub-regions but they can still be chosen in a logical fashion that avoids leaving holes in the map... making the
	//    optimistic assumption that they will be shadow-free when we get there.
	//TA                  - Input  - Time Available function
	//SubregionMissions   - Input  - A vector of drone Missions - Element n is the mission for sub-region n.
	//DroneStartPositions - Input  - Element k is the starting position of drone k
	//Sequences           - Output - Element k is a vector of sub-region indices to task drone k to (in order)
	void SelectSubregionSequnces(ShadowPropagation::TimeAvailableFunction const & TA, std::vector<DroneInterface::WaypointMission> const & SubregionMissions,
	                             std::vector<DroneInterface::Waypoint> const & DroneStartPositions, std::vector<std::vector<int>> & Sequences) {
		//TODO
		Sequences.clear();
	}
	
	
}





