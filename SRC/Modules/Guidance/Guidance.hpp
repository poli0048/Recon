//This module provides the main interface for the guidance system
//Authors: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.
#pragma once

//System Includes
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <system_error>

//External Includes
#include "../../../../handycpp/Handy.hpp"
#include <opencv2/opencv.hpp>

//Project Includes
#include "../../EigenAliases.h"
#include "../../SurveyRegionManager.hpp"
#include "../../Polygon.hpp"
#include "../Shadow-Propagation/ShadowPropagation.hpp"
#include "../DJI-Drone-Interface/DroneManager.hpp"
#include "../../UI/MapWidget.hpp"
#include "../../UI/GuidanceOverlay.hpp"
#include "../../Maps/MapUtils.hpp"

//Much like the other modules, we have a singleton class with a private thread for running guidance algorithms. When active, this
//thread will grab time available functions directly from the shadow propagation module and will send drone commands directly to the
//drone interface module. However, to support algorithm development we also make the lower-level guidance components public so they
//can be run offline in test benches.
namespace Guidance {
	//A struct containing a few parameters typically needed at the same time to define a conventional serpentine waypoint mission.
	//Also specifies some other mission options like whether to stagger drone heights, etc.
	//The SidelapFraction, HAG, and HFOV determine the row spacing for passes in a serpentine flight pattern according to equation:
	//Row spacing = 2 * HAG * tan(0.5 * HFOV) * (1 - SidelapFraction)
	struct MissionParameters {
		public:
			double HAG;             //Height Above Ground (m)
			double TargetSpeed;     //Target speed to fly while imaging (m/s). This is a speed limit- drones will often fly slower than this, e.g. approaching waypoint
			double SidelapFraction; //0-1: The fraction of the horizontal footprint of an image from one pass that should be visible in an adjacent pass
			double HFOV;            //Horizontal Field Of View (radians) for the camera on the low-flying drones

			double HeightStaggerInterval;     //Spacing between drone flight planes (m)
			double TakeoffStaggerInterval;    //Time between successive take-offs for drones on the ground at mission start (s)
			double SubregionTargetFlightTime; //Target subregion size (# of seconds it should take to fly subregion)

			MissionParameters() :
				HAG(150.0 * 0.3048),
				TargetSpeed(33.6 * 0.44704),
				SidelapFraction(0.75),
				HFOV(35.0 * 1.745329251994330e-02),
				HeightStaggerInterval(10.0 * 0.3048),
				TakeoffStaggerInterval(15.0),
				SubregionTargetFlightTime(100.0) { }
	};
	
	//Singleton class for the Guidance system
	class GuidanceEngine {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;

		private:
			std::thread       m_engineThread;
			bool              m_running;
			std::atomic<bool> m_abort;
			std::mutex        m_mutex;
			int               m_MessageToken1; //Set in constructor - no locking needed to read
			int               m_MessageToken2; //Set in constructor - no locking needed to read
			int               m_MessageToken3; //Set in constructor - no locking needed to read
			
			//This block holds the variables that we are given or that we latch when starting a mission
			std::vector<DroneInterface::Drone *> m_dronesUnderCommand; //pointers to the drones we are allowed to command for current mission
			PolygonCollection m_surveyRegion;
			MissionParameters m_MissionParams;

			//This block holds the results of any mission prep work (1-off things that need to be computed once per mission)
			bool m_missionPrepDone;
			std::Evector<PolygonCollection> m_surveyRegionPartition;
			std::vector<DroneInterface::WaypointMission> m_droneMissions; //Item n covers component n of the partition
			std::Eunordered_map<std::string, TimePoint> m_droneAllowedTakeoffTimes; //Serial -> timepoint after which drone can take off
			std::Eunordered_map<std::string, double> m_droneHAGs; //Serial -> HAG (m), Values may be different if staggered.

			//This block holds fields that are periodically updated throughout a mission
			ShadowPropagation::TimeAvailableFunction m_TA; //Local copy of most recent time available function

			//Drone state activity definition: 0 = On ground (available), 1 = In air (available), 2 = Tasked with mission (not started), 3 = Mission ongoing
			std::unordered_map<std::string, std::tuple<int, int, TimePoint>> m_droneStates; //Serial -> (activity, missionIndex, timestamp). missionIndex = -1 when not tasked or flying a real mission
			std::unordered_map<int, double> m_taskedMissionDistances; //MissionIndex -> Total travel distance for mission (including getting to WP0)
			std::unordered_map<int, double> m_taskedMissionProgress;  //MissionIndex -> Distance traveled since mission start
			std::Eunordered_map<std::string, Eigen::Vector3d> m_dronePositions; //Serial -> last known position (LLA)
			std::unordered_set<int> m_availableMissionIndices; //Indices of missions that have not been assigned yet
			
			void ModuleMain(void);
			void ResetIntermediateData(void); //Clear mission prep data and periodically updated fields
			void UpdateDronePositionsAndMissionProgress(void);
			void MissionPrepWork(int PartitioningMethod); //Any work that needs to be done before starting a mission is done here
			void TaskDroneWithMission(DroneInterface::Drone * MyDrone, DroneInterface::WaypointMission Mission);
			void UpdateDroneStatesBasedOnMissionProgress(void);
			std::vector<DroneInterface::Drone *> GetDronesAvailableForTasking(void);
			void TaskDroneToAvailableMission(DroneInterface::Drone * DroneToTask);
			void UpdateGuidanceOverlayWithMissionSequencesAndProgress(void);
			bool AreAnyDronesTaskedWithOrFlyingMissions(void);
			void AbortMissionsPredictedToGetHitWithShadows(void);
			
		public:
			static GuidanceEngine & Instance() { static GuidanceEngine Obj; return Obj; }
			
			//Constructors and Destructors
			GuidanceEngine() : m_running(false), m_abort(false), m_missionPrepDone(false) {
				m_TA.Timestamp  = std::chrono::steady_clock::now(); //Will ensure any new TA functions trigger an update
				m_MessageToken1 = MapWidget::Instance().m_messageBoxOverlay.GetAvailableToken();
				m_MessageToken2 = MapWidget::Instance().m_messageBoxOverlay.GetAvailableToken();
				m_MessageToken3 = MapWidget::Instance().m_messageBoxOverlay.GetAvailableToken();
				m_engineThread = std::thread(&GuidanceEngine::ModuleMain, this);
			}
			~GuidanceEngine() { Shutdown(); }
			inline void Shutdown(void); //Stop processing and terminate engine thread
			
			//Starting a survey mission and adding a drone will automatically instruct the vehicle control to stop commanding the relavent drones
			bool StartSurvey(std::vector<std::string> const & LowFlierSerials, MissionParameters const & Params); //Start a survey mission (currently active region) using the given drones
			bool AddDroneToMission(std::string const & Serial); //Take control of a drone and add it to the current mission
			bool RemoveDroneFromMission(std::string const & Serial); //Stop commanding the drone with the given serial
			bool IsCommandingDrone(std::string const & Serial); //Returns true if drone is under guidance module command - false otherwise
			bool IsRunning(void); //Returns true if currently commanding a mission, false otherwise

			inline std::vector<std::string> GetSerialsOfDronesUnderCommand(void);
			inline std::string GetMissionStatusStr(void); //Get status string for current mission
			inline std::string GetMissionProgressStr(void); //Get progress string for current mission
			
			inline void AbortMission(void); //Stop the current mission (if one is running) and stop commanding all drones
	};
	
	// *********************************   Functions needed to support Guidance Engine   *********************************
	//1 - Take two points and estimate the time (s) it would take a drone to fly from one to the other (stopped at start and end), assuming a max flight speed (m/s)
	double EstimateMissionTime(DroneInterface::Waypoint const & A, DroneInterface::Waypoint const & B, double TargetSpeed);
	
	//2 - Take a waypoint mission and estimate the time (s) it will take a drone to fly it (not including take-off and landing, or movement to the region).
	//    Support both the mode where we come to a stop at each waypoint and the mode where we do not stop at waypoints (CurvedTrajectory field of Mission)
	double EstimateMissionTime(DroneInterface::WaypointMission const & Mission, double TargetSpeed);
	
	//3 - Take a survey region, and break it into sub-regions of similar size that can all be flown in approximately the same flight time (argument).
	//    A good partition uses as few components as possible for a given target execution time. Hueristically, this generally means simple shapes.
	//    This function needs the target drone speed and imaging requirements because they impact what sized region can be flown in a given amount of time.
	//Arguments:
	//Region           - Input  - The input survey region to cover (polygon collection in NM coords)
	//Partition        - Output - A vector of sub-regions, each one a polygon collection in NM coords (typical case will have a single poly in each sub-region)
	//MissionParams    - Input  - Parameters specifying speed and row spacing (see definitions in struct declaration)
	//
	//Note: These functions are not defined in Guidance.cpp, but are instead each in their own source files (RegionPartitioning_*.cpp)
	void PartitionSurveyRegion_TriangleFusion(PolygonCollection const & Region, std::Evector<PolygonCollection> & Partition, MissionParameters const & MissionParams);
	void PartitionSurveyRegion_IteratedCuts  (PolygonCollection const & Region, std::Evector<PolygonCollection> & Partition, MissionParameters const & MissionParams);
	
	//4 - Take a region or sub-region and plan a trajectory to cover it at a given height that meets the specified imaging requirements. In this case we specify
	//    the imaging requirements using a maximum speed and sidelap fraction.
	//Arguments:
	//Region        - Input  - The input survey region or sub-region to cover (polygon collection in NM coords)
	//Mission       - Output - The planned mission that covers the input region
	//MissionParams - Input  - Parameters specifying speed and row spacing (see definitions in struct declaration)
	//StartPos      - Input  - Optional: Initial position of vehicle (does not impact waypoints, but may impact ordering)
	//
	//Note: This function is not defined in Guidance.cpp, but is instead in FlightPlanning.cpp
	void PlanMission(PolygonCollection const & Region, DroneInterface::WaypointMission & Mission, MissionParameters const & MissionParams,
	                 DroneInterface::Waypoint const * StartPos);
	
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
	                                       double DroneStartWaypoint, std::chrono::time_point<std::chrono::steady_clock> DroneStartTime, double & Margin);
	
	//6 - Given a Time Available function, a collection of sub-regions (with their pre-planned missions), and a start position for a drone, select a sub-region
	//    to task the drone to. We are balancing 2 things here - trying to do useful work while avoiding shadows, but also avoiding non-sensical jumping to a
	//    distant region. At a minimum we should ensure that we don't task a drone to a region that we don't expect it to be able to finish without getting hit
	//    with shadows.
	//Arguments:
	//TA                      - Input - Time Available function
	//SubRegionsNM            - Input - Polygon collections representing each sub-region (in Normalized Mercator)
	//SubregionMissions       - Input - A vector of drone Missions - Element n is the mission for sub-region n.
	//AvailableMissionIndices - Input - Set of indices of missions in SubregionMissions to consider
	//StartPos                - Input - The starting position of the drone (to tell us how far away from each sub-region mission it is)
	//MissionParams           - Input - Parameters specifying speed and row spacing (see definitions in struct declaration)
	//
	//Returns: The index of the drone mission (and sub-region) to task the drone to. Returns -1 if none are plausible
	int SelectSubRegion(ShadowPropagation::TimeAvailableFunction const & TA, std::Evector<PolygonCollection> const & SubRegionsNM,
	                    std::vector<DroneInterface::WaypointMission> const & SubregionMissions, std::unordered_set<int> const & AvailableMissionIndices,
	                    DroneInterface::Waypoint const & StartPos, MissionParameters const & MissionParams);
	
	//7 - Given a Time Available function, a collection of sub-regions (with their pre-planned missions), and a collection of drone start positions, choose
	//    sequences (of a given length) of sub-regions for each drone to fly, in order. When the mission time exceeds our prediction horizon the time available
	//    function is no longer useful in chosing sub-regions but they can still be chosen in a logical fashion that avoids leaving holes in the map... making the
	//    optimistic assumption that they will be shadow-free when we get there.
	//TA                  - Input  - Time Available function
	//SubregionMissions   - Input  - A vector of drone Missions - Element n is the mission for sub-region n.
	//DroneStartPositions - Input  - Element k is the starting position of drone k
	//Sequences           - Output - Element k is a vector of sub-region indices to task drone k to (in order)
	//
	//Note: This function is not currently used - and is not currently implemented. The initial implementation was removed since
	//it was unsafe (very high computational burden in many cases would lead to full program hang if not called very carefully)
	void SelectSubregionSequences(ShadowPropagation::TimeAvailableFunction const & TA, std::vector<DroneInterface::WaypointMission> const & SubregionMissions,
	                             std::vector<DroneInterface::Waypoint> const & DroneStartPositions, std::set<int> MissionIndicesToAssign, std::vector<std::vector<int>> & Sequences,
	                             MissionParameters const & MissionParams);

	// *********************************************************************************************************************************
	// ****************************************   GuidanceEngine Inline Functions Definitions   ****************************************
	// *********************************************************************************************************************************
	//Stop processing and terminate engine thread
	inline void GuidanceEngine::Shutdown(void) {
		m_abort = true;
		if (m_engineThread.joinable())
			m_engineThread.join();
	}

	inline std::vector<std::string> GuidanceEngine::GetSerialsOfDronesUnderCommand(void) {
		std::scoped_lock lock(m_mutex);
		std::vector<std::string> serials;
		serials.reserve(m_dronesUnderCommand.size());
		for (auto drone : m_dronesUnderCommand)
			serials.push_back(drone->GetDroneSerial());
		return serials;
	}
	
	//Get status string for current mission
	inline std::string GuidanceEngine::GetMissionStatusStr(void) {
		std::scoped_lock lock(m_mutex);
		if (! m_running)
			return "No mission in progress"s;
		else if (m_dronesUnderCommand.size() == 1U)
			return "Executing mission - Commanding 1 drone"s;
		else
			return "Executing mission - Commanding "s + std::to_string((unsigned int) m_dronesUnderCommand.size()) + " drones"s;
	}
	
	//Get progress string for current mission
	inline std::string GuidanceEngine::GetMissionProgressStr(void) {
		std::scoped_lock lock(m_mutex);
		if (m_running) {
			int numMissions         = (int) m_droneMissions.size();
			int numStillAvailable   = (int) m_availableMissionIndices.size();
			int numDoneOrInProgress = numMissions - numStillAvailable;
			std::string progressStr = std::to_string(numDoneOrInProgress) + " of "s +
			                          std::to_string(numMissions) + " sub-regions done/in progress"s;
			return progressStr;
		}
		else
			return ""s;
	}

	inline void GuidanceEngine::AbortMission(void) {
		std::scoped_lock lock(m_mutex);
		if (! m_running)
			return;
		m_running = false;
		m_dronesUnderCommand.clear();
	}
}





