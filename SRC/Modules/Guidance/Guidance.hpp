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
	//Singleton class for the Guidance system
	class GuidanceEngine {
		private:
			std::thread       m_engineThread;
			bool              m_running;
			std::atomic<bool> m_abort;
			std::mutex        m_mutex;
			
			bool m_missionPrepDone;
			std::vector<DroneInterface::Drone *> m_dronesUnderCommand; //pointers to the drones we are allowed to command for current mission
			std::unordered_map<std::string, std::tuple<int, DroneInterface::WaypointMission>> m_currentDroneMissions; //Serial -> (missionNum, Mission)
			
			//Add additional necessary state data to keep track of mission progress, etc.
			PolygonCollection m_surveyRegion;
			//other items... e.g. the partition of the survey region, pre-planned waypoint missions for each, a vector of completed sub-regions, etc.
			
			inline void ModuleMain(void);
			
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW /*May not be needed - check for fixed-size Eigen data types in fields*/
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
			
			static GuidanceEngine & Instance() { static GuidanceEngine Obj; return Obj; }
			
			//Constructors and Destructors
			GuidanceEngine() : m_running(false), m_abort(false), m_missionPrepDone(false) {
				m_engineThread = std::thread(&GuidanceEngine::ModuleMain, this);
			}
			~GuidanceEngine() {
				m_abort = true;
				if (m_engineThread.joinable())
					m_engineThread.join();
			}
			
			//Starting a survey mission and adding a drone will automatically instruct the vehicle control to stop commanding the relavent drones
			bool StartSurvey(std::vector<std::string> const & LowFlierSerials); //Start a survey mission (currently active region) using the given drones
			bool AddLowFlier(std::string const & Serial); //Add a drone to the collection of low fliers and start commanding it
			bool RemoveLowFlier(std::string const & Serial); //Stop commanding the drone with the given serial
			bool IsRunning(void); //Returns true if currently commanding a mission, false otherwise
			
			inline std::vector<std::string> GetSerialsOfDronesUnderCommand(void);
			inline std::string GetMissionStatusStr(void); //Get status string for current mission
			inline std::string GetMissionProgressStr(void); //Get progress string for current mission
			
			//Abort mission methods - after any of these are called the guidance module will stop issuing commands to drones.
			//We have 3 abort methods, each corresponding to a different state to put the drones in when aborting the mission.
			inline void AbortMission_AllDronesHover(void);
			inline void AbortMission_AllDronesLandNow(void);
			inline void AbortMission_AllDronesReturnHomeAndLand(void);
	};
	
	//A struct containing a few parameters typically needed at the same time to define a conventional serpentine waypoint mission.
	//The SidelapFraction, HAG, and HFOV detirmine the row spacing for passes in a serpentine flight pattern according to equation:
	//Row spacing = 2 * HAG * tan(0.5 * HFOV) * (1 - SidelapFraction)
	struct ImagingRequirements {
		public:
			double HAG;             //Height Above Ground (m)
			double TargetSpeed;     //Target speed to fly while imaging (m/s). This is a speed limit- drones will often fly slower than this, e.g. approaching waypoint
			double SidelapFraction; //0-1: The fraction of the horizontal footprint of an image from one pass that should be visible in an adjacent pass
			double HFOV;            //Horizontal Field Of View (radians) for the camera on the low-flying drones
	};
	
	// *********************************   Functions needed to support Guidance Engine   *********************************
	//1 - Take two points and estimate the time (s) it would take a drone to fly from one to the other (stopped at start and end), assuming a max flight speed (m/s)
	double EstimateMissionTime(DroneInterface::Waypoint const & A, DroneInterface::Waypoint const & B, double TargetSpeed);
	
	//2 - Take a waypoint mission and estimate the time (s) it will take a drone to fly it (not including take-off and landing, or movement to the region).
	//    Support both the mode where we come to a stop at each waypoint and the mode where we do not stop at waypoints (CurvedTrajectory field of Mission)
	double EstimateMissionTime(DroneInterface::WaypointMission const & Mission);
	
	//3 - Take a survey region, and break it into sub-regions of similar size that can all be flown in approximately the same flight time (argument).
	//    A good partition uses as few components as possible for a given target execution time. Hueristically, this generally means simple shapes.
	//    This function needs the target drone speed and imaging requirements because they impact what sized region can be flown in a given amount of time.
	//Arguments:
	//Region           - Input  - The input survey region to cover (polygon collection in NM coords)
	//Partition        - Output - A vector of sub-regions, each one a polygon collection in NM coords (typical case will have a single poly in each sub-region)
	//TargetFlightTime - Input  - The approx time (in seconds) a drone should be able to fly each sub-region in, given the max vehicle speed and sidelap
	//ImagingReqs      - Input  - Parameters specifying speed and row spacing (see definitions in struct declaration)
	void PartitionSurveyRegion(PolygonCollection const & Region, std::Evector<PolygonCollection> & Partition, double TargetFlightTime,
	                           ImagingRequirements const & ImagingReqs);
	
	//4 - Take a region or sub-region and plan a trajectory to cover it at a given height that meets the specified imaging requirements. In this case we specify
	//    the imaging requirements using a maximum speed and sidelap fraction.
	//Arguments:
	//Region      - Input  - The input survey region or sub-region to cover (polygon collection in NM coords)
	//Mission     - Output - The planned mission that covers the input region
	//ImagingReqs - Input  - Parameters specifying speed and row spacing (see definitions in struct declaration)
	void PlanMission(PolygonCollection const & Region, DroneInterface::WaypointMission & Mission, ImagingRequirements const & ImagingReqs);
	
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
	//TA                - Input - Time Available function
	//SubregionMissions - Input - A vector of drone Missions - Element n is the mission for sub-region n.
	//StartPos          - Input - The starting position of the drone (to tell us how far away from each sub-region mission it is)
	//
	//Returns: The index of the drone mission (and sub-region) to task the drone to. Returns -1 if none are plausable
	int SelectSubRegion(ShadowPropagation::TimeAvailableFunction const & TA, std::vector<DroneInterface::WaypointMission> const & SubregionMissions,
	                    DroneInterface::Waypoint const & StartPos);
	
	//7 - Given a Time Available function, a collection of sub-regions (with their pre-planned missions), and a collection of drone start positions, choose
	//    sequences (of a given length) of sub-regions for each drone to fly, in order. When the mission time exceeds our prediction horizon the time available
	//    function is no longer useful in chosing sub-regions but they can still be chosen in a logical fashion that avoids leaving holes in the map... making the
	//    optimistic assumption that they will be shadow-free when we get there.
	//TA                  - Input  - Time Available function
	//SubregionMissions   - Input  - A vector of drone Missions - Element n is the mission for sub-region n.
	//DroneStartPositions - Input  - Element k is the starting position of drone k
	//Sequences           - Output - Element k is a vector of sub-region indices to task drone k to (in order)
	void SelectSubregionSequnces(ShadowPropagation::TimeAvailableFunction const & TA, std::vector<DroneInterface::WaypointMission> const & SubregionMissions,
	                             std::vector<DroneInterface::Waypoint> const & DroneStartPositions, std::vector<std::vector<int>> & Sequences);
	
	
	// *********************************************************************************************************************************
	// ****************************************   GuidanceEngine Inline Functions Definitions   ****************************************
	// *********************************************************************************************************************************
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
		return "Progress info not available"s;
	}
	
	//Abort and put all commanded drones into Hover (P mode)
	inline void GuidanceEngine::AbortMission_AllDronesHover(void) {
		std::scoped_lock lock(m_mutex);
		if (! m_running)
			return;
		for (auto drone : m_dronesUnderCommand)
			drone->Hover();
		m_running = false;
		m_dronesUnderCommand.clear();
		m_currentDroneMissions.clear();
		m_surveyRegion.Clear();
	}
	
	//Abort and instruct all commanded drones to land immediately
	inline void GuidanceEngine::AbortMission_AllDronesLandNow(void) {
		std::scoped_lock lock(m_mutex);
		if (! m_running)
			return;
		for (auto drone : m_dronesUnderCommand)
			drone->LandNow();
		m_running = false;
		m_dronesUnderCommand.clear();
		m_currentDroneMissions.clear();
		m_surveyRegion.Clear();
	}
	
	//Abort and instruct all commanded drones to return home and land
	inline void GuidanceEngine::AbortMission_AllDronesReturnHomeAndLand(void) {
		std::scoped_lock lock(m_mutex);
		if (! m_running)
			return;
		for (auto drone : m_dronesUnderCommand)
			drone->GoHomeAndLand();
		m_running = false;
		m_dronesUnderCommand.clear();
		m_currentDroneMissions.clear();
		m_surveyRegion.Clear();
	}
	
	inline void GuidanceEngine::ModuleMain(void) {
		while (! m_abort) {
			m_mutex.lock();
			if (! m_running) {
				m_mutex.unlock();
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}
			
			//We are running - see if we have done the prep work yet. If not, do all the initial setup work that needs to be done
			if (! m_missionPrepDone) {
				//A - Partition the survey region
				//B - Compute pre-planned waypoint missions for each sub-region
				
				//TEMP - Just for testing the Guidance overlay
				MapWidget::Instance().m_guidanceOverlay.SetGuidanceMessage1("This is a message from the guidance module!");
				MapWidget::Instance().m_guidanceOverlay.SetGuidanceMessage2("Wow - Another message from the guidance module!");
				MapWidget::Instance().m_guidanceOverlay.SetGuidanceMessage3("Wow - A third message from the guidance module!");
				
				//Create a sample survey region partition to give to the guidance overlay
				std::Evector<PolygonCollection> Partition;
				
				double PI = 3.14159265358979;
				std::Evector<Eigen::Vector2d> vertices_NM;
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.239506, -95.314901)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.239493, -95.310003)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.242162, -95.309846)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.242162, -95.314849)*PI/180.0));
				Partition.emplace_back(); //Create a new element in the partition
				Partition.back().m_components.emplace_back(); //Add a component to the new element
				Partition.back().m_components.back().m_boundary.SetBoundary(vertices_NM);
				
				vertices_NM.clear();
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.242162, -95.309899)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.242162, -95.314831)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.245044, -95.314761)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.245019, -95.309846)*PI/180.0));
				Partition.emplace_back(); //Create a new element in the partition
				Partition.back().m_components.emplace_back(); //Add a component to the new element
				Partition.back().m_components.back().m_boundary.SetBoundary(vertices_NM);
				
				vertices_NM.clear();
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.244970, -95.309858)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.244988, -95.314735)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.246652, -95.314735)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.246634, -95.309884)*PI/180.0));
				Partition.emplace_back(); //Create a new element in the partition
				Partition.back().m_components.emplace_back(); //Add a component to the new element
				Partition.back().m_components.back().m_boundary.SetBoundary(vertices_NM);
				
				vertices_NM.clear();
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.244194, -95.308155)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.245155, -95.306839)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.241236, -95.301317)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.240349, -95.302530)*PI/180.0));
				Partition.emplace_back(); //Create a new element in the partition
				Partition.back().m_components.emplace_back(); //Add a component to the new element
				Partition.back().m_components.back().m_boundary.SetBoundary(vertices_NM);
				
				MapWidget::Instance().m_guidanceOverlay.SetSurveyRegionPartition(Partition);
				
				//Create a sample vector of triangles to give to the guidance overlay
				PolygonCollection tempPolyCollection;
				vertices_NM.clear();
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.239494, -95.310018)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.246525, -95.309825)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.246628, -95.309994)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.246645, -95.314757)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.243130, -95.314805)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.239528, -95.314901)*PI/180.0));
				tempPolyCollection.m_components.emplace_back();
				tempPolyCollection.m_components.back().m_boundary.SetBoundary(vertices_NM);
				
				vertices_NM.clear();
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.244209, -95.308160)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.245130, -95.306817)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.243331, -95.304286)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.241267, -95.301327)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.240332, -95.302515)*PI/180.0));
				vertices_NM.push_back(LatLonToNM(Eigen::Vector2d(44.241886, -95.304733)*PI/180.0));
				tempPolyCollection.m_components.emplace_back();
				tempPolyCollection.m_components.back().m_boundary.SetBoundary(vertices_NM);
				
				std::Evector<Triangle> triangles;
				tempPolyCollection.Triangulate(triangles);
				MapWidget::Instance().m_guidanceOverlay.SetTriangles(triangles);
				
				m_missionPrepDone = true; //Mark the prep work as done
			}
			
			//If we get here we are executing a mission
			//1 - Check to see if we need to do anything. We should do an update if:
			//   A - There are sub-regions without assigned drones and we have drones without an assigned mission
			//   B - A commanded drone has finished it's assigned mission
			//   C - A drone is flying a mission and it looks like it's going to get hit with a shadow before it can finish
			//If we decide there is no work to do, unlock, snooze and continue
			
			//2 - We need to do an update - identify which drones need to be re-assigned
			//If any drone has finished a mission, mark the sub-region it just flew as finished.
			//For each drone that needs to be given a mission, select an available sub-region (not already flown and not expected to be hit with shadows)
			//Upload the corresponding mission for that sub-region and update m_currentDroneMissions.
			
			//Unlock and snooze - updates shouldn't need to happen in rapid succession so don't worry about snoozing here
			m_mutex.unlock();
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
}





