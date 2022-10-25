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
#include "../../Utilities.hpp"
#include "../../Maps/MapUtils.hpp"
//#include "../../EigenAliases.h"
//#include "../../SurveyRegionManager.hpp"
//#include "../../Polygon.hpp"
//#include "../Shadow-Propagation/ShadowPropagation.hpp"
//#include "../DJI-Drone-Interface/DroneManager.hpp"

#define PI 3.14159265358979323846
//#define TOLERANCE 1e-10
namespace Guidance {
    // *********************************************************************************************************************************
    // **************************************   GuidanceEngine Non-Inline Functions Definitions   **************************************
    // *********************************************************************************************************************************
    //Start a survey mission (currently active region) using the given drones
    bool GuidanceEngine::StartSurvey(std::vector<std::string> const & LowFlierSerials, MissionParameters const & Params) {
        std::scoped_lock lock(m_mutex);
        if (m_running)
            return false; //Require stopping the previous mission first
        if (LowFlierSerials.empty())
            return false; //Require at least 1 drone to start a mission
        if (! SurveyRegionManager::Instance().GetCopyOfActiveRegionData(nullptr, &m_surveyRegion, nullptr))
            return false; //No active survey region
        m_MissionParams = Params;
        m_dronesUnderCommand.clear();
        for (std::string serial : LowFlierSerials) {
            DroneInterface::Drone * ptr = DroneInterface::DroneManager::Instance().GetDrone(serial);
            if (ptr != nullptr) {
                m_dronesUnderCommand.push_back(ptr);
                VehicleControlWidget::Instance().StopCommandingDrone(serial);
            }
        }
        if (m_dronesUnderCommand.size() == LowFlierSerials.size()) {
            ResetIntermediateData();
            m_missionPrepDone = false; //This will trigger the pre-planning work that needs to happen for a new mission
            MapWidget::Instance().m_guidanceOverlay.Reset(); //Clear any previous guidance overlay data
            m_running = true;
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
                //m_currentDroneMissions.erase(Serial);

                //If this drone was doing something useful, update things accordingly
                if (m_droneStates.count(Serial) > 0U) {
                	if (std::get<0>(m_droneStates.at(Serial)) > 1) {
                		//The drone was tasked with or flying a mission
                		int missionIndex = std::get<1>(m_droneStates.at(Serial));
                		m_availableMissionIndices.insert(missionIndex);
                	}
                }

                //Remove the drone from our vector of drone pointers under our command
                m_dronesUnderCommand.erase(m_dronesUnderCommand.begin() + n);
                std::cerr << "Removing drone from command.\r\n";

                //If we just removed the last drone, abort the mission - this makes it unnecessary to have an extra UI control
                //to cancel a mission that has effectively already been canceled through the removal of all drones.
                if (m_dronesUnderCommand.empty()) {
                    m_running = false;
                    //m_currentDroneMissions.clear();
                    m_surveyRegion.Clear();
                }

                return true;
            }
        }
        return false;
    }

    bool GuidanceEngine::IsCommandingDrone(std::string const & Serial) {
    	std::scoped_lock lock(m_mutex);
    	if (! m_running)
            return false;
        for (size_t n = 0U; n < m_dronesUnderCommand.size(); n++) {
            if (m_dronesUnderCommand[n]->GetDroneSerial() == Serial)
            	return true;
        }
        return false;
    }

    //Returns true if currently commanding a mission, false otherwise
    bool GuidanceEngine::IsRunning(void) {
        std::scoped_lock lock(m_mutex);
        return m_running;
    }

    // TODO: Move this function to a place that makes more sense (i.e., Utilities)
    /*static void print_vec(std::string const & name, std::vector<int> & v) {
        std::cout << name << "< ";
        for (auto x : v) {
            std::cout << x << " ";
        }
        std::cout << ">";
    }*/

    /*void GuidanceEngine::GetDroneCurrentPositions(std::vector<DroneInterface::Waypoint> & DronePositions) {
        DronePositions.clear();
        DronePositions.reserve(m_dronesUnderCommand.size());

        //Get a waypoint object for the drones current position
        for (auto drone : m_dronesUnderCommand) {
            DroneInterface::Drone::TimePoint Timestamp;
            double Latitude, Longitude, Altitude, HAG;
            drone->GetPosition(Latitude, Longitude, Altitude, Timestamp);
            drone->GetHAG(HAG, Timestamp);

            DroneInterface::Waypoint Waypoint;
            Waypoint.Latitude = Latitude;
            Waypoint.Longitude = Longitude;
            Waypoint.RelAltitude = HAG;

            DronePositions.push_back(Waypoint);
        }
    }*/

    // Decides whether sequence should be updated or not, and if so, executes those missions
    /*void GuidanceEngine::RefreshSequence(void) {
        std::vector<int> currentMissionIndices, newMissionIndices;

        // Get the mission indices that are currently being flown by drones and print them out
        if (!m_currentDroneMissions.empty() && !m_subregionSequences.empty()) {
            std::cout << "Current Mission Indices: ";
            for (size_t i = 0; i < m_dronesUnderCommand.size(); i++) {
                int missionIndex = std::get<0>(m_currentDroneMissions[m_dronesUnderCommand[i]->GetDroneSerial()]);
                // if mission index of drone is less than size of assigned sequence
                if ((int) m_subregionSequences[i].size() > missionIndex) {
                    currentMissionIndices.push_back(m_subregionSequences[i][missionIndex]);
                }
                else
                    currentMissionIndices.push_back(-1); // -1 indicates that this drone is done flying missions
                std::cout << currentMissionIndices[i] << " ";
            }
            std::cout << std::endl;
        }

        m_flyingMissionStatus.clear();

        std::vector<DroneInterface::Waypoint> droneCurrentPositions;
        GetDroneCurrentPositions(droneCurrentPositions);

        int currentSequenceCoverageExpected = 0;
        int newSequenceCoverageExpected     = 0;
        int currentStartUnclouded           = 0;
        int newStartUnclouded               = 0;
        std::vector<std::vector<int>> currentSequences = m_subregionSequences;
        std::vector<int> PredictedCoverage, MissionDurations;

        m_subregionSequences.clear();
        static int prevNumMissions = 0;
        // Select subregion sequences
        SelectSubregionSequences(m_TA, m_droneMissions, droneCurrentPositions, m_missionIndicesToAssign, m_subregionSequences, m_MissionParams);
        std::cout << "Size of mission indices: " << m_missionIndicesToAssign.size() << std::endl;

        // Below, we compare coverage of new best sequence to recalculate coverage (based on updated m_TA) of the previous best found sequence
        // We only care about comparison of new coverage to old coverage if old coverage > 0 (because why would we care if it was 0 before? anything would be better than that)
        if (m_coverageExpected > 0) {
            currentSequenceCoverageExpected = GetCoverage(PredictedCoverage, MissionDurations, m_TA, m_droneMissions, currentSequences, droneCurrentPositions, m_MissionParams, m_dronesUnderCommand.size());
            currentStartUnclouded = std::accumulate(PredictedCoverage.begin(), PredictedCoverage.end(), 0);
            PredictedCoverage.clear();
            MissionDurations.clear();
        }
        newSequenceCoverageExpected = GetCoverage(PredictedCoverage, MissionDurations, m_TA, m_droneMissions, m_subregionSequences, droneCurrentPositions, m_MissionParams, m_dronesUnderCommand.size());
        newStartUnclouded = std::accumulate(PredictedCoverage.begin(), PredictedCoverage.end(), 0);
        std::cout << "current coverage: " << currentSequenceCoverageExpected << ", new coverage: " << newSequenceCoverageExpected << std::endl;

        std::cout << "current sequence: <";
        for (auto sequence : currentSequences) {
            print_vec("", sequence);
        }
        std::cout << ">" << std::endl;

        std::cout << "potential new sequence: <";
        for (auto sequence : m_subregionSequences) {
            print_vec("", sequence);
        }
        std::cout << ">" << std::endl;

        for (size_t i = 0; i < m_dronesUnderCommand.size(); i++) {
            // if there's at least one sequence assigned to the drone
            if ((int) m_subregionSequences[i].size() >= 1) {
                newMissionIndices.push_back(m_subregionSequences[i][0]);
            } else {
                newMissionIndices.push_back(-1);
            }
        }

        bool isPermutation = std::is_permutation(currentMissionIndices.begin(), currentMissionIndices.end(), newMissionIndices.begin());


        // If the number of missions left to assign has changed (i.e., mission was completed), or if using new m_TA, new sequence coverage > old sequence coverage
        if ((((int) m_missionIndicesToAssign.size() != prevNumMissions) && !isPermutation) || 
            newSequenceCoverageExpected > currentSequenceCoverageExpected || 
            (newSequenceCoverageExpected == currentSequenceCoverageExpected && newStartUnclouded > currentStartUnclouded)) {
            m_currentDroneMissions.clear();
            m_coverageExpected = newSequenceCoverageExpected;
            std::vector<bool> startMissionFromBeginning;
            std::cout << "Confirmed New Mission Indices: ";
            for (size_t i = 0; i < m_dronesUnderCommand.size(); i++) {
                // if there's at least one sequence assigned to the drone
                if ((int) m_subregionSequences[i].size() >= 1) {
                    int missionIndex = m_subregionSequences[i][0];
                    std::cout << missionIndex << " ";
                    // update flag based on if new refreshed sequence doesn't change what mission the drone is already flying (to avoid starting a mission from the beginning if already flying it)
                    if (!currentMissionIndices.empty() && (missionIndex == currentMissionIndices[i])) {
                        startMissionFromBeginning.push_back(false);
                    } else {
                        startMissionFromBeginning.push_back(true);
                    }
                    // Set current mission index to zero after each call to select subregion sequences
                    if (missionIndex != -1) {
                        m_currentDroneMissions[m_dronesUnderCommand[i]->GetDroneSerial()] = std::make_tuple(0, m_droneMissions[missionIndex]);
                    }
                } else {
                    startMissionFromBeginning.push_back(false);
                }
            }
            std::cout << std::endl;

            // Execute currently assigned missions
            for (size_t i = 0; i < m_dronesUnderCommand.size(); i++) {
                DroneInterface::Drone *drone = m_dronesUnderCommand[i];
                if (m_currentDroneMissions.count(drone->GetDroneSerial())) {
                    DroneInterface::WaypointMission Mission = std::get<1>(m_currentDroneMissions[drone->GetDroneSerial()]);
                    if (startMissionFromBeginning[i]) { // only start mission again if flag for starting mission from the beginning is set
                        std::cout << "Starting mission " << i << " from beginning." << std::endl;
                        TaskDroneWithMission(drone, Mission);
                        //drone->ExecuteWaypointMission(Mission);
                    }
                    m_flyingMissionStatus.push_back(true);
                } else {
                    m_flyingMissionStatus.push_back(false);
                }
            }
        } else {
            // Criteria for changing sequence not met, so reset any state that has changed while running this function to what it has been
            m_coverageExpected = currentSequenceCoverageExpected;
            m_subregionSequences = currentSequences;
        }
        prevNumMissions = m_missionIndicesToAssign.size();
    }*/

    //Clear mission prep data and periodically updated fields
    void GuidanceEngine::ResetIntermediateData(void) {
        m_surveyRegionPartition.clear();
        m_droneMissions.clear();
        m_droneAllowedTakeoffTimes.clear();
        m_droneHAGs.clear();
        m_droneStates.clear();
        m_dronePositions.clear();
        m_availableMissionIndices.clear();

        //m_currentDroneMissions.clear();
        //m_coverageExpected = 0;
        //m_missionIndicesToAssign.clear();
        //m_subregionSequences.clear();
        //m_flyingMissionStatus.clear();
        //m_wasPredictedToFinishWithoutShadows.clear();
    }

	void GuidanceEngine::UpdateDronePositions(void) {
		for (DroneInterface::Drone * drone : m_dronesUnderCommand) {
			std::string serial = drone->GetDroneSerial();
			double Lat = 0.0;
			double Lon = 0.0;
			double Alt = 0.0;
			TimePoint Timestamp;
			if (drone->GetPosition(Lat, Lon, Alt, Timestamp))
				m_dronePositions[serial] = Eigen::Vector3d(Lat, Lon, Alt);
			else
				m_dronePositions[serial] = Eigen::Vector3d(0.0, 0.0, 0.0); //Not great, but this is what the drone defaults to anyways
		}
	}

    //Do mission prep work, if necessary
    void GuidanceEngine::MissionPrepWork(int PartitioningMethod) {
        if (! m_missionPrepDone) {
            //TODO: Copy any internal data needed for planning work, then unlock out mutex while we do the planning
            //Re-lock and copy the results when done. This will prevent the UI from locking up if the prep work takes
            //a non-trivial amount of time. Once done, we can also use the message overlay box to communicate state during prep.

            std::cerr << "Doing mission preparation work.\r\n\r\n";
            //MapWidget::Instance().m_messageBoxOverlay.AddMessage("Preparing mission for execution..."s, m_MessageToken1);

            //Partition the survey region into components
            m_surveyRegionPartition.clear();
            if (PartitioningMethod == 0)
                PartitionSurveyRegion_TriangleFusion(m_surveyRegion, m_surveyRegionPartition, m_MissionParams);
            else if (PartitioningMethod == 1)
                PartitionSurveyRegion_IteratedCuts(m_surveyRegion, m_surveyRegionPartition, m_MissionParams);
            else
                std::cerr << "Error: Unrecognized partitioning method. Skipping partitioning.\r\n";
            MapWidget::Instance().m_guidanceOverlay.SetSurveyRegionPartition(m_surveyRegionPartition);

            //Plan missions for each component of the partition
            m_droneMissions.clear();
            m_droneMissions.reserve(m_surveyRegionPartition.size());
            for (auto const & component : m_surveyRegionPartition) {
                DroneInterface::WaypointMission Mission;
                PlanMission(component, Mission, m_MissionParams);
                m_droneMissions.push_back(Mission);
            }
            MapWidget::Instance().m_guidanceOverlay.SetMissions(m_droneMissions);

            //Initialize states, and available indices
            m_droneStates.clear();
            m_dronePositions.clear();
            m_availableMissionIndices.clear();
            TimePoint NowTime = std::chrono::steady_clock::now();
            for (DroneInterface::Drone * drone : m_dronesUnderCommand) {
                std::string serial = drone->GetDroneSerial();
                bool isFlying = false;
                TimePoint isFlyingTimestamp;
                if (drone->IsCurrentlyFlying(isFlying, isFlyingTimestamp) && (SecondsElapsed(isFlyingTimestamp) <= 4.0)) {
                    if (isFlying)
                        m_droneStates[serial] = std::make_tuple(1, -1, NowTime); //Loitering (available for tasking anyhow)
                    else
                        m_droneStates[serial] = std::make_tuple(0, -1, NowTime); //On ground and available for tasking
                }
                else {
                    std::cerr << "Bad or old telemetry for drone " << serial << ". Treating as on ground.\r\n";
                    m_droneStates[serial] = std::make_tuple(0, -1, NowTime); //On ground and available for tasking
                }
            }
            for (int missionIndex = 0; missionIndex < (int) m_droneMissions.size(); missionIndex++) {
            	//Mark all non-trivial missions as available
            	if (! m_droneMissions[missionIndex].Waypoints.empty())
            		m_availableMissionIndices.insert(missionIndex);
            }

           /* for (size_t i = 0; i < m_droneMissions.size(); i++) {
                m_missionIndicesToAssign.insert(i);  
            }

            for (size_t i = 0; i < m_dronesUnderCommand.size(); i++) {
                m_wasPredictedToFinishWithoutShadows.push_back(true);
            }

            m_coverageExpected = 0;*/
            //RefreshSequence(); //TODO: Can hang apparently

            //Populate the allowed takeoff times for all drones (use now for drones in the air) and populate
            //the HAGs to use (m) per drone. These override mission HAG since we want height to be fixed per drone and not per mission
            TimePoint nextAllowedTakeoffTime = std::chrono::steady_clock::now();
            double nextAllowedHAG = m_MissionParams.HAG;
            if (m_dronesUnderCommand.size() > 1U)
            	nextAllowedHAG += 0.5*(m_MissionParams.HeightStaggerInterval*double(m_dronesUnderCommand.size() - 1U));
            for (DroneInterface::Drone * drone : m_dronesUnderCommand) {
                std::string serial = drone->GetDroneSerial();
                bool isFlying = false;
                TimePoint isFlyingTimestamp;
                if (drone->IsCurrentlyFlying(isFlying, isFlyingTimestamp) && (SecondsElapsed(isFlyingTimestamp) <= 4.0)) {
                    //Nothing unusual here - telemetry is recent
                    if (isFlying)
                        m_droneAllowedTakeoffTimes[serial] = std::chrono::steady_clock::now();
                    else {
                        m_droneAllowedTakeoffTimes[serial] = nextAllowedTakeoffTime;
                        nextAllowedTakeoffTime = AdvanceTimepoint(nextAllowedTakeoffTime, m_MissionParams.TakeoffStaggerInterval);
                    }   
                }
                else {
                    std::cerr << "Bad or old telemetry for drone " << serial << ". Treating as on ground.\r\n";
                    m_droneAllowedTakeoffTimes[serial] = nextAllowedTakeoffTime;
                    nextAllowedTakeoffTime = AdvanceTimepoint(nextAllowedTakeoffTime, m_MissionParams.TakeoffStaggerInterval);
                }
                m_droneHAGs[serial] = nextAllowedHAG;
                nextAllowedHAG -= m_MissionParams.HeightStaggerInterval;
            }

            m_missionPrepDone = true; //Mark the prep work as done
            //MapWidget::Instance().m_messageBoxOverlay.RemoveMessage(m_MessageToken1);
        }
    }

    void GuidanceEngine::TaskDroneWithMission(DroneInterface::Drone * MyDrone, DroneInterface::WaypointMission Mission) {
        //Override mission HAG with the HAG associated with this drone
        std::string serial = MyDrone->GetDroneSerial();
        if (m_droneHAGs.count(serial) > 0U) {
            double newHAG = m_droneHAGs.at(serial);
            for (auto & WP : Mission.Waypoints)
                WP.RelAltitude = newHAG;
        }
        else
            std::cerr << "Warning: Tasking drone that wasn't part of pre-mission plan. Not overriding HAG.\r\n";
        MyDrone->ExecuteWaypointMission(Mission);
    }

    //Check each drone that was tasked with a mission to see if it is done with it and update m_droneStates accordingly
    void GuidanceEngine::UpdateDroneStatesBasedOnMissionProgress(void) {
    	//If a drone was tasked with a mission and it finished it, mark the drone as loitering
    	for (DroneInterface::Drone * drone : m_dronesUnderCommand) {
            std::string serial = drone->GetDroneSerial();
            bool isFlying = false;
            bool isDoingMission = false;
            TimePoint TimestampA, TimestampB;
            if (drone->IsCurrentlyFlying(isFlying, TimestampA) &&
                drone->IsCurrentlyExecutingWaypointMission(isDoingMission, TimestampB) &&
                (SecondsElapsed(TimestampA) <= 5.0) && (SecondsElapsed(TimestampB) <= 5.0)) {
				//We have telemetry
				auto lastState = m_droneStates.at(serial);
				if (std::get<0>(lastState) == 2) {
					//Drone was tasked with a mission but it hasn't started yet
					if (isFlying && isDoingMission) {
						std::cerr << "Updating state for drone " << serial << " due to mission start.\r\n";
						std::get<0>(m_droneStates.at(serial)) = 3;
						std::get<2>(m_droneStates.at(serial)) = std::chrono::steady_clock::now();
					}
				}
				else if (std::get<0>(lastState) == 3) {
					//Drone was flying a mission
					if ((! isFlying) || (! isDoingMission)) {
						//Drone is no longer flying the mission
						std::cerr << "Updating state for drone " << serial << " due to sub-region completion.\r\n";
						if (! isFlying)
							m_droneStates.at(serial) = std::make_tuple(0, -1, std::chrono::steady_clock::now());
						else
							m_droneStates.at(serial) = std::make_tuple(1, -1, std::chrono::steady_clock::now());
					}
				}
            }
        }
    }

    std::vector<DroneInterface::Drone *> GuidanceEngine::GetDronesAvailableForTasking(void) {
        //A drone is available if it's on the ground and it's past the allowed takeoff time, or if it's in the air but not
        //flying a mission right now (loitering)
        std::vector<DroneInterface::Drone *> availableDrones;
        TimePoint currentTime = std::chrono::steady_clock::now();
        for (DroneInterface::Drone * drone : m_dronesUnderCommand) {
            std::string serial = drone->GetDroneSerial();
            if ((std::get<0>(m_droneStates.at(serial)) == 0) && (currentTime > m_droneAllowedTakeoffTimes.at(serial)))
            	availableDrones.push_back(drone);
            else if (std::get<0>(m_droneStates.at(serial)) == 1)
            	availableDrones.push_back(drone);
        }
        return availableDrones;
    }

    //Task a drone to an available mission and update states accordingly
    void GuidanceEngine::TaskDroneToAvailableMission(DroneInterface::Drone * DroneToTask) {
    	//Stupid right now - grab any mission
    	if (! m_availableMissionIndices.empty()) {
    		//int missionIndex = *(m_availableMissionIndices.begin());
    		//m_availableMissionIndices.erase(missionIndex);
    		//TaskDroneWithMission(DroneToTask, m_droneMissions[missionIndex]);
    		//m_droneStates.at(DroneToTask->GetDroneSerial()) = std::make_tuple(1, missionIndex, std::chrono::steady_clock::now());

    		//Only consider non-trivial and viable missions and then pick the closest one
    		std::string serial = DroneToTask->GetDroneSerial();
    		Eigen::Vector3d droneLLA = m_dronePositions.at(serial);
    		DroneInterface::Waypoint currentPos;
    		currentPos.Latitude = droneLLA(0);
    		currentPos.Longitude = droneLLA(1);
    		currentPos.RelAltitude = 0.0;
    		std::vector<int> availableRegionIndices(m_availableMissionIndices.begin(), m_availableMissionIndices.end());
    		std::vector<double> travelTimes(availableRegionIndices.size(), std::nan(""));
    		for (size_t n = 0U; n < availableRegionIndices.size(); n++) {
    			int regionIndex = availableRegionIndices[n];
    			if (! m_droneMissions[regionIndex].Waypoints.empty()) {
    				DroneInterface::Waypoint WP0 = m_droneMissions[regionIndex].Waypoints[0];
    				double timeToReachRegion = EstimateMissionTime(currentPos, WP0, m_MissionParams.TargetSpeed);
    				TimePoint missionStartTime = AdvanceTimepoint(std::chrono::steady_clock::now(), timeToReachRegion);
    				double margin;
    				if (IsPredictedToFinishWithoutShadows(m_TA, m_droneMissions[regionIndex], 0.0, missionStartTime, margin))
    					travelTimes[n] = timeToReachRegion; //Record the time it will take to get to the region
    			}
    		}

    		int bestRegionIndex = -1;
    		double shortestTravelTime = std::nan("");
    		int numViableRegions = 0;
    		for (size_t n = 0U; n < availableRegionIndices.size(); n++) {
    			if (! std::isnan(travelTimes[n])) {
    				//Region is viable
    				if ((bestRegionIndex < 0) || (travelTimes[n] < shortestTravelTime)) {
    					bestRegionIndex = availableRegionIndices[n];
    					shortestTravelTime = travelTimes[n];
    					numViableRegions++;
    				}
    			}
    		}
    		std::cerr << "Num viable regions: " << numViableRegions << "\r\n";

    		if (bestRegionIndex >= 0) {
    			//There is sub-region we may be able to fly - task drone
	    		m_availableMissionIndices.erase(bestRegionIndex);
	    		TaskDroneWithMission(DroneToTask, m_droneMissions[bestRegionIndex]);
	    		m_droneStates.at(DroneToTask->GetDroneSerial()) = std::make_tuple(2, bestRegionIndex, std::chrono::steady_clock::now());
    		}
    	}
    }

    void GuidanceEngine::UpdateGuidanceOverlayWithMissionSequences(void) {
    	std::vector<std::vector<int>> Sequences;
    	for (DroneInterface::Drone * drone : m_dronesUnderCommand) {
            std::string serial = drone->GetDroneSerial();
            if (std::get<0>(m_droneStates.at(serial)) > 1)
            	Sequences.emplace_back(1, std::get<1>(m_droneStates.at(serial)));
            else
            	Sequences.emplace_back();
        }
    	MapWidget::Instance().m_guidanceOverlay.SetDroneMissionSequences(Sequences);
    }

    bool GuidanceEngine::AreAnyDronesTaskedWithOrFlyingMissions(void) {
    	for (auto const & kv : m_droneStates) {
    		if (std::get<0>(kv.second) > 1)
    			return true;
    	}
    	return false;
    }

    void GuidanceEngine::AbortMissionsPredictedToGetHitWithShadows(void) {
    	for (DroneInterface::Drone * drone : m_dronesUnderCommand) {
            std::string serial = drone->GetDroneSerial();
            if (std::get<0>(m_droneStates.at(serial)) > 1) {
            	//Drone is tasked with or flying a mission right now
            	int missionIndex = std::get<1>(m_droneStates.at(serial));
            	double timeIntoMission = SecondsElapsed(std::get<2>(m_droneStates.at(serial)));

            	//Since we don't have access to the drones internal tracking of mission progress,
            	//we estimate it crudely based on the amount of time the drone has been flying the mission
            	double currentProgressFraction = std::min(timeIntoMission/m_MissionParams.SubregionTargetFlightTime, 1.0);
            	size_t lastWaypointIndex = std::max(m_droneMissions[missionIndex].Waypoints.size(), size_t(1)) - size_t(1);
            	double currentWaypoint = currentProgressFraction * double(lastWaypointIndex);

            	double margin = 0.0;
                bool willFinish = IsPredictedToFinishWithoutShadows(m_TA, m_droneMissions[missionIndex], currentWaypoint, std::chrono::steady_clock::now(), margin);
                if (! willFinish) {
					DroneInterface::WaypointMission LoiterMission;
					LoiterMission.Waypoints.push_back(m_droneMissions[missionIndex].Waypoints[0]);
					LoiterMission.LandAtLastWaypoint = false;
					TaskDroneWithMission(drone, LoiterMission);
					m_droneStates.at(drone->GetDroneSerial()) = std::make_tuple(1, -1, std::chrono::steady_clock::now());
					m_availableMissionIndices.insert(missionIndex); //Mark the mission as available again
                }
            }
        }
    }

    void GuidanceEngine::ModuleMain(void) {
        // float refresh_period = 35.0;
        //float refresh_period = 1.0; //Check up on progress and look to re-task every this many seconds
        //auto MostRecentSequenceTimestamp = AdvanceTimepoint(std::chrono::steady_clock::now(), -1.0*refresh_period);


    	double AnalysisPeriod = 1.0; //Analyse drone tasking every this many seconds
    	TimePoint LastAnalysisTP = AdvanceTimepoint(std::chrono::steady_clock::now(), -1.0*AnalysisPeriod);
        while (! m_abort) {
            //Grab any settings that we may need before starting the loop - ensure we don't hold a lock on m_mutex while locking the options mutex
            ProgOptions::Instance()->OptionsMutex.lock();
            int partitioningMethod = ProgOptions::Instance()->SurveyRegionPartitioningMethod;
            ProgOptions::Instance()->OptionsMutex.unlock();

            m_mutex.lock();
            if (! m_running) {
                MapWidget::Instance().m_messageBoxOverlay.RemoveMessage(m_MessageToken1);
                MapWidget::Instance().m_messageBoxOverlay.RemoveMessage(m_MessageToken2);
                MapWidget::Instance().m_messageBoxOverlay.RemoveMessage(m_MessageToken3);
                m_mutex.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            //We are running - see if we have done the prep work yet. If not, do all the initial setup work that needs to be done
            MissionPrepWork(partitioningMethod);

            //Update our record of drone positions
            UpdateDronePositions();

            //Make sure we have the most recent TA function (if any are available)
            std::chrono::time_point<std::chrono::steady_clock> newTimestamp;
            if (ShadowPropagation::ShadowPropagationEngine::Instance().GetTimestampOfMostRecentTimeAvailFun(newTimestamp)) {
                //A TA function is available and we now have it's timestamp
                if (newTimestamp > m_TA.Timestamp)
                    ShadowPropagation::ShadowPropagationEngine::Instance().GetMostRecentTimeAvailFun(m_TA);
            }

            if (SecondsElapsed(LastAnalysisTP) > AnalysisPeriod) {
	            //Record the time of this analysis
	            LastAnalysisTP = std::chrono::steady_clock::now();

	            //Abort any missions that won't finish before shadows hit the region
	            AbortMissionsPredictedToGetHitWithShadows();

	            //Update drone states based on starting or ending tasked missions
	            UpdateDroneStatesBasedOnMissionProgress();

	            //If there are any drones available for tasking, task them
	            std::vector<DroneInterface::Drone *> dronesForTasking = GetDronesAvailableForTasking();
	            for (DroneInterface::Drone * drone : dronesForTasking)
	            	TaskDroneToAvailableMission(drone);

	            UpdateGuidanceOverlayWithMissionSequences();

	            //Check to see if the mission is complete
	            if ((m_availableMissionIndices.empty()) && (! AreAnyDronesTaskedWithOrFlyingMissions()))
	            	m_running = false;
        	}







            //Debug Only - update partition labels with whether or not each component would be finished before shadows if started now
            // std::vector<std::string> partitionLabels;
            // partitionLabels.reserve(m_surveyRegionPartition.size());
            // for (size_t compIndex = 0U; compIndex < m_surveyRegionPartition.size(); compIndex++) {
            //     double margin = 0.0;
            //     bool willFinish = IsPredictedToFinishWithoutShadows(m_TA, m_droneMissions[compIndex], 0.0, std::chrono::steady_clock::now(), margin);
            //     std::ostringstream out;
            //     out << (willFinish ? "Will Finish" : "Will Not Finish")
            //         << "\nMargin: " << std::fixed << std::setprecision(1) << margin;
            //     partitionLabels.push_back(out.str());
            // }
            // MapWidget::Instance().m_guidanceOverlay.SetPartitionLabels(partitionLabels);

            // std::chrono::time_point<std::chrono::steady_clock> NowTimestamp = std::chrono::steady_clock::now();
            // if (SecondsElapsed(MostRecentSequenceTimestamp, NowTimestamp) > refresh_period) {
            //         std::cout << "\nREFRESHING SEQUENCE" << std::endl;
            //         RefreshSequence();
            //         MostRecentSequenceTimestamp = std::chrono::steady_clock::now();
            // }

            // for (size_t i = 0; i < m_dronesUnderCommand.size(); i++) {
            //     bool Result;
            //     DroneInterface::Drone::TimePoint Timestamp;
            //     auto drone = m_dronesUnderCommand[i];
            //     drone->IsCurrentlyExecutingWaypointMission(Result, Timestamp);
    
            //     // if flying --> not flying transition detected
            //     // also, only retask/increment mission if not currently flying "OneWaypointMission" (see below)
            //     if ((Result != m_flyingMissionStatus[i]) && (Result == false) && m_wasPredictedToFinishWithoutShadows[i]) {
            //         int currentMissionIndex = m_subregionSequences[i][std::get<0>(m_currentDroneMissions[drone->GetDroneSerial()])++];
            //         m_missionIndicesToAssign.erase(currentMissionIndex);
            //         if ((int) m_subregionSequences[i].size() > std::get<0>(m_currentDroneMissions[drone->GetDroneSerial()])) {
            //             int missionIndex = m_subregionSequences[i][std::get<0>(m_currentDroneMissions[drone->GetDroneSerial()])];
            //             std::get<1>(m_currentDroneMissions[drone->GetDroneSerial()]) = m_droneMissions[missionIndex];
            //             DroneInterface::WaypointMission Mission = std::get<1>(m_currentDroneMissions[drone->GetDroneSerial()]);
            //             //drone->ExecuteWaypointMission(Mission);
            //             TaskDroneWithMission(drone, Mission);
            //         }
            //     }
            //     m_flyingMissionStatus[i] = Result;
            // }

            // //RefreshDronePositions();

            // std::chrono::time_point<std::chrono::steady_clock> DroneStartTime = std::chrono::steady_clock::now();
            // for (size_t i = 0; i < m_dronesUnderCommand.size(); i++) {
            //     auto drone = m_dronesUnderCommand[i];
            //     double Margin;
                              
            //     int missionIndex = std::get<0>(m_currentDroneMissions[m_dronesUnderCommand[i]->GetDroneSerial()]);
            //     if (missionIndex < (int) m_subregionSequences[i].size()) {
            //         DroneInterface::WaypointMission Mission = std::get<1>(m_currentDroneMissions[drone->GetDroneSerial()]);
            //         bool willFinish = IsPredictedToFinishWithoutShadows(m_TA, Mission, 0, DroneStartTime, Margin);
            //         if (m_wasPredictedToFinishWithoutShadows[i] && !willFinish) { // falling edge
            //             std::cout << "At drone idx " << i << ", a falling edge was detected; hovering at first waypoint";
            //             DroneInterface::WaypointMission OneWaypointMission;
            //             OneWaypointMission.Waypoints.push_back(Mission.Waypoints[0]);
            //             OneWaypointMission.LandAtLastWaypoint = false;
            //             //drone->ExecuteWaypointMission(OneWaypointMission);
            //             TaskDroneWithMission(drone, OneWaypointMission);
            //         } else if (!m_wasPredictedToFinishWithoutShadows[i] && willFinish) { // rising edge
            //             std::cout << "At drone idx " << i << ", a rising edge was detected; starting waypoint mission from beginning";
            //             //drone->ExecuteWaypointMission(Mission);
            //             TaskDroneWithMission(drone, Mission);
            //         }
            //         m_wasPredictedToFinishWithoutShadows[i] = willFinish;
            //     }

            // }

            // // Track incomplete missions for the sake of turning completed missions blue in guidance overlay.
            // std::vector<std::vector<int>> m_subregionSequencesIncomplete;
            // for (size_t i = 0; i < m_dronesUnderCommand.size(); i++) {
            //     auto drone = m_dronesUnderCommand[i];
            //     int missionNum = std::get<0>(m_currentDroneMissions[drone->GetDroneSerial()]);
            //     std::vector<int> incompleteMissions(m_subregionSequences[i].begin() + missionNum, m_subregionSequences[i].end());
            //     m_subregionSequencesIncomplete.push_back(incompleteMissions);
            // }

            // //TODO: Only give these to the guidance overlay when something changes.
            // MapWidget::Instance().m_guidanceOverlay.SetDroneMissionSequences(m_subregionSequencesIncomplete);


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


    // *********************************************************************************************************************************
    // *******************************************   Guidance Algorithm Function Definitions   *****************************************
    // *********************************************************************************************************************************
    //1 - Take two points and estimate the time (s) it would take a drone to fly from one to the other (stopped at start and end), assuming a max flight speed (m/s)
    double EstimateMissionTime(DroneInterface::Waypoint const & A, DroneInterface::Waypoint const & B, double TargetSpeed) {
        Eigen::Vector3d A_ECEF = LLA2ECEF(Eigen::Vector3d(A.Latitude, A.Longitude, 0.0)); //A - projected to ref ellipsoid
        Eigen::Vector3d B_ECEF = LLA2ECEF(Eigen::Vector3d(B.Latitude, B.Longitude, 0.0)); //B - projected to ref ellipsoid
        double horizontalDist  = (B_ECEF - A_ECEF).norm();
        double verticalDist = std::fabs(B.RelAltitude - A.RelAltitude);
        double dist = std::sqrt(horizontalDist*horizontalDist + verticalDist*verticalDist);
        return dist/TargetSpeed;
    }

    //2 - Take a waypoint mission and estimate the time (s) it will take a drone to fly it (not including take-off and landing, or movement to the region).
    //    Support both the mode where we come to a stop at each waypoint and the mode where we do not stop at waypoints (CurvedTrajectory field of Mission)
    double EstimateMissionTime(DroneInterface::WaypointMission const & Mission, double TargetSpeed) {
        double totalTime = 0.0;
        for (int waypointIndex = 0; waypointIndex + 1 < (int) Mission.Waypoints.size(); waypointIndex++)
            totalTime += EstimateMissionTime(Mission.Waypoints[waypointIndex], Mission.Waypoints[waypointIndex+1], TargetSpeed);
        return totalTime;
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

    float custom_lerp(float a, float b, float t)
    {
        return a + t * (b - a);
    }

    //Samples a TA function at a given Lat and Lon (in radians). Returns the time available at the given location. If the TA function
    //contains the sential value (indicating no shadow for the forseeable future) or if the sample point is not in bounds
    //of the TA function, we return NaN, which should be interpreted optimistically. That is, if we get NaN from this function,
    //we should assume the area is available for as long as we need.
    float TimeRemainingAtPos(ShadowPropagation::TimeAvailableFunction const & TA, float Latitude, float Longitude){
        if (TA.TimeAvailable.rows == 0 || TA.TimeAvailable.cols == 0)
            return std::nanf("");

        int height = TA.TimeAvailable.size().height;
        int width = TA.TimeAvailable.size().width;

        //Lerp Equation: Percentage = (Current - Max) / (Min - Max)
        float Lat_Fraction = (Latitude - TA.LR_LL[0]) / (TA.UR_LL[0] - TA.LR_LL[0]);
        float Lon_Fraction = (Longitude - TA.LL_LL[1]) / (TA.LR_LL[1] - TA.LL_LL[1]);

        //Convert the fractions, or normalized image coordinates (0-1) to rounded actual coordinates
        int Lat_Index = (int) std::round(float(height - 1) * Lat_Fraction);
        int Lon_Index = (int) std::round(float(width - 1) * Lon_Fraction);

        if ((Lat_Index < 0) || (Lat_Index + 1 > height) || (Lon_Index < 0) || (Lon_Index + 1 > width))
            return std::nanf("");

        //If we get here the sample point is in the bounds of the TA function
        uint16_t SampledTA = TA.TimeAvailable.at<uint16_t>(height - 1 - Lat_Index, Lon_Index);

        if (SampledTA == std::numeric_limits<uint16_t>::max())
            return std::nanf("");
        
        return (float) SampledTA;
    }

    bool IsPredictedToFinishWithoutShadows(ShadowPropagation::TimeAvailableFunction const & TA, DroneInterface::WaypointMission const & Mission,
                                           double DroneStartWaypoint, std::chrono::time_point<std::chrono::steady_clock> DroneStartTime, double & Margin) {

        double fractional_initial_position = fmod(DroneStartWaypoint, 1);
        double elapsed_time = 0.0;
        double time_increment = 1.0; //in Seconds
        Margin = std::nan("");
        for (int i = (int)DroneStartWaypoint; i < (int) Mission.Waypoints.size()-1; i++){
            Eigen::Vector3d PosA = LLA2ECEF(Eigen::Vector3d(Mission.Waypoints[i].Latitude, Mission.Waypoints[i].Longitude, 0));
            Eigen::Vector3d PosB = LLA2ECEF(Eigen::Vector3d(Mission.Waypoints[i+1].Latitude, Mission.Waypoints[i+1].Longitude, 0));
            Eigen::Vector3d PosC = PosA - PosB;

            float delta_time = PosC.norm() / Mission.Waypoints[i].Speed;
            for (float p = fractional_initial_position; p < 1.0; p += (time_increment/delta_time) ){
                //Linear Interpolation(s)
                float current_lat = custom_lerp(Mission.Waypoints[i].Latitude, Mission.Waypoints[i+1].Latitude, p);
                float current_lon = custom_lerp(Mission.Waypoints[i].Longitude, Mission.Waypoints[i+1].Longitude, p);
                float timeRemaining = TimeRemainingAtPos(TA, current_lat, current_lon);
                float currentTime = (p - fractional_initial_position) * delta_time + elapsed_time;
                if ((! std::isnan(timeRemaining)) && (std::isnan(Margin) || (Margin > timeRemaining - currentTime)))
                    Margin = timeRemaining - currentTime;
                if ((! std::isnan(Margin)) && (Margin < 0.0))
                    return false;
            }
            elapsed_time += (1 - fractional_initial_position) * delta_time;
            fractional_initial_position = 0.0;
        }
        return true;
    }

    //6 - Given a Time Available function, a collection of sub-regions (with their pre-planned missions), and a start position for a drone, select a sub-region
    //    to task the drone to. We are balancing 2 things here - trying to do useful work while avoiding shadows, but also avoiding non-sensical jumping to a
    //    distant region. At a minimum we should ensure that we don't task a drone to a region that we don't expect it to be able to finish without getting hit
    //    with shadows.
    //Arguments:
    //TA                - Input - Time Available function
    //SubregionMissions - Input - A vector of drone Missions - Element n is the mission for sub-region n.
    //StartPos          - Input - The starting position of the drone (to tell us how far away from each sub-region mission it is)
    //MissionParams     - Input - Parameters specifying speed and row spacing (see definitions in struct declaration)
    //
    //Returns: The index of the drone mission (and sub-region) to task the drone to. Returns -1 if none are plausable
    int SelectSubRegion(ShadowPropagation::TimeAvailableFunction const & TA, std::vector<DroneInterface::WaypointMission> const & SubregionMissions,
                        DroneInterface::Waypoint const & StartPos, MissionParameters const & MissionParams) {
        //TODO
        return -1;
    }

    // Extends v_dest with v_src
    void extend(std::vector<int> & v_dest, const std::vector<int> & v_src) {
        v_dest.reserve(v_dest.size() + distance(v_src.begin(), v_src.end()));
        v_dest.insert(v_dest.end(), v_src.begin(), v_src.end());
    }

    // GenerateCombos helper function
    void GenerateCombosHelper(std::vector<std::vector<int>> & combos, std::vector<int> & combo, const std::vector<int> & elements, int left, int k) {
        if (k == 0) {
            combos.push_back(combo);
            return;
        }

        // first time left will be zero
        for (size_t i = left; i < elements.size(); i++) {
            combo.push_back(elements[i]);
            GenerateCombosHelper(combos, combo, elements, i + 1, k - 1);
            combo.pop_back();
        }
    }

    // Recursively generates all combinations of elements of length k
    std::vector<std::vector<int>> GenerateCombos(const std::vector<int> & elements, int k) {
        std::vector<std::vector<int>> combos;
        std::vector<int> combo;
        GenerateCombosHelper(combos, combo, elements, 0, k);
        return combos;
    }

    // Recursively generates all permutations of elements of length elements.size()
    std::vector<std::vector<int>> GeneratePerms(std::vector<int> & elements) {
        sort(elements.begin(), elements.end());
        std::vector<std::vector<int>> perms;
        do {
            perms.push_back(elements);
        } while(std::next_permutation(elements.begin(), elements.end()));
        return perms;
    }

    // Input: Assignable missions and number of drones i.e., <1, 3, 7, 0>, 3
    // Output: Every unordered way to assign missions to drones i.e., [3, 1] [7] [0]
    void RecurseAssignments(std::vector<std::vector<std::vector<int>>> & AllAssignments, std::vector<std::vector<int>> & CurrentAssignments, const std::vector<int> & AssignableMissions, const int MissionIndex, const int NumDrones) {
        if (MissionIndex == (int) AssignableMissions.size()) {
            AllAssignments.push_back(CurrentAssignments);
        } else {
            for (int drone_idx = 0; drone_idx < NumDrones; drone_idx++) {
                CurrentAssignments[drone_idx].push_back(AssignableMissions[MissionIndex]);
                RecurseAssignments(AllAssignments, CurrentAssignments, AssignableMissions, MissionIndex + 1, NumDrones);
                CurrentAssignments[drone_idx].pop_back();
            }
        }
    }

    // Input: Unordered way to assign missions to drones i.e., [3, 1] [7] [0]
    // Output: All ordered ways to assign missions to drones i.e., [3, 1] [7] [0], [1, 3] [7] [0]
    void RecurseSequences(std::vector<std::vector<std::vector<int>>> & AllSequences, std::vector<std::vector<int>> & CurrentSequences, int DroneIndex, const int NumDrones) {
        if (DroneIndex == NumDrones) {
            AllSequences.push_back(CurrentSequences);
        } else {
            std::vector<std::vector<int>> perms = GeneratePerms(CurrentSequences[DroneIndex]);
            for (std::vector<int> perm : perms) {
                CurrentSequences[DroneIndex] = perm;
                RecurseSequences(AllSequences, CurrentSequences, DroneIndex + 1, NumDrones);
            }
        }
    }

    // Return the number of subregions that can be completely imaged given TA
    int GetCoverage(std::vector<int> & PredictedCoverage, std::vector<int> & MissionDurations, ShadowPropagation::TimeAvailableFunction const & TA, std::vector<DroneInterface::WaypointMission> const & SubregionMissions, std::vector<std::vector<int>> const & Sequences, std::vector<DroneInterface::Waypoint> const & StartPositions, MissionParameters const & MissionParams, const int NumDrones) {
        MissionDurations.clear();
        PredictedCoverage.clear();
        int coverage = 0;

        // Initialize start time
        std::chrono::time_point<std::chrono::steady_clock> NowTime = std::chrono::steady_clock::now(), DroneStartTime;
        std::vector<double> time_elapsed(NumDrones, 0);
        double Margin, processingTime;

        for (int drone_idx = 0; drone_idx < NumDrones; drone_idx++) {
            DroneStartTime = NowTime;
            DroneInterface::Waypoint DronePos = StartPositions[drone_idx];
            PredictedCoverage.push_back(0);
            for (size_t destWaypointIndex = 0; destWaypointIndex < Sequences[drone_idx].size(); destWaypointIndex++) {
                DroneInterface::WaypointMission currentMission = SubregionMissions[Sequences[drone_idx][destWaypointIndex]];
                
                //If the mission is trivial, skip it - don't count it as being covered either
                if (! currentMission.Waypoints.empty()) {
                    DroneStartTime += std::chrono::seconds((int) EstimateMissionTime(DronePos, currentMission.Waypoints[0], MissionParams.TargetSpeed));
                    if (IsPredictedToFinishWithoutShadows(TA, currentMission, 0, DroneStartTime, Margin)) {
                        coverage++;
                        if (destWaypointIndex == 0) { // PredictedCoverage tracks the predicted shadowed-ness of the imminent mission, which is index 0.
                            PredictedCoverage[drone_idx]++;
                        }
                    }
                    // Add time to fly mission
                    DroneStartTime += std::chrono::seconds((int) EstimateMissionTime(currentMission, MissionParams.TargetSpeed));
                    DronePos = currentMission.Waypoints.back();
                }
            }
            MissionDurations.push_back(SecondsElapsed(NowTime, DroneStartTime));
        }

        return coverage; 
    }

    //7 - Given a Time Available function, a collection of sub-regions (with their pre-planned missions), and a collection of drone start positions, choose
    //    sequences (of a given length) of sub-regions for each drone to fly, in order. When the mission time exceeds our prediction horizon the time available
    //    function is no longer useful in chosing sub-regions but they can still be chosen in a logical fashion that avoids leaving holes in the map... making the
    //    optimistic assumption that they will be shadow-free when we get there.
    //TA                  - Input  - Time Available function
    //SubregionMissions   - Input  - A vector of drone Missions - Element n is the mission for sub-region n.
    //DroneStartPositions - Input  - Element k is the starting position of drone k
    //Sequences           - Output - Element k is a vector of sub-region indices to task drone k to (in order)
    void SelectSubregionSequences(ShadowPropagation::TimeAvailableFunction const & TA, std::vector<DroneInterface::WaypointMission> const & SubregionMissions,
                                 std::vector<DroneInterface::Waypoint> const & DroneStartPositions, std::set<int> MissionIndicesToAssign, std::vector<std::vector<int>> & Sequences,
                                 MissionParameters const & MissionParams) {
        
        int numDrones = DroneStartPositions.size();
        int numMissions = SubregionMissions.size();

        // Naive
        // Sequences.resize(numDrones, std::vector<int>());

        // if (numDrones > numMissions) {
        // } else if (numDrones <= numMissions) {
        //     for (int i = 0; i < numMissions; i++) {
        //         Sequences[i % numDrones].push_back(i);
        //     }
        // }

        double processingTime;


        // TO CHANGE: MAKE n = 1 CONFIGURABLE RATHER THAN HARDCODED;
        const int N = 1;

        // Depth N Best
        int num_to_assign, max_coverage = 0, max_start_unclouded = 0;

        std::vector<std::vector<std::vector<int>>> AllAssignments, AllSequences;
        std::vector<std::vector<int>> CurrentAssignments, CurrentSequences;
        Sequences.resize(numDrones, std::vector<int>());

        std::vector<int> BestPredictedCoverage;

        // Keep looping until all missions have been assigned
        while(!MissionIndicesToAssign.empty()) {
            num_to_assign = (int) MissionIndicesToAssign.size() < numDrones * N ? MissionIndicesToAssign.size() : numDrones * N;

            AllAssignments.clear();
            AllSequences.clear();

            CurrentAssignments.clear();
            CurrentAssignments.resize(numDrones, std::vector<int>());
            CurrentSequences.clear();
            CurrentSequences.resize(numDrones, std::vector<int>());


            std::vector<int> MissionIndices(MissionIndicesToAssign.begin(), MissionIndicesToAssign.end());

            std::chrono::time_point<std::chrono::steady_clock> T0 = std::chrono::steady_clock::now();

            // Iterate through all ways to choose num_to_assign missions and ways to assign those to drones
            std::vector<std::vector<int>> combos = GenerateCombos(MissionIndices, num_to_assign);

            processingTime = SecondsElapsed(T0, std::chrono::steady_clock::now());
			// std::cerr << "GenerateCombos: " << processingTime*1000.0 << " ms" << std::endl;
            T0 = std::chrono::steady_clock::now();
            for (std::vector<int> combo : combos) {
                for (int drone_idx = 0; drone_idx < numDrones; drone_idx++) {
                    CurrentAssignments[drone_idx].push_back(combo[0]);
                    RecurseAssignments(AllAssignments, CurrentAssignments, combo, 1, numDrones);
                    CurrentAssignments[drone_idx].pop_back();
                }
            }

            processingTime = SecondsElapsed(T0, std::chrono::steady_clock::now());
            T0 = std::chrono::steady_clock::now();

            // Generate all ordered ways to fly drone through assignments
            for (std::vector<std::vector<int>> assignment : AllAssignments) {
                RecurseSequences(AllSequences, assignment, 0, numDrones);
            }

            processingTime = SecondsElapsed(T0, std::chrono::steady_clock::now());
            T0 = std::chrono::steady_clock::now();
            int mission_time = -1, min_max_mission_time = -1;
            bool reset_mission_time = true;
            
            // Iterate through all ordered sequences to find the best one as measured by best coverage and then by minimum maximum fly time across all drones
            std::vector<std::vector<int>> bestSequence = AllSequences[0], newSequence;
            for (std::vector<std::vector<int>> sequence : AllSequences) {
                
                // Extend current sequence with potential sequence
                newSequence = Sequences;
                for (int i = 0; i < numDrones; i++) {
                    extend(newSequence[i], sequence[i]);
                }

                // Find coverage and maximum mission duration of potential sequence
                std::vector<int> PredictedCoverage, MissionDurations;
                int coverage = GetCoverage(PredictedCoverage, MissionDurations, TA, SubregionMissions, newSequence, DroneStartPositions, MissionParams, numDrones);
                int start_unclouded = std::accumulate(PredictedCoverage.begin(), PredictedCoverage.end(), 0);
                mission_time = *max_element(MissionDurations.begin(), MissionDurations.end());

                // Update best coverage and min_max_distance
                if ((coverage > max_coverage) ||
                    ((coverage == max_coverage) && (start_unclouded > max_start_unclouded)) || 
                    (((coverage == max_coverage) && (start_unclouded == max_start_unclouded)) && (reset_mission_time || mission_time < min_max_mission_time))) {
                    // std::cout << max_coverage << std::endl;
                    BestPredictedCoverage = PredictedCoverage;
                    max_coverage = coverage;
                    max_start_unclouded = start_unclouded;
                    bestSequence = sequence;
                    min_max_mission_time = mission_time;
                    reset_mission_time = false;
                }
            }

            processingTime = SecondsElapsed(T0, std::chrono::steady_clock::now());
            T0 = std::chrono::steady_clock::now();

            // Remove missions that were assigned
            for (size_t i = 0; i < bestSequence.size(); i++) {
                extend(Sequences[i], bestSequence[i]);
                for (size_t j = 0; j < bestSequence[i].size(); j++)
                    MissionIndicesToAssign.erase(bestSequence[i][j]);
            }
        }
        std::cout << "BestPredictedCoverage: " << std::endl;
        for (int i = 0; i < (int) BestPredictedCoverage.size(); i++) {
            std::cout << BestPredictedCoverage[i] << " ";
        }
        std::cout << std::endl;
    }
}
