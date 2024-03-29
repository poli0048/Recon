//This module provides the main interface for the guidance system
//Authors: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.

//System Includes

//External Includes

//Project Includes
#include "Guidance.hpp"
#include "../../UI/VehicleControlWidget.hpp"
#include "../../Utilities.hpp"
#include "../../Maps/MapUtils.hpp"

#define PI 3.14159265358979323846

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

	//Take control of a drone and add it to the current mission
	bool GuidanceEngine::AddDroneToMission(std::string const & Serial) {
		std::scoped_lock lock(m_mutex);
		if (! m_running)
			return false;
		else {
			DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(Serial);
			if (drone != nullptr) {
				VehicleControlWidget::Instance().StopCommandingDrone(Serial);

				m_dronesUnderCommand.push_back(drone);
				TimePoint NowTime = std::chrono::steady_clock::now();

				//Go through the allowed takeoff times - find the last takeoff time and advance it by the stagger time for the added drone
				TimePoint lastTimepoint = NowTime;
				double maxSecondsSinceT0Epoch = 0.0;
				for (auto const & kv : m_droneAllowedTakeoffTimes) {
					double secondsAfterT0 = SecondsSinceT0Epoch(kv.second);
					if (secondsAfterT0 > maxSecondsSinceT0Epoch) {
						maxSecondsSinceT0Epoch = secondsAfterT0;
						lastTimepoint = kv.second;
					}
				}
				if (maxSecondsSinceT0Epoch == 0.0)
					m_droneAllowedTakeoffTimes[Serial] = std::chrono::steady_clock::now(); //There are no other drones
				else
					m_droneAllowedTakeoffTimes[Serial] = AdvanceTimepoint(lastTimepoint, m_MissionParams.TakeoffStaggerInterval);
				
				//Go through the HAGs and find an available slot for the new drone
				if (m_droneHAGs.empty())
					m_droneHAGs[Serial] = m_MissionParams.HAG;
				else {
					if (m_MissionParams.HeightStaggerInterval <= 0.0)
						m_droneHAGs[Serial] = m_MissionParams.HAG;
					else {
						//Look at the drones currently assigned to the mission and get their HAGs. Identify the value X that is closest
						//to the target HAG but at least the staggering amount away from all existing drone HAGs.

						//Find the lowest HAG
						double lowestHAG = m_droneHAGs.begin()->second;
						for (auto const & kv : m_droneHAGs) {
							if (kv.second < lowestHAG)
								lowestHAG = kv.second;
						}

						std::vector<double> currentHAGSlots; //HAGs, as multiples of the stagger interval above the lowest HAG
						currentHAGSlots.reserve(m_droneHAGs.size());
						for (auto const & kv : m_droneHAGs)
							currentHAGSlots.push_back((kv.second - lowestHAG) / m_MissionParams.HeightStaggerInterval);
						std::sort(currentHAGSlots.begin(), currentHAGSlots.end());
						double maxSlot = currentHAGSlots.back();

						//std::cerr << "Current HAG slots: ";
						//for (double HAGSlot : currentHAGSlots)
						//	std::cerr << HAGSlot << " ";
						//std::cerr << "\r\n";

						//The slots should basically be integers, starting with 0. If there is an interior integer missing we can use the slot
						double interiorSlot = -1.0;
						for (double slot = 1.0; slot <= maxSlot; slot += 1.0) {
							bool slotOK = true;
							for (double currentSlot : currentHAGSlots) {
								if (std::fabs(slot - currentSlot) < 0.9) {
									slotOK = false;
									break;
								}
							}
							if (slotOK) {
								interiorSlot = slot;
								break;
							}
						}

						if (interiorSlot >= 0.0) {
							//HAG slot found
							m_droneHAGs[Serial] = lowestHAG + m_MissionParams.HeightStaggerInterval*interiorSlot;
							std::cerr << "Using existing HAG slot for added drone: " << m_droneHAGs.at(Serial) << " m.\r\n";
						}
						else {
							//No interior HAG slots found - add a slot on either the high or low end
							//Take whichever is closest to the target HAG.
							double candidateHAG1 = lowestHAG - m_MissionParams.HeightStaggerInterval;
							double candidateHAG2 = lowestHAG + (maxSlot + 1.0)*m_MissionParams.HeightStaggerInterval;
							if (std::fabs(candidateHAG1 - m_MissionParams.HAG) < std::fabs(candidateHAG2 - m_MissionParams.HAG))
								m_droneHAGs[Serial] = candidateHAG1;
							else
								m_droneHAGs[Serial] = candidateHAG2;
							std::cerr << "Using new HAG slot for added drone: " << m_droneHAGs.at(Serial) << " m.\r\n";
						}
					}
				}

				//Initialize the state of the drone
				bool isFlying = false;
				TimePoint isFlyingTimestamp;
				if (drone->IsCurrentlyFlying(isFlying, isFlyingTimestamp) && (SecondsElapsed(isFlyingTimestamp) <= 4.0)) {
					if (isFlying)
						m_droneStates[Serial] = std::make_tuple(1, -1, NowTime); //Loitering (available for tasking anyhow)
					else
						m_droneStates[Serial] = std::make_tuple(0, -1, NowTime); //On ground and available for tasking
				}
				else {
					std::cerr << "Bad or old telemetry for drone " << Serial << ". Treating as on ground.\r\n";
					m_droneStates[Serial] = std::make_tuple(0, -1, NowTime); //On ground and available for tasking
				}
				
				return true;
			}
			else
				return false;
		}
	}

	//Stop commanding the drone with the given serial
	bool GuidanceEngine::RemoveDroneFromMission(std::string const & Serial) {
		std::scoped_lock lock(m_mutex);
		if (! m_running)
			return false;
		for (size_t n = 0U; n < m_dronesUnderCommand.size(); n++) {
			if (m_dronesUnderCommand[n]->GetDroneSerial() == Serial) {
				//If this drone was doing something useful, update things accordingly
				if (m_droneStates.count(Serial) > 0U) {
					if (std::get<0>(m_droneStates.at(Serial)) > 1) {
						//The drone was tasked with or flying a mission
						int missionIndex = std::get<1>(m_droneStates.at(Serial));
						m_availableMissionIndices.insert(missionIndex);
						m_taskedMissionDistances.erase(missionIndex); //Clear mission progress data
						m_taskedMissionProgress.erase(missionIndex);  //Clear mission progress data
					}
					m_droneStates.erase(Serial);
				}

				//Remove the drone from our vector of drone pointers under our command
				m_dronesUnderCommand.erase(m_dronesUnderCommand.begin() + n);
				m_droneHAGs.erase(Serial);
				m_droneAllowedTakeoffTimes.erase(Serial);
				std::cerr << "Removing drone " << Serial << " from command.\r\n";

				//If we just removed the last drone, abort the mission - this makes it unnecessary to have an extra UI control
				//to cancel a mission that has effectively already been canceled through the removal of all drones.
				if (m_dronesUnderCommand.empty()) {
					m_running = false;
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

	//Clear mission prep data and periodically updated fields
	void GuidanceEngine::ResetIntermediateData(void) {
		m_surveyRegionPartition.clear();
		m_droneMissions.clear();
		m_droneAllowedTakeoffTimes.clear();
		m_droneHAGs.clear();
		m_droneStates.clear();
		m_taskedMissionDistances.clear();
		m_taskedMissionProgress.clear();
		m_dronePositions.clear();
		m_availableMissionIndices.clear();
	}

	//Update our latched drone positions - also update progress fields for drones tasked to missions
	void GuidanceEngine::UpdateDronePositionsAndMissionProgress(void) {
		for (DroneInterface::Drone * drone : m_dronesUnderCommand) {
			std::string serial = drone->GetDroneSerial();
			double Lat = 0.0;
			double Lon = 0.0;
			double Alt = 0.0;
			TimePoint Timestamp;
			if (drone->GetPosition(Lat, Lon, Alt, Timestamp)) {
				if ((m_droneStates.count(serial) > 0U) && (std::get<0>(m_droneStates.at(serial)) > 1)) {
					//This drone is tasked to a mission - if we are tracking the drone position update mission progress
					int missionIndex = std::get<1>(m_droneStates.at(serial));
					if (m_dronePositions.count(serial) > 0U) {
						Eigen::Vector3d oldPos_LLA  = m_dronePositions.at(serial);
						Eigen::Vector3d newPos_LLA  = Eigen::Vector3d(Lat, Lon, Alt);
						Eigen::Vector3d oldPos_ECEF = LLA2ECEF(Eigen::Vector3d(oldPos_LLA(0), oldPos_LLA(1), 0.0)); //Project to ref ellipsoid
						Eigen::Vector3d newPos_ECEF = LLA2ECEF(Eigen::Vector3d(newPos_LLA(0), newPos_LLA(1), 0.0)); //Project to ref ellipsoid
						m_taskedMissionProgress[missionIndex] += (newPos_ECEF - oldPos_ECEF).norm();
					}
				}
				m_dronePositions[serial] = Eigen::Vector3d(Lat, Lon, Alt);
			}
			else
				m_dronePositions[serial] = Eigen::Vector3d(0.0, 0.0, 0.0); //Not great, but this is what the drone defaults to anyways
		}
	}

	//Do mission prep work - will lock m_mutex internally when accessing protected fields
	void GuidanceEngine::MissionPrepWork(int PartitioningMethod) {
		//Make a local copy of the data necessary to do mission preparation
		m_mutex.lock();
		std::vector<DroneInterface::Drone *> dronesUnderCommand = m_dronesUnderCommand;
		PolygonCollection surveyRegion  = m_surveyRegion;
		MissionParameters missionParams = m_MissionParams;
		m_mutex.unlock();

		std::cerr << "Doing mission preparation work.\r\n\r\n";
		//MapWidget::Instance().m_messageBoxOverlay.AddMessage("Preparing mission for execution..."s, m_MessageToken1);

		//Partition the survey region into components
		std::Evector<PolygonCollection> surveyRegionPartition;
		if (PartitioningMethod == 0)
			PartitionSurveyRegion_TriangleFusion(surveyRegion, surveyRegionPartition, missionParams);
		else if (PartitioningMethod == 1)
			PartitionSurveyRegion_IteratedCuts(surveyRegion, surveyRegionPartition, missionParams);
		else
			std::cerr << "Error: Unrecognized partitioning method. Skipping partitioning.\r\n";
		std::vector<std::string> compLabels(surveyRegionPartition.size());
		for (size_t n = 0U; n < surveyRegionPartition.size(); n++) {
			Eigen::Vector4d AABB = surveyRegionPartition[n].GetAABB();
			double mPerNMUnit = NMUnitsToMeters(1.0, 0.5*AABB(2) + 0.5*AABB(3));
			double sqMPerNMAreaUnit = mPerNMUnit * mPerNMUnit;
			double area_sqM = sqMPerNMAreaUnit * surveyRegionPartition[n].GetArea();
			double area_acres = area_sqM / 4046.86;
			std::ostringstream outSS;
			outSS << "Sub-Region " << n << "\n" << std::fixed << std::setprecision(1) << area_acres << " acres";
			compLabels[n] = outSS.str();
		}
		MapWidget::Instance().m_guidanceOverlay.SetData_SurveyRegionPartition(surveyRegionPartition, compLabels);

		//Plan missions for each component of the partition - we are guaranteed a mission object for each sub-region, but
		//the missions may be empty if the region is too pathological. These should be treated as complete right off the bat.
		std::vector<DroneInterface::WaypointMission> droneMissions;
		droneMissions.reserve(surveyRegionPartition.size());
		for (auto const & component : surveyRegionPartition) {
			droneMissions.emplace_back();
			PlanMission(component, droneMissions.back(), missionParams, nullptr);
		}
		MapWidget::Instance().m_guidanceOverlay.SetData_PlannedMissions(droneMissions);

		//Set initial drone states
		std::unordered_map<std::string, std::tuple<int, int, TimePoint>> droneStates;
		TimePoint NowTime = std::chrono::steady_clock::now();
		for (DroneInterface::Drone * drone : dronesUnderCommand) {
			std::string serial = drone->GetDroneSerial();
			bool isFlying = false;
			TimePoint isFlyingTimestamp;
			if (drone->IsCurrentlyFlying(isFlying, isFlyingTimestamp) && (SecondsElapsed(isFlyingTimestamp) <= 4.0)) {
				if (isFlying)
					droneStates[serial] = std::make_tuple(1, -1, NowTime); //Loitering (available for tasking anyhow)
				else
					droneStates[serial] = std::make_tuple(0, -1, NowTime); //On ground and available for tasking
			}
			else {
				std::cerr << "Bad or old telemetry for drone " << serial << ". Treating as on ground.\r\n";
				droneStates[serial] = std::make_tuple(0, -1, NowTime); //On ground and available for tasking
			}
		}

		//Populate available mission indices
		std::unordered_set<int> availableMissionIndices;
		for (int missionIndex = 0; missionIndex < (int) droneMissions.size(); missionIndex++) {
			//Mark all non-trivial missions as available
			if (! droneMissions[missionIndex].Waypoints.empty())
				availableMissionIndices.insert(missionIndex);
		}

		//Populate the allowed takeoff times for all drones (use now for drones in the air) and populate
		//the HAGs to use (m) per drone. These override mission HAG since we want height to be fixed per drone and not per mission
		std::Eunordered_map<std::string, TimePoint> droneAllowedTakeoffTimes;
		std::Eunordered_map<std::string, double> droneHAGs;
		TimePoint nextAllowedTakeoffTime = std::chrono::steady_clock::now();
		double nextAllowedHAG = missionParams.HAG;
		if (dronesUnderCommand.size() > 1U)
			nextAllowedHAG += 0.5*(missionParams.HeightStaggerInterval*double(dronesUnderCommand.size() - 1U));
		for (DroneInterface::Drone * drone : dronesUnderCommand) {
			std::string serial = drone->GetDroneSerial();
			bool isFlying = false;
			TimePoint isFlyingTimestamp;
			if (drone->IsCurrentlyFlying(isFlying, isFlyingTimestamp) && (SecondsElapsed(isFlyingTimestamp) <= 4.0)) {
				//Nothing unusual here - telemetry is recent
				if (isFlying)
					droneAllowedTakeoffTimes[serial] = std::chrono::steady_clock::now();
				else {
					droneAllowedTakeoffTimes[serial] = nextAllowedTakeoffTime;
					nextAllowedTakeoffTime = AdvanceTimepoint(nextAllowedTakeoffTime, missionParams.TakeoffStaggerInterval);
				}
			}
			else {
				std::cerr << "Bad or old telemetry for drone " << serial << ". Treating as on ground.\r\n";
				droneAllowedTakeoffTimes[serial] = nextAllowedTakeoffTime;
				nextAllowedTakeoffTime = AdvanceTimepoint(nextAllowedTakeoffTime, missionParams.TakeoffStaggerInterval);
			}
			droneHAGs[serial] = nextAllowedHAG;
			nextAllowedHAG -= missionParams.HeightStaggerInterval;
		}

		//MapWidget::Instance().m_messageBoxOverlay.RemoveMessage(m_MessageToken1);

		//Save all the mission prep work and reset progress tracking fields
		m_mutex.lock();
		m_surveyRegionPartition = surveyRegionPartition;
		m_droneMissions = droneMissions;
		m_droneStates = droneStates;
		m_availableMissionIndices = availableMissionIndices;
		m_droneAllowedTakeoffTimes = droneAllowedTakeoffTimes;
		m_droneHAGs = droneHAGs;
		m_taskedMissionDistances.clear();
		m_taskedMissionProgress.clear();
		m_dronePositions.clear();
		m_missionPrepDone = true; //Mark the prep work as done
		m_mutex.unlock();
	}

	//Assign the given drone to fly the given mission. This will override the HAG for the mission based on the drones serial number
	//and the pre-computed, per-drone HAGs generated in the mission planning phase (if we computed a HAG for this drone). This will
	//not update m_droneStates since it does not know the purpose of the provided mission
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
						m_taskedMissionDistances.erase(std::get<1>(lastState));
						m_taskedMissionProgress.erase(std::get<1>(lastState));
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
		if (! m_availableMissionIndices.empty()) {
			std::string serial = DroneToTask->GetDroneSerial();
			Eigen::Vector3d droneLLA = m_dronePositions.at(serial);
			DroneInterface::Waypoint currentPos;
			currentPos.Latitude = droneLLA(0);
			currentPos.Longitude = droneLLA(1);
			currentPos.RelAltitude = 0.0;

			int missionIndex = SelectSubRegion(m_TA, m_surveyRegionPartition, m_droneMissions, m_availableMissionIndices, currentPos, m_MissionParams);
			if (missionIndex >= 0) {
				//There is sub-region we may be able to fly

				//Re-optimize the mission for this sub-region based on the drones current (starting) position
				//std::cerr << "Mission for sub-region " << missionIndex << " being re-optimized.\r\n";
				PlanMission(m_surveyRegionPartition[missionIndex], m_droneMissions[missionIndex], m_MissionParams, &currentPos);
				MapWidget::Instance().m_guidanceOverlay.SetData_PlannedMissions(m_droneMissions);

				//Task drone to the updated mission
				m_availableMissionIndices.erase(missionIndex);
				TaskDroneWithMission(DroneToTask, m_droneMissions[missionIndex]);
				m_droneStates.at(DroneToTask->GetDroneSerial()) = std::make_tuple(2, missionIndex, std::chrono::steady_clock::now());

				//Compute the total travel distance for the mission we are tasking and initialize progress fields
				m_taskedMissionDistances[missionIndex] = m_droneMissions[missionIndex].TotalMissionDistance2D(&currentPos);
				m_taskedMissionProgress[missionIndex]  = 0.0;
			}
		}
	}

	void GuidanceEngine::UpdateGuidanceOverlayWithMissionSequencesAndProgress(void) {
		std::vector<std::vector<int>> Sequences;
		std::unordered_set<int> missionsInProgress;
		for (DroneInterface::Drone * drone : m_dronesUnderCommand) {
			std::string serial = drone->GetDroneSerial();
			if (std::get<0>(m_droneStates.at(serial)) > 1) {
				int missionIndex = std::get<1>(m_droneStates.at(serial));
				Sequences.emplace_back(1, missionIndex);
				missionsInProgress.insert(missionIndex);
			}
			else
				Sequences.emplace_back();
		}
		MapWidget::Instance().m_guidanceOverlay.SetData_DroneMissionSequences(Sequences);

		std::vector<int> completedSubregionIndices;
		completedSubregionIndices.reserve(m_surveyRegionPartition.size());
		for (int n = 0; n < (int) m_surveyRegionPartition.size(); n++) {
			if ((m_availableMissionIndices.count(n) == 0U) && (missionsInProgress.count(n) == 0U))
				completedSubregionIndices.push_back(n);
		}
		MapWidget::Instance().m_guidanceOverlay.SetData_CompletedSubRegions(completedSubregionIndices);
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
				
				//Estimate mission progress so we can set a reasonable starting waypoint for the mission simulation
				//Use time into the mission and the target region flight time for a crude estimate
				//double timeIntoMission = SecondsElapsed(std::get<2>(m_droneStates.at(serial)));
				//double currentProgressFraction = std::min(timeIntoMission/m_MissionParams.SubregionTargetFlightTime, 1.0);
				//size_t lastWaypointIndex = std::max(m_droneMissions[missionIndex].Waypoints.size(), size_t(1)) - size_t(1);
				//double currentWaypoint = currentProgressFraction * double(lastWaypointIndex);
				
				//Use the total horizontal distance traveled and the total mission horizontal distance for a progress estimate
				//that is more resilient to flight time estimation error and accounts for travel to the first waypoint. As far
				//as heuristics go, this is pretty damn good, and much better than using flight time. However, it would still be
				//much better to get feedback from the drone about mission progress - if we find a way to get that, this should
				//be updated to use the drones actual target waypoint.
				double currentWaypoint = 0.0;
				if ((m_taskedMissionDistances.count(missionIndex) == 0U) || (m_taskedMissionProgress.count(missionIndex) == 0U))
					std::cerr << "No mission progress data - using waypoint 0 for evaluating current mission.\r\n";
				else {
					double totalDistance = m_taskedMissionDistances.at(missionIndex); //meters
					double distTraveled  = m_taskedMissionProgress.at(missionIndex); //meters
					double progressFraction  = std::clamp(distTraveled/totalDistance, 0.0, 1.0);
					size_t lastWaypointIndex = std::max(m_droneMissions[missionIndex].Waypoints.size(), size_t(1)) - size_t(1);
					currentWaypoint = progressFraction * double(lastWaypointIndex);
					//std::cerr << "Mission Progress: " << 100.0*progressFraction << " %\r\n"s;
				}

				//std::cerr << "currentWaypoint: " << currentWaypoint << "\r\n";

				double margin = 0.0;
				bool willFinish = IsPredictedToFinishWithoutShadows(m_TA, m_droneMissions[missionIndex], currentWaypoint, std::chrono::steady_clock::now(), margin);
				if (! willFinish) {
					DroneInterface::WaypointMission LoiterMission;
					LoiterMission.Waypoints.push_back(m_droneMissions[missionIndex].Waypoints[0]);
					LoiterMission.LandAtLastWaypoint = false;
					TaskDroneWithMission(drone, LoiterMission);
					m_droneStates.at(drone->GetDroneSerial()) = std::make_tuple(1, -1, std::chrono::steady_clock::now());
					m_availableMissionIndices.insert(missionIndex); //Mark the mission as available again
					m_taskedMissionDistances.erase(missionIndex); //Clear progress data for mission
					m_taskedMissionProgress.erase(missionIndex);  //Clear progress data for mission
				}
			}
		}
	}

	//Note: Carefully evaluate risk of deadlocking due to mutex locking. The guidance module holds it's lock for way
	//longer than I would like and it holds the lock while it calls methods in other modules that probably also lock
	//mutexes. Try to shorten the lock holds and avoid lock nests.

	void GuidanceEngine::ModuleMain(void) {
		double AnalysisPeriod = 1.0; //Analyze drone tasking every this many seconds
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
			if (! m_missionPrepDone) {
				m_mutex.unlock();
				MissionPrepWork(partitioningMethod); //May take non-trivial amount of time - locks m_mutex internally when needed
				m_mutex.lock();
			}

			//Update our record of drone positions and update progress of current missions
			UpdateDronePositionsAndMissionProgress();

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

				UpdateGuidanceOverlayWithMissionSequencesAndProgress();

				//Check to see if the mission is complete
				if ((m_availableMissionIndices.empty()) && (! AreAnyDronesTaskedWithOrFlyingMissions()))
					m_running = false;
			}

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

	//Samples a TA function at a given Lat and Lon (in radians). Returns the time available at the given location. If the TA function
	//contains the sential value (indicating no shadow for the forseeable future) or if the sample point is not in bounds
	//of the TA function, we return NaN, which should be interpreted optimistically. That is, if we get NaN from this function,
	//we should assume the area is available for as long as we need.
	static float TimeRemainingAtPos(ShadowPropagation::TimeAvailableFunction const & TA, float Latitude, float Longitude){
		if (TA.TimeAvailable.rows == 0 || TA.TimeAvailable.cols == 0)
			return std::nanf("");

		int height = TA.TimeAvailable.size().height;
		int width = TA.TimeAvailable.size().width;

		float Lat_Fraction = (Latitude  - TA.LR_LL[0]) / (TA.UR_LL[0] - TA.LR_LL[0]);
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
		//We simulate a mission from the given starting waypoint and from the given starting time. As the drone flies the mission we check to
		//see if it ever ends up hitting a point in the mission after shadows are expected to hit. If the simulated mission completes without
		//hitting shadows (according to the given TA map) we return true. If it doesn't we return false. When true, we also populate Margin,
		//which is the closest the drone comes to seeing a shadow (measured in seconds between drone and shadow over the same point). For
		//instance, if the margin is 5.0, it means the drone flies over some point in the mission just 5 seconds before that same point is hit
		//by shadows and all other points in the mission are flown at least 5 seconds before being shadowed. If the Margin is NaN, the mission
		//is clear of shadows for the full prediction time interval.
		double deltaT = 1.0; //seconds

		Margin = std::nan(""); //Initialize Margin

		//If the mission is trivial, return true (edge case, but handle as gracefully as possible)
		if (Mission.Waypoints.empty())
			return true;

		//Use the TA timestamp as our time-0 reference epoch. Convert start time to seconds after ref epoch.
		//Track position using a floating-point version of the waypoint index
		double currentTime = SecondsElapsed(TA.Timestamp, DroneStartTime);
		double currentPos  = std::max(DroneStartWaypoint, 0.0);

		//int numtests = 0;
		while (currentPos + 1.0 < (double) Mission.Waypoints.size()) {
			//The mission is not done yet
			int wpIndexA = (int) std::floor(currentPos);
			int wpIndexB = wpIndexA + 1;

			if (wpIndexB < (int) Mission.Waypoints.size()) {
				//In between waypoints A and B
				Eigen::Vector3d PosA_ECEF = LLA2ECEF(Eigen::Vector3d(Mission.Waypoints[wpIndexA].Latitude, Mission.Waypoints[wpIndexA].Longitude, 0.0));
				Eigen::Vector3d PosB_ECEF = LLA2ECEF(Eigen::Vector3d(Mission.Waypoints[wpIndexB].Latitude, Mission.Waypoints[wpIndexB].Longitude, 0.0));

				double t = currentPos - std::floor(currentPos);
				Eigen::Vector3d currentPos_ECEF = (1.0 - t)*PosA_ECEF + t*PosB_ECEF;
				Eigen::Vector3d currentPos_LLA = ECEF2LLA(currentPos_ECEF);

				float timeRemainingAtPos = TimeRemainingAtPos(TA, currentPos_LLA(0), currentPos_LLA(1));
				if ((! std::isnan(timeRemainingAtPos)) && (std::isnan(Margin) || (Margin > timeRemainingAtPos - currentTime)))
					Margin = timeRemainingAtPos - currentTime;
				if ((! std::isnan(Margin)) && (Margin < 0.0))
					return false;

				//Advance time and position - don't advance more than the next waypoint
				//This is important since the separation between waypoint can vary wildly, so going
				//back and forth between fractional waypoint index and time needs to be done within a given
				//pass only or we could even end up skipping entire segments.
				double distBetweenWaypoints = (PosB_ECEF - PosA_ECEF).norm(); //meters
				double targetSpeed = Mission.Waypoints[wpIndexA].Speed; //m/s
				double deltaPos = (deltaT*targetSpeed) / distBetweenWaypoints;
				if (t + deltaPos <= 1.0) {
					currentTime += deltaT;
					currentPos  += deltaPos;
				}
				else {
					double timeToNextWaypoint = ((1.0 - t)*distBetweenWaypoints) / targetSpeed;
					currentTime += timeToNextWaypoint;
					currentPos   = double(wpIndexB);
				}
			}
			else {
				//At last waypoint
				Eigen::Vector3d currentPos_LLA(Mission.Waypoints.back().Latitude, Mission.Waypoints.back().Longitude, 0.0);

				float timeRemainingAtPos = TimeRemainingAtPos(TA, currentPos_LLA(0), currentPos_LLA(1));
				if ((! std::isnan(timeRemainingAtPos)) && (std::isnan(Margin) || (Margin > timeRemainingAtPos - currentTime)))
					Margin = timeRemainingAtPos - currentTime;
				if ((! std::isnan(Margin)) && (Margin < 0.0))
					return false;

				//Advance time and position
				currentTime += deltaT;
				currentPos += 1.0; //Ensure this is the last pass of the loop
			}
			//numtests++;
		}
		//std::cerr << "No shadows. Num tests: " << numtests << "\r\n";

		return true;
	}

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
	//Returns: The index of the drone mission (and sub-region) to task the drone to. Returns -1 if none are viable
	int SelectSubRegion(ShadowPropagation::TimeAvailableFunction const & TA, std::Evector<PolygonCollection> const & SubRegionsNM,
	                    std::vector<DroneInterface::WaypointMission> const & SubregionMissions, std::unordered_set<int> const & AvailableMissionIndices,
	                    DroneInterface::Waypoint const & StartPos, MissionParameters const & MissionParams) {
		using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;

		if (AvailableMissionIndices.empty())
			return -1;

		//For each available mission, figure out if it is viable (can we fly it before getting hit with shadows?). Select the
		//best viable mission. We do this based on distance from the drone to the sub-region. This heuristic is appropriate
		//when we plan on re-planning the mission once a region is selected. If we don't, we could just go based on distance
		//from the first waypoint. What we are doing here has one drawback - the assessment of whether a region is viable is based
		//on the current planned mission for a region, which may not be optimal (given the drones starting point) and may be updated
		//after a drone is tasked to the sub-region. Consequently, we may overlook some regions that are barely viable with a better
		//mission, but not if we fly the original planned mission. This is relatively minor and means our shadow avoidance may err
		//slightly on the cautious side in such circumstances.
		Eigen::Vector2d StartPos_LL(StartPos.Latitude, StartPos.Longitude);
		Eigen::Vector2d StartPos_NM = LatLonToNM(StartPos_LL);

		int    bestSubregionIndex     = -1;
		//double timeToReachBestMission = std::nan("");
		double distToBestRegion       = std::nan("");
		int    numViableRegions       = 0;
		for (int subregionIndex : AvailableMissionIndices) {
			if (! SubregionMissions[subregionIndex].Waypoints.empty()) {
				DroneInterface::Waypoint WP0 = SubregionMissions[subregionIndex].Waypoints[0];
				double timeToReachRegion     = EstimateMissionTime(StartPos, WP0, MissionParams.TargetSpeed);
				TimePoint missionStartTime   = AdvanceTimepoint(std::chrono::steady_clock::now(), timeToReachRegion);
				double margin;
				if (IsPredictedToFinishWithoutShadows(TA, SubregionMissions[subregionIndex], 0.0, missionStartTime, margin)) {
					//This region is viable
					numViableRegions++;

					//Compute distance to sub-region. This is in NM units - we could convert to m using the drones position, but
					//since we are only using this to compare distances we can skip that and just compare distances in NM units.
					double distToSubregion = (SubRegionsNM[subregionIndex].ProjectPoint(StartPos_NM) - StartPos_NM).norm();
					//std::cerr << "Viable sub-region. Dist: " << distToSubregion << " NM units.\r\n";
					if ((bestSubregionIndex < 0) || (distToSubregion < distToBestRegion)) {
						bestSubregionIndex = subregionIndex;
						distToBestRegion   = distToSubregion;
						//std::cerr << "Updating best sub-region. Dist: " << distToBestRegion << " NM units.\r\n";
					}

					//TODO: Here

					/*if ((bestSubregionIndex < 0) || (timeToReachRegion < timeToReachBestMission)) {
						bestSubregionIndex     = subregionIndex;
						timeToReachBestMission = timeToReachRegion;
					}*/
				}
				//else
				//	std::cerr << "Region " << subregionIndex << " is non-viable.\r\n";
			}
		}
		//std::cerr << "Num viable regions: " << numViableRegions << "\r\n";

		return bestSubregionIndex;
	}

	//7 - Given a Time Available function, a collection of sub-regions (with their pre-planned missions), and a collection of drone start positions, choose
	//    sequences (of a given length) of sub-regions for each drone to fly, in order. When the mission time exceeds our prediction horizon the time available
	//    function is no longer useful in choosing sub-regions but they can still be chosen in a logical fashion that avoids leaving holes in the map... making the
	//    optimistic assumption that they will be shadow-free when we get there.
	//TA                  - Input  - Time Available function
	//SubregionMissions   - Input  - A vector of drone Missions - Element n is the mission for sub-region n.
	//DroneStartPositions - Input  - Element k is the starting position of drone k
	//Sequences           - Output - Element k is a vector of sub-region indices to task drone k to (in order)
	void SelectSubregionSequences(ShadowPropagation::TimeAvailableFunction const & TA, std::vector<DroneInterface::WaypointMission> const & SubregionMissions,
	                              std::vector<DroneInterface::Waypoint> const & DroneStartPositions, std::set<int> MissionIndicesToAssign, std::vector<std::vector<int>> & Sequences,
	                              MissionParameters const & MissionParams) {

		Sequences.clear();
		Sequences.resize(DroneStartPositions.size(), std::vector<int>());
		return;
	}
}
