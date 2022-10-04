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
//#include "../../EigenAliases.h"
//#include "../../SurveyRegionManager.hpp"
//#include "../../Polygon.hpp"
//#include "../Shadow-Propagation/ShadowPropagation.hpp"
//#include "../DJI-Drone-Interface/DroneManager.hpp"

#define PI 3.14159265358979323846
#define radiusEarth 6371000 // Radius of the earth metres
#define heightECEF 4.4272e+06;
#define TOLERANCE 1e-10
namespace Guidance {
    // *********************************************************************************************************************************
    // **************************************   GuidanceEngine Non-Inline Functions Definitions   **************************************
    // *********************************************************************************************************************************
    //Start a survey mission (currently active region) using the given drones
    bool GuidanceEngine::StartSurvey(std::vector<std::string> const & LowFlierSerials, ImagingRequirements const & Reqs) {
        std::scoped_lock lock(m_mutex);
        if (m_running)
            return false; //Require stopping the previous mission first
        if (LowFlierSerials.empty())
            return false; //Require at least 1 drone to start a mission
        if (! SurveyRegionManager::Instance().GetCopyOfActiveRegionData(nullptr, &m_surveyRegion, nullptr))
            return false; //No active survey region
        m_ImagingReqs = Reqs;
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

    // TODO: Move this function to a place that makes more sense (i.e., Utilities)
    static void print_vec(std::string const & name, std::vector<int> & v) {
        std::cout << name << "< ";
        for (auto x : v) {
            std::cout << x << " ";
        }
        std::cout << ">";
    }

    // Decides whether sequence should be updated or not, and if so, executes those missions
    void GuidanceEngine::RefreshSequence(void) {

        //m_TA is kept up to date in the main loop now. We don't need to update it here anymore, but we do still need
        //to properly handle the case that we have never received a TA function (in which case m_TA_initialized will be false)
        /*ShadowPropagation::TimeAvailableFunction TA;

        // if uninitialized, init it
        if (!m_TA_initialized) {
            ShadowPropagation::ShadowPropagationEngine::Instance().GetTimestampOfMostRecentTimeAvailFun(m_TA_timestamp);
        }

        // if yes initialized, compare the timestamps of the class variable TA and the one from the shadow prop instance TA
        std::chrono::time_point<std::chrono::steady_clock> newTimestamp;
        if (ShadowPropagation::ShadowPropagationEngine::Instance().GetTimestampOfMostRecentTimeAvailFun(newTimestamp)) {
            if ((newTimestamp > m_TA_timestamp) || !m_TA_initialized) {
                //We should have a new time available function
                std::cout << "New TA timestamp detected" << std::endl;
                if (ShadowPropagation::ShadowPropagationEngine::Instance().GetMostRecentTimeAvailFun(TA)) {
                    std::cout << "Updated TA" << std::endl;
                    m_TA_timestamp = newTimestamp;
                    m_TA_initialized = true;
                }
            }
        }*/

        std::vector<int> currentMissionIndices;

        // Get the mission indices that are currently being flone by drones and print them out
        if (!m_currentDroneMissions.empty() && !m_subregionSequences.empty()) {
            std::cout << "Current Mission Indices: ";
            for (size_t i = 0; i < m_dronesUnderCommand.size(); i++) {
                int missionIndex = std::get<0>(m_currentDroneMissions[m_dronesUnderCommand[i]->GetDroneSerial()]);
                // if mission index of drone is less than size of assigned sequence
                if ((int) m_subregionSequences[i].size() > missionIndex) {
                    currentMissionIndices.push_back(m_subregionSequences[i][missionIndex]);
                } else {
                    currentMissionIndices.push_back(-1); // -1 indicates that this drone is done flying missions
                }
                std::cout << currentMissionIndices[i] << " ";
            }
            std::cout << std::endl;
        }

        m_currentDroneMissions.clear();
        m_flyingMissionStatus.clear();
        m_droneStartPositions.clear();

        // update drone "start positions" with current position
        for (auto drone : m_dronesUnderCommand) {
            DroneInterface::Drone::TimePoint Timestamp;
            double Latitude, Longitude, Altitude, HAG;
            drone->GetPosition(Latitude, Longitude, Altitude, Timestamp);
            drone->GetHAG(HAG, Timestamp);

            DroneInterface::Waypoint Waypoint;
            Waypoint.Latitude = Latitude;
            Waypoint.Longitude = Longitude;
            Waypoint.RelAltitude = HAG;

            m_droneStartPositions.push_back(Waypoint);
        }
        int currentSequenceCoverageExpected = 0, newSequenceCoverageExpected;
        std::vector<std::vector<int>> currentSequences = m_subregionSequences;
        std::vector<int> PredictedCoverage, MissionDurations;

        m_subregionSequences.clear();
        static int prevNumMissions = 0;
        // Select subregion sequences
        SelectSubregionSequences(m_TA, m_droneMissions, m_droneStartPositions, m_missionIndicesToAssign, m_subregionSequences, m_ImagingReqs);
        std::cout << "Size of mission indices: " << m_missionIndicesToAssign.size() << std::endl;

        // Below, we compare coverage of new best sequence to recalculate coverage (based on updated m_TA) of the previous best found sequence
        // We only care about comparison of new coverage to old coverage if old coverage > 0 (because why would we care if it was 0 before? anything would be better than that)
        if (m_coverageExpected > 0) {
            std::cout << currentSequences.size() << std::endl;
            std::cout << m_dronesUnderCommand.size() << std::endl;
            currentSequenceCoverageExpected = GetCoverage(PredictedCoverage, MissionDurations, m_TA, m_droneMissions, currentSequences, m_droneStartPositions, m_ImagingReqs, m_dronesUnderCommand.size());
            PredictedCoverage.clear();
            MissionDurations.clear();
        }
        newSequenceCoverageExpected = GetCoverage(PredictedCoverage, MissionDurations, m_TA, m_droneMissions, m_subregionSequences, m_droneStartPositions, m_ImagingReqs, m_dronesUnderCommand.size());
        std::cout << "current coverage: " << currentSequenceCoverageExpected << ", new coverage: " << newSequenceCoverageExpected << std::endl;
        

        std::cout << "sequence: <";
        for (auto sequence : currentSequences) {
            print_vec("", sequence);
        }
        std::cout << ">" << std::endl;

        std::cout << "sequence: <";
        for (auto sequence : m_subregionSequences) {
            print_vec("", sequence);
        }
        std::cout << ">" << std::endl;

        // If the number of missions left to assign has changed (i.e., mission was completed), or if using new m_TA, new sequence coverage > old sequence coverage
        if (m_missionIndicesToAssign.size() != prevNumMissions || newSequenceCoverageExpected > currentSequenceCoverageExpected) {
            m_coverageExpected = newSequenceCoverageExpected;
            std::vector<bool> startMissionFromBeginning;
            std::cout << "New Mission Indices: ";
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
                    // only start mission again if flag for starting mission from the beginning is set
                    if (startMissionFromBeginning[i]) {
                        DroneInterface::WaypointMission Mission = std::get<1>(m_currentDroneMissions[drone->GetDroneSerial()]);
                        drone->ExecuteWaypointMission(Mission);
                    }
                    m_flyingMissionStatus.push_back(true);
                } else {
                    m_flyingMissionStatus.push_back(false);
                }
            }
        } else {
            // Criteria for changing sequence met, so reset any state that has changed while running this function to what it has been
            m_coverageExpected = currentSequenceCoverageExpected;
            m_subregionSequences = currentSequences;
        }
        prevNumMissions = m_missionIndicesToAssign.size();
    }

    inline void GuidanceEngine::ModuleMain(void) {
        // float refresh_period = 35.0;
        float refresh_period = 10.0;
        std::chrono::time_point<std::chrono::steady_clock> MostRecentSequenceTimestamp = std::chrono::steady_clock::now();
        while (! m_abort) {
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
                double TargetFlightTime = 100;

                // m_flyingMissionStatus.clear();

                PartitionSurveyRegion(m_surveyRegion, m_surveyRegionPartition, TargetFlightTime, m_ImagingReqs);

                for (auto partition : m_surveyRegionPartition) {
                    DroneInterface::WaypointMission Mission;
                    PlanMission(partition, Mission, m_ImagingReqs);
                    m_droneMissions.push_back(Mission);
                }

                for (size_t i = 0; i < m_droneMissions.size(); i++) {
                    m_missionIndicesToAssign.insert(i);    
                }

                m_coverageExpected = 0;
                RefreshSequence();

                m_missionPrepDone = true; //Mark the prep work as done
            }

            //Make sure we have the most recent TA function (if any are available)
            std::chrono::time_point<std::chrono::steady_clock> newTimestamp;
            if (ShadowPropagation::ShadowPropagationEngine::Instance().GetTimestampOfMostRecentTimeAvailFun(newTimestamp)) {
                //A TA function is available and we now have it's timestamp
                if ((!m_TA_initialized) || (newTimestamp > m_TA.Timestamp)) {
                    ShadowPropagation::ShadowPropagationEngine::Instance().GetMostRecentTimeAvailFun(m_TA);
                    m_TA_initialized = true;
                }
            }

            //Debug Only - update partition labels with whether or not each component would be finished before shadows if started now
            std::vector<std::string> partitionLabels;
            partitionLabels.reserve(m_surveyRegionPartition.size());
            for (size_t compIndex = 0U; compIndex < m_surveyRegionPartition.size(); compIndex++) {
                double margin = 0.0;
                bool willFinish = IsPredictedToFinishWithoutShadows(m_TA, m_droneMissions[compIndex], 0.0, std::chrono::steady_clock::now(), margin);
                std::string label = willFinish ? "Will Finish"s : "Will Not Finish"s;
                label += "\nMargin: "s + std::to_string(margin);
                partitionLabels.push_back(label);
            }
            MapWidget::Instance().m_guidanceOverlay.SetPartitionLabels(partitionLabels);

            std::chrono::time_point<std::chrono::steady_clock> NowTimestamp = std::chrono::steady_clock::now();
            if (SecondsElapsed(MostRecentSequenceTimestamp, NowTimestamp) > refresh_period) {
                    std::cout << "REFRESHING SEQUENCE" << std::endl;
                    RefreshSequence();
                    MostRecentSequenceTimestamp = std::chrono::steady_clock::now();
                    // refresh_period = 10.0;
            }


            for (size_t i = 0; i < m_dronesUnderCommand.size(); i++) {
                bool Result;
                DroneInterface::Drone::TimePoint Timestamp;
                auto drone = m_dronesUnderCommand[i];
                drone->IsCurrentlyExecutingWaypointMission(Result, Timestamp);

                // if flying --> not flying transition detected
                if ((Result != m_flyingMissionStatus[i]) && (Result == false)) {
                    int currentMissionIndex = m_subregionSequences[i][std::get<0>(m_currentDroneMissions[drone->GetDroneSerial()])++];
                    m_missionIndicesToAssign.erase(currentMissionIndex);
                    if ((int) m_subregionSequences[i].size() > std::get<0>(m_currentDroneMissions[drone->GetDroneSerial()])) {
                        int missionIndex = m_subregionSequences[i][std::get<0>(m_currentDroneMissions[drone->GetDroneSerial()])];
                        std::get<1>(m_currentDroneMissions[drone->GetDroneSerial()]) = m_droneMissions[missionIndex];
                        DroneInterface::WaypointMission Mission = std::get<1>(m_currentDroneMissions[drone->GetDroneSerial()]);
                        drone->ExecuteWaypointMission(Mission);
                    }
                }
                m_flyingMissionStatus[i] = Result;
            }

            MapWidget::Instance().m_guidanceOverlay.SetMissions(m_droneMissions);
            MapWidget::Instance().m_guidanceOverlay.SetDroneMissionSequences(m_subregionSequences);


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

    //3.1 Helper Function -- Gets the "score" (estimated flight time) of a triangle based on a square.
    double GetTriangleScore(Triangle & triangle, ImagingRequirements const & ImagingReqs){
        //Row spacing = 2 * HAG * tan(0.5 * HFOV) * (1 - SidelapFraction) <In Meters>
        double row_spacing = 2 * ImagingReqs.HAG * tan(0.5 * ImagingReqs.HFOV) * (1 - ImagingReqs.SidelapFraction);
        double length = sqrt(2 * triangle.GetArea());
        return length * (length / row_spacing) / ImagingReqs.TargetSpeed;
    }

    //3.2 Helper Function -- Recursively bisects triangles until they score less than the targeted flight time.
    void PartitionSurveyRegionRec(Triangle & mainTriangle, std::Evector<Triangle> & allTriangles, double TargetFlightTime, ImagingRequirements const & ImagingReqs){
        if (GetTriangleScore(mainTriangle, ImagingReqs) > TargetFlightTime){
            //Split the triangle into two triangles
            Triangle subTriangleA;
            Triangle subTriangleB;
            mainTriangle.Bisect(subTriangleA, subTriangleB);

            PartitionSurveyRegionRec(subTriangleA, allTriangles, TargetFlightTime, ImagingReqs);
            PartitionSurveyRegionRec(subTriangleB, allTriangles, TargetFlightTime, ImagingReqs);

        }
        else{
            allTriangles.push_back(mainTriangle);

        }
    }

    void PartitionTrianglesList(Triangle & mainTriangle, std::Evector<Triangle> & allTriangles, std::vector<Eigen::Vector2d> pointsList){
        // Base case, just one point
        if (pointsList.size() == 1){
            Triangle subTriangleA;
            Triangle subTriangleB;
            mainTriangle.BisectIntersection(pointsList[0], subTriangleA, subTriangleB);
            allTriangles.push_back(subTriangleA);
            allTriangles.push_back(subTriangleB);
        }
        // other base case: empty list
        else if (pointsList.empty()){
            allTriangles.push_back(mainTriangle);
        }

        else{
            Triangle subTriangleA;
            Triangle subTriangleB;
            std::vector<Eigen::Vector2d> pointsListA;
            std::vector<Eigen::Vector2d> pointsListB;
            mainTriangle.BisectIntersection(pointsList[0], subTriangleA, subTriangleB);

            for (int j = 1; j < (int) pointsList.size(); j++){
                if (subTriangleA.ToSimplePolygon().ContainsPoint(pointsList[j])){
                    pointsListA.push_back(pointsList[j]);
                }
                else{
                    pointsListB.push_back(pointsList[j]);
                }
            }
            PartitionTrianglesList(subTriangleA, allTriangles,pointsListA);
            PartitionTrianglesList(subTriangleB, allTriangles,pointsListB);

        }
    }

    //3 - Take a survey region, and break it into sub-regions of similar size that can all be flown in approximately the same flight time (argument).
    //    A good partition uses as few components as possible for a given target execution time. Hueristically, this generally means simple shapes.
    //    This function needs the target drone speed and imaging requirements because they impact what sized region can be flown in a given amount of time.
    //Arguments:
    //Region           - Input  - The input survey region to cover (polygon collection in NM coords)
    //Partition        - Output - A vector of sub-regions, each one a polygon collection in NM coords (typical case will have a single poly in each sub-region)
    //TargetFlightTime - Input  - The approx time (in seconds) a drone should be able to fly each sub-region in, given the max vehicle speed and sidelap
    //ImagingReqs      - Input  - Parameters specifying speed and row spacing (see definitions in struct declaration)

    void PartitionSurveyRegion(PolygonCollection const & Region, std::Evector<PolygonCollection> & Partition, double TargetFlightTime, ImagingRequirements const & ImagingReqs) {
        // *********************** MESHING SECTION **************************************************
        //Slice the polygon into triangles.
        std::Evector<Triangle> allTrianglesTriangulate;
        std::Evector<Triangle> allTriangles;
        TriangleAdjacencyMap map;
        std::vector<std::vector<Eigen::Vector2d>> intersectionsList;
        std::vector<Eigen::Vector2d> Intersection;
        for(Polygon shape: Region.m_components){
            std::Evector<Triangle> subTriangles;
            shape.Triangulate(subTriangles);
            for (Triangle & tri: subTriangles){
                allTrianglesTriangulate.push_back(tri);
            }
        }

        //Recursively slice triangles until they are smaller than the Target Flight Time.
        for(Triangle & triangle: allTrianglesTriangulate){
            PartitionSurveyRegionRec(triangle, allTriangles, TargetFlightTime/2, ImagingReqs);
        }


        allTrianglesTriangulate.clear();

        intersectionsList.resize(allTriangles.size());


        // This loop searches all triangles for vertices that cut an edge
        // A triangle may cut an edge 0, 1, or 2 times
        // All intersections are assembled into a list per triangle. We are trying to avoid duplicates
        for (int i = 0; i < (int) allTriangles.size(); i++) {
            for (int j = 0; j < (int) allTriangles.size(); j++) {
                Intersection.clear();
                if (i!=j && allTriangles[i].ToSimplePolygon().CheckIntersect(allTriangles[j].ToSimplePolygon(), Intersection)){
                    //std::cout<< Intersection.size() <<" intersections found!" <<std::endl;
                    for (auto point: Intersection){
                        bool isUnique = true;
                        // Checking list for existence of same point. If exists, not unique and do not add to list
                        for (int k = 0; k < (int) intersectionsList[i].size(); k++){
                            if ((intersectionsList[i][k] - point).norm() < TOLERANCE){
                                isUnique = false;
                            }
                        }
                        if (isUnique){
                            //std::cout<< "Adding point! " << std::endl;
                            intersectionsList[i].push_back(point);
                        }
                    }
                    std::cout<< "Triangle " << std::to_string(i) << " has intersection with triangle " << j  << std::endl;
                }
            }
        }

        for (int i = 0; i < (int) intersectionsList.size(); i++) {
            if (!intersectionsList[i].empty()){
                std::cout<< "Triangle " << std::to_string(i) << " has intersections " << intersectionsList[i].size() << std::endl;
            }
        }

        Triangle subTriangleA;
        Triangle subTriangleB;
        allTrianglesTriangulate.clear();
        for (int i = 0; i < (int) allTriangles.size(); i++) {
            if (intersectionsList[i].empty()){
                allTrianglesTriangulate.push_back(allTriangles[i]);
            }
            else{

                /*
                 *                 for (auto point: intersectionsList[i]){
                                    Triangle subTriangleA;
                                    Triangle subTriangleB;
                                    allTriangles[i].BisectIntersection(point, subTriangleA, subTriangleB);
                                    allTrianglesTriangulate.push_back(subTriangleA);
                                    allTrianglesTriangulate.push_back(subTriangleB);
                                }
                 */
                PartitionTrianglesList(allTriangles[i], allTrianglesTriangulate, intersectionsList[i]);
            }

        }

        std::vector<std::string> triangleLabels;

        for (int i = 0; i < (int) allTrianglesTriangulate.size(); i++) {
            triangleLabels.push_back(std::to_string(i));
        }
        //MapWidget::Instance().m_guidanceOverlay.ClearTriangleLabels();
        //MapWidget::Instance().m_guidanceOverlay.ClearTriangles();
        //MapWidget::Instance().m_guidanceOverlay.SetTriangles(allTrianglesTriangulate);
        //MapWidget::Instance().m_guidanceOverlay.SetTriangleLabels(triangleLabels);

        allTriangles = allTrianglesTriangulate; // ECHAI: Not great but I'm feeling lazy
        //return;
        //************************* END OF MESHING ****************************************
        // Up to this point, we are guaranteed that no vertex cuts another edge!

        //************************ ADJACENCY MAP CONSTRUCTION********************************
        //Add each triangle and its edges to the map.

        for (int i = 0; i < (int) allTriangles.size(); i++) {
            TriangleAdjacencyMap::Triangle tri;
            tri.id = i;
            tri.score = GetTriangleScore(allTriangles[i], ImagingReqs);
            map.nodes.push_back(tri);
            map.edges.push_back({});
            for (int j = 0; j < (int) allTriangles.size(); j++) {
                if((i!=j) && allTriangles[i].ToSimplePolygon().CheckAdjacency(allTriangles[j].ToSimplePolygon())){
                    map.edges[i].push_back(j);
                }
            }
        }
        // ****************** END OF ADJACENCY MAP CONSTRUCTION ************************************

        // ****************** GROUP/PARITION CONSTRUCTION ********************************************
        //Combine triangles greedily until adding another adjacent triangle would go above the Target Flight Time.
        std::vector<int> groupScoreList;
        std::vector<int> assignedGroups(allTriangles.size(), -1);
        for (auto node: map.nodes){
            std::vector<int> currentCandidateEdges;
            //If the node is already in a group, ignore it
            if (assignedGroups[node.id] != -1){
                continue;
            }

            //Otherwise, create a group for the solo node.
            map.groups.push_back({node.id});
            groupScoreList.push_back(node.score);
            assignedGroups[node.id] = map.groups.size()-1;

            //Add in edges from the starting node of the group.
            for(auto item: map.edges[node.id]){
                currentCandidateEdges.push_back(item);
            }

            //While there are edges attached to this group, try to add them in.
            while(currentCandidateEdges.size() > 0){
                int n = currentCandidateEdges[0];
                if(map.nodes[n].score + groupScoreList.back() < TargetFlightTime && assignedGroups[n] == -1){
                    map.groups.back().push_back(map.nodes[n].id);
                    groupScoreList.back() += map.nodes[n].score;
                    assignedGroups[n] = map.groups.size()-1;
                    //Add edges to currentCandidateEdges
                    for(auto item: map.edges[n]){
                        currentCandidateEdges.push_back(item);
                    }
                }
                currentCandidateEdges.erase(currentCandidateEdges.begin());
            }

            //Now, create the new polygon & add it to its own private collection in the Evector of PolygonCollections.
            PolygonCollection collection;
            Polygon new_polygon;
            std::Evector<LineSegment> list_of_segments;
            for (auto node: map.groups.back()){
                for (LineSegment line: allTriangles[node].ToSimplePolygon().GetLineSegments()){
                    list_of_segments.push_back(line);
                }
            }
            new_polygon.m_boundary.CustomSanitize(list_of_segments);
            collection.m_components.push_back(new_polygon);
            Partition.push_back(collection);
        }

        //Set labels for the grouped shapes.
        std::vector<std::string> groupLabels;
        std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
        for (int n = 0; n < (int) map.groups.size(); n++)
        	groupLabels.push_back(alphabet.substr(n, 1));

        //Display both the triangles and the grouped polygons on Recon.
        MapWidget::Instance().m_guidanceOverlay.ClearPartitionLabels();
        MapWidget::Instance().m_guidanceOverlay.ClearSurveyRegionPartition();
        MapWidget::Instance().m_guidanceOverlay.SetSurveyRegionPartition(Partition);
        MapWidget::Instance().m_guidanceOverlay.SetPartitionLabels(groupLabels);

        //Set labels for the individual triangles.
        //std::vector<std::string> triangleLabels;
        triangleLabels.clear();
        for (int i = 0; i < (int) map.nodes.size(); i++) {
            std::string fullLabel = alphabet[assignedGroups[i]] + std::to_string(i) + ":";
            for (int j = 0; j < (int) map.edges[i].size(); j++) {
                if (j!=0){
                    fullLabel.append(",");
                }
                fullLabel.append(std::to_string(map.edges[i][j]));
            }
            triangleLabels.push_back(fullLabel);
        }

        MapWidget::Instance().m_guidanceOverlay.ClearTriangleLabels();
        MapWidget::Instance().m_guidanceOverlay.ClearTriangles();
        MapWidget::Instance().m_guidanceOverlay.SetTriangles(allTriangles);
        MapWidget::Instance().m_guidanceOverlay.SetTriangleLabels(triangleLabels);

        //(NON-IMPACTING -- FOR DEVELOPER USE ONLY) Display polygon information on the terminal.
        std::cout<<"Target Score: " << TargetFlightTime << std::endl;
        std::cout<<"There are " << map.nodes.size() << " nodes in the graph. This should equal the number of triangles." << std::endl;
        std::cout<<"There are " << map.groups.size() << " groups in the graph." << std::endl;
        //Information on individual triangles.
        for (int i = 0; i < (int) map.nodes.size(); i++) {
            float value = (int)(map.nodes[i].score * 100 + .5);
            std::cout << "Edges of " << i <<" (Score " << (float)value / 100 << "; Group [" << alphabet[assignedGroups[i]] <<"]) : ";
            for (int j = 0; j < (int) map.edges[i].size(); j++) {
                std::cout << map.edges[i][j] << ", ";
            }
            std::cout << std::endl;
        }
        //Information on grouped triangles.
        for (int n = 0; n < (int) map.groups.size(); n++) {
            std::cout << "Group "<< alphabet.substr(n, 1) << " :";
            for (auto item: map.groups[n]){
                std::cout << " " << item;
            }
            std::cout << "." << std::endl;
        }

    }

    Eigen::Vector2d NM2ECEF(const Eigen::Vector2d & point_NM){
        Eigen::Vector3d tempCoordinate_LL3d;
        Eigen::Vector3d tempCoordinate_ECEF;
        Eigen::Vector2d point_ECEF;

        Eigen::Vector2d tempC = NMToLatLon(point_NM);

        tempCoordinate_LL3d << tempC(0), tempC(1), 0.0;
        tempCoordinate_ECEF = LLA2ECEF(tempCoordinate_LL3d);
        point_ECEF << tempCoordinate_ECEF(0), tempCoordinate_ECEF(1);
        return  point_ECEF;

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
        std::Evector<Eigen::Vector2d> vertices_NM;
        std::Evector<PolygonCollection> Partition;

        std::Evector<std::tuple<LineSegment, float, Eigen::Vector3f>> lineSegments;
        std::Evector<std::tuple<LineSegment, float, Eigen::Vector3f>> hatchLinesVisualize;
        std::Evector<std::tuple<Eigen::Vector2d, float, Eigen::Vector3f>> circles;

        float lineThickness = 2.0f;
        float circleRadius  = 6.5f;
        Eigen::Vector3f lineColor  (0.0f, 0.8f, 0.0f);
        Eigen::Vector3f circleColor(0.8f, 0.0f, 0.0f);

        for (auto subregion: Region.m_components){
            std::Evector<LineSegment> edges = subregion.m_boundary.GetLineSegments();
            SimplePolygon tempPolygon;

            // These variables are for the calculating the hatchlines
            std::vector<LineSegment> hatchLines;
            std::vector<LineSegment> hatchLines_NM;
            LineSegment longest_edge;
            double longest_side_length = 0.0;


            // Finding the longest edge of the simple polygon
            for (auto edge: edges){
                if (edge.GetLength() > longest_side_length){
                    longest_side_length = edge.GetLength();
                    longest_edge = edge;
                }
            }
            lineSegments.push_back(std::make_tuple(longest_edge, lineThickness, lineColor));
            // Step 1: Rotation
            // 1a: translation: I want to be able to view the rotation, which means this is a rotation around a point
            // Point (x1,y1) to be rotatated around point (x2,y2)
            // (xt, yt) = (x1,y1) - (x2,y2)
            // angle to x-axis given by arctan(x/y)
            // Point of rotation will be edge vertex closest to zero

            Eigen::Vector2d rotationPoint; // This will be the point of rotation.
            Eigen::Vector2d pointTranslated; // This will be the effective point to rotate

            if (longest_edge.m_endpoint1.norm() < longest_edge.m_endpoint2.norm()){
                rotationPoint = longest_edge.m_endpoint1;
                pointTranslated = longest_edge.m_endpoint2 - longest_edge.m_endpoint1;
            }
            else{
                rotationPoint = longest_edge.m_endpoint2;
                pointTranslated = longest_edge.m_endpoint1 - longest_edge.m_endpoint2;
            }
            double theta = atan2(pointTranslated(1), pointTranslated(0));

            // Calculating rotation matrix
            Eigen::Matrix2d rot2D;
            rot2D << cos(-theta), -sin(-theta),
            sin(-theta),  cos(-theta);

            // Create rotated polygon
            vertices_NM.clear();
            for (auto vertex: subregion.m_boundary.GetVertices()){
                vertices_NM.push_back((rot2D *(vertex-rotationPoint))+rotationPoint);
            }

            tempPolygon.SetBoundary(vertices_NM); // Create our rotated polygon

            //********************** FOR PLOTTING PURPOSES *********************************

            //For visualization purposes, we are plotting original and rotated polygon
            Partition.emplace_back(); //Create a new element in the partition
            Partition.back().m_components.emplace_back(); //Add a component to the new element
            Partition.back().m_components.back().m_boundary.SetBoundary(subregion.m_boundary.GetVertices());

            Partition.emplace_back(); //Create a new element in the partition
            Partition.back().m_components.emplace_back(); //Add a component to the new element
            Partition.back().m_components.back().m_boundary.SetBoundary(vertices_NM);
            // ****************** END OF FOR PLOTTING PURPOSES ********************
            //****************** END OF POLYGON ROTATION **************************

            //****************** POLYGON FILL *************************************
            // Create bounding box containing our polygon
            Eigen::Vector4d bounding_box = tempPolygon.GetAABB(); //vector (XMin, XMax, YMin, YMax)
            Eigen::Vector2d MinMin_NM; // Store (XMin, YMin)
            Eigen::Vector2d MaxMax_NM; // Store (XMax, YMax)
            Eigen::Vector2d MinMin_ECEF; // Store (XMin, YMin), for debugging purposes
            Eigen::Vector2d MaxMax_ECEF; // Store (XMax, YMax), for debugging purposes

            MinMin_NM << bounding_box(0), bounding_box(2);
            MaxMax_NM << bounding_box(1), bounding_box(3);
            vertices_NM.clear();
            vertices_NM.emplace_back(MinMin_NM);
            vertices_NM.emplace_back(MaxMax_NM);

            // ****************** HATCH LINE GENERATION ****************************************
            // Fill the polygon with hatch lines
            // 1) Figure out spacing.
            //      1a) Convert bounding box to meters
            //      1b) Get straight line trajectories with overlap using Imaging requirements

            // 1a) Convert bounding box to meters

            // 1b) Get straight line trajectories based on imaging requirements
            // Start first hatch line half the row spacing from top of box, which is yMax
            // Build horizontal lines until y-value is less than yMin
            // All lines go from Xmin to Xmax
            double rowSpacing = 2 * ImagingReqs.HAG * tan(0.5 * ImagingReqs.HFOV) * (1 - ImagingReqs.SidelapFraction);
            double rowSpacing_NM = MetersToNMUnits(rowSpacing, MaxMax_NM(1));

            // This is for debugging. Useful to understand distances in meters
            double numHatchLines = ceil((MaxMax_ECEF(1) - MinMin_ECEF(1)- (rowSpacing/2))/rowSpacing);
            double y_init = MaxMax_ECEF(1)-(rowSpacing/2.0);
            numHatchLines = ceil((MaxMax_NM(1) - MinMin_NM(1)- (rowSpacing_NM/2))/rowSpacing_NM);
            y_init = MaxMax_NM(1)-(rowSpacing_NM/2.0);


            for (int i = 0; i<numHatchLines; i++){
                //InputSegments.emplace_back(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0));
                hatchLines.emplace_back(
                        Eigen::Vector2d(MinMin_NM(0), y_init-(i*rowSpacing_NM)),
                        Eigen::Vector2d(MaxMax_NM(0), y_init-(i*rowSpacing_NM)));
            }

            //********************* END OF HATCHLINE GENERATION ********************************************

            // ******************* GENERATE FRAGMENTED LINES FOR TRAJECTORIES ******************************
            // Now we break the hatchlines
            // For each hatchline
            // 2a) Break line where it intersects with polygon edges
            // 2b) Keep segment that is internal to polygon. Note that a line segment that might break in multiple places
            std::Evector<LineSegment> FragmentedHatchLines;

            for (auto currentHatchLine: hatchLines){
                std::vector<Eigen::Vector2d> allIntersections;
                bool IsInteriorA, IsInteriorB;

                // For each polygon edge, collect all intersections
                std::vector<double> m1_norm;
                for (auto edge: tempPolygon.GetLineSegments() ){
                    Eigen::Vector2d Intersection;
                    if (edge.ComputeInteriorIntersection(currentHatchLine, Intersection, IsInteriorA, IsInteriorB)){
                        allIntersections.push_back(Intersection);
                        m1_norm.push_back((Intersection-currentHatchLine.m_endpoint1).norm());
                    }
                }
                // Sort intersections according to distance to one endpoint of hatchline
                std::vector<std::pair<double,int> >V;
                for (int i = 0; i < (int) m1_norm.size(); i++) {
                    std::pair<double,int>P=std::make_pair(m1_norm[i],i);
                    V.push_back(P);
                }
                sort(V.begin(),V.end());
                // V is now the intersections sorted by their norm distance to one end-point of hatchline

                // Now construct line segments, keeping only those inside the polygon
                // Guaranteed that line segment in each loop is possible shortest
                // Therefore it is either completely inside or outside polygon
                // Therefore the mid-point must be completely inside or outside polygon
                for (int i = 1; i < (int) V.size(); i++) {
                    Eigen::Vector2d midPoint = (allIntersections[V[i-1].second]+allIntersections[(V[i].second)])/2;
                    if(tempPolygon.ContainsPoint(midPoint)){
                        FragmentedHatchLines.push_back(LineSegment(allIntersections[V[i-1].second],allIntersections[(V[i].second)]));
                    }
                }
            }
            // ***************** FRAGMENTED LINES GENERATION COMPLETED *****************************

            // For each line segment
            //      1a) Populate waypoint vector with end points
            //              Shift end_points inwards by 0.5*rowSpacing_NM
            //              If line length is larger than rowspacing_NM, then the shift will result in a single point
            //                  Populate waypoint vector with just that single midpoint
            //              Otherwise, populate with m1 and m2
            //      1b) Create temp sorted list of norm and index of all remaining endpoints, see example code above
            //              The closest endpoint will be at index 0
            //              The associated line segment will be the next line
            //      1c) Undo the rotation of the points.
            //            NOTE WELL!!!! This function rotates around a point that is NOT the origin
            //            Therefore, the rotation operation is technically a translation, then rotation, then translation
            //

            //1a: Populate waypoint vector with end points
            for (auto & line: FragmentedHatchLines){
                if(line.GetLength() < rowSpacing_NM){
                    LineSegment tempLine;
                    tempLine.m_endpoint1 = (line.m_endpoint1 + line.m_endpoint2) / 2;
                    tempLine.m_endpoint2 = tempLine.m_endpoint1;
                    line = tempLine;
                } else {
                    if(line.m_endpoint1[0] > line.m_endpoint2[0]){
                        line.m_endpoint1[0] -= 0.5*rowSpacing_NM;
                        line.m_endpoint2[0] += 0.5*rowSpacing_NM;
                    } else {
                        line.m_endpoint1[0] += 0.5*rowSpacing_NM;
                        line.m_endpoint2[0] -= 0.5*rowSpacing_NM;
                    }
                }
            }
            //1b: Create temp sorted list of norm and index of all remaining endpoints, see example code above [WARNING: UNSORTED]
            std::vector<Eigen::Vector2d> orderedPoints;
            std::vector<bool> hasLineBeenUsed (FragmentedHatchLines.size(), false);
            bool isEndpointOne = true;
            int chosenLineIndex = 0;
            hasLineBeenUsed[0] = true;
            orderedPoints.push_back(FragmentedHatchLines[0].m_endpoint2);
            orderedPoints.push_back(FragmentedHatchLines[0].m_endpoint1);
            while (std::find(hasLineBeenUsed.begin(), hasLineBeenUsed.end(), false) != hasLineBeenUsed.end()){
                Eigen::Vector2d currentEndpoint;
                Eigen::Vector2d chosenPoint;
                if(isEndpointOne) currentEndpoint = FragmentedHatchLines[chosenLineIndex].m_endpoint1;
                else currentEndpoint = FragmentedHatchLines[chosenLineIndex].m_endpoint2;
                chosenPoint = currentEndpoint;
                //Find the closest unused point to this one
                for (int x = 0; x < (int) hasLineBeenUsed.size(); x++) {
                    if(hasLineBeenUsed[x]) continue;
                    if (chosenPoint == currentEndpoint){
                        chosenLineIndex = x;
                        isEndpointOne = false;
                        chosenPoint = FragmentedHatchLines[x].m_endpoint1;
                    }
                    if((chosenPoint - currentEndpoint).norm() > (FragmentedHatchLines[x].m_endpoint1 - currentEndpoint).norm()) {
                        chosenLineIndex = x;
                        isEndpointOne = false;
                        chosenPoint = FragmentedHatchLines[x].m_endpoint1;
                    }
                    if((chosenPoint - currentEndpoint).norm() > (FragmentedHatchLines[x].m_endpoint2 - currentEndpoint).norm()) {
                        chosenLineIndex = x;
                        isEndpointOne = true;
                        chosenPoint = FragmentedHatchLines[x].m_endpoint2;
                    }
                }
                hasLineBeenUsed[chosenLineIndex] = true;
                if(isEndpointOne){
                    orderedPoints.push_back(FragmentedHatchLines[chosenLineIndex].m_endpoint2);
                    orderedPoints.push_back(FragmentedHatchLines[chosenLineIndex].m_endpoint1);
                } else {
                    orderedPoints.push_back(FragmentedHatchLines[chosenLineIndex].m_endpoint1);
                    orderedPoints.push_back(FragmentedHatchLines[chosenLineIndex].m_endpoint2);
                }
            }

            //1c: Undo Rotation of the points.
            rot2D << cos(theta), -sin(theta),
            sin(theta),  cos(theta);
            for (auto & line: FragmentedHatchLines){
                line.m_endpoint1 = (rot2D *(line.m_endpoint1-rotationPoint))+rotationPoint;
                line.m_endpoint2 = (rot2D *(line.m_endpoint2-rotationPoint))+rotationPoint;
            }
            for (auto & point: orderedPoints){
                point = (rot2D *(point-rotationPoint))+rotationPoint;
                Eigen::Vector2d LatLonPoint;
                LatLonPoint = NMToLatLon(point);
                DroneInterface::Waypoint newWayPoint;
                newWayPoint.Latitude = LatLonPoint[0];
                newWayPoint.Longitude = LatLonPoint[1];
                newWayPoint.RelAltitude = ImagingReqs.HAG;
                newWayPoint.Speed = ImagingReqs.TargetSpeed;
                newWayPoint.LoiterTime   = std::nanf("");
                newWayPoint.GimbalPitch   = std::nanf("");
                newWayPoint.CornerRadius = 5.0f;
                Mission.Waypoints.push_back(newWayPoint);
            }
        }
        //MapWidget::Instance().m_guidanceOverlay.SetSurveyRegionPartition(Partition);
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
    //ImagingReqs       - Input - Parameters specifying speed and row spacing (see definitions in struct declaration)
    //
    //Returns: The index of the drone mission (and sub-region) to task the drone to. Returns -1 if none are plausable
    int SelectSubRegion(ShadowPropagation::TimeAvailableFunction const & TA, std::vector<DroneInterface::WaypointMission> const & SubregionMissions,
                        DroneInterface::Waypoint const & StartPos, ImagingRequirements const & ImagingReqs) {
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
    int GetCoverage(std::vector<int> & PredictedCoverage, std::vector<int> & MissionDurations, ShadowPropagation::TimeAvailableFunction const & TA, std::vector<DroneInterface::WaypointMission> const & SubregionMissions, std::vector<std::vector<int>> const & Sequences, std::vector<DroneInterface::Waypoint> const & StartPositions, ImagingRequirements const & ImagingReqs, const int NumDrones) {
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
                // Add time from current position to starting waypoint of mission
                DroneStartTime += std::chrono::seconds((int) EstimateMissionTime(DronePos, currentMission.Waypoints[0], ImagingReqs.TargetSpeed));
                // std::cout << "Time to starting waypoint " << (int) EstimateMissionTime(DronePos, currentMission.Waypoints[0], ImagingReqs.TargetSpeed) << ", ";
                // std::chrono::time_point<std::chrono::steady_clock> T0 = std::chrono::steady_clock::now();
                if (IsPredictedToFinishWithoutShadows(TA, currentMission, 0, DroneStartTime, Margin)) {
                    coverage++;
                    PredictedCoverage[drone_idx]++;
                    // std::cout << "Predicted to finish" << std::endl;
                    // std::cout << "Mission " << destWaypointIndex << " expected to finish: TRUE, ";
                } else {
                    // std::cout << "Predicted not to finish" << std::endl;
                    // std::cout << "Mission " << destWaypointIndex << " expected to finish: FALSE, ";
                }
                // std::cout << "Margin: " << Margin << ", ";
                // processingTime = SecondsElapsed(T0, std::chrono::steady_clock::now());
                // // std::cerr << "IsPredictedToFinishWithoutShadows: " << processingTime*1000.0 << " ms" << std::endl;
                // T0 = std::chrono::steady_clock::now();
                // Add time to fly mission
                // DroneStartTime += std::chrono::seconds(20);
                DroneStartTime += std::chrono::seconds((int) EstimateMissionTime(currentMission, ImagingReqs.TargetSpeed));
                // // std::cout << "Time to next waypoint " << (int) EstimateMissionTime(currentMission, ImagingReqs.TargetSpeed) << std::endl;
                // processingTime = SecondsElapsed(T0, std::chrono::steady_clock::now());
                // // std::cerr << "EstimateMissionTime: " << processingTime*1000.0 << " ms" << std::endl;
                // Set current position to last waypoint of mission
                DronePos = currentMission.Waypoints.back();

                // time from starting location of drone to first waypoint of next mission
                // time to fly mission
                // time from last waypoint of mission to first waypoint of next mission
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
                                 ImagingRequirements const & ImagingReqs) {
        
        int numDrones = DroneStartPositions.size();
        int numMissions = SubregionMissions.size();

        // std::cout << "numDrones: " << numDrones << ", numMissions: " << numMissions << std::endl;

//}

        // Naive
        // Sequences.resize(numDrones, std::vector<int>());

        // if (numDrones > numMissions) {
        // } else if (numDrones <= numMissions) {
        //     for (int i = 0; i < numMissions; i++) {
        //         Sequences[i % numDrones].push_back(i);
        //     }
        // }

        double processingTime;

        // Depth N Best
        int n = 1, num_to_assign, max_coverage = 0;

        std::vector<std::vector<std::vector<int>>> AllAssignments, AllSequences;
        std::vector<std::vector<int>> CurrentAssignments, CurrentSequences;
        Sequences.resize(numDrones, std::vector<int>());

        // std::set<int> MissionIndicesToAssign;
        // for (int i = 0; i < numMissions; i++) {
        //     MissionIndicesToAssign.insert(i);
        // }

        std::vector<int> BestPredictedCoverage;

        // Keep looping until all missions have been assigned
        while(!MissionIndicesToAssign.empty()) {
            num_to_assign = (int) MissionIndicesToAssign.size() < numDrones * n ? MissionIndicesToAssign.size() : numDrones * n;

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
			// std::cerr << "RecurseAssignments: " << processingTime*1000.0 << " ms" << std::endl;
            T0 = std::chrono::steady_clock::now();
            // std::cout << "Number of assignments: " << AllAssignments.size() << std::endl;

            // Generate all ordered ways to fly drone through assignments
            for (std::vector<std::vector<int>> assignment : AllAssignments) {
                RecurseSequences(AllSequences, assignment, 0, numDrones);
            }

            processingTime = SecondsElapsed(T0, std::chrono::steady_clock::now());
			// std::cerr << "RecurseSequences: " << processingTime*1000.0 << " ms" << std::endl;
            T0 = std::chrono::steady_clock::now();
            int mission_time = -1, min_max_mission_time = -1;
            bool reset_mission_time = true;
            
            // std::cout << "Number of sequences: " << AllSequences.size() << std::endl;

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
                int coverage = GetCoverage(PredictedCoverage, MissionDurations, TA, SubregionMissions, newSequence, DroneStartPositions, ImagingReqs, numDrones);
                mission_time = *max_element(MissionDurations.begin(), MissionDurations.end());

                // If coverage better than ever seen, also calculate min max distance. 
                // In future, this could save on recalculating previous mission times, and instead calculate just the appended potential sequence time
                // if (coverage >= max_coverage) {
                //     distance = 0;
                //     for (int i = 0; i < numDrones; i++) {
                //         DroneInterface::Waypoint startPos = DroneStartPositions[i]; 
                //         if (!Sequences[i].empty()) {
                //             startPos = SubregionMissions[i].Waypoints[Sequences[i].size() - 1];
                //         }
                //         MissionDurations.push_back(duration starting from startPos);
                //     }
                //     distance = *max_element(MissionDurations.begin(), MissionDurations.end());
                // }
                // Update best coverage and min_max_distance
                if ((coverage > max_coverage) || ((coverage == max_coverage) && (reset_mission_time || mission_time < min_max_mission_time))) {
                    // std::cout << max_coverage << std::endl;
                    BestPredictedCoverage = PredictedCoverage;
                    max_coverage = coverage;
                    bestSequence = sequence;
                    min_max_mission_time = mission_time;
                    reset_mission_time = false;
                }
            }

            processingTime = SecondsElapsed(T0, std::chrono::steady_clock::now());
			// std::cerr << "GetCoverage: " << processingTime*1000.0 << " ms" << std::endl;
            T0 = std::chrono::steady_clock::now();

            // Remove missions that were assigned
            for (size_t i = 0; i < bestSequence.size(); i++) {
                extend(Sequences[i], bestSequence[i]);
                for (size_t j = 0; j < bestSequence[i].size(); j++) {
                    MissionIndicesToAssign.erase(bestSequence[i][j]);
                }
            }
        }
        std::cout << "BestPredictedCoverage: " << std::endl;
        for (int i = 0; i < BestPredictedCoverage.size(); i++) {
            std::cout << BestPredictedCoverage[i] << " ";
        }
        std::cout << std::endl;
    }
}
