//This widget lists connected vehicles and allows for manual user interaction and control of individual vehicle
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <cmath>

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "../EigenAliases.h"
#include "ReconUI.hpp"
#include "MyGui.hpp"
#include "EmbeddedIcons.hpp"
#include "../Modules/DJI-Drone-Interface/DroneManager.hpp"
#include "../Utilities.hpp"
#include "MapWidget.hpp"
#include "../Maps/DataTileProvider.hpp"
#include "../Maps/MapUtils.hpp"
#include "../ProgOptions.hpp"
#include "../Modules/Guidance/Guidance.hpp"

class VehicleControlWidget {
	struct vehicleState {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		std::string                      m_serial;
		float                            m_targetFPS = 1.0f;
		float                            m_FeedZoom = 1.0f; //1 = No crop, 2 = Crop to centeral 50% width and height
		bool                             m_showFeed = false;
		int                              m_callbackHandle = -1; //-1 if not registered
		
		//Texture for live video from drone (texture holds most recent frame)
		std::mutex                       m_Texmutex;
		bool                             m_TexValid = false;
		ImTextureID                      m_Tex;
		float                            m_TexWidth;
		float                            m_TexHeight;
		DroneInterface::Drone::TimePoint m_TexTimestamp;
		
		//State info for (manual) user control
		bool                             m_widgetControlEnabled = false;
		float                            m_targetHAGFeet = 0.0f; //feet
		float                            m_targetYawDeg = 0.0f;
		bool                             m_autoYawOnMove = true;
		float                            m_vehicleSpeedMPH = 15.0f;
		Eigen::Vector2d                  m_targetLatLon = Eigen::Vector2d(std::nan(""), std::nan("")); //Lat and Lon (radians) of target pos
		bool                             m_flyAtDeck = false; //When true fly at min safe altitude (overrides m_targetHAGFeet)
		bool                             m_hazard = false; //True if in hazardous state
		float                            m_RTL_HAG = 0.0f; //Should be set before starting RTL sequence
		bool                             m_LastCommandWasHover = false;
		bool                             m_AtTargetState = false;
		bool                             m_AtTargetYaw = false;
		bool                             m_AtTarget2DPosition = false;
		bool                             m_AtTargetHAG = false;
		DroneInterface::Drone::TimePoint m_TimeAtWhichWeReachedTargetState;
		
		vehicleState() = default;
		vehicleState(std::string const & Serial) : m_serial(Serial) { }
		~vehicleState() = default;
	};
	
	public:
		static VehicleControlWidget & Instance() { static VehicleControlWidget Widget; return Widget; }
		
		//Constructors and Destructors
		VehicleControlWidget() : Log(*(ReconUI::Instance().Log)) {
			m_controlThread = std::thread(&VehicleControlWidget::ControlThreadMain, this);
			m_IconTexture_Drone = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_QuadCopterTopView_Light_84x84[0], 84, 84);
			m_IconTexture_DroneWithArrow = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_QuadCopterTopViewWithArrow_Light_96x96[0], 96, 96);
			m_IconTexture_HighlightedDrone = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_QuadCopterTopView_LightHighlighted_84x84[0], 84, 84);
			m_IconTexture_HighlightedDroneWithArrow = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_QuadCopterTopViewWithArrow_LightHighlighted_96x96[0], 96, 96);
			m_messageToken = MapWidget::Instance().m_messageBoxOverlay.GetAvailableToken();
		}
		~VehicleControlWidget() {
			ImGuiApp::Instance().DeleteImage(m_IconTexture_Drone);
			ImGuiApp::Instance().DeleteImage(m_IconTexture_DroneWithArrow);
			ImGuiApp::Instance().DeleteImage(m_IconTexture_HighlightedDrone);
			ImGuiApp::Instance().DeleteImage(m_IconTexture_HighlightedDroneWithArrow);
		}
		
		//Should be stopped before DataTileProvider since the private thread sometimes uses that system
		inline void Stop(void) {
			m_controlThreadAbort = true;
			if (m_controlThread.joinable())
				m_controlThread.join();
		}
		
		//Public accessors for getting the state of the window
		float GetWidgetRecommendedHeight(void) { return RecommendedHeight; } //Typically equals ContentHeight, but changes smoothly with time for animation.
		
		//The overlay returns true of the map widget should process mouse input and false if we have already processed it and the widget should ignore it
		inline void Draw();
		inline bool DrawMapOverlay(Eigen::Vector2d const & CursorPos_ScreenSpace, Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
		
		//Public controls for drones - not for internal use in this class... these are for other modules to order drones around
		//These methods take care of instructing the guidance module to start/stop commanding drones when necessary
		inline void AllDronesStopAndHover(void);
		inline void AllDronesHitTheDeck(void);
		inline void StopCommandingDrone(std::string const & Serial); //Stop manual control of drone... does nothing if this module was already not commanding the drone
		inline void AllDronesReturnHomeAndLand(void); //Safe, coordinated RTL for each drone - get all drones on ground ASAP
		inline bool ReturnHomeSequenceInProgress(void) { return (m_RTL_State >= 0); }
		
		//Set/Clear Hazard lets the vehicle control widget decide on how to respond to a hazard instead of requiring the command module to make the call.
		//This is needed so that the vehicle control widget can react by stopping the vehicle only if it isn't already in a hazardous state. If already in a
		//hazardous state we don't want to stop the drone again since it would interfere with our ability to command the vehicle out of the hazardous state.
		inline void SetHazardCondition(std::string Serial, bool StopOnHazard, bool DodgeUpOnHazard);
		inline void ClearHazardCondition(std::string Serial);
		
	private:
		Journal & Log;
		ImTextureID m_IconTexture_Drone;
		ImTextureID m_IconTexture_DroneWithArrow;
		ImTextureID m_IconTexture_HighlightedDrone;
		ImTextureID m_IconTexture_HighlightedDroneWithArrow;
		
		std::thread       m_controlThread;
		std::atomic<bool> m_controlThreadAbort = false;
		
		std::recursive_mutex m_dronesAndStatesMutex; //Protects fields in this block
		std::vector<std::string> m_currentDroneSerials; //Updated in Draw()
		std::unordered_map<std::string, vehicleState> m_vehicleStates;
		int m_messageToken = -1;

		std::atomic<int> m_RTL_State = -1; //-1 = off, 0 = start RTL (goto RTL_HAG), 1 = 2D Go home, 2 = Drop target drone HAG, 3 = land target drone
		int m_RTL_NumFlyingDrones = -1; //Used to track RTL progress (only accessed in control thread)

		int m_indexOfDroneWithContextMenuOpen = -1;
		int m_indexOfDroneUnderDrag = -1;
		
		float ContentHeight; //Height of widget content from last draw pass
		float RecommendedHeight; //Recommended height for widget
		
		inline void DrawDroneInteractable(DroneInterface::Drone & drone, vehicleState & State, size_t DroneIndex);
		inline void DrawContextMenu(bool Open, DroneInterface::Drone & drone); //Call for every drone to render context menu for item (usually doesn't draw anything)
		
		static void AddOutdatedDataNote(std::string & Str, std::chrono::time_point<std::chrono::steady_clock> const & Timestamp) {
			double Age = SecondsElapsed(Timestamp, std::chrono::steady_clock::now());
			if (Age > 4.0)
				Str += " (Data is "s + std::to_string(int(std::round(Age))) + " seconds old)"s;
		}
		//Print warning on same line if data is old. Pass 0.0f for XStart to print right after last item drawn
		static void PrintAgeWarning(DroneInterface::Drone::TimePoint const & Timestamp, float XStart) {
			double Age = SecondsElapsed(Timestamp, std::chrono::steady_clock::now());
			if (Age > 4.0) {
				ImGui::SameLine(XStart);
				ImGui::PushFont(Fonts::NormalBold);
				ImGui::TextColored(ImVec4(1,0,0,1), " (Data is %d seconds old)", int(std::round(Age)));
				ImGui::PopFont();
			}
		}
		
		static inline bool IsDroneHovered(Eigen::Vector2d const & CursorPos_ScreenSpace, Eigen::Vector2d const & drone_ScreenSpace,
		                                  Eigen::Matrix2d const & R, float IconWidth_pixels);
		
		static inline void EnableClickAndDragControl(DroneInterface::Drone & Drone, vehicleState & State, bool FlyAtDeck = false);
		static inline void DroneCommand_Hover(DroneInterface::Drone & Drone, vehicleState & State);
		static inline void DroneCommand_LandNow(DroneInterface::Drone & Drone, vehicleState & State);
		static inline void DroneCommand_GoHomeAndLand(DroneInterface::Drone & Drone, vehicleState & State);
		
		inline void DrawVideoWindows(void);
		
		inline bool TryGetDroneTelemetry(DroneInterface::Drone * Drone, bool & IsFlying, Eigen::Vector3d & DronePos_LLA, Eigen::Vector3d & DronePos_ECEF,
		                                 Eigen::Vector3d & DroneVel_ENU, double & DroneYaw, double & DroneHAG, double MaxTelAge);
		inline void Update_AtTarget_Fields(Eigen::Vector3d const & DronePos_LLA, Eigen::Vector3d const & DronePos_ECEF,
		                                   Eigen::Vector3d const & DroneVel_ENU, double DroneYaw, double DroneHAG, vehicleState & State);
		//inline bool AtTargetState(Eigen::Vector3d const & DronePos_LLA, Eigen::Vector3d const & DronePos_ECEF,
		//                          Eigen::Vector3d const & DroneVel_ENU, double DroneYaw, double DroneHAG, vehicleState const & State);
		inline void ControlThreadMain(void);

		//Smart RTL Helpers
		inline void AbortRTLSequence(void);
		static inline int  NumberOfFlyingDrones(std::vector<DroneInterface::Drone *> const & Drones);
		static inline bool AllFlyingDronesReachedTargetHAG(std::vector<DroneInterface::Drone *> const & Drones,
		                                                   std::vector<vehicleState *> const & DroneStates);
		static inline bool AllFlyingDronesReachedTargetPositions2D(std::vector<DroneInterface::Drone *> const & Drones,
		                                                           std::vector<vehicleState *> const & DroneStates);
		static inline bool LowestFlyingDroneReachedTargetHAG(std::vector<DroneInterface::Drone *> const & Drones,
		                                                     std::vector<vehicleState *> const & DroneStates);
		static inline void AllFlyingDronesSetTargetPositionToTakeoffPoint2D(std::vector<DroneInterface::Drone *> const & Drones,
		                                                                    std::vector<vehicleState *> const & DroneStates);
		static inline void AllFlyingDronesStageVerticalyForLanding(std::vector<DroneInterface::Drone *> const & Drones,
		                                                           std::vector<vehicleState *> const & DroneStates);
		static inline void LandLowestFlyingDrone(std::vector<DroneInterface::Drone *> const & Drones,
		                                         std::vector<vehicleState *> const & DroneStates);
		
		inline void DrawMission_ManualControl(vehicleState * State, Eigen::Vector2d const & dronePos_ScreenSpace, ImDrawList * DrawList);
		inline void DrawMission_Waypoints(size_t DroneNum, DroneInterface::Drone * Drone, Eigen::Vector2d const & dronePos_ScreenSpace, ImDrawList * DrawList);
};

//Try to get position, velocity, yaw, and HAG. Returns true if all can be read and all items have age under MaxTelAge (in seconds)
inline bool VehicleControlWidget::TryGetDroneTelemetry(DroneInterface::Drone * Drone, bool & IsFlying, Eigen::Vector3d & DronePos_LLA,
                                                       Eigen::Vector3d & DronePos_ECEF, Eigen::Vector3d & DroneVel_ENU, double & DroneYaw,
                                                       double & DroneHAG, double MaxTelAge) {
	DroneInterface::Drone::TimePoint isFlyingTimestamp;
	if (! Drone->IsCurrentlyFlying(IsFlying, isFlyingTimestamp))
		return false;
	else if (SecondsElapsed(isFlyingTimestamp) > MaxTelAge)
		return false;
	
	DroneInterface::Drone::TimePoint posTimestamp;
	double Latitude, Longitude, Altitude;
	if (! Drone->GetPosition(Latitude, Longitude, Altitude, posTimestamp))
		return false;
	else if (SecondsElapsed(posTimestamp) > MaxTelAge)
		return false;
	DronePos_LLA << Latitude, Longitude, Altitude;
	DronePos_ECEF = LLA2ECEF(DronePos_LLA);
	
	DroneInterface::Drone::TimePoint velTimestamp;
	double V_North, V_East, V_Down;
	if (! Drone->GetVelocity(V_North, V_East, V_Down, velTimestamp))
		return false;
	else if (SecondsElapsed(velTimestamp) > MaxTelAge)
		return false;
	DroneVel_ENU << V_East, V_North, -1.0*V_Down;
	
	DroneInterface::Drone::TimePoint orientationTimestamp;
	double Yaw, Pitch, Roll;
	if (! Drone->GetOrientation(Yaw, Pitch, Roll, orientationTimestamp))
		return false;
	else if (SecondsElapsed(orientationTimestamp) > MaxTelAge)
		return false;
	DroneYaw = Yaw;
	
	DroneInterface::Drone::TimePoint HAGTimestamp;
	double HAG;
	if (! Drone->GetHAG(HAG, HAGTimestamp))
		return false;
	else if (SecondsElapsed(HAGTimestamp) > MaxTelAge)
		return false;
	DroneHAG = HAG;
	
	return true;
}

//Update the fields: m_AtTargetState, m_AtTargetYaw, m_AtTarget2DPosition, and m_AtTargetHAG of argument "State"
inline void VehicleControlWidget::Update_AtTarget_Fields(Eigen::Vector3d const & DronePos_LLA, Eigen::Vector3d const & DronePos_ECEF,
                                                         Eigen::Vector3d const & DroneVel_ENU, double DroneYaw, double DroneHAG, vehicleState & State) {
	double PI = 3.14159265358979;
	
	//Check 2D position (require position to be close and 2D speed to be small)
	Eigen::Vector3d targetPos_LLA(State.m_targetLatLon(0), State.m_targetLatLon(1), DronePos_LLA(2));
	Eigen::Vector3d targetPos_ECEF = LLA2ECEF(targetPos_LLA);
	double posDelta2D = (targetPos_ECEF - DronePos_ECEF).norm();
	double speed2D = std::sqrt(DroneVel_ENU(0)*DroneVel_ENU(0) + DroneVel_ENU(1)*DroneVel_ENU(1));
	State.m_AtTarget2DPosition = ((posDelta2D <= 1.0) && (speed2D < 3.0*0.44704));
	
	//Check Yaw
	double yawDelta = State.m_targetYawDeg*PI/180.0 - DroneYaw;
	yawDelta = fmod(yawDelta, 2*PI);
	if (yawDelta < 0.0)
		yawDelta += 2*PI;
	if (yawDelta > PI)
		yawDelta = 2*PI - yawDelta;
	//yawDelta is now the difference between the current and commanded yaw (without sign)
	State.m_AtTargetYaw = (yawDelta < 5.0*PI/180.0);

	//Check HAG
	double targetHAG = State.m_targetHAGFeet/3.280839895;
	State.m_AtTargetHAG = (fabs(DroneHAG - targetHAG) < 1.0);
	
	//Update m_AtTargetState
	bool atStateNow = State.m_AtTarget2DPosition && State.m_AtTargetYaw && State.m_AtTargetHAG;
	if ((! State.m_AtTargetState) && (atStateNow))
		State.m_TimeAtWhichWeReachedTargetState = std::chrono::steady_clock::now();
	State.m_AtTargetState = atStateNow;
}

//Returns true if a drone is approximately at it's target state (i.e. no additional movement is needed)
/*inline bool VehicleControlWidget::AtTargetState(Eigen::Vector3d const & DronePos_LLA, Eigen::Vector3d const & DronePos_ECEF,
                                                Eigen::Vector3d const & DroneVel_ENU, double DroneYaw, double DroneHAG, vehicleState const & State) {
	double PI = 3.14159265358979;
	
	//Check 2D position
	Eigen::Vector3d targetPos_LLA(State.m_targetLatLon(0), State.m_targetLatLon(1), DronePos_LLA(2));
	Eigen::Vector3d targetPos_ECEF = LLA2ECEF(targetPos_LLA);
	double posDelta2D = (targetPos_ECEF - DronePos_ECEF).norm();
	if (posDelta2D > 1.0)
		return false;
	
	//Check HAG
	double targetHAG = State.m_targetHAGFeet/3.280839895;
	if (fabs(DroneHAG - targetHAG) > 1.0)
		return false;
	
	//Check Yaw
	double yawDelta = State.m_targetYawDeg*PI/180.0 - DroneYaw;
	yawDelta = fmod(yawDelta, 2*PI);
	if (yawDelta < 0.0)
		yawDelta += 2*PI;
	if (yawDelta > PI)
		yawDelta = 2*PI - yawDelta;
	//yawDelta is now the difference between the current and commanded yaw (without sign)
	if (yawDelta > 5.0*PI/180.0)
		return false;
	
	//Check 2D Speed (should be small)
	double speed2D = std::sqrt(DroneVel_ENU(0)*DroneVel_ENU(0) + DroneVel_ENU(1)*DroneVel_ENU(1));
	if (speed2D > 3.0*0.44704)
		return false;
	
	return true;
}*/

inline void VehicleControlWidget::ControlThreadMain(void) {
	double approxLoopPeriod  = 0.10; //seconds
	
	//Wait for DataTileProvider to initialize
	while (Maps::DataTileProvider::Instance() == nullptr)
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	while (! m_controlThreadAbort) {
		double PI = 3.14159265358979;
		m_dronesAndStatesMutex.lock();
		
		//Get pointers to all valid drones and their corresponding states (touching states as needed)
		std::vector<DroneInterface::Drone *> drones;       drones.reserve(m_currentDroneSerials.size());
		std::vector<vehicleState *>          droneStates;  droneStates.reserve(m_currentDroneSerials.size());
		for (std::string serial : m_currentDroneSerials) {
			DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(serial);
			if (m_vehicleStates.count(serial) == 0U)
				m_vehicleStates[serial].m_serial = serial;
			if (drone != nullptr) {
				drones.push_back(drone);
				droneStates.push_back(&(m_vehicleStates.at(serial)));
			}
		}

		//Iterate over each valid drone and dispatch control commands as needed
		for (size_t n = 0U; n < drones.size(); n++) {
			DroneInterface::Drone * drone = drones[n];
			vehicleState * myState = droneStates[n];
			if (! myState->m_widgetControlEnabled)
				continue;

			//Get current telemetry
			bool IsFlying;
			Eigen::Vector3d dronePos_LLA;
			Eigen::Vector3d dronePos_ECEF;
			Eigen::Vector3d droneVel_ENU;
			double droneYaw; //Radians
			double droneHAG; //meters
			if (TryGetDroneTelemetry(drone, IsFlying, dronePos_LLA, dronePos_ECEF, droneVel_ENU, droneYaw, droneHAG, 4.0)) {
				if (IsFlying) {
					//If we are supposed to fly at the deck, try to update the target HAG
					if (myState->m_flyAtDeck) {
						double MSA;
						Eigen::Vector2d Position_NM = LatLonToNM(Eigen::Vector2d(dronePos_LLA(0), dronePos_LLA(1)));
						if (Maps::DataTileProvider::Instance()->TryGetData(Position_NM, Maps::DataLayer::MinSafeAltitude, MSA)) {
							if (std::isnan(MSA))
								std::cerr << "Can't fly at deck - no min safe altitude at location.\r\n";
							else {
								double groundAlt = dronePos_LLA(2) - droneHAG;
								double minSafeHAG = MSA - groundAlt;
								double targetHAG = minSafeHAG + 1.0; //Fly 1 meter above MSA to avoid triggering watchdog alarms
								myState->m_targetHAGFeet = targetHAG*3.280839895;
							}
						}
					}
					
					if (std::isnan(myState->m_targetLatLon(0)) || std::isnan(myState->m_targetLatLon(1))) {
						//This shouldn't happen, but in this case we have no target position. Hover
						if (! myState->m_LastCommandWasHover) {
							drone->Hover();
							myState->m_LastCommandWasHover = true;
						}
					}
					else {
						//Check if we are at our target state and update state fields accordingly
						Update_AtTarget_Fields(dronePos_LLA, dronePos_ECEF, droneVel_ENU, droneYaw, droneHAG, *myState);
						
						if (myState->m_AtTargetState && (SecondsElapsed(myState->m_TimeAtWhichWeReachedTargetState) > 2.0)) {
							//We have been at our target state for enough time - switch to hover
							if (! myState->m_LastCommandWasHover) {
								drone->Hover();
								myState->m_LastCommandWasHover = true;
							}
						}
						else {
							//We need to command the drone in virtualstick mode - Compute the East-North velocity we need
							double currentSpeed2D = std::sqrt(droneVel_ENU(0)*droneVel_ENU(0) + droneVel_ENU(1)*droneVel_ENU(1));
							Eigen::Vector3d targetPos_LLA(myState->m_targetLatLon(0), myState->m_targetLatLon(1), dronePos_LLA(2));
							Eigen::Vector3d targetPos_ECEF = LLA2ECEF(targetPos_LLA);
							Eigen::Vector3d delta_ECEF     = targetPos_ECEF - dronePos_ECEF;
							Eigen::Matrix3d C_ECEF_ENU     = latLon_2_C_ECEF_ENU(dronePos_LLA(0), dronePos_LLA(1));
							Eigen::Vector3d delta_ENU      = C_ECEF_ENU * delta_ECEF;
							Eigen::Vector2d v_EN(delta_ENU(0), delta_ENU(1));
							double distFromTarget          = v_EN.norm();
							distFromTarget = std::max(distFromTarget - currentSpeed2D*0.15, 0.0); //Account for command latancy
							v_EN.normalize();
							
							//We will set our East-North velocity to a multiple of v_EN... we just need to scale it appropriately
							double maxSpeed_mps = myState->m_vehicleSpeedMPH * 0.44704f;
							double maxDecel = 2.5; //m/s/s - measured on Inspire 1
							double stoppingDist = maxSpeed_mps*maxSpeed_mps / (2.0 * maxDecel);
							if (distFromTarget > stoppingDist)
								v_EN *= maxSpeed_mps;
							else {
								//Slowing down on final approach - we shape target speed towards 0 to mitigate oscillations
								double targetSpeed = std::sqrt(2.0 * maxDecel * distFromTarget);
								if (targetSpeed < 1.0)
									targetSpeed = targetSpeed * targetSpeed;
								v_EN *= targetSpeed;
							}
							
							//Execute using Mode A
							DroneInterface::VirtualStickCommand_ModeA command;
							command.Yaw     = myState->m_targetYawDeg*PI/180.0;
							command.V_North = v_EN(1);
							command.V_East  = v_EN(0);
							command.HAG     = myState->m_targetHAGFeet/3.280839895;
							command.timeout = std::max(2.0, 10.0*approxLoopPeriod);
							drone->IssueVirtualStickCommand(command);
							myState->m_LastCommandWasHover = false;
							
							//Execute using Mode B
							/*Eigen::Matrix2d C_EN_Vehicle;
							C_EN_Vehicle << cos(droneYaw), -1.0*sin(droneYaw),
									      sin(droneYaw),      cos(droneYaw);
							Eigen::Vector2d V_Target_Vehicle = C_EN_Vehicle * v_EN;
							
							DroneInterface::VirtualStickCommand_ModeB command;
							command.Yaw       = myState->m_targetYawDeg*PI/180.0;
							command.V_Forward = V_Target_Vehicle(1);
							command.V_Right   = V_Target_Vehicle(0);
							command.HAG       = myState->m_targetHAGFeet/3.280839895;
							command.timeout   = std::max(2.0, 10.0*approxLoopPeriod);
							drone->IssueVirtualStickCommand(command);
							myState->m_LastCommandWasHover = false;*/
						}
					}
				}
			}
			else
				//Don't do anything (like hover) since if the drone is on the ground we would be telling it to take off.
				std::cerr << "Warning: skipping control command because drone has invalid or old telemetry.\r\n";
		}

		//Update global RTL state machine
		if (m_RTL_State >= 0) {
			//We are undergoing a smart RTL
			if (NumberOfFlyingDrones(drones) <= 0) {
				m_RTL_State = -1; //RTL sequence is done
				MapWidget::Instance().m_messageBoxOverlay.RemoveMessage(m_messageToken);
			}

			if (m_RTL_State == 0) {
				//Goto RTL_HAG
				//Check whether we have reached target HAGS - when we do... update target positions and advance
				if (AllFlyingDronesReachedTargetHAG(drones, droneStates)) {
					AllFlyingDronesSetTargetPositionToTakeoffPoint2D(drones, droneStates);
					m_RTL_State = 1;
				}
			}
			else if (m_RTL_State == 1) {
				//Drones are heading home (2D)
				//Check if they have reached their target 2D positions. When they have... advance
				if (AllFlyingDronesReachedTargetPositions2D(drones, droneStates)) {
					AllFlyingDronesStageVerticalyForLanding(drones, droneStates);
					m_RTL_State = 2;
				}
			}
			else if (m_RTL_State == 2) {
				//Vertical Staging - bring all drones down
				//Check if lowest drone has reached staging height... When it has, advance
				if (LowestFlyingDroneReachedTargetHAG(drones, droneStates)) {
					m_RTL_NumFlyingDrones = NumberOfFlyingDrones(drones);
					LandLowestFlyingDrone(drones, droneStates);
					m_RTL_State = 3;
				}
			}
			else if (m_RTL_State == 3) {
				//Landing lowest drone
				//Check to see if it has landed. When it has, goto staging if we have more flying drones. Otherwise we are done
				int numFlyingDrones = NumberOfFlyingDrones(drones);
				if (numFlyingDrones < m_RTL_NumFlyingDrones) {
					AllFlyingDronesStageVerticalyForLanding(drones, droneStates);
					m_RTL_State = 2;
				}
			}

		}
		//int m_RTL_State = -1; //-1 = off, 0 = start RTL (goto RTL_HAG), 1 = 2D Go home, 2 = Drop target drone HAG, 3 = land target drone

		m_dronesAndStatesMutex.unlock();
		
		std::this_thread::sleep_for(std::chrono::milliseconds(int32_t(approxLoopPeriod*1000.0)));
	}
}

inline void VehicleControlWidget::AbortRTLSequence(void) {
	MapWidget::Instance().m_messageBoxOverlay.RemoveMessage(m_messageToken); //Remove RTL message

	//If no RTL is in progress, there is nothing to do
	if (m_RTL_State < 0)
		return;

	m_RTL_State = -1; //Stop and disable RTL state machine
	std::scoped_lock lock(m_dronesAndStatesMutex); //Lock vector of drone serials and states

	//Iterate through each connected drone and re-enable click & drag control based on current state (stop drone motion)
	for (size_t n = 0; n < m_currentDroneSerials.size(); n++) {
		DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(m_currentDroneSerials[n]);
		if (drone == nullptr)
			continue;
		
		//Skip drone if not currently flying
		bool isFlying = false;
		DroneInterface::Drone::TimePoint isFlyingTimestamp;
		if ((! drone->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp) > 5.0))
			isFlying = false;
		if (! isFlying)
			continue;

		EnableClickAndDragControl(*drone, m_vehicleStates.at(m_currentDroneSerials[n]), false);
	}
}

inline int VehicleControlWidget::NumberOfFlyingDrones(std::vector<DroneInterface::Drone *> const & Drones) {
	int numFlyingDrones = 0;
	for (size_t n = 0U; n < Drones.size(); n++) {
		bool isFlying;
		DroneInterface::Drone::TimePoint isFlyingTimestamp;
		if ((Drones[n]->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) && (SecondsElapsed(isFlyingTimestamp) <= 4.0)) {
			if (isFlying)
				numFlyingDrones++;
		}
	}
	return numFlyingDrones;
}

//Returns true if all flying drones have reached their target HAGs
inline bool VehicleControlWidget::AllFlyingDronesReachedTargetHAG(std::vector<DroneInterface::Drone *> const & Drones,
                                                                  std::vector<vehicleState *> const & DroneStates) {
	for (size_t n = 0U; n < Drones.size(); n++) {
		bool isFlying;
		DroneInterface::Drone::TimePoint isFlyingTimestamp;
		if ((! Drones[n]->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp) > 4.0))
			isFlying = false;

		if (isFlying) {
			if (! DroneStates[n]->m_AtTargetHAG)
				return false;
		}
	}
	return true;
}

//Returns true if all flying drones have reached their target 2D positions
inline bool VehicleControlWidget::AllFlyingDronesReachedTargetPositions2D(std::vector<DroneInterface::Drone *> const & Drones,
                                                                          std::vector<vehicleState *> const & DroneStates) {
	for (size_t n = 0U; n < Drones.size(); n++) {
		bool isFlying;
		DroneInterface::Drone::TimePoint isFlyingTimestamp;
		if ((! Drones[n]->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp) > 4.0))
			isFlying = false;

		if (isFlying) {
			if (! DroneStates[n]->m_AtTarget2DPosition)
				return false;
		}
	}
	return true;
}

//Returns true if the flying drone with lowest target HAG has reached it's target HAG and false otherwise
//Also returns false if there are no flying drones
inline bool VehicleControlWidget::LowestFlyingDroneReachedTargetHAG(std::vector<DroneInterface::Drone *> const & Drones,
	                                                               std::vector<vehicleState *> const & DroneStates) {
	size_t indexOfLowestFlyingDrone = 0U;
	int numFlyingDrones = 0;
	for (size_t n = 0U; n < Drones.size(); n++) {
		bool isFlying;
		DroneInterface::Drone::TimePoint isFlyingTimestamp;
		if ((! Drones[n]->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp) > 4.0))
			isFlying = false;

		if (isFlying) {
			//Initialize indexOfLowestFlyingDrone if this is the first flying drone
			if (numFlyingDrones == 0)
				indexOfLowestFlyingDrone = n;
			numFlyingDrones++;
			if (DroneStates[n]->m_targetHAGFeet < DroneStates[indexOfLowestFlyingDrone]->m_targetHAGFeet)
				indexOfLowestFlyingDrone = n;
		}
	}
	if (numFlyingDrones == 0)
		return false;
	else
		return DroneStates[indexOfLowestFlyingDrone]->m_AtTargetHAG;
}

inline void VehicleControlWidget::AllFlyingDronesSetTargetPositionToTakeoffPoint2D(std::vector<DroneInterface::Drone *> const & Drones,
                                                                                   std::vector<vehicleState *> const & DroneStates) {
	for (size_t n = 0U; n < Drones.size(); n++) {
		bool isFlying;
		DroneInterface::Drone::TimePoint isFlyingTimestamp;
		if ((! Drones[n]->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp) > 4.0))
			isFlying = false;

		if (isFlying) {
			double takeoffLat, takeoffLon, takeoffAlt;
			if (Drones[n]->GetTakeoffPosition(takeoffLat, takeoffLon, takeoffAlt))
				DroneStates[n]->m_targetLatLon << takeoffLat, takeoffLon;
			else
				std::cerr << "Error: Can't command flying drone home since it's takeoff location is unknown.\r\n";
		}
	}
}

//Bring all drones down in height, with the lowest drone at the desired height for starting the landing sequence
inline void VehicleControlWidget::AllFlyingDronesStageVerticalyForLanding(std::vector<DroneInterface::Drone *> const & Drones,
                                                                          std::vector<vehicleState *> const & DroneStates) {
	std::vector<vehicleState *> flyingDroneStates;
	float minTargetHAG_Feet = std::nanf("");
	for (size_t n = 0U; n < Drones.size(); n++) {
		bool isFlying;
		DroneInterface::Drone::TimePoint isFlyingTimestamp;
		if ((! Drones[n]->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp) > 4.0))
			isFlying = false;

		if (isFlying) {
			flyingDroneStates.push_back(DroneStates[n]);
			if (std::isnan(minTargetHAG_Feet))
				minTargetHAG_Feet = DroneStates[n]->m_targetHAGFeet;
			else
				minTargetHAG_Feet = std::min(minTargetHAG_Feet, DroneStates[n]->m_targetHAGFeet);
		}
	}
	for (vehicleState * state : flyingDroneStates)
		state->m_targetHAGFeet -= (minTargetHAG_Feet - 50.0f);
}

//Start landing the flying drone with the lowest target HAG
inline void VehicleControlWidget::LandLowestFlyingDrone(std::vector<DroneInterface::Drone *> const & Drones,
                                                        std::vector<vehicleState *> const & DroneStates) {
	size_t indexOfLowestFlyingDrone = 0U;
	int numFlyingDrones = 0;
	for (size_t n = 0U; n < Drones.size(); n++) {
		bool isFlying;
		DroneInterface::Drone::TimePoint isFlyingTimestamp;
		if ((! Drones[n]->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp) > 4.0))
			isFlying = false;

		if (isFlying) {
			//Initialize indexOfLowestFlyingDrone if this is the first flying drone
			if (numFlyingDrones == 0)
				indexOfLowestFlyingDrone = n;
			numFlyingDrones++;
			if (DroneStates[n]->m_targetHAGFeet < DroneStates[indexOfLowestFlyingDrone]->m_targetHAGFeet)
				indexOfLowestFlyingDrone = n;
		}
	}
	if (numFlyingDrones > 0)
		DroneCommand_LandNow(*(Drones[indexOfLowestFlyingDrone]), *(DroneStates[indexOfLowestFlyingDrone]));
	else
		std::cerr << "Warning in VehicleControlWidget::LandLowestFlyingDrone(): No flying drones... nothing to do.\r\n";
}

inline void VehicleControlWidget::DrawVideoWindows(void) {
	for (size_t n = 0; n < m_currentDroneSerials.size(); n++) {
		std::string serial = m_currentDroneSerials[n];
		DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(serial);
		if (drone == nullptr)
			continue;
		
		//Get reference to vehicle state object
		vehicleState & myState(m_vehicleStates.at(serial));
		
		ImExt::Window::Options WinOpts;
		WinOpts.Flags = WindowFlags::NoTitleBar | WindowFlags::NoResize | WindowFlags::NoMove | WindowFlags::NoCollapse | WindowFlags::NoSavedSettings |
		                WindowFlags::NoDocking;
		WinOpts.POpen = &(myState.m_showFeed);
		WinOpts.Size(ImVec2(640.0f, 360.0f + 1.5f*ImGui::GetFontSize()), Condition::Once);
		if (ImExt::Window window("Live Video##"s + serial, WinOpts); window.ShouldDrawContents()) {
			//Check to make sure we have a callback registered
			if (myState.m_callbackHandle == -1) {
				//Register a callback to update the imagery we are displaying
				myState.m_callbackHandle = drone->RegisterCallback([&myState](cv::Mat const & Frame, DroneInterface::Drone::TimePoint const & Timestamp) {
					//Delayed destruction of old texture (if there is one)
					myState.m_Texmutex.lock();
					if (myState.m_TexValid) {
						ImGuiApp::Instance().DeleteImageAsyncWithDelay(myState.m_Tex, 2.0);
						myState.m_TexValid = false;
					}
					myState.m_Texmutex.unlock();
					
					//TODO: Specify channel ordering for images coming from the drone interface module.
					if (! Frame.isContinuous()) {
						std::cerr << "Warning: Can't display non-continuus image matrix.\r\n";
						return;
					}
					if (Frame.type() == CV_8UC4) {
						cv::Mat Frame_RGBA(Frame.size(), CV_8UC4);
						cv::cvtColor(Frame, Frame_RGBA, cv::COLOR_BGRA2RGBA, 4);
						ImTextureID Tex = ImGuiApp::Instance().CreateImageRGBA8888(Frame_RGBA.ptr(), Frame_RGBA.cols, Frame_RGBA.rows);
						std::scoped_lock lock(myState.m_Texmutex);
						myState.m_Tex = Tex;
						myState.m_TexWidth = (float) Frame_RGBA.cols;
						myState.m_TexHeight = (float) Frame_RGBA.rows;
						myState.m_TexValid = true;
						myState.m_TexTimestamp = Timestamp;
					}
					else if (Frame.type() == CV_8UC3) {
						cv::Mat Frame_RGBA(Frame.size(), CV_8UC4);
						cv::cvtColor(Frame, Frame_RGBA, cv::COLOR_BGR2RGBA, 4);
						ImTextureID Tex = ImGuiApp::Instance().CreateImageRGBA8888(Frame_RGBA.ptr(), Frame_RGBA.cols, Frame_RGBA.rows);
						std::scoped_lock lock(myState.m_Texmutex);
						myState.m_Tex = Tex;
						myState.m_TexWidth = (float) Frame_RGBA.cols;
						myState.m_TexHeight = (float) Frame_RGBA.rows;
						myState.m_TexValid = true;
						myState.m_TexTimestamp = Timestamp;
					}
					else {
						std::cerr << "Warning: Can't display image... unsupported type.\r\n";
						return;
					}
				});
				std::cerr << "Drone Imagery Callback Registered.\r\n";
			}
			
			ImGui::Text("Video feed for drone %s.", serial.c_str());
			std::scoped_lock lock(myState.m_Texmutex);
			if (myState.m_TexValid) {
				ImGui::SameLine();
				double age = SecondsElapsed(myState.m_TexTimestamp, std::chrono::steady_clock::now());
				ImGui::Text(" Image Age: %.1f seconds.", age);
				
				float sliderWidth = ImGui::GetFontSize();
				ImVec2 UV0(0.5f - 0.5f/myState.m_FeedZoom, 0.5f - 0.5f/myState.m_FeedZoom);
				ImVec2 UV1(0.5f + 0.5f/myState.m_FeedZoom, 0.5f + 0.5f/myState.m_FeedZoom);
				
				ImVec2 regionAvail = ImGui::GetContentRegionAvail();
				float scale1 = (regionAvail.x - sliderWidth - 1.5f*ImGui::GetStyle().ItemSpacing.x) / myState.m_TexWidth;
				float scale2 = regionAvail.y / myState.m_TexHeight;
				float scale = std::min(scale1, scale2);
				ImVec2 TexSize(scale*myState.m_TexWidth, scale*myState.m_TexHeight);
				ImGui::Image(myState.m_Tex, TexSize, UV0, UV1, ImVec4(1,1,1,1), ImVec4(0,0,0,0));
				
				ImGui::SameLine();
				ImGui::VSliderFloat("##Zoom", ImVec2(sliderWidth, TexSize.y), &(myState.m_FeedZoom), 1.0f, 2.0f, "", ImGuiSliderFlags_NoRoundToFormat);
			}
		}
		else {
			//Make sure we don't have a callback registered
			if (myState.m_callbackHandle != -1) {
				drone->UnRegisterCallback(myState.m_callbackHandle);
				myState.m_callbackHandle = -1;
				std::cerr << "Drone Imagery Callback Un-Registered.\r\n";
			}
		}
	}
}

inline bool VehicleControlWidget::IsDroneHovered(Eigen::Vector2d const & CursorPos_ScreenSpace, Eigen::Vector2d const & drone_ScreenSpace,
                                           Eigen::Matrix2d const & R, float IconWidth_pixels) {
	Eigen::Vector2d v = CursorPos_ScreenSpace - drone_ScreenSpace;
	Eigen::Vector2d w = R.transpose()*v;
	return ((w(0) >= -0.5f*IconWidth_pixels) && (w(0) <= 0.5f*IconWidth_pixels) &&
	        (w(1) >= -0.5f*IconWidth_pixels) && (w(1) <= 0.5f*IconWidth_pixels));
}

inline void VehicleControlWidget::DrawMission_ManualControl(vehicleState * State, Eigen::Vector2d const & dronePos_ScreenSpace, ImDrawList * DrawList) {
	float droneIconWidth_pixels = ProgOptions::Instance()->DroneIconScale*96.0f;
	Eigen::Vector2d targetPos_NM = LatLonToNM(State->m_targetLatLon);
	Eigen::Vector2d targetPos_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(targetPos_NM);
	Eigen::Vector2d v = targetPos_ScreenSpace - dronePos_ScreenSpace;
	if (v.norm() > droneIconWidth_pixels) {
		//Only draw if the drone isn't essentially on top of it's target position
		int numSegments = (int) std::floor(v.norm() / (3.0 * ImGui::GetFontSize()));
		
		v.normalize();
		Eigen::Vector2d p0 = targetPos_ScreenSpace - (ImGui::GetFontSize() - 1.0)*v;
		DrawList->AddLine(dronePos_ScreenSpace, p0, IM_COL32(180,180,180,128), ImGui::GetFontSize()/3.0f);
		
		Eigen::Matrix2d R;
		R << 0, -1,
			1,  0;
		Eigen::Vector2d w = R*v;
		Eigen::Vector2d p1 = targetPos_ScreenSpace;
		Eigen::Vector2d p2 = p1 - ImGui::GetFontSize()*v + ImGui::GetFontSize()/2.0f*w;
		Eigen::Vector2d p3 = p1 - ImGui::GetFontSize()*v - ImGui::GetFontSize()/2.0f*w;
		DrawList->AddTriangleFilled(p1, p2, p3, IM_COL32(180,180,180,255));
		
		//For simple animation: t goes from 0 to 1 every 3 seconds and then repeats
		double t = FractionalPart(SecondsSinceT0Epoch(std::chrono::steady_clock::now())/3.0);
		p1 = p1 + t*3.0*ImGui::GetFontSize()*v;
		
		for (int n = 0; n < numSegments; n++) {
			p1 = p1 - 3.0*ImGui::GetFontSize()*v;
			p2 = p1 - ImGui::GetFontSize()*v + ImGui::GetFontSize()/2.0f*w;
			p3 = p1 - ImGui::GetFontSize()*v - ImGui::GetFontSize()/2.0f*w;
			if (n + 1 < numSegments)
				DrawList->AddTriangleFilled(p1, p2, p3, IM_COL32(180,180,180,255));
			else
				DrawList->AddTriangleFilled(p1, p2, p3, IM_COL32(180,180,180,t*255));
		}
	}
}

//Note that we don't have a good way to query what portion of a mission has been complete or what the active next waypoint is yet, so
//we don't draw a line connecting the drone to any waypoints since it would usually be wrong.
inline void VehicleControlWidget::DrawMission_Waypoints(size_t DroneNum, DroneInterface::Drone * Drone, Eigen::Vector2d const & dronePos_ScreenSpace,
                                                        ImDrawList * DrawList) {
	float droneIconWidth_pixels = ProgOptions::Instance()->DroneIconScale*96.0f;
	
	DroneInterface::WaypointMission mission;
	if (Drone->GetCurrentWaypointMission(mission)) {
		std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>> segments_SS;
		//Draw lines between consecutive waypoints
		for (size_t n = 1; n < mission.Waypoints.size(); n++) {
			/*Eigen::Vector2d p0;
			if (n == 0)
				p0 = dronePos_ScreenSpace;
			else {
				Eigen::Vector2d lastWaypoint_NM = LatLonToNM(Eigen::Vector2d(mission.Waypoints[n - 1U].Latitude, mission.Waypoints[n - 1U].Longitude));
				p0 = MapWidget::Instance().NormalizedMercatorToScreenCoords(lastWaypoint_NM);
			}*/
			Eigen::Vector2d lastWaypoint_NM = LatLonToNM(Eigen::Vector2d(mission.Waypoints[n - 1U].Latitude, mission.Waypoints[n - 1U].Longitude));
			Eigen::Vector2d p0 = MapWidget::Instance().NormalizedMercatorToScreenCoords(lastWaypoint_NM);
			
			Eigen::Vector2d waypoint_NM = LatLonToNM(Eigen::Vector2d(mission.Waypoints[n].Latitude, mission.Waypoints[n].Longitude));
			Eigen::Vector2d p1 = MapWidget::Instance().NormalizedMercatorToScreenCoords(waypoint_NM);
			segments_SS.push_back(std::make_tuple(p0, p1));
			DrawList->AddLine(p0, p1, IM_COL32(180,180,180,128), ImGui::GetFontSize()/5.0f);
		}
		
		//Draw arrow indicators in the middle of each segment
		for (auto const & segment : segments_SS) {
			Eigen::Vector2d p_center = 0.5*std::get<0>(segment) + 0.5*std::get<1>(segment);
			Eigen::Vector2d v = std::get<1>(segment) - std::get<0>(segment);
			if (v.norm() >= 1.45*ImGui::GetFontSize()) {
				v.normalize();
				Eigen::Matrix2d R;
				R << 0, -1,
					1,  0;
				Eigen::Vector2d w = R*v;
				Eigen::Vector2d p1 = p_center + ImGui::GetFontSize()/3.0f*v;
				Eigen::Vector2d p2 = p_center - ImGui::GetFontSize()/3.0f*v + ImGui::GetFontSize()/3.0f*w;
				Eigen::Vector2d p3 = p_center - ImGui::GetFontSize()/3.0f*v - ImGui::GetFontSize()/3.0f*w;
				DrawList->AddTriangleFilled(p1, p2, p3, IM_COL32(240,240,240,255));
			}
		}
		
		//Draw circles over each waypoint
		if (! segments_SS.empty())
			DrawList->AddCircleFilled(std::get<0>(segments_SS[0]), ImGui::GetFontSize()/3.0f, IM_COL32(240,240,240,255), 11);
		for (auto const & segment : segments_SS)
			DrawList->AddCircleFilled(std::get<1>(segment), ImGui::GetFontSize()/3.0f, IM_COL32(240,240,240,255), 11);
		
		//Draw an indicator identifying which drone the drawn mission is for
		if (! segments_SS.empty()) {
			std::string droneLabel = std::to_string((unsigned int) DroneNum + 1U);
			float fullLabelWidth = ImGui::CalcTextSize(droneLabel.c_str()).x + 1.3f*ImGui::GetFontSize();
			
			Eigen::Vector2d p0 = std::get<0>(segments_SS[0]);
			Eigen::Vector2d v  = std::get<1>(segments_SS[0]) - std::get<0>(segments_SS[0]);
			v.normalize();
			
			Eigen::Vector2d p_a = p0 - (0.5*fullLabelWidth + 0.5*ImGui::GetFontSize())*v; //Center of label
			Eigen::Vector2d c1 = p_a - Eigen::Vector2d(fullLabelWidth/2.0, ImGui::GetFontSize()/2.0);
			Eigen::Vector2d c2 = c1  + Eigen::Vector2d(ImGui::GetFontSize(), ImGui::GetFontSize());
			Eigen::Vector2d c3 = c2  + Eigen::Vector2d(0.3*ImGui::GetFontSize(), -1.0*ImGui::GetFontSize());
			Eigen::Vector2d c4 = p_a + Eigen::Vector2d(fullLabelWidth/2.0, ImGui::GetFontSize()/2.0);
			
			//Only draw the indicator if it won't overlap with the drone itself
			double distThresh = 0.7 * droneIconWidth_pixels;
			if (((c1 - dronePos_ScreenSpace).norm() > distThresh) && ((c2 - dronePos_ScreenSpace).norm() > distThresh) &&
			    ((c3 - dronePos_ScreenSpace).norm() > distThresh) && ((c4 - dronePos_ScreenSpace).norm() > distThresh)) {
				DrawList->AddImage(m_IconTexture_Drone, c1, c2);
				DrawList->AddText(c3, IM_COL32_WHITE, droneLabel.c_str());
			}
		}
	}
}

inline bool VehicleControlWidget::DrawMapOverlay(Eigen::Vector2d const & CursorPos_ScreenSpace, Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList,
	                                            bool CursorInBounds) {
	float droneIconWidth_pixels = ProgOptions::Instance()->DroneIconScale*96.0f;
	
	std::scoped_lock lock(m_dronesAndStatesMutex); //Lock vector of drone serials and states
	
	//Iterate through the connected drones - put each pointer in a vector - also grab state objects (touching if necessary)
	//Leave out drones for which GetDrone returns a nullptr so we don't have to constantly check for that later in this function
	std::vector<DroneInterface::Drone *> drones;       drones.reserve(m_currentDroneSerials.size());
	std::vector<vehicleState *>          droneStates;  droneStates.reserve(m_currentDroneSerials.size());
	for (std::string serial : m_currentDroneSerials) {
		DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(serial);
		if (m_vehicleStates.count(serial) == 0U)
			m_vehicleStates[serial].m_serial = serial;
		if (drone != nullptr) {
			drones.push_back(drone);
			droneStates.push_back(&(m_vehicleStates.at(serial)));
		}
	}
	
	//Detirmine if any drones are hovered
	int droneHoveredIndex = -1;
	for (size_t n = 0; n < drones.size(); n++) {
		DroneInterface::Drone::TimePoint Timestamp;
		double Latitude, Longitude, Altitude;
		double Yaw, Pitch, Roll;
		if (drones[n]->GetPosition(Latitude, Longitude, Altitude, Timestamp) && drones[n]->GetOrientation(Yaw, Pitch, Roll, Timestamp)) {
			Eigen::Vector2d dronePos_LatLon(Latitude, Longitude);
			Eigen::Vector2d dronePos_NM = LatLonToNM(dronePos_LatLon);
			Eigen::Vector2d dronePos_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(dronePos_NM);
			
			Eigen::Matrix2d R;
			R << cos(Yaw), -sin(Yaw),
			     sin(Yaw),  cos(Yaw);
			if (IsDroneHovered(CursorPos_ScreenSpace, dronePos_ScreenSpace, R, droneIconWidth_pixels)) {
				droneHoveredIndex = int(n);
				break;
			}
		}
	}
	
	//Mouse Interaction
	if (m_indexOfDroneUnderDrag >= 0) {
		if (m_indexOfDroneUnderDrag >= (int) drones.size())
			m_indexOfDroneUnderDrag = -1; //Cancel drag
		else {
			//We are currently dragging a valid drone
			DroneInterface::Drone * drone = drones[m_indexOfDroneUnderDrag];
			vehicleState * state = droneStates[m_indexOfDroneUnderDrag];
			
			DroneInterface::Drone::TimePoint Timestamp;
			double Latitude, Longitude, Altitude;
			if (drone->GetPosition(Latitude, Longitude, Altitude, Timestamp)) {
				if (ImGui::IsMouseDown(0)) {
					if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape)))
						m_indexOfDroneUnderDrag = -1; //Cancel drag
					else {
						//Draw an indicator for moving the drone to cursor position
						if (CursorInBounds) {
							Eigen::Vector2d dronePos_LatLon(Latitude, Longitude);
							Eigen::Vector2d dronePos_NM = LatLonToNM(dronePos_LatLon);
							Eigen::Vector2d dronePos_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(dronePos_NM);
							
							//All calculations in this block are in screen space
							Eigen::Vector2d v = dronePos_ScreenSpace - CursorPos_ScreenSpace;
							if (v.norm() > 5.0) {
								v.normalize();
								Eigen::Matrix2d R;
								R << 0, -1,
									1,  0;
								Eigen::Vector2d w = R*v;
								Eigen::Vector2d p1 = CursorPos_ScreenSpace;
								Eigen::Vector2d p2 = CursorPos_ScreenSpace + ImGui::GetFontSize()*v + ImGui::GetFontSize()/2.0f*w;
								Eigen::Vector2d p3 = CursorPos_ScreenSpace + ImGui::GetFontSize()*v - ImGui::GetFontSize()/2.0f*w;
								Eigen::Vector2d p4 = CursorPos_ScreenSpace + (ImGui::GetFontSize() - 1.0)*v;
								DrawList->AddLine(dronePos_ScreenSpace, p4, IM_COL32_WHITE, ImGui::GetFontSize()/3.0f);
								DrawList->AddTriangleFilled(p1, p2, p3, IM_COL32_WHITE);
							}
						}
					}
				}
				else {
					//Stop drag event - if in map widget bounds, this is a command to move the drone
					if (CursorInBounds) {
						if (m_RTL_State >= 0) {
							AbortRTLSequence();
							std::cerr << "RTL Sequence canceled due to manual drone move command.\r\n";
						}

						//Set the target location
						state->m_targetLatLon = NMToLatLon(CursorPos_NM);
						
						//If "auto yaw on move" is enabled, set the yaw
						if (state->m_autoYawOnMove) {
							//Compute vector in ENU from current position to target position
							Eigen::Vector3d targetPos_LLA(state->m_targetLatLon(0), state->m_targetLatLon(1), Altitude);
							Eigen::Vector3d curentPos_ECEF = LLA2ECEF(Eigen::Vector3d(Latitude, Longitude, Altitude));
							Eigen::Vector3d targetPos_ECEF = LLA2ECEF(targetPos_LLA);
							Eigen::Vector3d delta_ECEF     = targetPos_ECEF - curentPos_ECEF;
							Eigen::Matrix3d C_ECEF_ENU     = latLon_2_C_ECEF_ENU(Latitude, Longitude);
							Eigen::Vector3d delta_ENU      = C_ECEF_ENU * delta_ECEF;
							Eigen::Vector2d delta_EN(delta_ENU(0), delta_ENU(1));
							delta_EN.normalize();
							
							double PI = 3.14159265358979;
							double theta = PI/2.0 - std::atan2(delta_EN(1), delta_EN(0));
							if (theta < 0.0)
								theta += 2.0*PI;
							state->m_targetYawDeg = theta*180.0/PI;
						}
						
						//TODO: Maybe check along path to target and adjust height if needed based on MSA (if available) or issue warning.
						//Maybe leave this to the watchdog... we don't need redundant functionality and the added complexity here.
						//No - it's a good idea. We can't leave it to the watchdog since we might want to auto adjust the height to avoid
						//obstacles, and we don't want to expose that level of control to outside this class. We should do a look-ahead
						//in the control loop. Update... actually guarenteeing that we will clear obstructions is tricky... we might need
						//to slow down or stop horizontal movement. If we can't guarentee it... we should actually have the watchdog look ahead
						//and stop the drones if they are on track to a violation. We may still want to do this check here, but it would be
						//extra (to try and avoid a future violation)
					}
					m_indexOfDroneUnderDrag = -1; //Cancel drag
				}
			}
			else
				m_indexOfDroneUnderDrag = -1; //Cancel drag
		}
	}
	else if (droneHoveredIndex >= 0) {
		//DroneInterface::Drone * drone = drones[droneHoveredIndex]; //will only be identified as hovered if the drone is valid
		if (ImGui::IsMouseClicked(0) && (droneStates[droneHoveredIndex]->m_widgetControlEnabled))
			m_indexOfDroneUnderDrag = droneHoveredIndex;
		else if (ImGui::IsMouseClicked(1)) {
			m_indexOfDroneWithContextMenuOpen = droneHoveredIndex;
			ImGui::OpenPopup("Drone Control Popup");
		}
	}
	
	//Iterate through each connected drone and draw it's current control task (move-to indicator or waypoint mission)
	for (size_t n = 0; n < drones.size(); n++) {
		DroneInterface::Drone::TimePoint Timestamp;
		double Latitude, Longitude, Altitude;
		if (drones[n]->GetPosition(Latitude, Longitude, Altitude, Timestamp)) {
			Eigen::Vector2d dronePos_NM = LatLonToNM(Eigen::Vector2d(Latitude, Longitude));
			Eigen::Vector2d dronePos_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(dronePos_NM);
			
			//Draw an indication of the current objective or mission for the drone (target location or current waypoint mission)
			if (droneStates[n]->m_widgetControlEnabled && (! std::isnan(droneStates[n]->m_targetLatLon(0))) && (! std::isnan(droneStates[n]->m_targetLatLon(1))))
				DrawMission_ManualControl(droneStates[n], dronePos_SS, DrawList);
			else
				DrawMission_Waypoints(n, drones[n], dronePos_SS, DrawList); //Draw waypoint mission (if flying one)
		}
	}

	//Iterate through each connected drone and draw it's last-known position (with yaw info) on the map
	for (size_t n = 0; n < drones.size(); n++) {
		DroneInterface::Drone::TimePoint Timestamp;
		double Latitude, Longitude, Altitude;
		double Yaw, Pitch, Roll;
		bool IsFlying;
		if (drones[n]->GetPosition(Latitude, Longitude, Altitude, Timestamp) &&
		    drones[n]->GetOrientation(Yaw, Pitch, Roll, Timestamp) &&
		    drones[n]->IsCurrentlyFlying(IsFlying, Timestamp)) {
			Eigen::Vector2d dronePos_NM = LatLonToNM(Eigen::Vector2d(Latitude, Longitude));
			Eigen::Vector2d dronePos_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(dronePos_NM);
			
			Eigen::Vector2d p1_SS = dronePos_SS + Eigen::Vector2d(-1.0*droneIconWidth_pixels/2.0f, -1.0*droneIconWidth_pixels/2.0f);
			Eigen::Vector2d p2_SS = dronePos_SS + Eigen::Vector2d(     droneIconWidth_pixels/2.0f, -1.0*droneIconWidth_pixels/2.0f);
			Eigen::Vector2d p3_SS = dronePos_SS + Eigen::Vector2d(     droneIconWidth_pixels/2.0f,      droneIconWidth_pixels/2.0f);
			Eigen::Vector2d p4_SS = dronePos_SS + Eigen::Vector2d(-1.0*droneIconWidth_pixels/2.0f,      droneIconWidth_pixels/2.0f);
			
			Eigen::Matrix2d R;
			R << cos(Yaw), -sin(Yaw),
			     sin(Yaw),  cos(Yaw);
			p1_SS = R*(p1_SS - dronePos_SS) + dronePos_SS;
			p2_SS = R*(p2_SS - dronePos_SS) + dronePos_SS;
			p3_SS = R*(p3_SS - dronePos_SS) + dronePos_SS;
			p4_SS = R*(p4_SS - dronePos_SS) + dronePos_SS;
			
			if (int(n) == droneHoveredIndex)
				DrawList->AddImageQuad(m_IconTexture_HighlightedDroneWithArrow, p1_SS, p2_SS, p3_SS, p4_SS);
			else
				DrawList->AddImageQuad(m_IconTexture_DroneWithArrow, p1_SS, p2_SS, p3_SS, p4_SS);
			
			//Draw drone label
			std::string droneLabel = std::to_string((unsigned int) n + 1U);
			bool hazardIcon  = droneStates[n]->m_hazard;
			std::string controlIcon;
			if (IsFlying) {
				if (droneStates[n]->m_widgetControlEnabled)
					controlIcon = u8"\uf8cc"; //Mouse Icon
				else if (Guidance::GuidanceEngine::Instance().IsCommandingDrone(drones[n]->GetDroneSerial()))
					controlIcon = u8"\uf04b"; //Guidance Module Control Icon
				else
					controlIcon = u8"\uf11b"; //Controller Icon
			}
			
			Eigen::Vector2d p5_ScreenSpace = 0.5*(p3_SS + p4_SS);
			Eigen::Vector2d v1 = R*Eigen::Vector2d(0.0, 1.0);
			if (hazardIcon && (! controlIcon.empty())) {
				//Use two lines
				std::string line1 = droneLabel + " "s + controlIcon;
				std::string line2 = u8" \uf071";
				
				double lineHeight = ImGui::GetTextLineHeightWithSpacing();
				Eigen::Vector2d p6_ScreenSpace = p5_ScreenSpace + v1*1.3*lineHeight; //center point of label
				
				Eigen::Vector2d p7_ScreenSpace = p6_ScreenSpace - Eigen::Vector2d(0.0, 0.5*lineHeight);
				MyGui::AddText(DrawList, p7_ScreenSpace, IM_COL32_WHITE, line1.c_str(), NULL, true, true);
				
				Eigen::Vector2d p8_ScreenSpace = p6_ScreenSpace + Eigen::Vector2d(0.0, 0.5*lineHeight);
				MyGui::AddText(DrawList, p8_ScreenSpace, IM_COL32_WHITE, line2.c_str(), NULL, true, true);
			}
			else {
				//Use one line
				if (hazardIcon)
					droneLabel += u8" \uf071";
				if (! controlIcon.empty())
					droneLabel += " "s + controlIcon;
				
				Eigen::Vector2d p6_ScreenSpace = p5_ScreenSpace + v1*double(ImGui::GetFontSize()); //Center point of label text
				MyGui::AddText(DrawList, p6_ScreenSpace, IM_COL32_WHITE, droneLabel.c_str(), NULL, true, true);
			}
		}
	}
	
	{
		//Draw Drone Control Popup
		ImExt::Style styleSitter(StyleVar::WindowPadding, ImVec2(4.0f, 4.0f));
		if (ImGui::BeginPopup("Drone Control Popup", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			if ((m_indexOfDroneWithContextMenuOpen < 0) || (m_indexOfDroneWithContextMenuOpen >= int(droneStates.size()))) {
				m_indexOfDroneWithContextMenuOpen = -1;
				ImGui::CloseCurrentPopup();
			}
			DroneInterface::Drone * drone = drones[m_indexOfDroneWithContextMenuOpen];
			std::string droneSerial = drone->GetDroneSerial();
			vehicleState * state = droneStates[m_indexOfDroneWithContextMenuOpen];
			
			if (state->m_widgetControlEnabled) {
				//Popup for drone under manual control
				ImGui::BeginChild("Drag Control Area", ImVec2(17.0f*ImGui::GetFontSize(), 0), true);
				float yStart = ImGui::GetCursorPosY();
				
				ImGui::TextUnformatted("Commanded State");
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
				ImGui::TextUnformatted("Height Above Ground");
				ImGuiStyle & style(ImGui::GetStyle());
				float checkWidgetWidth = ImGui::CalcTextSize("Hit Deck ").x + 2.0f*ImGui::GetFontSize() + 2.0f*style.FramePadding.x + 3.0f*style.ItemSpacing.x;
				ImGui::SameLine(ImGui::GetContentRegionMax().x - checkWidgetWidth);
				ImGui::TextUnformatted("Hit Deck ");
				ImGui::SameLine();
				ImGui::Checkbox("##HitDeck", &(state->m_flyAtDeck));
				ImGui::SameLine();
				ImGui::TextDisabled(u8"\uf059");
				if (ImGui::IsItemHovered()) {
					ImExt::Style tooltipStyle(StyleVar::WindowPadding, ImVec2(4.0f, 4.0f));
					ImGui::BeginTooltip();
					ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
					ImGui::TextUnformatted("If checked, the vehicle altitude will be adjusted to the Min Safe Altitude for it's current location. "
					                       "If the vehicle is moved with this box checked it's altitude will track the Min Safe Altitude function "
					                       "as the vehicle moves. If there is no min safe altitude set for the drones location, the drone will "
					                       "hold the last set altitude.");
					ImGui::PopTextWrapPos();
					ImGui::EndTooltip();
				}
				ImGui::SetNextItemWidth(-1.0f);
				if (state->m_flyAtDeck) {
					ImExt::Style styleSitter;
					Math::Vector4 inactiveWidgetColor = ImGui::GetStyle().Colors[ImGuiCol_Header];
					styleSitter(StyleCol::Button,        inactiveWidgetColor);
					styleSitter(StyleCol::ButtonHovered, inactiveWidgetColor);
					styleSitter(StyleCol::ButtonActive,  inactiveWidgetColor);
					std::string buttonText = std::to_string((int) std::round(state->m_targetHAGFeet)) + " (feet)"s;
					ImGui::Button(buttonText.c_str(), ImVec2(ImGui::GetContentRegionAvail().x, 0));
				}
				else
					ImGui::DragFloat("##HeightDrag", &(state->m_targetHAGFeet), 0.25f, 0.0f, 400.0f, "%.0f (feet)");
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
				
				ImGui::TextUnformatted("Movement Speed");
				ImGui::SameLine(ImGui::GetContentRegionMax().x - ImGui::CalcTextSize(" Max ").x - 3.0f*style.FramePadding.x);
				if (ImGui::SmallButton(" Max "))
					state->m_vehicleSpeedMPH = 33.6f;
				ImGui::SetNextItemWidth(-1.0f);
				ImGui::DragFloat("##SpeedDrag", &(state->m_vehicleSpeedMPH), 0.05f, 1.0f, 33.6f, "%.1f (mph)");
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
				
				ImGui::TextUnformatted("Yaw");
				ImGui::SetNextItemWidth(-1.0f);
				ImGui::DragFloat("##YawDrag", &(state->m_targetYawDeg), 0.25f, 0.0f, 359.0f, "%.0f (degrees)");
				ImGui::Spacing();
				ImGui::TextUnformatted("Auto-adjust Yaw on move: ");
				ImGui::SameLine();
				ImGui::Checkbox("##AutoYawCheckbox", &(state->m_autoYawOnMove));
				ImGui::SameLine();
				ImGui::Dummy(ImVec2(0.4f*ImGui::GetFontSize(), 1.0f));
				ImGui::SameLine();
				ImGui::TextDisabled(u8"\uf059");
				if (ImGui::IsItemHovered()) {
					ImExt::Style tooltipStyle(StyleVar::WindowPadding, ImVec2(4.0f, 4.0f));
					ImGui::BeginTooltip();
					ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
					ImGui::TextUnformatted("If checked, the vehicle will yaw to point in the direction of motion each time it is commanded to move.");
					ImGui::PopTextWrapPos();
					ImGui::EndTooltip();
				}
				ImGui::Spacing(); ImGui::Spacing();
				
				{
					ImExt::Style styleSitter(StyleVar::FramePadding, ImVec2(4.0f, 4.0f));
					if (ImGui::Button(" \uf015 Go Home & Land "))
						ImGui::OpenPopup("Home Button Popup");
					ImGui::SameLine();
					if (ImGui::Button(" \uf5af Land Now", ImVec2(ImGui::GetContentRegionAvail().x, 0)))
						ImGui::OpenPopup("Land Now Popup");
					if (ImGui::Button("Hover in Place ( \uf8cc \uf061 \uf11b )", ImVec2(ImGui::GetContentRegionAvail().x, 0)))
						ImGui::OpenPopup("Transfer To Controller Popup");
				}
				
				bool commandGiven = false;
				if (ImGui::BeginPopup("Home Button Popup", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
					if (ImGui::MenuItem("Confirm Command", NULL, false, true)) {
						DroneCommand_GoHomeAndLand(*drone, *state);
						commandGiven = true;
					}
					ImGui::EndPopup();
				}
				if (ImGui::BeginPopup("Land Now Popup", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
					if (ImGui::MenuItem("Confirm Command", NULL, false, true)) {
						DroneCommand_LandNow(*drone, *state);
						commandGiven = true;
					}
					ImGui::EndPopup();
				}
				if (ImGui::BeginPopup("Transfer To Controller Popup", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
					if (ImGui::MenuItem("Confirm Command", NULL, false, true)) {
						DroneCommand_Hover(*drone, *state);
						commandGiven = true;
					}
					ImGui::EndPopup();
				}
				if (commandGiven)
					ImGui::CloseCurrentPopup();
				
				/*if (ImGui::Button("Stop Manual Control & Hover", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
					DroneCommand_Hover(*drone, *state);
					m_indexOfDroneWithContextMenuOpen = -1;
					ImGui::CloseCurrentPopup();
				}*/
				
				float height = ImGui::GetCursorPosY() - yStart;
				ImGui::EndChild();
				
				ImGui::SameLine();
				
				//Make a fancy yaw widget
				ImU32 composCol = IM_COL32(255,100,100,255);
				Eigen::Vector2d startPos_ScreenSpace = ImGui::GetCursorScreenPos();
				float widgetSize = height + 2.0f*ImGui::GetStyle().FramePadding.y + ImGui::GetFontSize();
				ImGui::InvisibleButton("Fancy Yaw Widget", ImVec2(widgetSize, widgetSize));
				ImDrawList * draw_list = ImGui::GetWindowDrawList();
				Eigen::Vector2d center(startPos_ScreenSpace(0) + widgetSize/2.0f, startPos_ScreenSpace(1) + widgetSize/2.0f);
				float radius = 0.45f*widgetSize - ImGui::GetFontSize()/2.0f;
				draw_list->AddCircle(center, radius, composCol, 80, ImGui::GetFontSize());
				
				//Draw North marker
				Eigen::Vector2d NCenter = center - Eigen::Vector2d(0.0, radius - 0.55*ImGui::GetFontSize());
				draw_list->AddCircleFilled(NCenter, ImGui::GetFontSize(), composCol, 20);
				MyGui::AddText(draw_list, NCenter, IM_COL32_BLACK, "N", NULL, true, true);
				
				//Draw South marker
				Eigen::Vector2d SCenter = center + Eigen::Vector2d(0.0, radius - 0.55*ImGui::GetFontSize());
				draw_list->AddCircleFilled(SCenter, ImGui::GetFontSize(), composCol, 20);
				MyGui::AddText(draw_list, SCenter, IM_COL32_BLACK, "S", NULL, true, true);
				
				//Draw East marker
				Eigen::Vector2d ECenter = center + Eigen::Vector2d(radius - 0.55*ImGui::GetFontSize(), 0.0);
				draw_list->AddCircleFilled(ECenter, ImGui::GetFontSize(), composCol, 20);
				MyGui::AddText(draw_list, ECenter, IM_COL32_BLACK, "E", NULL, true, true);
				
				//Draw West marker
				Eigen::Vector2d WCenter = center - Eigen::Vector2d(radius - 0.55*ImGui::GetFontSize(), 0.0);
				draw_list->AddCircleFilled(WCenter, ImGui::GetFontSize(), composCol, 20);
				MyGui::AddText(draw_list, WCenter, IM_COL32_BLACK, "W", NULL, true, true);
				
				//Draw drone icon
				float iconWidth = std::min(0.75f*radius, 2.0f*droneIconWidth_pixels);
				Eigen::Vector2d p1_SS = center + Eigen::Vector2d(-1.0*iconWidth/2.0f, -1.0*iconWidth/2.0f);
				Eigen::Vector2d p2_SS = center + Eigen::Vector2d(     iconWidth/2.0f, -1.0*iconWidth/2.0f);
				Eigen::Vector2d p3_SS = center + Eigen::Vector2d(     iconWidth/2.0f,      iconWidth/2.0f);
				Eigen::Vector2d p4_SS = center + Eigen::Vector2d(-1.0*iconWidth/2.0f,      iconWidth/2.0f);
				Eigen::Matrix2d R;
				double PI = 3.14159265358979;
				double yaw = state->m_targetYawDeg*PI/180.0;
				R << cos(yaw), -sin(yaw),
					sin(yaw),  cos(yaw);
				p1_SS = R*(p1_SS - center) + center;
				p2_SS = R*(p2_SS - center) + center;
				p3_SS = R*(p3_SS - center) + center;
				p4_SS = R*(p4_SS - center) + center;
				draw_list->AddImageQuad(m_IconTexture_DroneWithArrow, p1_SS, p2_SS, p3_SS, p4_SS);
				
				//If the mouse is over the indicator, allow setting yaw through a click
				Eigen::Vector2d delta = CursorPos_ScreenSpace - center;
				if (delta.norm() <= radius) {
					if (ImGui::IsItemActive()) {
						state->m_targetYawDeg = float(std::atan2(delta(1), delta(0))*180.0/PI + 90.0);
						if (state->m_targetYawDeg < 0.0f)
							state->m_targetYawDeg += 360.0f;
					}
				}
			}
			else {
				//Popup for drone not under manual control
				float labelMargin = 1.5f*ImGui::GetFontSize();
				bool isFlying = false;
				DroneInterface::Drone::TimePoint isFlyingTimestamp;
				if ((! drone->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp, std::chrono::steady_clock::now()) > 5.0))
					isFlying = false;
				
				if (! isFlying) {
					if (MyGui::BeginMenu(u8"\uf5b0", labelMargin, "Take off & Hover")) {
						if (ImGui::MenuItem("Confirm Command", NULL, false, true)) {
							DroneCommand_Hover(*drone, *state);
							EnableClickAndDragControl(*drone, *state, false);
							state->m_targetHAGFeet = 50.0;
						}
						ImGui::EndMenu();
					}
					if ((Guidance::GuidanceEngine::Instance().IsRunning()) &&
					    (! Guidance::GuidanceEngine::Instance().IsCommandingDrone(droneSerial))) {
						if (MyGui::BeginMenu(u8"\uf04b", labelMargin, "Add to Current Mission")) {
							if (ImGui::MenuItem("Confirm Command", NULL, false, true))
								Guidance::GuidanceEngine::Instance().AddDroneToMission(droneSerial);
							ImGui::EndMenu();
						}
					}
				}
				else {
					if (MyGui::BeginMenu(u8"\uf04c", labelMargin, "Hover In Place")) {
						if (ImGui::MenuItem("Confirm Command", NULL, false, true))
							DroneCommand_Hover(*drone, *state);
						ImGui::EndMenu();
					}
					if (MyGui::BeginMenu(u8"\uf5af", labelMargin, "Land Now")) {
						if (ImGui::MenuItem("Confirm Command", NULL, false, true))
							DroneCommand_LandNow(*drone, *state);
						ImGui::EndMenu();
					}
					if (MyGui::BeginMenu(u8"\uf015", labelMargin, "Go Home and Land")) {
						if (ImGui::MenuItem("Confirm Command", NULL, false, true))
							DroneCommand_GoHomeAndLand(*drone, *state);
						ImGui::EndMenu();
					}
					if (MyGui::BeginMenu(u8"\uf8cc", labelMargin, "Click & Drag Control")) {
						if (ImGui::MenuItem("Enable", NULL, false, true))
							EnableClickAndDragControl(*drone, *state);
						ImGui::EndMenu();
					}
					if ((Guidance::GuidanceEngine::Instance().IsRunning()) &&
					    (! Guidance::GuidanceEngine::Instance().IsCommandingDrone(droneSerial))) {
						if (MyGui::BeginMenu(u8"\uf04b", labelMargin, "Add to Current Mission")) {
							if (ImGui::MenuItem("Confirm Command", NULL, false, true))
								Guidance::GuidanceEngine::Instance().AddDroneToMission(droneSerial);
							ImGui::EndMenu();
						}
					}
				}
				//TODO - this should be removed once waypoint mission functionality is tested
				if (ImGui::BeginMenu("Start Waypoint Mission")) {
					if (ImGui::MenuItem("Short Mission - Point 2 Point")) {
						state->m_widgetControlEnabled = false;
						drone->StartSampleWaypointMission(10, false, false, Eigen::Vector2d(5.0, 50.0), 30.0);
					}
					if (ImGui::MenuItem("Short Mission - Round Corners")) {
						state->m_widgetControlEnabled = false;
						drone->StartSampleWaypointMission(10, true, false, Eigen::Vector2d(5.0, 50.0), 30.0);
					}
					ImGui::EndMenu();
				}
			}
			
			ImGui::EndPopup();
		}
	}
	
	//We need to return true if the map widget still needs to process mouse inputs (i.e. if we haven't stolen them)
	return (! ImGui::IsPopupOpen("Drone Control Popup")) && (m_indexOfDroneUnderDrag < 0);
}

inline void VehicleControlWidget::AllDronesStopAndHover(void) {
	//Abort any mission the guidance module may be running since this is an All-drone command
	Guidance::GuidanceEngine::Instance().AbortMission();

	AbortRTLSequence(); //Abort any RTL that may be in progress

	//Iterate through each connected drone and execute the command
	std::scoped_lock lock(m_dronesAndStatesMutex); //Lock vector of drone serials and states
	for (size_t n = 0; n < m_currentDroneSerials.size(); n++) {
		DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(m_currentDroneSerials[n]);
		if (drone == nullptr)
			continue;
		
		//Skip drone if not currently flying
		bool isFlying = false;
		DroneInterface::Drone::TimePoint isFlyingTimestamp;
		if ((! drone->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp) > 5.0))
			isFlying = false;
		if (! isFlying)
			continue;

		EnableClickAndDragControl(*drone, m_vehicleStates.at(m_currentDroneSerials[n]), false);
	}
}

inline void VehicleControlWidget::AllDronesHitTheDeck(void) {
	//Abort any mission the guidance module may be running since this is an All-drone command
	Guidance::GuidanceEngine::Instance().AbortMission();

	AbortRTLSequence(); //Abort any RTL that may be in progress

	//Iterate through each connected drone and execute the command
	std::scoped_lock lock(m_dronesAndStatesMutex); //Lock vector of drone serials and states
	for (size_t n = 0; n < m_currentDroneSerials.size(); n++) {
		DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(m_currentDroneSerials[n]);
		if (drone == nullptr)
			continue;

		//Skip drone if not currently flying
		bool isFlying = false;
		DroneInterface::Drone::TimePoint isFlyingTimestamp;
		if ((! drone->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp) > 5.0))
			isFlying = false;
		if (! isFlying)
			continue;

		EnableClickAndDragControl(*drone, m_vehicleStates.at(m_currentDroneSerials[n]), true);
	}
}

//Stop manual control of drone... does nothing if this module was already not commanding the drone
inline void VehicleControlWidget::StopCommandingDrone(std::string const & Serial) {
	std::scoped_lock lock(m_dronesAndStatesMutex); //Lock vector of drone serials and states
	if (m_vehicleStates.count(Serial) > 0U)
		m_vehicleStates.at(Serial).m_widgetControlEnabled = false;
}

//Safe, coordinated RTL for each drone - get all drones on ground ASAP
inline void VehicleControlWidget::AllDronesReturnHomeAndLand(void) {
	//Step 1 - Stop horizontal motion for all drones.
	//Step 2 - Get all drones positions and Alts.
	//Step 3 - Get max of MSA on all lines from each drone to it's respective home point
	//Step 4 - Assign staggered RTL HAGs that keeps all drones above max MSA
	//Step 5 - Start RTL sequence for each drone, with corresponding RTL HAG
	
	//Abort any mission the guidance module may be running since this is an All-drone command
	Guidance::GuidanceEngine::Instance().AbortMission();

	std::scoped_lock lock(m_dronesAndStatesMutex); //Lock vector of drone serials and states
	std::cerr << "RTL Sequence initiated.\r\n";

	//Iterate through the connected (flying) drones - put each pointer in a vector - also grab state objects (touching if necessary)
	//Leave out drones for which GetDrone returns a nullptr so we don't have to constantly check for that later in this function
	std::vector<DroneInterface::Drone *> drones;             drones.reserve(m_currentDroneSerials.size());
	std::vector<vehicleState *>          droneStates;        droneStates.reserve(m_currentDroneSerials.size());
	std::vector<double>                  droneAlts;          droneAlts.reserve(m_currentDroneSerials.size());
	std::vector<double>                  droneHomeAlts;      droneHomeAlts.reserve(m_currentDroneSerials.size());
	std::vector<double>                  droneBattLevels;    droneBattLevels.reserve(m_currentDroneSerials.size());
	for (std::string serial : m_currentDroneSerials) {
		DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(serial);
		if (m_vehicleStates.count(serial) == 0U)
			m_vehicleStates[serial].m_serial = serial;
		if (drone != nullptr) {
			bool isFlying = false;
			DroneInterface::Drone::TimePoint isFlyingTimestamp;
			if ((! drone->IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp) > 5.0))
				isFlying = false;

			if (isFlying) {
				double HAG;
				DroneInterface::Drone::TimePoint HAGTimestamp;
				if ((! drone->GetHAG(HAG, HAGTimestamp)) || (SecondsElapsed(HAGTimestamp) > 5.0))
					HAG = std::nan("");

				double BattLevel;
				DroneInterface::Drone::TimePoint BattTimestamp;
				if ((! drone->GetVehicleBatteryLevel(BattLevel, BattTimestamp)) || (SecondsElapsed(BattTimestamp) > 5.0))
					BattLevel = 0.0; //Default if unknown or old battery data

				double Latitude, Longitude, Altitude;
				DroneInterface::Drone::TimePoint PositionTimestamp;
				if ((! drone->GetPosition(Latitude, Longitude, Altitude, PositionTimestamp)) || (SecondsElapsed(PositionTimestamp) > 5.0))
					Altitude = std::nan("");

				if ((! std::isnan(Altitude)) && (! std::isnan(HAG))) {
					drones.push_back(drone);
					droneStates.push_back(&(m_vehicleStates.at(serial)));
					droneAlts.push_back(Altitude);
					droneHomeAlts.push_back(Altitude - HAG);
					droneBattLevels.push_back(BattLevel);
				}
				else
					std::cerr << "Warning in AllDronesReturnHomeAndLand(): Skipping drone with unknown position or HAG.\r\n";
			}
		}
	}

	if (drones.empty()) {
		std::cerr << "Warning in AllDronesReturnHomeAndLand(): No connected, flying drone. Nothing to do.\r\n";
		return;
	}

	//Step 1 - Stop horizontal motion for all drones.
	for (size_t n = 0U; n < drones.size(); n++)
		EnableClickAndDragControl(*(drones[n]), *(droneStates[n]), false);

	//Step 2 - Get the max of all MSA values on all lines between drones and their respective home points
	double maxMSA = std::nan("");
	for (size_t n = 0U; n < drones.size(); n++) {
		double takeoffLat, takeoffLon, takeoffAlt;
		Eigen::Vector2d takeoffLatLon;
		if (! drones[n]->GetTakeoffPosition(takeoffLat, takeoffLon, takeoffAlt)) {
			std::cerr << "Dropping drone from alt detirmination because it has no takeoff location.\r\n";
			continue;
		}
		else
			takeoffLatLon << takeoffLat, takeoffLon;

		//Get endpoints of line between current commanded position and home position (in NM)
		Eigen::Vector2d PointA_NM = LatLonToNM(droneStates[n]->m_targetLatLon);
		Eigen::Vector2d PointB_NM = LatLonToNM(takeoffLatLon);

		double SampleDist = 1.0; //meters
		Maps::DataLayer layer = Maps::DataLayer::MinSafeAltitude;
		double Timeout = 5.0; //seconds
		double MSA_Min;
		double MSA_Max;
		bool NaNsDetected;
		if (Maps::DataTileProvider::Instance()->GetDataExtremesOnLine(PointA_NM, PointB_NM, SampleDist, layer, Timeout, MSA_Min, MSA_Max, NaNsDetected)) {
			if (std::isnan(maxMSA) && (! std::isnan(MSA_Max)))
				maxMSA = MSA_Max;
			if ((! std::isnan(maxMSA)) && (! std::isnan(MSA_Max)))
				maxMSA = std::max(maxMSA, MSA_Max);
			if (NaNsDetected)
				std::cerr << "Warning: Drone " << n + 1U << " will pass through no-fly zone as part of RTL sequence.\r\n";
		}
		else
			std::cerr << "Warning: MSA lookup failed for drone " << n + 1U << ". Excluding from alt detirmination.\r\n";
	}
	if (std::isnan(maxMSA)) {
		std::cerr << "Warning: Entire RTL sequence takes place in no-fly zone.\r\n";
	}
	else
		std::cerr << "Lowest drone must be above " << maxMSA << " m (WGS84 altitude).\r\n";

	//Step 3 - Assign staggered RTL HAGs that keeps all drones above max MSA
	//We will land the drones in order of their HAGs, so we want the drones sorted by battery level to ensure that the drones
	//with the lowest battery levels are landed first.
	std::vector<size_t> idx(drones.size());
	std::iota(idx.begin(), idx.end(), 0U);
	std::stable_sort(idx.begin(), idx.end(), [&droneBattLevels](size_t i1, size_t i2) {return droneBattLevels[i1] < droneBattLevels[i2];});

	//Set target HAGs - Lowest drone will be just above maxMSA or 100 feet, whichever is greater. Then start RTL sequences.
	double RTL_Alt = droneHomeAlts[idx[0]] + 100.0 * 0.3048;
	if (! std::isnan(maxMSA))
		RTL_Alt = std::max(maxMSA + 2.0, droneHomeAlts[idx[0]] + 100.0 * 0.3048);
	for (size_t n : idx) {
		droneStates[n]->m_targetYawDeg = 0.0f;
		droneStates[n]->m_RTL_HAG = RTL_Alt - droneHomeAlts[n];
		droneStates[n]->m_targetHAGFeet = (RTL_Alt - droneHomeAlts[n]) * 3.280839895;
		droneStates[n]->m_flyAtDeck = false;
		droneStates[n]->m_vehicleSpeedMPH = 33.6;
		RTL_Alt += 8.0;
	}
	m_RTL_State = 0; //Start the RTL sequence
	MapWidget::Instance().m_messageBoxOverlay.AddMessage("Warning: RTL Sequence in progress. Abort by issuing a move command to any drone (click and drag it)."s, m_messageToken);
}

//Set/Clear Hazard lets the vehicle control widget decide on how to respond to a hazard instead of requiring the command module to make the call.
//This is needed so that the vehicle control widget can react by stopping the vehicle only if it isn't already in a hazardous state. If already in a
//hazardous state we don't want to stop the drone again since it would interfere with our ability to command the vehicle out of the hazardous state.
//There are two flags that can be set: StopOnHazard and DodgeUpOnHazard.
//If either of these are set when a hazard condition begins, the vehicle control widget will take control of the corresponding drone.
//If StopOnHazard is set, the drone will be commanded to hold it's current position.
//If DodgeUpOnHazard is set, the drone will hold it's current horizontal position, but will ascend a few meters as well.
//Setting both to true has the same behavior as just setting DodgeUpOnHazard to true.
//Dodging up is a reasonable action if two drones are on a collision course - even if they can't be stopped in time, moving the upper drone higher
//in altitude can avoid a collision.
inline void VehicleControlWidget::SetHazardCondition(std::string Serial, bool StopOnHazard, bool DodgeUpOnHazard) {
	std::scoped_lock lock(m_dronesAndStatesMutex); //Lock vector of drone serials and states
	
	//Iterate through each connected drone - if the serial matches, start manual control of the drone and return true.
	for (size_t n = 0; n < m_currentDroneSerials.size(); n++) {
		if (m_currentDroneSerials[n] == Serial) {
			vehicleState & state(m_vehicleStates.at(Serial));
			if ((StopOnHazard || DodgeUpOnHazard) && (! state.m_hazard)) {
				DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(m_currentDroneSerials[n]);
				if (drone != nullptr) {
					EnableClickAndDragControl(*drone, state);
					if (DodgeUpOnHazard) {
						std::cerr << "Drone with serial " << Serial << " is dodging up.\r\n";
						state.m_targetHAGFeet += 12.0;
					}
				}
			}
			state.m_hazard = true;
			return;
		}
	}
}

inline void VehicleControlWidget::ClearHazardCondition(std::string Serial) {
	std::scoped_lock lock(m_dronesAndStatesMutex); //Lock vector of drone serials and states
	
	//Iterate through each connected drone - if the serial matches, stop manual control of the drone.
	for (size_t n = 0; n < m_currentDroneSerials.size(); n++) {
		if (m_currentDroneSerials[n] == Serial) {
			m_vehicleStates.at(m_currentDroneSerials[n]).m_hazard = false;
			return;
		}
	}
}

inline void VehicleControlWidget::EnableClickAndDragControl(DroneInterface::Drone & Drone, vehicleState & State, bool FlyAtDeck) {
	//std::cerr << "Starting manual control for drone: " << Drone.GetDroneSerial() << ".\r\n";
	
	//Instruct the guidance module to stop commanding this drone (if it currently is)
	Guidance::GuidanceEngine::Instance().RemoveDroneFromMission(Drone.GetDroneSerial());
	
	//Initialize the target position to the drone's current position (or NaN if unknown to zero out commanded velocity)
	DroneInterface::Drone::TimePoint Timestamp;
	double Latitude, Longitude, Altitude;
	State.m_targetLatLon << std::nan(""), std::nan("");
	if (Drone.GetPosition(Latitude, Longitude, Altitude, Timestamp) && (SecondsElapsed(Timestamp, std::chrono::steady_clock::now()) < 5.0))
		State.m_targetLatLon << Latitude, Longitude;
	else
		std::cerr << "Warning: Starting manual control without full & current telemetry. No initial position target.\r\n";
	
	//Initialize the target yaw and height above ground based on current state, as well as FlyAtDeck - then turn on manual control
	double Yaw, Pitch, Roll, HAG;
	double PI = 3.14159265358979;
	State.m_targetHAGFeet = 100.0f; //Default value
	State.m_targetYawDeg  = 0.0f;   //Default value
	if (Drone.GetHAG(HAG, Timestamp) && (SecondsElapsed(Timestamp, std::chrono::steady_clock::now()) < 5.0))
		State.m_targetHAGFeet = HAG*3.280839895;
	else
		std::cerr << "Warning: Starting manual control without full & current telemetry. Defaulting target height to 100 feet.\r\n";
	if (Drone.GetOrientation(Yaw, Pitch, Roll, Timestamp) && (SecondsElapsed(Timestamp, std::chrono::steady_clock::now()) < 5.0))
		State.m_targetYawDeg = Yaw*180.0/PI;
	else
		std::cerr << "Warning: Starting manual control without full & current telemetry. Defaulting yaw to 0 degrees.\r\n";
	State.m_flyAtDeck = FlyAtDeck;
	State.m_widgetControlEnabled = true;
	State.m_LastCommandWasHover = false;
	State.m_AtTargetState = false;
}

inline void VehicleControlWidget::DroneCommand_Hover(DroneInterface::Drone & Drone, vehicleState & State) {
	std::cerr << "Drone Command to " << Drone.GetDroneSerial() << ": Hover In Place.\r\n";
	Guidance::GuidanceEngine::Instance().RemoveDroneFromMission(Drone.GetDroneSerial()); //Tell the guidance module to stop commanding drone
	State.m_widgetControlEnabled = false; //Stop our control loop from commanding the drone
	Drone.Hover();
}

inline void VehicleControlWidget::DroneCommand_LandNow(DroneInterface::Drone & Drone, vehicleState & State) {
	std::cerr << "Drone Command to " << Drone.GetDroneSerial() << ": Land Now.\r\n";
	Guidance::GuidanceEngine::Instance().RemoveDroneFromMission(Drone.GetDroneSerial()); //Tell the guidance module to stop commanding drone
	State.m_widgetControlEnabled = false; //Stop our control loop from commanding the drone
	Drone.LandNow();
}

inline void VehicleControlWidget::DroneCommand_GoHomeAndLand(DroneInterface::Drone & Drone, vehicleState & State) {
	std::cerr << "Drone Command to " << Drone.GetDroneSerial() << ": Go Home and Land.\r\n";
	Guidance::GuidanceEngine::Instance().RemoveDroneFromMission(Drone.GetDroneSerial()); //Tell the guidance module to stop commanding drone
	State.m_widgetControlEnabled = false; //Stop our control loop from commanding the drone
	Drone.GoHomeAndLand();
}

inline void VehicleControlWidget::DrawContextMenu(bool Open, DroneInterface::Drone & drone) {
	float labelMargin = 1.5f*ImGui::GetFontSize();
	std::string popupStrIdentifier = drone.GetDroneSerial() + "-ContextMenu"s;
	
	//Get reference to vehicle state object and the drone serial number
	vehicleState & myState(m_vehicleStates.at(drone.GetDroneSerial()));
	std::string droneSerial = drone.GetDroneSerial();
	
	if (Open)
		ImGui::OpenPopup(popupStrIdentifier.c_str());
	
	if (ImGui::BeginPopup(popupStrIdentifier.c_str())) {
		bool isFlying = false;
		DroneInterface::Drone::TimePoint isFlyingTimestamp;
		if ((! drone.IsCurrentlyFlying(isFlying, isFlyingTimestamp)) || (SecondsElapsed(isFlyingTimestamp, std::chrono::steady_clock::now()) > 5.0))
			isFlying = false;
		
		ImGui::TextUnformatted(("Drone Serial: "s + drone.GetDroneSerial()).c_str());
		ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
		
		//The command & control options depend on whether the drone is currently flying or not
		if (! isFlying) {
			if (MyGui::BeginMenu(u8"\uf5b0", labelMargin, "Take off & Hover")) {
				if (ImGui::MenuItem("Confirm Command", NULL, false, true)) {
					DroneCommand_Hover(drone, myState);
					EnableClickAndDragControl(drone, myState, false);
					myState.m_targetHAGFeet = 50.0;
				}
				ImGui::EndMenu();
			}
			if ((Guidance::GuidanceEngine::Instance().IsRunning()) &&
			    (! Guidance::GuidanceEngine::Instance().IsCommandingDrone(droneSerial))) {
				if (MyGui::BeginMenu(u8"\uf04b", labelMargin, "Add to Current Mission")) {
					if (ImGui::MenuItem("Confirm Command", NULL, false, true))
						Guidance::GuidanceEngine::Instance().AddDroneToMission(droneSerial);
					ImGui::EndMenu();
				}
			}
		}
		else {
			if (MyGui::BeginMenu(u8"\uf04c", labelMargin, "Hover In Place")) {
				if (ImGui::MenuItem("Confirm Command", NULL, false, true))
					DroneCommand_Hover(drone, myState);
				ImGui::EndMenu();
			}
			if (MyGui::BeginMenu(u8"\uf5af", labelMargin, "Land Now")) {
				if (ImGui::MenuItem("Confirm Command", NULL, false, true))
					DroneCommand_LandNow(drone, myState);
				ImGui::EndMenu();
			}
			if (MyGui::BeginMenu(u8"\uf015", labelMargin, "Go Home and Land")) {
				if (ImGui::MenuItem("Confirm Command", NULL, false, true))
					DroneCommand_GoHomeAndLand(drone, myState);
				ImGui::EndMenu();
			}
			if (MyGui::BeginMenu(u8"\uf8cc", labelMargin, "Click & Drag Control")) {
				if (myState.m_widgetControlEnabled) {
					if (ImGui::MenuItem("Disable: Transfer to controller", NULL, false, true))
						DroneCommand_Hover(drone, myState);
				}
				else {
					if (ImGui::MenuItem("Enable", NULL, false, true))
						EnableClickAndDragControl(drone, myState);
				}
				ImGui::EndMenu();
			}
			if ((Guidance::GuidanceEngine::Instance().IsRunning()) &&
			    (! Guidance::GuidanceEngine::Instance().IsCommandingDrone(droneSerial))) {
				if (MyGui::BeginMenu(u8"\uf04b", labelMargin, "Add to Current Mission")) {
					if (ImGui::MenuItem("Confirm Command", NULL, false, true))
						Guidance::GuidanceEngine::Instance().AddDroneToMission(droneSerial);
					ImGui::EndMenu();
				}
			}
		}
		//TODO - this should be removed once waypoint mission functionality is tested
		if (ImGui::BeginMenu("Start Waypoint Mission")) {
			if (ImGui::MenuItem("Short Mission - Point 2 Point")) {
				myState.m_widgetControlEnabled = false;
				drone.StartSampleWaypointMission(10, false, false, Eigen::Vector2d(5.0, 50.0), 30.0);
			}
			if (ImGui::MenuItem("Short Mission - Round Corners")) {
				myState.m_widgetControlEnabled = false;
				drone.StartSampleWaypointMission(10, true, false, Eigen::Vector2d(5.0, 50.0), 30.0);
			}
			ImGui::EndMenu();
		}
		
		ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
		if (drone.IsDJICamConnected()) {
			if (MyGui::BeginMenu(u8"\uf03d", labelMargin, "Video Feed")) {
				if (drone.IsCamImageFeedOn()) {
					if (ImGui::MenuItem("Stop Feed", NULL, false, true))
						drone.StopDJICamImageFeed();
				}
				else {
					ImGui::DragFloat("Target FPS", &(myState.m_targetFPS), 0.02f, 0.25f, 30.0f, "%.1f");
					float framePeriod = std::clamp(std::round(30.0f/myState.m_targetFPS), 1.0f, 120.0f);
					float actualFPS = 30.0/framePeriod;
					ImGui::Text("Actual FPS will be: %.2f", actualFPS);
					
					DroneInterface::SimulatedDrone * mySimDrone = dynamic_cast<DroneInterface::SimulatedDrone *>(& drone);
					if (mySimDrone != nullptr) {
						//This is a simulated drone - make it possible to set the video source
						if (ImGui::BeginMenu("Video Source")) {
							std::vector<std::filesystem::path> subDirs = Handy::SubDirectories(Handy::Paths::ThisExecutableDirectory().parent_path() /
							                                                                   "Simulation-Data-Sets"s);
							std::sort(subDirs.begin(), subDirs.end(), [](std::string const & A, std::string const & B) ->
							                                          bool { return StringNumberAwareCompare_LessThan(A, B); });
							for (auto const & subdir : subDirs) {
								if (ImGui::MenuItem(subdir.stem().string().c_str())) {
									mySimDrone->SetSourceVideoFile(GetSimVideoFilePath(subdir));
									mySimDrone->SetRealTime(true);
								}
							}
							ImGui::EndMenu();
						}
					}
					
					if (ImGui::MenuItem("Start Feed", NULL, false, true)) {
						//If the drone is actually a simulated drone we need to tell it to feed imagery in real time
						DroneInterface::SimulatedDrone * mySimDrone = dynamic_cast<DroneInterface::SimulatedDrone *>(& drone);
						if (mySimDrone != nullptr)
							mySimDrone->SetRealTime(true);
						drone.StartDJICamImageFeed(myState.m_targetFPS);
					}
				}
				if (! myState.m_showFeed) {
					if (ImGui::MenuItem("Open Feed Window", NULL, false, true))
						myState.m_showFeed = true;
				}
				else {
					if (ImGui::MenuItem("Close Feed Window", NULL, false, true))
						myState.m_showFeed = false;
				}
				
				ImGui::EndMenu();
			}
			ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
		}
		if (MyGui::MenuItem(u8"\uf05b", labelMargin, "Zoom To Drone", NULL, false, true)) {
			double Latitude, Longitude, Altitude;
			DroneInterface::Drone::TimePoint Timestamp;
			if (drone.GetPosition(Latitude, Longitude, Altitude, Timestamp)) {
				double const eps = 0.00004; //Radians
				MapWidget::Instance().StartAnimation(Latitude - eps, Latitude + eps, Longitude - eps, Longitude + eps);
			}
			else
				std::cerr << "Error - Can't zoom to drone because position is unknown.\r\n";
		}
		if (MyGui::BeginMenu(u8"\uf05a", labelMargin, "Full Info")) {
			double PI = 3.14159265358979;
			float col2Start = ImGui::CalcTextSize("Height Above Ground:   ").x;
			float col2P5Start = col2Start + ImGui::CalcTextSize("77777 feet  ").x;
			float col3Start = col2Start + ImGui::CalcTextSize("Connected (Image feed OFF)    ").x;
			
			DroneInterface::Drone::TimePoint Timestamp;
			double Latitude, Longitude, Altitude, HAG;
			if (drone.GetPosition(Latitude, Longitude, Altitude, Timestamp)) {
				ImGui::TextUnformatted("Latitude: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.6f deg", Latitude*180.0/PI);
				PrintAgeWarning(Timestamp, col3Start);
				
				ImGui::TextUnformatted("Longitude: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.6f deg", Longitude*180.0/PI);
				PrintAgeWarning(Timestamp, col3Start);
				
				ImGui::TextUnformatted("Altitude: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.0f feet", Altitude*3.280839895);
				ImGui::SameLine(col2P5Start);
				ImGui::Text("(%.1f m)", Altitude);
				PrintAgeWarning(Timestamp, col3Start);
			}
			else {
				ImGui::TextUnformatted("Position:");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Unknown");
			}
			if (drone.GetHAG(HAG, Timestamp)) {
				ImGui::TextUnformatted("Height Above Ground: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.0f feet", HAG*3.280839895);
				ImGui::SameLine(col2P5Start);
				ImGui::Text("(%.1f m)", HAG);
				PrintAgeWarning(Timestamp, col3Start);
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
			}
			else {
				ImGui::TextUnformatted("Height Above Ground:");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Unknown");
			}
			
			double V_North, V_East, V_Down;
			if (drone.GetVelocity(V_North, V_East, V_Down, Timestamp)) {
				ImGui::TextUnformatted("North Velocity: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.1f mph", V_North*2.23694);
				ImGui::SameLine(col2P5Start);
				ImGui::Text("(%.1f m/s)", V_North);
				PrintAgeWarning(Timestamp, col3Start);
				
				ImGui::TextUnformatted("East Velocity: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.1f mph", V_East*2.23694);
				ImGui::SameLine(col2P5Start);
				ImGui::Text("(%.1f m/s)", V_East);
				PrintAgeWarning(Timestamp, col3Start);
				
				ImGui::TextUnformatted("Down Velocity: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.1f mph", V_Down*2.23694);
				ImGui::SameLine(col2P5Start);
				ImGui::Text("(%.1f m/s)", V_Down);
				PrintAgeWarning(Timestamp, col3Start);
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
			}
			else {
				ImGui::TextUnformatted("Velocity:");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Unknown");
			}
			
			double Yaw, Pitch, Roll;
			if (drone.GetOrientation(Yaw, Pitch, Roll, Timestamp)) {
				ImGui::TextUnformatted("Yaw: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.0f deg", Yaw*180.0/PI);
				PrintAgeWarning(Timestamp, col3Start);
				
				ImGui::TextUnformatted("Pitch: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.0f deg", Pitch*180.0/PI);
				PrintAgeWarning(Timestamp, col3Start);
				
				ImGui::TextUnformatted("Roll: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.0f deg", Roll*180.0/PI);
				PrintAgeWarning(Timestamp, col3Start);
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
			}
			else {
				ImGui::TextUnformatted("Orientation:");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Unknown");
			}
			
			double BattLevel;
			if (drone.GetVehicleBatteryLevel(BattLevel, Timestamp)) {
				ImGui::TextUnformatted("Vehicle Battery: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.0f%%", 100.0*BattLevel);
				PrintAgeWarning(Timestamp, col3Start);
			}
			else {
				ImGui::TextUnformatted("Vehicle Battery: ");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Unknown");
			}
			
			bool MaxHAG, MaxDistFromHome;
			if (drone.GetActiveLimitations(MaxHAG, MaxDistFromHome, Timestamp)) {
				ImGui::TextUnformatted("Active Limitations: ");
				ImGui::SameLine(col2Start);
				if (MaxHAG && MaxDistFromHome)
					ImGui::TextUnformatted("Max distance from home + Max height reached");
				else if (MaxHAG)
					ImGui::TextUnformatted("Max height reached");
				else if (MaxDistFromHome)
					ImGui::TextUnformatted("Max distance from home reached");
				else
					ImGui::TextUnformatted("None");
				PrintAgeWarning(Timestamp, col3Start);
			}
			else {
				ImGui::TextUnformatted("Active Limitations: ");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Unknown");
			}
			
			std::vector<std::string> ActiveWarnings;
			if (drone.GetActiveWarnings(ActiveWarnings, Timestamp)) {
				ImGui::TextUnformatted("Warnings: ");
				ImGui::SameLine(col2Start);
				if (ActiveWarnings.empty())
					ImGui::TextUnformatted("None");
				else {
					for (size_t n = 0U; n < ActiveWarnings.size(); n++) {
						if (n > 0U) {
							ImGui::Dummy(ImVec2(1.0f, 1.0f));
							ImGui::SameLine(col2Start);
						}
						ImGui::TextUnformatted(ActiveWarnings[n].c_str());
						PrintAgeWarning(Timestamp, col3Start);
					}
				}
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
			}
			else {
				ImGui::TextUnformatted("Warnings: ");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Unknown");
			}
			
			unsigned int GNSSSatCount;
			int GNSSSignalLevel;
			if (drone.GetGNSSStatus(GNSSSatCount, GNSSSignalLevel, Timestamp)) {
				ImGui::TextUnformatted("GNSS Sat Count: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%u", GNSSSatCount);
				PrintAgeWarning(Timestamp, col3Start);
				
				ImGui::TextUnformatted("GNSS Signal Strength: ");
				ImGui::SameLine(col2Start);
				if (GNSSSignalLevel < 0)
					ImGui::TextUnformatted("No Signal");
				else if (GNSSSignalLevel == 0)
					ImGui::TextUnformatted("0 (Extremely Week - Hover and RTL may not work)");
				else if (GNSSSignalLevel == 1)
					ImGui::TextUnformatted("1 (Very Week - Hover and RTL may not work)");
				else if (GNSSSignalLevel == 2)
					ImGui::TextUnformatted("2 (Week - Hover may not work)");
				else if (GNSSSignalLevel == 3)
					ImGui::TextUnformatted("3 (Good)");
				else if (GNSSSignalLevel == 4)
					ImGui::TextUnformatted("4 (Very Good)");
				else if (GNSSSignalLevel == 5)
					ImGui::TextUnformatted("5 (Excellent)");
				PrintAgeWarning(Timestamp, col3Start);
			}
			else {
				ImGui::TextUnformatted("GNSS Status: ");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Unknown");
			}
			
			bool DJICamConnected = drone.IsDJICamConnected();
			bool DJICamRunning = drone.IsCamImageFeedOn();
			ImGui::TextUnformatted("DJI Camera: ");
			ImGui::SameLine(col2Start);
			if (DJICamConnected && DJICamRunning)
				ImGui::TextUnformatted("Connected (Image feed ON)");
			else if (DJICamConnected)
				ImGui::TextUnformatted("Connected (Image feed OFF)");
			else
				ImGui::TextUnformatted("None");
			
			std::string FlightModeStr;
			if (drone.GetFlightMode(FlightModeStr, Timestamp)) {
				ImGui::TextUnformatted("Flight Mode: ");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted(FlightModeStr.c_str());
				PrintAgeWarning(Timestamp, col3Start);
			}
			else {
				ImGui::TextUnformatted("Flight Mode: ");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Unknown");
			}
			
			ImGui::EndMenu();
		}
		
		ImGui::EndPopup();
	}
}

//This function draws the interactable drone object for a single drone - this is the widget that appears under "Connected Vehicles"
//TODO: So... drone should be passed as a const & but we need the drone class to be const-correct.
inline void VehicleControlWidget::DrawDroneInteractable(DroneInterface::Drone & drone, vehicleState & State, size_t DroneIndex) {
	ImDrawList * draw_list = ImGui::GetWindowDrawList();
	draw_list->ChannelsSplit(2);

	auto style = ImGui::GetStyle();
	ImGui::PushID(drone.GetDroneSerial().c_str());

	Eigen::Vector2d pos = ImGui::GetCursorScreenPos();
	ImGui::SetCursorScreenPos((Eigen::Vector2d) (pos + (Eigen::Vector2d) style.ItemInnerSpacing));

	//Display node foreground first
	draw_list->ChannelsSetCurrent(1);
	
	bool IsFlying = false;
	DroneInterface::Drone::TimePoint Timestamp;
	if (! drone.IsCurrentlyFlying(IsFlying, Timestamp))
		IsFlying = false;
	
	float startX = ImGui::GetCursorPosX();
	ImGui::Image(m_IconTexture_Drone, ImVec2(2.0f*ImGui::GetFontSize(), 2.0f*ImGui::GetFontSize()));
	ImGui::SameLine(startX + 2.8f*ImGui::GetFontSize());

	//Draw the drone number, hazard indicator (if necessary) and control mode icons (if necessary).
	//We do this is a way that seems overly complex, but we always use 2 lines when the drone is in the air
	//so that the drone number doesn't bounce around vertically if entering/exiting a hazard state. This looks
	//better overall than just having a blank second line when not in hazard (which looks like a mistake).
	std::string droneNumStr = std::to_string((unsigned int) DroneIndex + 1U);
	if (State.m_hazard)
		droneNumStr += u8" \uf071";
	ImGui::BeginGroup();
	if (IsFlying) {
		if (State.m_widgetControlEnabled) {
			//The vehicle control widget is controlling the drone (Click & Drag) - draw mouse icon
			ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (ImGui::CalcTextSize("\uf8cc").x - ImGui::CalcTextSize(droneNumStr.c_str()).x)/2.0f);
			ImGui::TextUnformatted(droneNumStr.c_str());
			ImGui::TextUnformatted("\uf8cc");
		}
		else if (Guidance::GuidanceEngine::Instance().IsCommandingDrone(drone.GetDroneSerial())) {
			//The guidance module is controlling the drone - draw icon to indicate this
			ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (ImGui::CalcTextSize("\uf04b").x - ImGui::CalcTextSize(droneNumStr.c_str()).x)/2.0f);
			ImGui::TextUnformatted(droneNumStr.c_str());
			ImGui::TextUnformatted("\uf04b");
		}
		else {
			//Neither the vehicle control widget or the guidance module is in control - draw controller icon
			ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (ImGui::CalcTextSize("\uf11b").x - ImGui::CalcTextSize(droneNumStr.c_str()).x)/2.0f);
			ImGui::TextUnformatted(droneNumStr.c_str());
			ImGui::TextUnformatted("\uf11b");
		}
	}
	else {
		//Drone is on ground
		ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 0.5f*ImGui::GetFontSize());
		ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (ImGui::CalcTextSize("\uf8cc").x - ImGui::CalcTextSize(droneNumStr.c_str()).x)/2.0f);
		ImGui::TextUnformatted(droneNumStr.c_str());
	}
	ImGui::EndGroup();
	
	ImGui::SameLine(startX + 4.6f*ImGui::GetFontSize());
	ImGui::BeginGroup();
	std::string FlightModeStr;
	if (drone.GetFlightMode(FlightModeStr, Timestamp))
		AddOutdatedDataNote(FlightModeStr, Timestamp);
	else
		FlightModeStr = "Unknown"s;
	ImGui::TextUnformatted(("Mode: "s + FlightModeStr).c_str());
	
	double BattLevel;
	if (drone.GetVehicleBatteryLevel(BattLevel, Timestamp)) {
		ImGui::TextUnformatted("Bat: ");
		ImGui::SameLine();
		ImGui::PushFont(Fonts::NormalBold);
		if (BattLevel >= 0.5)
			ImGui::TextColored(ImVec4(0,0.8f,0,1), "%d%%  ", (int) std::round(BattLevel*100.0));
		else if (BattLevel >= 0.35)
			ImGui::TextColored(ImVec4(1,1,0,1), "%d%%  ", (int) std::round(BattLevel*100.0));
		else if (BattLevel >= 0.20)
			ImGui::TextColored(ImVec4(1,0.5f,0,1), "%d%%  ", (int) std::round(BattLevel*100.0));
		else
			ImGui::TextColored(ImVec4(1,0,0,1), "%d%%  ", (int) std::round(BattLevel*100.0));
		double Age = SecondsElapsed(Timestamp, std::chrono::steady_clock::now());
		if (Age > 4.0) {
			ImGui::SameLine();
			ImGui::Text(" (Data is %d seconds old)", (int) std::round(Age));
		}
		ImGui::PopFont();
	}
	else
		ImGui::TextUnformatted("Bat: ?  ");
	
	ImGui::SameLine();
	double HAG;
	std::string HAGStr("Height: ?");
	if (drone.GetHAG(HAG, Timestamp)) {
		HAGStr = "Height: "s + std::to_string((int) std::round(HAG*3.280839895)) + " ft"s;
		AddOutdatedDataNote(HAGStr, Timestamp);
	}
	ImGui::TextUnformatted(HAGStr.c_str());
	ImGui::EndGroup();
	
	//Save the size of what we have emitted and reset cursor position
	float contentHeight = ImGui::GetItemRectSize().y + 2.0f*style.ItemInnerSpacing.y;
	ImGui::SetCursorScreenPos(pos);
	Eigen::Vector2d itemRectSize(std::max(ImGui::GetContentRegionAvail().x, 1.0f), std::max(contentHeight, 1.0f)); //Make sure no 0's or InvisibleButton will raise.
	
	//Display node background next
	draw_list->ChannelsSetCurrent(0);
	
	//Set background color for interactable. Should indicate hovering and should flash on hazards
	/*ImGui::InvisibleButton("node", itemRectSize);
	double t = 0.0;
	if (State.m_hazard)
		t = 1.0 - std::abs(2.0*FractionalPart(SecondsSinceT0Epoch(std::chrono::steady_clock::now())/2.0) - 1.0); //(0 - 1 - 0) every 2 s
	//double t = State.m_hazard ? FractionalPart(SecondsSinceT0Epoch(std::chrono::steady_clock::now())/3.0) : 0.0;
	Eigen::Vector4d bgColorA = ImGui::IsItemHovered() ? Eigen::Vector4d(0.47,0.47,0.31,1.0) : Eigen::Vector4d(style.Colors[ImGuiCol_Header]);
	Eigen::Vector4d bgColorB = Eigen::Vector4d(0.6,0.3,0.1,1.0);
	Eigen::Vector4d bgColor  = t * bgColorB + (1.0 - t) * bgColorA;
	draw_list->AddRectFilled(pos, (Eigen::Vector2d) (pos + itemRectSize), ImColor(bgColor), 3.0f);*/

	ImGui::InvisibleButton("node", itemRectSize);
	if (ImGui::IsItemHovered())
		draw_list->AddRectFilled(pos, (Eigen::Vector2d) (pos + itemRectSize), IM_COL32(125,125,75,255), 3.0f);
	else
		draw_list->AddRectFilled(pos, (Eigen::Vector2d) (pos + itemRectSize), ImColor(style.Colors[ImGuiCol_Header]), 3.0f);

	//Cleanup - pop object ID and merge draw list channels
	ImGui::PopID();
	draw_list->ChannelsMerge();
}

inline void VehicleControlWidget::Draw() {
	float cursorStartYPos = ImGui::GetCursorPos().y;
	MyGui::HeaderLabel("Connected Vehicles");
	
	std::scoped_lock lock(m_dronesAndStatesMutex); //Lock vector of drone serials and states
	
	//Get list of connected drones
	m_currentDroneSerials = DroneInterface::DroneManager::Instance().GetConnectedDroneSerialNumbers();
	std::sort(m_currentDroneSerials.begin(), m_currentDroneSerials.end());
	
	//For each connected drone, make sure we have a valid DroneState
	for (std::string serial : m_currentDroneSerials) {
		if (m_vehicleStates.count(serial) == 0U)
			m_vehicleStates[serial].m_serial = serial;
	}
	
	//Iterate through each connected drone
	for (size_t n = 0; n < m_currentDroneSerials.size(); n++) {
		DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(m_currentDroneSerials[n]);
		if (drone == nullptr)
			continue;
		
		DrawDroneInteractable(*drone, m_vehicleStates.at(m_currentDroneSerials[n]), n);
		DrawContextMenu(ImGui::IsItemClicked(0) || ImGui::IsItemClicked(1), *drone);
	}
	
	//After drawing, update content height and recommended widget height
	ContentHeight = ImGui::GetCursorPos().y - cursorStartYPos;
	RecommendedHeight = 0.85f*RecommendedHeight + 0.15f*ContentHeight;
	if (std::abs(RecommendedHeight - ContentHeight) < 1.0f)
		RecommendedHeight = ContentHeight;
	
	//Finally, draw any live video feed windows (if open)
	DrawVideoWindows();
}


