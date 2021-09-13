//This widget exposes high-level command and control... things like starting a mission or pausing all drones
//This widget also contains the watchdog thread for hazard monitoring
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
#include "soloud.h"
#include "soloud_wav.h"

//Project Includes
#include "../EigenAliases.h"
#include "../SurveyRegionManager.hpp"
#include "VehicleControlWidget.hpp"
#include "ReconUI.hpp"
//#include "MyGui.hpp"
#include "EmbeddedIcons.hpp"
#include "../Modules/DJI-Drone-Interface/DroneManager.hpp"
#include "../Modules/Shadow-Detection/ShadowDetection.hpp"
#include "../Modules/Shadow-Propagation/ShadowPropagation.hpp"
#include "../Modules/Guidance/Guidance.hpp"
#include "../Utilities.hpp"

class CommandWidget {
	public:
		static CommandWidget & Instance() { static CommandWidget Widget; return Widget; }
		
		//Constructors and Destructors
		CommandWidget() : Log(*(ReconUI::Instance().Log)) {
			m_watchdogThread = std::thread(&CommandWidget::WatchdogThreadMain, this);
			m_IconTexture_Mission   = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_Play_84x84[0],           84, 84);
			m_IconTexture_Watchdog  = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_Watchdog_Light_84x84[0], 84, 84);
			m_IconTexture_Emergency = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_Warning_84x84[0],        84, 84);
			
			m_warningSoundWav.load((Handy::Paths::ThisExecutableDirectory() / "Sounds" / "TF046.WAV").string().c_str());
		}
		~CommandWidget() {
			ImGuiApp::Instance().DeleteImage(m_IconTexture_Mission);
			ImGuiApp::Instance().DeleteImage(m_IconTexture_Watchdog);
			ImGuiApp::Instance().DeleteImage(m_IconTexture_Emergency);
		}
		
		//Should be stopped before DataTileProvider since the private thread sometimes uses that system
		inline void Stop(void) {
			m_watchdogThreadAbort = true;
			if (m_watchdogThread.joinable())
				m_watchdogThread.join();
		}
		
		//Public accessors for getting the state of the window
		float GetWidgetRecommendedHeight(void) { return RecommendedHeight; } //Typically equals ContentHeight, but changes smoothly with time for animation.
		
		inline void Draw();
	
	private:
		Journal & Log;
		ImTextureID m_IconTexture_Mission;
		ImTextureID m_IconTexture_Watchdog;
		ImTextureID m_IconTexture_Emergency;
		
		std::thread       m_watchdogThread;
		std::atomic<bool> m_watchdogThreadAbort = false;
		
		std::mutex  m_watchdowMutex;
		
		bool           m_playSoundOnViolation = true;
		SoLoud::Wav    m_warningSoundWav;
		SoLoud::handle m_warningSoundPlayHandle = 0U;
		
		//An MSA violation occurs when a drone is below the MSA and not in a landing zone
		bool  m_CheckMSA = true;
		bool  m_pauseDronesOnMSAViolation = true;
		
		//A proximity violation occurs when two flying drones get too close to one another
		bool  m_CheckVehicleProximity = true;
		bool  m_AutoPauseDronesOnProximityViolation = true;
		float m_vehicleProximityWarnThreshold = 3.0; //meters
		
		//An avoidance zone violation occurs when a drone is in an avoidance zone
		bool  m_CheckAvoidanceZones = true;
		
		std::vector<bool> m_useDroneFlags;
		
		float ContentHeight; //Height of widget content from last draw pass
		float RecommendedHeight; //Recommended height for widget
		
		inline void WatchdogThreadMain(void);
		static inline Eigen::Vector3d positionLLA2ECEF(double lat, double lon, double alt);
};

inline Eigen::Vector3d CommandWidget::positionLLA2ECEF(double lat, double lon, double alt) {
	double a = 6378137.0;           //Semi-major axis of reference ellipsoid
	double ecc = 0.081819190842621; //First eccentricity of the reference ellipsoid
	double eccSquared = ecc*ecc;
	double N = a/sqrt(1.0 - eccSquared*sin(lat)*sin(lat));
	double X = (N + alt)*cos(lat)*cos(lon);
	double Y = (N + alt)*cos(lat)*sin(lon);
	double Z = (N*(1 - eccSquared) + alt)*sin(lat);
	return(Eigen::Vector3d(X, Y, Z));
}

inline void CommandWidget::WatchdogThreadMain(void) {
	double approxCheckPeriod  = 0.25; //seconds (doesn't impact abort latancy)
	
	//Wait for DataTileProvider to initialize
	while (Maps::DataTileProvider::Instance() == nullptr)
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	std::chrono::time_point<std::chrono::steady_clock> lastCheckTimepoint = std::chrono::steady_clock::now();
	while (! m_watchdogThreadAbort) {
		if (SecondsElapsed(lastCheckTimepoint) >= approxCheckPeriod) {
			//Get drone telemetry and check for hazards
			std::scoped_lock lock(m_watchdowMutex);
			
			//Get pointers to all valid, flying drones
			std::vector<std::string> droneSerials = DroneInterface::DroneManager::Instance().GetConnectedDroneSerialNumbers();
			std::vector<DroneInterface::Drone *> drones;
			drones.reserve(droneSerials.size());
			for (std::string serial : droneSerials) {
				DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(serial);
				if (drone == nullptr)
					continue;
				bool isFlying;
				DroneInterface::Drone::TimePoint T0;
				if ((! drone->IsCurrentlyFlying(isFlying, T0)) || (! isFlying) || (SecondsElapsed(T0, std::chrono::steady_clock::now()) >= 5.0))
					continue;
				drones.push_back(drone);
			}
			
			//Initialize a vector to track which drones are in a hazardous state
			std::vector<bool> hazardStates(drones.size(), false);
			
			//Check for violations of no-fly zones, avoidance zones, or min safe altitude rules. We look ahead of each drone based on it's
			//current position and velocity vector so we can trigger a hazard state early enough to stop the drone before the violation.
			if (m_CheckMSA || m_CheckAvoidanceZones) {
				for (size_t n = 0U; n < drones.size(); n++) {
					DroneInterface::Drone::TimePoint T0, T1;
					double Latitude, Longitude, Altitude;
					double V_North, V_East, V_Down;
					if (drones[n]->GetPosition(Latitude, Longitude, Altitude, T0) && (SecondsElapsed(T0) < 5.0) &&
					    drones[n]->GetVelocity(V_North, V_East, V_Down, T1)       && (SecondsElapsed(T1) < 5.0)) {
						//We have up-to-date position and velocity for this drone
						Eigen::Vector2d Position_NM = LatLonToNM(Eigen::Vector2d(Latitude, Longitude));
						
						double speed2D = std::sqrt(V_North*V_North + V_East*V_East);
						double maxDecel = 2.5; //m/s/s - measured on Inspire 1
						double stoppingDist = speed2D*speed2D / (2.0 * maxDecel); //m
						double lookaheadDist = 1.25*stoppingDist;
						double lookaheadDist_NM = MetersToNMUnits(lookaheadDist, Position_NM(1));
						double sampleDist_NM = MetersToNMUnits(2.0, Position_NM(1));
						int numberOfForwardSamples = int(std::round(lookaheadDist_NM / sampleDist_NM));
						
						std::Evector<Eigen::Vector2d> samplePoints_NM;
						samplePoints_NM.push_back(Position_NM);
						
						Eigen::Vector2d V_NM(V_East, V_North);
						if (V_NM.norm() > 1e-8) {
							V_NM.normalize();
							for (int n = 0; n < numberOfForwardSamples; n++)
								samplePoints_NM.push_back(Position_NM + (n + 1)*sampleDist_NM*V_NM);
							samplePoints_NM.push_back(Position_NM + lookaheadDist_NM*V_NM);
						}
						
						double MSA;
						double LandingZone;
						double AvoidanceZone;
						for (auto const & samplePoint_NM : samplePoints_NM) {
							if (m_CheckMSA &&
							    Maps::DataTileProvider::Instance()->TryGetData(samplePoint_NM, Maps::DataLayer::MinSafeAltitude, MSA) &&
							    Maps::DataTileProvider::Instance()->TryGetData(samplePoint_NM, Maps::DataLayer::SafeLandingZones, LandingZone)) {
								if (std::isnan(MSA)) {
									std::cerr << "Hazard - Drone serial: " << drones[n]->GetDroneSerial() << " - No-fly zone violation.\r\n";
									VehicleControlWidget::Instance().SetHazardCondition(drones[n]->GetDroneSerial(), m_pauseDronesOnMSAViolation);
									hazardStates[n] = true;
								}
								else if ((Altitude < MSA) && (std::isnan(LandingZone) || (LandingZone < 0.5))) {
									std::cerr << "Hazard - Drone serial: " << drones[n]->GetDroneSerial() << " - MSA violation.\r\n";
									VehicleControlWidget::Instance().SetHazardCondition(drones[n]->GetDroneSerial(), m_pauseDronesOnMSAViolation);
									hazardStates[n] = true;
								}
							}
							if (m_CheckAvoidanceZones &&
							    Maps::DataTileProvider::Instance()->TryGetData(samplePoint_NM, Maps::DataLayer::AvoidanceZones, AvoidanceZone)) {
								if ((! std::isnan(AvoidanceZone)) && (AvoidanceZone >= 0.5)) {
									std::cerr << "Hazard - Drone serial: " << drones[n]->GetDroneSerial() << " - avoidance zone violation.\r\n";
									VehicleControlWidget::Instance().SetHazardCondition(drones[n]->GetDroneSerial(), false);
									hazardStates[n] = true;
								}
							}
							if (hazardStates[n]) {
								double hazardDist = NMUnitsToMeters((samplePoint_NM - Position_NM).norm(), Position_NM(1));
								std::cerr << "Hazard distance: " << hazardDist << " m (Lookahead is " << lookaheadDist << " m)\r\n";
								break;
							}
						}
					}
				}
			}
			
			//If enabled, check for violations of MSA and avoidance zones
			//The commented out version here only checks the drones current location - it doesn't look ahead.
			/*if (m_CheckMSA || m_CheckAvoidanceZones) {
				for (size_t n = 0U; n < drones.size(); n++) {
					DroneInterface::Drone::TimePoint T0;
					double Latitude, Longitude, Altitude;
					if ((drones[n]->GetPosition(Latitude, Longitude, Altitude, T0)) && (SecondsElapsed(T0, std::chrono::steady_clock::now()) < 5.0)) {
						Eigen::Vector2d Position_NM = LatLonToNM(Eigen::Vector2d(Latitude, Longitude));
						double MSA;
						double LandingZone;
						double AvoidanceZone;
						if (m_CheckMSA &&
						    Maps::DataTileProvider::Instance()->TryGetData(Position_NM, Maps::DataLayer::MinSafeAltitude, MSA) &&
						    Maps::DataTileProvider::Instance()->TryGetData(Position_NM, Maps::DataLayer::SafeLandingZones, LandingZone)) {
							if (std::isnan(MSA)) {
								std::cerr << "Hazard: Drone with serial " << drones[n]->GetDroneSerial() << " in no-fly zone.\r\n";
								VehicleControlWidget::Instance().SetHazardCondition(drones[n]->GetDroneSerial(), m_pauseDronesOnMSAViolation);
								hazardStates[n] = true;
							}
							else if ((Altitude < MSA) && (std::isnan(LandingZone) || (LandingZone < 0.5))) {
								std::cerr << "Hazard: Drone with serial " << drones[n]->GetDroneSerial() << " below MSA outside of landing zone.\r\n";
								VehicleControlWidget::Instance().SetHazardCondition(drones[n]->GetDroneSerial(), m_pauseDronesOnMSAViolation);
								hazardStates[n] = true;
							}
						}
						if (m_CheckAvoidanceZones &&
						    Maps::DataTileProvider::Instance()->TryGetData(Position_NM, Maps::DataLayer::AvoidanceZones, AvoidanceZone)) {
							if ((! std::isnan(AvoidanceZone)) && (AvoidanceZone >= 0.5)) {
								std::cerr << "Hazard: Drone with serial " << drones[n]->GetDroneSerial() << " in an avoidance zone.\r\n";
								VehicleControlWidget::Instance().SetHazardCondition(drones[n]->GetDroneSerial(), false);
								hazardStates[n] = true;
							}
						}
					}
				}
			}*/
			
			//If enabled, check for proximity violations
			//TODO: For this to be useful, it really needs to look ahead in time like the checks above do. We could have a relative velocity of
			//almost 70 mph with these drones... we can't wait until they are almost on top of each other to react.
			//Also, our mitigation of stopping the drones may be a poor choice. It might be better to drop one in height and raise the other
			//and have the drones dodge each other instead of trying to come to a full stop.
			if (m_CheckVehicleProximity) {
				for (size_t n = 0U; n < drones.size(); n++) {
					DroneInterface::Drone::TimePoint T0;
					double Lat0, Lon0, Alt0;
					if ((! drones[n]->GetPosition(Lat0, Lon0, Alt0, T0)) || (SecondsElapsed(T0, std::chrono::steady_clock::now()) >= 5.0))
						continue;
					Eigen::Vector3d pos0_ECEF = positionLLA2ECEF(Lat0, Lon0, Alt0);
					for (size_t m = n + 1U; m < drones.size(); m++) {
						DroneInterface::Drone::TimePoint T1;
						double Lat1, Lon1, Alt1;
						if ((! drones[m]->GetPosition(Lat1, Lon1, Alt1, T1)) || (SecondsElapsed(T1, std::chrono::steady_clock::now()) >= 5.0))
							continue;
						Eigen::Vector3d pos1_ECEF = positionLLA2ECEF(Lat1, Lon1, Alt1);
						
						//If we get here we have current telemetry for both drones - check proximity
						if ((pos1_ECEF - pos0_ECEF).norm() < m_vehicleProximityWarnThreshold) {
							std::cerr << "Hazard: Drones with serials " << drones[n]->GetDroneSerial() << " and";
							std::cerr << drones[m]->GetDroneSerial() << " are too close.\r\n";
							VehicleControlWidget::Instance().SetHazardCondition(drones[n]->GetDroneSerial(), m_AutoPauseDronesOnProximityViolation);
							VehicleControlWidget::Instance().SetHazardCondition(drones[m]->GetDroneSerial(), m_AutoPauseDronesOnProximityViolation);
							hazardStates[n] = true;
							hazardStates[m] = true;
						}
					}
				}
			}
			
			//For any drones not in a hazardous state, inform the vehicle control widget
			for (size_t n = 0U; n < drones.size(); n++) {
				if (! hazardStates[n])
					VehicleControlWidget::Instance().ClearHazardCondition(drones[n]->GetDroneSerial());
			}
			
			//If any drones are in a hazardous state, play hazard sound (if enabled)
			if (m_playSoundOnViolation) {
				for (bool hazardState : hazardStates) {
					if (hazardState) {
						std::scoped_lock lock(ReconUI::Instance().soLoudMutex);
						if (! ReconUI::Instance().gSoloud.isValidVoiceHandle(m_warningSoundPlayHandle))
							m_warningSoundPlayHandle = ReconUI::Instance().gSoloud.play(m_warningSoundWav);
						break;
					}
				}
			}
			
			lastCheckTimepoint = std::chrono::steady_clock::now();
		}
		
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

inline void CommandWidget::Draw() {
	float cursorStartYPos = ImGui::GetCursorPos().y;
	float widgetWidth = ImGui::GetContentRegionAvail().x;
	
	ImVec2 iconSize(1.4f*ImGui::GetFontSize(), 1.4f*ImGui::GetFontSize());
	ImGuiStyle & style(ImGui::GetStyle());
	ImExt::Style styleSitter(StyleVar::FramePadding, Math::Vector2(std::max((widgetWidth/3.0f - iconSize.x - style.ItemSpacing.x)/2.0f, 0.0f), 3.0f));
	ImVec2 buttonSize(iconSize.x + 2.0f*style.FramePadding.x, iconSize.y + 2.0f*style.FramePadding.y);
	float button2Start = ImGui::GetCursorPos().x + widgetWidth/2.0f - buttonSize.x/2.0f;
	float button3Start = ImGui::GetCursorPos().x + widgetWidth - buttonSize.x;
	
	//Mission Button
	if (ImGui::ImageButton(m_IconTexture_Mission, iconSize, ImVec2(0, 0), ImVec2(1,1), -1, ImVec4(0,0,0,0), ImVec4(1,1,1,1)))
		ImGui::OpenPopup("Mission Popup");
	
	//Watchdog Button
	ImGui::SameLine(button2Start);
	if (ImGui::ImageButton(m_IconTexture_Watchdog, iconSize, ImVec2(0, 0), ImVec2(1,1), -1, ImVec4(0,0,0,0), ImVec4(1,1,1,1)))
		ImGui::OpenPopup("Watchdog Popup");
	
	//Emergency Button
	ImGui::SameLine(button3Start);
	if (ImGui::ImageButton(m_IconTexture_Emergency, iconSize, ImVec2(0, 0), ImVec2(1,1), -1, ImVec4(0,0,0,0), ImVec4(1,1,1,1)))
		ImGui::OpenPopup("Emergency Popup");
	
	//Mission Popup
	{
		ImExt::Style styleSitter(StyleVar::WindowPadding, ImVec2(4.0f, 4.0f));
		if (ImGui::BeginPopup("Mission Popup", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			{
				ImExt::Font fnt(Fonts::NormalBold);
				float col2start = ImGui::CalcTextSize("Shadow Avoidance::  ").x;
				
				ImGui::TextUnformatted("Mission Status:");
				ImGui::SameLine(col2start);
				ImGui::TextUnformatted(Guidance::GuidanceEngine::Instance().GetMissionStatusStr().c_str());
				
				if (Guidance::GuidanceEngine::Instance().IsRunning()) {
					ImGui::TextUnformatted("Mission Progress:");
					ImGui::SameLine(col2start);
					ImGui::TextUnformatted(Guidance::GuidanceEngine::Instance().GetMissionProgressStr().c_str());
				}
				
				ImGui::TextUnformatted("Shadow Avoidance:");
				ImGui::SameLine(col2start);
				bool shadowDetectionRunning = ShadowDetection::ShadowDetectionEngine::Instance().IsRunning();
				bool shadowPropagationRunning = ShadowPropagation::ShadowPropagationEngine::Instance().IsRunning();
				if (shadowDetectionRunning && shadowPropagationRunning)
					ImGui::TextColored(ImVec4(0,0.9,0,1), "Enabled");
				else
					ImGui::TextColored(ImVec4(0.9,0,0,1), "Disabled");
			}
			
			if (! Guidance::GuidanceEngine::Instance().IsRunning()) {
				//UI for starting a mission
				
				//Don't try to start shadow modules here since this in non-trivial, but clearly indicate whether starting a mission
				//now will use or not use shadow detection and propogation capabilities.
				
				ImGui::NewLine();
				SurveyRegion * activeSurveyRegion = SurveyRegionManager::Instance().GetActiveSurveyRegion();
				if (activeSurveyRegion == nullptr)
					ImGui::TextUnformatted("Set active survey region to enable mission controls.");
				else {
					//List all connected drones with checkboxes to pick the drones to be used as low-fliers. If a drone is being used
					//by the shadow detection module, don't let it be selected here.
					std::vector<std::string> connectedDroneSerials = DroneInterface::DroneManager::Instance().GetConnectedDroneSerialNumbers();
					std::vector<std::string> availableDroneSerials;
					if (ShadowDetection::ShadowDetectionEngine::Instance().IsRunning()) {
						std::string usedDroneSerial = ShadowDetection::ShadowDetectionEngine::Instance().GetProviderDroneSerial();
						availableDroneSerials.reserve(connectedDroneSerials.size() - 1U);
						for (std::string serial : connectedDroneSerials) {
							if (serial != usedDroneSerial)
								availableDroneSerials.push_back(serial);
						}
					}
					else
						availableDroneSerials = connectedDroneSerials;
					
					if (m_useDroneFlags.size() != availableDroneSerials.size())
						m_useDroneFlags = std::vector<bool>(availableDroneSerials.size(), true); //Re-initialize flag vector
					
					if (availableDroneSerials.empty())
						ImGui::TextUnformatted("No available drones for new mission.");
					else {
						{
							ImExt::Style m_style(StyleCol::Header,  Math::Vector4(0.2f, 0.75f, 0.2f, 1.0f));
							MyGui::HeaderLabel("New Mission");
						}
						ImGui::Text("Survey Region: %s", activeSurveyRegion->m_Name.c_str());
						
						bool atLeast1DroneSelected = false;
						for (size_t n = 0U; n < availableDroneSerials.size(); n++) {
							bool flag = m_useDroneFlags[n];
							std::string label = "Use drone: "s + availableDroneSerials[n];
							//std::string label = availableDroneSerials[n];
							ImGui::Checkbox(label.c_str(), &flag);
							m_useDroneFlags[n] = flag;
							atLeast1DroneSelected = atLeast1DroneSelected || flag;
						}
						
						if (ImGui::BeginMenu("Start Mission", atLeast1DroneSelected)) {
							if (ImGui::MenuItem("Confirm Command", NULL, false, true)) {
								//std::cerr << "Starting Mission!\r\n";
								
								std::vector<std::string> serialsToCommand;
								serialsToCommand.reserve(availableDroneSerials.size());
								for (size_t n = 0U; n < availableDroneSerials.size(); n++) {
									if (m_useDroneFlags[n])
										serialsToCommand.push_back(availableDroneSerials[n]);
								}
								Guidance::GuidanceEngine::Instance().StartSurvey(serialsToCommand);
							}
							ImGui::EndMenu();
						}
					}
				}
			}
			else {
				//UI for a mission in progress
				ImGui::NewLine();
				if (ImGui::BeginMenu("Stop Mission (Drones Hover)")) {
					if (ImGui::MenuItem("Confirm Command", NULL, false, true))
						VehicleControlWidget::Instance().AllDronesStopAndHover();
					ImGui::EndMenu();
				}
			}
			
			
			//ImGui::CloseCurrentPopup();
			
			
			ImGui::EndPopup();
		}
	}
	
	//Watchdog Popup
	{
		//Note: We draw dummies for items we want to hide instead of not drawing anything... this is to keep the window size from changing once the popup is open,
		//which can cause issues with the window manager.
		ImExt::Style styleSitter(StyleVar::WindowPadding, ImVec2(4.0f, 4.0f));
		if (ImGui::BeginPopup("Watchdog Popup", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			MyGui::HeaderLabel("Watchdog Settings");
			
			std::scoped_lock lock(m_watchdowMutex);
			
			float col1Start = ImGui::GetCursorPos().x;
			float col2Start = col1Start + 1.75f*ImGui::GetFontSize();
			float col3Start = col2Start + ImGui::CalcTextSize("Pause drone on Min Safe Altitude hazard  ").x;
			
			ImGui::TextUnformatted(u8"\uf028");
			ImGui::SameLine(col2Start);
			ImGui::TextUnformatted("Play warning sound on hazard");
			ImGui::SameLine(col3Start);
			ImGui::Checkbox("##PlaySoundOnViolation", &m_playSoundOnViolation);
			
			ImGui::TextUnformatted(" ");
			ImGui::SameLine(col2Start);
			ImGui::TextUnformatted("Check Min Safe Altitude");
			ImGui::SameLine(col3Start);
			ImGui::Checkbox("##CheckMSA", &m_CheckMSA);
			
			if (m_CheckMSA) {
				ImGui::TextUnformatted(u8"\uf28b");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Pause drone on Min Safe Altitude hazard");
				ImGui::SameLine(col3Start);
				ImGui::Checkbox("##PauseOnMSAViolation", &m_pauseDronesOnMSAViolation);
				
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
			}
			else {
				ImGui::Dummy(ImVec2(1, ImGui::GetFrameHeight()));
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
			}
			
			ImGui::TextUnformatted(" ");
			ImGui::SameLine(col2Start);
			ImGui::TextUnformatted("Check Vehicle Proximity");
			ImGui::SameLine(col3Start);
			ImGui::Checkbox("##CheckVehicleProximity", &m_CheckVehicleProximity);
			
			if (m_CheckVehicleProximity) {
				ImGui::TextUnformatted(u8"\uf28b");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Pause drones on Proximity hazard");
				ImGui::SameLine(col3Start);
				ImGui::Checkbox("##PauseOnProximityHazard", &m_AutoPauseDronesOnProximityViolation);
				
				ImGui::TextUnformatted(u8"\uf547");
				ImGui::SameLine(col2Start);
				ImGui::TextUnformatted("Required Separation");
				ImGui::SameLine(col3Start);
				ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
				ImGui::DragFloat("##ProximityThreshold", &m_vehicleProximityWarnThreshold, 0.01f, 0.5f, 5.0f, "%.1f m");
				
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
			}
			else {
				ImGui::Dummy(ImVec2(1, ImGui::GetFrameHeight()));
				ImGui::Dummy(ImVec2(1, ImGui::GetFrameHeight()));
				ImGui::SameLine(col3Start);
				ImGui::Dummy(ImVec2(15.0f*ImGui::GetFontSize(), 1));
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
			}
			
			ImGui::TextUnformatted(" ");
			ImGui::SameLine(col2Start);
			ImGui::TextUnformatted("Check Avoidance Zones");
			ImGui::SameLine(col3Start);
			ImGui::Checkbox("##CheckAvoidanceZones", &m_CheckAvoidanceZones);
			
			ImGui::EndPopup();
		}
	}
	
	//Emergency Popup
	{
		ImExt::Style styleSitter(StyleVar::WindowPadding, ImVec2(4.0f, 4.0f));
		if (ImGui::BeginPopup("Emergency Popup", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			float labelMargin = 1.5f*ImGui::GetFontSize();
			if (MyGui::BeginMenu(u8"\uf28b", labelMargin, "All drones Stop & Hover")) {
				if (ImGui::MenuItem("Confirm Command", NULL, false, true))
					VehicleControlWidget::Instance().AllDronesStopAndHover();
				ImGui::EndMenu();
			}
			if (MyGui::BeginMenu(u8"\uf0ab", labelMargin, "All drones Hit the Deck")) {
				if (ImGui::MenuItem("Confirm Command", NULL, false, true))
					VehicleControlWidget::Instance().AllDronesHitTheDeck();
				ImGui::EndMenu();
			}
			
			ImGui::EndPopup();
		}
	}
	
	
	//After drawing, update content height and recommended widget height
	ContentHeight = ImGui::GetCursorPos().y - cursorStartYPos;
	RecommendedHeight = 0.85f*RecommendedHeight + 0.15f*ContentHeight;
	if (std::abs(RecommendedHeight - ContentHeight) < 1.0f)
		RecommendedHeight = ContentHeight;
}




