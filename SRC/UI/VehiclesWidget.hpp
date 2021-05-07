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
#include "../Maps/MapUtils.hpp"
#include "../ProgOptions.hpp"
#include "../Modules/Guidance/Guidance.hpp"

class VehiclesWidget {
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
		bool                             m_userControlEnabled = false;
		float                            m_targetHAGFeet = 0.0f; //feet
		float                            m_targetYawDeg = 0.0f;
		bool                             m_autoYawOnMove = true;
		float                            m_vehicleSpeedMPH = 15.0f;
		Eigen::Vector2d                  m_targetLatLon = Eigen::Vector2d(std::nan(""), std::nan("")); //Lat and Lon (radians) of target pos
		
		vehicleState() = default;
		vehicleState(std::string const & Serial) : m_serial(Serial) { }
		~vehicleState() = default;
	};
	
	public:
		static VehiclesWidget & Instance() { static VehiclesWidget Widget; return Widget; }
		
		//Constructors and Destructors
		VehiclesWidget() : Log(*(ReconUI::Instance().Log)), m_controlThread(&VehiclesWidget::ControlThreadMain, this) {
			m_IconTexture_Drone = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_QuadCopterTopView_Light_84x84[0], 84, 84);
			m_IconTexture_DroneWithArrow = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_QuadCopterTopViewWithArrow_Light_96x96[0], 96, 96);
			m_IconTexture_HighlightedDrone = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_QuadCopterTopView_LightHighlighted_84x84[0], 84, 84);
			m_IconTexture_HighlightedDroneWithArrow = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_QuadCopterTopViewWithArrow_LightHighlighted_96x96[0], 96, 96);
		}
		~VehiclesWidget() {
			ImGuiApp::Instance().DeleteImage(m_IconTexture_Drone);
			ImGuiApp::Instance().DeleteImage(m_IconTexture_DroneWithArrow);
			ImGuiApp::Instance().DeleteImage(m_IconTexture_HighlightedDrone);
			ImGuiApp::Instance().DeleteImage(m_IconTexture_HighlightedDroneWithArrow);
			m_controlThreadAbort = true;
			if (m_controlThread.joinable())
				m_controlThread.join();
		}
		
		//Public accessors for getting the state of the window
		float GetWidgetRecommendedHeight(void) { return RecommendedHeight; } //Typically equals ContentHeight, but changes smoothly with time for animation.
		
		//The overlay returns true of the map widget should process mouse input and false if we have already processed it and the widget should ignore it
		inline void Draw();
		inline bool DrawMapOverlay(ImVec2 CursorPos_ScreenSpace, Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
	
	private:
		Journal & Log;
		ImTextureID m_IconTexture_Drone;
		ImTextureID m_IconTexture_DroneWithArrow;
		ImTextureID m_IconTexture_HighlightedDrone;
		ImTextureID m_IconTexture_HighlightedDroneWithArrow;
		
		std::thread       m_controlThread;
		std::atomic<bool> m_controlThreadAbort = false;
		
		std::mutex m_dronesAndStatesMutex; //Protects m_currentDroneSerials and m_vehicleStates
		std::vector<std::string> m_currentDroneSerials; //Updated in Draw()
		std::unordered_map<std::string, vehicleState> m_vehicleStates;
		
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
		
		static inline std::filesystem::path GetPathForSimVideoInput(void);
		static inline bool IsDroneHovered(ImVec2 CursorPos_ScreenSpace, Eigen::Vector2d const & drone_ScreenSpace, Eigen::Matrix2d const & R, float IconWidth_pixels);
		
		inline void StartManualControl(DroneInterface::Drone & Drone, vehicleState & State);
		inline void StopManualControl(DroneInterface::Drone & Drone, vehicleState & State);
		inline void DroneCommand_Hover(DroneInterface::Drone & Drone, vehicleState & State);
		inline void DroneCommand_LandNow(DroneInterface::Drone & Drone, vehicleState & State);
		inline void DroneCommand_GoHomeAndLand(DroneInterface::Drone & Drone, vehicleState & State);
		
		inline void DrawVideoWindows(void);
		inline void ControlThreadMain(void);
		
		inline void DrawMission_ManualControl(vehicleState * State, ImVec2 dronePos_ScreenSpace, ImDrawList * DrawList);
		
		//Limited coordinate conversion utilities
		static inline Eigen::Vector3d positionLLA2ECEF(double lat, double lon, double alt);
		static inline Eigen::Matrix3d latLon_2_C_ECEF_ENU(double lat, double lon);
		
};

inline Eigen::Vector3d VehiclesWidget::positionLLA2ECEF(double lat, double lon, double alt) {
	double a = 6378137.0;           //Semi-major axis of reference ellipsoid
	double ecc = 0.081819190842621; //First eccentricity of the reference ellipsoid
	double eccSquared = ecc*ecc;
	double N = a/sqrt(1.0 - eccSquared*sin(lat)*sin(lat));
	double X = (N + alt)*cos(lat)*cos(lon);
	double Y = (N + alt)*cos(lat)*sin(lon);
	double Z = (N*(1 - eccSquared) + alt)*sin(lat);
	return(Eigen::Vector3d(X, Y, Z));
}

inline Eigen::Matrix3d VehiclesWidget::latLon_2_C_ECEF_ENU(double lat, double lon) {
	//Populate C_ECEF_NED
	Eigen::Matrix3d C_ECEF_NED;
	C_ECEF_NED << -sin(lat)*cos(lon), -sin(lat)*sin(lon),  cos(lat),
	              -sin(lon),                    cos(lon),       0.0,
	              -cos(lat)*cos(lon), -cos(lat)*sin(lon), -sin(lat);
	
	//Compute C_ECEF_ENU from C_ECEF_NED
	Eigen::Matrix3d C_NED_ENU;
	C_NED_ENU << 0.0,  1.0,  0.0,
	             1.0,  0.0,  0.0,
	             0.0,  0.0, -1.0;
	Eigen::Matrix3d C_ECEF_ENU = C_NED_ENU * C_ECEF_NED;
	return C_ECEF_ENU;
}

inline void VehiclesWidget::ControlThreadMain(void) {
	double approxLoopPeriod  = 0.15; //seconds
	
	while (! m_controlThreadAbort) {
		double PI = 3.14159265358979;
		m_dronesAndStatesMutex.lock();
		for (size_t n = 0; n < m_currentDroneSerials.size(); n++) {
			std::string serial = m_currentDroneSerials[n];
			DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(serial);
			if (drone == nullptr)
				continue;
			
			//Get reference to vehicle state object - we don't need to check existance; we touch states right after updating serials in the draw loop (while locked)
			vehicleState & myState(m_vehicleStates.at(serial));
			if (! myState.m_userControlEnabled)
				continue;
			
			//If we get here the drone exists and is under maual control
			DroneInterface::VirtualStickCommand_ModeA command;
			command.Yaw     = myState.m_targetYawDeg*PI/180.0;
			command.V_North = 0.0;
			command.V_East  = 0.0;
			command.HAG     = myState.m_targetHAGFeet/3.280839895;
			command.timeout = std::max(2.0, 10.0*approxLoopPeriod);
			
			//If we have up-to-date drone telemetry, we set the EN velocity to hold a desired position
			if ((! std::isnan(myState.m_targetLatLon(0))) && (! std::isnan(myState.m_targetLatLon(1)))) {
				DroneInterface::Drone::TimePoint TS1, TS2;
				double Latitude, Longitude, Altitude, V_North, V_East, V_Down;
				if (drone->GetPosition(Latitude, Longitude, Altitude, TS1) && drone->GetVelocity(V_North, V_East, V_Down, TS2)) {
					double age1 = SecondsElapsed(TS1, std::chrono::steady_clock::now());
					double age2 = SecondsElapsed(TS2, std::chrono::steady_clock::now());
					if ((age1 > 4.0) || (age2 > 4.0))
						std::cerr << "Warning in ControlThreadMain(): Drone telemetry is old! Zeroing velocity.\r\n";
					else {
						//Compute 2D distance from target and unit vector in EN from current position to target position
						Eigen::Vector3d curentPos_ECEF = positionLLA2ECEF(Latitude, Longitude, Altitude);
						Eigen::Vector3d targetPos_ECEF = positionLLA2ECEF(myState.m_targetLatLon(0), myState.m_targetLatLon(1), Altitude);
						Eigen::Vector3d delta_ECEF     = targetPos_ECEF - curentPos_ECEF;
						Eigen::Matrix3d C_ECEF_ENU     = latLon_2_C_ECEF_ENU(Latitude, Longitude);
						Eigen::Vector3d delta_ENU      = C_ECEF_ENU * delta_ECEF;
						Eigen::Vector2d v_EN(delta_ENU(0), delta_ENU(1));
						double distFromTarget          = v_EN.norm();
						v_EN.normalize();
						
						//We will set our East-North velocity to a multiple of v_EN... we just need to scale it appropriately
						double maxSpeed_mps = myState.m_vehicleSpeedMPH * 0.44704f;
						double maxDecel = 3.0; //m/s/s - measured on Inspire 1
						double stoppingDist = maxSpeed_mps*maxSpeed_mps / (2.0 * maxDecel);
						if (distFromTarget > stoppingDist)
							v_EN *= maxSpeed_mps;
						else
							v_EN *= std::sqrt(2.0 * maxDecel * distFromTarget);
						command.V_North = v_EN(1);
						command.V_East  = v_EN(0);
					}
				}
			}
			
			//Issue the virtual stick command
			drone->IssueVirtualStickCommand(command);
		}
		m_dronesAndStatesMutex.unlock();
		
		std::this_thread::sleep_for(std::chrono::milliseconds(int32_t(approxLoopPeriod*1000.0)));
	}
}

inline void VehiclesWidget::DrawVideoWindows(void) {
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
						cv::cvtColor(Frame, Frame_RGBA, CV_BGRA2RGBA, 4);
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
						cv::cvtColor(Frame, Frame_RGBA, CV_BGR2RGBA, 4);
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

inline bool VehiclesWidget::IsDroneHovered(ImVec2 CursorPos_ScreenSpace, Eigen::Vector2d const & drone_ScreenSpace, Eigen::Matrix2d const & R, float IconWidth_pixels) {
	Eigen::Vector2d cursorScreenSpace(CursorPos_ScreenSpace.x, CursorPos_ScreenSpace.y);
	Eigen::Vector2d v = cursorScreenSpace - drone_ScreenSpace;
	Eigen::Vector2d w = R.transpose()*v;
	return ((w(0) >= -0.5f*IconWidth_pixels) && (w(0) <= 0.5f*IconWidth_pixels) &&
	        (w(1) >= -0.5f*IconWidth_pixels) && (w(1) <= 0.5f*IconWidth_pixels));
}

inline void VehiclesWidget::DrawMission_ManualControl(vehicleState * State, ImVec2 dronePos_ScreenSpace, ImDrawList * DrawList) {
	float droneIconWidth_pixels = ProgOptions::Instance()->DroneIconScale*96.0f;
	Eigen::Vector2d targetPos_NM = LatLonToNM(State->m_targetLatLon);
	ImVec2 targetPos_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(targetPos_NM);
	Eigen::Vector2d target(targetPos_ScreenSpace.x, targetPos_ScreenSpace.y);
	Eigen::Vector2d droneCenter(dronePos_ScreenSpace.x, dronePos_ScreenSpace.y);
	Eigen::Vector2d v = target - droneCenter;
	if (v.norm() > droneIconWidth_pixels) {
		//Only draw if the drone isn't essentially on top of it's target position
		int numSegments = (int) std::floor(v.norm() / (3.0 * ImGui::GetFontSize()));
		
		v.normalize();
		Eigen::Vector2d p0 = target - (ImGui::GetFontSize() - 1.0)*v;
		DrawList->AddLine(dronePos_ScreenSpace, ImVec2(p0(0), p0(1)), IM_COL32(180,180,180,128), ImGui::GetFontSize()/3.0f);
		
		Eigen::Matrix2d R;
		R << 0, -1,
			1,  0;
		Eigen::Vector2d w = R*v;
		Eigen::Vector2d p1 = target;
		Eigen::Vector2d p2 = p1 - ImGui::GetFontSize()*v + ImGui::GetFontSize()/2.0f*w;
		Eigen::Vector2d p3 = p1 - ImGui::GetFontSize()*v - ImGui::GetFontSize()/2.0f*w;
		DrawList->AddTriangleFilled(ImVec2(p1(0), p1(1)), ImVec2(p2(0), p2(1)), ImVec2(p3(0), p3(1)), IM_COL32(180,180,180,255));
		
		//For simple animation: t goes from 0 to 1 every 3 seconds and then repeats
		double t = FractionalPart(SecondsSinceT0Epoch(std::chrono::steady_clock::now())/3.0);
		p1 = p1 + t*3.0*ImGui::GetFontSize()*v;
		
		for (int n = 0; n < numSegments; n++) {
			p1 = p1 - 3.0*ImGui::GetFontSize()*v;
			p2 = p1 - ImGui::GetFontSize()*v + ImGui::GetFontSize()/2.0f*w;
			p3 = p1 - ImGui::GetFontSize()*v - ImGui::GetFontSize()/2.0f*w;
			if (n + 1 < numSegments)
				DrawList->AddTriangleFilled(ImVec2(p1(0), p1(1)), ImVec2(p2(0), p2(1)), ImVec2(p3(0), p3(1)), IM_COL32(180,180,180,255));
			else
				DrawList->AddTriangleFilled(ImVec2(p1(0), p1(1)), ImVec2(p2(0), p2(1)), ImVec2(p3(0), p3(1)), IM_COL32(180,180,180,t*255));
		}
	}
}

inline bool VehiclesWidget::DrawMapOverlay(ImVec2 CursorPos_ScreenSpace, Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
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
			ImVec2 dronePos_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(dronePos_NM);
			Eigen::Vector2d drone_ScreenSpace(dronePos_ScreenSpace.x, dronePos_ScreenSpace.y);
			
			Eigen::Matrix2d R;
			R << cos(Yaw), -sin(Yaw),
			     sin(Yaw),  cos(Yaw);
			if (IsDroneHovered(CursorPos_ScreenSpace, drone_ScreenSpace, R, droneIconWidth_pixels)) {
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
							ImVec2 dronePos_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(dronePos_NM);
							
							//All calculations in this block are in screen space
							Eigen::Vector2d cursor(CursorPos_ScreenSpace.x, CursorPos_ScreenSpace.y);
							Eigen::Vector2d drone(dronePos_ScreenSpace.x, dronePos_ScreenSpace.y);
							Eigen::Vector2d v = drone - cursor;
							if (v.norm() > 5.0) {
								v.normalize();
								Eigen::Matrix2d R;
								R << 0, -1,
									1,  0;
								Eigen::Vector2d w = R*v;
								Eigen::Vector2d p1 = cursor;
								Eigen::Vector2d p2 = cursor + ImGui::GetFontSize()*v + ImGui::GetFontSize()/2.0f*w;
								Eigen::Vector2d p3 = cursor + ImGui::GetFontSize()*v - ImGui::GetFontSize()/2.0f*w;
								Eigen::Vector2d p4 = cursor + (ImGui::GetFontSize() - 1.0)*v;
								DrawList->AddLine(dronePos_ScreenSpace, ImVec2(p4(0), p4(1)), IM_COL32_WHITE, ImGui::GetFontSize()/3.0f);
								DrawList->AddTriangleFilled(ImVec2(p1(0), p1(1)), ImVec2(p2(0), p2(1)), ImVec2(p3(0), p3(1)), IM_COL32_WHITE);
							}
						}
					}
				}
				else {
					//Stop drag event - if in map widget bounds, this is a command to move the drone
					if (CursorInBounds) {
						//Set the target location
						state->m_targetLatLon = NMToLatLon(CursorPos_NM);
						
						//If "auto yaw on move" is enabled, set the yaw
						if (state->m_autoYawOnMove) {
							//Compute vector in ENU from current position to target position
							Eigen::Vector3d curentPos_ECEF = positionLLA2ECEF(Latitude, Longitude, Altitude);
							Eigen::Vector3d targetPos_ECEF = positionLLA2ECEF(state->m_targetLatLon(0), state->m_targetLatLon(1), Altitude);
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
		if (ImGui::IsMouseClicked(0) && (droneStates[droneHoveredIndex]->m_userControlEnabled))
			m_indexOfDroneUnderDrag = droneHoveredIndex;
		else if (ImGui::IsMouseClicked(1)) {
			m_indexOfDroneWithContextMenuOpen = droneHoveredIndex;
			ImGui::OpenPopup("Drone Control Popup");
		}
	}
	
	//Iterate through each connected drone and draw it's last-known position (with yaw info) on the map, as well as current control task (move-to or waypoints)
	for (size_t n = 0; n < drones.size(); n++) {
		DroneInterface::Drone::TimePoint Timestamp;
		double Latitude, Longitude, Altitude;
		double Yaw, Pitch, Roll;
		if (drones[n]->GetPosition(Latitude, Longitude, Altitude, Timestamp) && drones[n]->GetOrientation(Yaw, Pitch, Roll, Timestamp)) {
			Eigen::Vector2d dronePos_LatLon(Latitude, Longitude);
			Eigen::Vector2d dronePos_NM = LatLonToNM(dronePos_LatLon);
			ImVec2 dronePos_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(dronePos_NM);
			Eigen::Vector2d droneCenter(dronePos_ScreenSpace.x, dronePos_ScreenSpace.y);
			
			//Draw an indication of the current objective or mission for the drone (target location or current waypoint mission)
			if (droneStates[n]->m_userControlEnabled && (! std::isnan(droneStates[n]->m_targetLatLon(0))) && (! std::isnan(droneStates[n]->m_targetLatLon(1))))
				DrawMission_ManualControl(droneStates[n], dronePos_ScreenSpace, DrawList);
			else {
				//TODO: Check for current waypoint mission and if the drone has one draw it
			}
			
			Eigen::Vector2d p1_ScreenSpace = droneCenter + Eigen::Vector2d(-1.0*droneIconWidth_pixels/2.0f, -1.0*droneIconWidth_pixels/2.0f);
			Eigen::Vector2d p2_ScreenSpace = droneCenter + Eigen::Vector2d(     droneIconWidth_pixels/2.0f, -1.0*droneIconWidth_pixels/2.0f);
			Eigen::Vector2d p3_ScreenSpace = droneCenter + Eigen::Vector2d(     droneIconWidth_pixels/2.0f,      droneIconWidth_pixels/2.0f);
			Eigen::Vector2d p4_ScreenSpace = droneCenter + Eigen::Vector2d(-1.0*droneIconWidth_pixels/2.0f,      droneIconWidth_pixels/2.0f);
			
			Eigen::Matrix2d R;
			R << cos(Yaw), -sin(Yaw),
			     sin(Yaw),  cos(Yaw);
			p1_ScreenSpace = R*(p1_ScreenSpace - droneCenter) + droneCenter;
			p2_ScreenSpace = R*(p2_ScreenSpace - droneCenter) + droneCenter;
			p3_ScreenSpace = R*(p3_ScreenSpace - droneCenter) + droneCenter;
			p4_ScreenSpace = R*(p4_ScreenSpace - droneCenter) + droneCenter;
			
			ImVec2 p1(p1_ScreenSpace(0), p1_ScreenSpace(1));
			ImVec2 p2(p2_ScreenSpace(0), p2_ScreenSpace(1));
			ImVec2 p3(p3_ScreenSpace(0), p3_ScreenSpace(1));
			ImVec2 p4(p4_ScreenSpace(0), p4_ScreenSpace(1));
			if (int(n) == droneHoveredIndex)
				DrawList->AddImageQuad(m_IconTexture_HighlightedDroneWithArrow, p1, p2, p3, p4);
			else
				DrawList->AddImageQuad(m_IconTexture_DroneWithArrow, p1, p2, p3, p4);
			
			//Add Text for n+1 under drone (and manual control indicator)
			std::string droneLabel = std::to_string((unsigned int) n + 1U);
			if (droneStates[n]->m_userControlEnabled)
				droneLabel += u8" \uf11b";
			ImVec2 textSize = ImGui::CalcTextSize(droneLabel.c_str());
			
			Eigen::Vector2d p5_ScreenSpace = 0.5*(p3_ScreenSpace + p4_ScreenSpace);
			Eigen::Vector2d v1 = R*Eigen::Vector2d(0.0, 1.0);
			Eigen::Vector2d p6_ScreenSpace = p5_ScreenSpace + v1*double(textSize.y); //Center point of text
			//ImVec2 p6(p6_ScreenSpace(0) - textSize.x/2.0f, p6_ScreenSpace(1) - textSize.y/2.0f); //Offset half the text size so p6 represents center of text
			//DrawList->AddText(p6, IM_COL32_WHITE, droneLabel.c_str());
			MyGui::AddText(DrawList, p6_ScreenSpace, IM_COL32_WHITE, droneLabel.c_str(), NULL, true, true);
		}
	}
	
	{
		//Draw Drone Control Popup
		ImExt::Style styleSitter(StyleVar::WindowPadding, Math::Vector2(4.0f));
		if (ImGui::BeginPopup("Drone Control Popup", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			if ((m_indexOfDroneWithContextMenuOpen < 0) || (m_indexOfDroneWithContextMenuOpen >= int(droneStates.size()))) {
				m_indexOfDroneWithContextMenuOpen = -1;
				ImGui::CloseCurrentPopup();
			}
			DroneInterface::Drone * drone = drones[m_indexOfDroneWithContextMenuOpen];
			vehicleState * state = droneStates[m_indexOfDroneWithContextMenuOpen];
			
			if (state->m_userControlEnabled) {
				//Popup for drone under manual control
				ImGui::BeginChild("Drag Control Area", ImVec2(15.0f*ImGui::GetFontSize(), 0), true);
				float yStart = ImGui::GetCursorPosY();
				
				ImGui::TextUnformatted("Commanded State");
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
				ImGui::TextUnformatted("Height Above Ground:");
				ImGui::SetNextItemWidth(-1.0f);
				ImGui::DragFloat("##HeightDrag", &(state->m_targetHAGFeet), 0.25f, 0.0f, 400.0f, "%.0f (feet)");
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
				
				ImGui::TextUnformatted("Movement Speed:");
				ImGui::SetNextItemWidth(-1.0f);
				ImGui::DragFloat("##SpeedDrag", &(state->m_vehicleSpeedMPH), 0.05f, 1.0f, 33.6f, "%.1f (mph)");
				ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
				
				ImGui::TextUnformatted("Yaw:");
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
					ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
					ImGui::BeginTooltip();
					ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
					ImGui::TextUnformatted("If checked, the vehicle will yaw to point in the direction of motion each time it is commanded to move.");
					ImGui::PopTextWrapPos();
					ImGui::EndTooltip();
				}
				ImGui::Spacing(); ImGui::Spacing();
				
				if (ImGui::Button("Stop Manual Control & Hover", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
					state->m_userControlEnabled = false;
					m_indexOfDroneWithContextMenuOpen = -1;
					ImGui::CloseCurrentPopup();
				}
				
				float height = ImGui::GetCursorPosY() - yStart;
				ImGui::EndChild();
				
				ImGui::SameLine();
				
				//Make a fancy yaw widget
				ImU32 composCol = IM_COL32(255,100,100,255);
				ImVec2 startPos_ScreenSpace = ImGui::GetCursorScreenPos();
				float widgetSize = height + 2.0f*ImGui::GetStyle().FramePadding.y + ImGui::GetFontSize();
				ImGui::Dummy(ImVec2(widgetSize, widgetSize));
				ImDrawList * draw_list = ImGui::GetWindowDrawList();
				Eigen::Vector2d center(startPos_ScreenSpace.x + widgetSize/2.0f, startPos_ScreenSpace.y + widgetSize/2.0f);
				float radius = 0.45f*widgetSize - ImGui::GetFontSize()/2.0f;
				draw_list->AddCircle(ImVec2(center(0), center(1)), radius, composCol, 80, ImGui::GetFontSize());
				
				//Draw North marker
				Eigen::Vector2d NCenter = center - Eigen::Vector2d(0.0, radius - 0.55*ImGui::GetFontSize());
				draw_list->AddCircleFilled(ImVec2(NCenter(0), NCenter(1)), ImGui::GetFontSize(), composCol, 20);
				MyGui::AddText(draw_list, ImVec2(NCenter(0), NCenter(1)), IM_COL32_BLACK, "N", NULL, true, true);
				
				//Draw South marker
				Eigen::Vector2d SCenter = center + Eigen::Vector2d(0.0, radius - 0.55*ImGui::GetFontSize());
				draw_list->AddCircleFilled(ImVec2(SCenter(0), SCenter(1)), ImGui::GetFontSize(), composCol, 20);
				MyGui::AddText(draw_list, ImVec2(SCenter(0), SCenter(1)), IM_COL32_BLACK, "S", NULL, true, true);
				
				//Draw East marker
				Eigen::Vector2d ECenter = center + Eigen::Vector2d(radius - 0.55*ImGui::GetFontSize(), 0.0);
				draw_list->AddCircleFilled(ImVec2(ECenter(0), ECenter(1)), ImGui::GetFontSize(), composCol, 20);
				MyGui::AddText(draw_list, ImVec2(ECenter(0), ECenter(1)), IM_COL32_BLACK, "E", NULL, true, true);
				
				//Draw West marker
				Eigen::Vector2d WCenter = center - Eigen::Vector2d(radius - 0.55*ImGui::GetFontSize(), 0.0);
				draw_list->AddCircleFilled(ImVec2(WCenter(0), WCenter(1)), ImGui::GetFontSize(), composCol, 20);
				MyGui::AddText(draw_list, ImVec2(WCenter(0), WCenter(1)), IM_COL32_BLACK, "W", NULL, true, true);
				
				//Draw drone icon
				float iconWidth = std::min(0.75f*radius, 2.0f*droneIconWidth_pixels);
				Eigen::Vector2d p1_ScreenSpace = center + Eigen::Vector2d(-1.0*iconWidth/2.0f, -1.0*iconWidth/2.0f);
				Eigen::Vector2d p2_ScreenSpace = center + Eigen::Vector2d(     iconWidth/2.0f, -1.0*iconWidth/2.0f);
				Eigen::Vector2d p3_ScreenSpace = center + Eigen::Vector2d(     iconWidth/2.0f,      iconWidth/2.0f);
				Eigen::Vector2d p4_ScreenSpace = center + Eigen::Vector2d(-1.0*iconWidth/2.0f,      iconWidth/2.0f);
				Eigen::Matrix2d R;
				double PI = 3.14159265358979;
				double yaw = state->m_targetYawDeg*PI/180.0;
				R << cos(yaw), -sin(yaw),
					sin(yaw),  cos(yaw);
				p1_ScreenSpace = R*(p1_ScreenSpace - center) + center;
				p2_ScreenSpace = R*(p2_ScreenSpace - center) + center;
				p3_ScreenSpace = R*(p3_ScreenSpace - center) + center;
				p4_ScreenSpace = R*(p4_ScreenSpace - center) + center;
				ImVec2 p1(p1_ScreenSpace(0), p1_ScreenSpace(1));
				ImVec2 p2(p2_ScreenSpace(0), p2_ScreenSpace(1));
				ImVec2 p3(p3_ScreenSpace(0), p3_ScreenSpace(1));
				ImVec2 p4(p4_ScreenSpace(0), p4_ScreenSpace(1));
				draw_list->AddImageQuad(m_IconTexture_DroneWithArrow, p1, p2, p3, p4);
				
				//If the mouse is over the indicator, allow setting yaw through a click
				float dx = CursorPos_ScreenSpace.x - (float) center(0);
				float dy = CursorPos_ScreenSpace.y - (float) center(1);
				if (std::sqrt(dx*dx + dy*dy) <= radius) {
					if (ImGui::IsMouseDown(0)) {
						state->m_targetYawDeg = float(std::atan2(double(dy), double(dx))*180.0/PI + 90.0);
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
						if (ImGui::MenuItem("Confirm Command", NULL, false, true))
							DroneCommand_Hover(*drone, *state);
						ImGui::EndMenu();
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
					if (MyGui::BeginMenu(u8"\uf11b", labelMargin, "Manual Control")) {
						if (state->m_userControlEnabled) {
							if (ImGui::MenuItem("Stop Manual Control and Hover In Place", NULL, false, true))
								StopManualControl(*drone, *state);
						}
						else {
							if (ImGui::MenuItem("Start Manual Control", NULL, false, true))
								StartManualControl(*drone, *state);
						}
						ImGui::EndMenu();
					}
				}
			}
			
			ImGui::EndPopup();
		}
	}
	
	//We need to return true if the map widget still needs to process mouse inputs (i.e. if we haven't stolen them)
	return (! ImGui::IsPopupOpen("Drone Control Popup")) && (m_indexOfDroneUnderDrag < 0);
}

//Get the path of the first .MOV file in the first sim data set folder in "Simulation-Data-Sets" ("first" in a number-aware lexicographical sense)
inline std::filesystem::path VehiclesWidget::GetPathForSimVideoInput(void) {
	auto datasetDirs = Handy::SubDirectories(Handy::Paths::ThisExecutableDirectory().parent_path() / "Simulation-Data-Sets"s);
	std::sort(datasetDirs.begin(), datasetDirs.end(), [](std::string const & A, std::string const & B) -> bool { return StringNumberAwareCompare_LessThan(A, B); });
	if (datasetDirs.empty())
		return std::filesystem::path();
	std::vector<std::filesystem::path> files = GetNormalFilesInDirectory(datasetDirs[0]);
	std::sort(files.begin(), files.end(), [](std::string const & A, std::string const & B) -> bool { return StringNumberAwareCompare_LessThan(A, B); });
	for (auto const & file : files) {
		std::string ext = file.extension().string();
		if ((ext == ".mov"s) || (ext == ".MOV") || (ext == ".Mov"))
			return file;
	}
	return std::filesystem::path();
}

inline void VehiclesWidget::StartManualControl(DroneInterface::Drone & Drone, vehicleState & State) {
	std::cerr << "Starting manual control for drone: " << Drone.GetDroneSerial() << ".\r\n";
	
	//Instruct the guidance module to stop commanding this drone (if it currently is) and hover the drone to cancel any current mission
	Guidance::GuidanceEngine::Instance().RemoveLowFlier(Drone.GetDroneSerial());
	Drone.Hover();
	
	//Initialize the target position to the drone's current position (or NaN if unknown to zero out commanded velocity)
	DroneInterface::Drone::TimePoint Timestamp;
	double Latitude, Longitude, Altitude;
	State.m_targetLatLon << std::nan(""), std::nan("");
	if (Drone.GetPosition(Latitude, Longitude, Altitude, Timestamp) && (SecondsElapsed(Timestamp, std::chrono::steady_clock::now()) < 5.0))
		State.m_targetLatLon << Latitude, Longitude;
	else
		std::cerr << "Warning: Starting manual control without full & current telemetry. No initial position target.\r\n";
	
	//Initialize the target yaw and height above ground based on current state - then turn on manual control
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
	State.m_userControlEnabled = true;
}

inline void VehiclesWidget::StopManualControl(DroneInterface::Drone & Drone, vehicleState & State) {
	std::cerr << "Stopping manual control for drone: " << Drone.GetDroneSerial() << ".\r\n";
	State.m_userControlEnabled = false;
	Drone.Hover();
}

inline void VehiclesWidget::DroneCommand_Hover(DroneInterface::Drone & Drone, vehicleState & State) {
	std::cerr << "Drone Command to " << Drone.GetDroneSerial() << ": Hover In Place.\r\n";
	Guidance::GuidanceEngine::Instance().RemoveLowFlier(Drone.GetDroneSerial()); //Tell the guidance module to stop commanding drone
	State.m_userControlEnabled = false; //Stop our control loop from commanding the drone
	Drone.Hover();
}

inline void VehiclesWidget::DroneCommand_LandNow(DroneInterface::Drone & Drone, vehicleState & State) {
	std::cerr << "Drone Command to " << Drone.GetDroneSerial() << ": Land Now.\r\n";
	Guidance::GuidanceEngine::Instance().RemoveLowFlier(Drone.GetDroneSerial()); //Tell the guidance module to stop commanding drone
	State.m_userControlEnabled = false; //Stop our control loop from commanding the drone
	Drone.LandNow();
}

inline void VehiclesWidget::DroneCommand_GoHomeAndLand(DroneInterface::Drone & Drone, vehicleState & State) {
	std::cerr << "Drone Command to " << Drone.GetDroneSerial() << ": Go Home and Land.\r\n";
	Guidance::GuidanceEngine::Instance().RemoveLowFlier(Drone.GetDroneSerial()); //Tell the guidance module to stop commanding drone
	State.m_userControlEnabled = false; //Stop our control loop from commanding the drone
	Drone.GoHomeAndLand();
}

inline void VehiclesWidget::DrawContextMenu(bool Open, DroneInterface::Drone & drone) {
	float labelMargin = 1.5f*ImGui::GetFontSize();
	std::string popupStrIdentifier = drone.GetDroneSerial() + "-ContextMenu"s;
	
	//Get reference to vehicle state object
	vehicleState & myState(m_vehicleStates.at(drone.GetDroneSerial()));
	
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
				if (ImGui::MenuItem("Confirm Command", NULL, false, true))
					DroneCommand_Hover(drone, myState);
				ImGui::EndMenu();
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
			if (MyGui::BeginMenu(u8"\uf11b", labelMargin, "Manual Control")) {
				if (myState.m_userControlEnabled) {
					if (ImGui::MenuItem("Stop Manual Control and Hover In Place", NULL, false, true))
						StopManualControl(drone, myState);
				}
				else {
					if (ImGui::MenuItem("Start Manual Control", NULL, false, true))
						StartManualControl(drone, myState);
				}
				ImGui::EndMenu();
			}
		}
		
		ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
		if (drone.IsDJICamConnected()) {
			if (MyGui::BeginMenu(u8"\uf03d", labelMargin, "Video Feed")) {
				if (drone.IsCamImageFeedOn()) {
					if (MyGui::MenuItem(u8"\uf03d", labelMargin, "Stop Feed", NULL, false, true))
						drone.StopDJICamImageFeed();
				}
				else {
					ImGui::DragFloat("Target FPS", &(myState.m_targetFPS), 0.02f, 0.1f, 15.0f, "%.1f");
					if (ImGui::MenuItem("Start Feed", NULL, false, true)) {
						//If the drone is actually a simulated drone we need to set the source video file
						DroneInterface::SimulatedDrone * mySimDrone = dynamic_cast<DroneInterface::SimulatedDrone *>(& drone);
						if (mySimDrone != nullptr) {
							mySimDrone->SetRealTime(true);
							mySimDrone->SetSourceVideoFile(GetPathForSimVideoInput());
						}
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
				PrintAgeWarning(Timestamp, col3Start);
				
				ImGui::TextUnformatted("East Velocity: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.1f mph", V_East*2.23694);
				PrintAgeWarning(Timestamp, col3Start);
				
				ImGui::TextUnformatted("Down Velocity: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%.1f mph", V_Down*2.23694);
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
			
			uint16_t WaypointMissionID;
			if (drone.GetCurrentWaypointMissionID(WaypointMissionID, Timestamp)) {
				ImGui::TextUnformatted("Waypoint Mission ID: ");
				ImGui::SameLine(col2Start);
				ImGui::Text("%u", (unsigned int) WaypointMissionID);
				PrintAgeWarning(Timestamp, col3Start);
			}
			
			ImGui::EndMenu();
		}
		
		ImGui::EndPopup();
	}
}

//TODO: So... drone should be passed as a const & but we need the drone class to be const-correct. Do this later to avoid disrupting Ben's work
inline void VehiclesWidget::DrawDroneInteractable(DroneInterface::Drone & drone, vehicleState & State, size_t DroneIndex) {
	ImDrawList * draw_list = ImGui::GetWindowDrawList();
	draw_list->ChannelsSplit(2);

	auto style = ImGui::GetStyle();
	ImGui::PushID(drone.GetDroneSerial().c_str());

	Math::Vector2 pos = ImGui::GetCursorScreenPos();
	ImGui::SetCursorScreenPos(pos + style.ItemInnerSpacing);

	//Display node foreground first
	draw_list->ChannelsSetCurrent(1);
	
	float startX = ImGui::GetCursorPosX();
	ImGui::Image(m_IconTexture_Drone, ImVec2(2.0f*ImGui::GetFontSize(), 2.0f*ImGui::GetFontSize()));
	//ImVec2 p0 = ImGui::GetCursorScreenPos();
	//ImVec2 p1(p0.x + 2.0f*ImGui::GetFontSize(), p0.y + 2.0f*ImGui::GetFontSize());
	//draw_list->AddImage(m_IconTexture_Drone, p0, p1);
	//ImGui::Dummy(ImVec2(1,1));
	ImGui::SameLine(startX + 2.4f*ImGui::GetFontSize());
	std::string droneNumStr = std::to_string((unsigned int) DroneIndex + 1U);
	ImGui::BeginGroup();
	if (State.m_userControlEnabled) {
		ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (ImGui::CalcTextSize("\uf11b").x - ImGui::CalcTextSize(droneNumStr.c_str()).x)/2.0f);
		ImGui::TextUnformatted(droneNumStr.c_str());
		ImGui::TextUnformatted("\uf11b");
	}
	else {
		ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 0.5f*ImGui::GetFontSize());
		ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (ImGui::CalcTextSize("\uf11b").x - ImGui::CalcTextSize(droneNumStr.c_str()).x)/2.0f);
		ImGui::TextUnformatted(droneNumStr.c_str());
	}
	ImGui::EndGroup();
	
	ImGui::SameLine(startX + 4.0f*ImGui::GetFontSize());
	ImGui::BeginGroup();
	std::string FlightModeStr;
	DroneInterface::Drone::TimePoint Timestamp;
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
	Math::Vector2 itemRectSize(ImGui::GetContentRegionAvail().x, contentHeight);
	
	//Display node background next
	draw_list->ChannelsSetCurrent(0);
	
	ImGui::InvisibleButton("node", itemRectSize);
	if (ImGui::IsItemHovered())
		//draw_list->AddRectFilled(pos, pos + itemRectSize, ImColor(style.Colors[ImGuiCol_HeaderActive]), 3.0f);
		draw_list->AddRectFilled(pos, pos + itemRectSize, IM_COL32(120,120,80,255), 3.0f);
	else
		draw_list->AddRectFilled(pos, pos + itemRectSize, ImColor(style.Colors[ImGuiCol_Header]), 3.0f);

	//Cleanup - pop object ID and merge draw list channels
	ImGui::PopID();
	draw_list->ChannelsMerge();
}

inline void VehiclesWidget::Draw() {
	float cursorStartYPos = ImGui::GetCursorPos().y;
	MyGui::HeaderLabel("Connected Vehicles");
	
	std::scoped_lock lock(m_dronesAndStatesMutex); //Lock vector of drone serials and states
	
	//Get list of connected drones
	m_currentDroneSerials = DroneInterface::DroneManager::Instance().GetConnectedDroneSerialNumbers();
	std::sort(m_currentDroneSerials.begin(), m_currentDroneSerials.end());
	
	//Temporary - Add simulated drone to the drone serials vector (this one isn't advertised by the DroneManager module)
	m_currentDroneSerials.push_back("Simulation"s);
	
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
		DrawContextMenu(ImGui::IsItemClicked(1), *drone);
	}
	
	//After drawing, update content height and recommended widget height
	ContentHeight = ImGui::GetCursorPos().y - cursorStartYPos;
	RecommendedHeight = 0.85f*RecommendedHeight + 0.15f*ContentHeight;
	if (std::abs(RecommendedHeight - ContentHeight) < 1.0f)
		RecommendedHeight = ContentHeight;
	
	//Finally, draw any live video feed windows (if open)
	DrawVideoWindows();
}


