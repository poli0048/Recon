//This widget lists connected vehicles. It doesn't do anything else yet - more forward-looking
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>
#include <chrono>

//External Includes
#include "../../../handycpp/Handy.hpp"
#include "../../../imgui/app/ImGuiApp.hpp"

//Project Includes
#include "ReconUI.hpp"
#include "MyGui.hpp"
#include "EmbeddedIcons.hpp"
#include "../Modules/DJI-Drone-Interface/DroneManager.hpp"
#include "../Utilities.hpp"
#include "MapWidget.hpp"
#include "../Maps/MapUtils.hpp"
#include "../ProgOptions.hpp"

class VehiclesWidget {
	struct vehicleState {
		std::string                      m_serial;
		float                            m_targetFPS = 1.0f;
		float                            m_FeedZoom = 1.0f; //1 = No crop, 2 = Crop to centeral 50% width and height
		bool                             m_showFeed = false;
		int                              m_callbackHandle = -1; //-1 if not registered
		
		std::mutex                       m_Texmutex;
		bool                             m_TexValid = false;
		ImTextureID                      m_Tex;
		float                            m_TexWidth;
		float                            m_TexHeight;
		DroneInterface::Drone::TimePoint m_TexTimestamp;
		
		vehicleState() = default;
		vehicleState(std::string const & Serial) : m_serial(Serial) { }
		~vehicleState() = default;
	};
	
	public:
		static VehiclesWidget & Instance() { static VehiclesWidget Widget; return Widget; }
		
		//Constructors and Destructors
		VehiclesWidget() : Log(*(ReconUI::Instance().Log)) {
			m_IconTexture_Drone = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_QuadCopterTopView_Light_84x84[0], 84, 84);
			m_IconTexture_DroneWithArrow = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_QuadCopterTopViewWithArrow_Light_96x96[0], 96, 96);
		}
		~VehiclesWidget() {
			ImGuiApp::Instance().DeleteImage(m_IconTexture_Drone);
			ImGuiApp::Instance().DeleteImage(m_IconTexture_DroneWithArrow);
		}
		
		//Public accessors for getting the state of the window
		float GetWidgetRecommendedHeight(void) { return RecommendedHeight; } //Typically equals ContentHeight, but changes smoothly with time for animation.

		inline void Draw();
		inline void DrawMapOverlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
	
	private:
		Journal & Log;
		ImTextureID m_IconTexture_Drone;
		ImTextureID m_IconTexture_DroneWithArrow;
		std::vector<std::string> m_currentDroneSerials; //Updated in Draw()
		std::unordered_map<std::string, vehicleState> m_vehicleStates;
		
		float ContentHeight; //Height of widget content from last draw pass
		float RecommendedHeight; //Recommended height for widget
		
		inline void DrawDroneInteractable(DroneInterface::Drone & drone, size_t DroneIndex);
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
		
		inline void DrawVideoWindows(void);
};

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

inline void VehiclesWidget::DrawMapOverlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
	float droneIconWidth_pixels = ProgOptions::Instance()->DroneIconScale*96.0f;
	
	//Iterate through each connected drone and draw it's last-known position (with yaw info) on the map
	for (size_t n = 0; n < m_currentDroneSerials.size(); n++) {
		DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(m_currentDroneSerials[n]);
		if (drone == nullptr)
			continue;
		
		DroneInterface::Drone::TimePoint Timestamp;
		double Latitude, Longitude, Altitude;
		double Yaw, Pitch, Roll;
		if (drone->GetPosition(Latitude, Longitude, Altitude, Timestamp) && drone->GetOrientation(Yaw, Pitch, Roll, Timestamp)) {
			Eigen::Vector2d dronePos_LatLon(Latitude, Longitude);
			Eigen::Vector2d dronePos_NM = LatLonToNM(dronePos_LatLon);
			ImVec2 dronePos_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(dronePos_NM);
			
			Eigen::Vector2d center(dronePos_ScreenSpace.x, dronePos_ScreenSpace.y);
			Eigen::Vector2d p1_ScreenSpace = center + Eigen::Vector2d(-1.0*droneIconWidth_pixels/2.0f, -1.0*droneIconWidth_pixels/2.0f);
			Eigen::Vector2d p2_ScreenSpace = center + Eigen::Vector2d(     droneIconWidth_pixels/2.0f, -1.0*droneIconWidth_pixels/2.0f);
			Eigen::Vector2d p3_ScreenSpace = center + Eigen::Vector2d(     droneIconWidth_pixels/2.0f,      droneIconWidth_pixels/2.0f);
			Eigen::Vector2d p4_ScreenSpace = center + Eigen::Vector2d(-1.0*droneIconWidth_pixels/2.0f,      droneIconWidth_pixels/2.0f);
			
			Eigen::Matrix2d R;
			R << cos(Yaw), -sin(Yaw),
			     sin(Yaw),  cos(Yaw);
			p1_ScreenSpace = R*(p1_ScreenSpace - center) + center;
			p2_ScreenSpace = R*(p2_ScreenSpace - center) + center;
			p3_ScreenSpace = R*(p3_ScreenSpace - center) + center;
			p4_ScreenSpace = R*(p4_ScreenSpace - center) + center;
			
			ImVec2 p1(p1_ScreenSpace(0), p1_ScreenSpace(1));
			ImVec2 p2(p2_ScreenSpace(0), p2_ScreenSpace(1));
			ImVec2 p3(p3_ScreenSpace(0), p3_ScreenSpace(1));
			ImVec2 p4(p4_ScreenSpace(0), p4_ScreenSpace(1));
			DrawList->AddImageQuad(m_IconTexture_DroneWithArrow, p1, p2, p3, p4);
			
			//Add Text for n+1 under drone
			std::string droneLabel = std::to_string((unsigned int) n + 1U);
			ImVec2 textSize = ImGui::CalcTextSize(droneLabel.c_str());
			
			Eigen::Vector2d p5_ScreenSpace = 0.5*(p3_ScreenSpace + p4_ScreenSpace);
			Eigen::Vector2d v1 = R*Eigen::Vector2d(0.0, 1.0);
			Eigen::Vector2d p6_ScreenSpace = p5_ScreenSpace + v1*double(textSize.x);
			ImVec2 p6(p6_ScreenSpace(0) - textSize.x/2.0f, p6_ScreenSpace(1) - textSize.y/2.0f); //Offset half the text size so p6 represents center of text
			DrawList->AddText(p6, IM_COL32_WHITE, droneLabel.c_str());
		}
	}
}

inline void VehiclesWidget::DrawContextMenu(bool Open, DroneInterface::Drone & drone) {
	float labelMargin = 1.5f*ImGui::GetFontSize();
	std::string popupStrIdentifier = drone.GetDroneSerial() + "-ContextMenu"s;
	
	//Get reference to vehicle state object
	vehicleState & myState(m_vehicleStates.at(drone.GetDroneSerial()));
	
	if (Open)
		ImGui::OpenPopup(popupStrIdentifier.c_str());
	
	if (ImGui::BeginPopup(popupStrIdentifier.c_str())) {
		if (MyGui::BeginMenu(u8"\uf04c", labelMargin, "Hover In Place")) {
			if (ImGui::MenuItem("Confirm Command", NULL, false, true)) {
				std::cerr << "Drone Command to " << drone.GetDroneSerial() << ": Hover In Place.\r\n";
				drone.Hover();
			}
			ImGui::EndMenu();
		}
		if (MyGui::BeginMenu(u8"\uf5af", labelMargin, "Land Now")) {
			if (ImGui::MenuItem("Confirm Command", NULL, false, true)) {
				std::cerr << "Drone Command to " << drone.GetDroneSerial() << ": Land Now.\r\n";
				drone.LandNow();
			}
			ImGui::EndMenu();
		}
		if (MyGui::BeginMenu(u8"\uf015", labelMargin, "Go Home and Land")) {
			if (ImGui::MenuItem("Confirm Command", NULL, false, true)) {
				std::cerr << "Drone Command to " << drone.GetDroneSerial() << ": Go Home and Land.\r\n";
				drone.GoHomeAndLand();	
			}
			ImGui::EndMenu();
		}
		
		ImGui::Separator();
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
							mySimDrone->SetSourceVideoFile(Handy::Paths::ThisExecutableDirectory() / "SimSourceVideo.mov"s);
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
			ImGui::Separator();
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
inline void VehiclesWidget::DrawDroneInteractable(DroneInterface::Drone & drone, size_t DroneIndex) {
	ImDrawList * draw_list = ImGui::GetWindowDrawList();
	draw_list->ChannelsSplit(2);

	auto style = ImGui::GetStyle();
	ImGui::PushID(drone.GetDroneSerial().c_str());

	Math::Vector2 pos = ImGui::GetCursorScreenPos();
	ImGui::SetCursorScreenPos(pos + style.ItemInnerSpacing);

	//Display node foreground first
	draw_list->ChannelsSetCurrent(1);
	
	ImGui::BeginGroup();
	ImGui::Dummy(ImVec2(0.15f*ImGui::GetFontSize(), 1.0f));
	ImGui::SameLine();
	ImGui::Image(m_IconTexture_Drone, ImVec2(2.0f*ImGui::GetFontSize(), 2.0f*ImGui::GetFontSize()));
	ImGui::Dummy(ImVec2(1.15f*ImGui::GetFontSize() - ImGui::CalcTextSize("4").x/2.0f, 1.0f));
	ImGui::SameLine();
	ImGui::Text("%u", (unsigned int) DroneIndex + 1U);
	ImGui::EndGroup();
	
	ImGui::SameLine();
	ImGui::Dummy(ImVec2(0.25f*ImGui::GetFontSize(), 1.0f));
	ImGui::SameLine();
	
	ImGui::BeginGroup();
	ImGui::TextUnformatted(("Serial: "s + drone.GetDroneSerial()).c_str());
	
	std::string FlightModeStr;
	DroneInterface::Drone::TimePoint Timestamp;
	if (drone.GetFlightMode(FlightModeStr, Timestamp))
		AddOutdatedDataNote(FlightModeStr, Timestamp);
	else
		FlightModeStr = "Unknown"s;
	ImGui::TextUnformatted(("Flight Mode: "s + FlightModeStr).c_str());
	
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
		draw_list->AddRectFilled(pos, pos + itemRectSize, ImColor(style.Colors[ImGuiCol_HeaderActive]), 3.0f);
	else
		draw_list->AddRectFilled(pos, pos + itemRectSize, ImColor(style.Colors[ImGuiCol_Header]), 3.0f);

	//Cleanup - pop object ID and merge draw list channels
	ImGui::PopID();
	draw_list->ChannelsMerge();
}

inline void VehiclesWidget::Draw() {
	float cursorStartYPos = ImGui::GetCursorPos().y;
	MyGui::HeaderLabel("Connected Vehicles");
	
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
		
		DrawDroneInteractable(*drone, n);
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


