//Handles the main menu in the primary UI window
//Author: Bryan Poling
//Copyright (c) 2019 Sentek Systems, LLC. All rights reserved. 
#pragma once

//External Includes
#include "../HandyImGuiInclude.hpp"
#include <opencv2/core.hpp>

//Project Includes
#include "ReconUI.hpp"
#include "MapWidget.hpp"
#include "AboutWindow.hpp"
#include "SettingsWindow.hpp"
#include "ConsoleWidget.hpp"
#include "ModalDialogs.hpp"
#include "../Journal.h"
#include "MyGui.hpp"
#include "SimFiducialsWidget.hpp"
#include "../Modules/DJI-Drone-Interface/DroneManager.hpp"
#include "../Modules/GNSS-Receiver/GNSSReceiver.hpp"
#include "../Modules/Shadow-Detection/ShadowDetection.hpp"
#include "../Modules/Shadow-Propagation/ShadowPropagation.hpp"
#include "GNSSReceiverWindow.hpp"
#include "LiveFiducialsWidget.hpp"

class MainMenu {
	public:
		MainMenu() = default;
		~MainMenu() = default;
		static MainMenu & Instance() { static MainMenu menu; return menu; }
		
		void Draw();
	
	private:
		float m_shadowDetectionModuleVidFeedFPS = 1.0f;
};

inline void MainMenu::Draw() {
	float labelMargin = 1.5f*ImGui::GetFontSize();
	
	if (ImGui::BeginMenuBar()) {
		if (ImGui::BeginMenu("File")) {
			if (MyGui::MenuItem(u8"\uf013", labelMargin, "Settings")) {
				SettingsWindow::Instance().Visible = true;
				ImExt::Window::FocusWindow("Settings");
			}
			
			if (MyGui::MenuItem(u8"\uf057", labelMargin, "Exit"))
				ReconUI::Instance().DrawLoopEnabled = false;
			ImGui::EndMenu();
		}
		
		if (ImGui::BeginMenu("View")) {
			ImGui::MenuItem("Draw Data", "", &(MapWidget::Instance().DrawDataTiles));
			ImGui::MenuItem("Draw Data Tooltip", "", &(MapWidget::Instance().DrawDataTooltip));
			
			std::string consoleText = ConsoleWidget::Instance().IsClosedOrClosing() ? "Open Console" : "Close Console";
			if (ImGui::MenuItem(consoleText.c_str()))
				ConsoleWidget::Instance().AppearHideToggle();
			
			if (ImGui::MenuItem("Show Simulation GCP Marker Tool")) {
				SimFiducialsWidget::Instance().m_visible = true;
				ImExt::Window::FocusWindow("Simulation Fiducials Marker");
			}
			
			{
				float cursorX = ImGui::GetCursorPosX();
				float selectableWidth = 16.0f*ImGui::GetFontSize() + ImGui::CalcTextSize("\xef\x9e\xbf").x;
				if (ImGui::Selectable("Show GNSS Receiver Window", false, 0, ImVec2(selectableWidth,0))) {
					GNSSReceiverWindow::Instance().Visible = true;
					ImExt::Window::FocusWindow("GNSS Receiver");
				}
				ImGui::SameLine();
				ImGui::SetCursorPosX(cursorX + 16.0f*ImGui::GetFontSize());
				ImGui::TextUnformatted("\xef\x9e\xbf");
				ImGui::Separator();
			}
			
			//if (ImGui::MenuItem("Show GNSS Receiver Window"))
			//	GNSSReceiverWindow::Instance().Visible = true;
			
			if (ImGui::MenuItem("Show/Hide Demo Window"))
				ReconUI::Instance().show_demo_window = ! ReconUI::Instance().show_demo_window;
			ImGui::EndMenu();
		}
		
		if (ImGui::BeginMenu("Locations")) {
			Eigen::Vector3d GCS_LLA;
			std::chrono::time_point<std::chrono::steady_clock> GCS_Pos_Timestamp;
			if (GNSSReceiver::GNSSManager::Instance().GetPosition_LLA(GCS_LLA, GCS_Pos_Timestamp)) {
				float cursorX = ImGui::GetCursorPosX();
				float selectableWidth = 18.4f*ImGui::GetFontSize() + ImGui::CalcTextSize("\uf057").x;
				if (ImGui::Selectable("Current Location (GNSS)", false, 0, ImVec2(selectableWidth,0))) {
					double const eps = 0.00004; //Radians
					MapWidget::Instance().StartAnimation(GCS_LLA(0) - eps, GCS_LLA(0) + eps, GCS_LLA(1) - eps, GCS_LLA(1) + eps);
				}
				ImGui::SameLine();
				ImGui::SetCursorPosX(cursorX + 18.4f*ImGui::GetFontSize());
				ImGui::TextUnformatted("\uf124");
				ImGui::Separator();
			}
			for (size_t bookmarkIndex = 0U; bookmarkIndex < BookmarkManager::Instance().Bookmarks.size(); bookmarkIndex++) {
				LocationBookmark const & bookmark(BookmarkManager::Instance().Bookmarks[bookmarkIndex]);
				ImGui::PushID((int) bookmarkIndex);
				if (ImGui::Selectable(bookmark.Name.c_str(), false, 0, ImVec2(18.0f*ImGui::GetFontSize(),0)))
					MapWidget::Instance().StartAnimation(bookmark.MinLat, bookmark.MaxLat, bookmark.MinLon, bookmark.MaxLon);
				ImGui::SameLine(0.0f, 0.4f*ImGui::GetFontSize());
				float cursorX = ImGui::GetCursorPosX();
				ImGui::TextUnformatted("\uf057");
				ImGui::SameLine(cursorX);
				if (ImGui::InvisibleButton("DeleteBookmark", ImGui::GetItemRectSize()))
					DeleteBookmarkDialog::Instance().Show(bookmarkIndex);
				ImGui::PopID();
			}
			ImGui::Separator();
			if (ImGui::MenuItem("New Bookmark (Current View)")) {
				Eigen::Vector2d LatBounds, LonBounds;
				MapWidget::Instance().GetCurrentLatLonBounds(LatBounds, LonBounds);
				NewBookmarkDialog::Instance().Show(LatBounds, LonBounds);
			}
			if (ImGui::MenuItem("Navigate To GPS Coords"))
				ZoomToCoordsDialog::Instance().Show();
			
			ImGui::EndMenu();
		}
		
		if (ImGui::BeginMenu("Modules")) {
			bool shadowDetectionRunning = ShadowDetection::ShadowDetectionEngine::Instance().IsRunning();
			bool shadowPropagationRunning = ShadowPropagation::ShadowPropagationEngine::Instance().IsRunning();
			float col2start = ImGui::CalcTextSize("Shadow Propagation  ").x;
			
			std::string   statusStr = shadowDetectionRunning ? "(Running)"s : "(Not Running)"s;
			Math::Vector4 statusCol = shadowDetectionRunning ? ImVec4(0,0.9,0,1) : ImVec4(0.9,0,0,1);
			if (MyGui::BeginMenuWithStatus("Shadow Detection", col2start, statusStr.c_str(), statusCol)) {
				if (shadowDetectionRunning) {
					if (ImGui::MenuItem("Stop Module")) {
						std::string serial = ShadowDetection::ShadowDetectionEngine::Instance().GetProviderDroneSerial();
						ShadowDetection::ShadowDetectionEngine::Instance().Stop();
						
						DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(serial);
						if (drone != nullptr)
							drone->StopDJICamImageFeed();
					}
				}
				else {
					//Get a vector of serials of drones that could be used to serve imagery to the shadow detection module
					std::vector<std::string> connectedDroneSerials = DroneInterface::DroneManager::Instance().GetConnectedDroneSerialNumbers();
					std::vector<std::string> DroneSerialsWithDJICams;
					for (std::string serial : connectedDroneSerials) {
						DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(serial);
						if ((drone != nullptr) && (drone->IsDJICamConnected()))
							DroneSerialsWithDJICams.push_back(serial);
					}
					
					if (DroneSerialsWithDJICams.empty())
						ImGui::TextUnformatted("No connected drones with DJI cams");
					else {
						ImGui::DragFloat("Target FPS", &(m_shadowDetectionModuleVidFeedFPS), 0.02f, 0.1f, 15.0f, "%.1f");
						
						for (std::string serial : DroneSerialsWithDJICams) {
							//std::string menuItemText = "Start using drone: "s + serial;
							if (ImGui::MenuItem(("Start using drone: "s + serial).c_str())) {
								//If the drone is simulated, load the ref frame and GCPs corresponding to the chosen video feed
								//If the drone is real, launch the fiducial marking tool to select a reference frame and mark GCPs
								DroneInterface::Drone * drone = DroneInterface::DroneManager::Instance().GetDrone(serial);
								DroneInterface::SimulatedDrone * mySimDrone = dynamic_cast<DroneInterface::SimulatedDrone *>(drone);
								if (mySimDrone != nullptr) {
									mySimDrone->SetRealTime(true);
									std::filesystem::path datasetPath = mySimDrone->GetSourceVideoFile().parent_path();
									cv::Mat refFrame = GetRefFrame(datasetPath);
									std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> GCPs = LoadFiducialsFromFile(datasetPath);
									ShadowDetection::ShadowDetectionEngine::Instance().SetReferenceFrame(refFrame);
									ShadowDetection::ShadowDetectionEngine::Instance().SetFiducials(GCPs);
									
									//Start (or restart) the cam feed with the specified frame rate
									if (drone->IsCamImageFeedOn())
										drone->StopDJICamImageFeed();
									drone->StartDJICamImageFeed(m_shadowDetectionModuleVidFeedFPS);
									
									//Instruct the shadow detection module to start (using this drone serial)
									ShadowDetection::ShadowDetectionEngine::Instance().Start(serial);
								}
								else {
									//Start (or restart) the cam feed with the specified frame rate
									if (drone->IsCamImageFeedOn())
										drone->StopDJICamImageFeed();
									drone->StartDJICamImageFeed(m_shadowDetectionModuleVidFeedFPS);
									
									LiveFiducialsWidget::Instance().Show(serial);
								}
							}
						}
					}
				}
				
				ImGui::EndMenu();
			}
			statusStr = shadowPropagationRunning ? "(Running)"s : "(Not Running)"s;
			statusCol = shadowPropagationRunning ? ImVec4(0,0.9,0,1) : ImVec4(0.9,0,0,1);
			if (MyGui::BeginMenuWithStatus("Shadow Propagation", col2start, statusStr.c_str(), statusCol, shadowDetectionRunning || shadowPropagationRunning)) {
				if (shadowPropagationRunning) {
					if (ImGui::MenuItem("Stop Module"))
						ShadowPropagation::ShadowPropagationEngine::Instance().Stop();
				}
				else {
					if (ImGui::MenuItem("Start Module"))
						ShadowPropagation::ShadowPropagationEngine::Instance().Start();
				}
				
				ImGui::EndMenu();
			}
			
			ImGui::EndMenu();
		}
		
		if (ImGui::BeginMenu("Simulator")) {
			double PI = 3.14159265358979;
			double const eps = 0.00004; //Radians... used for zooming to drones
			std::filesystem::path DatasetFolder = Handy::Paths::ThisExecutableDirectory().parent_path() / "Simulation-Data-Sets";
			if (MyGui::MenuItem(u8"\uf04b", labelMargin, "Becker (1 Drone)")) {
				double lat = 45.344097*PI/180.0;
				double lon = -93.858990*PI/180.0;
				double alt = 289.56;
				DroneInterface::DroneManager::Instance().ClearSimulatedDrones();
				auto dronePtr = DroneInterface::DroneManager::Instance().AddSimulatedDrone("Simulation A1"s, Eigen::Vector3d(lat, lon, alt));
				if (dronePtr != nullptr)
					dronePtr->SetSourceVideoFile(GetSimVideoFilePath(DatasetFolder / "Dataset-1 (Becker 7-24-2020)"));
				MapWidget::Instance().StartAnimation(lat - eps, lat + eps, lon - eps, lon + eps);
			}
			if (MyGui::MenuItem(u8"\uf04b", labelMargin, "Lamberton (1 Drone)")) {
				double lat = 44.236124*PI/180.0;
				double lon = -95.308418*PI/180.0;
				double alt = 345.03;
				DroneInterface::DroneManager::Instance().ClearSimulatedDrones();
				DroneInterface::DroneManager::Instance().AddSimulatedDrone("Simulation A2"s, Eigen::Vector3d(lat, lon, alt));
				MapWidget::Instance().StartAnimation(lat - eps, lat + eps, lon - eps, lon + eps);
			}
			if (MyGui::MenuItem(u8"\uf04b", labelMargin, "Lamberton (3 Drones)")) {
				double lat = 44.236124*PI/180.0;
				double lon = -95.308418*PI/180.0;
				double alt = 345.03;
				DroneInterface::DroneManager::Instance().ClearSimulatedDrones();
				DroneInterface::DroneManager::Instance().AddSimulatedDrone("Simulation A3"s, Eigen::Vector3d(lat, lon, alt));
				
				lat = 44.236120*PI/180.0;
				lon = -95.308018*PI/180.0;
				DroneInterface::DroneManager::Instance().AddSimulatedDrone("Simulation B3"s, Eigen::Vector3d(lat, lon, alt));
				
				lat = 44.236544*PI/180.0;
				lon = -95.307398*PI/180.0;
				DroneInterface::DroneManager::Instance().AddSimulatedDrone("Simulation C3"s, Eigen::Vector3d(lat, lon, alt));
				MapWidget::Instance().StartAnimation(lat - eps, lat + eps, lon - eps, lon + eps);
			}
			unsigned int NumSimDrones = DroneInterface::DroneManager::Instance().NumSimulatedDrones();
			ImGui::Separator();
			if (MyGui::MenuItem(u8"\uf04d", labelMargin, "End Simulation (Destroy all sim drones)", NULL, false, (NumSimDrones > 0)))
				DroneInterface::DroneManager::Instance().ClearSimulatedDrones();
			ImGui::EndMenu();
		}
		
		if (ImGui::BeginMenu("Help")) {
			if (MyGui::MenuItem(u8"\uf05a", labelMargin, "About")) {
				AboutWindow::Instance().Visible = true;
				ImExt::Window::FocusWindow("About");
			}
			ImGui::EndMenu();
		}
		
		ImGui::EndMenuBar();
	}
}


