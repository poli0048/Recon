//This widget lets you view, mark, and edit fiducials in data sets used for simulations
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>
#include <chrono>

//External Includes
#include "../HandyImGuiInclude.hpp"
#include "../../../nativefiledialog/src/include/nfd.h"
#include <opencv2/opencv.hpp>

//Project Includes
#include "../Utilities.hpp"

class SimFiducialsWidget {
	private:
		bool m_datasetSelected = false;
		std::filesystem::path m_datasetPath;
		bool m_referenceFrameLoaded = false;
		cv::Mat m_RefFrame;
		ImVec2 IMViewUV0; //The UV Coords of the upper-left corner of the view of the ref frame
		ImVec2 IMViewUV1; //The UV Coords of the lower-right corner of the view of the ref frame
		ImTextureID m_RefFrameTex;
		
		int m_gcpIndexBeingDragged = -1;
		bool m_panning = false;
		ImVec2 m_mousePosOnPanStart_ScreenSpace;
		ImVec2 m_IMViewUV0OnPanStart;
		ImVec2 m_IMViewUV1OnPanStart;
		
		bool m_firstDrawPass = true; //Used to set column widths in first draw pass
		
		std::Evector<std::tuple<std::string, Eigen::Vector2d>> m_fiducials; //<Name, PixCoords>, where PixCoords = (col, row)
		std::vector<std::tuple<std::string, std::string, std::string>> m_fiducialCoordStrings; //Lat (deg), Lon (deg), Alt (m)
		
		inline void LoadRefFrame(void);
		inline void UnloadRefFrame(void);
		inline void LoadGCPs(void);
		inline void SaveGCPs(void);
		
		inline Eigen::Vector2d ScreenSpaceToImageCoords(ImVec2 Pos_ScreenSpace, ImVec2 ULCorner_ScreenSpace, ImVec2 LRCorner_ScreenSpace);
		inline ImVec2 ImageCoordsToScreenSpace(Eigen::Vector2d const & Pos_ImageCoords, ImVec2 ULCorner_ScreenSpace, ImVec2 LRCorner_ScreenSpace);
		
		inline std::string GetNewGCPName(void);
		inline void SanitizeUVBounds(void);
		
	public:
		static SimFiducialsWidget & Instance() { static SimFiducialsWidget widget; return widget; }
		
		SimFiducialsWidget() = default;
		~SimFiducialsWidget() = default;
		
		inline void Draw();
		
		bool m_visible = false;
};

inline void SimFiducialsWidget::LoadRefFrame(void) {
	m_referenceFrameLoaded = false;
	std::filesystem::path refFramePath = m_datasetPath / "RefFrame.jpeg"s;
	if (! std::filesystem::exists(refFramePath))
		return;
	
	m_RefFrame = cv::imread(refFramePath.string(), cv::IMREAD_COLOR);
	cv::Mat refFrame_RGBA(m_RefFrame.size(), CV_8UC4);
	cv::cvtColor(m_RefFrame, refFrame_RGBA, cv::COLOR_BGRA2RGBA, 4);
	m_RefFrameTex = ImGuiApp::Instance().CreateImageRGBA8888(refFrame_RGBA.ptr(), refFrame_RGBA.cols, refFrame_RGBA.rows);
	IMViewUV0 = ImVec2(0, 0);
	IMViewUV1 = ImVec2(1, 1);
	m_referenceFrameLoaded = true;
}

inline void SimFiducialsWidget::UnloadRefFrame(void) {
	m_referenceFrameLoaded = false;
	m_RefFrame = cv::Mat();
	IMViewUV0 = ImVec2(0, 0);
	IMViewUV1 = ImVec2(1, 1);
	ImGuiApp::Instance().DeleteImage(m_RefFrameTex);
}

inline void SimFiducialsWidget::LoadGCPs(void) {
	m_fiducials.clear();
	m_fiducialCoordStrings.clear();
	std::filesystem::path GCPsFilePath = m_datasetPath / "GCPs.txt"s;
	if (! std::filesystem::exists(GCPsFilePath))
		return;
	
	//Load the file and parse it
	std::ifstream file;
	file.open(GCPsFilePath.string().c_str());
	if (! file.good())
		return;
	
	std::string line;
	char character;
	size_t index;
	while (!file.eof()) {
		std::getline(file, line);
		index = line.find_first_not_of(" \t"s);
		if (index != std::string::npos) {
			//Line is non-empty. Check if it is a comment line
			character = line.at(index);
			if (character != '#') {
				//Line is non-empty and not a comment line
				line = line.substr(index);
				std::vector<std::string> parts = StringSplit(line, ","s);
				if (parts.size() != 6U)
					std::cerr << "Warning in LoadGCPs(): Dropping invalid line.\r\n";
				else {
					std::string name = parts[0];
					StringStrip(parts[3]);
					StringStrip(parts[4]);
					StringStrip(parts[5], " \t\r\n");
					double col, row;
					if (str2double(parts[1], col) && str2double(parts[2], row)) {
						Eigen::Vector2d PixCoords(col, row);
						m_fiducials.push_back(std::make_tuple(name, PixCoords));
						m_fiducialCoordStrings.push_back(std::make_tuple(parts[3], parts[4], parts[5]));
					}
				}
			}
		}
	}
}

inline void SimFiducialsWidget::SaveGCPs(void) {
	// Create the file stream and check its status
	std::filesystem::path GCPsFilePath = m_datasetPath / "GCPs.txt"s;
	std::ofstream stream(GCPsFilePath.string().c_str(), std::ios::out | std::ios::binary);
	if (! stream.is_open()) {
		std::cerr << "Error in SaveGCPs(): Could not open file for writing: " << GCPsFilePath.string() << "\r\n";
		return;
	}
	
	//Write header
	stream << "#This file holds the GPS locations of ground control points and their corresponding locations (column and row) in the reference image.\r\n";
	stream << "#The center of the upper-left pixel is taken to have column 0 and row 0. Rows increase as you move down and columns increase as you move right.\r\n";
	stream << "#The structure of this file should not be changed since it will be parsed programatically.\r\n";
	stream << "#\r\n";
	stream << "#Identifier, Column in ref image (pixels), Row in ref image (pixels), Latitude (Decimal Degrees), Longitude (Decimal Degrees), WGS-84 Altitude (m)\r\n";
	
	//Write fiducials
	for (size_t n = 0U; n < m_fiducials.size(); n++) {
		auto const & fiducial(m_fiducials[n]);
		auto const & coords(m_fiducialCoordStrings[n]);
		Eigen::Vector2d const & PixCoords(std::get<1>(fiducial));
		stream << std::get<0>(fiducial) << ", ";
		stream << PixCoords(0) << ", " << PixCoords(1) << ", ";
		stream << std::get<0>(coords) << ", " << std::get<1>(coords) << ", " << std::get<2>(coords) << "\r\n";
	}
	stream.flush();
	stream.close();
}

//Returns (col, row) in image corresponding to a point in screen space
inline Eigen::Vector2d SimFiducialsWidget::ScreenSpaceToImageCoords(ImVec2 Pos_ScreenSpace, ImVec2 ULCorner_ScreenSpace, ImVec2 LRCorner_ScreenSpace) {
	float xNorm = (Pos_ScreenSpace.x - ULCorner_ScreenSpace.x)/(LRCorner_ScreenSpace.x - ULCorner_ScreenSpace.x);
	float yNorm = (Pos_ScreenSpace.y - ULCorner_ScreenSpace.y)/(LRCorner_ScreenSpace.y - ULCorner_ScreenSpace.y);
	
	float U = IMViewUV0.x + xNorm*(IMViewUV1.x - IMViewUV0.x);
	float V = IMViewUV0.y + yNorm*(IMViewUV1.y - IMViewUV0.y);
	return Eigen::Vector2d(U*m_RefFrame.cols - 0.5, V*m_RefFrame.rows - 0.5);
}

inline ImVec2 SimFiducialsWidget::ImageCoordsToScreenSpace(Eigen::Vector2d const & Pos_ImageCoords, ImVec2 ULCorner_ScreenSpace, ImVec2 LRCorner_ScreenSpace) {
	float U = (float(Pos_ImageCoords(0)) + 0.5f)/m_RefFrame.cols;
	float V = (float(Pos_ImageCoords(1)) + 0.5f)/m_RefFrame.rows;
	
	float xNorm = (U - IMViewUV0.x)/(IMViewUV1.x - IMViewUV0.x);
	float yNorm = (V - IMViewUV0.y)/(IMViewUV1.y - IMViewUV0.y);
	
	float pos_ScreenSpace_X = xNorm*(LRCorner_ScreenSpace.x - ULCorner_ScreenSpace.x) + ULCorner_ScreenSpace.x;
	float pos_ScreenSpace_Y = yNorm*(LRCorner_ScreenSpace.y - ULCorner_ScreenSpace.y) + ULCorner_ScreenSpace.y;
	return ImVec2(pos_ScreenSpace_X, pos_ScreenSpace_Y);
}

inline void SimFiducialsWidget::SanitizeUVBounds(void) {
	float dx = IMViewUV1.x - IMViewUV0.x;
	float dy = IMViewUV1.y - IMViewUV0.y;
	//Change zoom level if needed without changing the aspect ratio - adjust about center pixel
	if ((dx > 1.0f) || (dy > 1.0f)) {
		float scale = 1.0f/std::max(dx, dy);
		float xc = 0.5f*IMViewUV0.x + 0.5f*IMViewUV1.x;
		float yc = 0.5f*IMViewUV0.y + 0.5f*IMViewUV1.y;
		IMViewUV0 = ImVec2(xc - 0.5f*scale*dx, yc - 0.5f*scale*dy);
		IMViewUV1 = ImVec2(xc + 0.5f*scale*dx, yc + 0.5f*scale*dy);
		dx = IMViewUV1.x - IMViewUV0.x;
		dy = IMViewUV1.y - IMViewUV0.y;
	}
	
	//Adjust pan as needed
	if (IMViewUV0.x < 0.0f) {
		IMViewUV1.x -= IMViewUV0.x;
		IMViewUV0.x  = 0.0f;
	}
	if (IMViewUV0.y < 0.0f) {
		IMViewUV1.y -= IMViewUV0.y;
		IMViewUV0.y  = 0.0f;
	}
	if (IMViewUV1.x > 1.0f) {
		IMViewUV0.x -= (IMViewUV1.x - 1.0f);
		IMViewUV1.x  = 1.0f;
	}
	if (IMViewUV1.y > 1.0f) {
		IMViewUV0.y -= (IMViewUV1.y - 1.0f);
		IMViewUV1.y  = 1.0f;
	}
}

inline std::string SimFiducialsWidget::GetNewGCPName(void) {
	//Try to find the first unused single letter
	for (int num = 0; num < 26; num++) {
		char letter = char(65 + num);
		std::string name(1, letter);
		bool inUse = false;
		for (auto const & item : m_fiducials) {
			if (name == std::get<0>(item)) {
				inUse = true;
				break;
			}
		}
		if (! inUse)
			return name;
	}
	//Try to find the first unused double letter
	for (int num = 0; num < 26; num++) {
		char letter = char(65 + num);
		std::string name(2, letter);
		bool inUse = false;
		for (auto const & item : m_fiducials) {
			if (name == std::get<0>(item)) {
				inUse = true;
				break;
			}
		}
		if (! inUse)
			return name;
	}
	return "New GCP"s;
}

inline void SimFiducialsWidget::Draw() {
	if (! m_visible)
		return;
	
	ImExt::Window::Options WinOpts;
	WinOpts.Flags = WindowFlags::NoTitleBar | WindowFlags::NoResize | WindowFlags::NoMove | WindowFlags::NoCollapse | WindowFlags::NoSavedSettings |
	                WindowFlags::NoDocking;
	WinOpts.POpen = &(m_visible);
	WinOpts.Size(ImVec2(640.0f, 360.0f + 1.5f*ImGui::GetFontSize()), Condition::Once);
	if (ImExt::Window window("Simulation Fiducials Marker"s, WinOpts); window.ShouldDrawContents()) {
		if (! m_datasetSelected) {
			//Dataset selection mode
			if (ImGui::Button("Select Dataset")) {
				std::string DefaultPath = (Handy::Paths::ThisExecutableDirectory().parent_path() / "Simulation-Data-Sets"s).string();
				nfdchar_t * DatasetPath = NULL;
				nfdresult_t result = NFD_PickFolder(DefaultPath.c_str(), &DatasetPath);
				if (result == NFD_OKAY) {
					m_datasetSelected = true;
					m_datasetPath = std::filesystem::path(DatasetPath);
					free(DatasetPath);
					
					//Try to load the reference frame and GCPs
					LoadRefFrame();
					LoadGCPs();
				}
			}
		}
		else if (! m_referenceFrameLoaded) {
			//Select ref frame
			ImGui::TextUnformatted("Reference frame selection not implemented yet - Manually create RefFrame.jpeg");
		}
		else {
			//Main view - show fiducials on imagery
			if (ImGui::Button("Save & Close Dataset")) {
				SaveGCPs();
				m_fiducials.clear();
				m_fiducialCoordStrings.clear();
				UnloadRefFrame();
				m_datasetSelected = false;
			}
			ImGui::SameLine();
			if (ImGui::Button("Close Dataset Without Saving")) {
				m_fiducials.clear();
				m_fiducialCoordStrings.clear();
				UnloadRefFrame();
				m_datasetSelected = false;
			}
			ImGui::SameLine();
			ImGui::TextDisabled(u8"\uf059");
			if (ImGui::IsItemHovered()) {
				ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
				ImGui::BeginTooltip();
				ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
				ImGui::TextUnformatted("Press 'n' to create new GCP under cursor.\nPress 'backspace' while hovering over GCP to delete it.");
				ImGui::PopTextWrapPos();
				ImGui::EndTooltip();
			}
			
			ImGui::Columns(2, "Sim Fiducial Marking Win H Columns");
			ImVec2 regionAvail = ImGui::GetContentRegionAvail();
			float scale1 = (regionAvail.x - 1.5f*ImGui::GetStyle().ItemSpacing.x) / float(m_RefFrame.cols);
			float scale2 = regionAvail.y / float(m_RefFrame.rows);
			float scale = std::min(scale1, scale2);
			ImVec2 TexSize(scale*float(m_RefFrame.cols), scale*float(m_RefFrame.rows));
			ImGui::Image(m_RefFrameTex, TexSize, IMViewUV0, IMViewUV1, ImVec4(1,1,1,1), ImVec4(0,0,0,0));
			//ImGui::Image(m_RefFrameTex, TexSize, ImVec2(0,0), ImVec2(1,1), ImVec4(1,1,1,1), ImVec4(0,0,0,0));
			bool ImageHovered = ImGui::IsItemHovered();
			ImVec2 ImageULCorner_ScreenSpace = ImGui::GetItemRectMin();
			ImVec2 ImageLRCorner_ScreenSpace = ImGui::GetItemRectMax();
			
			ImVec2 mousePosScreenSpace = ImGui::GetMousePos();
			Eigen::Vector2d mousePosImageCoords = ScreenSpaceToImageCoords(mousePosScreenSpace, ImageULCorner_ScreenSpace, ImageLRCorner_ScreenSpace);
			
			ImDrawList * draw_list = ImGui::GetWindowDrawList(); //Current draw list being assembled for this frame
			
			float GCPCircleRadiusPixels = 5.0f;
			
			//See if the mouse is hovered over an item
			int hoveredIndex = -1;
			for (int n = 0; n < (int) m_fiducials.size(); n++) {
				ImVec2 GCP_ScreenSpace = ImageCoordsToScreenSpace(std::get<1>(m_fiducials[n]), ImageULCorner_ScreenSpace, ImageLRCorner_ScreenSpace);
				float dx = GCP_ScreenSpace.x - mousePosScreenSpace.x;
				float dy = GCP_ScreenSpace.y - mousePosScreenSpace.y;
				float dist = std::sqrt(dx*dx + dy*dy);
				if (dist <= GCPCircleRadiusPixels) {
					hoveredIndex = n;
					break;
				}
			}
			
			//Use primitives to add GCP markers
			for (int n = 0; n < (int) m_fiducials.size(); n++) {
				if (n != m_gcpIndexBeingDragged) {
					ImVec2 GCP_ScreenSpace = ImageCoordsToScreenSpace(std::get<1>(m_fiducials[n]), ImageULCorner_ScreenSpace, ImageLRCorner_ScreenSpace);
					if ((GCP_ScreenSpace.x < ImageULCorner_ScreenSpace.x) || (GCP_ScreenSpace.x > ImageLRCorner_ScreenSpace.x))
						continue;
					if ((GCP_ScreenSpace.y < ImageULCorner_ScreenSpace.y) || (GCP_ScreenSpace.y > ImageLRCorner_ScreenSpace.y))
						continue;
					ImU32 circleColor = IM_COL32(255, 40, 40, 255);
					if (n == hoveredIndex)
						circleColor = IM_COL32(255, 110, 110, 255);
					draw_list->AddCircleFilled(GCP_ScreenSpace, GCPCircleRadiusPixels, circleColor, 30);
					std::string GCPName = std::get<0>(m_fiducials[n]);
					ImVec2 textSize = ImGui::CalcTextSize(GCPName.c_str());
					ImVec2 GCPTextPos(GCP_ScreenSpace.x - 0.5*textSize.x, GCP_ScreenSpace.y + GCPCircleRadiusPixels + 2.0f);
					draw_list->AddText(GCPTextPos, IM_COL32(255, 40, 40, 255), GCPName.c_str());
				}
			}
			if (m_gcpIndexBeingDragged >= 0) {
				//Also draw a circle under the cursor
				draw_list->AddCircleFilled(mousePosScreenSpace, GCPCircleRadiusPixels, IM_COL32(255, 40, 40, 255), 30);
			}
			
			if (ImageHovered) {
				if (m_gcpIndexBeingDragged >= 0) {
					//Dragging GCP
					if (! ImGui::IsMouseDown(0)) {
						//Stop Drag
						std::get<1>(m_fiducials[m_gcpIndexBeingDragged]) = mousePosImageCoords;
						m_gcpIndexBeingDragged = -1;
					}
				}
				else if (ImGui::IsMouseDown(0)) {
					if ((! m_panning) && (hoveredIndex < 0)) {
						m_panning = true;
						m_mousePosOnPanStart_ScreenSpace = mousePosScreenSpace;
						m_IMViewUV0OnPanStart = IMViewUV0;
						m_IMViewUV1OnPanStart = IMViewUV1;
					}
					if (m_panning) {
						float dx_screenSpace = mousePosScreenSpace.x - m_mousePosOnPanStart_ScreenSpace.x;
						float dy_screenSpace = mousePosScreenSpace.y - m_mousePosOnPanStart_ScreenSpace.y;
						float UScale = (IMViewUV1.x - IMViewUV0.x)/(ImageLRCorner_ScreenSpace.x - ImageULCorner_ScreenSpace.x);
						float VScale = (IMViewUV1.y - IMViewUV0.y)/(ImageLRCorner_ScreenSpace.y - ImageULCorner_ScreenSpace.y);
						
						IMViewUV0 = ImVec2(m_IMViewUV0OnPanStart.x - dx_screenSpace*UScale, m_IMViewUV0OnPanStart.y - dy_screenSpace*VScale);
						IMViewUV1 = ImVec2(m_IMViewUV1OnPanStart.x - dx_screenSpace*UScale, m_IMViewUV1OnPanStart.y - dy_screenSpace*VScale);
					}
				}
				else {
					m_panning = false;
					SanitizeUVBounds();
				}
				if (ImGui::IsMouseDoubleClicked(0)) {
					IMViewUV0 = ImVec2(0, 0);
					IMViewUV1 = ImVec2(1, 1);
					m_panning = false;
				}
				
				//Process scroll events
				if ((ImGui::GetIO().MouseWheel != 0.0f) && (m_gcpIndexBeingDragged < 0) && (! ImGui::IsMouseDown(0))) {
					float dx = IMViewUV1.x - IMViewUV0.x;
					float dy = IMViewUV1.y - IMViewUV0.y;
					float diam = std::sqrt(dx*dx + dy*dy);
					float newdiam = std::min(std::max(diam * (1.0f - ImGui::GetIO().MouseWheel*.1f), 0.1f), 1.4142136f);
					float new_dx = dx*newdiam/diam;
					float new_dy = dy*newdiam/diam;
					
					//Get the UV point under the cursor
					float xNorm = (mousePosScreenSpace.x - ImageULCorner_ScreenSpace.x)/(ImageLRCorner_ScreenSpace.x - ImageULCorner_ScreenSpace.x);
					float yNorm = (mousePosScreenSpace.y - ImageULCorner_ScreenSpace.y)/(ImageLRCorner_ScreenSpace.y - ImageULCorner_ScreenSpace.y);
					float U = IMViewUV0.x + xNorm*(IMViewUV1.x - IMViewUV0.x);
					float V = IMViewUV0.y + yNorm*(IMViewUV1.y - IMViewUV0.y);
					
					//Update the UV limits to the new zoom level
					IMViewUV0 = ImVec2(0.5f - new_dx/2.0f, 0.5 - new_dy/2.0f);
					IMViewUV1 = ImVec2(0.5f + new_dx/2.0f, 0.5 + new_dy/2.0f);
					
					//Get the UV offset needed to keep the point under the mouse fixed
					float dU = IMViewUV0.x + xNorm*(IMViewUV1.x - IMViewUV0.x) - U;
					float dV = IMViewUV0.y + yNorm*(IMViewUV1.y - IMViewUV0.y) - V;
					
					//Adjust the UV bounds to keep the point under the mouse fixed
					IMViewUV0 = ImVec2(IMViewUV0.x - dU, IMViewUV0.y - dV);
					IMViewUV1 = ImVec2(IMViewUV1.x - dU, IMViewUV1.y - dV);
					
					//Finally, sanitize the UV bounds
					SanitizeUVBounds();
				}
				
				if ((m_gcpIndexBeingDragged < 0) && (! m_panning)) {
					//Not dragging GCP and not scrolling - see if a drag should start or a GCP should be deleted
					if (hoveredIndex >= 0) {
						if (ImGui::IsMouseClicked(0))
							m_gcpIndexBeingDragged = hoveredIndex;
						else if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Delete)) || ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Backspace))) {
							m_fiducials.erase(m_fiducials.begin() + hoveredIndex); //Delete GCP
							m_fiducialCoordStrings.erase(m_fiducialCoordStrings.begin() + hoveredIndex);
						}
					}
					else {
						//Nothing is hovered
						if (ImGui::IsKeyPressed(78)) {
							//Add a GCP under the cursor
							m_fiducials.push_back(std::make_tuple(GetNewGCPName(), mousePosImageCoords));
							m_fiducialCoordStrings.push_back(std::make_tuple("0.0"s, "0.0"s, "0.0"s));
						}
					}
				}
			}
			else {
				m_gcpIndexBeingDragged = -1; //Cancel a drag if the mouse leaves the image
				m_panning = false; //Cancel pan if the mouse leaves the image
			}
			
			ImGui::NextColumn();
			if (m_firstDrawPass) {
				ImGui::SetColumnWidth(0, 0.75f*ImGui::GetWindowWidth());
				m_firstDrawPass = false;
			}
			
			ImGui::BeginChild("GPS Coords", ImVec2(0, 0), true);
			ImGui::TextUnformatted("GCP GPS Coordinates");
			for (size_t n = 0U; n < m_fiducials.size(); n++) {
				ImGui::Text("GCP %s:", std::get<0>(m_fiducials[n]).c_str());
				float col2start = ImGui::CalcTextSize("Longitude (Deg): ").x;
				
				ImGui::TextUnformatted("Latitude (Deg): ");
				ImGui::SameLine(col2start);
				ImGui::PushItemWidth(std::min(8.0f*ImGui::GetFontSize(), 800.0f));
				std::string inputTextLabelLat = "##InputText-Lat-"s + std::get<0>(m_fiducials[n]);
				ImGui::InputText(inputTextLabelLat.c_str(), &(std::get<0>(m_fiducialCoordStrings[n])));
				ImGui::PopItemWidth();
				
				ImGui::TextUnformatted("Longitude (Deg): ");
				ImGui::SameLine(col2start);
				ImGui::PushItemWidth(std::min(8.0f*ImGui::GetFontSize(), 800.0f));
				std::string inputTextLabelLon = "##InputText-Lon-"s + std::get<0>(m_fiducials[n]);
				ImGui::InputText(inputTextLabelLon.c_str(), &(std::get<1>(m_fiducialCoordStrings[n])));
				ImGui::PopItemWidth();
				
				ImGui::TextUnformatted("Altitude (m): ");
				ImGui::SameLine(col2start);
				ImGui::PushItemWidth(std::min(8.0f*ImGui::GetFontSize(), 800.0f));
				std::string inputTextLabelAlt = "##InputText-Alt-"s + std::get<0>(m_fiducials[n]);
				ImGui::InputText(inputTextLabelAlt.c_str(), &(std::get<2>(m_fiducialCoordStrings[n])));
				ImGui::PopItemWidth();
				
				if (n + 1U < m_fiducials.size()) {
					ImGui::Spacing();
					ImGui::Spacing();
				}
			}
			ImGui::EndChild();
			
			ImGui::NextColumn();
			ImGui::Columns(1);
		}
		
	}
}







