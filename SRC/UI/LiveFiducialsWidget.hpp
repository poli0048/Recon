//This widget lets you select a reference frame from live imagery and mark fiducials to be used for registration
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.
#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>
#include <chrono>

//External Includes
#include "../HandyImGuiInclude.hpp"
#include <opencv2/opencv.hpp>

//Project Includes
#include "../Utilities.hpp"
#include "../Modules/GNSS-Receiver/GNSSReceiver.hpp"

class LiveFiducialsWidget {
	private:
		int m_state = -1; //State of the widget: -1: Inactive, 0: Select Ref frame, 1: Mark Fiducials
		DroneInterface::Drone * m_drone = nullptr;
		int m_ImageryCallbackHandle = -1;
		
		//Texture and Mat for live video from drone (Most recent frame). Also holds the ref frame once selected.
		std::mutex                       m_Image_mutex;
		cv::Mat                          m_Image;
		DroneInterface::Drone::TimePoint m_Image_Timestamp;
		bool                             m_Image_TexValid = false;
		ImTextureID                      m_Image_Tex;
		
		//Controls in state 0 (select ref frame)
		float  m_FeedZoom = 1.0f; //1 = No crop, 2 = Crop to centeral 50% width and height
		
		//Fields for state 1 (mark fiducials)
		ImVec2 IMViewUV0; //The UV Coords of the upper-left corner of the view of the ref frame
		ImVec2 IMViewUV1; //The UV Coords of the lower-right corner of the view of the ref frame
		std::Evector<std::tuple<std::string, Eigen::Vector2d>> m_fiducials; //<Name, PixCoords>, where PixCoords = (col, row)
		std::vector<std::tuple<std::string, std::string, std::string>> m_fiducialCoordStrings; //Lat (deg), Lon (deg), Alt (m)
		
		int m_gcpIndexBeingDragged = -1;
		bool m_panning = false;
		ImVec2 m_mousePosOnPanStart_ScreenSpace;
		ImVec2 m_IMViewUV0OnPanStart;
		ImVec2 m_IMViewUV1OnPanStart;
		
		bool m_firstDrawPass = true; //Used to set column widths in first draw pass
		
		inline Eigen::Vector2d ScreenSpaceToImageCoords(ImVec2 Pos_ScreenSpace, ImVec2 ULCorner_ScreenSpace, ImVec2 LRCorner_ScreenSpace);
		inline ImVec2 ImageCoordsToScreenSpace(Eigen::Vector2d const & Pos_ImageCoords, ImVec2 ULCorner_ScreenSpace, ImVec2 LRCorner_ScreenSpace);
		
		inline std::string GetNewGCPName(void);
		inline void SanitizeUVBounds(void);
		
	public:
		static LiveFiducialsWidget & Instance() { static LiveFiducialsWidget widget; return widget; }
		
		LiveFiducialsWidget() = default;
		~LiveFiducialsWidget() = default;
		
		inline void Draw();
		inline void Show(std::string const & DroneSerial);
		inline void Hide();
};

inline void LiveFiducialsWidget::Show(std::string const & DroneSerial) {
	//If hide was never called and we were previously used - call Hide now (necessary to unregister video callback)
	if (m_drone != nullptr) {
		std::cerr << "Internal Error: Calling Show without subsequent call to Hide leaves stuff running in background. Fix this.\r\n";
		Hide();
	}
	
	//Get pointer to drone object
	m_drone = DroneInterface::DroneManager::Instance().GetDrone(DroneSerial);
	if (m_drone == nullptr) {
		std::cerr << "Error: Can't grab ptr to drone with serial: " << DroneSerial << "\r\n";
		return;
	}
	
	//Clear old Image data (if there was any)
	m_Image_mutex.lock();
	m_Image = cv::Mat();
	m_Image_Timestamp = std::chrono::steady_clock::now();
	if (m_Image_TexValid) {
		ImGuiApp::Instance().DeleteImageAsyncWithDelay(m_Image_Tex, 2.0);
		m_Image_TexValid = false;
	}
	m_Image_mutex.unlock();
	
	//Register callback to receive imagery from target drone
	m_ImageryCallbackHandle = m_drone->RegisterCallback([this](cv::Mat const & Frame, DroneInterface::Drone::TimePoint const & Timestamp) {
		//Delayed destruction of old texture (if there is one) and clear the last frame
		m_Image_mutex.lock();
		m_Image = cv::Mat();
		if (m_Image_TexValid) {
			ImGuiApp::Instance().DeleteImageAsyncWithDelay(m_Image_Tex, 2.0);
			m_Image_TexValid = false;
		}
		m_Image_mutex.unlock();
		
		if (! Frame.isContinuous()) {
			std::cerr << "Warning: Can't display non-continuus image matrix.\r\n";
			return;
		}
		if (Frame.type() == CV_8UC4) {
			cv::Mat Frame_RGBA(Frame.size(), CV_8UC4);
			cv::cvtColor(Frame, Frame_RGBA, cv::COLOR_BGRA2RGBA, 4);
			ImTextureID Tex = ImGuiApp::Instance().CreateImageRGBA8888(Frame_RGBA.ptr(), Frame_RGBA.cols, Frame_RGBA.rows);
			std::scoped_lock lock(m_Image_mutex);
			cv::cvtColor(Frame, m_Image, cv::COLOR_BGRA2BGR, 3);
			m_Image_Timestamp = Timestamp;
			m_Image_TexValid = true;
			m_Image_Tex = Tex;
		}
		else if (Frame.type() == CV_8UC3) {
			cv::Mat Frame_RGBA(Frame.size(), CV_8UC4);
			cv::cvtColor(Frame, Frame_RGBA, cv::COLOR_BGR2RGBA, 4);
			ImTextureID Tex = ImGuiApp::Instance().CreateImageRGBA8888(Frame_RGBA.ptr(), Frame_RGBA.cols, Frame_RGBA.rows);
			std::scoped_lock lock(m_Image_mutex);
			Frame.copyTo(m_Image); //Deep copy to avoid modifications from impacting other callback handlers
			m_Image_Timestamp = Timestamp;
			m_Image_TexValid = true;
			m_Image_Tex = Tex;
		}
		else {
			std::cerr << "Warning: Can't process image... unsupported type.\r\n";
			return;
		}
	});
	
	m_FeedZoom = 1.0f;
	
	IMViewUV0 = ImVec2(0, 0);
	IMViewUV1 = ImVec2(1, 1);
	m_fiducials.clear();
	m_fiducialCoordStrings.clear();
	
	m_gcpIndexBeingDragged = -1;
	m_panning = false;
	m_firstDrawPass = true;
	
	//Set tool state to select reference frame
	m_state = 0;
}

inline void LiveFiducialsWidget::Hide() {
	if (m_drone == nullptr) {
		std::cerr << "Warning from LiveFiducialsWidget: Ignoring unnecessary call to Hide()\r\n";
		return;
	}
	
	//Unregister callback for receiving imagery
	if (m_ImageryCallbackHandle >= 0) {
		m_drone->UnRegisterCallback(m_ImageryCallbackHandle);
		m_ImageryCallbackHandle = -1;
	}
	
	m_drone = nullptr;
	m_state = -1;
}

//Returns (col, row) in image corresponding to a point in screen space (in state 1)
inline Eigen::Vector2d LiveFiducialsWidget::ScreenSpaceToImageCoords(ImVec2 Pos_ScreenSpace, ImVec2 ULCorner_ScreenSpace, ImVec2 LRCorner_ScreenSpace) {
	float xNorm = (Pos_ScreenSpace.x - ULCorner_ScreenSpace.x)/(LRCorner_ScreenSpace.x - ULCorner_ScreenSpace.x);
	float yNorm = (Pos_ScreenSpace.y - ULCorner_ScreenSpace.y)/(LRCorner_ScreenSpace.y - ULCorner_ScreenSpace.y);
	
	float U = IMViewUV0.x + xNorm*(IMViewUV1.x - IMViewUV0.x);
	float V = IMViewUV0.y + yNorm*(IMViewUV1.y - IMViewUV0.y);
	return Eigen::Vector2d(U*m_Image.cols - 0.5, V*m_Image.rows - 0.5);
}

//Returns the screen space coordinates for the given image coordinates (col, row) in state 1
inline ImVec2 LiveFiducialsWidget::ImageCoordsToScreenSpace(Eigen::Vector2d const & Pos_ImageCoords, ImVec2 ULCorner_ScreenSpace, ImVec2 LRCorner_ScreenSpace) {
	float U = (float(Pos_ImageCoords(0)) + 0.5f)/m_Image.cols;
	float V = (float(Pos_ImageCoords(1)) + 0.5f)/m_Image.rows;
	
	float xNorm = (U - IMViewUV0.x)/(IMViewUV1.x - IMViewUV0.x);
	float yNorm = (V - IMViewUV0.y)/(IMViewUV1.y - IMViewUV0.y);
	
	float pos_ScreenSpace_X = xNorm*(LRCorner_ScreenSpace.x - ULCorner_ScreenSpace.x) + ULCorner_ScreenSpace.x;
	float pos_ScreenSpace_Y = yNorm*(LRCorner_ScreenSpace.y - ULCorner_ScreenSpace.y) + ULCorner_ScreenSpace.y;
	return ImVec2(pos_ScreenSpace_X, pos_ScreenSpace_Y);
}

inline void LiveFiducialsWidget::SanitizeUVBounds(void) {
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

inline std::string LiveFiducialsWidget::GetNewGCPName(void) {
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

inline void LiveFiducialsWidget::Draw() {
	if (m_state < 0)
		return;
	
	ImExt::Window::Options WinOpts;
	WinOpts.Flags = WindowFlags::NoTitleBar | WindowFlags::NoResize | WindowFlags::NoMove | WindowFlags::NoCollapse | WindowFlags::NoSavedSettings |
	                WindowFlags::NoDocking;
	bool visible = true;
	WinOpts.POpen = &visible;
	WinOpts.Size(ImVec2(1400.0f, 900.0f), Condition::Once);
	if (ImExt::Window window("Live Fiducial Marking Tool"s, WinOpts); window.ShouldDrawContents()) {
		if (m_state == 0) {
			//Show live video and allow the user to select a reference frame or cancel
			m_Image_mutex.lock();
			bool texValid = m_Image_TexValid;
			m_Image_mutex.unlock();
			if (texValid) {
				if (ImGui::Button("Select Reference Frame")) {
					//Unregister callback for receiving imagery - this stops image updates
					if (m_ImageryCallbackHandle >= 0) {
						m_drone->UnRegisterCallback(m_ImageryCallbackHandle);
						m_ImageryCallbackHandle = -1;
					}
					
					//Advance tool state so we can start marking fiducials
					m_state = 1;
				}
				ImGui::SameLine();
			}
			if (ImGui::Button("Cancel & Close"))
				Hide();
			std::scoped_lock lock(m_Image_mutex);
			if (m_Image_TexValid) {
				float sliderWidth = ImGui::GetFontSize();
				ImVec2 UV0(0.5f - 0.5f/m_FeedZoom, 0.5f - 0.5f/m_FeedZoom);
				ImVec2 UV1(0.5f + 0.5f/m_FeedZoom, 0.5f + 0.5f/m_FeedZoom);
				
				ImVec2 regionAvail = ImGui::GetContentRegionAvail();
				float scale1 = (regionAvail.x - sliderWidth - 1.5f*ImGui::GetStyle().ItemSpacing.x) / float(m_Image.cols);
				float scale2 = regionAvail.y / float(m_Image.rows);
				float scale = std::min(scale1, scale2);
				ImVec2 TexSize(scale*float(m_Image.cols), scale*float(m_Image.rows));
				ImGui::Image(m_Image_Tex, TexSize, UV0, UV1, ImVec4(1,1,1,1), ImVec4(0,0,0,0));
				
				ImGui::SameLine();
				ImGui::VSliderFloat("##ZoomSlider", ImVec2(sliderWidth, TexSize.y), &(m_FeedZoom), 1.0f, 2.0f, "", ImGuiSliderFlags_NoRoundToFormat);
			}
		}
		else if (m_state == 1) {
			//Mark fiducials
			if (ImGui::Button("Cancel & Close"))
				Hide();
			if (m_fiducials.size() >= 3U) {
				ImGui::SameLine();
				if (ImGui::Button("Start Shadow Detection")) {
					std::cerr << "Starting shadow detection module.";
					
					//Set the reference frame
					m_Image_mutex.lock();
					ShadowDetection::ShadowDetectionEngine::Instance().SetReferenceFrame(m_Image);
					m_Image_mutex.unlock();
					
					//Set fiducials
					std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> GCPs;
					//GCPs are passed in in the form <PixCoords, LLA>
					//PixCoords are the coordinates (in pixels) in the reference image of a fiducial: (col, row) with upper-left corner origin
					//LLA are the WGS84 latitude (radians), longitude (radians), and altitude (m) of the fiducial, in that order.
					double PI = 3.14159265358979;
					for (size_t n = 0; n < m_fiducials.size(); n++) {
						Eigen::Vector2d PixCoords = std::get<1>(m_fiducials[n]);
						double lat, lon, alt;
						if (str2double(std::get<0>(m_fiducialCoordStrings[n]), lat))
							lat = PI/180.0*lat;
						else {
							std::cerr << "Dropping fiducial " << std::get<1>(m_fiducials[n]) << " because of invalid latitude.\r\n";
							continue;
						}
						if (str2double(std::get<1>(m_fiducialCoordStrings[n]), lon))
							lon = PI/180.0*lon;
						else {
							std::cerr << "Dropping fiducial " << std::get<1>(m_fiducials[n]) << " because of invalid longitude.\r\n";
							continue;
						}
						if (! str2double(std::get<2>(m_fiducialCoordStrings[n]), alt)) {
							std::cerr << "Dropping fiducial " << std::get<1>(m_fiducials[n]) << " because of invalid altitude.\r\n";
							continue;
						}
						
						Eigen::Vector3d LLA(lat, lon, alt);
						GCPs.push_back(std::make_tuple(PixCoords, LLA));
					}
					ShadowDetection::ShadowDetectionEngine::Instance().SetFiducials(GCPs);
					
					//Start the shadow detection engine
					ShadowDetection::ShadowDetectionEngine::Instance().Start(m_drone->GetDroneSerial());
					
					//Close this tool
					Hide();
				}
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
			
			std::scoped_lock lock(m_Image_mutex);
			if (! m_Image_TexValid) {
				std::cerr << "Internal Error in LiveFiducialsWidget::Draw(): Shouldn't have been able to get to tool state 1 without valid image tex.\r\n";
				return;
			}
			
			ImGui::Columns(2, "Live Fiducial Marking Win H Columns");
			ImVec2 regionAvail = ImGui::GetContentRegionAvail();
			float scale1 = (regionAvail.x - 1.5f*ImGui::GetStyle().ItemSpacing.x) / float(m_Image.cols);
			float scale2 = regionAvail.y / float(m_Image.rows);
			float scale = std::min(scale1, scale2);
			ImVec2 TexSize(scale*float(m_Image.cols), scale*float(m_Image.rows));
			ImGui::Image(m_Image_Tex, TexSize, IMViewUV0, IMViewUV1, ImVec4(1,1,1,1), ImVec4(0,0,0,0));
			//ImGui::Image(m_Image_Tex, TexSize, ImVec2(0,0), ImVec2(1,1), ImVec4(1,1,1,1), ImVec4(0,0,0,0));
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
							double GCP_GroundAlt = 0.0; //Default alt of 0 if no GNSS receiver is connected.
							GNSSReceiver::GNSSManager::Instance().GetGroundAlt(GCP_GroundAlt); //Use GCS alt if connected for initialization
							
							std::ostringstream outSS;
							outSS << std::fixed << std::setprecision(1) << GCP_GroundAlt;
							std::string AltStr = outSS.str();
							
							m_fiducials.push_back(std::make_tuple(GetNewGCPName(), mousePosImageCoords));
							m_fiducialCoordStrings.push_back(std::make_tuple("0.0"s, "0.0"s, AltStr));
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
				float col2start = ImGui::CalcTextSize("Longitude (Deg):   ").x;
				ImGui::Text("GCP %s:", std::get<0>(m_fiducials[n]).c_str());
				ImGui::SameLine(col2start);
				std::string FromClipboardLabel = "From Clipboard##"s + std::get<0>(m_fiducials[n]);
				if (ImGui::Button(FromClipboardLabel.c_str())) {
					std::string clipboardText(ImGui::GetClipboardText());
					std::vector<std::string> components = StringSplit(clipboardText, ", \t"s);
					if (components.size() > 0)
						std::get<0>(m_fiducialCoordStrings[n]) = components[0];
					if (components.size() > 1)
						std::get<1>(m_fiducialCoordStrings[n]) = components[1];
					if (components.size() > 2)
						std::get<2>(m_fiducialCoordStrings[n]) = components[2];
				}
				
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
		else {
			std::cerr << "Warning in LiveFiducialsWidget::Draw(): Invalid tool state. Disabling tool.\r\n";
			m_state = -1;
		}
	}
	
	//If the user tried to close the window from the window titlebar, treat it as a cancelation
	if (! visible)
		Hide();
}







