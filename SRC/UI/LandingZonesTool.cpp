//The Landing Zone Paint Tool holds the state of and provides the draw calls for a tool used for
//editing safe landing zones.
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//Project Includes
#include "LandingZonesTool.hpp"
#include "MapWidget.hpp"
#include "../ImVecOps.hpp"
#include "../Maps/MapUtils.hpp"
#include "../Maps/DataTileProvider.hpp"

#define PI 3.14159265358979

LandingZonesTool::LandingZonesTool() :
	m_popupDims(200,200),
	m_popupOpen(false),
	toolActive(false),
	m_shape(0),
	m_radius(1.0f),
	m_lengthX(1.0f),
	m_lengthY(1.0f),
	m_angleDeg(0.0f),
	m_EditMode(0)
{ }

bool LandingZonesTool::Draw_Button(void) {
	bool wasToolActive = toolActive;
	ImDrawList * draw_list = ImGui::GetWindowDrawList();
	draw_list->ChannelsSplit(2);

	auto style = ImGui::GetStyle();
	ImGui::PushID("Landing Zones Tool");

	Math::Vector2 RectULCorner = ImGui::GetCursorScreenPos();
	RectULCorner.x -= style.ItemInnerSpacing.x;
	ImGui::SetCursorScreenPos(RectULCorner + style.ItemInnerSpacing);

	//Display foreground first
	draw_list->ChannelsSetCurrent(1);
						
	ImGui::Text(u8" \uf5af  Landing Zones ");
	
	//Compute sizes and bounds of button regions
	Math::Vector2 RectLRCorner = RectULCorner + ImGui::GetItemRectSize() + style.ItemInnerSpacing * Math::Vector2(2.0f, 2.0f);
	Math::Vector2 ArrowULCorner = Math::Vector2(RectLRCorner.x, RectULCorner.y);
	Math::Vector2 ArrowLRCorner = Math::Vector2(RectLRCorner.x + ImGui::GetFontSize(), RectLRCorner.y);
	
	DrawTriangle(ArrowULCorner, 1.0f, draw_list);
	
	//Display background for toggle button
	ImGui::SetCursorScreenPos(RectULCorner);
	draw_list->ChannelsSetCurrent(0);
	if (ImGui::InvisibleButton("LandingZonesButton", RectLRCorner - RectULCorner))
		toolActive = !toolActive;
	
	ImU32 bg_color;
	if (toolActive)
		bg_color = ImGui::IsItemHovered() ? ImColor(style.Colors[ImGuiCol_ButtonHovered]) : ImColor(style.Colors[ImGuiCol_Button]);
	else
		bg_color = ImGui::IsItemHovered() ? ImColor(style.Colors[ImGuiCol_TabHovered]) : ImColor(style.Colors[ImGuiCol_Tab]);
	draw_list->AddRectFilled(RectULCorner, ArrowLRCorner, bg_color, 0.0f);
	
	//Draw shadow
	ImU32 col_upr_left  = (ImU32)ImColor(255, 255, 255,  80);
	ImU32 col_upr_right = (ImU32)ImColor(255, 255, 255,  80);
	ImU32 col_bot_right = (ImU32)ImColor(  0,   0,   0,  60);
	ImU32 col_bot_left  = (ImU32)ImColor(  0,   0,   0,  20);
	draw_list->AddRectFilledMultiColor(RectULCorner, ArrowLRCorner, col_upr_left, col_upr_right, col_bot_right, col_bot_left);
	
	//Display background for arrow drop-down button
	ImGui::SetCursorScreenPos(ArrowULCorner);
	if (ImGui::InvisibleButton("LandingZonesArrow", ArrowLRCorner - ArrowULCorner) && (!ImGui::IsPopupOpen("Popup")))
		ImGui::OpenPopup("Landing Zones DropDown");
	bg_color = ImGui::IsItemHovered() ? ImColor(style.Colors[ImGuiCol_ButtonHovered]) : ImColor(style.Colors[ImGuiCol_Button]);
	draw_list->AddRectFilled(ArrowULCorner, ArrowLRCorner, bg_color, 0.0f);
	draw_list->AddRectFilledMultiColor(ArrowULCorner, ArrowLRCorner, col_upr_left, col_upr_right, col_bot_right, col_bot_right);
	
	//Merge draw list channels
	draw_list->ChannelsMerge();
	
	Draw_DropDown(Math::Vector2(RectULCorner.x, RectLRCorner.y), ArrowLRCorner.x - RectULCorner.x);
	
	ImGui::PopID();
	return ((! wasToolActive) && toolActive);
}

void LandingZonesTool::Draw_DropDown(Math::Vector2 PopupULCorner, float PopupWidth) {
	auto style = ImGui::GetStyle();
	
	ImRect popup_rect(PopupULCorner, PopupULCorner + m_popupDims);
	ImGui::SetNextWindowPos(popup_rect.Min);
	ImGui::SetNextWindowSize(popup_rect.GetSize());
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, style.FramePadding);
	
	if (ImGui::BeginPopup("Landing Zones DropDown")) {
		m_popupOpen = true;
		
		Math::Vector2 WindowStartCursorPos = ImGui::GetCursorPos();
		
		ImGui::RadioButton("Circle", &m_shape, 0);
		ImGui::RadioButton("Rectangle", &m_shape, 1);
		
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		
		if (m_shape == 0) {
			ImGui::Text("Radius (m):");
			ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
			ImGui::InputFloat("##RadiusInputFloat", &m_radius, 0.1f, 0.5f, "%.1f");
			m_radius = std::max(std::min(m_radius, 30.0f), 0.1f);
			ImGui::PopItemWidth();
		}
		else if (m_shape == 1) {
			ImGui::Text("Width (m):");
			ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
			ImGui::InputFloat("##LengthXInputFloat", &m_lengthX, 0.1f, 0.5f, "%.1f");
			m_lengthX = std::max(std::min(m_lengthX, 60.0f), 0.1f);
			ImGui::PopItemWidth();
			ImGui::Text("Length (m):");
			ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
			ImGui::InputFloat("##LengthYInputFloat", &m_lengthY, 0.1f, 0.5f, "%.1f");
			m_lengthY = std::max(std::min(m_lengthY, 60.0f), 0.1f);
			ImGui::PopItemWidth();
			ImGui::Text("Angle (degrees):");
			ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
			ImGui::InputFloat("##AngleDegInputFloat", &m_angleDeg, 0.25f, 4.0f, "%.1f");
			ImGui::PopItemWidth();
		}
		
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		
		ImGui::Text("Edit Mode:");
		ImGui::RadioButton("Draw", &m_EditMode, 0);
		ImGui::RadioButton("Erase", &m_EditMode, 1);
		
		float popupWidth  = std::max(100.0f, ImGui::CalcTextSize("Angle (degrees): 00").x + 2.0f*style.ItemSpacing.x);
		float popupHeight = ImGui::GetCursorPosY() - WindowStartCursorPos.y + 2.0f*style.FramePadding.y;
		m_popupDims       = Math::Vector2(popupWidth, popupHeight);
	
		ImGui::EndPopup();
	}
	else
		m_popupOpen = false;
	
	ImGui::PopStyleVar();
}

void LandingZonesTool::DrawTriangle(Math::Vector2 p_min, float scale, ImDrawList * DrawList) {
	const float h = ImGui::GetFontSize() * 1.00f;
	const float r = h * 0.40f * scale;
	Math::Vector2 center = p_min + Math::Vector2(h*0.50f, h*0.50f*scale);

	Math::Vector2 a, b, c;
	center.y += r*0.25f;
	a = center + Math::Vector2(0,1)*r;
	b = center + Math::Vector2(-0.866f,-0.5f)*r;
	c = center + Math::Vector2(0.866f,-0.5f)*r;

	DrawList->AddTriangleFilled(a, b, c, ImGui::GetColorU32(ImGuiCol_Text));
}

void LandingZonesTool::Draw_Overlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
	//If the parameter popup is open, draw a toy sample region in the middle of the map window and return
	if (m_popupOpen) {
		//Draw sample shape in middle of the screen
		Eigen::Vector2d MapCenter_ScreenSpace = MapWidget::Instance().MapWidgetULCorner_ScreenSpace + 0.5*MapWidget::Instance().MapWidgetDims;
		ImVec2 center_ScreenSpace(float(MapCenter_ScreenSpace(0)), float(MapCenter_ScreenSpace(1)));
		Eigen::Vector2d center_NM = MapWidget::Instance().ScreenCoordsToNormalizedMercator(center_ScreenSpace);
		DrawTool(center_NM, DrawList);
		return;
	}
	else if (toolActive && CursorInBounds) {
		DrawTool(CursorPos_NM, DrawList);
		if (ImGui::IsMouseDown(0))
			ExecuteEdit(CursorPos_NM);
	}
}

void LandingZonesTool::DrawTool(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList) {
	if (m_shape == 0) { //Circle
		double radiusInPixels = MetersToPixels(m_radius, CursorPos_NM(1), MapWidget::Instance().zoom);
		ImVec2 samplePos_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(CursorPos_NM);
		
		DrawList->AddCircle(samplePos_ScreenSpace, float(radiusInPixels)+1.0f, IM_COL32(255, 255, 255, 255), 30, 2.0f);
		DrawList->AddCircle(samplePos_ScreenSpace, float(radiusInPixels)-1.0f, IM_COL32(0, 0, 0, 255), 30, 2.0f);
	}
	else { //Rectangle
		double lengthX_NM = MetersToNMUnits(m_lengthX, CursorPos_NM(1));
		double lengthY_NM = MetersToNMUnits(m_lengthY, CursorPos_NM(1));
		
		Eigen::Vector2d P1_NM = CursorPos_NM + Eigen::Vector2d(-0.5*lengthX_NM, -0.5*lengthY_NM);
		Eigen::Vector2d P2_NM = CursorPos_NM + Eigen::Vector2d( 0.5*lengthX_NM, -0.5*lengthY_NM);
		Eigen::Vector2d P3_NM = CursorPos_NM + Eigen::Vector2d( 0.5*lengthX_NM,  0.5*lengthY_NM);
		Eigen::Vector2d P4_NM = CursorPos_NM + Eigen::Vector2d(-0.5*lengthX_NM,  0.5*lengthY_NM);
		
		//Rotate all vertices clockwise about CursorPos_NM
		double theta = m_angleDeg*PI/180.0;
		Eigen::Matrix2d R_CW;
		R_CW << cos(-theta), -sin(-theta),
			   sin(-theta),  cos(-theta);
		P1_NM = R_CW * (P1_NM - CursorPos_NM) + CursorPos_NM;
		P2_NM = R_CW * (P2_NM - CursorPos_NM) + CursorPos_NM;
		P3_NM = R_CW * (P3_NM - CursorPos_NM) + CursorPos_NM;
		P4_NM = R_CW * (P4_NM - CursorPos_NM) + CursorPos_NM;
		
		Math::Vector2 P1_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(P1_NM);
		Math::Vector2 P2_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(P2_NM);
		Math::Vector2 P3_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(P3_NM);
		Math::Vector2 P4_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(P4_NM);
		
		//Eigen::Matrix2d R = smp->GetR().transpose();
		Eigen::Vector2d V1_E = R_CW * Eigen::Vector2d( 0.707106781186547, 0.707106781186547);
		Eigen::Vector2d V2_E = R_CW * Eigen::Vector2d(-0.707106781186547, 0.707106781186547);
		Math::Vector2 V1 = EigenToImVec(V1_E) * Math::Vector2(1, -1);
		Math::Vector2 V2 = EigenToImVec(V2_E) * Math::Vector2(1, -1);
		Math::Vector2 P1P_ScreenSpace = P1_ScreenSpace + 1.414214f*2.0f*V1;
		Math::Vector2 P2P_ScreenSpace = P2_ScreenSpace + 1.414214f*2.0f*V2;
		Math::Vector2 P3P_ScreenSpace = P3_ScreenSpace - 1.414214f*2.0f*V1;
		Math::Vector2 P4P_ScreenSpace = P4_ScreenSpace - 1.414214f*2.0f*V2;
		
		DrawList->AddQuad(P1_ScreenSpace, P2_ScreenSpace, P3_ScreenSpace, P4_ScreenSpace, IM_COL32(255, 255, 255, 255), 2.0f);
		DrawList->AddQuad(P1P_ScreenSpace, P2P_ScreenSpace, P3P_ScreenSpace, P4P_ScreenSpace, IM_COL32(0, 0, 0, 255), 2.0f);
	}	
}

void LandingZonesTool::ExecuteEdit(Eigen::Vector2d const & CursorPos_NM) {
	if ((m_shape == 0) && (m_EditMode == 0))
		Maps::DataTileProvider::Instance()->Paint_Circle(CursorPos_NM, double(m_radius), Maps::DataLayer::SafeLandingZones, 1.0);
	else if ((m_shape == 0) && (m_EditMode == 1))
		Maps::DataTileProvider::Instance()->Erase_Circle(CursorPos_NM, double(m_radius), Maps::DataLayer::SafeLandingZones);
	else if ((m_shape == 1) && (m_EditMode == 0))
		Maps::DataTileProvider::Instance()->Paint_Rect(CursorPos_NM, m_lengthX, m_lengthY, m_angleDeg, Maps::DataLayer::SafeLandingZones, 1.0);
	else if ((m_shape == 1) && (m_EditMode == 1))
		Maps::DataTileProvider::Instance()->Erase_Rect(CursorPos_NM, m_lengthX, m_lengthY, m_angleDeg, Maps::DataLayer::SafeLandingZones);
}





