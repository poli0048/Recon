//The Landing Zone Paint Tool holds the state of and provides the draw calls for a tool used for
//editing safe landing zones.
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//Project Includes
#include "LandingZonesTool.hpp"
#include "MapWidget.hpp"
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

	Eigen::Vector2d RectULCorner = ImGui::GetCursorScreenPos();
	RectULCorner(0) -= style.ItemInnerSpacing.x;
	Eigen::Vector2d cursorScreenPos = RectULCorner + (Eigen::Vector2d) style.ItemInnerSpacing;
	ImGui::SetCursorScreenPos(cursorScreenPos);

	//Display foreground first
	draw_list->ChannelsSetCurrent(1);
						
	ImGui::Text(u8" \uf5af  Landing Zones ");
	
	//Compute sizes and bounds of button regions
	Eigen::Vector2d RectLRCorner = RectULCorner + (Eigen::Vector2d) ImGui::GetItemRectSize() + 2.0 * (Eigen::Vector2d) style.ItemInnerSpacing;
	Eigen::Vector2d ArrowULCorner(RectLRCorner(0), RectULCorner(1));
	Eigen::Vector2d ArrowLRCorner(RectLRCorner(0) + ImGui::GetFontSize(), RectLRCorner(1));
	
	DrawTriangle(ArrowULCorner, 1.0f, draw_list);
	
	//Display background for toggle button
	ImGui::SetCursorScreenPos(RectULCorner);
	draw_list->ChannelsSetCurrent(0);
	if (ImGui::InvisibleButton("LandingZonesButton", (Eigen::Vector2d) (RectLRCorner - RectULCorner)))
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
	if (ImGui::InvisibleButton("LandingZonesArrow", (Eigen::Vector2d) (ArrowLRCorner - ArrowULCorner)) && (!ImGui::IsPopupOpen("Popup")))
		ImGui::OpenPopup("Landing Zones DropDown");
	bg_color = ImGui::IsItemHovered() ? ImColor(style.Colors[ImGuiCol_ButtonHovered]) : ImColor(style.Colors[ImGuiCol_Button]);
	draw_list->AddRectFilled(ArrowULCorner, ArrowLRCorner, bg_color, 0.0f);
	draw_list->AddRectFilledMultiColor(ArrowULCorner, ArrowLRCorner, col_upr_left, col_upr_right, col_bot_right, col_bot_right);
	
	//Merge draw list channels
	draw_list->ChannelsMerge();
	
	Eigen::Vector2d PopupULCorner(RectULCorner(0), RectLRCorner(1));
	Draw_DropDown(PopupULCorner, ArrowLRCorner(0) - RectULCorner(0));
	
	ImGui::PopID();
	return ((! wasToolActive) && toolActive);
}

void LandingZonesTool::Draw_DropDown(Eigen::Vector2d const & PopupULCorner, float PopupWidth) {
	auto style = ImGui::GetStyle();
	
	ImRect popup_rect(PopupULCorner, (Eigen::Vector2d) (PopupULCorner + m_popupDims));
	ImGui::SetNextWindowPos(popup_rect.Min);
	ImGui::SetNextWindowSize(popup_rect.GetSize());
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, style.FramePadding);
	
	if (ImGui::BeginPopup("Landing Zones DropDown")) {
		m_popupOpen = true;
		
		Eigen::Vector2d WindowStartCursorPos = ImGui::GetCursorPos();
		
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
		float popupHeight = ImGui::GetCursorPosY() - WindowStartCursorPos(1) + 2.0f*style.FramePadding.y;
		m_popupDims       << popupWidth, popupHeight;
	
		ImGui::EndPopup();
	}
	else
		m_popupOpen = false;
	
	ImGui::PopStyleVar();
}

void LandingZonesTool::DrawTriangle(Eigen::Vector2d const & p_min, float scale, ImDrawList * DrawList) {
	const float h = ImGui::GetFontSize() * 1.00f;
	const float r = h * 0.40f * scale;
	Eigen::Vector2d center = p_min + Eigen::Vector2d(h*0.50f, h*0.50f*scale);

	center(1) += r*0.25f;
	Eigen::Vector2d a = center + Eigen::Vector2d(   0.0,  1.0)*r;
	Eigen::Vector2d b = center + Eigen::Vector2d(-0.866, -0.5)*r;
	Eigen::Vector2d c = center + Eigen::Vector2d( 0.866, -0.5)*r;

	DrawList->AddTriangleFilled(a, b, c, ImGui::GetColorU32(ImGuiCol_Text));
}

void LandingZonesTool::Draw_Overlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
	//If the parameter popup is open, draw a toy sample region in the middle of the map window and return
	if (m_popupOpen) {
		//Draw sample shape in middle of the screen
		Eigen::Vector2d MapCenter_ScreenSpace = MapWidget::Instance().MapWidgetULCorner_ScreenSpace + 0.5*MapWidget::Instance().MapWidgetDims;
		Eigen::Vector2d center_NM = MapWidget::Instance().ScreenCoordsToNormalizedMercator(MapCenter_ScreenSpace);
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
		Eigen::Vector2d samplePos_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(CursorPos_NM);
		
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
		
		Eigen::Vector2d P1_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(P1_NM);
		Eigen::Vector2d P2_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(P2_NM);
		Eigen::Vector2d P3_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(P3_NM);
		Eigen::Vector2d P4_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(P4_NM);
		
		//Eigen::Matrix2d R = smp->GetR().transpose();
		Eigen::Vector2d V1_E = R_CW * Eigen::Vector2d( 0.707106781186547, 0.707106781186547);
		Eigen::Vector2d V2_E = R_CW * Eigen::Vector2d(-0.707106781186547, 0.707106781186547);
		Eigen::Vector2d V1(V1_E(0), -1.0 * V1_E(1));
		Eigen::Vector2d V2(V2_E(0), -1.0 * V2_E(1));
		Eigen::Vector2d P1P_ScreenSpace = P1_ScreenSpace + 1.414214f*2.0f*V1;
		Eigen::Vector2d P2P_ScreenSpace = P2_ScreenSpace + 1.414214f*2.0f*V2;
		Eigen::Vector2d P3P_ScreenSpace = P3_ScreenSpace - 1.414214f*2.0f*V1;
		Eigen::Vector2d P4P_ScreenSpace = P4_ScreenSpace - 1.414214f*2.0f*V2;
		
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





