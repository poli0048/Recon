//The Survey Region Tool supports drawing and editing vector survey regions. A survey region
//is defined as a union of polygons with optional hole polygons (arbitrarily many in each polygon)
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//Project Includes
#include "SurveyRegionTool.hpp"
#include "MapWidget.hpp"
#include "../ImVecOps.hpp"
#include "../Maps/MapUtils.hpp"
#include "ModalDialogs.hpp"
#include "MyGui.hpp"
#include "VisWidget.hpp"

#define PI 3.14159265358979

static ImVec2 RoundToNearestPixel(ImVec2 P) {
	P.x = std::round(P.x);
	P.y = std::round(P.y);
	return P;
}

SurveyRegionsTool::SurveyRegionsTool() :
	m_popupDims(200,200),
	toolActive(false),
	m_toolState(0)
{ }

bool SurveyRegionsTool::Draw_Button(void) {
	bool wasToolActive = toolActive;
	ImDrawList * draw_list = ImGui::GetWindowDrawList();
	draw_list->ChannelsSplit(2);

	auto style = ImGui::GetStyle();
	ImGui::PushID("Survey Regions Tool");

	Math::Vector2 RectULCorner = ImGui::GetCursorScreenPos();
	RectULCorner.x -= style.ItemInnerSpacing.x;
	ImGui::SetCursorScreenPos(RectULCorner + style.ItemInnerSpacing);

	//Display foreground first
	draw_list->ChannelsSetCurrent(1);
						
	ImGui::Text(u8" \uf5ee  Survey Region ");
	
	//Compute sizes and bounds of button regions
	Math::Vector2 RectLRCorner = RectULCorner + ImGui::GetItemRectSize() + style.ItemInnerSpacing * Math::Vector2(2.0f, 2.0f);
	Math::Vector2 ArrowULCorner = Math::Vector2(RectLRCorner.x, RectULCorner.y);
	Math::Vector2 ArrowLRCorner = Math::Vector2(RectLRCorner.x + ImGui::GetFontSize(), RectLRCorner.y);
	
	DrawTriangle(ArrowULCorner, 1.0f, draw_list);
	
	//Display background for toggle button
	ImGui::SetCursorScreenPos(RectULCorner);
	draw_list->ChannelsSetCurrent(0);
	if (ImGui::InvisibleButton("SurveyRegionButton", RectLRCorner - RectULCorner))
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
	if (ImGui::InvisibleButton("SurveyRegionArrow", ArrowLRCorner - ArrowULCorner) && (!ImGui::IsPopupOpen("Popup")))
		ImGui::OpenPopup("Survey Region DropDown");
	bg_color = ImGui::IsItemHovered() ? ImColor(style.Colors[ImGuiCol_ButtonHovered]) : ImColor(style.Colors[ImGuiCol_Button]);
	draw_list->AddRectFilled(ArrowULCorner, ArrowLRCorner, bg_color, 0.0f);
	draw_list->AddRectFilledMultiColor(ArrowULCorner, ArrowLRCorner, col_upr_left, col_upr_right, col_bot_right, col_bot_right);
	
	//Merge draw list channels
	draw_list->ChannelsMerge();
	
	Draw_DropDown(Math::Vector2(RectULCorner.x, RectLRCorner.y), ArrowLRCorner.x - RectULCorner.x);
	
	ImGui::PopID();
	return ((! wasToolActive) && toolActive);
}

void SurveyRegionsTool::Draw_DropDown(Math::Vector2 PopupULCorner, float PopupWidth) {
	auto style = ImGui::GetStyle();
	
	ImRect popup_rect(PopupULCorner, PopupULCorner + m_popupDims);
	ImGui::SetNextWindowPos(popup_rect.Min);
	ImGui::SetNextWindowSize(popup_rect.GetSize());
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, style.FramePadding);
	
	if (ImGui::BeginPopup("Survey Region DropDown")) {
		Math::Vector2 WindowStartCursorPos = ImGui::GetCursorPos();
		
		//Get the name of the currently active survey region
		std::string regionName("None");
		SurveyRegion * surveyRegion = SurveyRegionManager::Instance().GetActiveSurveyRegion();
		if (surveyRegion != nullptr) {
			std::scoped_lock lock(surveyRegion->m_mutex);
			regionName = surveyRegion->m_Name;
		}
		
		ImGui::TextUnformatted("Active Region  ");
		ImGui::SameLine();
		{
			ImExt::Style m_style(StyleVar::FramePadding, ImVec2(0.0f, 0.0f));
			if (ImGui::Button(" Change ", ImVec2(ImGui::GetContentRegionAvail().x, 0)))
				SelectActiveRegionDialog::Instance().Show();
		}
		MyGui::HeaderLabel(regionName.c_str());
		
		if (surveyRegion != nullptr) {
			ImGui::Spacing();
			ImGui::Separator();
			ImGui::Spacing();
			
			if (! toolActive)
				ImGui::TextUnformatted("Activate tool to edit");
			else {
				if (ImGui::Selectable(" New Polygon or Hole "))
					m_toolState = 2;
				
				ImGui::Spacing();
				ImGui::Separator();
				ImGui::Spacing();
				
				if (ImGui::Selectable(" Delete Polygon "))
					m_toolState = 4;
				if (ImGui::Selectable(" Delete Hole "))
					m_toolState = 5;
			}
		}
		
		float popupWidth  = std::max(13.0f*ImGui::GetFontSize(), ImGui::CalcTextSize(regionName.c_str()).x + 2.0f*ImGui::GetFontSize());
		float popupHeight = ImGui::GetCursorPosY() - WindowStartCursorPos.y + 2.0f*style.FramePadding.y;
		m_popupDims       = Math::Vector2(popupWidth, popupHeight);
	
		ImGui::EndPopup();
	}
	
	ImGui::PopStyleVar();
}

void SurveyRegionsTool::DrawTriangle(Math::Vector2 p_min, float scale, ImDrawList * DrawList) {
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

static void DrawPlus(ImDrawList * DrawList, ImVec2 Center_ScreenSpace, ImU32 Color) {
	ImVec2 p_min = Center_ScreenSpace;
	ImVec2 p_max = Center_ScreenSpace;
	p_min.x -= 2.0f;    p_min.y -= 10.0f;
	p_max.x += 2.0f;    p_max.y += 10.0f;
	DrawList->AddRectFilled(p_min, p_max, Color, 0.0f, ImDrawCornerFlags_None);
	//DrawList->AddRectFilled(p_min, p_max, Color, 0.0f, ImDrawCornerFlags_All);
	p_min = Center_ScreenSpace;
	p_max = Center_ScreenSpace;
	p_min.x -= 10.0f;    p_min.y -= 2.0f;
	p_max.x += 10.0f;    p_max.y += 2.0f;
	DrawList->AddRectFilled(p_min, p_max, Color, 0.0f, ImDrawCornerFlags_None);
	//DrawList->AddRectFilled(p_min, p_max, Color, 0.0f, ImDrawCornerFlags_All);
}

void SurveyRegionsTool::Draw_Instructions(std::string const & Text, ImDrawList * DrawList) {
	if (Text.empty())
		return;
	
	ImVec2 TextSize = ImGui::CalcTextSize(Text.c_str());
	ImVec2 BoxSize(TextSize.x + 6.0f*ImGui::GetStyle().FramePadding.x, TextSize.y + 6.0f*ImGui::GetStyle().FramePadding.y);
	
	MapWidget & map(MapWidget::Instance());
	Eigen::Vector2d MapULCorner_ScreenSpace = map.MapWidgetULCorner_ScreenSpace;
	Eigen::Vector2d MapLRCorner_ScreenSpace = MapULCorner_ScreenSpace + map.MapWidgetDims;
	
	float BoxMinX = float(MapULCorner_ScreenSpace(0)) + 6.0f*ImGui::GetStyle().FramePadding.x;
	float BoxMinY = float(MapLRCorner_ScreenSpace(1)) - BoxSize.y - 6.0f*ImGui::GetStyle().FramePadding.y;
	
	float BoxMaxX = BoxMinX + BoxSize.x;
	float BoxMaxY = BoxMinY + BoxSize.y;
	
	ImVec2 BoxMin(BoxMinX, BoxMinY);
	ImVec2 BoxMax(BoxMaxX, BoxMaxY);
	ImVec2 TextMin(BoxMin.x + 3.0f*ImGui::GetStyle().FramePadding.x, BoxMin.y + 3.0f*ImGui::GetStyle().FramePadding.y);
	
	DrawList->AddRectFilled(BoxMin, BoxMax, IM_COL32(70, 70, 70, 255), 5.0f, ImDrawCornerFlags_All);
	DrawList->AddText(TextMin, IM_COL32(255, 255, 255, 255), Text.c_str());
}


void SurveyRegionsTool::Draw_Overlay(ImVec2 CursorPos_ScreenSpace, Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
	//If the survey region layer isn't supposed to be visible just return now
	if (! VisWidget::Instance().LayerVisible_SurveyRegion)
		return;
	
	//Get the active survey region. Return if there isn't one and lock it if there is one.
	SurveyRegion * surveyRegion = SurveyRegionManager::Instance().GetActiveSurveyRegion();
	if (surveyRegion == nullptr)
		return;
	std::scoped_lock lock(surveyRegion->m_mutex);
	
	//Regardless of whether the tool is active, we need to draw the polygon collection
	std::array<float, 3> const & fillRGB(VisWidget::Instance().SurveyRegionColor);
	ImU32 fillColor = ImGui::GetColorU32(ImVec4(fillRGB[0], fillRGB[1], fillRGB[2], VisWidget::Instance().Opacity_SurveyRegion/100.0f));
	auto prevFlags = DrawList->Flags;
	DrawList->Flags = (DrawList->Flags & (~ImDrawListFlags_AntiAliasedFill)); //Disable anti-aliasing to prevent seams along triangle edges
	for (auto const & triangle : surveyRegion->m_triangulation) {
		ImVec2 pointA_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointA);
		ImVec2 pointB_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointB);
		ImVec2 pointC_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointC);
		DrawList->AddTriangleFilled(pointA_ScreenSpace, pointB_ScreenSpace, pointC_ScreenSpace, fillColor);
	}
	DrawList->Flags = prevFlags; //Restore previous flags (restore previous anti-aliasing state)
	
	//If the tool is active, draw additional indicators based on the tool state
	if (toolActive) {
		float nodeRadius_pixels = VisWidget::Instance().SurveyRegionVertexRadius;
		float edgeThickness_pixels = VisWidget::Instance().SurveyRegionEdgeThickness;
		std::string InstructionsStr;
		
		//Get a vector of pointers to each simple polygon in the collection and the address in the collection (outer boundaries and holes)
		std::vector<std::tuple<SimplePolyAddress, SimplePolygon *>> simplePolys;
		for (int compIndex = 0; compIndex < int(surveyRegion->m_Region.m_components.size()); compIndex++) {
			auto & comp(surveyRegion->m_Region.m_components[compIndex]);
			simplePolys.push_back(std::make_tuple(SimplePolyAddress(compIndex, -1), &(comp.m_boundary)));
			for (int holeIndex = 0; holeIndex < int(comp.m_holes.size()); holeIndex++)
				simplePolys.push_back(std::make_tuple(SimplePolyAddress(compIndex, holeIndex), &(comp.m_holes[holeIndex])));
		}
		
		//Convert vertex vectors for each simple poly to screen space and hover-test them
		std::vector<std::vector<ImVec2>> simplePolyVertices_ScreenSpace;
		simplePolyVertices_ScreenSpace.reserve(simplePolys.size());
		VertexAddress hoveredVertexAddress; //Left with componentIndex = -1 if nothing is hovered
		for (auto const & item : simplePolys) {
			auto simplePolyAddress = std::get<0>(item);
			auto simplePolyPtr     = std::get<1>(item);
			std::Evector<Eigen::Vector2d> const & vertices_NM(simplePolyPtr->GetVertices());
			simplePolyVertices_ScreenSpace.emplace_back();
			simplePolyVertices_ScreenSpace.back().reserve(vertices_NM.size());
			for (int index = 0; index < int(vertices_NM.size()); index++) {
				ImVec2 vertex_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(vertices_NM[index]);
				simplePolyVertices_ScreenSpace.back().push_back(vertex_ScreenSpace);
				float deltaX = CursorPos_ScreenSpace.x - vertex_ScreenSpace.x;
				float deltaY = CursorPos_ScreenSpace.y - vertex_ScreenSpace.y;
				float cursorDist_pixels = std::sqrt(deltaX * deltaX + deltaY * deltaY);
				if (cursorDist_pixels < nodeRadius_pixels)
					hoveredVertexAddress = VertexAddress(simplePolyAddress, index);
			}
		}
		
		//Draw Edges (not under edit)
		for (int simplePolyIndex = 0; simplePolyIndex < int(simplePolys.size()); simplePolyIndex++) {
			auto simpPolyAddr = std::get<0>(simplePolys[simplePolyIndex]);
			if ((m_toolState != 1) || (simpPolyAddr.componentIndex != m_editNodeAddress.componentIndex) || (simpPolyAddr.holeIndex != m_editNodeAddress.holeIndex)) {
				std::vector<ImVec2> const & Vertices_ScreenSpace(simplePolyVertices_ScreenSpace[simplePolyIndex]);
				//Do not use the busted-ass AddPolyline() function for this or edge thicknesses will be all over the place
				for (int index = 0; index < int(Vertices_ScreenSpace.size()); index++) {
					ImVec2 p1 = Vertices_ScreenSpace[index];
					ImVec2 p2 = (index + 1 == int(Vertices_ScreenSpace.size())) ? Vertices_ScreenSpace[0] : Vertices_ScreenSpace[index + 1];
					DrawList->AddLine(p1, p2, IM_COL32(0, 0, 0, 255), edgeThickness_pixels);
				}
			}
		}
		
		//Draw edge under edit
		if (! m_editPolyLine_NM.empty()) {
			std::vector<ImVec2> vertices_ScreenSpace;
			vertices_ScreenSpace.reserve(m_editPolyLine_NM.size());
			for (auto const & vertex_NM : m_editPolyLine_NM)
				vertices_ScreenSpace.push_back(MapWidget::Instance().NormalizedMercatorToScreenCoords(vertex_NM));
			for (int index = 0; index < int(vertices_ScreenSpace.size()); index++) {
				ImVec2 p1 = vertices_ScreenSpace[index];
				ImVec2 p2 = (index + 1 == int(vertices_ScreenSpace.size())) ? vertices_ScreenSpace[0] : vertices_ScreenSpace[index + 1];
				if ((m_toolState < 2) || (index + 1 < int(vertices_ScreenSpace.size())))
					DrawList->AddLine(p1, p2, IM_COL32(0, 0, 0, 255), edgeThickness_pixels);
			}
		}
		
		//Draw Nodes of simple polygons not under edit
		if (m_toolState < 2) {
			for (int simplePolyIndex = 0; simplePolyIndex < int(simplePolys.size()); simplePolyIndex++) {
				auto simplePolyAddress = std::get<0>(simplePolys[simplePolyIndex]);
				//auto simplePolyPtr     = std::get<1>(simplePolys[simplePolyIndex]);
				std::vector<ImVec2> const & Vertices_ScreenSpace(simplePolyVertices_ScreenSpace[simplePolyIndex]);
				if ((m_editNodeAddress.componentIndex != simplePolyAddress.componentIndex) || (m_editNodeAddress.holeIndex != simplePolyAddress.holeIndex)) {
					for (int vertexIndex = 0; vertexIndex < int(Vertices_ScreenSpace.size()); vertexIndex++) {
						ImU32 nodeColor = IM_COL32(150, 150, 150, 255);
						if ((m_toolState == 0) && (VertexAddress(simplePolyAddress, vertexIndex) == hoveredVertexAddress))
							nodeColor = IM_COL32(255, 255, 255, 255);
						DrawList->AddCircleFilled(Vertices_ScreenSpace[vertexIndex], nodeRadius_pixels, nodeColor, 30);
					}
				}
			}
		}
		
		//Draw Nodes of simple polygons under edit
		for (int vertexIndex = 0; vertexIndex < int(m_editPolyLine_NM.size()); vertexIndex++) {
			ImVec2 vertex_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(m_editPolyLine_NM[vertexIndex]);
			ImU32 nodeColor = IM_COL32(150, 150, 150, 255);
			if (vertexIndex == m_editNodeAddress.vertexIndex)
				nodeColor = IM_COL32(255, 255, 255, 255);
			DrawList->AddCircleFilled(vertex_ScreenSpace, nodeRadius_pixels, nodeColor, 30);
		}
		
		//Mouse and Keyboard Interaction
		double PixelsPerNMUnit = 1.0 / PixelsToNMUnits(1.0, CursorPos_NM(1), MapWidget::Instance().zoom);
		if (m_toolState == 0) {
			//Default state (nothing going on)
			if (CursorInBounds) {
				//Set cursor and add markings when hovering over a vertex or edge
				if (hoveredVertexAddress.componentIndex >= 0) {
					//We are hovering over vertex
					ImGui::SetMouseCursor(ImGuiMouseCursor_Hand); //Set cursor to hand for this frame
					InstructionsStr = std::string("Press 'Del' or 'Backspace' to delete vertex.");
					if (ImGui::IsMouseClicked(0)) {
						m_editNodeAddress = hoveredVertexAddress;
						Polygon & poly(surveyRegion->m_Region.m_components[m_editNodeAddress.componentIndex]);
						if (m_editNodeAddress.holeIndex < 0)
							m_editPolyLine_NM = poly.m_boundary.GetVertices();
						else
							m_editPolyLine_NM = poly.m_holes[m_editNodeAddress.holeIndex].GetVertices();
						m_toolState = 1;
					}
					else if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Delete)) || ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Backspace))) {
						Polygon & poly(surveyRegion->m_Region.m_components[hoveredVertexAddress.componentIndex]);
						SimplePolygon * simplePoly = &(poly.m_boundary);
						if (hoveredVertexAddress.holeIndex >= 0)
							simplePoly = &(poly.m_holes[hoveredVertexAddress.holeIndex]);
						std::Evector<Eigen::Vector2d> newVertices_NM = simplePoly->GetVertices();
						if ((hoveredVertexAddress.vertexIndex >= 0) && (hoveredVertexAddress.vertexIndex < int(newVertices_NM.size()))) {
							newVertices_NM.erase(newVertices_NM.begin() + hoveredVertexAddress.vertexIndex);
							simplePoly->SetBoundary(newVertices_NM);
							if (simplePoly->NumVertices() == 0) {
								surveyRegion->m_Region.RemoveEmptyHoles();
								surveyRegion->m_Region.RemoveEmptyComponents();
							}
							surveyRegion->m_Region.Triangulate(surveyRegion->m_triangulation); //Update survey region triangulation
						}
					}
				}
				else {
					//Check to see if we are hovering over an edge and handle it if we are
					for (auto const & item : simplePolys) {
						auto simplePolyAddress = std::get<0>(item);
						auto simplePolyPtr     = std::get<1>(item);
						std::Evector<Eigen::Vector2d> const & vertices_NM(simplePolyPtr->GetVertices());
						for (int vertexIndex = 0; vertexIndex < int(vertices_NM.size()); vertexIndex++) {
							Eigen::Vector2d p1 = vertices_NM[vertexIndex];
							Eigen::Vector2d p2 = (vertexIndex + 1 == int(vertices_NM.size())) ? vertices_NM[0] : vertices_NM[vertexIndex + 1];
							LineSegment segment(p1, p2);
							Eigen::Vector2d projection_NM = segment.ProjectPoint(CursorPos_NM);
							double delta_NM = (projection_NM - CursorPos_NM).norm();
							double delta_pixels = PixelsPerNMUnit * delta_NM;
							if (delta_pixels < nodeRadius_pixels) {
								ImVec2 projection_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(projection_NM);
								//DrawPlus(DrawList, projection_ScreenSpace, IM_COL32(255, 255, 255, 255));
								InstructionsStr = std::string("Click to add vertex.");
								if (ImGui::IsMouseClicked(0)) {
									std::Evector<Eigen::Vector2d> newVertices_NM = vertices_NM;
									newVertices_NM.insert(newVertices_NM.begin() + vertexIndex + 1, projection_NM);
									simplePolyPtr->SetBoundary(newVertices_NM);
									surveyRegion->m_Region.Triangulate(surveyRegion->m_triangulation); //Update survey region triangulation
									
									//Jump into drag mode for new vertex
									m_editNodeAddress = VertexAddress(simplePolyAddress, vertexIndex + 1);
									Polygon & poly(surveyRegion->m_Region.m_components[m_editNodeAddress.componentIndex]);
									if (m_editNodeAddress.holeIndex < 0)
										m_editPolyLine_NM = poly.m_boundary.GetVertices();
									else
										m_editPolyLine_NM = poly.m_holes[m_editNodeAddress.holeIndex].GetVertices();
									m_toolState = 1;
								}
								
								break;
							}
						}
					}
				}
			}
		}
		else if (m_toolState == 1) {
			//Dragging node (not during polygon or hole creation)
			if (ImGui::IsMouseDown(0) && (CursorInBounds)) {
				m_editPolyLine_NM[m_editNodeAddress.vertexIndex] = CursorPos_NM;
				ImGui::SetMouseCursor(ImGuiMouseCursor_Hand); //Set cursor to hand for this frame
			}
			else {
				//Drag finished
				Polygon & poly(surveyRegion->m_Region.m_components[m_editNodeAddress.componentIndex]);
				if (m_editNodeAddress.holeIndex < 0)
					poly.m_boundary.SetBoundary(m_editPolyLine_NM);
				else
					poly.m_holes[m_editNodeAddress.holeIndex].SetBoundary(m_editPolyLine_NM);
				surveyRegion->m_Region.Triangulate(surveyRegion->m_triangulation); //Update survey region triangulation
				
				m_editNodeAddress.Reset();
				m_editPolyLine_NM.clear();
				m_toolState = 0;
			}
		}
		else if (m_toolState == 2) {
			//2 = Creating polygon or hole - not dragging node
			if (CursorInBounds) {
				bool vertexHovered = false;
				for (int vertexIndex = 0; vertexIndex < int(m_editPolyLine_NM.size()); vertexIndex++) {
					ImVec2 vertex_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(m_editPolyLine_NM[vertexIndex]);
					float deltaX = CursorPos_ScreenSpace.x - vertex_ScreenSpace.x;
					float deltaY = CursorPos_ScreenSpace.y - vertex_ScreenSpace.y;
					float cursorDist_pixels = std::sqrt(deltaX * deltaX + deltaY * deltaY);
					if (cursorDist_pixels < nodeRadius_pixels) {
						//This vertex is hovered
						vertexHovered = true;
						
						//Hovering over the initial vertex
						if ((vertexIndex == 0) && (m_editPolyLine_NM.size() >= 3U)) {
							InstructionsStr = std::string("Click to finish object.\nPress 'Del' or 'Backspace' to delete vertex.");
							
							//Add a visual indicator at the point of action that finishing the polygon is happening
							ImVec2 vertex_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(m_editPolyLine_NM[0]);
							DrawList->AddCircleFilled(vertex_ScreenSpace, 2.0f*nodeRadius_pixels, IM_COL32(255, 255, 255, 255), 30);
							
							if (ImGui::IsMouseClicked(0)) {
								FinishNewPolyOrHole(surveyRegion);
								break;
							}
						}
						else {
							ImGui::SetMouseCursor(ImGuiMouseCursor_Hand); //Set cursor to hand for this frame
							InstructionsStr = std::string("Press 'Del' or 'Backspace' to delete vertex.");
							if (ImGui::IsMouseClicked(0)) {
								m_editNodeAddress = VertexAddress(-1, -1, vertexIndex);
								m_toolState = 3;
								break;
							}
						}
						
						if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Delete)) || ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Backspace))) {
							if (m_editPolyLine_NM.size() <= 1U) {
								//They are deleting the only vertex - destroy the object
								m_editNodeAddress.Reset();
								m_editPolyLine_NM.clear();
								m_toolState = 0;
							}
							else {
								//They are deleting a vertex, but there are more than 1
								m_editPolyLine_NM.erase(m_editPolyLine_NM.begin() + vertexIndex);
							}
						}
						
						break;
					}
				}
				if (! vertexHovered) {
					InstructionsStr = std::string("Click to place new vertex.");
					if (ImGui::IsMouseClicked(0)) {
						m_editNodeAddress = VertexAddress(-1, -1, (int) m_editPolyLine_NM.size());
						m_editPolyLine_NM.push_back(CursorPos_NM);
						m_toolState = 3;
					}
				}
			}
		}
		else if (m_toolState == 3) {
			//Creating polygon or hole - dragging node
			bool lastVertexBeingDragged = false;
			bool lastVertexIsOverFirst  = false;
			if ((m_editPolyLine_NM.size() > 3U) && (m_editNodeAddress.vertexIndex + 1 == (int) m_editPolyLine_NM.size())) {
				lastVertexBeingDragged = true;
				Eigen::Vector2d p1_NM = m_editPolyLine_NM[m_editNodeAddress.vertexIndex];
				Eigen::Vector2d p2_NM = m_editPolyLine_NM[0];
				double delta_NM = (p2_NM - p1_NM).norm();
				double delta_pixels = PixelsPerNMUnit * delta_NM;
				lastVertexIsOverFirst = (delta_pixels < nodeRadius_pixels);
			}
			
			if (ImGui::IsMouseDown(0) && (CursorInBounds)) {
				m_editPolyLine_NM[m_editNodeAddress.vertexIndex] = CursorPos_NM;
				ImGui::SetMouseCursor(ImGuiMouseCursor_Hand); //Set cursor to hand for this frame
				if (lastVertexBeingDragged && (! lastVertexIsOverFirst))
					InstructionsStr = "Drop on initial vertex to complete object."s;
				else if (lastVertexBeingDragged && lastVertexIsOverFirst) {
					InstructionsStr = "Drop to complete object."s;
					
					//Add a visual indicator at the point of action that finishing the polygon is happening
					ImVec2 vertex_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(m_editPolyLine_NM[0]);
					DrawList->AddCircleFilled(vertex_ScreenSpace, 2.0f*nodeRadius_pixels, IM_COL32(255, 255, 255, 255), 30);
				}
			}
			else {
				//Drag finished
				if (lastVertexIsOverFirst) {
					m_editPolyLine_NM.pop_back(); //Merge the first and last vertices by deleting the last one
					FinishNewPolyOrHole(surveyRegion);
				}
				else {
					m_editNodeAddress.Reset();
					m_toolState = 2;
				}
			}
		}
		else if (m_toolState == 4) {
			//Deleting Polygon (awaiting click)
			InstructionsStr = "Click on polygon to delete."s;
			if (ImGui::IsMouseClicked(0)) {
				for (int compIndex = 0; compIndex < (int) surveyRegion->m_Region.m_components.size(); compIndex++) {
					if (surveyRegion->m_Region.m_components[compIndex].ContainsPoint(CursorPos_NM)) {
						surveyRegion->m_Region.m_components.erase(surveyRegion->m_Region.m_components.begin() + compIndex);
						surveyRegion->m_Region.Triangulate(surveyRegion->m_triangulation); //Update survey region triangulation
						break;
					}
				}
				m_toolState = 0;
			}
		}
		else if (m_toolState == 5) {
			//Deleting Hole (awaiting click)
			InstructionsStr = "Click on hole to delete."s;
			if (ImGui::IsMouseClicked(0)) {
				for (auto & comp : surveyRegion->m_Region.m_components) {
					if (comp.m_boundary.ContainsPoint(CursorPos_NM)) {
						//If a hole was clicked on it must be in this component
						for (size_t holeIndex = 0U; holeIndex < comp.m_holes.size(); holeIndex++) {
							if (comp.m_holes[holeIndex].ContainsPoint(CursorPos_NM)) {
								comp.m_holes.erase(comp.m_holes.begin() + holeIndex);
								surveyRegion->m_Region.Triangulate(surveyRegion->m_triangulation); //Update survey region triangulation
								break;
							}
						}
						
						break;
					}
				}
				m_toolState = 0;
			}
		}
		
		Draw_Instructions(InstructionsStr, DrawList);
	}
}

//Finish a new polygon or hole - update the survey region and tool state accordingly. A lock should already be held on the surveyRegion.
void SurveyRegionsTool::FinishNewPolyOrHole(SurveyRegion * surveyRegion) {
	//Detirmine whether this is a new polygon or a hole. This is detirmined based on whether the first vertex
	//was placed within the interior of the existing collection. If so this is a hole. Otherwise it is a new polygon.
	int indexOfCompContainingFirstVertex = -1;
	for (int compIndex = 0; compIndex < (int) surveyRegion->m_Region.m_components.size(); compIndex++) {
		if (surveyRegion->m_Region.m_components[compIndex].ContainsPoint(m_editPolyLine_NM[0])) {
			indexOfCompContainingFirstVertex = compIndex;
			break;
		}
	}
	
	if (indexOfCompContainingFirstVertex >= 0) {
		//This is a hole - add it to the polygon that contains the first vertex
		SimplePolygon newSimplePoly(m_editPolyLine_NM);
		surveyRegion->m_Region.m_components[indexOfCompContainingFirstVertex].m_holes.push_back(newSimplePoly);
		surveyRegion->m_Region.Triangulate(surveyRegion->m_triangulation); //Update survey region triangulation
		m_editNodeAddress.Reset();
		m_editPolyLine_NM.clear();
		m_toolState = 0;
	}
	else {
		//This is a new polygon
		SimplePolygon newSimplePoly(m_editPolyLine_NM);
		surveyRegion->m_Region.m_components.emplace_back(newSimplePoly);
		surveyRegion->m_Region.Triangulate(surveyRegion->m_triangulation); //Update survey region triangulation
		m_editNodeAddress.Reset();
		m_editPolyLine_NM.clear();
		m_toolState = 0;
	}
}





