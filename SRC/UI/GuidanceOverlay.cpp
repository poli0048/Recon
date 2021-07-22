//The guidance overlay is used by the Guidance module to draw guidance-related data on the map widget
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//Project Includes
#include "GuidanceOverlay.hpp"
#include "MapWidget.hpp"
#include "VisWidget.hpp"
#include "../Modules/Guidance/Guidance.hpp"

//Select a color based on an items index in a container of size N. This is for selecting colors to render objects in that will generally
//result in high visual distinction between objects with nearby indices. Opacity is on a [0,1] scale
ImU32 GuidanceOverlay::IndexToColor(size_t Index, size_t N, float Opacity) {
	float R = 0.0f, G = 0.0f, B = 0.0f;
	
	//Give each triangle a unique'ish hue, evenly divided in angle-space
	//float H = float(Index) / float(N);
	
	//Cycle between manually chosen, highly distinctive colors
	float H = 0.0f;
	switch (((unsigned int) Index) % 7U) {
		case 0U: H = 0.0f;          break;
		case 1U: H = 120.0f/360.0f; break;
		case 2U: H = 180.0f/360.0f; break;
		case 3U: H = 300.0f/360.0f; break;
		case 4U: H = 60.0f/360.0f;  break;
		case 5U: H = 225.0f/360.0f; break;
		case 6U: H = 30.0f/360.0f;  break;
		default: H = 0.0f; break;
	}
	
	float S = 1.0f, V = 1.0f;
	ImGui::ColorConvertHSVtoRGB(H, S, V, R, G, B);
	return ImGui::GetColorU32(ImVec4(R, G, B, Opacity));
}

//Called in the draw loop for the map widget
void GuidanceOverlay::Draw_Overlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
	//Nothing to do if the overlay is disabled
	if (! VisWidget::Instance().LayerVisible_GuidanceOverlay)
		return;
	
	//Don't draw the overlay if the guidance module isn't running
	if (! Guidance::GuidanceEngine::Instance().IsRunning())
		return;
	
	std::scoped_lock lock(m_mutex);
	if (VisWidget::Instance().GuidanceOverlay_ShowTrianglesInsteadOfPartition) {
		//Draw the triangle collection
		if ((! m_Triangles.empty()) && (VisWidget::Instance().Opacity_GuidanceOverlay > 0.0f)) {
			//Draw the triangle interiors
			auto prevFlags = DrawList->Flags;
			DrawList->Flags = (DrawList->Flags & (~ImDrawListFlags_AntiAliasedFill)); //Disable anti-aliasing to prevent seams along triangle edges
			for (size_t triangleIndex = 0U; triangleIndex < m_Triangles.size(); triangleIndex++) {
				Triangle const & triangle(m_Triangles[triangleIndex]);
				
				//Get a good color to render this triangle in
				ImU32 triangleColor = IndexToColor(triangleIndex, m_Triangles.size(), VisWidget::Instance().Opacity_GuidanceOverlay/100.0f);
				
				Eigen::Vector2d pointA_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointA);
				Eigen::Vector2d pointB_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointB);
				Eigen::Vector2d pointC_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointC);
				DrawList->AddTriangleFilled(pointA_ScreenSpace, pointB_ScreenSpace, pointC_ScreenSpace, triangleColor);
			}
			DrawList->Flags = prevFlags; //Restore previous flags (restore previous anti-aliasing state)
			
			//Draw the triangle boundaries
			float edgeThickness_pixels = VisWidget::Instance().SurveyRegionEdgeThickness; //Just use the edge thickness for survey regions for now
			for (size_t triangleIndex = 0U; triangleIndex < m_Triangles.size(); triangleIndex++) {
				Triangle const & triangle(m_Triangles[triangleIndex]);
				Eigen::Vector2d PA_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointA);
				Eigen::Vector2d PB_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointB);
				Eigen::Vector2d PC_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointC);
				
				DrawList->AddLine(PA_SS, PB_SS, IM_COL32(0, 0, 0, 255.0f*VisWidget::Instance().Opacity_GuidanceOverlay/100.0f), edgeThickness_pixels);
				DrawList->AddLine(PB_SS, PC_SS, IM_COL32(0, 0, 0, 255.0f*VisWidget::Instance().Opacity_GuidanceOverlay/100.0f), edgeThickness_pixels);
				DrawList->AddLine(PC_SS, PA_SS, IM_COL32(0, 0, 0, 255.0f*VisWidget::Instance().Opacity_GuidanceOverlay/100.0f), edgeThickness_pixels);
			}
		}
	}
	else {
		//Draw the survey region partition
		if ((! m_SurveyRegionPartitionTriangulation.empty()) && (VisWidget::Instance().Opacity_GuidanceOverlay > 0.0f)) {
			//Render triangles (the interiors of each component)
			for (size_t compIndex = 0U; compIndex < m_SurveyRegionPartitionTriangulation.size(); compIndex++) {
				std::Evector<Triangle> const & compTriangulation(m_SurveyRegionPartitionTriangulation[compIndex]);
				
				//Get a good color to render this component in
				ImU32 compColor = IndexToColor(compIndex, m_SurveyRegionPartitionTriangulation.size(), VisWidget::Instance().Opacity_GuidanceOverlay/100.0f);
				
				auto prevFlags = DrawList->Flags;
				DrawList->Flags = (DrawList->Flags & (~ImDrawListFlags_AntiAliasedFill)); //Disable anti-aliasing to prevent seams along triangle edges
				for (Triangle const & triangle : compTriangulation) {
					Eigen::Vector2d pointA_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointA);
					Eigen::Vector2d pointB_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointB);
					Eigen::Vector2d pointC_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointC);
					DrawList->AddTriangleFilled(pointA_ScreenSpace, pointB_ScreenSpace, pointC_ScreenSpace, compColor);
				}
				DrawList->Flags = prevFlags; //Restore previous flags (restore previous anti-aliasing state)
			}
			
			//After rendering the interiors of all components, render their boundaries
			float edgeThickness_pixels = VisWidget::Instance().SurveyRegionEdgeThickness; //Just use the edge thickness for survey regions for now
			for (PolygonCollection const & polyCollection : m_SurveyRegionPartition) {
				for (Polygon const & poly : polyCollection.m_components) {
					std::vector<SimplePolygon const *> simplePolys;
					simplePolys.reserve(1U + poly.m_holes.size());
					simplePolys.push_back(&(poly.m_boundary));
					for (SimplePolygon const & simplePoly : poly.m_holes)
						simplePolys.push_back(& simplePoly);
					
					//Draw each simple poly
					for (SimplePolygon const * simplePoly : simplePolys) {
						std::Evector<Eigen::Vector2d> const & Vertices_NM(simplePoly->GetVertices());
						std::Evector<Eigen::Vector2d> Vertices_SS;
						Vertices_SS.reserve(Vertices_NM.size());
						
						for (auto const & Vertex_NM : Vertices_NM)
							Vertices_SS.push_back(MapWidget::Instance().NormalizedMercatorToScreenCoords(Vertex_NM));
						
						for (int index = 0; index < int(Vertices_SS.size()); index++) {
							Eigen::Vector2d p1 = Vertices_SS[index];
							Eigen::Vector2d p2 = (index + 1 == int(Vertices_SS.size())) ? Vertices_SS[0] : Vertices_SS[index + 1];
							DrawList->AddLine(p1, p2, IM_COL32(0, 0, 0, 255.0f*VisWidget::Instance().Opacity_GuidanceOverlay/100.0f), edgeThickness_pixels);
						}
					}
				}
			}
		}
	}
}

void GuidanceOverlay::Draw_MessageBox(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
	//Nothing to do if the overlay is disabled
	if (! VisWidget::Instance().LayerVisible_GuidanceOverlay)
		return;
	
	//Don't draw the overlay if the guidance module isn't running
	if (! Guidance::GuidanceEngine::Instance().IsRunning())
		return;
	
	std::scoped_lock lock(m_mutex);
	
	bool somethingToDisplay = (! m_GuidanceMessage_1.empty()) || (! m_GuidanceMessage_2.empty()) || (! m_GuidanceMessage_3.empty());
	if (VisWidget::Instance().GuidanceOverlay_ShowMessageBox && somethingToDisplay) {
		std::string text = m_GuidanceMessage_1;
		if ((! m_GuidanceMessage_1.empty()) && ((! m_GuidanceMessage_2.empty()) || (! m_GuidanceMessage_3.empty())))
			text += "\n"s;
		text += m_GuidanceMessage_2;
		if ((! m_GuidanceMessage_2.empty()) && (! m_GuidanceMessage_3.empty()))
			text += "\n"s;
		text += m_GuidanceMessage_3;
	
		ImVec2 TextSize = ImGui::CalcTextSize(text.c_str());
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
		DrawList->AddText(TextMin, IM_COL32(255, 255, 255, 255), text.c_str());
	}
}

//Data Setter Methods
void GuidanceOverlay::Reset() {
	std::scoped_lock lock(m_mutex);
	m_SurveyRegionPartition.clear();
	m_SurveyRegionPartitionTriangulation.clear();
	m_Triangles.clear();
	m_GuidanceMessage_1.clear();
	m_GuidanceMessage_2.clear();
	m_GuidanceMessage_3.clear();
}

//Set the partition of the survey region to draw
void GuidanceOverlay::SetSurveyRegionPartition(std::Evector<PolygonCollection> const & Partition) {
	std::scoped_lock lock(m_mutex);
	m_SurveyRegionPartition = Partition;
	m_SurveyRegionPartitionTriangulation.clear();
	m_SurveyRegionPartitionTriangulation.reserve(Partition.size());
	for (auto const & comp : Partition) {
		std::Evector<Triangle> triangles;
		comp.Triangulate(triangles);
		m_SurveyRegionPartitionTriangulation.push_back(triangles);
	}
}

//Clear/delete the partition of the survey region
void GuidanceOverlay::ClearSurveyRegionPartition() {
	std::scoped_lock lock(m_mutex);
	m_SurveyRegionPartition.clear();
	m_SurveyRegionPartitionTriangulation.clear();
}

//Set the collection of triangles
void GuidanceOverlay::SetTriangles(std::Evector<Triangle> const & Triangles) {
	std::scoped_lock lock(m_mutex);
	m_Triangles = Triangles;
}

//Clear/delete the collection of triangles
void GuidanceOverlay::ClearTriangles(void) {
	std::scoped_lock lock(m_mutex);
	m_Triangles.clear();
}

//Display optional message in box on map (give empty string to disable)
void GuidanceOverlay::SetGuidanceMessage1(std::string const & Message) {
	std::scoped_lock lock(m_mutex);
	m_GuidanceMessage_1 = Message;
}

//Display optional message in box on map (give empty string to disable)
void GuidanceOverlay::SetGuidanceMessage2(std::string const & Message) {
	std::scoped_lock lock(m_mutex);
	m_GuidanceMessage_2 = Message;
}

//Display optional message in box on map (give empty string to disable)
void GuidanceOverlay::SetGuidanceMessage3(std::string const & Message) {
	std::scoped_lock lock(m_mutex);
	m_GuidanceMessage_3 = Message;
}




