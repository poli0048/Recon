//The guidance overlay is used by the Guidance module to draw guidance-related data on the map widget
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//Project Includes
#include "GuidanceOverlay.hpp"
#include "MapWidget.hpp"
#include "VisWidget.hpp"
#include "../Modules/Guidance/Guidance.hpp"
#include "MyGui.hpp"

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

//Get Axis-Aligned Bounding Box for a triangle
Math::Vector4 GetAABB(Eigen::Vector2d const & A, Eigen::Vector2d const & B, Eigen::Vector2d const & C) {
	double xmin = std::min(std::min(A(0), B(0)), C(0));
	double xmax = std::max(std::max(A(0), B(0)), C(0));
	double ymin = std::min(std::min(A(1), B(1)), C(1));
	double ymax = std::max(std::max(A(1), B(1)), C(1));
	return Math::Vector4(xmin, xmax, ymin, ymax);
}

//Get the centroid and Axis-Aligned Bounding Box for a collection of points
void GetCentroidAndAABB(std::Evector<Eigen::Vector2d> const & points, Eigen::Vector2d & Centroid, Math::Vector4 & AABB) {
	if (points.empty())
		return;
	double xmin = (points[0])(0);
	double xmax = (points[0])(0);
	double ymin = (points[0])(1);
	double ymax = (points[0])(1);
	Centroid << 0.0, 0.0;
	for (auto const & p : points) {
		xmin = std::min(xmin, p(0));
		xmax = std::max(xmax, p(0));
		ymin = std::min(ymin, p(1));
		ymax = std::max(ymax, p(1));
		Centroid += p;
	}
	Centroid = Centroid / double(points.size());
	AABB = Math::Vector4(xmin, xmax, ymin, ymax);
}

Eigen::Vector2d GetCentroid(std::Evector<Eigen::Vector2d> const & points) {
	Eigen::Vector2d sum(0.0, 0.0);
	for (auto const & p : points)
		sum += p;
	return sum / std::max(double(points.size()), 1.0);
}

//If CenteredLabels is true, labels are centered in the first polygon component of each item in the partition. If false,
//labels are drawn underneath the first polygon component.
//If HideComponentsMarketComplete is true, hide sub-regions listed in m_CompletedSubRegions. Otherwise, draw all components
void GuidanceOverlay::Draw_Partition(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds,
                                     std::vector<ImU32> const & Colors, std::vector<std::string> const & Labels, bool CenteredLabels,
                                     bool HideComponentsMarketComplete) const {
	float opacity = VisWidget::Instance().Opacity_GuidanceOverlay; //Opacity for overlay (0-100)
	if ((! m_SurveyRegionPartitionTriangulation.empty()) && (opacity > 0.0f)) {
		//Render triangles (the interiors of each component)
		for (size_t compIndex = 0U; compIndex < m_SurveyRegionPartitionTriangulation.size(); compIndex++) {
			std::Evector<Triangle> const & compTriangulation(m_SurveyRegionPartitionTriangulation[compIndex]);
			
			//Get a good color to render this component in
			//ImU32 compColor = IndexToColor(compIndex, m_SurveyRegionPartitionTriangulation.size(), opacity/100.0f);
			
			bool drawComp = ((! HideComponentsMarketComplete) || (m_CompletedSubRegions.count((int) compIndex) == 0U));
			if (drawComp) {
				auto prevFlags = DrawList->Flags;
				DrawList->Flags = (DrawList->Flags & (~ImDrawListFlags_AntiAliasedFill)); //Disable anti-aliasing to prevent seams along triangle edges
				for (Triangle const & triangle : compTriangulation) {
					Eigen::Vector2d pointA_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointA);
					Eigen::Vector2d pointB_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointB);
					Eigen::Vector2d pointC_ScreenSpace = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointC);
					DrawList->AddTriangleFilled(pointA_ScreenSpace, pointB_ScreenSpace, pointC_ScreenSpace, Colors[compIndex]);
				}
				DrawList->Flags = prevFlags; //Restore previous flags (restore previous anti-aliasing state)
			}
		}
		
		//After rendering the interiors of all components, render their boundaries
		float edgeThickness_pixels = VisWidget::Instance().SurveyRegionEdgeThickness; //Just use the edge thickness for survey regions for now
		std::Evector<std::Evector<Eigen::Vector2d>> compVertices_SS;
		compVertices_SS.reserve(m_SurveyRegionPartition.size());
		for (size_t compIndex = 0U; compIndex < m_SurveyRegionPartition.size(); compIndex++) {
			PolygonCollection const & polyCollection(m_SurveyRegionPartition[compIndex]);
			
			bool drawComp = ((! HideComponentsMarketComplete) || (m_CompletedSubRegions.count((int) compIndex) == 0U));
			if (drawComp) {
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
							DrawList->AddLine(p1, p2, IM_COL32(0, 0, 0, 255.0f*opacity/100.0f), edgeThickness_pixels);
						}
					}
				}
			}
			
			//Get the vertices of the boundary of the first component simple poly and convert to screen space for label placement
			compVertices_SS.emplace_back();
			if (! polyCollection.m_components.empty()) {
				std::Evector<Eigen::Vector2d> Vertices_NM = polyCollection.m_components[0].m_boundary.GetVertices();
				compVertices_SS.back().reserve(Vertices_NM.size());
				for (auto const & Vertex_NM : Vertices_NM)
					compVertices_SS.back().push_back(MapWidget::Instance().NormalizedMercatorToScreenCoords(Vertex_NM));
			}
		}
		
		//Draw labels
		for (size_t compIndex = 0U; compIndex < Labels.size(); compIndex++) {
			if (compIndex >= m_SurveyRegionPartition.size())
				break;
			
			std::string const & label(Labels[compIndex]);
			if (label.empty())
				continue;

			bool drawComp = ((! HideComponentsMarketComplete) || (m_CompletedSubRegions.count((int) compIndex) == 0U));
			if (! drawComp)
				continue;

			std::Evector<Eigen::Vector2d> const & Vertices_SS(compVertices_SS[compIndex]);
			Eigen::Vector2d textSize = ImGui::CalcTextSize(label.c_str());

			if (CenteredLabels) {
				//Draw if the text can be fully contained within the first component simple polygon (maybe overkill, but nice)
				Eigen::Vector2d Centroid_SS = GetCentroid(Vertices_SS);
				SimplePolygon sp_SS(Vertices_SS);
				Eigen::Vector2d p1 = Centroid_SS + Eigen::Vector2d(-0.5*textSize(0), -0.5*textSize(1));
				Eigen::Vector2d p2 = Centroid_SS + Eigen::Vector2d(-0.5*textSize(0),  0.5*textSize(1));
				Eigen::Vector2d p3 = Centroid_SS + Eigen::Vector2d( 0.5*textSize(0), -0.5*textSize(1));
				Eigen::Vector2d p4 = Centroid_SS + Eigen::Vector2d( 0.5*textSize(0),  0.5*textSize(1));
				if (sp_SS.ContainsPoint(p1) && sp_SS.ContainsPoint(p2) && sp_SS.ContainsPoint(p3) && sp_SS.ContainsPoint(p4))
					MyGui::AddText(DrawList, Centroid_SS, IM_COL32(255, 255, 255, 255), label.c_str(), NULL, true, true);
			}
			else {
				//Draw under first component simple polygon if the text isn't way wider than region
				Eigen::Vector2d Centroid_SS;
				Math::Vector4 AABB;
				GetCentroidAndAABB(Vertices_SS, Centroid_SS, AABB);
				Eigen::Vector2d loweredCentroid_SS(Centroid_SS(0), AABB.w + 0.6 * textSize(1));
				if (AABB.y - AABB.x > 0.75f * textSize(0))
					MyGui::AddText(DrawList, loweredCentroid_SS, IM_COL32(255, 255, 255, 255), label.c_str(), NULL, true, true);
			}
		}
	}
}

//Get a point inside the given component that is roughly near the center of the first polygon in the component poly collection
//The returned point is in Normalized Mercator. CompIndex must be a valid index - this is not checked.
Eigen::Vector2d GuidanceOverlay::GetCentralPointForComponentOfPartition(size_t CompIndex) const {
	PolygonCollection const & polyCollection(m_SurveyRegionPartition[CompIndex]);
	Polygon const & poly(polyCollection.m_components[0]);
	SimplePolygon const & boundary(poly.m_boundary);
	std::Evector<Eigen::Vector2d> const & vertices(boundary.GetVertices());

	Eigen::Vector2d centroid(0.0, 0.0);
	for (Eigen::Vector2d const & v : vertices)
		centroid += v;
	if (! vertices.empty())
		centroid /= double(vertices.size());

	return boundary.ProjectPoint(centroid);
}

//Find the intersection of the line from A to B with the boundary of the first element of the given component.
//If there are multiple intersections, find the closest one to point A.
Eigen::Vector2d GuidanceOverlay::FindLinesIntersectionWithBoundary(size_t CompIndex, Eigen::Vector2d const & A_NM, Eigen::Vector2d const & B_NM) const {
	PolygonCollection const & polyCollection(m_SurveyRegionPartition[CompIndex]);
	Polygon const & poly(polyCollection.m_components[0]);
	SimplePolygon const & boundary(poly.m_boundary);
	std::Evector<LineSegment> boundarySegments = boundary.GetLineSegments();

	Eigen::Vector2d V_NM = B_NM - A_NM;
	V_NM.normalize();
	if (V_NM.norm() < 0.5) {
		//A and B are equal to machine precision - we can't really do what we want, so as a fallback project A to the boundary
		return boundary.ProjectPointToBoundary(A_NM);
	}
	
	LineSegment line(A_NM, B_NM);
	Eigen::Vector2d firstIntersection(0.0, 0.0);
	double bestDist = std::nanf("");
	for (auto const & seg : boundarySegments) {
		Eigen::Vector2d intersection;
		if (line.ComputeIntersection(seg, intersection)) {
			double forwardDistFromA = (intersection - A_NM).dot(V_NM);
			if (forwardDistFromA > 0.0) {
				//The point projects on the B side of A and not the opposite side.
				if ((std::isnan(bestDist)) || (forwardDistFromA < bestDist)) {
					firstIntersection = intersection;
					bestDist = forwardDistFromA;
				}
			}
		}
	}
	if (std::isnan(bestDist)) {
		//There is no intersection between the line A --- B and the component boundary. This might be a round-off error
		//issue though, possibly because A is very nearly inside the polygon already. Regardless, the best fallback we can
		//do here is to project A to the boundary and use that instead.
		return boundary.ProjectPointToBoundary(A_NM);
	}
	else
		return firstIntersection;
}

//Get colors for each component of the partition based on the component index (so each comp gets a different color)
void GuidanceOverlay::GetPartColorsByComponent(std::vector<ImU32> & Colors, float Opacity) const {
	Colors.resize(m_SurveyRegionPartition.size());
	for (size_t compIndex = 0U; compIndex < m_SurveyRegionPartition.size(); compIndex++)
		Colors[compIndex] = IndexToColor(compIndex, m_SurveyRegionPartition.size(), Opacity/100.0f);;
}

//Get colors for each component of the partition based on which drone is tasked to fly it
//TaskedDrones: Object n is the index of the drone that will fly component n
void GuidanceOverlay::GetPartColorsByTaskedDrone(std::vector<ImU32> & Colors, float Opacity, std::vector<int> const & TaskedDrones) const {
	Colors.resize(m_SurveyRegionPartition.size());
	for (size_t compIndex = 0U; compIndex < m_SurveyRegionPartition.size(); compIndex++) {
		if (TaskedDrones[compIndex] < 0)
			Colors[compIndex] = ImGui::GetColorU32(ImVec4(1.0f, 0.5f, 0.2f, Opacity/200.0f));
		else
			Colors[compIndex] = IndexToColor((size_t) TaskedDrones[compIndex], m_Sequences.size(), Opacity/100.0f);
	}
}

//Draw visual indication of mission sequences - arrows between regions, in the order they will be flown
void GuidanceOverlay::Draw_MissionSequenceArrows(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds, float Opacity) const {
	for (int droneIndex = 0; droneIndex < (int) m_Sequences.size(); droneIndex++) {
		for (int missionNumber = 0; missionNumber + 1 < (int) m_Sequences[droneIndex].size(); missionNumber++) {
			int comp1Index = (m_Sequences[droneIndex])[missionNumber];
			int comp2Index = (m_Sequences[droneIndex])[missionNumber + 1];
			if ((comp1Index >= 0) && (comp1Index < (int) m_SurveyRegionPartition.size()) &&
			    (comp2Index >= 0) && (comp2Index < (int) m_SurveyRegionPartition.size())) {
				Eigen::Vector2d comp1Center_NM = GetCentralPointForComponentOfPartition(size_t(comp1Index));
				Eigen::Vector2d comp2Center_NM = GetCentralPointForComponentOfPartition(size_t(comp2Index));

				Eigen::Vector2d arrowStart_NM  = FindLinesIntersectionWithBoundary(comp1Index, comp2Center_NM, comp1Center_NM);
				Eigen::Vector2d arrowEnd_NM    = FindLinesIntersectionWithBoundary(comp2Index, comp1Center_NM, comp2Center_NM);

				Eigen::Vector2d comp1Center_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(comp1Center_NM);
				Eigen::Vector2d comp2Center_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(comp2Center_NM);
				Eigen::Vector2d arrowStart_SS  = MapWidget::Instance().NormalizedMercatorToScreenCoords(arrowStart_NM);
				Eigen::Vector2d arrowEnd_SS    = MapWidget::Instance().NormalizedMercatorToScreenCoords(arrowEnd_NM);

				//Push the arrow endpoints past the boundaries a little - this also ensures a min arrow length
				Eigen::Vector2d v_SS = comp2Center_SS - comp1Center_SS;
				v_SS.normalize();
				if (v_SS.norm() > 0.5) {
					arrowStart_SS = arrowStart_SS - 1.5 * ImGui::GetFontSize() * v_SS;
					arrowEnd_SS   = arrowEnd_SS   + 1.5 * ImGui::GetFontSize() * v_SS;
				}
				if ((arrowEnd_SS - arrowStart_SS).norm() < 1.5*(comp2Center_SS - comp1Center_SS).norm()) {
					//Only draw if at an appropriate zoom level
					ImVec4 colorVec4(1.0f, 1.0f, 1.0f, Opacity/100.0f);
					MyGui::AddArrow(DrawList, arrowStart_SS, arrowEnd_SS, ImGui::GetColorU32(colorVec4), 3.0f, 0.8f*ImGui::GetFontSize());
				}
			}
		}
	}
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
	float opacity = VisWidget::Instance().Opacity_GuidanceOverlay; //Opacity for overlay (0-100)
	if (VisWidget::Instance().GuidanceOverlay_View == 0) {
		//Draw the survey region partition
		std::vector<ImU32> colors;
		GetPartColorsByComponent(colors, opacity);
		Draw_Partition(CursorPos_NM, DrawList, CursorInBounds, colors, m_PartitionLabels, true, false);
	}
	else if (VisWidget::Instance().GuidanceOverlay_View == 1) {
		//Draw the triangle collection
		if ((! m_Triangles.empty()) && (opacity > 0.0f)) {
			//Draw the triangle interiors
			auto prevFlags = DrawList->Flags;
			DrawList->Flags = (DrawList->Flags & (~ImDrawListFlags_AntiAliasedFill)); //Disable anti-aliasing to prevent seams along triangle edges
			std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d>> triangles_SS; //triangles in screen space
			triangles_SS.reserve(m_Triangles.size());
			for (size_t triangleIndex = 0U; triangleIndex < m_Triangles.size(); triangleIndex++) {
				//Convert the triangle vertices to screen space
				Triangle const & triangle(m_Triangles[triangleIndex]);
				Eigen::Vector2d pointA_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointA);
				Eigen::Vector2d pointB_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointB);
				Eigen::Vector2d pointC_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(triangle.m_pointC);
				triangles_SS.push_back(std::make_tuple(pointA_SS, pointB_SS, pointC_SS));
				
				//Get a good color to render this triangle in, then draw it
				ImU32 triangleColor = IndexToColor(triangleIndex, m_Triangles.size(), opacity/100.0f);
				DrawList->AddTriangleFilled(pointA_SS, pointB_SS, pointC_SS, triangleColor);
			}
			DrawList->Flags = prevFlags; //Restore previous flags (restore previous anti-aliasing state)
			
			//Draw the triangle boundaries
			float edgeThickness_pixels = VisWidget::Instance().SurveyRegionEdgeThickness; //Just use the edge thickness for survey regions for now
			for (size_t triangleIndex = 0U; triangleIndex < m_Triangles.size(); triangleIndex++) {
				std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d> const & T_SS(triangles_SS[triangleIndex]);
				
				DrawList->AddLine(std::get<0>(T_SS), std::get<1>(T_SS), IM_COL32(0, 0, 0, 255.0f*opacity/100.0f), edgeThickness_pixels);
				DrawList->AddLine(std::get<1>(T_SS), std::get<2>(T_SS), IM_COL32(0, 0, 0, 255.0f*opacity/100.0f), edgeThickness_pixels);
				DrawList->AddLine(std::get<2>(T_SS), std::get<0>(T_SS), IM_COL32(0, 0, 0, 255.0f*opacity/100.0f), edgeThickness_pixels);
			}
			
			//Draw the triangle labels
			for (size_t triangleIndex = 0U; triangleIndex < m_TriangleLabels.size(); triangleIndex++) {
				if (triangleIndex >= m_Triangles.size())
					break;
				
				std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d> const & T_SS(triangles_SS[triangleIndex]);
				std::string const & label(m_TriangleLabels[triangleIndex]);
				
				Eigen::Vector2d Center_SS = (std::get<0>(T_SS) + std::get<1>(T_SS) + std::get<2>(T_SS))/3.0;
				Math::Vector4 AABB = GetAABB(std::get<0>(T_SS), std::get<1>(T_SS), std::get<2>(T_SS));
				double minDimLength = std::min(AABB.y - AABB.x, AABB.w - AABB.z);
				ImVec2 textSize = ImGui::CalcTextSize(label.c_str());
				double textSizeMaxDim = double(std::max(textSize.x, textSize.y));
				if (minDimLength > textSizeMaxDim)
					MyGui::AddText(DrawList, Center_SS, IM_COL32(0, 0, 0, 255), label.c_str(), NULL, true, true);
			}
		}
	}
	else if (VisWidget::Instance().GuidanceOverlay_View == 2) {
		//Draw current tasking, future plans, and (optionally) planned missions
		std::vector<int> taskedDrones(m_SurveyRegionPartition.size(), -1); //Object n is the index of the drone that will fly component n
		std::vector<std::string> labels;
		for (int droneIndex = 0; droneIndex < (int) m_Sequences.size(); droneIndex++) {
			for (int missionNumber = 0; missionNumber < (int) m_Sequences[droneIndex].size(); missionNumber++) {
				int compIndex = (m_Sequences[droneIndex])[missionNumber];
				if ((compIndex >= 0) && (compIndex < (int) m_SurveyRegionPartition.size()))
					taskedDrones[compIndex] = droneIndex;
			}
		}

		std::vector<ImU32> colors;
		GetPartColorsByTaskedDrone(colors, opacity, taskedDrones);
		bool hideCompleteSubregions = VisWidget::Instance().GuidanceOverlay_HideCompleteSubregions;
		Draw_Partition(CursorPos_NM, DrawList, CursorInBounds, colors, labels, false, hideCompleteSubregions);

		//Draw visual indication of mission sequences
		Draw_MissionSequenceArrows(CursorPos_NM, DrawList, CursorInBounds, opacity);

		//Lightly draw planned missions on top of the partition
		if (VisWidget::Instance().GuidanceOverlay_ShowMissions) {
			for (size_t compIndex = 0U; compIndex < m_SurveyRegionPartition.size(); compIndex++) {
				bool drawComp = ((! hideCompleteSubregions) || (m_CompletedSubRegions.count((int) compIndex) == 0U));
				if (drawComp) {
					auto const & mission(m_Missions[compIndex]);
					for (size_t n = 0U; (n + 1U) < mission.Waypoints.size(); n++) {
						DroneInterface::Waypoint const & wp1(mission.Waypoints[n]);
						DroneInterface::Waypoint const & wp2(mission.Waypoints[n + 1U]);

						Eigen::Vector2d wp1_NM = LatLonToNM(Eigen::Vector2d(wp1.Latitude, wp1.Longitude));
						Eigen::Vector2d wp1_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(wp1_NM);

						Eigen::Vector2d wp2_NM = LatLonToNM(Eigen::Vector2d(wp2.Latitude, wp2.Longitude));
						Eigen::Vector2d wp2_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(wp2_NM);

						DrawList->AddLine(wp1_SS, wp2_SS, IM_COL32(255, 255, 255, 255.0f*opacity/100.0f), 2.0f);
					}
				}
			}
		}
	}
}

// Data Setters   ***************************************************************************************************************
void GuidanceOverlay::Reset() {
	std::scoped_lock lock(m_mutex);
	m_SurveyRegionPartition.clear();
	m_SurveyRegionPartitionTriangulation.clear();
	m_PartitionLabels.clear();
	m_Triangles.clear();
	m_TriangleLabels.clear();
	m_Missions.clear();
	m_Sequences.clear();
	m_CompletedSubRegions.clear();
}

void GuidanceOverlay::SetData_SurveyRegionPartition(std::Evector<PolygonCollection> const & Partition) {
	std::scoped_lock lock(m_mutex);
	m_SurveyRegionPartition = Partition;
	m_SurveyRegionPartitionTriangulation.clear();
	m_SurveyRegionPartitionTriangulation.reserve(Partition.size());
	for (auto const & comp : Partition) {
		std::Evector<Triangle> triangles;
		comp.Triangulate(triangles);
		m_SurveyRegionPartitionTriangulation.push_back(triangles);
	}
	m_PartitionLabels.clear();
}

void GuidanceOverlay::SetData_SurveyRegionPartition(std::Evector<PolygonCollection> const & Partition, std::vector<std::string> const & Labels) {
	std::scoped_lock lock(m_mutex);
	m_SurveyRegionPartition = Partition;
	m_SurveyRegionPartitionTriangulation.clear();
	m_SurveyRegionPartitionTriangulation.reserve(Partition.size());
	for (auto const & comp : Partition) {
		std::Evector<Triangle> triangles;
		comp.Triangulate(triangles);
		m_SurveyRegionPartitionTriangulation.push_back(triangles);
	}
	m_PartitionLabels = Labels;
}

void GuidanceOverlay::ClearData_SurveyRegionPartition(void) {
	std::scoped_lock lock(m_mutex);
	m_SurveyRegionPartition.clear();
	m_SurveyRegionPartitionTriangulation.clear();
	m_PartitionLabels.clear();
}

void GuidanceOverlay::SetData_Triangulation(std::Evector<Triangle> const & Triangles) {
	std::scoped_lock lock(m_mutex);
	m_Triangles = Triangles;
	m_TriangleLabels.clear();
}

void GuidanceOverlay::SetData_Triangulation(std::Evector<Triangle> const & Triangles, std::vector<std::string> const & Labels) {
	std::scoped_lock lock(m_mutex);
	m_Triangles = Triangles;
	m_TriangleLabels = Labels;
}

void GuidanceOverlay::ClearData_Triangulation(void) {
	std::scoped_lock lock(m_mutex);
	m_Triangles.clear();
	m_TriangleLabels.clear();
}

void GuidanceOverlay::SetData_PlannedMissions(std::vector<DroneInterface::WaypointMission> const & Missions) {
	std::scoped_lock lock(m_mutex);
	m_Missions = Missions;
}

void GuidanceOverlay::ClearData_PlannedMissions(void) {
	std::scoped_lock lock(m_mutex);
	m_Missions.clear();
}

void GuidanceOverlay::SetData_DroneMissionSequences(std::vector<std::vector<int>> const & Sequences) {
	std::scoped_lock lock(m_mutex);
	m_Sequences = Sequences;
}

void GuidanceOverlay::ClearData_DroneMissionSequences(void) {
	std::scoped_lock lock(m_mutex);
	m_Sequences.clear();
}

void GuidanceOverlay::SetData_CompletedSubRegions(std::vector<int> const & CompletedSubregionIndices) {
	std::scoped_lock lock(m_mutex);
	m_CompletedSubRegions.clear();
	m_CompletedSubRegions.insert(CompletedSubregionIndices.begin(), CompletedSubregionIndices.end());
}

void GuidanceOverlay::ClearData_CompletedSubRegions(void) {
	std::scoped_lock lock(m_mutex);
	m_CompletedSubRegions.clear();
}


// Introspection   **************************************************************************************************************
bool GuidanceOverlay::DoesSupportView_SurveyRegionPartition(void) {
	std::scoped_lock lock(m_mutex);
	return (! m_SurveyRegionPartition.empty());
}

bool GuidanceOverlay::DoesSupportView_Triangulation(void) {
	std::scoped_lock lock(m_mutex);
	return (! m_Triangles.empty());
}

bool GuidanceOverlay::DoesSupportView_DroneTasking(void) {
	std::scoped_lock lock(m_mutex);
	return (! m_Sequences.empty());
}

bool GuidanceOverlay::DoesSupportOption_DrawMissions(void) {
	std::scoped_lock lock(m_mutex);
	return (! m_Missions.empty());
}

bool GuidanceOverlay::DoesSupportOption_HideCompleteSubregions(void) {
	std::scoped_lock lock(m_mutex);
	return (! m_CompletedSubRegions.empty());
}





