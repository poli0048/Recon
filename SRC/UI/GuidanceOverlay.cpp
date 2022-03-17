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

void GuidanceOverlay::Draw_Partition(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds,
                                     std::vector<ImU32> const & Colors, std::vector<std::string> const & Labels) {
	float opacity = VisWidget::Instance().Opacity_GuidanceOverlay; //Opacity for overlay (0-100)
	if ((! m_SurveyRegionPartitionTriangulation.empty()) && (opacity > 0.0f)) {
		//Render triangles (the interiors of each component)
		for (size_t compIndex = 0U; compIndex < m_SurveyRegionPartitionTriangulation.size(); compIndex++) {
			std::Evector<Triangle> const & compTriangulation(m_SurveyRegionPartitionTriangulation[compIndex]);
			
			//Get a good color to render this component in
			//ImU32 compColor = IndexToColor(compIndex, m_SurveyRegionPartitionTriangulation.size(), opacity/100.0f);
			
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
		
		//After rendering the interiors of all components, render their boundaries
		float edgeThickness_pixels = VisWidget::Instance().SurveyRegionEdgeThickness; //Just use the edge thickness for survey regions for now
		std::Evector<std::Evector<Eigen::Vector2d>> compVertices_SS;
		compVertices_SS.reserve(m_SurveyRegionPartition.size());
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
						DrawList->AddLine(p1, p2, IM_COL32(0, 0, 0, 255.0f*opacity/100.0f), edgeThickness_pixels);
					}
				}
			}
			
			//Get the vertices of the boundary of the first component simple poly and convert to scrren space for label placement
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
			
			std::Evector<Eigen::Vector2d> const & Vertices_SS(compVertices_SS[compIndex]);
			std::string const & label(Labels[compIndex]);
			
			Eigen::Vector2d Centroid_SS;
			Math::Vector4 AABB;
			GetCentroidAndAABB(Vertices_SS, Centroid_SS, AABB);
			double minDimLength = std::min(AABB.y - AABB.x, AABB.w - AABB.z);
			ImVec2 textSize = ImGui::CalcTextSize(label.c_str());
			double textSizeMaxDim = double(std::max(textSize.x, textSize.y));
			if (minDimLength > textSizeMaxDim)
				MyGui::AddText(DrawList, Centroid_SS, IM_COL32(255, 255, 255, 255), label.c_str(), NULL, true, true);
		}
	}
}

//Get a point inside the given component that is roughly near the center of the first polygon in the component poly collection
//The returned point is in Normalized Mercator. CompIndex must be a valid index - this is not checked.
Eigen::Vector2d GuidanceOverlay::GetCentralPointForComponentOfPartition(size_t CompIndex) {
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
	if (VisWidget::Instance().GuidanceOverlay_Vis == 0) {
		//Draw the survey region partition
		std::vector<ImU32> colors(m_SurveyRegionPartition.size());
		for (size_t compIndex = 0U; compIndex < m_SurveyRegionPartition.size(); compIndex++)
			colors[compIndex] = IndexToColor(compIndex, m_SurveyRegionPartition.size(), opacity/100.0f);;

		Draw_Partition(CursorPos_NM, DrawList, CursorInBounds, colors, m_PartitionLabels);
	}
	else if (VisWidget::Instance().GuidanceOverlay_Vis == 1) {
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
	else if (VisWidget::Instance().GuidanceOverlay_Vis == 2) {
		//Draw current plans
		std::vector<int> taskedDrones(m_SurveyRegionPartition.size(), -1); //Object n is the index of the drone that will fly component n
		std::vector<std::string> labels(m_SurveyRegionPartition.size());
		for (int droneIndex = 0; droneIndex < (int) m_Sequences.size(); droneIndex++) {
			for (int missionNumber = 0; missionNumber < (int) m_Sequences[droneIndex].size(); missionNumber++) {
				int compIndex = (m_Sequences[droneIndex])[missionNumber];
				if ((compIndex >= 0) && (compIndex < (int) m_SurveyRegionPartition.size())) {
					taskedDrones[compIndex] = droneIndex;
					
					//labels[compIndex] = "Drone "s + std::to_string(droneIndex + 1) + "\n"s + "Mission "s + std::to_string(missionNumber);
					if (missionNumber == 0)
						labels[compIndex] = "Drone "s + std::to_string(droneIndex + 1);
				}
			}
		}

		std::vector<ImU32> colors(m_SurveyRegionPartition.size());
		for (size_t compIndex = 0U; compIndex < m_SurveyRegionPartition.size(); compIndex++) {
			if (taskedDrones[compIndex] < 0)
				colors[compIndex] = ImGui::GetColorU32(ImVec4(0, 0, 0, 0));
			else
				colors[compIndex] = IndexToColor((size_t) taskedDrones[compIndex], m_Sequences.size(), opacity/100.0f);
		}

		Draw_Partition(CursorPos_NM, DrawList, CursorInBounds, colors, labels);

		//Draw visual indication of mission sequences
		for (int droneIndex = 0; droneIndex < (int) m_Sequences.size(); droneIndex++) {
			for (int missionNumber = 0; missionNumber + 1 < (int) m_Sequences[droneIndex].size(); missionNumber++) {
				int comp1Index = (m_Sequences[droneIndex])[missionNumber];
				int comp2Index = (m_Sequences[droneIndex])[missionNumber + 1];
				if ((comp1Index >= 0) && (comp1Index < (int) m_SurveyRegionPartition.size()) &&
				    (comp2Index >= 0) && (comp2Index < (int) m_SurveyRegionPartition.size())) {
					Eigen::Vector2d comp1Center_NM = GetCentralPointForComponentOfPartition(size_t(comp1Index));
					Eigen::Vector2d comp2Center_NM = GetCentralPointForComponentOfPartition(size_t(comp2Index));
					Eigen::Vector2d comp1Center_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(comp1Center_NM);
					Eigen::Vector2d comp2Center_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(comp2Center_NM);

					Eigen::Vector2d A_SS = 0.8*comp1Center_SS + 0.2*comp2Center_SS;
					Eigen::Vector2d B_SS = 0.2*comp1Center_SS + 0.8*comp2Center_SS;

					ImVec4 colorVec4(1.0f, 1.0f, 1.0f, opacity/100.0f);
					MyGui::AddArrow(DrawList, A_SS, B_SS, ImGui::GetColorU32(colorVec4), 3.0f, 0.8f*ImGui::GetFontSize());
				}
			}
		}
	}
	else if (VisWidget::Instance().GuidanceOverlay_Vis == 3) {
		//Draw current plans & Missions
		std::vector<int> taskedDrones(m_SurveyRegionPartition.size(), -1); //Object n is the index of the drone that will fly component n
		std::vector<std::string> labels(m_SurveyRegionPartition.size());
		for (int droneIndex = 0; droneIndex < (int) m_Sequences.size(); droneIndex++) {
			for (int missionNumber = 0; missionNumber < (int) m_Sequences[droneIndex].size(); missionNumber++) {
				int compIndex = (m_Sequences[droneIndex])[missionNumber];
				if ((compIndex >= 0) && (compIndex < (int) m_SurveyRegionPartition.size())) {
					taskedDrones[compIndex] = droneIndex;
					labels[compIndex] = "Drone "s + std::to_string(droneIndex) + "\n"s + "Mission "s + std::to_string(missionNumber);
				}
			}
		}

		std::vector<ImU32> colors(m_SurveyRegionPartition.size());
		for (size_t compIndex = 0U; compIndex < m_SurveyRegionPartition.size(); compIndex++) {
			if (taskedDrones[compIndex] < 0)
				colors[compIndex] = ImGui::GetColorU32(ImVec4(0, 0, 0, 0));
			else
				colors[compIndex] = IndexToColor((size_t) taskedDrones[compIndex], m_Sequences.size(), opacity/100.0f);
		}

		Draw_Partition(CursorPos_NM, DrawList, CursorInBounds, colors, labels);
		//TODO: Lightly draw planned missions on top of the partition
	}
	//DrawLines(CursorPos_NM, DrawList, CursorInBounds);
	//DrawCircles(CursorPos_NM, DrawList, CursorInBounds);
}

/*void GuidanceOverlay::DrawLines(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
	float opacity = VisWidget::Instance().Opacity_GuidanceOverlay; //Opacity for overlay (0-100)
	for (auto const & item : m_Lines) {
		LineSegment     const & L(std::get<0>(item));
		float           const & thickness(std::get<1>(item));
		Eigen::Vector3f const & color(std::get<2>(item));
		
		Eigen::Vector2d endpoint1_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(L.m_endpoint1);
		Eigen::Vector2d endpoint2_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(L.m_endpoint2);
		ImVec4 colorVec4(color(0), color(1), color(2), opacity/100.0f);
		DrawList->AddLine(endpoint1_SS, endpoint2_SS, ImGui::GetColorU32(colorVec4), thickness);
	}
}

void GuidanceOverlay::DrawCircles(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
	float opacity = VisWidget::Instance().Opacity_GuidanceOverlay; //Opacity for overlay (0-100)
	for (auto const & item : m_Circles) {
		Eigen::Vector2d const & center(std::get<0>(item));
		float           const & radius(std::get<1>(item));
		Eigen::Vector3f const & color(std::get<2>(item));
		
		Eigen::Vector2d center_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(center);
		ImVec4 colorVec4(color(0), color(1), color(2), opacity/100.0f);
		DrawList->AddCircleFilled(center_SS, radius, ImGui::GetColorU32(colorVec4), 11);
	}
}*/

//Data Setter Methods
void GuidanceOverlay::Reset() {
	std::scoped_lock lock(m_mutex);
	m_SurveyRegionPartition.clear();
	m_SurveyRegionPartitionTriangulation.clear();
	m_Triangles.clear();
}

//Set the partition of the survey region to draw
void GuidanceOverlay::SetSurveyRegionPartition(std::Evector<PolygonCollection> const & Partition) {
	std::cerr << "Number of components in survey region partition: " << Partition.size() << "\r\n";
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

//Set labels to draw over components in the partition (optional)
void GuidanceOverlay::SetPartitionLabels(std::vector<std::string> const & Labels) {
	std::scoped_lock lock(m_mutex);
	m_PartitionLabels = Labels;
}

//Clear/delete labels for the partition
void GuidanceOverlay::ClearPartitionLabels(void) {
	std::scoped_lock lock(m_mutex);
	m_PartitionLabels.clear();
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

//Set labels to draw over triangles (optional)
void GuidanceOverlay::SetTriangleLabels(std::vector<std::string> const & Labels) {
	std::scoped_lock lock(m_mutex);
	m_TriangleLabels = Labels;
}

//Clear/delete labels for triangles
void GuidanceOverlay::ClearTriangleLabels(void) {
	std::scoped_lock lock(m_mutex);
	m_TriangleLabels.clear();
}

//Provide the missions planned for each of the components of the partition of the survey region
//Item n will be interpereted as the mission that covers component n of the provided partition.
void GuidanceOverlay::SetMissions(std::vector<DroneInterface::WaypointMission> const & Missions) {
	std::scoped_lock lock(m_mutex);
	m_Missions = Missions;
}

//Provide the currently planned mission sequences for a collection of drones
void GuidanceOverlay::SetDroneMissionSequences(std::vector<std::vector<int>> const & Sequences) {
	std::scoped_lock lock(m_mutex);
	m_Sequences = Sequences;
}

//Set line segments
/*void GuidanceOverlay::SetLineSegments(std::Evector<std::tuple<LineSegment, float, Eigen::Vector3f>> const & Lines) {
	std::scoped_lock lock(m_mutex);
	m_Lines = Lines;
}

//Clear/delete line segments
void GuidanceOverlay::ClearLineSegments(void) {
	std::scoped_lock lock(m_mutex);
	m_Lines.clear();
}

//Set circles
void GuidanceOverlay::SetCircles(std::Evector<std::tuple<Eigen::Vector2d, float, Eigen::Vector3f>> const & Circles) {
	std::scoped_lock lock(m_mutex);
	m_Circles = Circles;
}

//Clear/delete circles
void GuidanceOverlay::ClearCircles() {
	std::scoped_lock lock(m_mutex);
	m_Circles.clear();
}*/



