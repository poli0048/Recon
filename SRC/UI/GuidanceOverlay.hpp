//The guidance overlay is used by the Guidance module to draw guidance-related data on the map widget
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.
#pragma once

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "../EigenAliases.h"
#include "../Polygon.hpp"
#include "../Modules/DJI-Drone-Interface/Drone.hpp"

class GuidanceOverlay {
	private:
		std::mutex m_mutex;
		std::Evector<PolygonCollection>      m_SurveyRegionPartition;
		std::Evector<std::Evector<Triangle>> m_SurveyRegionPartitionTriangulation;
		std::vector<std::string>             m_PartitionLabels;
		
		std::Evector<Triangle>   m_Triangles;
		std::vector<std::string> m_TriangleLabels;
		
		std::vector<DroneInterface::WaypointMission> m_Missions;
		std::vector<std::vector<int>> m_Sequences;

		//std::Evector<std::tuple<LineSegment, float, Eigen::Vector3f>> m_Lines;
		//std::Evector<std::tuple<Eigen::Vector2d, float, Eigen::Vector3f>> m_Circles;
		
		static ImU32 IndexToColor(size_t Index, size_t N, float Opacity);
		void Draw_Partition(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds,
		                    std::vector<ImU32> const & Colors, std::vector<std::string> const & Labels, bool CenteredLabels) const;
		
		//Get a point inside the given component that is roughly near the center of the first polygon in the component poly collection
		//The returned point is in Normalized Mercator. CompIndex must be a valid index - this is not checked.
		Eigen::Vector2d GetCentralPointForComponentOfPartition(size_t CompIndex) const;

		//Find the first point on the orthogonal projection of the first polygon component onto the line from A to B.
		//The returned point is in Normalized Mercator. CompIndex must be a valid index - this is not checked.
		Eigen::Vector2d FindFirstIntersectionOfLineAndComp(size_t CompIndex, Eigen::Vector2d const & A_NM, Eigen::Vector2d const & B_NM) const;
		
		void GetPartColorsByComponent(std::vector<ImU32> & Colors, float Opacity) const;
		void GetPartColorsByTaskedDrone(std::vector<ImU32> & Colors, float Opacity, std::vector<int> const & TaskedDrones) const;
		
		void Draw_MissionSequenceArrows(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds, float Opacity) const;

		//void DrawLines(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
		//void DrawCircles(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
		
	public:
		 GuidanceOverlay() = default;
		~GuidanceOverlay() = default;
		
		//Called in the draw loop for the map widget
		void Draw_Overlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
		
		//Data Setter Methods. Things the overlay can display:
		//1 - A partition of a survey region (optionally with labels)
		//2 - A collection of triangles (optionally with labels)
		//3 - Planned missions for each component in the given partition
		//4 - The planned mission sequences for a collection of drones (components flown, in order)

		//Important note:
		//All points in provided triangles or polygon objects must be in Normalized Mercator.
		
		void Reset();
		
		//A partition of a survey region is provided as a vector of polygon collections. Each element in the vector represents a component of the partition.
		void SetSurveyRegionPartition(std::Evector<PolygonCollection> const & Partition); //Set the partition of the survey region to draw
		void ClearSurveyRegionPartition(void);                                            //Clear/delete the partition of the survey region
		
		//Labels are optional; if you don't set them they won't be drawn. If specified, Labels[n] will be drawn on top of element n in the partition.
		//Label placement is relatively primitive and is based on the centroid of the vertex set of the boundary of the first component of the polygon
		//collection corresponding to each element in the partition (so holes are not taken into account). Consequently, if you have components that
		//are disconnected, odly shaped, or have holes, labels may not be drawn in ideal spots, but placement should be fine in most cases. Labels
		//are culled at zoom levels where they can't fit roughly in the bounding box of the corresponding element of the partition.
		void SetPartitionLabels(std::vector<std::string> const & Labels);
		void ClearPartitionLabels(void);
		
		//Triangles are independent from the partition, although the inteanded use is to show a triangulation of the survey region
		void SetTriangles(std::Evector<Triangle> const & Triangles); //Set the collection of triangles
		void ClearTriangles(void);                                   //Clear/delete the collection of triangles
		
		//Here also, labels are optional; if you don't set them they won't be drawn. If specified, Labels[n] will be drawn on top of triangle n.
		//Labels are culled at zoom levels where the can't fit roughly inside the interior of the corresponding triangle.
		void SetTriangleLabels(std::vector<std::string> const & Labels);
		void ClearTriangleLabels(void);
		
		//Provide the missions planned for each of the components of the partition of the survey region
		//Item n will be interpereted as the mission that covers component n of the provided partition.
		void SetMissions(std::vector<DroneInterface::WaypointMission> const & Missions);

		//Provide the currently planned mission sequences for a collection of drones
		void SetDroneMissionSequences(std::vector<std::vector<int>> const & Sequences);





		//Deprecated

		//Line segments are provided as a vector of tuples. Each tuple is of the form: <Line, thickness, color>
		//Line is a LineSegment object with coordinates in Normalized Mercator
		//thickness is the line thickness, in pixels
		//color is a 3-vector object: (R,G,B) where each component is in the range 0 to 1
		//void SetLineSegments(std::Evector<std::tuple<LineSegment, float, Eigen::Vector3f>> const & Lines);
		//void ClearLineSegments(void);
		
		//Circles are provided as a vector of tuples. Each tuple is of the form: <center, radius, color>
		//center is in Normalized Mercator
		//radius is in pixels
		//color is a 3-vector object: (R,G,B) where each component is in the range 0 to 1
		//Note that circles are drawn on top of lines (if specified)
		//void SetCircles(std::Evector<std::tuple<Eigen::Vector2d, float, Eigen::Vector3f>> const & Circles);
		//void ClearCircles();
};




