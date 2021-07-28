//The guidance overlay is used by the Guidance module to draw guidance-related data on the map widget
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "../EigenAliases.h"
#include "../Polygon.hpp"

class GuidanceOverlay {
	private:
		std::mutex m_mutex;
		std::Evector<PolygonCollection>      m_SurveyRegionPartition;
		std::Evector<std::Evector<Triangle>> m_SurveyRegionPartitionTriangulation;
		std::vector<std::string>             m_PartitionLabels;
		std::Evector<Triangle>   m_Triangles;
		std::vector<std::string> m_TriangleLabels;
		std::string m_GuidanceMessage_1;
		std::string m_GuidanceMessage_2;
		std::string m_GuidanceMessage_3;
		
		static ImU32 IndexToColor(size_t Index, size_t N, float Opacity);
		
	public:
		 GuidanceOverlay() = default;
		~GuidanceOverlay() = default;
		
		//Called in the draw loop for the map widget
		void Draw_Overlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
		void Draw_MessageBox(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
		
		//Data Setter Methods. Things the overlay can display:
		//1 - A partition of a survey region
		//2 - A collection of triangles
		//3 - A message box showing up to 3 lines of persistent messages
		//
		//A partition of a survey region is provided as a vector of polygon collection. Each element in the vector represents a component of the partition.
		//Item 2 (a collection of triangles) is independent from item 1 (a survey region partition) and the vis widget lets you toggle which is being displayed.
		//All points in provided triangles or polygon objects must be in Normalized Mercator.
		
		void Reset();
		
		void SetSurveyRegionPartition(std::Evector<PolygonCollection> const & Partition); //Set the partition of the survey region to draw
		void ClearSurveyRegionPartition(void);                                            //Clear/delete the partition of the survey region
		
		//Labels are optional; if you don't set them they won't be drawn. If specified, Labels[n] will be drawn on top of element n in the partition.
		//Label placement is relatively primitive and is based on the centroid of the vertex set of the boundary of the first component of the polygon
		//collection corresponding to each element in the partition (so holes are not taken into account). Consequently, if you have components that
		//are disconnected, odly shaped, or have holes, labels may not be drawn in ideal spots, but placement should be fine in most cases. Labels
		//are culled at zoom levels where they can't fit roughly in the bounding box of the corresponding element of the partition.
		void SetPartitionLabels(std::vector<std::string> const & Labels);
		void ClearPartitionLabels(void);
		
		void SetTriangles(std::Evector<Triangle> const & Triangles); //Set the collection of triangles
		void ClearTriangles(void);                                   //Clear/delete the collection of triangles
		
		//Here also, labels are optional; if you don't set them they won't be drawn. If specified, Labels[n] will be drawn on top of triangle n.
		//Labels are culled at zoom levels where the can't fit roughly inside the interior of the corresponding triangle.
		void SetTriangleLabels(std::vector<std::string> const & Labels);
		void ClearTriangleLabels(void);
		
		void SetGuidanceMessage1(std::string const & Message); //Display optional message in box on map (give empty string to disable)
		void SetGuidanceMessage2(std::string const & Message); //Display optional message in box on map (give empty string to disable)
		void SetGuidanceMessage3(std::string const & Message); //Display optional message in box on map (give empty string to disable)
};




