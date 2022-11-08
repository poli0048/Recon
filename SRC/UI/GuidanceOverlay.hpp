//The guidance overlay is used by the Guidance module to draw guidance-related data on the map widget
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.
#pragma once

//System Includes
#include <vector>
#include <unordered_set>

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
		std::unordered_set<int> m_CompletedSubRegions;
		
		static ImU32 IndexToColor(size_t Index, size_t N, float Opacity);
		void Draw_Partition(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds,
		                    std::vector<ImU32> const & Colors, std::vector<std::string> const & Labels, bool CenteredLabels,
		                    bool HideComponentsMarketComplete) const;
		
		//Get a point inside the given component that is roughly near the center of the first polygon in the component poly collection
		//The returned point is in Normalized Mercator. CompIndex must be a valid index - this is not checked.
		Eigen::Vector2d GetCentralPointForComponentOfPartition(size_t CompIndex) const;

		//Find the first intersection of the line from A to B with the first polygon item in the given partition component
		Eigen::Vector2d FindLinesIntersectionWithBoundary(size_t CompIndex, Eigen::Vector2d const & A_NM, Eigen::Vector2d const & B_NM) const;
		
		void GetPartColorsByComponent(std::vector<ImU32> & Colors, float Opacity) const;
		void GetPartColorsByTaskedDrone(std::vector<ImU32> & Colors, float Opacity, std::vector<int> const & TaskedDrones) const;
		
		void Draw_MissionSequenceArrows(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds, float Opacity) const;
		
	public:
		 GuidanceOverlay() = default;
		~GuidanceOverlay() = default;
		
		//Called in the draw loop for the map widget
		void Draw_Overlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);

		//New design. Several views are supported:
		// 0: Partition of the survey region (with optional labels)
		// 1: Triangulation of the survey region (with optional labels)
		// 2: Progress - Planned tasking for drones (vector or subregions for each drone)
		//    - Option: Draw planned missions
		//    - Option: Hide complete subregions

		// Data Setters   *****************************************************************************************************
		// All provided geometry objects (polygons, triangles, etc.) should all use Normalized Mercator coordinates.
		// Each SetData() function replaces the corresponding data from any previous calls - you don't need to explicitly call
		// ClearData() in between calls to a data setter.
		void Reset(); //Clear all internal data

		void SetData_SurveyRegionPartition(std::Evector<PolygonCollection> const & Partition);
		void SetData_SurveyRegionPartition(std::Evector<PolygonCollection> const & Partition, std::vector<std::string> const & Labels);
		void ClearData_SurveyRegionPartition(void);

		void SetData_Triangulation(std::Evector<Triangle> const & Triangles);
		void SetData_Triangulation(std::Evector<Triangle> const & Triangles, std::vector<std::string> const & Labels);
		void ClearData_Triangulation(void);

		void SetData_PlannedMissions(std::vector<DroneInterface::WaypointMission> const & Missions);
		void ClearData_PlannedMissions(void);

		void SetData_DroneMissionSequences(std::vector<std::vector<int>> const & Sequences);
		void ClearData_DroneMissionSequences(void);

		void SetData_CompletedSubRegions(std::vector<int> const & CompletedSubregionIndices);
		void ClearData_CompletedSubRegions(void);

		// Introspection   ****************************************************************************************************
		// These can be used to detirmine whether enough data has been provided to draw various views and options. The overlay
		// is required to handle any combination of visualization settings chosen as best as it can (and without crashing), but
		// these can be used to only show views and options in the vis widget that can actually be drawn.
		bool DoesSupportView_SurveyRegionPartition(void);
		bool DoesSupportView_Triangulation(void);
		bool DoesSupportView_DroneTasking(void);
		bool DoesSupportOption_DrawMissions(void);
		bool DoesSupportOption_HideCompleteSubregions(void);
};




