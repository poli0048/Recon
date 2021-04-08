//The Map Window displays satellite imagery and GIS data superimposed on top of the satellite imagery.
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved.
#pragma once

//System Includes
#include <chrono>
#include <tuple>

//External Includes
#include "../../../handycpp/Handy.hpp"
#include "../../../imgui/app/ImGuiApp.hpp"

//Eigen Includes
#include "../../../eigen/Eigen/Core"

//Project Includes
#include "../Colormaps.hpp"
#include "../Maps/DataTileTypes.hpp"
#include "../Journal.h"
#include "../EigenAliases.h"
#include "../TimeSeries.h"
#include "AvoidanceZonesTool.hpp"
#include "LandingZonesTool.hpp"
#include "MSATool.hpp"
#include "SurveyRegionTool.hpp"

class MapWidget {
		using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
		static constexpr std::chrono::milliseconds AnimationTimeMs = std::chrono::milliseconds(2000);
		static constexpr int32_t tileWidth = 256;
	
		//These two state variables control the location and zoom level of the map
		Eigen::Vector2d WindowULCorner_NormalizedMercator;
		double zoom; //When zoom is 0, 1 screen pixel equals 1 pixel on pyramid level 0. When zoom is 1, 1 screen pixel equals 1 pixel on pyramid level 1, etc.
		
		//Secondary state variables - kept up to date in the draw loop
		int32_t numSatTilesDrawn = 0;
		int32_t numDataTilesDrawn = 0;
		ImVec2 mousePosScreenSpace; //Position of mouse cursor - updated everytime we go through our draw loop.
		Eigen::Vector2d mousePosNM; //Position of mouse cursor - updated everytime we go through our draw loop.
		bool mouseInBounds;         //Whether the cursor is over the widget - updated each time through the draw loop.
		Eigen::Vector2d MapWidgetULCorner_ScreenSpace; //updated in draw loop - screen-space position of upper-left corner of widget
		Eigen::Vector2d MapWidgetDims; //Dimensions of the map widget - updated every time we go through our draw loop. Format: <Width, Height>
		
		//State for map drag logic
		bool            dragging = false;
		Eigen::Vector2d MousePos_NormalizedMercator_OnLastClick;
		
		//Tools
		AvoidanceZonesTool m_AvoidanceZonesTool;
		LandingZonesTool m_LandingZonesTool;
		MSATool m_MSATool;
		SurveyRegionsTool m_SurveyRegionsTool;
		
		//These fields are used for animated map navigation.
		std::tuple<TimePoint, TimePoint> AnimationTimeInterval; //Actual time interval over which the animation takes place
		timeSeries1D                     zoomProfile;           //zoom as a function of normalized time
		timeSeries2D                     panProfile;            //UL corner position of map widget (in Normalized Mercator) as a function of normalized time
		bool                             AnimationInProgress;   //Set when the animation is done and the End State has been effected.
		
		//Pan/Zoom adjustment and animation
		void SetMapPan(ImVec2 ScreenCoords, Eigen::Vector2d const & NMCoords); //Adjust map pan so the given NMCoords align with the given screen coords
		bool Draw_NavigationAnimation(void); //Update state based on current animation, if applicable. Returns true during animations. Called from Draw().
		void ComputeNavigationProfiles(std::tuple<Eigen::Vector2d, double> const & StartState, std::tuple<Eigen::Vector2d, double> const & EndState);
		
		//Coordinate conversion utilities (using current map widget state)
		Eigen::Vector2d ScreenCoordsToNormalizedMercator(ImVec2                  ScreenCords);
		ImVec2          NormalizedMercatorToScreenCoords(Eigen::Vector2d const & NMCoords   );
		
		//Functions for drawing visible satellite and data tiles and flight paths
		void Draw_SatTiles(int32_t MaxSatZoomLevel, Eigen::Vector4d const & ViewableAreaNM, ImDrawList * DrawList);
		void Draw_DataTiles(int32_t MaxDataZoomLevel, Eigen::Vector4d const & ViewableAreaNM, ImDrawList * DrawList);
		
		//Compute and/or set state variables so that the viewable map area approximately corresponds to the given area
		std::tuple<Eigen::Vector2d, double> GetMapLocationAndZoomForGivenLatLonBounds(Eigen::Vector2d const & LatBounds, Eigen::Vector2d const & LonBounds);
		void ZoomToLocation(Eigen::Vector2d const & LatBounds, Eigen::Vector2d const & LonBounds);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		friend class AvoidanceZonesTool;
		friend class LandingZonesTool;
		friend class MSATool;
		friend class SurveyRegionsTool;
		friend class VehiclesWidget;
		
		static MapWidget & Instance() { static MapWidget widget; return widget; }
		
		MapWidget();
		~MapWidget() = default;
		
		bool DrawDataTiles = true;
		bool DrawDataTooltip = false;
		
		void Draw(void);
		void Update(); //Should be called from main draw loop, regardless of whether widget is being drawn
		
		void GetCurrentLatLonBounds(Eigen::Vector2d & LatBounds, Eigen::Vector2d & LonBounds);
		void StartAnimation(Eigen::Vector2d const & LatBounds, Eigen::Vector2d const & LonBounds); //Should be called only from main draw thread
		void StartAnimation(double MinLat, double MaxLat, double MinLon, double MaxLon); //Should be called only from main draw thread
		int GetNumberOfSatTilesDrawn(void)  const { return int(numSatTilesDrawn);  }
		int GetNumberOfDataTilesDrawn(void) const { return int(numDataTilesDrawn); }
};



