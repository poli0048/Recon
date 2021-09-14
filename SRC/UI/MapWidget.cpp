//The Map Window displays satellite imagery and GIS data superimposed on top of the satellite imagery.
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved.

//System Includes
#include <cmath>
#include <tuple>
#include <sstream>

//Project Includes
#include "MapWidget.hpp"
#include "VisWidget.hpp"
#include "../Maps/SatelliteCacheMaster.hpp"
#include "../Maps/DataTileProvider.hpp"
#include "../ProgOptions.hpp"
#include "../Maps/MapUtils.hpp"
#include "VehicleControlWidget.hpp"
#include "../Modules/GNSS-Receiver/GNSSReceiver.hpp"
#include "EmbeddedIcons.hpp"

#define PI 3.14159265358979

static inline Eigen::Vector2d roundVec(Eigen::Vector2d const & v) { return Eigen::Vector2d(std::round(v(0)), std::round(v(1))); }

static uint64_t factorial(uint64_t n) { return (n == 1U || n == 0U) ? 1U : factorial(n - 1U) * n; }
static uint64_t nChoosek(uint64_t n, uint64_t k) { return factorial(n)/(factorial(k)*factorial(n-k)); }
static double Smoothstep(double x, int order) {
	if (x <= 0.0) return 0.0;
	if (x >= 1.0) return 1.0;
	switch (order) {
		case 0: return x;
		case 1: return   -2.0*pow(x, 3) +    3.0*pow(x, 2);
		case 2: return    6.0*pow(x, 5) -   15.0*pow(x, 4) +   10.0*pow(x,3);
		case 3: return  -20.0*pow(x, 7) +   70.0*pow(x, 6) -   84.0*pow(x,5) +   35.0*pow(x,4);
		case 4: return   70.0*pow(x, 9) -  315.0*pow(x, 8) +  540.0*pow(x,7) -  420.0*pow(x,6) +  126.0*pow(x,5);
		case 5: return -252.0*pow(x,11) + 1386.0*pow(x,10) - 3080.0*pow(x,9) + 3465.0*pow(x,8) - 1980.0*pow(x,7) + 462.0*pow(x,6);
		default: double sum = 0.0;
		         for (int n = 0; n <= order; n++) {
		              double sign = 1.0 - 2.0*double(n % 2); //This is -1.0^n
		              sum += sign * nChoosek(order + n, n) * nChoosek(2.0*order + 1.0, order - n) * pow(x, order + n + 1);
		         }
		         return sum;
	}
}

constexpr int32_t MapWidget::tileWidth; //Constexprs need to be defines in a translation unit unless using C++17

//Coordinate conversion utilities for screenSpace <--> Normalized Mercator (using current map widget state)
Eigen::Vector2d MapWidget::ScreenCoordsToNormalizedMercator(Eigen::Vector2d const & ScreenCords) {
	Eigen::Vector2d WidgetCoords = ScreenCords - MapWidgetULCorner_ScreenSpace;
	return WidgetCoordsToNormalizedMercator(WidgetCoords, WindowULCorner_NormalizedMercator, zoom, tileWidth);
}

Eigen::Vector2d MapWidget::NormalizedMercatorToScreenCoords(Eigen::Vector2d const & NMCoords) {
	Eigen::Vector2d WidgetCoords = NormalizedMercatorToWidgetCoords(NMCoords, WindowULCorner_NormalizedMercator, zoom, tileWidth);
	Eigen::Vector2d ScreenCoords = WidgetCoords + MapWidgetULCorner_ScreenSpace;
	return ScreenCoords;
}

//Adjust map pan so the given NMCoords align with the given screen coords
void MapWidget::SetMapPan(Eigen::Vector2d const & ScreenCoords, Eigen::Vector2d const & NMCoords) {
	Eigen::Vector2d WidgetCoords = ScreenCoords - MapWidgetULCorner_ScreenSpace;
	
	//Solve for WindowULCorner_NormalizedMercator so that:
	//WidgetCoordsToNormalizedMercator(WidgetCoords, WindowULCorner_NormalizedMercator, zoom, tileWidth) = NMCoords
	double screenPixelLengthNM = 2.0 / (pow(2.0, zoom) * ((double) tileWidth));
	WindowULCorner_NormalizedMercator = NMCoords - Eigen::Vector2d(WidgetCoords(0)*screenPixelLengthNM, -1.0*WidgetCoords(1)*screenPixelLengthNM);
}

MapWidget::MapWidget()
	: WindowULCorner_NormalizedMercator(-0.72, 0.34),
	  zoom(4),
	  mousePosNM(std::nan(""), std::nan("")),
	  MapWidgetULCorner_ScreenSpace(0.0, 0.0),
	  MapWidgetDims(0.0, 0.0),
	  MousePos_NormalizedMercator_OnLastClick(0.0, 0.0),
	  AnimationTimeInterval(std::make_tuple(std::chrono::steady_clock::now(), std::chrono::steady_clock::now())),
	  AnimationInProgress(false)
{
	m_IconTexture_Laptop = ImGuiApp::Instance().CreateImageRGBA8888(&Icon_Laptop_Light_96x75[0], 96, 75);
}

MapWidget::~MapWidget() {
	ImGuiApp::Instance().DeleteImage(m_IconTexture_Laptop);
}

//Set state variables so that the viewable map area approximately corresponds to the given area
void MapWidget::ZoomToLocation(Eigen::Vector2d const & LatBounds, Eigen::Vector2d const & LonBounds) {
	if (std::isfinite(LatBounds(0)) && std::isfinite(LatBounds(1)) && std::isfinite(LonBounds(0)) && std::isfinite(LonBounds(1))) {
		std::tuple<Eigen::Vector2d, double> newState = GetMapLocationAndZoomForGivenLatLonBounds(LatBounds, LonBounds);
		this->WindowULCorner_NormalizedMercator = std::get<0>(newState);
		this->zoom = std::get<1>(newState);
	}
}

//Return <ULCorner_NM, zoom> state for map for ideal viewing of the given Lat and Lon 
std::tuple<Eigen::Vector2d, double> MapWidget::GetMapLocationAndZoomForGivenLatLonBounds(Eigen::Vector2d const & LatBounds, Eigen::Vector2d const & LonBounds) {
	//Convert Lat and Lon bounds to Normalized Mercator
	Eigen::Vector2d ULCorner_NM = LatLonToNM(Eigen::Vector2d(LatBounds(1), LonBounds(0)));
	Eigen::Vector2d LRCorner_NM = LatLonToNM(Eigen::Vector2d(LatBounds(0), LonBounds(1)));
	if (LonBounds(1) < LonBounds(0)) {
		//The region we have been asked to cover spans the longitude discontinuity. We will adjust the bound
		//so that we only try to zoom in on the portion west of the discontinuity.
		LRCorner_NM(0) = 1.0;
	}
	
	//Compute good zoom level so the full region fits inside the map widget
	double zoomBasedOnWidth  = log2(2.0 * MapWidgetDims(0) / (double(tileWidth) * (LRCorner_NM(0) - ULCorner_NM(0))));
	double zoomBasedOnHeight = log2(2.0 * MapWidgetDims(1) / (double(tileWidth) * (ULCorner_NM(1) - LRCorner_NM(1))));
	double newZoom = std::min(zoomBasedOnWidth, zoomBasedOnHeight);
	if (std::isnan(newZoom) || std::isinf(newZoom) || (newZoom > 21.0))
		newZoom = 21.0;
	
	//Now we adjust the map position so the center of the map widget corresponds to the center of the given region
	Eigen::Vector2d Center_NM = 0.5*ULCorner_NM + 0.5*LRCorner_NM;
	Eigen::Vector2d Center_Widget = 0.5*MapWidgetDims;
	
	double screenPixelLengthNM = 2.0 / (pow(2.0, newZoom) * ((double) tileWidth));
	Eigen::Vector2d newULCorner_NM;
	newULCorner_NM(0) = Center_NM(0) - Center_Widget(0)*screenPixelLengthNM;
	newULCorner_NM(1) = Center_NM(1) + Center_Widget(1)*screenPixelLengthNM;
	return std::make_tuple(newULCorner_NM, newZoom);
}

void MapWidget::GetCurrentLatLonBounds(Eigen::Vector2d & LatBounds, Eigen::Vector2d & LonBounds) {
	Eigen::Vector2d MapWidgetLRCorner_ScreenSpace = MapWidgetULCorner_ScreenSpace + MapWidgetDims;
	Eigen::Vector2d WindowLRCorner_NormalizedMercator = ScreenCoordsToNormalizedMercator(MapWidgetLRCorner_ScreenSpace);
	
	Eigen::Vector2d ULCornerLatLon = NMToLatLon(WindowULCorner_NormalizedMercator);
	Eigen::Vector2d LRCornerLatLon = NMToLatLon(WindowLRCorner_NormalizedMercator);
	
	LatBounds << LRCornerLatLon(0), ULCornerLatLon(0);
	LonBounds << ULCornerLatLon(1), LRCornerLatLon(1);
	
	//std::cerr << "Lat: [" << LatBounds(0)*180.0/PI << ", " << LatBounds(1)*180.0/PI << "] degrees\r\n";
	//std::cerr << "Lon: [" << LonBounds(0)*180.0/PI << ", " << LonBounds(1)*180.0/PI << "] degrees\r\n";
}

void MapWidget::StartAnimation(Eigen::Vector2d const & LatBounds, Eigen::Vector2d const & LonBounds) {
	if (std::isfinite(LatBounds(0)) && std::isfinite(LatBounds(1)) && std::isfinite(LonBounds(0)) && std::isfinite(LonBounds(1))) {
		TimePoint now = std::chrono::steady_clock::now();
		AnimationTimeInterval = std::make_tuple(now, now + AnimationTimeMs);
		std::tuple<Eigen::Vector2d, double> StartState = std::make_tuple(WindowULCorner_NormalizedMercator, zoom);
		std::tuple<Eigen::Vector2d, double> EndState   = GetMapLocationAndZoomForGivenLatLonBounds(LatBounds, LonBounds);
		ComputeNavigationProfiles(StartState, EndState);
		AnimationInProgress = true;
	}
}

void MapWidget::StartAnimation(double MinLat, double MaxLat, double MinLon, double MaxLon) {
	Eigen::Vector2d LatBounds(MinLat, MaxLat);
	Eigen::Vector2d LonBounds(MinLon, MaxLon);
	StartAnimation(LatBounds, LonBounds);
}

//Update state based on current animation, if applicable. Returns true during animations. Called from Draw()
bool MapWidget::Draw_NavigationAnimation(void) {
	if (AnimationInProgress) {
		TimePoint now = std::chrono::steady_clock::now();
		if ((now >= std::get<0>(AnimationTimeInterval)) && (now <= std::get<1>(AnimationTimeInterval))) {
			//The animation is in progress
			std::chrono::duration<float> d1_duration = now                                - std::get<0>(AnimationTimeInterval);
			std::chrono::duration<float> d2_duration = std::get<1>(AnimationTimeInterval) - std::get<0>(AnimationTimeInterval);
			float d1 = d1_duration.count();
			float d2 = d2_duration.count();
			if (d2 > 0.001) {
				//Compute "normalized time". This goes from 0 to 1 linearly during the animation
				double t = std::max(std::min((double) (d1/d2), 1.0), 0.0);
				this->zoom = zoomProfile(t);
				this->WindowULCorner_NormalizedMercator = panProfile(t);
			}
			return true;
		}
		else {
			//Perform final update and end animation
			this->zoom = zoomProfile.back();
			this->WindowULCorner_NormalizedMercator = panProfile.back();
			AnimationInProgress = false;
			return false;
		}
	}
	return false;
}

void MapWidget::ComputeNavigationProfiles(std::tuple<Eigen::Vector2d, double> const & StartState, std::tuple<Eigen::Vector2d, double> const & EndState) {
	zoomProfile.clear();
	panProfile.clear();
	
	//Lets figure out roughly how far back we need to zoom in order to see both locations
	double mapWidthPixels  = std::max(MapWidgetDims(0), 1.0);
	double mapHeightPixels = std::max(MapWidgetDims(1), 1.0);
	double horizontalSeparation_NM = std::max(std::abs(std::get<0>(StartState)(0) - std::get<0>(EndState)(0)), 7.4506e-09);
	double verticalSeparation_NM   = std::max(std::abs(std::get<0>(StartState)(1) - std::get<0>(EndState)(1)), 7.4506e-09);
	
	double minZoomForWidth  = log2(mapWidthPixels  / (double(tileWidth) * horizontalSeparation_NM)) + 1.0;
	double minZoomForHeight = log2(mapHeightPixels / (double(tileWidth) * verticalSeparation_NM  )) + 1.0;
	double minZoom = std::min(minZoomForWidth, minZoomForHeight);
	minZoom = std::min(minZoom, std::get<1>(StartState));
	minZoom = std::min(minZoom, std::get<1>(EndState));

	//Now lets detirmine the zoom profile. We need to find the transition time. This is when we switch from zooming out to zooming in
	double zoomTransition_t;
	double a1 = std::abs(std::get<1>(StartState) - minZoom);
	double a2 = std::abs(std::get<1>(EndState)   - minZoom);
	if (a1 + a2 < 0.1)
		zoomTransition_t = 0.5;
	else
		zoomTransition_t = a1 / (a1 + a2);
	
	//Make sure the zoom transition point is not right at 0 or 1
	zoomTransition_t = std::min(zoomTransition_t, 0.99);
	zoomTransition_t = std::max(zoomTransition_t, 0.01);
	
	double deltaT = 0.005; //Time interval in normalized time units
	
	std::Evector<double> zoomValues;
	std::vector<double> unscaledPanSpeeds;
	double sumOfUnscaledPanSpeeds = 0.0;
	zoomValues.reserve((size_t) (1.0/deltaT) + 5U);
	unscaledPanSpeeds.reserve((size_t) (1.0/deltaT) + 5U);
	for (double t = 0.0; t <= 1.0; t += deltaT) {
		if (t < zoomTransition_t) {
			double f = 0.5 - 0.5*cos(t/zoomTransition_t*PI);
			zoomValues.push_back((1.0 - f)*std::get<1>(StartState) + f*minZoom);
		}
		else {
			double f = 0.5 - 0.5*cos((t - zoomTransition_t)/(1.0 - zoomTransition_t)*PI);
			zoomValues.push_back((1.0 - f)*minZoom + f*std::get<1>(EndState));
		}
		
		double phaseInTime = 0.2;
		double phaseOutTime = 0.2;
		double panSpeed = 2.0 / (pow(2.0, zoomValues.back()) * double(tileWidth));
		if (t < phaseInTime)
			panSpeed *= Smoothstep(t/phaseInTime, 3);
		if (t > 1.0 - phaseOutTime)
			panSpeed *= Smoothstep((1.0 - t)/phaseOutTime, 3);
		
		unscaledPanSpeeds.push_back(panSpeed);
		sumOfUnscaledPanSpeeds += unscaledPanSpeeds.back();
	}
	
	//Finalize the zoom prifile - use uniform series for faster sampling
	zoomProfile = timeSeries1D(0.0, deltaT, zoomValues);
	
	//Now we evaluate the pan profile
	Eigen::Vector2d centerNM_Start = WidgetCoordsToNormalizedMercator(0.5*MapWidgetDims, std::get<0>(StartState), std::get<1>(StartState), tileWidth);
	Eigen::Vector2d centerNM_End   = WidgetCoordsToNormalizedMercator(0.5*MapWidgetDims, std::get<0>(EndState),   std::get<1>(EndState),   tileWidth);
	
	std::Evector<Eigen::Vector2d> panValues(unscaledPanSpeeds.size());
	double integral = 0.0;
	for (size_t n = 0U; n < unscaledPanSpeeds.size(); n++) {
		integral += unscaledPanSpeeds[n];
		double panProgress = integral / sumOfUnscaledPanSpeeds;
		
		//Compute the NM position of the center of the map window for this time
		Eigen::Vector2d centerNM_Now = (1.0 - panProgress)*centerNM_Start + panProgress*centerNM_End;
		
		//Now evaluate the NM position of the upper-left corner of the map window and save to panValues
		double screenPixelLengthNM = 2.0 / (pow(2.0, zoomProfile(n * deltaT)) * ((double) tileWidth));
		panValues[n](0) = centerNM_Now(0) - 0.5*MapWidgetDims(0)*screenPixelLengthNM;
		panValues[n](1) = centerNM_Now(1) + 0.5*MapWidgetDims(1)*screenPixelLengthNM;
	}
	
	//Finalize the pan profile - use uniform series for faster sampling
	panProfile = timeSeries2D(0.0, deltaT, panValues);
}

void MapWidget::Draw_SatTiles(int32_t MaxSatZoomLevel, Eigen::Vector4d const & ViewableAreaNM, ImDrawList * DrawList) {
	for (int32_t level = std::max(0, MaxSatZoomLevel - 10); level <= MaxSatZoomLevel; level++) {
		//fprintf(stderr,"Populating level: %d\r\n", level);
		std::tuple<int32_t, int32_t> ULTileCoords = getCoordsOfTileContainingPoint(Eigen::Vector2d(ViewableAreaNM(0), ViewableAreaNM(3)), level);
		std::tuple<int32_t, int32_t> LRTileCoords = getCoordsOfTileContainingPoint(Eigen::Vector2d(ViewableAreaNM(1), ViewableAreaNM(2)), level);
		
		for (int32_t tileX = std::get<0>(ULTileCoords); tileX <= std::get<0>(LRTileCoords); tileX++) {
			for (int32_t tileY = std::get<1>(ULTileCoords); tileY <= std::get<1>(LRTileCoords); tileY++) {
				
				ImTextureID satTile_tid = Maps::SatelliteCacheMaster::Instance()->TryGetTouchReq(Maps::Tile(tileX, tileY, level));
				if (satTile_tid) {
					Eigen::Vector2d UL_NM = GetNMCoordsOfULCornerOfTile(tileX, tileY, level);
					Eigen::Vector2d LR_NM = GetNMCoordsOfLRCornerOfTile(tileX, tileY, level);
				
					Eigen::Vector2d UL_Widget = NormalizedMercatorToWidgetCoords(UL_NM, WindowULCorner_NormalizedMercator, zoom, tileWidth);
					Eigen::Vector2d LR_Widget = NormalizedMercatorToWidgetCoords(LR_NM, WindowULCorner_NormalizedMercator, zoom, tileWidth);
				
					Eigen::Vector2d UL_Screen = roundVec(UL_Widget + MapWidgetULCorner_ScreenSpace);
					Eigen::Vector2d LR_Screen = roundVec(LR_Widget + MapWidgetULCorner_ScreenSpace);
				
					ImVec2 ul(UL_Screen(0), UL_Screen(1));
					ImVec2 lr(LR_Screen(0), LR_Screen(1));
				
					numSatTilesDrawn++;
					DrawList->AddImage(satTile_tid, ul, lr);
				}
			}
		}
	}
}

//Returns the number of data sets that data tiles were drawn for (this doesn't include data sets we drew a marker for due to zoom level)
void MapWidget::Draw_DataTiles(int32_t RecDataZoomLevel, Eigen::Vector4d const & ViewableAreaNM, ImDrawList * DrawList) {
	//Draw visible data tiles. This is done in an apparently strange way. We fetch all visible tiles on all zoom levels at or below the screen zoom
	//level. We start by fetching resources on the screen zoom level and drop the zoom level from there so we effectively fetch the highest-res tiles
	//first. We don't draw them in this pass because we need to draw only resources from the highest 1 or 2 available zoom levels (for speed and to
	//avoid blurring edges), and we need to draw them lowest-zoom-level first. Touching visible tiles like this on all zoom levels keeps the entire
	//visible stack cached in memory so we don't have screen blanking when we zoom out or pan.
	
	//Build VizualizationTileKey based on viz parameters (still need to set tile field)
	VisWidget & visWidget(VisWidget::Instance());
	Maps::VizualizationTileKey vizKey;
	vizKey.Opacity_MSA = (uint8_t) std::max(std::min(std::round(visWidget.Opacity_MSA * 2.55f), 255.0f), 0.0f);
	vizKey.Opacity_AvoidanceZones = (uint8_t) std::max(std::min(std::round(visWidget.Opacity_AvoidanceZones * 2.55f), 255.0f), 0.0f);
	vizKey.Opacity_SafeLandingZones = (uint8_t) std::max(std::min(std::round(visWidget.Opacity_SafeLandingZones * 2.55f), 255.0f), 0.0f);
	if (! visWidget.LayerVisible_MSA)
		vizKey.Opacity_MSA = 0_u8;
	if (! visWidget.LayerVisible_AvoidanceZones)
		vizKey.Opacity_AvoidanceZones = 0_u8;
	if (! visWidget.LayerVisible_SafeLandingZones)
		vizKey.Opacity_SafeLandingZones = 0_u8;
	vizKey.MSA_colormap = Colormap::BlueToRed; //Hard-coded since we don't expose this as an option right now
	vizKey.MSA_ColormapMinVal = visWidget.MSA_CmapMinVal;
	vizKey.MSA_ColormapMaxVal = visWidget.MSA_CmapMaxVal;
	vizKey.MSA_NoFlyDrawMode  = visWidget.MSA_NoFlyDrawMode;
	
	//Pass 1 - Touch-Load the viz tiles
	std::unordered_map<Maps::Tile, ImTextureID> loadedDataTiles;
	int32_t dataTileHighestZoomLevel = -1;
	for (int32_t level = RecDataZoomLevel; level >= std::max(RecDataZoomLevel - 1, int32_t(0)); level--) {
		std::tuple<int32_t, int32_t> ULTileCoords = getCoordsOfTileContainingPoint(Eigen::Vector2d(ViewableAreaNM(0), ViewableAreaNM(3)), level);
		std::tuple<int32_t, int32_t> LRTileCoords = getCoordsOfTileContainingPoint(Eigen::Vector2d(ViewableAreaNM(1), ViewableAreaNM(2)), level);

		for (int32_t tileX = std::get<0>(ULTileCoords); tileX <= std::get<0>(LRTileCoords); tileX++) {
			for (int32_t tileY = std::get<1>(ULTileCoords); tileY <= std::get<1>(LRTileCoords); tileY++) {
				vizKey.tile = Maps::Tile(tileX, tileY, level);
				ImTextureID dataTile_tid = Maps::DataTileProvider::Instance()->TryGetLoadUpdate_VizTile(vizKey);
				if (dataTile_tid) {
					loadedDataTiles[vizKey.tile] = dataTile_tid;
					
					//Once we set the highest zoom level we never need to touch it again since tiles are traversed
					//in descending order of zoom level
					if (dataTileHighestZoomLevel == -1)
						dataTileHighestZoomLevel = level;
				}
			}
		}
	}
	
	//Pass 2 - Draw tiles: Only draw tiles from the highest zoom level that we have tiles from - we don't do multiple levels because data tiles can have
	//non-trivial transparancy values. If we draw multiple layers they will blend and look wierd.
	if (dataTileHighestZoomLevel >= 0) {
		std::tuple<int32_t, int32_t> ULTileCoords = getCoordsOfTileContainingPoint(Eigen::Vector2d(ViewableAreaNM(0), ViewableAreaNM(3)), dataTileHighestZoomLevel);
		std::tuple<int32_t, int32_t> LRTileCoords = getCoordsOfTileContainingPoint(Eigen::Vector2d(ViewableAreaNM(1), ViewableAreaNM(2)), dataTileHighestZoomLevel);
		
		for (int32_t tileX = std::get<0>(ULTileCoords); tileX <= std::get<0>(LRTileCoords); tileX++) {
			for (int32_t tileY = std::get<1>(ULTileCoords); tileY <= std::get<1>(LRTileCoords); tileY++) {
				Eigen::Vector2d UL_NM = GetNMCoordsOfULCornerOfTile(tileX, tileY, dataTileHighestZoomLevel);
				Eigen::Vector2d LR_NM = GetNMCoordsOfLRCornerOfTile(tileX, tileY, dataTileHighestZoomLevel);
		
				Eigen::Vector2d UL_Widget = NormalizedMercatorToWidgetCoords(UL_NM, WindowULCorner_NormalizedMercator, zoom, tileWidth);
				Eigen::Vector2d LR_Widget = NormalizedMercatorToWidgetCoords(LR_NM, WindowULCorner_NormalizedMercator, zoom, tileWidth);
		
				Eigen::Vector2d UL_Screen = roundVec(UL_Widget + MapWidgetULCorner_ScreenSpace);
				Eigen::Vector2d LR_Screen = roundVec(LR_Widget + MapWidgetULCorner_ScreenSpace);
		
				ImVec2 ul(UL_Screen(0), UL_Screen(1));
				ImVec2 lr(LR_Screen(0), LR_Screen(1));
				
				//Draw tile
				Maps::Tile tile(tileX, tileY, dataTileHighestZoomLevel);
				if (loadedDataTiles.count(tile) > 0U) {
					numDataTilesDrawn++;
					DrawList->AddImage(loadedDataTiles.at(tile), ul, lr);
				}
			}
		}
	}
}

void MapWidget::Draw(void) {
	//Draw header and toolbar
	ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetStyle().ItemInnerSpacing.x);
	if (m_AvoidanceZonesTool.Draw_Button()) {
		m_LandingZonesTool.toolActive = false;
		m_MSATool.toolActive = false;
		m_SurveyRegionsTool.toolActive = false;
	}
	ImGui::SameLine(0.0f, 10.0f);
	if (m_LandingZonesTool.Draw_Button()) {
		m_AvoidanceZonesTool.toolActive = false;
		m_MSATool.toolActive = false;
		m_SurveyRegionsTool.toolActive = false;
	}
	ImGui::SameLine(0.0f, 10.0f);
	if (m_MSATool.Draw_Button()) {
		m_AvoidanceZonesTool.toolActive = false;
		m_LandingZonesTool.toolActive = false;
		m_SurveyRegionsTool.toolActive = false;
	}
	ImGui::SameLine(0.0f, 10.0f);
	if (m_SurveyRegionsTool.Draw_Button()) {
		m_AvoidanceZonesTool.toolActive = false;
		m_LandingZonesTool.toolActive = false;
		m_MSATool.toolActive = false;
	}
	
	//If the mouse is in bounds of the widget, print the cursor position
	if (mouseInBounds) {
		Eigen::Vector2d mousePosLatLon = NMToLatLon(mousePosNM);
		ImVec2 LatStringSize = ImGui::CalcTextSize("Lat: -100.000000\u00B0   ");
		ImVec2 LonStringSize = ImGui::CalcTextSize("Lon: -100.000000\u00B0 ");
		
		ImGui::SameLine(ImGui::GetContentRegionMax().x - LonStringSize.x - LatStringSize.x);
		ImGui::Text("Lat: % .6f\u00B0", mousePosLatLon(0)*180.0/PI);
		
		ImGui::SameLine(ImGui::GetContentRegionMax().x - LonStringSize.x);
		ImGui::Text("Lon: % .6f\u00B0", mousePosLatLon(1)*180.0/PI);
	}
	
	//Update the position of the upper-left corner of the map widget, the map widget dims (in pixels)
	MapWidgetULCorner_ScreenSpace << double(ImGui::GetCursorScreenPos().x), double(ImGui::GetCursorScreenPos().y);
	MapWidgetDims << ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y;
	
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(1,1));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0,0));
	ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(60.0f / 256.0f, 60.0f / 256.0f, 70.0f / 256.0f, 200.0f / 256.0f));
	ImGui::BeginChild("panning_region", ImVec2(0,0), true, ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoMove); //All about mouse events
	
	ImDrawList * draw_list = ImGui::GetWindowDrawList(); //Current draw list being assembled for this frame
	
	//Get the region of the earth that is currently visible in the map widget (in Normalized Mercator) and get the current ideal tile zoom level
	Eigen::Vector4d ViewableAreaNM = GetViewableArea_NormalizedMercator(WindowULCorner_NormalizedMercator, MapWidgetDims, zoom, tileWidth);
	int32_t MaxTileZoomLevel = std::max((int32_t) ceil(zoom + log2(ProgOptions::Instance()->MapDPIPercentage / 100.0)), 0);
	
	//Reset tile counters
	numSatTilesDrawn = 0;
	numDataTilesDrawn = 0;
	
	//Draw the satellite imagery - we must draw all satellite imagery before drawing any data tiles to make sure our data is on top of all sat imagery
	int32_t maxSatTileZoomLevel = std::min(MaxTileZoomLevel, 20);
	Draw_SatTiles(maxSatTileZoomLevel, ViewableAreaNM, draw_list);
	
	//Draw data tiles
	VisWidget & visWidget(VisWidget::Instance());
	bool someLayerVisible = visWidget.LayerVisible_MSA || visWidget.LayerVisible_AvoidanceZones || visWidget.LayerVisible_SafeLandingZones;
	if (DrawDataTiles && someLayerVisible && (! AnimationInProgress)) {
		int32_t RecDataTileZoomLevel = std::min(MaxTileZoomLevel, Maps::DataTileProvider::TileEditZoomLevel);
		if (RecDataTileZoomLevel >= Maps::DataTileProvider::DataTileMinZoomLevel)
			Draw_DataTiles(RecDataTileZoomLevel, ViewableAreaNM, draw_list);
	}
	
	//std::cerr << "Sat Tiles Drawn: " << numSatTilesDrawn << "\r\n";
	//std::cerr << "Data Tiles Drawn: " << numDataTilesDrawn << "\r\n";
	
	//Update the mouse position and in-bounds state
	mousePosScreenSpace = ImGui::GetMousePos();
	mousePosNM = ScreenCoordsToNormalizedMercator(mousePosScreenSpace);
	
	//Draw passes for tools
	if (MaxTileZoomLevel >= 12) {
		if ((! dragging) && (! AnimationInProgress)) {
			m_AvoidanceZonesTool.Draw_Overlay(mousePosNM, draw_list, mouseInBounds);
			m_LandingZonesTool.Draw_Overlay(mousePosNM, draw_list, mouseInBounds);
			m_MSATool.Draw_Overlay(mousePosNM, draw_list, mouseInBounds);
		}
		m_SurveyRegionsTool.Draw_Overlay(mousePosScreenSpace, mousePosNM, draw_list, mouseInBounds);
		m_shadowMapOverlay.Draw_Overlay(mousePosNM, draw_list, mouseInBounds);
		m_guidanceOverlay.Draw_Overlay(mousePosNM, draw_list, mouseInBounds);
	}
	
	//Draw pass for vehicle widget
	bool processMouseInputs = true;
	if (MaxTileZoomLevel >= 12)
		processMouseInputs = VehicleControlWidget::Instance().DrawMapOverlay(mousePosScreenSpace, mousePosNM, draw_list, mouseInBounds);
	
	//Draw GCS location (if available)
	Eigen::Vector2d GCS_Pos_NM;
	GNSSReceiver::GNSSManager::TimePoint GCS_Pos_Timestamp;
	if (GNSSReceiver::GNSSManager::Instance().GetPosition_NM(GCS_Pos_NM, GCS_Pos_Timestamp)) {
		ImExt::Font fnt(Fonts::NormalBold);
		Eigen::Vector2d GCS_Pos_SS = NormalizedMercatorToScreenCoords(GCS_Pos_NM);
		float IconWidth_pixels = ProgOptions::Instance()->DroneIconScale*96.0f;
		Eigen::Vector2d GCS_SS_Min = GCS_Pos_SS - 0.5*Eigen::Vector2d(IconWidth_pixels, 75.0/96.0*IconWidth_pixels);
		Eigen::Vector2d GCS_SS_Max = GCS_Pos_SS + 0.5*Eigen::Vector2d(IconWidth_pixels, 75.0/96.0*IconWidth_pixels);
		draw_list->AddImage(m_IconTexture_Laptop, GCS_SS_Min, GCS_SS_Max);
		Eigen::Vector2d TextPos_SS = GCS_Pos_SS + Eigen::Vector2d(0.0, 0.5*75.0/96.0*IconWidth_pixels + ImGui::GetStyle().ItemSpacing.y);
		MyGui::AddText(draw_list, TextPos_SS, IM_COL32_WHITE, "GCS", NULL, true, false);
	}
	
	//TODO: Break out the message box from the m_SurveyRegionsTool like we did with the guidance overlay. Then call the method to draw the message box down here
	//so it shows up on top of things instead of potentially under things like the vehicle icons.
	
	//Draw message boxes
	m_guidanceOverlay.Draw_MessageBox(mousePosNM, draw_list, mouseInBounds);
	
	ImGui::EndChild();
	mouseInBounds = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
	
	//Update mouse position
	//if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem)) {
	//	mousePosScreenSpace = ImGui::GetMousePos();
	//	mousePosNM = ScreenCoordsToNormalizedMercator(mousePosScreenSpace);
	//}
	
	//Draw data tooltip
	if (DrawDataTooltip && (! dragging) && (! AnimationInProgress) && mouseInBounds) {
		Maps::DataTileProvider * DataProvider = Maps::DataTileProvider::Instance();
		double MSAValue, AvoidanceZonesValue, SafeLandingZonesValue;
		if (DataProvider->TryGetData(mousePosNM, Maps::DataLayer::MinSafeAltitude,  MSAValue) &&
		    DataProvider->TryGetData(mousePosNM, Maps::DataLayer::AvoidanceZones,   AvoidanceZonesValue) &&
		    DataProvider->TryGetData(mousePosNM, Maps::DataLayer::SafeLandingZones, SafeLandingZonesValue)) {
			std::string tooltipText;
			if (std::isnan(MSAValue))
				tooltipText += "No Fly Zone";
			else {
				std::ostringstream outSS;
				outSS << std::fixed << std::setprecision(1) << MSAValue;
				tooltipText += "Min Safe Altitude: "s + outSS.str() + " m"s;
			}
			if ((! std::isnan(AvoidanceZonesValue)) && (AvoidanceZonesValue > 0.5))
				tooltipText += "\r\nAvoidance Zone";
			if ((! std::isnan(SafeLandingZonesValue)) && (SafeLandingZonesValue > 0.5))
				tooltipText += "\r\nSafe Landing Zone";
			ImGui::SetTooltip(tooltipText.c_str());
		}
	}
	
	//If we aren't in an animation, handle mouse scroll and drag events
	if (! Draw_NavigationAnimation()) {
		//We keep track of the drag state ourselves so we can allow dragging to work even if the mouse leaves the widget
		bool toolActive = m_AvoidanceZonesTool.toolActive || m_LandingZonesTool.toolActive || m_MSATool.toolActive || m_SurveyRegionsTool.toolActive;
		bool startDrag = (mouseInBounds && processMouseInputs && ((ImGui::IsMouseClicked(1)) || (ImGui::IsMouseClicked(0) && (! toolActive))));
		
		//On ctrl-click, copy mouse lat/lon to clipboard
		if (ImGui::IsMouseClicked(0) && mouseInBounds && (ImGui::GetIO().KeyCtrl)) {
			Eigen::Vector2d mousePos_NM = ScreenCoordsToNormalizedMercator(ImGui::GetMousePos());
			Eigen::Vector2d mousePos_LatLon = NMToLatLon(mousePos_NM);
			
			std::ostringstream out;
			out << std::fixed << std::setprecision(6) << mousePos_LatLon(0)*180.0/PI << ", " << mousePos_LatLon(1)*180.0/PI;
			std::string clipboardText = out.str();
			ImGui::SetClipboardText(clipboardText.c_str());
			startDrag = false;
		}
		
		if (startDrag) {
			dragging = true;
			MousePos_NormalizedMercator_OnLastClick = ScreenCoordsToNormalizedMercator(ImGui::GetMousePos());
		}
		else if ((! ImGui::IsMouseDown(0)) && (! ImGui::IsMouseDown(1)))
			dragging = false;
		
		if (dragging)
			SetMapPan(ImGui::GetMousePos(), MousePos_NormalizedMercator_OnLastClick); //Adjust map pan to the ground point is still under the cursor
		
		//Adjust zoom level on scroll
		float scroll = ImGui::GetIO().MouseWheel;
		if ((scroll != 0.0f) && (dragging || mouseInBounds)) {
			Eigen::Vector2d GroundPointNM = ScreenCoordsToNormalizedMercator(ImGui::GetMousePos());
			zoom *= 1.0f + 0.01f *scroll * ProgOptions::Instance()->zoomSpeed;
			SetMapPan(ImGui::GetMousePos(), GroundPointNM); //Adjust map pan to the ground point is still under the cursor
		}
	}
	
	ImGui::PopStyleColor();
	ImGui::PopStyleVar(2);
}

//Should be called from main draw loop, regardless of whether widget is being drawn
void MapWidget::Update() {
	//Currently nothing to do... Can remove this if it stays this way
}




