//This module provides utility functions for coordinate conversions, tile lookup, and related things
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <cstdint>
#include <tuple>
#include <cmath>

//Eigen Includes
#include "../../../eigen/Eigen/Core"

//We have two map coordinate systems. The first is Widget coordinates. This is the position of a pixel relative to the upper-left corner of the
//map widget, in pixels. X goes right and y goes down. The second coordinate system is Normalized Mercator, which is used to reference
//locations on the Earth. We have utilities to map back and forth between NM and widget coordinates. We also sometimes need to go back and
//forth to Lat/Lon, but normalized mercator should be used internally whenever possible for consistency.

inline Eigen::Vector2d NMToLatLon(Eigen::Vector2d const & NMCoords) {
	const double PI = 3.14159265358979;
	double x = NMCoords(0);
	double y = NMCoords(1);
	double lon = PI*x;
	double lat = 2.0*(atan(exp(y*PI)) - PI/4.0);
	return Eigen::Vector2d(lat,lon);
}

inline Eigen::Vector2d LatLonToNM(Eigen::Vector2d const & LatLon) {
	const double PI = 3.14159265358979;
	double lat = LatLon(0);
	double lon = LatLon(1);
	double x = lon / PI;
	double y = log(tan(lat/2.0 + PI/4.0)) / PI;
	return Eigen::Vector2d(x,y);
}

inline Eigen::Vector2d WidgetCoordsToNormalizedMercator(Eigen::Vector2d const & WidgetCords, Eigen::Vector2d const & ULCorner_NM, double Zoom, int32_t tileWidth) {
	double screenPixelLengthNM = 2.0 / (pow(2.0, Zoom) * ((double) tileWidth));
	return (ULCorner_NM + Eigen::Vector2d(WidgetCords(0)*screenPixelLengthNM, -1.0*WidgetCords(1)*screenPixelLengthNM));
}

inline Eigen::Vector2d NormalizedMercatorToWidgetCoords(Eigen::Vector2d const & NMCoords, Eigen::Vector2d const & ULCorner_NM, double Zoom, int32_t tileWidth) {
	double screenPixelLengthNM = 2.0 / (pow(2.0, Zoom) * ((double) tileWidth));
	return Eigen::Vector2d(NMCoords(0) - ULCorner_NM(0), ULCorner_NM(1) - NMCoords(1))/screenPixelLengthNM;
}

//Convert a distance in meters to normalized mercator units at the given point on Earth (only the y-coord matters).
//Be careful with this - it is not perfect and gets especially bad over long distances.
inline double MetersToNMUnits(double Meters, double yPos_NM) {
	const double PI = 3.14159265358979;
	const double C = 40075017.0; //meters (Wikipedia)
	
	double lat = 2.0*(atan(exp(yPos_NM*PI)) - PI/4.0);
	double NMUnitsPerMeter = 2.0/(C*cos(lat));
	return Meters*NMUnitsPerMeter;
}

//Convert a distance in meters to pixels at the given zoom level and point on Earth (only the y-coord matters).
//Be careful with this - it is not perfect and gets especially bad over long distances.
inline double MetersToPixels(double Meters, double yPos_NM, double MapZoom) {
	const double PI = 3.14159265358979;
	const double C = 40075017.0; //meters (Wikipedia)
	
	double lat = 2.0*(atan(exp(yPos_NM*PI)) - PI/4.0);
	double PixelsPerMeter = pow(2.0, MapZoom + 8.0)/(C*cos(lat));
	return Meters*PixelsPerMeter;
}

//Convert a distance in pixels at a given zoom level to NM units at the given point on Earth (only the y-coord matters).
//Be careful with this - it is not perfect and gets especially bad over long distances.
inline double PixelsToNMUnits(double Pixels, double yPos_NM, double MapZoom) {
	const double PI = 3.14159265358979;
	const double C = 40075017.0; //meters (Wikipedia)
	
	double lat = 2.0*(atan(exp(yPos_NM*PI)) - PI/4.0);
	double NMUnitsPerMeter = 2.0/(C*cos(lat));
	double PixelsPerMeter = pow(2.0, MapZoom + 8.0)/(C*cos(lat));
	double NMUnitsPerPixel = NMUnitsPerMeter / PixelsPerMeter;
	return Pixels*NMUnitsPerPixel;
}

//Take the Upper-Left corner location of a map in Normalized Mercator coordinates and the zoom level and map dimensions (Width x Height in piexls) and get
//the X and Y limits of the viewable area in Normalized Mercator coordinates. Returned in form (XMin, XMax, YMin, YMax)
inline Eigen::Vector4d GetViewableArea_NormalizedMercator(Eigen::Vector2d const & ULCorner_NM, Eigen::Vector2d const & WindowDims, double Zoom, int32_t tileWidth) {
	Eigen::Vector2d LRCorner_NM = WidgetCoordsToNormalizedMercator(WindowDims, ULCorner_NM, Zoom, tileWidth);
	return Eigen::Vector4d(ULCorner_NM(0), LRCorner_NM(0), LRCorner_NM(1), ULCorner_NM(1));
}

//Get the Normalized-Mercator coordinates of the center of pixel (row, col) in the given tile
inline Eigen::Vector2d TilePixelToNM(int32_t TileX, int32_t TileY, int32_t PyramidLevel, int Row, int Col, int32_t tileWidth) {
	int32_t tilesOnThisLevel = (int32_t) (1U << PyramidLevel); //In each dimension
	double xNM = (double(TileX) + (double(Col) + 0.5)/double(tileWidth))*2.0 / (double(tilesOnThisLevel)) - 1.0;
	double yNM = 1.0 - (double(TileY) + (double(Row) + 0.5)/double(tileWidth))*2.0 / double(tilesOnThisLevel);
	return Eigen::Vector2d(xNM, yNM);
}

//Get the pixel coordinates of the given Normalized-Mercator position in the given tile. Returned in form <col, row>.
//This is the inverse of TilePixelToNM()
inline Eigen::Vector2d NMToTilePixel(int32_t TileX, int32_t TileY, int32_t PyramidLevel, Eigen::Vector2d const & Position_NM, int32_t tileWidth) {
	int32_t tilesOnThisLevel = (int32_t) (1U << PyramidLevel); //In each dimension
	double Col = ((1.0 + Position_NM(0))*double(tilesOnThisLevel)/2.0 - double(TileX))*double(tileWidth) - 0.5;
	double Row = ((1.0 - Position_NM(1))*double(tilesOnThisLevel)/2.0 - double(TileY))*double(tileWidth) - 0.5;
	return Eigen::Vector2d(Col, Row);
}

//Get the pixel containing the given Normalized-Mercator position in the given tile. Returned in form <col, row>. This version
//saturates each coordinate to [0, tileWidth-1]
inline std::tuple<int, int> NMToTilePixel_int(int32_t TileX, int32_t TileY, int32_t PyramidLevel, Eigen::Vector2d const & Position_NM, int32_t tileWidth) {
	int32_t tilesOnThisLevel = (int32_t) (1U << PyramidLevel); //In each dimension
	double Col = ((1.0 + Position_NM(0))*double(tilesOnThisLevel)/2.0 - double(TileX))*double(tileWidth) - 0.5;
	double Row = ((1.0 - Position_NM(1))*double(tilesOnThisLevel)/2.0 - double(TileY))*double(tileWidth) - 0.5;
	Col = floor(std::min(std::max(Col, 0.0), double(tileWidth - 1)));
	Row = floor(std::min(std::max(Row, 0.0), double(tileWidth - 1)));
	return std::make_tuple(int(Col), int(Row));
}

//Get <tileCol, tileRow> for the tile on the given pyramid level containing the given point in Normalized Mercator coordinates
inline std::tuple<int32_t, int32_t> getCoordsOfTileContainingPoint(Eigen::Vector2d const & PointNM, int32_t PyramidLevel) {
	int32_t tilesOnThisLevel = (int32_t) (1U << PyramidLevel); //In each dimension
	int32_t tileX = floor((PointNM(0) + 1.0)*tilesOnThisLevel/2.0);
	int32_t tileY = floor((1.0 - PointNM(1))*tilesOnThisLevel/2.0);
	tileX = std::max((int32_t) 0, std::min(tilesOnThisLevel - 1, tileX));
	tileY = std::max((int32_t) 0, std::min(tilesOnThisLevel - 1, tileY));
	return std::make_tuple(tileX, tileY);
}

//Get NM coords of upper-left corner of upper-left pixel of a given tile
inline Eigen::Vector2d GetNMCoordsOfULCornerOfTile(int32_t TileX, int32_t TileY, int32_t PyramidLevel) {
	int32_t tilesOnThisLevel = (int32_t) (1U << PyramidLevel); //In each dimension
	double xNM = ((double) TileX)*2.0 / ((double) tilesOnThisLevel) - 1.0;
	double yNM = 1.0 - ((double) TileY)*2.0 / ((double) tilesOnThisLevel);
	return Eigen::Vector2d(xNM, yNM);
}

//Get NM coords of lower-right corner of lower-right pixel of a given tile
inline Eigen::Vector2d GetNMCoordsOfLRCornerOfTile(int32_t TileX, int32_t TileY, int32_t PyramidLevel) {
	int32_t tilesOnThisLevel = (int32_t) (1U << PyramidLevel); //In each dimension
	double xNM = ((double) (TileX+1))*2.0 / ((double) tilesOnThisLevel) - 1.0;
	double yNM = 1.0 - ((double) (TileY+1))*2.0 / ((double) tilesOnThisLevel);
	return Eigen::Vector2d(xNM, yNM);
}




