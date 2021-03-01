//This header defines some basic structs for referencing visualization tiles, derived from FRF tiles
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 
#pragma once

//External Includes
#include "../../../handycpp/Handy.hpp"

//Project Includes
#include "Tile.hpp"
#include "../Colormaps.hpp"

namespace Maps {
	enum class DataLayer {
		None,
		MinSafeAltitude,
		AvoidanceZones,
		SafeLandingZones
	};
	
	//Get string representation of a DataLayer (use this for the FRF layer name)
	inline std::string DataLayerToString(DataLayer layer) {
		switch(layer) {
			case DataLayer::None:             return std::string("None");
			case DataLayer::MinSafeAltitude:  return std::string("MSA");
			case DataLayer::AvoidanceZones:   return std::string("Avoidance Zones");
			case DataLayer::SafeLandingZones: return std::string("Safe Landing Zones");
			default:                          return std::string("None");
		}
	}
	
	//Convert string representation of DataLayer to enum class object
	inline DataLayer StringToDataLayer(std::string const & layer) {
		if (layer == std::string("None"))
			return DataLayer::None;
		else if (layer == std::string("MSA"))
			return DataLayer::MinSafeAltitude;
		else if (layer == std::string("Avoidance Zones"))
			return DataLayer::AvoidanceZones;
		else if (layer == std::string("Safe Landing Zones"))
			return DataLayer::SafeLandingZones;
		else
			return DataLayer::None;
	}

	//Visualizations are identified by the tile coordinates (X, Y, Zoom), the layer being viewed, and the specific parameters of the visualization.
	struct VizualizationTileKey {
		Tile tile;
		
		uint8_t Opacity_MSA;
		uint8_t Opacity_AvoidanceZones;
		uint8_t Opacity_SafeLandingZones;
		
		Colormap MSA_colormap;
		float MSA_ColormapMinVal;
		float MSA_ColormapMaxVal;
		int   MSA_NoFlyDrawMode;
	
		VizualizationTileKey() = default;
		~VizualizationTileKey() = default;
	
		bool operator==(VizualizationTileKey const & other) const {
			return (this->tile                     == other.tile                     &&
			        this->Opacity_MSA              == other.Opacity_MSA              &&
			        this->Opacity_AvoidanceZones   == other.Opacity_AvoidanceZones   &&
			        this->Opacity_SafeLandingZones == other.Opacity_SafeLandingZones &&
			        this->MSA_colormap             == other.MSA_colormap             &&
			        this->MSA_ColormapMinVal       == other.MSA_ColormapMinVal       &&
			        this->MSA_ColormapMaxVal       == other.MSA_ColormapMaxVal       &&
			        this->MSA_NoFlyDrawMode        == other.MSA_NoFlyDrawMode);
		}
		bool operator!=(VizualizationTileKey const & other) const { return (! operator==(other)); }
	};
}

MAKE_HASHABLE(Maps::VizualizationTileKey, t.tile, t.Opacity_MSA, t.Opacity_AvoidanceZones, t.Opacity_SafeLandingZones,
              t.MSA_colormap, t.MSA_ColormapMinVal, t.MSA_ColormapMaxVal, t.MSA_NoFlyDrawMode)



