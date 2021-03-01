//This header defines satellite sources and provides functions to get tile identifiers and download URLs
#pragma once

//System Includes
#include <string>
#include <sstream>
#include <cstdint>

//Project Includes
#include "Tile.hpp"

namespace Maps {
	enum class SatelliteSource : int32_t {
		OSMPublicServer,
		HERESatelliteMaps,
		HEREHybridMaps,
	};
	
	//Build URL to access a HERE Maps tile at the given row, column, and zoom level. Size should be 256 or 512.
	//If Hybrid is true, the URL will be for the Satellite + Roads tile. Otherwise it will be for satellite only.
	inline std::string assembleHERE_Tile_URL(int Row, int Col, int ZoomLevel, int Size, bool Hybrid) {
		const std::string HERE_API_KEY("5DFQVt3IN7cc_nosiffriNh03bkp8elNZlmX6wyKH2I"); //Created for Recon on 12/11/2020
		
		int server = 1 + ((Row + Col) % 4);
		std::string BaseURL = std::string("https://") + std::to_string(server) + std::string(".aerial.maps.ls.hereapi.com");
		std::string Path("/maptile/2.1/");
		std::string Resource("maptile");
		std::string MapID("newest");
		std::string Format("jpg");
	
		std::string Scheme = Hybrid ? std::string("hybrid.day") : std::string("satellite.day");
		std::string Zoom = std::to_string(ZoomLevel);
		std::string ColumnNumber = std::to_string(Col);
		std::string RowNumber = std::to_string(Row);
		std::string TileSize = std::to_string(Size);

		std::string assembledURL = BaseURL + Path + Resource + "/" + MapID + "/" + Scheme + "/" + Zoom
			                    + "/" + ColumnNumber + "/" + RowNumber + "/" + TileSize + "/" + Format
			                    + "?apikey=" + HERE_API_KEY;

		//fprintf(stderr,"URL: %s\r\n", assembledURL.c_str());
		return(assembledURL);
	}
	
	//Get a unique identifier for the given tile and source.
	inline std::string GetTileIdentifier(Tile tile, SatelliteSource Source) {
		std::string SourceString;
		switch (Source) {
			case SatelliteSource::OSMPublicServer:   SourceString = std::string("OSMPublicServer");   break;
			case SatelliteSource::HERESatelliteMaps: SourceString = std::string("HERESatelliteMaps"); break;
			case SatelliteSource::HEREHybridMaps:    SourceString = std::string("HEREHybridMaps");    break;
			default:                                 SourceString = std::string("InvalidSource");     break;
		}
		return (SourceString + "-" + std::to_string(tile.Xi) + "-" + std::to_string(tile.Yi) + "-" + std::to_string(tile.Zoom));
	}
	
	//Get the download URL for a given tile and satellite source
	inline std::string GetDownloadURL(Tile tile, SatelliteSource Source) {
		if (Source == SatelliteSource::OSMPublicServer) {
			//Form: http://{a, b, c}.tile.openstreetmap.org/{Zoom}/{Xi}/{Yi}.png
			static int32_t serverNum = 0;
			if (++serverNum > 2)
				serverNum = 0;
		
			std::stringstream s;
			s << "http://";
			switch (serverNum) {
				case 0: s << "a"; break;
				case 1: s << "b"; break;
				case 2: s << "c"; break;
				default: break;
			}
		
			s << ".tile.openstreetmap.org/" << tile.Zoom << "/" << tile.Xi << "/" << tile.Yi << ".png";
			return s.str();
		}
		else if (Source == SatelliteSource::HERESatelliteMaps)
			return assembleHERE_Tile_URL(tile.Yi, tile.Xi, tile.Zoom, 256, false);
		else if (Source == SatelliteSource::HEREHybridMaps)
			return assembleHERE_Tile_URL(tile.Yi, tile.Xi, tile.Zoom, 256, true);

		return std::string();
	}
}



