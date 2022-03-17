//Interfaces for asynchronous web, FRF file, and cache file item retrieval
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <memory>

//Project Includes
#include "Tile.hpp"
#include "SatelliteSources.hpp"
#include "FRF.h"
#include "DataTileTypes.hpp"

namespace Maps {

	struct ITileFileReceiver {
		virtual ~ITileFileReceiver() = default;
		virtual void OnReceivedFile(Tile tile, SatelliteSource source, std::shared_ptr<std::vector<uint8_t>> data) = 0;
	};

	struct ITileWebReceiver {
		virtual ~ITileWebReceiver() = default;
		virtual void OnReceivedWeb(Tile tile, SatelliteSource source, std::shared_ptr<std::vector<uint8_t>> data) = 0;
	};
	
	struct IFRFFileReceiver {
		virtual ~IFRFFileReceiver() = default;
		virtual void OnReceivedFRFTile(Tile tile, FRFImage * Data) = 0;
	};

}
