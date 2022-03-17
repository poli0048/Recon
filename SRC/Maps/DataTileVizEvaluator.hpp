//This module provides the functions for evaluating visualization tiles from FRF tiles.
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <memory>

//External Includes
#include "../../../handycpp/Handy.hpp"
#include "../../../imgui/imgui.h"

//Project Includes
#include "DataTileTypes.hpp"
#include "FRF.h"

namespace Maps {
	//Try to evaluate a visualization tile (create an RGBA image) and load into GPU memory. Returns the TextureID of the result.
	//Returns nullptr on failure.
	ImTextureID EvaluateVisualizationAndLoadIntoGPUMem(FRFImage const * sourceFRFImage, VizualizationTileKey Key);
}
