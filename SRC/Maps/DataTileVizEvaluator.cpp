//This module provides the functions for evaluating visualization tiles from FRF tiles.
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//System Includes
#include <iostream>

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "DataTileVizEvaluator.hpp"
#include "../UI/TextureUploadFlowRestrictor.hpp"

namespace Maps {
	static void evaluateRGBAFromColormap(double Value, double MinValue, double MaxValue, std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> const & map,
	                                     uint8_t & Red, uint8_t & Green, uint8_t & Blue, uint8_t & Alpha) {
		if (map.size() == 0U) {
			Red   = 0U;
			Green = 0U;
			Blue  = 0U;
			Alpha = 0U;
			return;
		}
		else if (map.size() == 1U) {
			Red   = std::get<0>(map[0]);
			Green = std::get<1>(map[0]);
			Blue  = std::get<2>(map[0]);
			Alpha = 255U;
			return;
		}
		
		double range = MaxValue - MinValue;
		if ((range < 1e-9) || (Value <= MinValue)) {
			Red   = std::get<0>(map[0]);
			Green = std::get<1>(map[0]);
			Blue  = std::get<2>(map[0]);
			Alpha = 255U;
			return;
		}
		else if (Value >= MaxValue) {
			Red   = std::get<0>(map.back());
			Green = std::get<1>(map.back());
			Blue  = std::get<2>(map.back());
			Alpha = 255U;
			return;
		}
		else {
			double t = (Value - MinValue)/range;
			double scaled_t = t*((double) (map.size() - 1));
			int interval = std::max(std::min((int) floor(scaled_t), (int) map.size() - 2), 0);
			std::tuple<uint8_t,uint8_t,uint8_t> c1 = map[interval];
			std::tuple<uint8_t,uint8_t,uint8_t> c2 = map[interval + 1];
			double s = scaled_t - (double) interval;
			
			Red   = (uint8_t) std::max(std::min(std::round((1.0 - s)*((double) std::get<0>(c1)) + s*((double) std::get<0>(c2))), 255.0), 0.0);
			Green = (uint8_t) std::max(std::min(std::round((1.0 - s)*((double) std::get<1>(c1)) + s*((double) std::get<1>(c2))), 255.0), 0.0);
			Blue  = (uint8_t) std::max(std::min(std::round((1.0 - s)*((double) std::get<2>(c1)) + s*((double) std::get<2>(c2))), 255.0), 0.0);
			Alpha = 255U;
			return;
		}
	}
	
	//Optimized A-over-B alpha compositing
	static Math::RGBA8888 AlphaCompositePixel(Math::RGBA8888 Front, Math::RGBA8888 Back) {
		Math::Vector3 rgbF((float)Front.R, (float)Front.G, (float)Front.B);
		Math::Vector3 rgbB((float) Back.R, (float) Back.G, (float) Back.B);
		float aF = (float)Front.A;
		float aB = (float) Back.A;
		float aDiff = aB * (255_f - aF) * (1_f / 255_f);
		float aOut = aF + aDiff;
		Math::Vector3 rgbOut = (rgbF * aF + rgbB * aDiff) / aOut;
		return Math::RGBA8888((uint8_t)rgbOut.X, (uint8_t)rgbOut.Y, (uint8_t)rgbOut.Z, (uint8_t)aOut);
	}

	//Try to evaluate a visualization tile (create an RGBA image) and load into GPU memory. Returns the TextureID of the result.
	//Returns nullptr on failure.
	ImTextureID EvaluateVisualizationAndLoadIntoGPUMem(FRFImage const * sourceFRFImage, VizualizationTileKey Key) {
		//Make sure the FRF tile has the correct dimensions
		if ((sourceFRFImage->Width() != 256U) || (sourceFRFImage->Height() != 256U))
			return nullptr;
		
		//Get layer mapping
		int MSAIndex              = -1;
		int AvoidanceZonesIndex   = -1;
		int SafeLandingZonesIndex = -1;
		for (uint16_t LayerIndex = 0U; LayerIndex < sourceFRFImage->NumberOfLayers(); LayerIndex++) {
			if (sourceFRFImage->Layer(LayerIndex)->Name == "MSA")
				MSAIndex = (int) LayerIndex;
			else if (sourceFRFImage->Layer(LayerIndex)->Name == "Avoidance Zones")
				AvoidanceZonesIndex = (int) LayerIndex;
			else if (sourceFRFImage->Layer(LayerIndex)->Name == "Safe Landing Zones")
				SafeLandingZonesIndex = (int) LayerIndex;
		}
		
		//Create a buffer for the visualization tile
		std::vector<uint8_t> data(256*256*4, 0);
		
		//Evaluate the visualization on a pixel-by-pixel basis
		std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> const & MSA_cmap = Colormaps::GetColormap(Key.MSA_colormap);
		unsigned int pixelNum = 0U;
		for (unsigned int row = 0U; row < 256U; row++) {
			for (unsigned int col = 0U; col < 256U; col++) {
				Math::RGBA8888 RGBA(255_u8, 255_u8, 255_u8, 0_u8); //Initialize to transparent white
				
				//Render MSA first
				if (Key.Opacity_MSA > (uint8_t) 0U) {
					double MSA = sourceFRFImage->GetValue(MSAIndex, (uint16_t) row, (uint16_t) col);
					if (std::isnan(MSA)) {
						//No-Fly Zone
						switch (Key.MSA_NoFlyDrawMode) {
							case 0: break; //Transparent No-Fly Zones
							case 1:        //Striped No-Fly Zones
								if ((row + col) % 32 < 4)
									RGBA = Math::RGBA8888(255_u8, 40_u8, 40_u8, Key.Opacity_MSA);
								break;
							default: break;
						}
					}
					else {
						//Not in a No-Fly Zone
						uint8_t MSA_R, MSA_G, MSA_B, MSA_A;
						evaluateRGBAFromColormap(MSA, Key.MSA_ColormapMinVal, Key.MSA_ColormapMaxVal, MSA_cmap, MSA_R, MSA_G, MSA_B, MSA_A);
						RGBA = Math::RGBA8888(MSA_R, MSA_G, MSA_B, Key.Opacity_MSA);
					}
				}
				
				//Render Safe Landing Zones second
				if (Key.Opacity_SafeLandingZones > (uint8_t) 0U) {
					double value = sourceFRFImage->GetValue(SafeLandingZonesIndex, (uint16_t) row, (uint16_t) col);
					if ((! std::isnan(value)) && (value > 0.5)) {
						//This pixel is in a zone
						Math::RGBA8888 front(0_u8, 200_u8, 0_u8, Key.Opacity_SafeLandingZones); //Green
						RGBA = AlphaCompositePixel(front, RGBA);
					}
				}
				
				//Render Avoidance Zones third
				if (Key.Opacity_AvoidanceZones > (uint8_t) 0U) {
					double value = sourceFRFImage->GetValue(AvoidanceZonesIndex, (uint16_t) row, (uint16_t) col);
					if ((! std::isnan(value)) && (value > 0.5)) {
						//This pixel is in a zone
						Math::RGBA8888 front(240_u8, 240_u8, 0_u8, Key.Opacity_AvoidanceZones); //Yellow
						RGBA = AlphaCompositePixel(front, RGBA);
					}
				}
				
				//Write to buffer
				data[4U*pixelNum     ] = RGBA.R;
				data[4U*pixelNum + 1U] = RGBA.G;
				data[4U*pixelNum + 2U] = RGBA.B;
				data[4U*pixelNum + 3U] = RGBA.A;
				
				pixelNum++;
			}
		}
		
		TextureUploadFlowRestrictor::Instance().WaitUntilUploadIsAllowed();
		ImTextureID tex = ImGuiApp::Instance().CreateImageRGBA8888(&data[0], 256, 256);

		return tex;
	}
}



