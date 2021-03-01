//This module defines colormaps
//Author: Bryan Poling
#pragma once

//System Includes
#include <vector>
#include <tuple>
#include <cstdint>

enum class Colormap {
	None,
	BlackToWhite,
	ClassicNIR,
	RedToGreen,
	RedToBlue,
	BlueToRed
};

//Colormap RGB Definitions - each is a vector of <R,G,B> tuples
namespace Colormaps {
	const std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> cmap_None;
	const std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> cmap_BlackToWhite{ std::make_tuple((uint8_t)   0U, (uint8_t)   0U, (uint8_t)   0U),
		                                                                     std::make_tuple((uint8_t) 255U, (uint8_t) 255U, (uint8_t) 255U) };
	const std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> cmap_ClassicNIR{   std::make_tuple((uint8_t)   0U, (uint8_t)   0U, (uint8_t)   0U),
		                                                                     std::make_tuple((uint8_t)  76U, (uint8_t) 152U, (uint8_t) 152U),
		                                                                     std::make_tuple((uint8_t) 117U, (uint8_t) 185U, (uint8_t) 170U),
		                                                                     std::make_tuple((uint8_t) 138U, (uint8_t) 255U, (uint8_t) 255U),
		                                                                     std::make_tuple((uint8_t) 217U, (uint8_t) 105U, (uint8_t) 141U),
		                                                                     std::make_tuple((uint8_t) 195U, (uint8_t)   0U, (uint8_t)  50U) };
	const std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> cmap_RedToGreen{   std::make_tuple((uint8_t) 215U, (uint8_t)  25U, (uint8_t)  28U),
		                                                                     std::make_tuple((uint8_t) 253U, (uint8_t) 174U, (uint8_t)  97U),
		                                                                     std::make_tuple((uint8_t) 255U, (uint8_t) 255U, (uint8_t)  89U),
		                                                                     std::make_tuple((uint8_t) 166U, (uint8_t) 217U, (uint8_t) 106U),
		                                                                     std::make_tuple((uint8_t)  25U, (uint8_t) 150U, (uint8_t)  65U) };
	const std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> cmap_RedToBlue{    std::make_tuple((uint8_t) 202U, (uint8_t)   0U, (uint8_t)  32U),
		                                                                     std::make_tuple((uint8_t) 244U, (uint8_t) 165U, (uint8_t) 130U),
		                                                                     std::make_tuple((uint8_t) 247U, (uint8_t) 247U, (uint8_t) 247U),
		                                                                     std::make_tuple((uint8_t) 146U, (uint8_t) 197U, (uint8_t) 222U),
		                                                                     std::make_tuple((uint8_t)   5U, (uint8_t) 113U, (uint8_t) 176U) };
	const std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> cmap_BlueToRed{    std::make_tuple((uint8_t)   5U, (uint8_t) 113U, (uint8_t) 176U),
		                                                                     std::make_tuple((uint8_t) 146U, (uint8_t) 197U, (uint8_t) 222U),
		                                                                     std::make_tuple((uint8_t) 247U, (uint8_t) 247U, (uint8_t) 247U),
		                                                                     std::make_tuple((uint8_t) 244U, (uint8_t) 165U, (uint8_t) 130U),
		                                                                     std::make_tuple((uint8_t) 202U, (uint8_t)   0U, (uint8_t)  32U) };
	
	inline std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> const & GetColormap(Colormap cmap) {
		switch(cmap) {
			case Colormap::None:         return cmap_None;
			case Colormap::BlackToWhite: return cmap_BlackToWhite;
			case Colormap::ClassicNIR:   return cmap_ClassicNIR;
			case Colormap::RedToGreen:   return cmap_RedToGreen;
			case Colormap::RedToBlue:    return cmap_RedToBlue;
			case Colormap::BlueToRed:    return cmap_BlueToRed;
			default:                     return cmap_None;
		}
	}
}



