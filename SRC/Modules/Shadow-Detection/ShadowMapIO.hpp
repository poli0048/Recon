#pragma once

//System Includes
#include <stdint.h>
#include <vector>
#include <string>
#include <tuple>

//Project Includes
#include "FRF.h"

namespace ShadowMapIO {
	//The FRF serialization functions have internal linkage to the FRF translation unit. We need to use a few of them in order to
	//read and write the shadow map information block, so we copy them to our ShadowMapIO namespace.
	static inline void encodeField_uint32 (std::vector<uint8_t> & Buffer, uint32_t x);
	static inline void encodeField_uint64 (std::vector<uint8_t> & Buffer, uint64_t x);
	static inline void encodeField_float64(std::vector<uint8_t> & Buffer, double x);
	static inline void encodeField_GPST (std::vector<uint8_t> & Buffer, uint32_t Week, double TOW);
	
	static inline uint32_t decodeField_uint32 (std::vector<uint8_t>::const_iterator & Iter);
	static inline uint64_t decodeField_uint64 (std::vector<uint8_t>::const_iterator & Iter);
	static inline double decodeField_float64 (std::vector<uint8_t>::const_iterator & Iter);
	static inline std::tuple<uint32_t,double> decodeField_GPST (std::vector<uint8_t>::const_iterator & Iter);
}

static inline bool IsShadowMapFile(FRFImage const & File) { return File.HasCustomBlock((uint64_t) 0U); }

class ShadowMapInfoBlock {
	public:
		uint32_t FileTimeEpoch_Week = 0U;           //The GPS week of the time-0 epoch used for file time. Set to 0 when absolute time is unknown
		double   FileTimeEpoch_TOW  = std::nan(""); //The GPW Time of Week of the time-0 epoch used for file time. Set to NaN when absolute time is unknown
		std::vector<double> LayerTimeTags;          //File-time tags for each layer. item n corresponds to layer n. Must have exactly one item per layer
		
		bool AttachToFRFFile(FRFImage & File) const {
			if (size_t(File.NumberOfLayers()) != LayerTimeTags.size())
				return false;
			FRFCustomBlock * shadowMapInfoBlock = File.AddCustomBlock((uint64_t) 0U);
			ShadowMapIO::encodeField_GPST(shadowMapInfoBlock->BlockPayload, FileTimeEpoch_Week, FileTimeEpoch_TOW); //Add the FileTimeEpoch field
			for (double time : LayerTimeTags)
				ShadowMapIO::encodeField_float64(shadowMapInfoBlock->BlockPayload, time);
			return true;
		}
		
		bool LoadFromFRFFile(FRFImage const & File) {
			if (! File.HasCustomBlock((uint64_t) 0U))
				return false;
			FRFCustomBlock const * shadowMapInfoBlock = File.CustomBlock((uint64_t) 0U) ;
			
			//Check the size of the block - the CustomBlockCode field is already stripped from the custom block payload
			size_t expectedSize = 12U + 8U*File.NumberOfLayers();
			if (shadowMapInfoBlock->BlockPayload.size() != expectedSize)
				return false;
			
			auto iter = shadowMapInfoBlock->BlockPayload.cbegin();
			std::tuple<uint32_t, double> GPST = ShadowMapIO::decodeField_GPST(iter);
			FileTimeEpoch_Week = std::get<0>(GPST);
			FileTimeEpoch_TOW  = std::get<1>(GPST);
			LayerTimeTags.clear();
			LayerTimeTags.reserve(File.NumberOfLayers());
			for (uint16_t layerIndex = 0U; layerIndex < File.NumberOfLayers(); layerIndex++)
				LayerTimeTags.push_back(ShadowMapIO::decodeField_float64(iter));
			return true;
		}
};

// *******************************************************************************************************************************
// ********************   Copy the handful of FRF field encoders needed to create the shadow map info block   ********************
// *******************************************************************************************************************************
static inline void ShadowMapIO::encodeField_uint32 (std::vector<uint8_t> & Buffer, uint32_t x) {
	Buffer.push_back((uint8_t) (x >> 24));
	Buffer.push_back((uint8_t) (x >> 16));
	Buffer.push_back((uint8_t) (x >> 8 ));
	Buffer.push_back((uint8_t)  x       );
}

static inline void ShadowMapIO::encodeField_uint64 (std::vector<uint8_t> & Buffer, uint64_t x) {
	Buffer.push_back((uint8_t) (x >> 56));
	Buffer.push_back((uint8_t) (x >> 48));
	Buffer.push_back((uint8_t) (x >> 40));
	Buffer.push_back((uint8_t) (x >> 32));
	Buffer.push_back((uint8_t) (x >> 24));
	Buffer.push_back((uint8_t) (x >> 16));
	Buffer.push_back((uint8_t) (x >> 8 ));
	Buffer.push_back((uint8_t)  x       );
}

static inline void ShadowMapIO::encodeField_float64(std::vector<uint8_t> & Buffer, double x)  {encodeField_uint64(Buffer, reinterpret_cast<uint64_t &>(x));}

static inline void ShadowMapIO::encodeField_GPST (std::vector<uint8_t> & Buffer, uint32_t Week, double TOW) {
	encodeField_uint32(Buffer, Week);
	encodeField_float64(Buffer, TOW);
}

// *******************************************************************************************************************************
// *********************   Copy the handful of FRF field deconder needed to read the shadow map info block   *********************
// *******************************************************************************************************************************
static inline uint32_t ShadowMapIO::decodeField_uint32 (std::vector<uint8_t>::const_iterator & Iter) {
	uint32_t value;
	value  = (uint32_t) *Iter++;  value <<= 8;
	value += (uint32_t) *Iter++;  value <<= 8;
	value += (uint32_t) *Iter++;  value <<= 8;
	value += (uint32_t) *Iter++;
	return(value);
}

static inline uint64_t ShadowMapIO::decodeField_uint64 (std::vector<uint8_t>::const_iterator & Iter) {
	uint64_t value;
	value  = (uint64_t) *Iter++;  value <<= 8;
	value += (uint64_t) *Iter++;  value <<= 8;
	value += (uint64_t) *Iter++;  value <<= 8;
	value += (uint64_t) *Iter++;  value <<= 8;
	value += (uint64_t) *Iter++;  value <<= 8;
	value += (uint64_t) *Iter++;  value <<= 8;
	value += (uint64_t) *Iter++;  value <<= 8;
	value += (uint64_t) *Iter++;
	return(value);
}

static inline double ShadowMapIO::decodeField_float64 (std::vector<uint8_t>::const_iterator & Iter) {
	uint64_t bitPattern = decodeField_uint64(Iter);
	return reinterpret_cast<double &>(bitPattern);
}

static inline std::tuple<uint32_t,double> ShadowMapIO::decodeField_GPST (std::vector<uint8_t>::const_iterator & Iter) {
	uint32_t Week = decodeField_uint32(Iter);
	double   TOW  = decodeField_float64(Iter);
	return std::make_tuple(Week, TOW);
}



