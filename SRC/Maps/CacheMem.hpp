//Memory cache for satellite tiles - keeps track of the last time an item was accessed and allows you to purge old items
#pragma once

//System Includes
//#include <map>
#include <unordered_map>
#include <mutex>
#include <chrono>
#include <tuple>
#include <ratio>

//External Includes
#include "../../../handycpp/Handy.hpp"
#include "../../../imgui/imgui.h"

//Project Includes
#include "Tile.hpp"
#include "SatelliteSources.hpp"
#include "../Journal.h"

namespace Maps {

struct CacheMem {
		using TimePoint = std::chrono::time_point<std::chrono::system_clock>;
		static constexpr std::chrono::seconds ExpirationTimeSeconds = std::chrono::seconds(20);
//		static constexpr std::chrono::seconds ExpirationTimeSeconds = std::chrono::seconds(3600);

	private:
		Journal & Log;
		std::unordered_map<std::tuple<Tile, SatelliteSource>, std::tuple<ImTextureID, TimePoint>> m_tiles;
		std::mutex m_mutex;
		
	public:
		CacheMem(Journal & LogRef) : Log(LogRef) { }
		~CacheMem() = default;

		void Add(Tile tile, SatelliteSource source, std::vector<uint8_t> & data, int width = 256, int height = 256);
		void Add(Tile tile, SatelliteSource source, uint8_t * data,              int width = 256, int height = 256);

		bool Has(Tile tile, SatelliteSource source);

		ImTextureID Get(Tile tile, SatelliteSource source);

		void PurgeAll();
		void PurgeExpired(double MaxFractionToPurge);
};

}
