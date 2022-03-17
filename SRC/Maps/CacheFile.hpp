//Class for managing a cache file for satellite image tiles. Support asynchronous and synchronous item retrieval
#pragma once

//System Includes
#include <mutex>
#include <memory>
#include <tuple>

//Project Includes
#include "Tile.hpp"
#include "Interfaces.hpp"
#include "SatelliteSources.hpp"
#include "../Journal.h"

class SimpleKVStore;

namespace Maps {
	class CacheFile {
		public:
			static constexpr int32_t MaxOutstandingRequests = 16;

		private:
			Journal & Log;
			std::mutex m_requests_mtx;
			
			SimpleKVStore     * m_file     = nullptr;
			ITileFileReceiver * m_receiver = nullptr;
			std::unordered_set<std::tuple<Tile,SatelliteSource>> m_requests;

			Handy::ThreadPool m_threads;

		public:
			size_t NumOutstandingRequests() {
				std::lock_guard<std::mutex> lock(m_requests_mtx);
				return m_requests.size();
			}

			CacheFile(ITileFileReceiver * receiver, Journal & LogRef);
			~CacheFile();

			void BlockAdd(Tile tile, SatelliteSource source, std::vector<uint8_t> const & data);
			void BlockRemove(Tile tile, SatelliteSource source);

			void RetrieveAsync(Tile tile, SatelliteSource source);
			void WaitFinish();

			void PurgeAll();
			
			//Cache file size inspection
			uint64_t GetNumItems();
			uint64_t GetNumBytes();
			uint64_t GetNumBytesOnDisk();

			std::shared_ptr<std::vector<uint8_t>> BlockTryGet(Tile tile, SatelliteSource source);
	};
}


