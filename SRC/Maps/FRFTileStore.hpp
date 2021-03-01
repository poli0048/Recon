//Class for managing a KV store of FRF tiles. Support asynchronous and synchronous item retrieval
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <mutex>
#include <memory>
#include <tuple>

//Project Includes
#include "Tile.hpp"
#include "Interfaces.hpp"
#include "../Journal.h"

class SimpleKVStore;

namespace Maps {
	class FRFTileStore {
		public:
			static constexpr int32_t MaxOutstandingRequests = 16;

		private:
			Journal & Log;
			std::mutex m_requests_mtx;
			
			SimpleKVStore    * m_file     = nullptr;
			IFRFFileReceiver * m_receiver = nullptr;
			std::unordered_set<Tile> m_requests;

			Handy::ThreadPool m_threads;

		public:
			size_t NumOutstandingRequests() {
				std::scoped_lock lock(m_requests_mtx);
				return m_requests.size();
			}

			FRFTileStore(IFRFFileReceiver * receiver, Journal & LogRef);
			~FRFTileStore();
			
			void BlockAdd(Tile tile, FRFImage const * Data);
			void BlockRemove(Tile tile);

			void RetrieveAsync(Tile tile);
			void WaitFinish();

			void PurgeAll();
			
			//Cache file size inspection
			uint64_t GetNumItems();
			uint64_t GetNumBytes();
			uint64_t GetNumBytesOnDisk();

			std::shared_ptr<std::vector<uint8_t>> BlockTryGet(Tile tile);
	};
}


