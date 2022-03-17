#pragma once

//System Includes
#include <cstdint>
#include <mutex>
#include <unordered_set>
#include <chrono>

//Project Includes
#include "SatelliteSources.hpp"
#include "Tile.hpp"
#include "Interfaces.hpp"
#include "../Journal.h"

//External Includes
#include "../../../handycpp/Handy.hpp"

namespace Maps {

	class WebRetriever {
		public:
			static constexpr int32_t MaxOutstandingRequests = 10;

		private:
			Journal & Log;
			std::mutex m_mutex;
			ITileWebReceiver * m_receiver = nullptr;
			std::unordered_set< std::tuple<Tile,SatelliteSource> > m_outstandingRequests;
	
			//m_failedRequests keeps a map of failed requests and maps them to the time of the most recent failure.
			//This can be used to organize warnings and avoid server spamming.
			std::unordered_map<std::tuple<Tile,SatelliteSource>, std::chrono::steady_clock::time_point> m_failedRequests;

			Handy::ThreadPool m_threads;

		public:
			size_t NumOutstandingRequests() const { return m_outstandingRequests.size(); }

			WebRetriever(ITileWebReceiver * receiver, Journal & LogRef);
			~WebRetriever();

			void WaitFinish();
			void RetrieveAsync(Tile tile, SatelliteSource source);
	};

}
