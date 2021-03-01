
//System Includes
#include <vector>
#include <random>
#include <iostream>
#include <memory>

//External Includes
#include "../../../restclient-cpp/include/restclient-cpp/connection.h"
#include "../../../restclient-cpp/include/restclient-cpp/restclient.h"
#include "../../../handycpp/Handy.hpp"

//Project Includes
#include "Tile.hpp"
#include "Interfaces.hpp"
#include "WebRetriever.hpp"

namespace Maps {
	WebRetriever::WebRetriever(ITileWebReceiver * receiver, Journal & LogRef)
		: Log(LogRef),
		  m_receiver(receiver),
		  m_threads(4) {
	}
	WebRetriever::~WebRetriever() { WaitFinish(); }

	void WebRetriever::WaitFinish() { m_threads.Wait(); }

	void WebRetriever::RetrieveAsync(Tile tile, SatelliteSource source) {
		std::tuple<Tile, SatelliteSource> key = std::make_tuple(tile, source);
		std::lock_guard<std::mutex> lock(m_mutex);
	
		//If we have the maximum allowed number of outstanding requests, don't add this one. Since we issue new requests every frame this does not put
		//us in danger of "dropping" the tile. We will simply keep trying until there is room in the job pool.
		if (NumOutstandingRequests() > MaxOutstandingRequests)
			return;
	
		//If the request is already in the job pool, we don't need to add it again - just return
		if (Handy::Contains(m_outstandingRequests, key))
			return;
	
		//If we already tried this request and it failed, make sure some time has elapsed before trying again
		if (m_failedRequests.count(key) > 0U) {
			auto TimeOfLastFailure = m_failedRequests[key];
			double secondsElapsed = (std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - TimeOfLastFailure)).count();
			if (secondsElapsed < 5.0)
				return;
			//std::cout << "Time since last failure of this tile: " << secondsElapsed << " seconds. Retrying\r\n";
		}
	
		//We are going ahead with the request - take note of it so we don't add additional copies while the request is processing
		m_outstandingRequests.insert(key);

		std::string url = GetDownloadURL(tile, source);
		if (url == "")
			return;
	
		//Add fetching this tile to the front of the job queue. This gives the new job top priority
		m_threads.AddJobToFront([this,tile,source,key,url]() {
			int code = -1;
			std::vector<uint8_t>   * data = nullptr;
			RestClient::Connection * conn = nullptr;

			//std::this_thread::sleep_for(std::chrono::milliseconds(1000));

			try {
				RestClient::Connection * conn = new RestClient::Connection("");
				conn->SetTimeout(10);
				//  conn->FollowRedirects(true);
				RestClient::Response r = conn->get(url);
				code = r.code;
			
				data = new std::vector<uint8_t>();
				if (code == 200)
					data->assign(r.body.data(), r.body.data() + r.body.length());
			
				Handy::SafeDelete(conn);
			} catch (...) {
				code = -42;
				Handy::SafeDelete(data);
				Handy::SafeDelete(conn);

				std::lock_guard<std::mutex> lock(this->m_mutex);
				m_outstandingRequests.erase(key);
				return;
			}

			if (!data || data->size() <= 0) {
				code = -42;
				Handy::SafeDelete(data);
			}

		
			if (code == 200) {
				//SUCCESS
				{
					std::lock_guard<std::mutex> lock(this->m_mutex);
					m_failedRequests.erase(key);
				}
			
				m_receiver->OnReceivedWeb(tile, source, std::shared_ptr<std::vector<uint8_t>>(data));
			}
			else {
				if (m_failedRequests.count(key) == 0U)
					Log.printf("Failed to download file (RESPONSE = %d)\r\nURL: %s", code, url.c_str());
			
				std::lock_guard<std::mutex> lock(this->m_mutex);
				m_failedRequests[key] = std::chrono::steady_clock::now();
			}
		
			std::lock_guard<std::mutex> lock(this->m_mutex);
			m_outstandingRequests.erase(key);
		});
	}
}


