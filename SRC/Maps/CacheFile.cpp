//Class for managing a cache file for satellite image tiles. Support asynchronous and synchronous item retrieval

//System Includes
#include <thread>
#include <iostream>
#include <system_error>

//External Includes
#include "../../../handycpp/Handy.hpp"

//Project Includes
#include "CacheFile.hpp"
#include "../SimpleKVStore.hpp"

namespace Maps {

CacheFile::CacheFile(ITileFileReceiver * receiver, Journal & LogRef) : Log(LogRef), m_receiver(receiver), m_threads(4) {
	//Find the Local App cache directory (create it is it doesn't exist) and compute the path to the cache file
	std::filesystem::path SatCachePath = Handy::Paths::CacheDirectory("SentekRecon") / "SatCache";
	
	m_file = new SimpleKVStore(SatCachePath, Log);
	if (! m_file->IsOpen())
		Log.print("Error: Failed to open sat tile cache: " + SatCachePath.string());
}

CacheFile::~CacheFile() {
	WaitFinish();
	Handy::SafeDelete(m_file);
}

void CacheFile::PurgeAll() {
	m_file->Clear();
}

//Cache file size inspection
uint64_t CacheFile::GetNumItems()       { return m_file->GetNumItems();       }
uint64_t CacheFile::GetNumBytes()       { return m_file->GetNumBytes();       }
uint64_t CacheFile::GetNumBytesOnDisk() { return m_file->GetNumBytesOnDisk(); }

std::shared_ptr<std::vector<uint8_t>> CacheFile::BlockTryGet(Tile tile, SatelliteSource source) {
	std::string Identifier = GetTileIdentifier(tile, source);
	std::vector<uint8_t> * v = new std::vector<uint8_t>();
	if (m_file->Get(Identifier, *v))
		return std::shared_ptr<std::vector<uint8_t>>(v);
	else {
		delete v;
		return nullptr;
	}
}

void CacheFile::BlockAdd(Tile tile, SatelliteSource source, std::vector<uint8_t> const & data) {
	std::string Identifier = GetTileIdentifier(tile, source);
	if (! m_file->PutIfNew(Identifier, data))
		Log.print("Warning in CacheFile::BlockAdd: Item either already exists in cache or adding to store failed. Tile: " + tile.ToString());
}

void CacheFile::BlockRemove(Tile tile, SatelliteSource source) {
	std::string Identifier = GetTileIdentifier(tile, source);
	m_file->Delete(Identifier);
}

void CacheFile::WaitFinish() {
	m_threads.Wait();
}

void CacheFile::RetrieveAsync(Tile tile, SatelliteSource source) {
	std::tuple<Tile, SatelliteSource> request(tile, source);
	bool begin = false;
	{
		std::lock_guard<std::mutex> lock(m_requests_mtx);
		if ((! Handy::Contains(m_requests, request)) && (m_requests.size() < MaxOutstandingRequests)) {
			m_requests.insert(request);
			begin = true;
		}
	}
	
	if (begin) {
		m_threads.AddJobToFront([this,tile,source,request]() {
			try {
				auto data = BlockTryGet(tile, source);
				m_receiver->OnReceivedFile(tile, source, data); //Called, even if we failed to retrieve the file (Triggers web retrieval)
			} catch(...) {
				Log.print("Error in CacheFile::RetrieveAsync: Retrieval failed for tile: " + tile.ToString());
			}
			std::lock_guard<std::mutex> lock(m_requests_mtx);
			m_requests.erase(request);
		});
	}
}
}


