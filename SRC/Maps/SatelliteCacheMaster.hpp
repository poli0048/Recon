#pragma once

//System Includes
#include <mutex>

//Project Includes
#include "Tile.hpp"
#include "Interfaces.hpp"
#include "CacheMem.hpp"
#include "CacheFile.hpp"
#include "WebRetriever.hpp"
#include "SatelliteSources.hpp"
#include "../Journal.h"

namespace Maps {
	//Singleton class for serving and managing sat imagery
	class SatelliteCacheMaster : ITileWebReceiver, ITileFileReceiver {
	private:
		static constexpr int32_t MaxZoom = 20;
		static SatelliteCacheMaster * s_instance;
		
		//SatelliteSource m_source = SatelliteSource::OSMPublicServer;
		//SatelliteSource m_source = SatelliteSource::HERESatelliteMaps;
		SatelliteSource m_source = SatelliteSource::HEREHybridMaps;
		Journal & Log;
		
		CacheMem     * m_cacheMem;
		CacheFile    * m_cacheFile;
		WebRetriever * m_webRetriever;
	
		//The SatelliteCacheMaster runs a simple garbage collector in a separate thread to periodically throw out expired tiles
		void GarbageCollectionThreadMain(void);
		std::thread m_garbageCollectionThread;
		std::atomic_bool m_garbageCollectionThreadAbort; //When true, the garbage collection thread will finish up and return

	public:
		static void                   Init(Journal & LogRef) { s_instance = new SatelliteCacheMaster(LogRef); }
		static void                   Destroy(void)          { delete s_instance; s_instance = nullptr; }
		static SatelliteCacheMaster * Instance(void)         { return s_instance; }
		
		SatelliteCacheMaster(Journal & LogRef);
		~SatelliteCacheMaster();
		
		size_t NumOutstandingRequests() const { return m_cacheFile->NumOutstandingRequests() + m_webRetriever->NumOutstandingRequests(); }
		ImTextureID TryGetTouchReq(Tile tile);

		//Force-purge tiles from cache. Note that the cache will self-garbage-collect so this is only needed if you are changing things on disk.
		void PurgeAll();
		
		//Cache file size inspection
		uint64_t CacheFile_GetNumItems()       { return m_cacheFile->GetNumItems();       }
		uint64_t CacheFile_GetNumBytes()       { return m_cacheFile->GetNumBytes();       }
		uint64_t CacheFile_GetNumBytesOnDisk() { return m_cacheFile->GetNumBytesOnDisk(); }

		void OnReceivedFile(Tile tile, SatelliteSource source, std::shared_ptr<std::vector<uint8_t>> data) override;
		void OnReceivedWeb (Tile tile, SatelliteSource source, std::shared_ptr<std::vector<uint8_t>> data) override;
	};
}


