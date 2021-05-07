//System Includes
#include <thread>

//External Includes
#include "../HandyImGuiInclude.hpp"
#include "../../../handycpp/Extended/stb_image.h"

//Project Includes
#include "SatelliteCacheMaster.hpp"
//#include "../Context.hpp"

namespace Maps {

//This instantiates the s_instance static field, but the constructor will not be called until Init is run
SatelliteCacheMaster * SatelliteCacheMaster::s_instance = nullptr;

SatelliteCacheMaster::SatelliteCacheMaster(Journal & LogRef)
	: Log(LogRef),
	  m_garbageCollectionThreadAbort(false)
{
	m_cacheMem     = new CacheMem(Log);
	m_cacheFile    = new CacheFile(this, Log);
	m_webRetriever = new WebRetriever(this, Log);
	m_garbageCollectionThread = std::thread(&SatelliteCacheMaster::GarbageCollectionThreadMain, this);
}

SatelliteCacheMaster::~SatelliteCacheMaster() {
	m_garbageCollectionThreadAbort = true;
	m_garbageCollectionThread.join();
	Handy::SafeDelete(m_webRetriever);
	Handy::SafeDelete(m_cacheFile);
	Handy::SafeDelete(m_cacheMem);
}

ImTextureID SatelliteCacheMaster::TryGetTouchReq(Tile tile) {
	if (tile.Zoom > MaxZoom || tile.Zoom < 0)
		return nullptr;
	
	ImTextureID res = m_cacheMem->Get(tile, m_source);
	if (res != nullptr)
		return res;
	else {
		//m_webRetriever->RetrieveAsync(tile, m_source);
		m_cacheFile->RetrieveAsync(tile, m_source);
		return nullptr;
	}
}

void SatelliteCacheMaster::PurgeAll() {
	m_webRetriever->WaitFinish();
	m_cacheFile->WaitFinish();

	m_cacheFile->PurgeAll();
	m_cacheMem->PurgeAll();
}

void SatelliteCacheMaster::GarbageCollectionThreadMain(void) {
	while (true) {
		m_cacheMem->PurgeExpired(0.25);
		
		//Sleep, but check abort flag periodically.
		for (int n = 0; n < 2; n++) {
			if (m_garbageCollectionThreadAbort) return;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
}

void SatelliteCacheMaster::OnReceivedFile(Tile tile, SatelliteSource source, std::shared_ptr<std::vector<uint8_t>> data) {
	if (data == nullptr) {
		//The cache file did not have the file - pass on to the web retriever
		//Log.print("Cache file miss. Passing tile request to web retriever: " + tile.ToString());
		m_webRetriever->RetrieveAsync(tile, source);
	}
	else {
		//The cache file did have the file
		//Log.print("Cache file hit. Retrieval succeeded for tile: " + tile.ToString());
		int width, height, bpp;
		unsigned char * rgba = stbi_load_from_memory(&(*data)[0], (int)data->size(), &width, &height, &bpp, 4);
		if (rgba != nullptr) {
			m_cacheMem->Add(tile, source, rgba, width, height);
			stbi_image_free(rgba);
		}
		else {
			Log.print("Failed to decode image data from cache for tile: " + tile.ToString());
			m_cacheFile->BlockRemove(tile, source);
			m_webRetriever->RetrieveAsync(tile, source);
		}
	}
}

void SatelliteCacheMaster::OnReceivedWeb(Tile tile, SatelliteSource source, std::shared_ptr<std::vector<uint8_t>> data) {
	int width, height, bpp;
	unsigned char * rgba = stbi_load_from_memory(&(*data)[0], (int)data->size(), &width, &height, &bpp, 4);
	if (rgba != nullptr) {
		m_cacheMem->Add(tile, source, rgba, width, height);
		m_cacheFile->BlockAdd(tile, source, *data); //Only save to cache file if we were able to decode the image properly
		stbi_image_free(rgba);
	}
	else
		Log.print("Failed to decode image data from web for tile: " + tile.ToString());
}

}


