//Class for managing a cache file for satellite image tiles. Support asynchronous and synchronous item retrieval
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 

//System Includes
#include <thread>
#include <iostream>
#include <system_error>

//External Includes
#include "../../../handycpp/Handy.hpp"

//Project Includes
#include "FRFTileStore.hpp"
#include "../SimpleKVStore.hpp"

namespace Maps {

FRFTileStore::FRFTileStore(IFRFFileReceiver * receiver, Journal & LogRef) : Log(LogRef), m_receiver(receiver), m_threads(4) {
	std::filesystem::path KVStorePath = Handy::Paths::ThisExecutableDirectory() / "Recon_FRFTileStore";
	
	m_file = new SimpleKVStore(KVStorePath, Log);
	if (! m_file->IsOpen())
		Log.print("Error: Failed to open FRF Tile Store: " + KVStorePath.string());
}

FRFTileStore::~FRFTileStore() {
	WaitFinish();
	Handy::SafeDelete(m_file);
}

void FRFTileStore::PurgeAll() {
	m_file->Clear();
}

//Cache file size inspection
uint64_t FRFTileStore::GetNumItems()       { return m_file->GetNumItems();       }
uint64_t FRFTileStore::GetNumBytes()       { return m_file->GetNumBytes();       }
uint64_t FRFTileStore::GetNumBytesOnDisk() { return m_file->GetNumBytesOnDisk(); }

std::shared_ptr<std::vector<uint8_t>> FRFTileStore::BlockTryGet(Tile tile) {
	std::string Identifier = tile.ToString();
	std::vector<uint8_t> * v = new std::vector<uint8_t>();
	if (m_file->Get(Identifier, *v))
		return std::shared_ptr<std::vector<uint8_t>>(v);
	else {
		delete v;
		return nullptr;
	}
}

void FRFTileStore::BlockAdd(Tile tile, FRFImage const * Data) {
	std::string Identifier = tile.ToString();
	std::vector<uint8_t> buffer;
	Data->SaveToRAM(buffer);
	if (! m_file->Put(Identifier, buffer))
		Log.print("Adding tile to cache failed - Tile: " + tile.ToString());
}

void FRFTileStore::BlockRemove(Tile tile) {
	std::string Identifier = tile.ToString();
	m_file->Delete(Identifier);
}

void FRFTileStore::WaitFinish() {
	m_threads.Wait();
}

void FRFTileStore::RetrieveAsync(Tile tile) {
	bool begin = false;
	{
		std::scoped_lock lock(m_requests_mtx);
		if ((! Handy::Contains(m_requests, tile)) && (m_requests.size() < MaxOutstandingRequests)) {
			m_requests.insert(tile);
			begin = true;
		}
	}
	
	if (begin) {
		m_threads.AddJobToFront([this,tile]() {
			try {
				auto data = BlockTryGet(tile);
				
				//Decode the data in the buffer into an FRFImage Object. ImagePtr will be nullptr on failure
				FRFImage * ImagePtr = nullptr;
				if (data != nullptr) {
					ImagePtr = new FRFImage;
					ImagePtr->LoadFromRAM(*data); //Will be an empty image on failure
					if ((ImagePtr->Width() == 0U) || (ImagePtr->Height() == 0U)) {
						delete ImagePtr;
						ImagePtr = nullptr;
					}
				}
				
				//Call OnReceivedFRFTile even if we failed to retrieve or decode the file. This serves to notify of failed attempt.
				m_receiver->OnReceivedFRFTile(tile, ImagePtr);
			} catch(...) {
				Log.print("Error in FRFTileStore::RetrieveAsync: Retrieval failed for tile: " + tile.ToString());
			}
			std::scoped_lock lock(m_requests_mtx);
			m_requests.erase(tile);
		});
	}
}
}


