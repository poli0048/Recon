//This module provides an interface for accessing GIS data stored in an FRF KV store. It can provide vis tiles of GIS
//data for display with ImGui and it has utilities to retrieve and edit raw data at any point on Earth. Essentially,
//the DataTileProvider allows other modules to pretend that we have a raster canvas covering the entire planet in a
//Normalized Mercator (NM) projection at a certain resolution level. We can sample and edit this canvas by providing the
//NM coordinates of the point we want. Internally, it manages a global pyramid of FRF tiles. Edits occur at a fixed resolution
//level and lower-resolution tiles are updated based on the contents of the edit level. Tiles are only created when touched
//by an edit, and they are destroyed automatically when they reach a state equivalent to a default empty tile. Tiles are only
//held in memory when they are in use (or have been used within a certain number of seconds). When a tile is not held in memory
//it is saved back to an on-disk KV store.
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 

//System Includes
#include <algorithm>

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "DataTileProvider.hpp"
#include "DataTileVizEvaluator.hpp"
#include "MapUtils.hpp"
#include "../Utilities.hpp"

namespace Maps {
	//Instantiate static fields
	DataTileProvider * DataTileProvider::s_instance = nullptr;
	
	// ******************************************************************************************************************************************
	// **************************************************   DataTileCacheItem Implementation   **************************************************
	// ******************************************************************************************************************************************
	//Note: We only save FRF items back to the store when destroying the cache item, which should generally be when the garbage collector removes
	//an item from the cache or on program close. This is to avoid hammering the store file since each write will increase the file size until
	//the store is closed and densified.
	DataTileCacheItem::~DataTileCacheItem() {
		//If edited, save FRF tile back to the data store
		if ((m_FRFEdited) && (m_FRFTile != nullptr)) {
			if (FRFImageIsTrivial()) {
				//std::cerr << "Removing tile " << m_tileKey.ToString() << " from store because contents are trivial.\r\n";
				m_FRFFileStore->BlockRemove(m_tileKey); //Remove from tile store
			}
			else {
				//std::cerr << "Updating tile " << m_tileKey.ToString() << " in store because contents are non-trivial.\r\n";
				m_FRFFileStore->BlockAdd(m_tileKey, m_FRFTile.get()); //Save updated non-empty tile
			}
		}
		
		//If the viz texture is valid, destroy it
		if (m_vizValid)
			ImGuiApp::Instance().DeleteImageAsync(m_TextureID);
	}
	
	//Returns true if image has the same contents as a freshly-initialized tile. m_FRFTile should already be confirmed valid (not null)
	bool DataTileCacheItem::FRFImageIsTrivial(void) {
		static std::unique_ptr<FRFImage> EmptyFRFTile;
		if (EmptyFRFTile == nullptr) {
			EmptyFRFTile.reset(new FRFImage);
			InitializeFRFTileContents(EmptyFRFTile.get());
		}
		
		if ((m_FRFTile->Width() != EmptyFRFTile->Width()) || (m_FRFTile->Height() != EmptyFRFTile->Height()))
			return false;
		if (m_FRFTile->NumberOfLayers() != EmptyFRFTile->NumberOfLayers())
			return false;
		for (uint16_t LayerIndex = 0U; LayerIndex < m_FRFTile->NumberOfLayers(); LayerIndex++) {
			if (m_FRFTile->Layer(LayerIndex)->Name != EmptyFRFTile->Layer(LayerIndex)->Name)
				return false;
			if (m_FRFTile->Layer(LayerIndex)->Data != EmptyFRFTile->Layer(LayerIndex)->Data)
				return false;
		}
		return true;
	}
	
	void DataTileCacheItem::InitializeFRFTileContents(FRFImage * FRFTile) {
		FRFTile->SetWidth((uint16_t) 256);
		FRFTile->SetHeight((uint16_t) 256);
		
		FRFLayer * newLayer = FRFTile->AddLayer();
		newLayer->Name = std::string("MSA");
		newLayer->Description = std::string("Minimum Safe WGS84 Altitude For Flight");
		newLayer->UnitsCode = (int32_t) 1;    //Meters
		newLayer->SetTypeCode((uint8_t) 70U); //Float32
		newLayer->AllocateStorage();
		for (int row = 0; row < 256; row++)
			for (int col = 0; col < 256; col++)
				newLayer->SetValue(row, col, std::nan(""));
		
		//Avoidance Zone layer: 0 = Not in zone, 1= In zone. No validity needed since invalid can be treated the same as not in zone.
		newLayer = FRFTile->AddLayer();
		newLayer->Name = std::string("Avoidance Zones");
		newLayer->Description = std::string("Minimize time spent flying in these areas");
		newLayer->UnitsCode = (int32_t) -1;   //No Units
		newLayer->SetTypeCode((uint8_t) 1U);  //1-bit unsigned int
		newLayer->AllocateStorage();
		for (int row = 0; row < 256; row++)
			for (int col = 0; col < 256; col++)
				newLayer->SetValue(row, col, std::nan(""));
		
		//Safe Landing Zone layer: 0 = Not in zone, 1= In zone. No validity needed since invalid can be treated the same as not in zone.
		newLayer = FRFTile->AddLayer();
		newLayer->Name = std::string("Safe Landing Zones");
		newLayer->Description = std::string("Safe to land vehicles anywhere in these areas");
		newLayer->UnitsCode = (int32_t) -1;   //No Units
		newLayer->SetTypeCode((uint8_t) 1U);  //1-bit unsigned int
		newLayer->AllocateStorage();
		for (int row = 0; row < 256; row++)
			for (int col = 0; col < 256; col++)
				newLayer->SetValue(row, col, std::nan(""));
		
		//FRF requires a default visualization. Show colormapped avoidance zones
		FRFVisualizationColormap * defaultViz = FRFTile->AddVisualizationColormap();
		defaultViz->Name = std::string("Avoidance Zones");
		defaultViz->Description = std::string("White = Not in Zone, Red = In Zone");
		defaultViz->LayerIndex = (uint16_t) 1U;
		defaultViz->SetPoints.push_back(std::make_tuple(0.0, 1.0, 1.0, 1.0));
		defaultViz->SetPoints.push_back(std::make_tuple(1.0, 1.0, 0.0, 0.0));
	}
	
	void DataTileCacheItem::UpdateFRFTileTimeTag(FRFImage * FRFTile) {
		//We use the GeoTag GPS time to record the time of last edit so we can navigate conflicts when merging tile stores.
		//This is not ideal since it will generally be based on the system clock, which may be wrong by an arbitrary amount, but it's the best we can do here.
		FRFGeoTag geoTag;
		geoTag.P_ECEF << std::nan(""), std::nan(""), std::nan("");
		geoTag.C_ECEF_Cam << std::nan(""), std::nan(""), std::nan(""),
		                     std::nan(""), std::nan(""), std::nan(""),
		                     std::nan(""), std::nan(""), std::nan("");
		//Use std::chrono::gps_clock if we switch on C++2020. Before C++2020, system_clock doesn't guarantee use of the Unix T0 epoch.
		const auto now = std::chrono::system_clock::now();
		uint64_t secondsSinceUnixT0Epoch = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
		uint64_t secondsSinceGPST0Epoch = secondsSinceUnixT0Epoch - (uint64_t) 315964800;
		uint64_t GPSWeek = secondsSinceGPST0Epoch / (uint64_t) 604800;
		uint64_t GPSTOW = secondsSinceGPST0Epoch - GPSWeek * ((uint64_t) 604800);
		geoTag.GPST_Week = (uint32_t) GPSWeek;
		geoTag.GPST_TOW = (double) GPSTOW;
		FRFTile->SetGeoTag(geoTag);
	}
	
	
	// ******************************************************************************************************************************************
	// ***************************************************   PaintActionItem Implementation   ***************************************************
	// ******************************************************************************************************************************************
	//Returns true if the given item is essentially the same as this item (i.e. close enough that if we can skip one of them)
	bool PaintActionItem::IsEssentiallyIdentical(PaintActionItem const & OtherItem) {
		if (this->m_shape != OtherItem.m_shape)
			return false;
		if (this->m_layer != OtherItem.m_layer)
			return false;
		if (this->m_Value != OtherItem.m_Value)
			return false;
		
		//If we get here, the items have the same shape, layer, and value
		if (this->m_shape == 0) {
			//Comparing circles
			if (this->m_Radius_meters != OtherItem.m_Radius_meters)
				return false;
			
			//See if the centers are within a quarter of the radius
			double epsilon = MetersToNMUnits(0.25*this->m_Radius_meters, this->m_Center_NM(1));
			return ((this->m_Center_NM - OtherItem.m_Center_NM).norm() < epsilon);
		}
		else {
			//Comparing rectangles
			if (this->m_LengthX != OtherItem.m_LengthX)
				return false;
			if (this->m_LengthY != OtherItem.m_LengthY)
				return false;
			if (this->m_Theta != OtherItem.m_Theta)
				return false;
				
			//See if the centers are within an eighth of the shorter edge length
			double epsilon = MetersToNMUnits(0.125*std::min(this->m_LengthX, this->m_LengthY), this->m_Center_NM(1));
			return ((this->m_Center_NM - OtherItem.m_Center_NM).norm() < epsilon);
		}
	}
	
	//Get Normalized-Mercator Axis-Aligned Bounding Box for region impacted by action (XMin, XMax, YMin, YMax).
	Eigen::Vector4d PaintActionItem::GetNM_AABB(void) {
		if (m_shape == 0) {
			//Circle
			double R_NM_Units = MetersToNMUnits(1.01*m_Radius_meters, m_Center_NM(1));
			return Eigen::Vector4d(m_Center_NM(0) - R_NM_Units, m_Center_NM(0) + R_NM_Units, m_Center_NM(1) - R_NM_Units, m_Center_NM(1) + R_NM_Units);
		}
		else {
			//Rectangle
			Eigen::Vector2d P1_NM, P2_NM, P3_NM, P4_NM;
			GetCorners(P1_NM, P2_NM, P3_NM, P4_NM);
			double xMin = std::min(std::min(P1_NM(0), P2_NM(0)), std::min(P3_NM(0), P4_NM(0)));
			double xMax = std::max(std::max(P1_NM(0), P2_NM(0)), std::max(P3_NM(0), P4_NM(0)));
			double yMin = std::min(std::min(P1_NM(1), P2_NM(1)), std::min(P3_NM(1), P4_NM(1)));
			double yMax = std::max(std::max(P1_NM(1), P2_NM(1)), std::max(P3_NM(1), P4_NM(1)));
			double xEps = 0.01*(xMax - xMin);
			double yEps = 0.01*(yMax - yMin);
			return Eigen::Vector4d(xMin - xEps, xMax + xEps, yMin - yEps, yMax + yEps);
		}
	}
	
	//Only for rectangle actions
	void PaintActionItem::GetCorners(Eigen::Vector2d & P1_NM, Eigen::Vector2d & P2_NM, Eigen::Vector2d & P3_NM, Eigen::Vector2d & P4_NM) {
		double lengthX_NM = MetersToNMUnits(m_LengthX, m_Center_NM(1));
		double lengthY_NM = MetersToNMUnits(m_LengthY, m_Center_NM(1));
		
		P1_NM = m_Center_NM + Eigen::Vector2d(-0.5*lengthX_NM, -0.5*lengthY_NM);
		P2_NM = m_Center_NM + Eigen::Vector2d( 0.5*lengthX_NM, -0.5*lengthY_NM);
		P3_NM = m_Center_NM + Eigen::Vector2d( 0.5*lengthX_NM,  0.5*lengthY_NM);
		P4_NM = m_Center_NM + Eigen::Vector2d(-0.5*lengthX_NM,  0.5*lengthY_NM);
		
		//Rotate all vertices clockwise about m_Center_NM
		Eigen::Matrix2d R_CW;
		R_CW << cos(-m_Theta), -sin(-m_Theta),
			   sin(-m_Theta),  cos(-m_Theta);
		P1_NM = R_CW * (P1_NM - m_Center_NM) + m_Center_NM;
		P2_NM = R_CW * (P2_NM - m_Center_NM) + m_Center_NM;
		P3_NM = R_CW * (P3_NM - m_Center_NM) + m_Center_NM;
		P4_NM = R_CW * (P4_NM - m_Center_NM) + m_Center_NM;
	}
	
	
	// ******************************************************************************************************************************************
	// **************************************************   DataTileProvider Implementation   ***************************************************
	// ******************************************************************************************************************************************
	DataTileProvider::DataTileProvider(Journal & LogRef) :
		Log(LogRef),
		m_FRFFileStore(new FRFTileStore(this, LogRef)),
		m_abort(false),
		m_threads_VisEval(NumberOfVizEvaluationThreads) {
		m_garbageCollectionThread = std::thread(&DataTileProvider::GarbageCollectionThreadMain, this);
		m_DataEditThread = std::thread(&DataTileProvider::DataEditThreadMain, this);
	}
	
	DataTileProvider::~DataTileProvider() {
		m_abort = true;
		m_garbageCollectionThread.join();
		m_DataEditThread.join();
		PurgeAllTiles();
		m_threads_VisEval.Wait();
		Handy::SafeDelete(m_FRFFileStore);
	}
	
	//Touch an FRF tile if it is loaded. If it isn't loaded - start an asynchronous load operation on it.
	//We assume that a lock is already held on m_cache_mtx
	void DataTileProvider::TouchLoadFRFTile(Tile Key) {
		if (m_cache.count(Key) > 0U)
			m_cache.at(Key).m_lastTouch = std::chrono::steady_clock::now();
		else
			m_FRFFileStore->RetrieveAsync(Key);
	}

	//Retrieve the given visualization tile. Load the source FRF tile into the memory cache if necessary.
	//If a visualization tile is already loaded with the same tile key as the argument, update the existing
	//texture instead of creating a new texture. If a visualization tile is loaded with the same tile key
	//as the argument, we return the texture ID of that visualization tile, even if the visualization tile key
	//is different. This is so we have something to display while visualization tiles are being updated in the background (prevents screen blanking).
	ImTextureID DataTileProvider::TryGetLoadUpdate_VizTile(VizualizationTileKey Key) {
		std::scoped_lock cacheLock(m_cache_mtx); //Lock the cache for the full function
		TouchLoadFRFTile(Key.tile); //Mark this cache item as in use and start an asynchronous load of the FRF resource if necessary
		bool FRFTileInCache = (m_cache.count(Key.tile) > 0U);
		bool requestedTextureInCache = false;
		if (FRFTileInCache) {
			DataTileCacheItem & cacheItem(m_cache.at(Key.tile));
			requestedTextureInCache = ((cacheItem.m_vizValid) && (cacheItem.m_VizKey == Key) &&
			                          ((!cacheItem.m_FRFEdited) || (cacheItem.m_VizEvalTime > cacheItem.m_LastEditTime)));
		}
		
		//If the requested texture doesn't exist, try to queue up a viz eval job for it
		if ((FRFTileInCache) && (! requestedTextureInCache)) {
			std::scoped_lock currentJobsLock(m_mtx_TilesWithcurrentVizEvalJobs);
			if ((m_TilesWithcurrentVizEvalJobs.count(Key.tile) == 0U) && (m_TilesWithcurrentVizEvalJobs.size() < MaxNumberOfJobsInThreadPool)) {
				//We have no jobs in queue or in process right now for this tile and there is room in the queue. We can queue up the eval job.
				//Since we locked the cache at the start of the function the item can't be garbage collected between our checking and now. We add
				//the job to m_TilesWithcurrentVizEvalJobs so the garbage collector knows that it can't touch this item for the time being.
				m_TilesWithcurrentVizEvalJobs.insert(Key.tile);
				FRFImage * SourceFRFImagePtr = m_cache.at(Key.tile).m_FRFTile.get();
				m_threads_VisEval.AddJob([this, SourceFRFImagePtr, Key]() {
					//Evaluate the vizualization tile
					ImTextureID tex = EvaluateVisualizationAndLoadIntoGPUMem(SourceFRFImagePtr, Key);
					if (tex == nullptr)
						Log.print("Error: Evaluation of viz tile failed.");
					
					//Lock the cache (to update the item) and the job set (to remove this job)
					std::scoped_lock lock(m_cache_mtx, m_mtx_TilesWithcurrentVizEvalJobs);
					
					DataTileCacheItem & cacheItem(m_cache.at(Key.tile));
					
					//If we are replacing an existing texture, perform a delayed deletion of the old texture
					if (cacheItem.m_vizValid)
						ImGuiApp::Instance().DeleteImageAsyncWithDelay(cacheItem.m_TextureID, 0.5);
					cacheItem.m_vizValid = true;
					cacheItem.m_VizKey = Key;
					cacheItem.m_TextureID = tex;
					cacheItem.m_VizEvalTime = std::chrono::steady_clock::now();
					
					//Remove this job from the set of current viz evaluation jobs
					m_TilesWithcurrentVizEvalJobs.erase(Key.tile);
				});
			}
		}
		
		//If we have anything valid for this tile return it. Otherwise return nullptr
		if ((FRFTileInCache) && (m_cache.at(Key.tile).m_vizValid))
			return m_cache.at(Key.tile).m_TextureID;
		else
			return nullptr;
	}
	
	void DataTileProvider::GarbageCollectionThreadMain(void) {
		while (! m_abort) {
			{
				std::scoped_lock lock(m_cache_mtx, m_mtx_TilesWithcurrentVizEvalJobs);
				TimePoint now = std::chrono::steady_clock::now();
				std::vector<Tile> cacheItemsToDestroy;
				cacheItemsToDestroy.reserve(64U);
				for (auto & kv : m_cache) {
					auto ageSeconds = std::chrono::duration_cast<std::chrono::seconds>(now - kv.second.m_lastTouch);
					if ((ageSeconds > ExpirationTimeSeconds) && (m_TilesWithcurrentVizEvalJobs.count(kv.first) == 0U)) {
						//The tile has not been used in a long time and we don't have a job queued up that needs it.
						cacheItemsToDestroy.push_back(kv.first);
					}
				}
				for (Tile tile : cacheItemsToDestroy)
					m_cache.erase(tile);
			}
			
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	
	void DataTileProvider::DataEditThreadMain(void) {
		std::unordered_set<Tile> modifiedTilesAtEditLevel; //When editing tiles, add them to this set. After low-res derived tiles are updated, remove them.
		TimePoint timeOfLastLowResUpdate = std::chrono::steady_clock::now();
		while (! m_abort) {
			bool workToDo = false;
			PaintActionItem workItem;
			{
				std::scoped_lock lock(m_editStream_mtx);
				if (! m_actionQueue.empty()) {
					//Grab first item from queue and discard subsequent items until we find one that is sufficiently distinct from it
					workToDo = true;
					workItem = m_actionQueue.front();
					m_actionQueue.pop_front();
					while (! m_actionQueue.empty()) {
						if (workItem.IsEssentiallyIdentical(m_actionQueue.front())) {
							//std::cerr << "Discarding essentially identical paint action.\r\n";
							m_actionQueue.pop_front();
						}
						else
							break;
					}
				}
			}
			
			if (workToDo) {
				//Identify the tiles that are needed to execute this edit
				Eigen::Vector4d AABB_NM = workItem.GetNM_AABB();
				Eigen::Vector2d LL_Corner_NM(AABB_NM(0), AABB_NM(2));
				Eigen::Vector2d UR_Corner_NM(AABB_NM(1), AABB_NM(3));
				
				std::tuple<int32_t, int32_t> tileCoordsA = getCoordsOfTileContainingPoint(LL_Corner_NM, TileEditZoomLevel);
				std::tuple<int32_t, int32_t> tileCoordsB = getCoordsOfTileContainingPoint(UR_Corner_NM, TileEditZoomLevel);
				int32_t tileMinCol = std::min(std::get<0>(tileCoordsA), std::get<0>(tileCoordsB));
				int32_t tileMaxCol = std::max(std::get<0>(tileCoordsA), std::get<0>(tileCoordsB));
				int32_t tileMinRow = std::min(std::get<1>(tileCoordsA), std::get<1>(tileCoordsB));
				int32_t tileMaxRow = std::max(std::get<1>(tileCoordsA), std::get<1>(tileCoordsB));
				std::vector<Tile> impactedTiles;
				for (int32_t tileCol = tileMinCol; tileCol <= tileMaxCol; tileCol++) {
					for (int32_t tileRow = tileMinRow; tileRow <= tileMaxRow; tileRow++)
						impactedTiles.emplace_back(int(tileCol), int(tileRow), int(TileEditZoomLevel));
				}
				
				//Initiate a load on all impacted tiles and wait until they are all loaded
				TouchLoadFRFTilesAndWait(impactedTiles);
				
				//For each impacted tile, execute the edit action, update the last edit time tag, and add the tile to modifiedTilesAtEditLevel
				{
					std::scoped_lock cacheLock(m_cache_mtx); //Lock the cache
					if (! AllTilesPresent(impactedTiles))
						Log.print("Internal Error - needed tile removed from cache before edit could take place.");
					else {
						for (Tile tile : impactedTiles) {
							bool tileModified = false;
							if (workItem.m_shape == 0)
								tileModified = ExecuteEditActionOnTile_Circle(workItem, tile, AABB_NM);
							else
								tileModified = ExecuteEditActionOnTile_Rectangle(workItem, tile, AABB_NM);
							if (tileModified) {
								m_cache.at(tile).m_FRFEdited = true;
								m_cache.at(tile).m_LastEditTime = std::chrono::steady_clock::now();
								DataTileCacheItem::UpdateFRFTileTimeTag(m_cache.at(tile).m_FRFTile.get());
								modifiedTilesAtEditLevel.insert(tile);
							}
						}
					}
				}
			}
			
			if (std::chrono::steady_clock::now() - timeOfLastLowResUpdate > LowResUpdatePeriodSeconds) {
				if (! modifiedTilesAtEditLevel.empty()) {
					UpdateLowerResTiles(modifiedTilesAtEditLevel);
					modifiedTilesAtEditLevel.clear();
				}
				timeOfLastLowResUpdate = std::chrono::steady_clock::now();
			}
			
			bool actionQueueEmpty = false;
			{
				std::scoped_lock lock(m_editStream_mtx);
				actionQueueEmpty = m_actionQueue.empty();
			}
			if (actionQueueEmpty)
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		
		//Do one final low-res update on abort if there is any work left to be done
		if (! modifiedTilesAtEditLevel.empty())
			UpdateLowerResTiles(modifiedTilesAtEditLevel);
	}
	
	//A lock should already be held on m_cache_mtx
	bool DataTileProvider::AllTilesPresent(std::vector<Tile> const & Tiles) {
		for (Tile tile : Tiles) {
			if (m_cache.count(tile) == 0U)
				return false;
		}
		return true;
	}
	
	//Queue up the given tiles for asynchronous loading and don't return until they are all loaded
	void DataTileProvider::TouchLoadFRFTilesAndWait(std::vector<Tile> const & Tiles) {
		while (true) {
			bool allTilesPresent = true;
			{
				std::scoped_lock cacheLock(m_cache_mtx); //Lock the cache
				for (Tile tile : Tiles)
					TouchLoadFRFTile(tile);
				allTilesPresent = AllTilesPresent(Tiles);
			}
			if (allTilesPresent)
				break;
			else
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	
	//Compute the average of 4 adjacent pixels (in a square) from the source FRF image and put the result in the given location in the destination image.
	//The average happens layer-by-layer and assumes identical layer ordering (for speed).
	static void BlockAverageFRF(FRFImage * SourceImage, FRFImage * DestImage, int SourceFirstRow, int SourceFirstCol, int DestRow, int DestCol) {
		for (uint16_t layerIndex = 0U; layerIndex < SourceImage->NumberOfLayers(); layerIndex++) {
			double val00 = SourceImage->Layer(layerIndex)->GetValue(uint32_t(SourceFirstRow),  uint32_t(SourceFirstCol));
			double val01 = SourceImage->Layer(layerIndex)->GetValue(uint32_t(SourceFirstRow),  uint32_t(SourceFirstCol+1));
			double val10 = SourceImage->Layer(layerIndex)->GetValue(uint32_t(SourceFirstRow+1), uint32_t(SourceFirstCol));
			double val11 = SourceImage->Layer(layerIndex)->GetValue(uint32_t(SourceFirstRow+1), uint32_t(SourceFirstCol+1));
			int tally = 0;
			double sum = 0.0;
			if (! std::isnan(val00)) {
				sum += val00;
				tally++;
			}
			if (! std::isnan(val01)) {
				sum += val01;
				tally++;
			}
			if (! std::isnan(val10)) {
				sum += val10;
				tally++;
			}
			if (! std::isnan(val11)) {
				sum += val11;
				tally++;
			}
			double destVal = (tally == 0) ? std::nan("") : sum / double(tally);
			DestImage->Layer(layerIndex)->SetValue(uint32_t(DestRow), uint32_t(DestCol), destVal);
		}
	}
	
	//We could do some sophistocated stuff here like trying to figure out on which zoom level a given edit will result in a change of size less than
	//a certain number of pixels. However, to keep it simple we will just go up a fixed number of levels from the edit level. Note that if we go
	//up 8 zoom levels an entire tile at the edit level will ocupy a single pixel, so this seems like overkill.
	void DataTileProvider::UpdateLowerResTiles(std::unordered_set<Tile> const & EditedTiles) {
		std::cerr << "Updating low res tiles. Edited tiles at edit level: " << EditedTiles.size() << "\r\n";
		
		int numLevels = int(TileEditZoomLevel - DataTileMinZoomLevel); //The number of zoom levels to populate (below the edit level)
		//int numLevels = 4; //The number of zoom levels to populate (below the edit level)
		
		//Detirmine which tiles we need to update and build a vector of all tiles needed in memory
		std::vector<std::unordered_set<Tile>> RelaventTiles;
		RelaventTiles.reserve(numLevels + 1);
		RelaventTiles.push_back(EditedTiles);
		std::vector<Tile> TilesRequired(EditedTiles.cbegin(), EditedTiles.cend());
		for (int counter = 0; counter < numLevels; counter++) {
			RelaventTiles.emplace_back();
			for (Tile tile : RelaventTiles[RelaventTiles.size() - 2U]) {
				if (tile.Zoom > int32_t(0))
					RelaventTiles.back().insert(tile.GetParentTile());
			}
			TilesRequired.insert(TilesRequired.end(), RelaventTiles.back().begin(), RelaventTiles.back().end());
		}
		
		//Initiate a load on all required tiles and wait until they are all loaded - note that we need the edited tiles loaded even though we don't modify them
		TouchLoadFRFTilesAndWait(TilesRequired);
		
		{
			std::scoped_lock cacheLock(m_cache_mtx); //Lock the cache
			if (! AllTilesPresent(TilesRequired))
				Log.print("Internal Error - needed tile removed from cache before low-res update could take place.");
			else {
				//Starting with the highest zoom level and working down, update tiles based on the next highest level.
				for (size_t batch = 1U; batch < RelaventTiles.size(); batch++) {
					for (Tile lowResTile : RelaventTiles[batch]) {
						//We don't want to just fill in every pixel because in general we won't have (and don't need)
						//all of the child tiles in memory... we just need to update the portion of the tile impacted by possible
						//changes in one of the edited tiles. So we do it in quarters and check for whether the corresponding
						//child tiles for a given quarter is in the RelaventTiles vector on the previous level.
						FRFImage * DestTile = m_cache.at(lowResTile).m_FRFTile.get();
						
						//Update UL Quadrant if needed
						Tile ULChild(lowResTile.Xi*2, lowResTile.Yi*2, lowResTile.Zoom + 1);
						if (RelaventTiles[batch - 1U].count(ULChild) > 0U) {
							//Update the UL Quadrant
							for (int row = 0; row < 128; row++) {
								for (int col = 0; col < 128; col++) {
									FRFImage * SourceTile = m_cache.at(ULChild).m_FRFTile.get();
									BlockAverageFRF(SourceTile, DestTile, 2*row, 2*col, row, col);
								}
							}
						}
						
						//Update UR Quadrant if needed
						Tile URChild(lowResTile.Xi*2 + 1, lowResTile.Yi*2, lowResTile.Zoom + 1);
						if (RelaventTiles[batch - 1U].count(URChild) > 0U) {
							//Update the UR Quadrant
							for (int row = 0; row < 128; row++) {
								for (int col = 0; col < 128; col++) {
									FRFImage * SourceTile = m_cache.at(URChild).m_FRFTile.get();
									BlockAverageFRF(SourceTile, DestTile, 2*row, 2*col, row, 128 + col);
								}
							}
						}
						
						//Update LL Quadrant if needed
						Tile LLChild(lowResTile.Xi*2, lowResTile.Yi*2 + 1, lowResTile.Zoom + 1);
						if (RelaventTiles[batch - 1U].count(LLChild) > 0U) {
							//Update the LL Quadrant
							for (int row = 0; row < 128; row++) {
								for (int col = 0; col < 128; col++) {
									FRFImage * SourceTile = m_cache.at(LLChild).m_FRFTile.get();
									BlockAverageFRF(SourceTile, DestTile, 2*row, 2*col, 128 + row, col);
								}
							}
						}
						
						//Update LR Quadrant if needed
						Tile LRChild(lowResTile.Xi*2 + 1, lowResTile.Yi*2 + 1, lowResTile.Zoom + 1);
						if (RelaventTiles[batch - 1U].count(LRChild) > 0U) {
							//Update the LR Quadrant
							for (int row = 0; row < 128; row++) {
								for (int col = 0; col < 128; col++) {
									FRFImage * SourceTile = m_cache.at(LRChild).m_FRFTile.get();
									BlockAverageFRF(SourceTile, DestTile, 2*row, 2*col, 128 + row, 128 + col);
								}
							}
						}
						
						//Mark the tile as edited and update timestamps
						m_cache.at(lowResTile).m_FRFEdited = true;
						m_cache.at(lowResTile).m_LastEditTime = std::chrono::steady_clock::now();
						DataTileCacheItem::UpdateFRFTileTimeTag(DestTile);
					}
				}
			}
		}
	}
	
	//Return index of the given layer in an FRF file. Returns -1 if not found.
	static int GetFRFLayerIndex(FRFImage const & FRF, DataLayer layer) {
		std::string layerName = DataLayerToString(layer);
		for (uint16_t LayerIndex = 0U; LayerIndex < FRF.NumberOfLayers(); LayerIndex++) {
			if (FRF.Layer(LayerIndex)->Name == layerName)
				return int(LayerIndex);
		}
		return -1;
	}
	
	//Returns true if any pixels were modified. A lock should already be held on m_cache_mtx and the tile should already be verified to be in-cache.
	bool DataTileProvider::ExecuteEditActionOnTile_Circle(PaintActionItem const & Action, Tile tile, Eigen::Vector4d const & AABB_NM) {
		//std::cerr << "Executing circle edit action on tile: " << tile.ToString() << "\r\n";
		FRFImage * FRFTile = m_cache.at(tile).m_FRFTile.get();
		int layerIndex = GetFRFLayerIndex(*FRFTile, Action.m_layer);
		if (layerIndex < 0) {
			std::cerr << "Internal Error: FRF Layer not found.\r\n";
			return false;
		}
		
		//Get the range of pixels in this tile that might be impacted by the edit action
		Eigen::Vector2d LL_Corner_NM(AABB_NM(0), AABB_NM(2));
		Eigen::Vector2d UR_Corner_NM(AABB_NM(1), AABB_NM(3));
		std::tuple<int, int> ColRowA = NMToTilePixel_int(int32_t(tile.Xi), int32_t(tile.Yi), int32_t(tile.Zoom), LL_Corner_NM, int32_t(256));
		std::tuple<int, int> ColRowB = NMToTilePixel_int(int32_t(tile.Xi), int32_t(tile.Yi), int32_t(tile.Zoom), UR_Corner_NM, int32_t(256));
		int minRow = std::min(std::get<1>(ColRowA), std::get<1>(ColRowB));
		int maxRow = std::max(std::get<1>(ColRowA), std::get<1>(ColRowB));
		int minCol = std::min(std::get<0>(ColRowA), std::get<0>(ColRowB));
		int maxCol = std::max(std::get<0>(ColRowA), std::get<0>(ColRowB));
		
		//For speed, convert the circle center and radius to pixels to avoid pixel -> NM conversion inside the loop. This isn't techniclly as accurate
		//as testing each pixel properly, but this needs to be pretty fast and this should be good enough since our edit regions are small.
		Eigen::Vector2d Center_TilePixelCoords = NMToTilePixel(int32_t(tile.Xi), int32_t(tile.Yi), int32_t(tile.Zoom), Action.m_Center_NM, int32_t(256));
		double radius_pixels = MetersToPixels(Action.m_Radius_meters, Action.m_Center_NM(1), double(tile.Zoom));
		
		//Iterate over the range of possibly impacted pixels and test each one. If impacted, edit the pixel
		bool editMade = false;
		for (int row = minRow; row <= maxRow; row++) {
			for (int col = minCol; col <= maxCol; col++) {
				Eigen::Vector2d pixelCoords(col, row);
				double r_pixels = (pixelCoords - Center_TilePixelCoords).norm();
				if (r_pixels <= radius_pixels) {
					//Edit this pixel
					FRFTile->Layer(layerIndex)->SetValue(uint32_t(row), uint32_t(col), Action.m_Value);
					editMade = true;
				}
			}
		}
		
		return editMade;
	}
	
	//Returns true if any pixels were modified. A lock should already be held on m_cache_mtx and the tile should already be verified to be in-cache.
	bool DataTileProvider::ExecuteEditActionOnTile_Rectangle(PaintActionItem const & Action, Tile tile, Eigen::Vector4d const & AABB_NM) {
		//std::cerr << "Executing rectangle edit action on tile: " << tile.ToString() << "\r\n";
		FRFImage * FRFTile = m_cache.at(tile).m_FRFTile.get();
		int layerIndex = GetFRFLayerIndex(*FRFTile, Action.m_layer);
		if (layerIndex < 0) {
			std::cerr << "Internal Error: FRF Layer not found.\r\n";
			return false;
		}
		
		//Get the range of pixels in this tile that might be impacted by the edit action
		Eigen::Vector2d LL_Corner_NM(AABB_NM(0), AABB_NM(2));
		Eigen::Vector2d UR_Corner_NM(AABB_NM(1), AABB_NM(3));
		std::tuple<int, int> ColRowA = NMToTilePixel_int(int32_t(tile.Xi), int32_t(tile.Yi), int32_t(tile.Zoom), LL_Corner_NM, int32_t(256));
		std::tuple<int, int> ColRowB = NMToTilePixel_int(int32_t(tile.Xi), int32_t(tile.Yi), int32_t(tile.Zoom), UR_Corner_NM, int32_t(256));
		int minRow = std::min(std::get<1>(ColRowA), std::get<1>(ColRowB));
		int maxRow = std::max(std::get<1>(ColRowA), std::get<1>(ColRowB));
		int minCol = std::min(std::get<0>(ColRowA), std::get<0>(ColRowB));
		int maxCol = std::max(std::get<0>(ColRowA), std::get<0>(ColRowB));
		
		//Compute matrix for rotating points counter-clockwise by theta about the origin
		Eigen::Matrix2d R_CCW;
		R_CCW << cos(Action.m_Theta), -sin(Action.m_Theta),
		         sin(Action.m_Theta),  cos(Action.m_Theta);
		
		//Compute rectangle dimensions in NM units
		double lengthX_NM = MetersToNMUnits(Action.m_LengthX, Action.m_Center_NM(1));
		double lengthY_NM = MetersToNMUnits(Action.m_LengthY, Action.m_Center_NM(1));
		
		//Iterate over the range of possibly impacted pixels and test each one. If impacted, edit the pixel
		bool editMade = false;
		for (int row = minRow; row <= maxRow; row++) {
			for (int col = minCol; col <= maxCol; col++) {
				Eigen::Vector2d pixelCenter_NM = TilePixelToNM(tile.Xi, tile.Yi, tile.Zoom, row, col, int32_t(256));
				Eigen::Vector2d v_NM = R_CCW*(pixelCenter_NM - Action.m_Center_NM);
				if ((fabs(v_NM(0)) <= lengthX_NM/2.0) && (fabs(v_NM(1)) <= lengthY_NM/2.0)) {
					//Edit this pixel
					FRFTile->Layer(layerIndex)->SetValue(uint32_t(row), uint32_t(col), Action.m_Value);
					editMade = true;
				}
			}
		}
		
		return editMade;
	}
	
	//Data access - get a value from a single layer at a single location. The retrieved data is passed back through reference argument "Value".
	//Will return True on success and false on failure. Note that access will fail if the needed tile isn't cached yet, but the needed tile
	//will be queued up for loading so later calls for the same value will eventually succeed.
	bool DataTileProvider::TryGetData(Eigen::Vector2d const & Position_NM, DataLayer layer, double & Value) {
		std::tuple<int32_t, int32_t> tileCoords = getCoordsOfTileContainingPoint(Position_NM, TileEditZoomLevel);
		Tile key(int(std::get<0>(tileCoords)), int(std::get<1>(tileCoords)), int(TileEditZoomLevel));
		
		std::scoped_lock cacheLock(m_cache_mtx); //Lock the cache
		TouchLoadFRFTile(key); //Mark this cache item as in use and start an asynchronous load of the FRF resource if necessary
		bool FRFTileInCache = (m_cache.count(key) > 0U);
		
		if (FRFTileInCache) {
			//Get FRF tile and pixel coords of location in tile
			FRFImage * FRFTile = m_cache.at(key).m_FRFTile.get();
			std::tuple<int, int> ColRow = NMToTilePixel_int(std::get<0>(tileCoords), std::get<1>(tileCoords), TileEditZoomLevel, Position_NM, int32_t(256));
			
			//Get layer index
			std::string layerName = DataLayerToString(layer);
			int layerIndex = -1;
			for (uint16_t LayerIndex = 0U; LayerIndex < FRFTile->NumberOfLayers(); LayerIndex++) {
				if (FRFTile->Layer(LayerIndex)->Name == layerName) {
					layerIndex = (int) LayerIndex;
					break;
				}
			}
			if (layerIndex < 0)
				return false;
			
			Value = FRFTile->Layer(layerIndex)->GetValue(uint32_t(std::get<1>(ColRow)), uint32_t(std::get<0>(ColRow)));
			return true;
		}
		else
			return false;
	}
	
	//Same as TryGetData, but will block until the requested data is available (up to a given number of seconds, after which it will return NaN)
	double DataTileProvider::GetData(Eigen::Vector2d const & Position_NM, DataLayer layer, double Timeout) {
		double value;
		std::chrono::time_point<std::chrono::steady_clock> startTime = std::chrono::steady_clock::now();
		while (! TryGetData(Position_NM, layer, value)) {
			if (SecondsElapsed(startTime, std::chrono::steady_clock::now()) < Timeout)
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			else
				return std::nan("");
		}
		return value;
	}

	void DataTileProvider::PurgeAllTiles() {
		m_threads_VisEval.Wait();
		std::scoped_lock lock(m_cache_mtx, m_mtx_TilesWithcurrentVizEvalJobs);
		std::vector<Tile> cacheItemsToDestroy;
		cacheItemsToDestroy.reserve(m_cache.size());
		for (auto & kv : m_cache) {
			if (m_TilesWithcurrentVizEvalJobs.count(kv.first) > 0U)
				Log.print("Warning in PurgeAllTiles(): Skipping item because a viz eval job is underway for it.");
			else
				cacheItemsToDestroy.push_back(kv.first);
		}
		for (Tile tile : cacheItemsToDestroy)
			m_cache.erase(tile);
	}

	//Called after an asynchronous FRFImage retrieval - We are responsible for deletion of Data.
	void DataTileProvider::OnReceivedFRFTile(Tile TileKey, FRFImage * Data) {
		std::scoped_lock lock(m_cache_mtx);
		//Only put received data in the cache if it's not in the cache already. This is important. If an item is in the cache, that is the
		//master copy since it may contain edited data. We can't overwrite that if we get a duplicate load request.
		if (m_cache.count(TileKey) == 0U) {
			//Item is not already in the cache
			if (Data != nullptr) {
				//Received valid data - put it in the cache
				m_cache.emplace(TileKey, m_FRFFileStore);
				DataTileCacheItem & item(m_cache.at(TileKey));
				item.m_tileKey = TileKey;
				item.m_FRFTile.reset(Data);
				item.m_lastTouch = std::chrono::steady_clock::now();
			}
			else {
				//Did not receive valid data - tile doesn't exist or couldn't be decoded (corrupt). Create new empty tile in the cache
				m_cache.emplace(TileKey, m_FRFFileStore);
				DataTileCacheItem & item(m_cache.at(TileKey));
				item.m_tileKey = TileKey;
				
				item.m_FRFTile.reset(new FRFImage);
				DataTileCacheItem::InitializeFRFTileContents(item.m_FRFTile.get());
				DataTileCacheItem::UpdateFRFTileTimeTag(item.m_FRFTile.get());
				
				item.m_lastTouch = std::chrono::steady_clock::now();
			}
		}
		else {
			//Item is already in the cache - do nothing
			std::cout << "Received FRF tile already in mem cache: " << TileKey.ToString() << "\r\n";
		}
	}
}





