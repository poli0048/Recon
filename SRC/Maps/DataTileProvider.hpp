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
#pragma once

//System Includes
#include <unordered_map>
#include <mutex>
#include <chrono>
#include <tuple>
#include <ratio>
#include <deque>

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "../EigenAliases.h"
#include "Tile.hpp"
#include "SatelliteSources.hpp"
#include "Interfaces.hpp"
#include "DataTileTypes.hpp"
#include "FRFTileStore.hpp"
#include "../Journal.h"

namespace Maps {
	//m_FRFTile should be valid once an object is in the cache and the FRF tile should remain valid until destruction
	struct DataTileCacheItem {
		using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
		
		public:
			Tile m_tileKey;
			TimePoint m_lastTouch; //Last time this item was accessed in any way
			std::unique_ptr<FRFImage> m_FRFTile;
			bool m_FRFEdited = false;
			TimePoint m_LastEditTime;
			
			//Vis tiles are not explicitly double-buffered. Instead, to prevent screen blanking, we use delayed texture deletion
			bool m_vizValid = false;
			VizualizationTileKey m_VizKey;
			ImTextureID m_TextureID;
			TimePoint m_VizEvalTime;
			
			DataTileCacheItem() = delete;
			DataTileCacheItem(FRFTileStore * AssociatedFileStore) : m_FRFFileStore(AssociatedFileStore) { }
			~DataTileCacheItem();
			
			static void InitializeFRFTileContents(FRFImage * FRFTile);
			static void UpdateFRFTileTimeTag(FRFImage * FRFTile);
		
		private:
			FRFTileStore * m_FRFFileStore; //Pointer to the file store associated with the cache - used for write-back in destructor
			bool FRFImageIsTrivial(void); //Returns true if FRF tile has the same contents as a freshly-initialized tile
	};
	
	struct PaintActionItem {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		int m_shape;  //0=Circle, 1=Rectangle
		Eigen::Vector2d m_Center_NM;
		DataLayer m_layer;
		double m_Value; //NaN for erase action
		
		//Circle Properties
		double m_Radius_meters;
		
		//Rectangle Properties
		double m_LengthX; //Meters
		double m_LengthY; //Meters
		double m_Theta;   //Radians
		
		PaintActionItem() = default;
		~PaintActionItem() = default;
		
		//Returns true if the given item is essentially the same as this item (i.e. close enough that if we can skip one of them)
		bool IsEssentiallyIdentical(PaintActionItem const & OtherItem);
		
		Eigen::Vector4d GetNM_AABB(void);
		void GetCorners(Eigen::Vector2d & P1_NM, Eigen::Vector2d & P2_NM, Eigen::Vector2d & P3_NM, Eigen::Vector2d & P4_NM); //Only for rectangle actions
	};
	
	//DataTileProvider is a singleton class. It is thread-safe so many different modules can share it on or off main-thread.
	struct DataTileProvider : IFRFFileReceiver {
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
			static constexpr std::chrono::seconds ExpirationTimeSeconds = std::chrono::seconds(20);
			static constexpr std::chrono::seconds LowResUpdatePeriodSeconds = std::chrono::seconds(3);
			static constexpr int32_t NumberOfVizEvaluationThreads = 4;
			static constexpr size_t MaxNumberOfJobsInThreadPool = 8; //New jobs will only be added to the queue if there is room
			static constexpr int32_t TileEditZoomLevel = 20; //The pyramid level on which edits are done (lower-res tiles are derived from these)
			static constexpr int32_t DataTileMinZoomLevel = 15; //The lowest pyramid level for which viz tiles are maintained and returned (empty below this)

		private:
			static DataTileProvider * s_instance;
			
			Journal & Log;
			FRFTileStore * m_FRFFileStore;
			std::atomic_bool m_abort; //Set to True to signal to secondary threads that it's time to wrap things up.
			
			std::mutex m_cache_mtx;
			std::unordered_map<Tile, DataTileCacheItem> m_cache;
			
			std::mutex m_editStream_mtx;
			std::Edeque<PaintActionItem> m_actionQueue; //Push to back and pop from front
			
			Handy::ThreadPool m_threads_VisEval; //Thread pool for evaluating visualizations from FRF tiles and loading them into GPU memory
			std::mutex m_mtx_TilesWithcurrentVizEvalJobs;
			std::unordered_set<Tile> m_TilesWithcurrentVizEvalJobs; //Just a set of Tile keys - we only allow one job per tile in queue at once
			
			//The DataTileProvider runs a simple garbage collector in a separate thread to periodically purge unused tiles from the memory cache
			void GarbageCollectionThreadMain(void);
			std::thread m_garbageCollectionThread;
			
			//The DataTileProvider processed edit actions in a separate thread
			void DataEditThreadMain(void);
			std::thread m_DataEditThread;
			
			void TouchLoadFRFTile(Tile Key);
			
			//FRF Edit Support Functions
			bool AllTilesPresent(std::vector<Tile> const & Tiles);
			void TouchLoadFRFTilesAndWait(std::vector<Tile> const & Tiles);
			void UpdateLowerResTiles(std::unordered_set<Tile> const & EditedTiles);
			bool ExecuteEditActionOnTile_Circle(PaintActionItem const & Action, Tile tile, Eigen::Vector4d const & AABB_NM);
			bool ExecuteEditActionOnTile_Rectangle(PaintActionItem const & Action, Tile tile, Eigen::Vector4d const & AABB_NM);
			
		public:
			static void               Init(Journal & LogRef) { s_instance = new DataTileProvider(LogRef); }
			static void               Destroy(void)          { delete s_instance; s_instance = nullptr; }
			static DataTileProvider * Instance(void)         { return s_instance; }
			
			DataTileProvider(Journal & LogRef);
			~DataTileProvider();
			
			//Retrieve the given visualization tile. If not cached a request will be issued to start generating it (loading the FRF resource if needed).
			//If another viz tile is already present for the same tile it will be returned if the one requested needs to be generated and isn't ready yet.
			//This is inteanded to prevent screen blanking while chanhing visualization parameters.
			ImTextureID TryGetLoadUpdate_VizTile(VizualizationTileKey Key);
			
			//Data access - get a value from a single layer at a single location. The retrieved data is passed back through reference argument "Value".
			//Important note: This function follows the immediate-mode paradigm used in many other parts of this program, which may seem unexpected. This
			//means that if the requested value isn't immediately available this function will fail and return false. However, it will queue up the needed
			//resources for loading so later calls accessing the same location will eventually succeed (and return True).
			bool TryGetData(Eigen::Vector2d const & Position_NM, DataLayer layer, double & Value);
			
			//Same as TryGetData, but will block until the requested data is available (up to a given number of seconds, after which it will return NaN)
			double GetData(Eigen::Vector2d const & Position_NM, DataLayer layer, double Timeout);

			//Get the min and max (non-NaN) values of a given data layer along a line from point A to point B, sampled every "sampleDist" meters.
			//Aborts if the values can't be resolved within "Timeout" seconds. Returns true on success and false on timeout.
			//On success, NaNsDetected will also be populated... true if any NaNs detected on line and false otherwise.
			bool GetDataExtremesOnLine(Eigen::Vector2d const & PointA_NM, Eigen::Vector2d const & PointB_NM, double SampleDist, DataLayer layer,
			                           double Timeout, double & Data_Min, double & Data_Max, bool & NaNsDetected);
			
			//FRF Edit Tools - when painting, tiles will be created as needed. Painting is done at a fixed pyramid level (set in a constexpr above)
			//and lower-res levels are derived from this level and updated as needed. We make a special exception and take the rectangle angle in
			//degrees so it can be exposed in degrees by ImGui without conversion. Painting is done asynchronously.
			inline void Paint_Circle(Eigen::Vector2d const & Center_NM, double Radius_meters, DataLayer layer, double Value);
			inline void Erase_Circle(Eigen::Vector2d const & Center_NM, double Radius_meters, DataLayer layer);
			inline void Paint_Rect(Eigen::Vector2d const & Center_NM, double LengthX, double LengthY, double AngleDeg, DataLayer layer, double Value);
			inline void Erase_Rect(Eigen::Vector2d const & Center_NM, double LengthX, double LengthY, double AngleDeg, DataLayer layer);
			
			//Force-purge tiles from cache. Note that the cache will self-garbage-collect so this is only needed if you are changing things on disk
			//or want to force a flush of cached data back to disk (like on exit)
			void PurgeAllTiles();
			
			//Called after an asynchronous FRFImage retrieval - We are responsible for deletion of Data.
			void OnReceivedFRFTile(Tile TileKey, FRFImage * Data) override;
	};
	
	inline void DataTileProvider::Paint_Circle(Eigen::Vector2d const & Center_NM, double Radius_meters, DataLayer layer, double Value) {
		std::scoped_lock lock(m_editStream_mtx);
		m_actionQueue.emplace_back();
		m_actionQueue.back().m_shape = 0;
		m_actionQueue.back().m_Center_NM = Center_NM;
		m_actionQueue.back().m_layer = layer;
		m_actionQueue.back().m_Value = Value;
		m_actionQueue.back().m_Radius_meters = Radius_meters;
	}
	
	inline void DataTileProvider::Erase_Circle(Eigen::Vector2d const & Center_NM, double Radius_meters, DataLayer layer) {
		std::scoped_lock lock(m_editStream_mtx);
		m_actionQueue.emplace_back();
		m_actionQueue.back().m_shape = 0;
		m_actionQueue.back().m_Center_NM = Center_NM;
		m_actionQueue.back().m_layer = layer;
		m_actionQueue.back().m_Value = std::nan("");
		m_actionQueue.back().m_Radius_meters = Radius_meters;
	}
	
	inline void DataTileProvider::Paint_Rect(Eigen::Vector2d const & Center_NM, double LengthX, double LengthY, double AngleDeg, DataLayer layer, double Value) {
		double const PI = 3.14159265358979;
		std::scoped_lock lock(m_editStream_mtx);
		m_actionQueue.emplace_back();
		m_actionQueue.back().m_shape = 1;
		m_actionQueue.back().m_Center_NM = Center_NM;
		m_actionQueue.back().m_layer = layer;
		m_actionQueue.back().m_Value = Value;
		m_actionQueue.back().m_LengthX = LengthX;
		m_actionQueue.back().m_LengthY = LengthY;
		m_actionQueue.back().m_Theta = AngleDeg*PI/180.0;
	}
	
	inline void DataTileProvider::Erase_Rect(Eigen::Vector2d const & Center_NM, double LengthX, double LengthY, double AngleDeg, DataLayer layer) {
		double const PI = 3.14159265358979;
		std::scoped_lock lock(m_editStream_mtx);
		m_actionQueue.emplace_back();
		m_actionQueue.back().m_shape = 1;
		m_actionQueue.back().m_Center_NM = Center_NM;
		m_actionQueue.back().m_layer = layer;
		m_actionQueue.back().m_Value = std::nan("");
		m_actionQueue.back().m_LengthX = LengthX;
		m_actionQueue.back().m_LengthY = LengthY;
		m_actionQueue.back().m_Theta = AngleDeg*PI/180.0;
	}
}





