//The bookmark manager maintains a vector of named location bookmarks in a file
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>
#include <chrono>

//External Includes
#include "../HandyImGuiInclude.hpp"

//Cereal Includes
#include "cereal/types/vector.hpp"
#include "cereal/types/string.hpp"
#include "cereal/archives/json.hpp"

//Project Includes
#include "../EigenAliases.h"
#include "ReconUI.hpp"

struct LocationBookmark {
	std::string Name;
	double MinLat; //Radians
	double MaxLat; //Radians
	double MinLon; //Radians
	double MaxLon; //Radians
	
	LocationBookmark() = default;
	LocationBookmark(std::string const & NameArg, Eigen::Vector2d const & LatBounds, Eigen::Vector2d const & LonBounds) {
		Name = NameArg;
		MinLat = LatBounds(0);
		MaxLat = LatBounds(1);
		MinLon = LonBounds(0);
		MaxLon = LonBounds(1);
	}
	~LocationBookmark() = default;
	
	//Tell Cereal which members to serialize (must be public)
	template<class Archive> void serialize(Archive & archive) { archive(Name, MinLat, MaxLat, MinLon, MaxLon); }
};

//Singleton Class
class BookmarkManager {
	public:
		static BookmarkManager & Instance() { static BookmarkManager Obj; return Obj; }
		
		//Constructors and Destructors
		BookmarkManager() : Log(*(ReconUI::Instance().Log)) { LoadFromDisk(); }
		~BookmarkManager() { SaveToDisk(); }
		
		//We allow other components to just modify our vector of Bookmarks directly, for simplicity
		std::vector<LocationBookmark> Bookmarks;
		
		//Tell Cereal which members to serialize (must be public)
		template<class Archive> void serialize(Archive & archive) { archive(CEREAL_NVP(Bookmarks)); }
	
	private:
		Journal & Log;
		
		inline void SaveToDisk(void);     //Immediately save current object
		inline void LoadFromDisk(void);   //Immediately load from disk, overwriting current object
};

//Immediately save current object
inline void BookmarkManager::SaveToDisk(void) {
	std::string filePath = (Handy::Paths::ThisExecutableDirectory() / "Bookmarks.json").string();
	std::ofstream fileStream(filePath, std::ofstream::out | std::ofstream::binary);
	if (! fileStream.is_open())
		Log.print("Error in BookmarkManager::SaveToDisk: Could not open file for writing.");
	else {
		try {
	  		cereal::JSONOutputArchive oArchive( fileStream );
	  		oArchive(*this);
	  	}
	  	catch (...) { Log.print("Error in BookmarkManager::SaveToDisk: Writing to Cereal archive failed."); }
	}
}

//Immediately load from disk, overwriting current object
inline void BookmarkManager::LoadFromDisk(void) {
	std::string filePath = (Handy::Paths::ThisExecutableDirectory() / "Bookmarks.json").string();
	std::ifstream fileStream(filePath, std::ifstream::in | std::ifstream::binary);
	if (! fileStream.is_open()) {
		Log.print("Error in BookmarkManager::LoadFromDisk: Could not open file for reading.");
		Bookmarks.clear();
	}
	else {
		try {
			cereal::JSONInputArchive iArchive( fileStream );
			iArchive(*this);
		}
		catch (...) {
			Log.print("Error in BookmarkManager::LoadFromDisk: Reading from Cereal archive failed - loading defaults.");
			Bookmarks.clear();
		}
	}
}


