//This module provides an encapsulation for survey regions and a singleton manager class for keeping track of
//an "active" survey region and providing access to it.
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>
//#include <chrono>
#include <system_error>

//External Includes
#include "../../handycpp/Handy.hpp"

//Cereal Includes
#include "cereal/types/vector.hpp"
#include "cereal/types/string.hpp"
#include "cereal/archives/portable_binary.hpp"
#include "cereal/archives/json.hpp"

//Project Includes
#include "EigenAliases.h"
#include "Polygon.hpp"
#include "Utilities.hpp"

class SurveyRegion {
	public:
		std::mutex m_mutex;                     //Lock the object when accessing for thread safety! If changed, update triangulation before unlocking.
		std::string m_Name;                     //This defines the name of the file the object is saved to and read from
		PolygonCollection m_Region;             //NM coordinates
		std::Evector<Triangle> m_triangulation; //NM coordinates
		
		//Constructors and Destructors - objects given a name are automatically loaded from disk on construction and saved on destruction
		SurveyRegion() = default;
		SurveyRegion(std::string Name) : m_Name(Name) {
			std::scoped_lock lock(m_mutex);
			if (! Name.empty())
				LoadFromDisk();
		}
		~SurveyRegion() {
			std::scoped_lock lock(m_mutex);
			if (! m_Name.empty())
				SaveToDisk();
		}
		
		//Objects attempt to load from disk on construction (if given a name) and they save to disk on destruction.
		//Consequently, they should not be copied or moved... only one instance of a single named survey region should exist at once
		SurveyRegion(SurveyRegion const &)            = delete;
		SurveyRegion(SurveyRegion &&)                 = delete;
		SurveyRegion& operator=(SurveyRegion const &) = delete;
		SurveyRegion& operator=(SurveyRegion &&)      = delete;
		
		//Tell Cereal which members to serialize
		template<class Archive> void serialize(Archive & archive) { archive(m_Region); }
	
	private:
		inline void SaveToDisk(void);   //Immediately save object
		inline void LoadFromDisk(void); //Immediately load from disk, overwriting current object
};

//Singleton Class
class SurveyRegionManager {
	public:
		static SurveyRegionManager & Instance() { static SurveyRegionManager Obj; return Obj; }
		
		//Constructors and Destructors - State loaded on construction and saved on destruction
		SurveyRegionManager()  { std::scoped_lock lock(m_mutex); LoadStateFromDisk(); }
		~SurveyRegionManager() { std::scoped_lock lock(m_mutex); SaveStateToDisk();   }
		
		//Get a pointer to the active SurveyRegion - this pointer may be invalidated on a call to SetActiveSurveyRegion().
		SurveyRegion * GetActiveSurveyRegion(void) {
			std::scoped_lock lock(m_mutex);
			return m_ActiveRegion.get();
		}
		
		//Change the active survey region to the one with the given name (which may or may not exist on disk)
		//Pass an empty string to set no active region. This should only be called from within the main draw loop,
		//or pointers in use may be invalidated.
		void SetActiveSurveyRegion(std::string RegionName) {
			std::scoped_lock lock(m_mutex);
			if (RegionName.empty()) {
				m_ActiveRegionName.clear();
				m_ActiveRegion.reset();
			}
			else if (m_ActiveRegionName != RegionName) {
				m_ActiveRegionName = RegionName;
				m_ActiveRegion.reset(new SurveyRegion(RegionName));
			}
		}
		
		//Tell Cereal which members to serialize (must be public)
		template<class Archive> void serialize(Archive & archive) { archive(CEREAL_NVP(m_ActiveRegionName)); }
	
	private:
		std::mutex m_mutex;
		std::string m_ActiveRegionName;
		std::unique_ptr<SurveyRegion> m_ActiveRegion;
		
		inline void SaveStateToDisk(void);   //Immediately save current object
		inline void LoadStateFromDisk(void); //Immediately load from disk, overwriting current object
};


// ************************************************************************************************************************************************
// ***********************************************************   SurveyRegion Definitions   *******************************************************
// ************************************************************************************************************************************************

//Immediately save object
inline void SurveyRegion::SaveToDisk(void) {
	if (! isFilenameReasonable(m_Name)) {
		std::cerr << "Can't save survey region because name is not a usable filename.\r\n";
		return;
	}
	std::filesystem::path folderPath = Handy::Paths::ThisExecutableDirectory() / "Survey Regions";
	if (! std::filesystem::exists(folderPath)) {
		std::error_code ec;
		std::filesystem::create_directory(folderPath, ec);
	}
	if ((! std::filesystem::exists(folderPath)) || (! std::filesystem::is_directory(folderPath))) {
		std::cerr << "Error: Can't save survey region because sub-directory 'Survey Regions' doesn't exist and can't be created.\r\n";
		return;
	}
	
	//If we get here, the folder exists so we should be able to try and save the file
	std::string filePath = (folderPath / (m_Name + ".region")).string();
	std::ofstream fileStream(filePath, std::ofstream::out | std::ofstream::binary);
	if (! fileStream.is_open())
		std::cerr << "Error in SurveyRegion::SaveToDisk: Could not open file for writing.\r\n";
	else {
		try {
	  		cereal::PortableBinaryOutputArchive oArchive( fileStream );
	  		oArchive(*this);
	  	}
	  	catch (...) { std::cerr << "Error in SurveyRegion::SaveToDisk: Writing to Cereal archive failed.\r\n"; }
	}
}

//Immediately load from disk, overwriting current object
inline void SurveyRegion::LoadFromDisk(void) {
	if (m_Name.empty()) {
		std::cerr << "Error in SurveyRegion::LoadFromDisk: Name must be set to load from disk.\r\n";
		return;
	}
	
	std::filesystem::path filePath = Handy::Paths::ThisExecutableDirectory() / "Survey Regions" / (m_Name + ".region");
	//If the name is valid but the file doesn't exist, this isn't an error - we are probably just creating a new region.
	//If there is nothing to load, doing nothing is the correct behavior.
	if (! std::filesystem::exists(filePath))
		return;
	
	std::ifstream fileStream(filePath.string(), std::ifstream::in | std::ifstream::binary);
	if (! fileStream.is_open()) {
		std::cerr << "Error in SurveyRegion::LoadFromDisk: Could not open file for reading.\r\n";
		return;
	}
	else {
		try {
			cereal::PortableBinaryInputArchive iArchive( fileStream );
			iArchive(*this);
		}
		catch (...) {
			std::cerr << "Error in SurveyRegion::LoadFromDisk: Reading from Cereal archive failed.";
			return;
		}
	}
	//Sanitize by throwing out empty and invalid polygons from polygon collection
	int trashCounter = 0;
	size_t compIndex = 0U;
	while (compIndex < m_Region.m_components.size()) {
		if ((m_Region.m_components[compIndex].m_boundary.NumVertices() < 3U) || (! m_Region.m_components[compIndex].IsValid())) {
			m_Region.m_components.erase(m_Region.m_components.begin() + compIndex);
			trashCounter++;
		}
		else
			compIndex++;
	}
	m_Region.Triangulate(m_triangulation);
	if (trashCounter > 0)
		std::cerr << "Warning: " << trashCounter << " empty or invalid polygons removed from collection.\r\n";
	//std::cerr << m_Region << "\r\n";
}

// ************************************************************************************************************************************************
// *******************************************************   SurveyRegionManager Definitions   ****************************************************
// ************************************************************************************************************************************************

//Immediately save current object
inline void SurveyRegionManager::SaveStateToDisk(void) {
	std::string filepath = (Handy::Paths::CacheDirectory("SentekRecon") / "SurveyRegionManagerState.json").string();
	std::ofstream fileStream(filepath, std::ofstream::out | std::ofstream::binary);
	if (! fileStream.is_open())
		std::cerr <<"Error in SurveyRegionManager::SaveStateToDisk: Could not open file for writing.\r\n";
	else {
		try {
	  		cereal::JSONOutputArchive oArchive( fileStream );
	  		oArchive(*this);
	  	}
	  	catch (...) { std::cerr << "Error in SurveyRegionManager::SaveStateToDisk: Writing to Cereal archive failed.\r\n"; }
	}
}

//Immediately load from disk, overwriting current object
inline void SurveyRegionManager::LoadStateFromDisk(void) {
	std::string filepath = (Handy::Paths::CacheDirectory("SentekRecon") / "SurveyRegionManagerState.json").string();
	std::ifstream fileStream(filepath, std::ifstream::in | std::ifstream::binary);
	if (! fileStream.is_open()) {
		std::cerr << "Error in SurveyRegionManager::LoadStateFromDisk: Could not open file for reading.\r\n";
		m_ActiveRegionName.clear();
		m_ActiveRegion.reset();
		return;
	}
	else {
		try {
			cereal::JSONInputArchive iArchive( fileStream );
			iArchive(*this);
		}
		catch (...) {
			std::cerr << "Error in SurveyRegionManager::LoadStateFromDisk: Reading from Cereal archive failed - loading defaults.\r\n";
			m_ActiveRegionName.clear();
			m_ActiveRegion.reset();
			return;
		}
	}
	if (! m_ActiveRegionName.empty())
		m_ActiveRegion.reset(new SurveyRegion(m_ActiveRegionName));
}





