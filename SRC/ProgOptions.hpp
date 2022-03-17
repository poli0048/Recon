//This singleton class holds program options
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <iostream>
#include <fstream>

//External Includes
#include "../../handycpp/Handy.hpp"

//Cereal Includes
#include "cereal/types/vector.hpp"
#include "cereal/types/string.hpp"
#include "cereal/archives/json.hpp"

//GEMS-Core Includes
#include "Journal.h"

//Project Includes
#include "UI/Themes.hpp"

//There should be exactly one instance of the options struct.
class ProgOptions {
	private:
		static ProgOptions * s_instance; //Must be instantiated in exactly one translation unit (we do it in CheetahMain.cpp)
		
		Journal & Log;
		std::filesystem::path m_optionFilePath;
		
	public:
		ProgOptions() = delete;
		ProgOptions(std::filesystem::path OptionFilePath, Journal & LogRef) : Log(LogRef), m_optionFilePath(OptionFilePath) {
			LoadDefaults();
			LoadFromDisk();
		}
		~ProgOptions() { SaveToDisk(); }
		
		static void Init(std::filesystem::path OptionFilePath, Journal & LogRef) { s_instance = new ProgOptions(OptionFilePath, LogRef); }
		static void Destroy() { delete s_instance; s_instance = nullptr; }
		static ProgOptions * Instance(void) { return s_instance; }
		
		//Important: options may be accessed off-main-thread, which means we need a mechanism for negotiating access. We opt to just add a
		//mutex as a public field and require that this be locked while accessing (reading or writing) any other fields. This is locked in
		//ReconUI::Draw() so all access from the main draw thread are covered by that; you only need to manually lock this when accessing
		//fields of this class off the main thread.
		std::mutex OptionsMutex;
		
		//The options are public fields so we can hand pointers to them directy to ImGui to expose the options.
		float UIScaleFactor;     //Scale factor for all UI elements
		float MapDPIPercentage;  //Val = X: Map tiles selected so 1 screen pixel = X/100 tile pixels. Generally set between 50 and 100.
		float DroneIconScale;    //Value of 1 means 1 pixel to 1 pixel. Value of 2 means 2 screen pixels per image pixel, etc.
		float zoomSpeed;         //Normalized so that 1.0 is reasonable.
		Themes::Theme UITheme;   //Light, Dark, etc.
		bool HighContrastCursor; //ImGui draws the cursor and tries to make it bigger and easier to see when true
		
		bool GNSSModuleEnabled;             //If true, the GNSS receiver module should listen for GNSS receivers at the specified device path
		bool GNSSModuleVerbose;             //If true, the GNSS receiver module will print out info about receiver status (for debugging GNSS)
		std::string GNSSReceiverDevicePath; //Path to serial device representing the GNSS receiver (or the port number on Windows)
		int GNSSReceiverBaudRate;           //Baud rate for serial device
		
		void LoadDefaults(void);    //Set all options to defaults (good fallback if file loading fails)
		void SanitizeOptions(void); //Make sure all options are reasonable
		void SaveToDisk(void);      //Immediately save current object
		void LoadFromDisk(void);    //Immediately load from disk, overwriting current object
		
		//Tell Cereal which members to serialize
		template<class Archive> void serialize(Archive & archive) {
			archive(CEREAL_NVP(UIScaleFactor),
			        CEREAL_NVP(MapDPIPercentage),
			        CEREAL_NVP(DroneIconScale),
			        CEREAL_NVP(zoomSpeed),
			        CEREAL_NVP(UITheme),
			        CEREAL_NVP(HighContrastCursor),
			        CEREAL_NVP(GNSSModuleEnabled),
			        CEREAL_NVP(GNSSModuleVerbose),
			        CEREAL_NVP(GNSSReceiverDevicePath),
			        CEREAL_NVP(GNSSReceiverBaudRate));
		}
};

inline void ProgOptions::LoadDefaults(void) {
	UIScaleFactor          = 1.0f;
	MapDPIPercentage       = 100.0f;
	DroneIconScale         = 0.5f;
	zoomSpeed              = 1.0f;
	UITheme                = Themes::Theme::Dark;
	HighContrastCursor     = false;
	GNSSModuleEnabled      = true;
	GNSSModuleVerbose      = false;
	GNSSReceiverDevicePath = "/dev/ttyACM0"s;
	GNSSReceiverBaudRate   = 9600;
}

inline void ProgOptions::SanitizeOptions(void) {
	UIScaleFactor        = std::min(std::max(UIScaleFactor,    0.250f),   4.0f);
	MapDPIPercentage     = std::min(std::max(MapDPIPercentage, 50.00f), 100.0f);
	DroneIconScale       = std::min(std::max(DroneIconScale,   0.250f),   2.0f);
	zoomSpeed            = std::min(std::max(zoomSpeed,        0.125f),   8.0f);
	GNSSReceiverBaudRate = std::max(GNSSReceiverBaudRate, 1200);
}

inline void ProgOptions::SaveToDisk(void) {
	std::ofstream fileStream(m_optionFilePath.string(), std::ofstream::out | std::ofstream::binary);
	if (! fileStream.is_open())
		Log.print("Error in ProgOptions::SaveToDisk: Could not open file for writing.");
	else {
		try {
	  		cereal::JSONOutputArchive oArchive( fileStream );
	  		oArchive(*this);
	  	}
	  	catch (...) { Log.print("Error in ProgOptions::SaveToDisk: Writing to Cereal archive failed."); }
	}
}

inline void ProgOptions::LoadFromDisk(void) {
	std::ifstream fileStream(m_optionFilePath.string(), std::ifstream::in | std::ifstream::binary);
	if (! fileStream.is_open()) {
		Log.print("Warning in ProgOptions::LoadFromDisk: Could not open file for reading. Loading default.");
		LoadDefaults();
	}
	else {
		try {
			cereal::JSONInputArchive iArchive( fileStream );
			iArchive(*this);
		}
		catch (...) {
			Log.print("Error in ProgOptions::LoadFromDisk: Reading from Cereal archive failed - loading defaults.");
			LoadDefaults();
		}
	}
	SanitizeOptions();
}

//Enable saving of theme field in a human-readable form
namespace cereal {
	template <class Archive> std::string save_minimal(Archive const &, Themes::Theme const & obj) {
	    return Themes::ThemeToString(obj);
	}

	template <class Archive> void load_minimal(Archive const &, Themes::Theme & obj, std::string const & value) {
	    obj = Themes::StringToTheme(value);
	}
}



