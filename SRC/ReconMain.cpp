//Recon is a GIS system for storing scene and obstacle data for drones and is a multi-vehicle ground control station
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 

//System Includes
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <cstdio>

//External Includes
#include "../imgui/app/ImGuiApp.hpp"
#include "../handycpp/Extended/HandyArgs.hpp"
#include "restclient-cpp/restclient.h"

//Project Includes
#include "Journal.h"
#include "UI/ReconUI.hpp"
#include "ProgOptions.hpp"
#include "Maps/DataTileProvider.hpp"
#include "Maps/SatelliteCacheMaster.hpp"
#include "TestBenches.hpp"

#if defined IS_WINDOWS
	#include <tchar.h>
	#include "../HandyPrebuilts/CrashRpt/Release_2017_x64MD/include/CrashRpt.h"
#endif

//Instantiate the static member variables (static initialization)
ProgOptions * ProgOptions::s_instance = nullptr;

// Command-line argument parsing  ***********************************************************************************************
//All variables in the Arguments namespace are valid after parsing the command-line arguments (calling parseArgs())
namespace Arguments {
	bool          Verbose      = false;                   //Enable extra log/terminal output for debugging
	RenderingAPI  GraphicsAPI  = RenderingAPI::INVALID;   //Rendering API (DirectX, OpenGL, etc.)
	FrameSyncType FrameSync    = FrameSyncType::INVALID;  //VSync on/off or enabled dynamically
	int           TestBenchNum = -1;                      //-1: Run Recon, -2: List Tests, >=0: Run TestBench and exit
};

static RenderingAPI RenderingAPIFromString(std::string const & s) {
	#if defined IS_WINDOWS
		if      (s == "dx11")  return RenderingAPI::DX11;
		else if (s == "dx11w") return RenderingAPI::DX11W;
		else if (s == "dx10")  return RenderingAPI::DX10;
		else if (s == "dx10w") return RenderingAPI::DX10W;
		else if (s == "ogl")   return RenderingAPI::GLFW;
		else if (s == "sw")    return RenderingAPI::SoftwareRasterizer;
	#else
		if (s == "ogl")        return RenderingAPI::GLFW;
	#endif
	return RenderingAPI::INVALID;
}

static std::string RenderingAPIToString(RenderingAPI API, bool Descriptive = false) {
	if (! Descriptive) {
		switch (API) {
			case RenderingAPI::DX11:               return std::string("dx11");
			case RenderingAPI::DX11W:              return std::string("dx11w");
			case RenderingAPI::DX10:               return std::string("dx10");
			case RenderingAPI::DX10W:              return std::string("dx10w");
			case RenderingAPI::GLFW:               return std::string("ogl");
			case RenderingAPI::SoftwareRasterizer: return std::string("sw");
			default:                               return std::string("INVALID");
		}
	}
	else {
		switch (API) {
			case RenderingAPI::DX11:               return std::string("dx11 (DirectX 11)");
			case RenderingAPI::DX11W:              return std::string("dx11w (DirectX 11 Warp - No GPU)");
			case RenderingAPI::DX10:               return std::string("dx10 (DirectX 10)");
			case RenderingAPI::DX10W:              return std::string("dx10w (DirectX 10 Warp - No GPU)");
			case RenderingAPI::GLFW:               return std::string("ogl (OpenGL)");
			case RenderingAPI::SoftwareRasterizer: return std::string("sw (Software - No GPU)");
			default:                               return std::string("INVALID");
		}
	}
}

static RenderingAPI GetDefaultRenderingAPI(void) {
	#if defined IS_WINDOWS
		return RenderingAPI::DX11;
	#else
		return RenderingAPI::GLFW;
	#endif
}

static FrameSyncType FrameSyncTypeFromString(std::string const & s) {
	if      (s == "dynamic") return FrameSyncType::Dynamic;
	else if (s == "vsync")   return FrameSyncType::VSync;
	else if (s == "nosync")  return FrameSyncType::NoSync;
	return FrameSyncType::INVALID;
}

static std::string FrameSyncToString(FrameSyncType Sync) {
	switch (Sync) {
		case FrameSyncType::Dynamic: return std::string("dynamic");
		case FrameSyncType::VSync:   return std::string("vsync");
		case FrameSyncType::NoSync:  return std::string("nosync");
		default:                     return std::string("INVALID");
	}
}

//Make sure arguments are reasonable - called at the end of argument parsing
static void SanitizeArguments(Journal & Log) {
	if (Arguments::GraphicsAPI == RenderingAPI::INVALID) {
		Arguments::GraphicsAPI = GetDefaultRenderingAPI();
		std::string APIString = RenderingAPIToString(Arguments::GraphicsAPI, true);
		std::cout << "Invalid Drawing API option. Defaulting to \"" << APIString << "\"" << std::endl; 
	}
	if (Arguments::FrameSync == FrameSyncType::INVALID) {
		Arguments::FrameSync = FrameSyncType::Dynamic;
		std::string FrameSyncString = FrameSyncToString(Arguments::FrameSync);
		std::cout << "Invalid Framesync option. Defaulting to \"" << FrameSyncString << "\"" << std::endl;
	}
}

void parseArgs(int argc, const char * argv[], std::string const & VersionString, Journal & Log) {
	//Set default arguments
	Arguments::Verbose     = false;
	Arguments::GraphicsAPI = GetDefaultRenderingAPI();
	//#if defined IS_WINDOWS
	//	Arguments::FrameSync   = FrameSyncType::Dynamic;
	//#else
	Arguments::FrameSync   = FrameSyncType::VSync;
	//#endif

	//Wrap everything in a try block.  Do this every time, because exceptions will be thrown for problems.
	try {
		std::stringstream drawingDesc; drawingDesc << std::endl
			<< "The API used to draw all graphics in the application:" << std::endl
			<< "\tdx11   -> DirectX11               (Windows Only) - Windows Default" << std::endl
			<< "\tdx11w  -> DirectX11 WARP - No GPU (Windows Only)" << std::endl
			<< "\tdx10   -> DirectX10               (Windows Only)" << std::endl
			<< "\tdx10w  -> DirectX10 WARP - No GPU (Windows Only)" << std::endl
			<< "\tsw     -> Software       - No GPU (Windows Only)" << std::endl
			<< "\togl    -> OpenGL                                 - NIX Default" << std::endl;

		std::stringstream fsDesc; fsDesc << std::endl
			<< "The strategy used for frame timing in DirectX and OpenGL." << std::endl
			<< "This option is ignored when the drawing API is SW." << std::endl
			<< "Valid options for frame syncing:"                    << std::endl
			<< "\tdynamic -> Adjust Dynamically (balance CPU usage and performance)" << std::endl
			<< "\tvsync   -> Always VSync       (low CPU, less responsive         ) - Default" << std::endl
			<< "\tnosync  -> Never VSync        (high CPU, most responsive        )" << std::endl;
		
		std::string DefaultAPIString = RenderingAPIToString(Arguments::GraphicsAPI, false);
		std::string DefaultFrameSyncString = FrameSyncToString(Arguments::FrameSync);
		
		Handy::Args::CmdLine cmd("Recon - Sentek Systems LLC.", ' ', VersionString, true);
		
		Handy::Args::SwitchArg               verboseSwitch("V", "verbose", "Verbose Mode - enables additional logging for debugging", cmd, Arguments::Verbose);
		Handy::Args::ValueArg<std::string>       APIOption("g", "graphics",  drawingDesc.str(), false, DefaultAPIString.c_str(), "Graphics API Name",  cmd);
		Handy::Args::ValueArg<std::string> frameSyncOption("s", "framesync",      fsDesc.str(), false, DefaultFrameSyncString.c_str(), "Frame Syncing Type", cmd);
		Handy::Args::ValueArg<int>         TestBenchOption("t", "test", "Run testbench instead of running Recon"s, false, -1, "int", cmd);
		Handy::Args::SwitchArg             ListTestsSwitch("l", "list", "List available testbenches and exit."s, cmd, false);
		cmd.parse(argc, argv);
		
		Arguments::Verbose      = verboseSwitch.getValue();
		Arguments::GraphicsAPI  = RenderingAPIFromString(APIOption.getValue());
		Arguments::FrameSync    = FrameSyncTypeFromString(frameSyncOption.getValue());
		Arguments::TestBenchNum = TestBenchOption.getValue();
		if (ListTestsSwitch.getValue())
			Arguments::TestBenchNum = -2;
	}
	catch (Handy::Args::ArgException &e) {std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl;}
	
	SanitizeArguments(Log);
}
//End Command-line argument parsing  ********************************************************************************************

//Set up crash reporting system - WINDOWS ONLY!
//Returns true on success and false on failure
#if defined IS_WINDOWS
bool InstallCrashReporting(void) {
	// Define CrashRpt configuration parameters
	CR_INSTALL_INFO info;  
	std::memset(&info, 0, sizeof(CR_INSTALL_INFO));  
	info.cb = sizeof(CR_INSTALL_INFO);    
	info.pszAppName      = _T("Recon");  
	info.pszAppVersion   = _T("1.0");  
	info.pszEmailSubject = _T("Recon Error Report");  
	info.pszEmailTo      = _T("support@senteksystems.com");    
	info.pszUrl          = _T("http://senteksystems.com");  
	info.uPriorities[CR_HTTP]  = 3; // First try send report over HTTP 
	info.uPriorities[CR_SMTP]  = 2; // Second try send report over SMTP  
	info.uPriorities[CR_SMAPI] = 1; // Third try send report over Simple MAPI    
	info.dwFlags |= CR_INST_ALL_POSSIBLE_HANDLERS; // Install all available exception handlers
	info.dwFlags |= CR_INST_ALLOW_ATTACH_MORE_FILES; 
	info.dwFlags |= CR_INST_SHOW_ADDITIONAL_INFO_FIELDS;
	info.dwFlags |= CR_INST_DONT_SEND_REPORT;
	//info.dwFlags |= CR_INST_SEND_QUEUED_REPORTS; 
	//info.dwFlags |= CR_INST_APP_RESTART;  // Restart the app on crash 
	//info.pszRestartCmdLine = _T("/restart");

	info.pszPrivacyPolicyURL = _T("http://senteksystems.com/privacypolicy.html"); // Define the Privacy Policy URL 
	
	// Install crash reporting
	if (crInstall(&info) != 0) {
		// Something goes wrong. Get error message.
		TCHAR szErrorMsg[512] = _T("");        
		crGetLastErrorMsg(szErrorMsg, 512);    
		_tprintf_s(_T("%s\n"), szErrorMsg);    
		return false;
	}
	return true;
}
#endif

void SetupTerminalIOHighlights(void) {
	Handy::Console::AddHighlights(HC::fgB::red, {
		"ERROR",  "Error",  "error", 
		"FAILED", "Failed", "failed",
		"FAIL",   "Fail",   "fail", 
		"ALERT",  "Alert",  "alert" 
	});

	Handy::Console::AddHighlights(HC::fgB::yellow, { 
		"WARNING",   "Warning",   "warning", 
		"WARN",      "Warn",      "warn", 
		"IMPORTANT", "Important", "important", 
	});

	Handy::Console::AddHighlights(HC::fgB::gray, { 
		"INFO", "Info", "info",
		"DEBUG", "Debug", "debug"
	});
}

void PrintProgramLaunchInfo(std::string const & VersionString, Journal & Log) {
	std::cerr << HC::st::push << HC::fgB::cyan;
	Log.print("Recon - " + VersionString);
	std::cerr << HC::st::pop;
	
	std::cerr << HC::st::push << HC::fg::gray;
	Log.print(Handy::BuildInfoString());
	std::cerr << HC::st::pop;
	
	Log.print ("Info: Selected Drawing API:      " + RenderingAPIToString(Arguments::GraphicsAPI, true));
	Log.print ("Info: Selected Frame Sync Mode:  " + FrameSyncToString(Arguments::FrameSync));
	Log.printf("Info: Verbose Mode:              %s", Arguments::Verbose ? "Enabled" : "Disabled");
	Log.print ("Info: Monitor Refresh Rate:      " + std::to_string(ImGuiApp::Instance().monitorRefreshRate()));
	Log.print ("Info: Executable Directory:      " + Handy::Paths::ThisExecutableDirectory()       .u8string());
	Log.print ("Info: Executable Path:           " + Handy::Paths::ThisExecutableFile()            .u8string());
	Log.print ("Info: Cache Directory:           " + Handy::Paths::CacheDirectory   ("SentekRecon").u8string());
	Log.print ("Info: User Data Directory:       " + Handy::Paths::ThisExecutableDirectory()       .u8string());
}

int main(int argc, const char * argv[]) {
	//Setup stdout and stderr capture
	Handy::Console::Capture(std::cout);
	Handy::Console::Capture(std::cerr);
	Handy::Console::SetOutputCallback([](std::string const & text) { });
	
	//Get paths to local cache folder and user data folder
	//std::filesystem::path UserDataFolderPath = Handy::Paths::UserDataDirectory("SentekRecon");
	std::filesystem::path UserDataFolderPath = Handy::Paths::ThisExecutableDirectory();
	std::filesystem::path CacheFolderPath    = Handy::Paths::CacheDirectory("SentekRecon");
	
	//Create a journal file - overwrite mode. This is to help debug crashes.
	Journal log(UserDataFolderPath / "ReconSessionLog.txt", &(std::cerr), false);
	SetupTerminalIOHighlights();
	log.print_continued("Program Started: ");
	
	//Parse command-line arguments and print initial launch info
	std::string ReconVersionString = "v1.0";
	parseArgs(argc, argv, ReconVersionString, log);
	PrintProgramLaunchInfo(ReconVersionString, log);
	
	//If we have been asked to run a testbench instead of starting Recon, do that
	if (Arguments::TestBenchNum == -2) {
		std::cerr << "Available Test Benches:\r\n";
		for (size_t index = 0U; index < TestBenches::AvailableTestBenches.size(); index++)
			std::cerr << index << ": " << TestBenches::AvailableTestBenches[index] << "\r\n";
		return 0;
	}
	else if (Arguments::TestBenchNum >= 0) {
		TestBenches::RunTestBench(Arguments::TestBenchNum);
		return 0;
	}
	
	//Setup crash handling on Windows
	#if defined IS_WINDOWS
		Handy::OnScopeExit ose([] { crUninstall(); }); // Uninitialize CrashRpt on main function exit
		InstallCrashReporting();
	#endif
	
	//Prevent more than one instance of program from running at once
	using Handy::SingleInstanceLock;
	SingleInstanceLock instanceLock(CacheFolderPath / "Instance.lock");
	SingleInstanceLock::ResultType result = instanceLock.GetLock();
	if (result == SingleInstanceLock::ResultType::Lock_Failed) {
		log.print("Recon UI Already Running. Only one instance is allowed at one time. Aborting.");
		return -1;
	}
	else if (result == SingleInstanceLock::ResultType::Lock_Succeeded_ButPreviousInstanceCrashed)
		log.print("Warning: It looks like Recon didn't exit properly last time it ran.");
	
	log.print_continued("Loading program options ... ... ... ");
	ProgOptions::Init(UserDataFolderPath / "ProgOptionsV2.json", log);
	log.print("Done.");
	
	log.print_continued("Initializing RestClient ... ... ... ");
	RestClient::init();
	log.print("Done.");
	
	log.print_continued("Initializing Tile Providers ... ... ");
	Maps::DataTileProvider::Init(log);
	Maps::SatelliteCacheMaster::Instance()->Init(log);
	log.print("Done.");
	
	//Get reference to ImGuiApp Singleton and launch UI
	ImGuiApp & app = ImGuiApp::Instance();
	app.Init(Arguments::GraphicsAPI, Arguments::FrameSync, false);
	app.SetScaling(ProgOptions::Instance()->UIScaleFactor); //Set initial UI scale factor
	app.SetFontScaling(1.25f);                              //Make all fonts bigger
	ImGui::GetIO().IniFilename = nullptr;                   //Disable INI file
	ImGui::GetIO().ConfigViewportsNoAutoMerge = true;
	ImGui::GetIO().ConfigViewportsNoDecoration = false;
	ReconUI & ui = ReconUI::Instance();
	ui.Log = &log;
	app.Main(&ui);
	
	log.print_continued("Destroying Tile Providers . ... ... ");
	Maps::SatelliteCacheMaster::Instance()->Destroy();
	Maps::DataTileProvider::Destroy();
	log.print("Done.");
	
	log.print_continued("Disabling RestClient .. ... ... ... ");
	RestClient::disable();
	log.print("Done.");
	
	
	log.print_continued("Saving program options  ... ... ... ");
	ProgOptions::Destroy();
	log.print("Done.");
	
	log.print("Exiting Recon.");

	app.Deinit();
	return 0;
}




