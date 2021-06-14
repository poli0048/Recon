//The About Window shows information about the software, its build configuration and storage locations.
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.

//System Includes
#include <thread>

//Project Includes
#include "AboutWindow.hpp"
//#include "../ImVecOps.hpp"
#include "../Maps/SatelliteCacheMaster.hpp"

AboutWindow::AboutWindow() {
	m_UserDataPath       = Handy::Paths::ThisExecutableDirectory();
	m_CachePath          = Handy::Paths::CacheDirectory("SentekRecon");
}

void AboutWindow::Draw() {
	ImExt::Window::Options wOpts;
	wOpts.Flags = WindowFlags::NoCollapse | WindowFlags::NoSavedSettings | WindowFlags::NoDocking | WindowFlags::NoTitleBar | WindowFlags::NoResize;
	wOpts.POpen = &Visible;
	wOpts.Size(Math::Vector2(40.0f*ImGui::GetFontSize(), 18.0f*ImGui::GetFontSize()), Condition::Appearing);
	if (ImExt::Window window("About", wOpts); window.ShouldDrawContents()) {
		ImExt::Style style(StyleVar::WindowPadding, Math::Vector2(20.0f));
		
		ImGui::BeginChild("About Scrollable Region", ImVec2(0,0), true, ImGuiWindowFlags_AlwaysUseWindowPadding);
	
		const char HeadingLabel[] = "Recon: A Multi-Vehicle Ground Control Station for small UAS";
		float HeadingWidth = ImGui::CalcTextSize(HeadingLabel).x;
		float gapWidth = std::max(0.5f*(ImGui::GetContentRegionAvail().x - HeadingWidth), 0.0f);
		ImGui::SetCursorPosX(ImGui::GetCursorPosX() + gapWidth);
		ImGui::TextUnformatted(HeadingLabel);
		ImExt::Dummy(5_f, 20_f);
		
		//Print core version, platform, and build configuration
		float col2Start = ImGui::GetCursorPosX() + ImGui::CalcTextSize("Sat tile cache:        ").x;
		ImGui::TextUnformatted("Version:");
		ImGui::SameLine(col2Start);
		ImGui::TextUnformatted("1.0");
		
		#if defined(_WIN32)
			std::string PlatformString = std::string("Windows 64-bit");
		#elif defined (__APPLE__) || defined(MACOSX)
			std::string PlatformString = std::string("Apple OSX");
		#elif defined(__unix__) || defined(__unix) || defined(unix)
			std::string PlatformString = std::string("Linux 64-bit");
		#else
			std::string PlatformString = std::string("Unrecognized");
		#endif
		#if defined(OPTIMISED_BUILD)
			PlatformString += " (Optimized build)";
		#elif defined(LEGACY_BUILD)
			PlatformString += " (Legacy build)";
		#endif
		ImGui::TextUnformatted("Platform:");
		ImGui::SameLine(col2Start);
		ImGui::TextUnformatted(PlatformString.c_str());
		
		//Print user data and local cache folder paths
		ImGui::TextUnformatted("Data Folder:");
		ImGui::SameLine(col2Start);
		ImGui::TextUnformatted(m_UserDataPath.string().c_str());
		ImGui::TextUnformatted("Cache Folder:");
		ImGui::SameLine(col2Start);
		ImGui::TextUnformatted(m_CachePath.string().c_str());
		ImGui::TextUnformatted("Sat tile cache:");
		ImGui::SameLine(col2Start);
		double cachedSatTiles = double(Maps::SatelliteCacheMaster::Instance()->CacheFile_GetNumItems());
		if (cachedSatTiles > 1e6)
			ImGui::Text("%.1f Million Tiles  ", cachedSatTiles*1e-6);
		else if (cachedSatTiles > 1e3)
			ImGui::Text("%.1f Thousand Tiles  ", cachedSatTiles*1e-3);
		else
			ImGui::Text("%u Tiles  ", (unsigned int) std::round(cachedSatTiles));
		ImGui::SameLine();
		double satCacheBytesOnDisk = double(Maps::SatelliteCacheMaster::Instance()->CacheFile_GetNumBytesOnDisk());
		double size_MiB = double(satCacheBytesOnDisk)/1048576.0;
		double size_GiB = double(satCacheBytesOnDisk)/1073741824.0;
		if (size_GiB > 1.0)
			ImGui::Text("(%.1f GiB)  ", size_GiB);
		else
			ImGui::Text("(%.1f MiB)  ", size_MiB);
		ImGui::SameLine();
		{
			ImExt::Style buttonStyle;
			buttonStyle(StyleCol::Button,        Math::Vector4(0.90f, 0.3f, 0.3f, 1.0f));
			buttonStyle(StyleCol::ButtonHovered, Math::Vector4(0.95f, 0.2f, 0.2f, 1.0f));
			buttonStyle(StyleCol::ButtonActive,  Math::Vector4(1.00f, 0.1f, 0.1f, 1.0f));
			if (ImGui::SmallButton(" Clear Cache "))
				Maps::SatelliteCacheMaster::Instance()->PurgeAll();
		}
		
		ImGui::EndChild();
	}
}






