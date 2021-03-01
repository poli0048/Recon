//The About Window shows information about the software, its build configuration and storage locations.
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.
#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>

//External Includes
#include "../../../handycpp/Handy.hpp"
#include "../../../imgui/app/ImGuiApp.hpp"

class AboutWindow {
	public:
		AboutWindow();
		~AboutWindow() { }
		static AboutWindow & Instance() { static AboutWindow win; return win; }
	
		void Draw();
		
		bool Visible = false;
	
	private:
		std::filesystem::path m_UserDataPath;
		std::filesystem::path m_CachePath;
};
