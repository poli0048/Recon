//The DJI Comm Link Analysis Window shows useful info about packet throughput, jitter, and latancy
//to help diagnose comm link problems between Recon and a DJI drone
//Author: Bryan Poling
//Copyright (c) 2022 Sentek Systems, LLC. All rights reserved.
#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>

//External Includes
#include "../HandyImGuiInclude.hpp"

class DJICommLinkAnalysisWindow {
	public:
		DJICommLinkAnalysisWindow();
		~DJICommLinkAnalysisWindow() { }
		static DJICommLinkAnalysisWindow & Instance() { static DJICommLinkAnalysisWindow win; return win; }
	
		void Draw();
		
		bool Visible = false;
};
