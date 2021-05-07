//This class provides a simple status bar with basic state information about Cheetah
//Author: Bryan Poling
//Copyright (c) 2019 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <string>
#include <vector>

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "ConsoleWidget.hpp"
#include "MapWidget.hpp"

//The status bar has no internal state, so we just use a namespace instead of creating a singleton class
namespace StatusBar {
	void Draw() {
   		ImGui::Text("# of Data Tiles Drawn: %d", MapWidget::Instance().GetNumberOfDataTilesDrawn());
   		
   		//Draw "Show Console" Button
   		if (ConsoleWidget::Instance().IsClosedOrClosing()) {
	   		float ConsoleButtonWidth = ImGui::CalcTextSize(u8"\uf102  Show Console  \uf102").x + 2.0f*ImGui::GetStyle().FramePadding.x;
	   		ImGui::SameLine();
	   		ImGui::Dummy(ImVec2(ImGui::GetContentRegionAvail().x - ConsoleButtonWidth - 10.0f, 1.0f));
	   		ImGui::SameLine();
	   		if (ImGui::SmallButton(u8"\uf102  Show Console  \uf102"))
				ConsoleWidget::Instance().AppearHideToggle();
		}
	}
};


