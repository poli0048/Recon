//This widget lists connected vehicles. It doesn't do anything else yet - more forward-looking
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>
#include <chrono>

//External Includes
#include "../../../handycpp/Handy.hpp"
#include "../../../imgui/app/ImGuiApp.hpp"

//Project Includes
#include "ReconUI.hpp"
#include "MyGui.hpp"

class VehiclesWidget {
	public:
		static VehiclesWidget & Instance() { static VehiclesWidget Widget; return Widget; }
		
		//Constructors and Destructors
		VehiclesWidget() : Log(*(ReconUI::Instance().Log)) { }
		~VehiclesWidget() = default;
		
		//Public accessors for getting the state of the window
		float GetWidgetRecommendedHeight(void) { return RecommendedHeight; } //Typically equals ContentHeight, but changes smoothly with time for animation.

		inline void Draw();
	
	private:
		Journal & Log;
		float ContentHeight; //Height of widget content from last draw pass
		float RecommendedHeight; //Recommended height for widget
};

inline void VehiclesWidget::Draw() {
	float cursorStartYPos = ImGui::GetCursorPos().y;
	MyGui::HeaderLabel("Connected Vehicles");
	
	
	//After drawing, update content height and recommended widget height
	ContentHeight = ImGui::GetCursorPos().y - cursorStartYPos;
	RecommendedHeight = 0.85f*RecommendedHeight + 0.15f*ContentHeight;
	if (std::abs(RecommendedHeight - ContentHeight) < 1.0f)
		RecommendedHeight = ContentHeight;
}


