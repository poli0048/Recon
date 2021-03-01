//This widget exposes layer visibility and visualization parameters for data layers
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

//Cereal Includes
#include "cereal/types/vector.hpp"
#include "cereal/types/array.hpp"
#include "cereal/types/string.hpp"
#include "cereal/archives/json.hpp"

//Project Includes
#include "ReconUI.hpp"
#include "MyGui.hpp"

class VisWidget {
	public:
		static VisWidget & Instance() { static VisWidget Widget; return Widget; }
		
		bool LayerVisible_MSA = false;
		bool LayerVisible_AvoidanceZones = false;
		bool LayerVisible_SafeLandingZones = false;
		bool LayerVisible_SurveyRegion = false;
		
		//All opacities are in the range 0 to 100
		float Opacity_MSA = 100.0f;
		float Opacity_AvoidanceZones = 100.0f;
		float Opacity_SafeLandingZones = 100.0f;
		float Opacity_SurveyRegion = 100.0f;
		
		float MSA_CmapMinVal = 0.0f;   //MSA corresponding to low end of colormap (m)
		float MSA_CmapMaxVal = 400.0f; //MSA corresponding to high end of colormap (m)
		int   MSA_NoFlyDrawMode = 0;   //0 = Transparrent, 1 = Red Stripes
		
		std::array<float, 3> SurveyRegionColor = {0.0f, 0.0f, 0.8f}; //RGB in range 0 to 1
		float SurveyRegionVertexRadius = 5.0f;  //Vertex radius (in pixels) when editing.
		float SurveyRegionEdgeThickness = 3.0f; //Edge thickness (in pixels) when editing.
		
		//Constructors and Destructors
		VisWidget() : Log(*(ReconUI::Instance().Log)) { LoadDefaults(); LoadFromDisk(); }
		~VisWidget() { SaveToDisk(); }
		
		inline void Draw();
		
		//Tell Cereal which members to serialize (must be public)
		template<class Archive> void serialize(Archive & archive) {
			archive(CEREAL_NVP(LayerVisible_MSA),
			        CEREAL_NVP(LayerVisible_AvoidanceZones),
			        CEREAL_NVP(LayerVisible_SafeLandingZones),
			        CEREAL_NVP(LayerVisible_SurveyRegion),
			        CEREAL_NVP(Opacity_MSA),
			        CEREAL_NVP(Opacity_AvoidanceZones),
			        CEREAL_NVP(Opacity_SafeLandingZones),
			        CEREAL_NVP(Opacity_SurveyRegion),
			        CEREAL_NVP(MSA_CmapMinVal),
			        CEREAL_NVP(MSA_CmapMaxVal),
			        CEREAL_NVP(MSA_NoFlyDrawMode),
			        CEREAL_NVP(SurveyRegionColor),
			        CEREAL_NVP(SurveyRegionVertexRadius),
			        CEREAL_NVP(SurveyRegionEdgeThickness));
		}
	
	private:
		Journal & Log;
		
		inline void SaveToDisk(void);     //Immediately save current object
		inline void LoadFromDisk(void);   //Immediately load from disk, overwriting current object
		inline void LoadDefaults(void);   //Set all parameters to defaults (good fallback if file loading fails)
		inline void SanitizeState(void);  //Make sure all vis parameters are reasonable
};

//Immediately save current object
inline void VisWidget::SaveToDisk(void) {
	std::string filePath = (Handy::Paths::CacheDirectory("SentekRecon") / "VisParams.txt").string();
	std::ofstream fileStream(filePath, std::ofstream::out | std::ofstream::binary);
	if (! fileStream.is_open())
		Log.print("Error in VisWidget::SaveToDisk: Could not open file for writing.");
	else {
		try {
	  		cereal::JSONOutputArchive oArchive( fileStream );
	  		oArchive(*this);
	  	}
	  	catch (...) { Log.print("Error in VisWidget::SaveToDisk: Writing to Cereal archive failed."); }
	}
}

//Immediately load from disk, overwriting current object
inline void VisWidget::LoadFromDisk(void) {
	std::string filePath = (Handy::Paths::CacheDirectory("SentekRecon") / "VisParams.txt").string();
	std::ifstream fileStream(filePath, std::ifstream::in | std::ifstream::binary);
	if (! fileStream.is_open()) {
		Log.print("Error in VisWidget::LoadFromDisk: Could not open file for reading.");
		LoadDefaults();
	}
	else {
		try {
			cereal::JSONInputArchive iArchive( fileStream );
			iArchive(*this);
		}
		catch (...) {
			Log.print("Error in VisWidget::LoadFromDisk: Reading from Cereal archive failed - loading defaults.");
			LoadDefaults();
		}
	}
	SanitizeState();
}

//Set all parameters to defaults (good fallback if file loading fails)
inline void VisWidget::LoadDefaults(void) {
	LayerVisible_MSA = false;
	LayerVisible_AvoidanceZones = false;
	LayerVisible_SafeLandingZones = false;
	LayerVisible_SurveyRegion = false;
	
	Opacity_MSA = 100.0f;
	Opacity_AvoidanceZones = 100.0f;
	Opacity_SafeLandingZones = 100.0f;
	Opacity_SurveyRegion = 100.0f;
	
	MSA_CmapMinVal = 0.0f;
	MSA_CmapMaxVal = 400.0f;
	MSA_NoFlyDrawMode = 0;
	
	SurveyRegionColor = {0.0f, 0.0f, 0.8f};
	SurveyRegionVertexRadius = 5.0f;
	SurveyRegionEdgeThickness = 3.0f;
}

//Make sure all vis parameters are reasonable
inline void VisWidget::SanitizeState(void) {
	Opacity_MSA = std::max(std::min(Opacity_MSA, 100.0f), 0.0f);
	Opacity_AvoidanceZones = std::max(std::min(Opacity_AvoidanceZones, 100.0f), 0.0f);
	Opacity_SafeLandingZones = std::max(std::min(Opacity_SafeLandingZones, 100.0f), 0.0f);
	Opacity_SurveyRegion = std::max(std::min(Opacity_SurveyRegion, 100.0f), 0.0f);
	
	MSA_CmapMinVal = std::max(std::min(MSA_CmapMinVal, 10000.0f), -1000.0f);
	MSA_CmapMaxVal = std::max(std::min(MSA_CmapMaxVal, 10000.0f), -1000.0f);
	MSA_NoFlyDrawMode = std::max(std::min(MSA_NoFlyDrawMode, 1), 0);
	
	SurveyRegionColor[0] = std::max(std::min(SurveyRegionColor[0], 1.0f), 0.0f);
	SurveyRegionColor[1] = std::max(std::min(SurveyRegionColor[1], 1.0f), 0.0f);
	SurveyRegionColor[2] = std::max(std::min(SurveyRegionColor[2], 1.0f), 0.0f);
	SurveyRegionVertexRadius = std::max(std::min(SurveyRegionVertexRadius, 20.0f), 1.0f);
	SurveyRegionEdgeThickness = std::max(std::min(SurveyRegionEdgeThickness, 10.0f), 1.0f);
}

inline void VisWidget::Draw() {
	MyGui::HeaderLabel("Layer Visibility and Parameters");
	
	ImGui::Checkbox("Min Safe Altitude (m)", &LayerVisible_MSA);
	if (LayerVisible_MSA) {
		ImGui::Text("Opacity");
		ImGui::SetNextItemWidth(-1.0f);
		ImGui::SliderFloat("##Opacity_MSA", &Opacity_MSA, 0.0f, 100.0f, "%.0f");
		
		ImGui::Text("Colormap Min Value: ");
		ImGui::SetNextItemWidth(-1.0f);
		ImGui::DragFloat("##Colormap Min Value", &MSA_CmapMinVal, 0.1f, -1000.0f, 10000.0f, "%.1f");
		
		ImGui::Text("Colormap Max Value: ");
		ImGui::SetNextItemWidth(-1.0f);
		ImGui::DragFloat("##Colormap Max Value", &MSA_CmapMaxVal, 0.1f, -1000.0f, 10000.0f, "%.1f");
		
		/*if (ImGui::Button(" Colormap Auto ")) {
			//TODO: This is actually a bit more involved than it seems. We can't go over every pixel in every tile over large areas.
			//We could just limit this to working at certain zoom levels, but regardless this is a perk, not an essential feature.
		}
		ImGui::SameLine();
		ImGui::TextDisabled(u8"\uf059");
		if (ImGui::IsItemHovered()) {
			ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::TextUnformatted("Adjust the colormap range automatically based on the current view. The colormap min value will be "
			                       "set to the lowest min safe altitude currently in view, and the colormap max value will be set to the "
			                       "highest min safe altitude currently in view.");
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}*/
		
		{
			ImGui::Text("No Fly Zones: ");
			ImExt::Style popupStyle(StyleVar::WindowPadding, Math::Vector2(4.0f, 4.0f));
			ImGui::Combo("## NoFlyDrawMode", &MSA_NoFlyDrawMode, "Transparent\0Stripes\0");
		}
		
		ImGui::Dummy(ImVec2(1.0f, ImGui::GetFontSize()));
	}
	
	ImGui::Checkbox("Avoidance Zones", &LayerVisible_AvoidanceZones);
	if (LayerVisible_AvoidanceZones) {
		ImGui::Text("Opacity");
		ImGui::SetNextItemWidth(-1.0f);
		ImGui::SliderFloat("##Opacity_AvoidanceZones", &Opacity_AvoidanceZones, 0.0f, 100.0f, "%.0f");
		
		ImGui::Dummy(ImVec2(1.0f, ImGui::GetFontSize()));
	}
	
	ImGui::Checkbox("Safe Landing Zones", &LayerVisible_SafeLandingZones);
	if (LayerVisible_SafeLandingZones) {
		ImGui::Text("Opacity");
		ImGui::SetNextItemWidth(-1.0f);
		ImGui::SliderFloat("##Opacity_SafeLandingZones", &Opacity_SafeLandingZones, 0.0f, 100.0f, "%.0f");
		
		ImGui::Dummy(ImVec2(1.0f, ImGui::GetFontSize()));
	}
	
	ImGui::Checkbox("Survey Region", &LayerVisible_SurveyRegion);
	if (LayerVisible_SurveyRegion) {
		ImGui::Text("Opacity");
		ImGui::SetNextItemWidth(-1.0f);
		ImGui::SliderFloat("##Opacity_SurveyRegion", &Opacity_SurveyRegion, 0.0f, 100.0f, "%.0f");
		//ImGui::SetNextItemWidth(-1.0f);
		ImGui::Text("Color: ");
		ImGui::SameLine();
		ImGui::ColorEdit3("##SurveyRegionColor", &(SurveyRegionColor[0]), ImGuiColorEditFlags_NoInputs);
		
		ImGui::Text("Vertex Radius (when editing): ");
		ImGui::SetNextItemWidth(-1.0f);
		if (ImGui::DragFloat("##SurveyRegionVertexRadius", &SurveyRegionVertexRadius, 0.01f, 1.0f, 20.0f, "%.1f"))
			SurveyRegionVertexRadius = std::max(std::min(SurveyRegionVertexRadius, 20.0f), 1.0f);
		
		ImGui::Text("Edge Thickness (when editing): ");
		ImGui::SetNextItemWidth(-1.0f);
		if (ImGui::DragFloat("##SurveyRegionEdgeThickness", &SurveyRegionEdgeThickness, 0.01f, 1.0f, 10.0f, "%.1f"))
			SurveyRegionEdgeThickness = std::max(std::min(SurveyRegionEdgeThickness, 10.0f), 1.0f);
		
		ImGui::Dummy(ImVec2(1.0f, ImGui::GetFontSize()));
	}
}


