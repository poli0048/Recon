//This widget exposes layer visibility and visualization parameters for data layers
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved.
#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <algorithm>

//External Includes
#include "../HandyImGuiInclude.hpp"

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
		//This object should only be accessed from the main draw thread - no locks are neaded if this is respected.
		//This generally means only "Draw" methods in UI classes should access the singleton (since these are invoked by the main thread)
		static VisWidget & Instance() { static VisWidget Widget; return Widget; }
		
		bool LayerVisible_MSA;
		bool LayerVisible_AvoidanceZones;
		bool LayerVisible_SafeLandingZones;
		bool LayerVisible_SurveyRegion;
		bool LayerVisible_GuidanceOverlay;
		bool LayerVisible_ShadowMapOverlay;
		bool LayerVisible_TimeAvailableOverlay;
		
		//All opacities are in the range 0 to 100
		float Opacity_MSA;
		float Opacity_AvoidanceZones;
		float Opacity_SafeLandingZones;
		float Opacity_SurveyRegion;
		float Opacity_GuidanceOverlay;
		float Opacity_ShadowMapOverlay;
		float Opacity_TimeAvailableOverlay;
		
		float MSA_CmapMinVal;    //MSA corresponding to low end of colormap (m)
		float MSA_CmapMaxVal;    //MSA corresponding to high end of colormap (m)
		int   MSA_NoFlyDrawMode; //0 = Transparrent, 1 = Red Stripes
		
		std::array<float, 3> SurveyRegionColor; //RGB in range 0 to 1
		float SurveyRegionVertexRadius;         //Vertex radius (in pixels) when editing.
		float SurveyRegionEdgeThickness;        //Edge thickness (in pixels) when editing.
		
		int  GuidanceOverlay_View; //0=Partition, 1=Triangles, 2=Sequences
		bool GuidanceOverlay_ShowMissions; //Relavent when view is "Sequences"
		bool GuidanceOverlay_HideCompleteSubregions; //Relavent when view is "Sequences"
		
		std::array<float, 3> ShadowMapColor; //RGB in range 0 to 1
		
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
			        CEREAL_NVP(LayerVisible_GuidanceOverlay),
			        CEREAL_NVP(LayerVisible_ShadowMapOverlay),
			        CEREAL_NVP(LayerVisible_TimeAvailableOverlay),
			        CEREAL_NVP(Opacity_MSA),
			        CEREAL_NVP(Opacity_AvoidanceZones),
			        CEREAL_NVP(Opacity_SafeLandingZones),
			        CEREAL_NVP(Opacity_SurveyRegion),
			        CEREAL_NVP(Opacity_GuidanceOverlay),
			        CEREAL_NVP(Opacity_ShadowMapOverlay),
			        CEREAL_NVP(Opacity_TimeAvailableOverlay),
			        CEREAL_NVP(MSA_CmapMinVal),
			        CEREAL_NVP(MSA_CmapMaxVal),
			        CEREAL_NVP(MSA_NoFlyDrawMode),
			        CEREAL_NVP(SurveyRegionColor),
			        CEREAL_NVP(SurveyRegionVertexRadius),
			        CEREAL_NVP(SurveyRegionEdgeThickness),
			        CEREAL_NVP(GuidanceOverlay_View),
			        CEREAL_NVP(GuidanceOverlay_ShowMissions),
			        CEREAL_NVP(GuidanceOverlay_HideCompleteSubregions),
			        CEREAL_NVP(ShadowMapColor));
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
		Log.print("Warning in VisWidget::LoadFromDisk: Could not open file for reading. Loading defaults.");
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
	LayerVisible_MSA                  = true;
	LayerVisible_AvoidanceZones       = true;
	LayerVisible_SafeLandingZones     = true;
	LayerVisible_SurveyRegion         = true;
	LayerVisible_GuidanceOverlay      = true;
	LayerVisible_ShadowMapOverlay     = true;
	LayerVisible_TimeAvailableOverlay = true;
	
	Opacity_MSA                  = 50.0f;
	Opacity_AvoidanceZones       = 50.0f;
	Opacity_SafeLandingZones     = 50.0f;
	Opacity_SurveyRegion         = 50.0f;
	Opacity_GuidanceOverlay      = 50.0f;
	Opacity_ShadowMapOverlay     = 50.0f;
	Opacity_TimeAvailableOverlay = 50.0f;
	
	MSA_CmapMinVal = 0.0f;
	MSA_CmapMaxVal = 400.0f;
	MSA_NoFlyDrawMode = 0;
	
	SurveyRegionColor = {0.8f, 0.8f, 0.0f};
	SurveyRegionVertexRadius = 5.0f;
	SurveyRegionEdgeThickness = 3.0f;
	
	GuidanceOverlay_View = 0;
	GuidanceOverlay_ShowMissions = true;
	GuidanceOverlay_HideCompleteSubregions = true;
	
	ShadowMapColor = {0.4f, 0.4f, 0.7f};
}

//Make sure all vis parameters are reasonable
inline void VisWidget::SanitizeState(void) {
	Opacity_MSA                  = std::clamp(Opacity_MSA,                  0.0f, 100.0f);
	Opacity_AvoidanceZones       = std::clamp(Opacity_AvoidanceZones,       0.0f, 100.0f);
	Opacity_SafeLandingZones     = std::clamp(Opacity_SafeLandingZones,     0.0f, 100.0f);
	Opacity_SurveyRegion         = std::clamp(Opacity_SurveyRegion,         0.0f, 100.0f);
	Opacity_GuidanceOverlay      = std::clamp(Opacity_GuidanceOverlay,      0.0f, 100.0f);
	Opacity_ShadowMapOverlay     = std::clamp(Opacity_ShadowMapOverlay,     0.0f, 100.0f);
	Opacity_TimeAvailableOverlay = std::clamp(Opacity_TimeAvailableOverlay, 0.0f, 100.0f);
	
	MSA_CmapMinVal    = std::clamp(MSA_CmapMinVal, -1000.0f, 10000.0f);
	MSA_CmapMaxVal    = std::clamp(MSA_CmapMaxVal, -1000.0f, 10000.0f);
	MSA_NoFlyDrawMode = std::clamp(MSA_NoFlyDrawMode, 0, 1);
	
	SurveyRegionColor[0]      = std::clamp(SurveyRegionColor[0],      0.0f, 1.0f);
	SurveyRegionColor[1]      = std::clamp(SurveyRegionColor[1],      0.0f, 1.0f);
	SurveyRegionColor[2]      = std::clamp(SurveyRegionColor[2],      0.0f, 1.0f);
	SurveyRegionVertexRadius  = std::clamp(SurveyRegionVertexRadius,  1.0f, 20.0f);
	SurveyRegionEdgeThickness = std::clamp(SurveyRegionEdgeThickness, 1.0f, 10.0f);
	
	GuidanceOverlay_View = std::clamp(GuidanceOverlay_View, 0, 3);

	ShadowMapColor[0] = std::clamp(ShadowMapColor[0], 0.0f, 1.0f);
	ShadowMapColor[1] = std::clamp(ShadowMapColor[1], 0.0f, 1.0f);
	ShadowMapColor[2] = std::clamp(ShadowMapColor[2], 0.0f, 1.0f);
}

inline void VisWidget::Draw() {
	MyGui::HeaderLabel("Layer Visibility and Parameters");
	float checkBoxXPos       = ImGui::GetWindowContentRegionMax().x - 3.0f*ImGui::GetFontSize();
	float settingsButtonXPos = ImGui::GetWindowContentRegionMax().x - 1.4f*ImGui::GetFontSize();
	
	ImGui::TextUnformatted("Min Safe Altitude (m)");
	ImGui::SameLine(checkBoxXPos);
	ImGui::Checkbox("##Min Safe Altitude (m)", &LayerVisible_MSA);
	ImGui::SameLine(settingsButtonXPos);
	if (ImGui::Button("\uf013##Min Safe Altitude Settings"))
		ImGui::OpenPopup("Min Safe Altitude Settings");
	{
		ImExt::Style styleSitter(StyleVar::WindowPadding, Math::Vector2(4.0f));
		if (ImGui::BeginPopup("Min Safe Altitude Settings", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			float col2Start = ImGui::GetCursorPosX() + ImGui::CalcTextSize("Colormap Max Value  ").x;
			std::string label = "Opacity  "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::Text(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			ImGui::SliderFloat("##Opacity_MSA", &Opacity_MSA, 0.0f, 100.0f, "%.0f");
			
			label = "Colormap Min Value  "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::Text(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			ImGui::DragFloat("##Colormap Min Value", &MSA_CmapMinVal, 0.1f, -1000.0f, 10000.0f, "%.1f");
			
			label = "Colormap Max Value  "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::Text(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			ImGui::DragFloat("##Colormap Max Value", &MSA_CmapMaxVal, 0.1f, -1000.0f, 10000.0f, "%.1f");
			
			label = "No Fly Zones  "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::Text(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			ImGui::Combo("## NoFlyDrawMode", &MSA_NoFlyDrawMode, "Transparent\0Stripes\0");
			
			ImGui::EndPopup();
		}
	}
	
	
	ImGui::TextUnformatted("Avoidance Zones");
	ImGui::SameLine(checkBoxXPos);
	ImGui::Checkbox("##Avoidance Zones", &LayerVisible_AvoidanceZones);
	ImGui::SameLine(settingsButtonXPos);
	if (ImGui::Button("\uf013##Avoidance Zones Settings"))
		ImGui::OpenPopup("Avoidance Zones Settings");
	{
		ImExt::Style styleSitter(StyleVar::WindowPadding, Math::Vector2(4.0f));
		if (ImGui::BeginPopup("Avoidance Zones Settings", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			float col2Start = ImGui::GetCursorPosX() + ImGui::CalcTextSize("Opacity  ").x;
			std::string label = "Opacity  "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::Text(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			ImGui::SliderFloat("##Opacity_AvoidanceZones", &Opacity_AvoidanceZones, 0.0f, 100.0f, "%.0f");
			
			ImGui::EndPopup();
		}
	}
	
	
	ImGui::TextUnformatted("Safe Landing Zones");
	ImGui::SameLine(checkBoxXPos);
	ImGui::Checkbox("##Safe Landing Zones", &LayerVisible_SafeLandingZones);
	ImGui::SameLine(settingsButtonXPos);
	if (ImGui::Button("\uf013##Safe Landing Zones Settings"))
		ImGui::OpenPopup("Safe Landing Zones Settings");
	{
		ImExt::Style styleSitter(StyleVar::WindowPadding, Math::Vector2(4.0f));
		if (ImGui::BeginPopup("Safe Landing Zones Settings", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			float col2Start = ImGui::GetCursorPosX() + ImGui::CalcTextSize("Opacity  ").x;
			std::string label = "Opacity  "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::Text(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			ImGui::SliderFloat("##Opacity_SafeLandingZones", &Opacity_SafeLandingZones, 0.0f, 100.0f, "%.0f");
			
			ImGui::EndPopup();
		}
	}

	
	ImGui::TextUnformatted("Survey Region");
	ImGui::SameLine(checkBoxXPos);
	ImGui::Checkbox("##Survey Region", &LayerVisible_SurveyRegion);
	ImGui::SameLine(settingsButtonXPos);
	if (ImGui::Button("\uf013##Survey Region Settings"))
		ImGui::OpenPopup("Survey Region Settings");
	{
		ImExt::Style styleSitter(StyleVar::WindowPadding, Math::Vector2(4.0f));
		if (ImGui::BeginPopup("Survey Region Settings", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			float col2Start = ImGui::GetCursorPosX() + ImGui::CalcTextSize("Edge Thickness (when editing)  ").x;
			std::string label = "Opacity  "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::Text(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			ImGui::SliderFloat("##Opacity_SurveyRegion", &Opacity_SurveyRegion, 0.0f, 100.0f, "%.0f");
			
			label = "Color "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::Text(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::ColorEdit3("##SurveyRegionColor", &(SurveyRegionColor[0]), ImGuiColorEditFlags_NoInputs);
			
			label = "Vertex Radius (when editing) "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::Text(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			if (ImGui::DragFloat("##SurveyRegionVertexRadius", &SurveyRegionVertexRadius, 0.01f, 1.0f, 20.0f, "%.1f"))
				SurveyRegionVertexRadius = std::max(std::min(SurveyRegionVertexRadius, 20.0f), 1.0f);
			
			label = "Edge Thickness (when editing) "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::Text(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			if (ImGui::DragFloat("##SurveyRegionEdgeThickness", &SurveyRegionEdgeThickness, 0.01f, 1.0f, 10.0f, "%.1f"))
				SurveyRegionEdgeThickness = std::max(std::min(SurveyRegionEdgeThickness, 10.0f), 1.0f);
			
			ImGui::EndPopup();
		}
	}
	
	ImGui::TextUnformatted("Guidance Overlay");
	ImGui::SameLine(checkBoxXPos);
	ImGui::Checkbox("##Guidance Overlay", &LayerVisible_GuidanceOverlay);
	ImGui::SameLine(settingsButtonXPos);
	if (ImGui::Button("\uf013##Guidance Overlay Settings"))
		ImGui::OpenPopup("Guidance Overlay Settings");
	{
		ImExt::Style styleSitter(StyleVar::WindowPadding, Math::Vector2(4.0f));
		if (ImGui::BeginPopup("Guidance Overlay Settings", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			ImGui::TextUnformatted("Opacity  ");
			ImGui::SameLine();
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			ImGui::SliderFloat("##Opacity_GuidanceOverlay", &Opacity_GuidanceOverlay, 0.0f, 100.0f, "%.0f");
			
			ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
			float col2Start = ImGui::GetCursorPosX() + ImGui::CalcTextSize("Current Plans & Missions  ").x;

			ImGui::TextUnformatted("Region Partition ");
			ImGui::SameLine(col2Start);
			ImGui::RadioButton("##Region Partition ", &GuidanceOverlay_View, 0);

			ImGui::TextUnformatted("Triangle Decomposition ");
			ImGui::SameLine(col2Start);
			ImGui::RadioButton("##Triangle Decomposition ", &GuidanceOverlay_View, 1);
			
			ImGui::TextUnformatted("Progress & Plans ");
			ImGui::SameLine(col2Start);
			ImGui::RadioButton("##Progress & Plans ", &GuidanceOverlay_View, 2);

			if (GuidanceOverlay_View == 2) {
				ImGui::Separator();
				ImGui::NewLine();
				ImGui::TextUnformatted("Show Planned Missions ");
				ImGui::SameLine(col2Start);
				ImGui::Checkbox("##Show Planned Missions", &GuidanceOverlay_ShowMissions);

				ImGui::TextUnformatted("Hide Completed Areas ");
				ImGui::SameLine(col2Start);
				ImGui::Checkbox("##Hide Completed Areas", &GuidanceOverlay_HideCompleteSubregions);
			}

			//ImGui::TextUnformatted("Current Plans & Missions ");
			//ImGui::SameLine(col2Start);
			//ImGui::RadioButton("##Current Plans & Missions ", &GuidanceOverlay_View, 3);
			
			ImGui::EndPopup();
		}
	}
	
	ImGui::TextUnformatted("Shadow Map Overlay");
	ImGui::SameLine(checkBoxXPos);
	ImGui::Checkbox("##Shadow Map Overlay", &LayerVisible_ShadowMapOverlay);
	ImGui::SameLine(settingsButtonXPos);
	if (ImGui::Button("\uf013##Shadow Map Overlay Settings"))
		ImGui::OpenPopup("Shadow Map Overlay Settings");
	{
		ImExt::Style styleSitter(StyleVar::WindowPadding, Math::Vector2(4.0f));
		if (ImGui::BeginPopup("Shadow Map Overlay Settings", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			float col2Start = ImGui::GetCursorPosX() + ImGui::CalcTextSize("Opacity  ").x;
			std::string label = "Opacity  "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::TextUnformatted(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			ImGui::SliderFloat("##Opacity_ShadowMapOverlay", &Opacity_ShadowMapOverlay, 0.0f, 100.0f, "%.0f");
			
			label = "Color "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::TextUnformatted(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::ColorEdit3("##ShadowMapColor", &(ShadowMapColor[0]), ImGuiColorEditFlags_NoInputs);
			
			ImGui::EndPopup();
		}
	}
	
	ImGui::TextUnformatted("Time Available Overlay");
	ImGui::SameLine(checkBoxXPos);
	ImGui::Checkbox("##Time Available Overlay", &LayerVisible_TimeAvailableOverlay);
	ImGui::SameLine(settingsButtonXPos);
	if (ImGui::Button("\uf013##Time Available Overlay Settings"))
		ImGui::OpenPopup("Time Available Overlay Settings");
	{
		ImExt::Style styleSitter(StyleVar::WindowPadding, Math::Vector2(4.0f));
		if (ImGui::BeginPopup("Time Available Overlay Settings", ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
			float col2Start = ImGui::GetCursorPosX() + ImGui::CalcTextSize("Opacity  ").x;
			std::string label = "Opacity  "s;
			ImGui::SetCursorPosX(col2Start - ImGui::CalcTextSize(label.c_str()).x);
			ImGui::TextUnformatted(label.c_str());
			ImGui::SameLine(col2Start);
			ImGui::SetNextItemWidth(15.0f*ImGui::GetFontSize());
			ImGui::SliderFloat("##Opacity_TimeAvailableOverlay", &Opacity_TimeAvailableOverlay, 0.0f, 100.0f, "%.0f");
			
			ImGui::EndPopup();
		}
	}
}





