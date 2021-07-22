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
		static VisWidget & Instance() { static VisWidget Widget; return Widget; }
		
		bool LayerVisible_MSA = false;
		bool LayerVisible_AvoidanceZones = false;
		bool LayerVisible_SafeLandingZones = false;
		bool LayerVisible_SurveyRegion = false;
		bool LayerVisible_GuidanceOverlay = false;
		
		//All opacities are in the range 0 to 100
		float Opacity_MSA = 100.0f;
		float Opacity_AvoidanceZones = 100.0f;
		float Opacity_SafeLandingZones = 100.0f;
		float Opacity_SurveyRegion = 100.0f;
		float Opacity_GuidanceOverlay = 100.0f;
		
		float MSA_CmapMinVal = 0.0f;   //MSA corresponding to low end of colormap (m)
		float MSA_CmapMaxVal = 400.0f; //MSA corresponding to high end of colormap (m)
		int   MSA_NoFlyDrawMode = 0;   //0 = Transparrent, 1 = Red Stripes
		
		std::array<float, 3> SurveyRegionColor = {0.0f, 0.0f, 0.8f}; //RGB in range 0 to 1
		float SurveyRegionVertexRadius = 5.0f;  //Vertex radius (in pixels) when editing.
		float SurveyRegionEdgeThickness = 3.0f; //Edge thickness (in pixels) when editing.
		
		bool GuidanceOverlay_ShowMessageBox = true;
		bool GuidanceOverlay_ShowTrianglesInsteadOfPartition = false;
		
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
			        CEREAL_NVP(Opacity_MSA),
			        CEREAL_NVP(Opacity_AvoidanceZones),
			        CEREAL_NVP(Opacity_SafeLandingZones),
			        CEREAL_NVP(Opacity_SurveyRegion),
			        CEREAL_NVP(Opacity_GuidanceOverlay),
			        CEREAL_NVP(MSA_CmapMinVal),
			        CEREAL_NVP(MSA_CmapMaxVal),
			        CEREAL_NVP(MSA_NoFlyDrawMode),
			        CEREAL_NVP(SurveyRegionColor),
			        CEREAL_NVP(SurveyRegionVertexRadius),
			        CEREAL_NVP(SurveyRegionEdgeThickness),
			        CEREAL_NVP(GuidanceOverlay_ShowMessageBox),
			        CEREAL_NVP(GuidanceOverlay_ShowTrianglesInsteadOfPartition));
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
	LayerVisible_MSA = false;
	LayerVisible_AvoidanceZones = false;
	LayerVisible_SafeLandingZones = false;
	LayerVisible_SurveyRegion = false;
	LayerVisible_GuidanceOverlay = false;
	
	Opacity_MSA = 100.0f;
	Opacity_AvoidanceZones = 100.0f;
	Opacity_SafeLandingZones = 100.0f;
	Opacity_SurveyRegion = 100.0f;
	Opacity_GuidanceOverlay = 100.0f;
	
	MSA_CmapMinVal = 0.0f;
	MSA_CmapMaxVal = 400.0f;
	MSA_NoFlyDrawMode = 0;
	
	SurveyRegionColor = {0.0f, 0.0f, 0.8f};
	SurveyRegionVertexRadius = 5.0f;
	SurveyRegionEdgeThickness = 3.0f;
	
	GuidanceOverlay_ShowMessageBox = true;
	GuidanceOverlay_ShowTrianglesInsteadOfPartition = false;
}

//Make sure all vis parameters are reasonable
inline void VisWidget::SanitizeState(void) {
	Opacity_MSA              = std::max(std::min(Opacity_MSA,              100.0f), 0.0f);
	Opacity_AvoidanceZones   = std::max(std::min(Opacity_AvoidanceZones,   100.0f), 0.0f);
	Opacity_SafeLandingZones = std::max(std::min(Opacity_SafeLandingZones, 100.0f), 0.0f);
	Opacity_SurveyRegion     = std::max(std::min(Opacity_SurveyRegion,     100.0f), 0.0f);
	Opacity_GuidanceOverlay  = std::max(std::min(Opacity_GuidanceOverlay,  100.0f), 0.0f);
	
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
			
			float col2Start = ImGui::GetCursorPosX() + ImGui::CalcTextSize("Show Region Decomposition  ").x;
			ImGui::TextUnformatted("Show Message Box ");
			ImGui::SameLine(col2Start);
			ImGui::Checkbox("##Show Message Box", &GuidanceOverlay_ShowMessageBox);
			
			ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
			
			ImGui::TextUnformatted("Show Region Partition ");
			ImGui::SameLine(col2Start);
			if (ImGui::RadioButton("##Show Partition ", !GuidanceOverlay_ShowTrianglesInsteadOfPartition))
				GuidanceOverlay_ShowTrianglesInsteadOfPartition = false;
			
			ImGui::TextUnformatted("Show Region Decomposition ");
			ImGui::SameLine(col2Start);
			if (ImGui::RadioButton("##Show Decomposition ", GuidanceOverlay_ShowTrianglesInsteadOfPartition))
				GuidanceOverlay_ShowTrianglesInsteadOfPartition = true;
			
			//ImGui::TextUnformatted("Show Triangles ");
			//ImGui::SameLine(col2Start);
			//ImGui::Checkbox("##Show Triangulation", &GuidanceOverlay_ShowTrianglesInsteadOfPartition);
			
			ImGui::EndPopup();
		}
	}
}





