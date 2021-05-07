//The settings window exposes program options.
//Author: Bryan Poling
//Copyright (c) 2018 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>
#include <iostream>

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "../ProgOptions.hpp"

class SettingsWindow {
	public:
		SettingsWindow() { m_themeOptions = Themes::GetThemeStrings_OneLine(); }
		~SettingsWindow() = default;
		
		static SettingsWindow & Instance() { static SettingsWindow win; return win; }
	
		void Draw();
		
		bool Visible = false;
	
	private:
		std::string m_themeOptions;
		float defaultButtonWidth = 10.0f; //Updated in draw loop
};

inline void SettingsWindow::Draw() {
	ImExt::Window::Options wOpts;
	wOpts.Flags = WindowFlags::NoCollapse | WindowFlags::NoSavedSettings | WindowFlags::NoDocking | WindowFlags::NoTitleBar | WindowFlags::NoResize;
	wOpts.POpen = &Visible;
	if (ImExt::Window window("Settings", wOpts); window.ShouldDrawContents()) {
		//If the window is appearing, set a reasonable size
		if (window.IsAppearing()) {
			const char * widestText = "You need to restart the program for your changes to take effect.";
			float winWidth  = std::max(ImGui::CalcTextSize(widestText).x + 100.0f, 700.0f);
			float winHeight = std::max(0.5625f*winWidth, 300.0f);
			ImGui::SetWindowSize(ImVec2(winWidth, winHeight));
		}
		
		ImExt::Style style(StyleVar::WindowPadding, Math::Vector2(20.0f));
		
		ImGui::BeginChild("Options Scrollable Region", ImVec2(0,0), true, ImGuiWindowFlags_AlwaysUseWindowPadding);
		ImGui::TextUnformatted("User Interface:");
		ImGui::Dummy(ImVec2(1, 10));
		
		float col2Start = ImGui::GetCursorPosX() + ImGui::CalcTextSize("User Interface Theme:").x + 15.0f;
		float col3Start = std::max(ImGui::GetContentRegionAvail().x - defaultButtonWidth, col2Start + 50.0f);
		float sliderWidth = std::max(col3Start - col2Start - 10.0f, 50.0f);
		
		ImGui::TextUnformatted("Interface Scale:");
		ImGui::SameLine(col2Start);
		ImGui::PushItemWidth(sliderWidth);
		ImGui::SliderFloat("##UI Scale", &(ProgOptions::Instance()->UIScaleFactor), 0.25f, 4.0f, "%.2f", 1.0f);
		if (ImGui::IsItemDeactivatedAfterEdit())
			ImGuiApp::Instance().SetScaling(ProgOptions::Instance()->UIScaleFactor);
		ImGui::PopItemWidth();
		ImGui::SameLine(col3Start);
		if (ImGui::Button(" Default ##UI Scale")) {
			ProgOptions::Instance()->UIScaleFactor = 1.0f;
			ImGuiApp::Instance().SetScaling(ProgOptions::Instance()->UIScaleFactor);
		}
		defaultButtonWidth = ImGui::GetItemRectSize().x;
		
		ImGui::TextUnformatted("Theme:");
		ImGui::SameLine(col2Start);
		ImGui::PushItemWidth(sliderWidth);
		{
			ImExt::Style popupStyle(StyleVar::WindowPadding, Math::Vector2(4.0f, 4.0f));
			int selection = Themes::ThemeToSelectionNum(ProgOptions::Instance()->UITheme);
			ImGui::Combo("## UI Theme Combo", &selection, m_themeOptions.c_str());
			ProgOptions::Instance()->UITheme = Themes::SelectionNumToTheme(selection);
		}
		ImGui::PopItemWidth();
		ImGui::SameLine(col3Start);
		if (ImGui::Button(" Default ##UI Theme"))
			ProgOptions::Instance()->UITheme = Themes::Theme::Dark;
		
		ImGui::TextUnformatted("Drone Icon Size:");
		ImGui::SameLine(col2Start);
		ImGui::PushItemWidth(sliderWidth);
		ImGui::SliderFloat("##Drone Icon Size", &(ProgOptions::Instance()->DroneIconScale), 0.25f, 2.0f, "%.2f", 1.0f);
		ImGui::PopItemWidth();
		ImGui::SameLine(col3Start);
		if (ImGui::Button(" Default ##Drone Icon Size"))
			ProgOptions::Instance()->DroneIconScale = 0.5f;
		
		ImGui::Dummy(ImVec2(1, ImGui::GetFontSize()));
		ImGui::TextUnformatted("Map Behavior:");
		ImGui::Dummy(ImVec2(1, 10));
		
		ImGui::TextUnformatted("Map DPI ");
		ImGui::SameLine();
		ImGui::TextDisabled(u8"\uf059");
		if (ImGui::IsItemHovered()) {
			ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::TextUnformatted("100%: Full resolution.\n50%: 1 tile pixel takes up 2 pixels x 2 pixels on your screen.\n\n"
			                       "Reducing the map DPI improves UI responsiveness (recommended for computers with high resolution displays "
			                       "and slower integrated graphics). This setting does not prevent you from viewing mosaics at their full resolution. "
			                       "Instead it adjusts the map zoom levels at which different resolution imagery is loaded. If you zoom in enough "
			                       "you will always be able to see the highest resolution imagery.");
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
		ImGui::SameLine();
		ImGui::TextUnformatted(":");
		ImGui::SameLine(col2Start);
		ImGui::PushItemWidth(sliderWidth);
		ImGui::SliderFloat("##MapDPISilder", &(ProgOptions::Instance()->MapDPIPercentage), 50.0f, 100.0f, "%.0f %%");
		ImGui::PopItemWidth();
		ImGui::SameLine(col3Start);
		if (ImGui::Button(" Default ##Map DPI"))
			ProgOptions::Instance()->MapDPIPercentage = 100.0f;
		
		ImGui::TextUnformatted("Zoom Speed:");
		ImGui::SameLine(col2Start);
		ImGui::PushItemWidth(sliderWidth);
		ImGui::SliderFloat("##Zoom speed", &(ProgOptions::Instance()->zoomSpeed), 0.125f, 8.0f, "%.2f", 1.0f);
		ImGui::PopItemWidth();
		ImGui::SameLine(col3Start);
		if (ImGui::Button(" Default ##Zoom speed"))
			ProgOptions::Instance()->zoomSpeed = 1.0f;
		
		ImGui::Dummy(ImVec2(1, ImGui::GetFontSize()));
		if (ImGui::Button("      Close      "))
			Visible = false;
		
		ImGui::SameLine();
		ImGui::Dummy(ImVec2(0.4f*ImGui::GetFontSize(), 1.0f));
		ImGui::SameLine();
		ImGui::TextDisabled(u8"\uf059");
		if (ImGui::IsItemHovered()) {
			ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::TextUnformatted("Changes take effect immediately and are saved automatically.");
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
		
		ImGui::EndChild();
	}
}




