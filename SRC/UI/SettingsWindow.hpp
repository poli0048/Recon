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
#include "../Modules/GNSS-Receiver/GNSSReceiver.hpp"

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
	wOpts.Size(Math::Vector2(35.0f*ImGui::GetFontSize(), 35.0f*ImGui::GetFontSize()), Condition::Appearing);
	if (ImExt::Window window("Settings", wOpts); window.ShouldDrawContents()) {
		ImExt::Style style(StyleVar::WindowPadding, Math::Vector2(20.0f));
		
		ImGui::BeginChild("Options Scrollable Region", ImVec2(0,0), true, ImGuiWindowFlags_AlwaysUseWindowPadding);
		ImGui::TextUnformatted("User Interface:");
		ImGui::Dummy(ImVec2(1, 10));
		
		float col2Start = ImGui::GetCursorPosX() + ImGui::CalcTextSize("  High Visibility Cursor:  ").x + 15.0f;
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
		
		ImGui::TextUnformatted("High Visibility Cursor:");
		ImGui::SameLine(col2Start);
		ImGui::Checkbox("##High Contrast Cursor", &(ProgOptions::Instance()->HighContrastCursor));
		ImGui::SameLine(col3Start);
		if (ImGui::Button(" Default ##High Contrast Cursor"))
			ProgOptions::Instance()->HighContrastCursor = false;
		
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
			                       "and slower integrated graphics).");
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
		ImGui::TextUnformatted("GNSS Receiver Setting:");
		ImGui::Dummy(ImVec2(1, 10));
		ImGui::TextUnformatted("GNSS Enabled ");
		ImGui::SameLine();
		ImGui::TextDisabled(u8"\uf059");
		if (ImGui::IsItemHovered()) {
			ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::TextUnformatted("If enabled, Recon will look for a supported GNSS receiver on the specified port and try to interact with it. "
			                       "If a receiver is present, your location will appear on the map and in the Locations menu. This also enables "
			                       "absolute timestamps on certain collected data (such as generated shadow map files).\n\n"
			                       "Currently, only UBLOX receivers (Gen 6 and later) are supported. If you are looking for a good, supported receiver, " 
			                       "we recommend the GPS-17285 NEO-M9N breakout board from SparkFun, coupled with a Taoglas Magma AA.170.301111 "
			                       "antenna (DigiKey Part # 931-1388-ND). Connect using the boards USB-C interface.");
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
		ImGui::SameLine();
		ImGui::TextUnformatted(":");
		ImGui::SameLine(col2Start);
		ImGui::Checkbox("##GNSS-Receiver-Enabled", &(ProgOptions::Instance()->GNSSModuleEnabled));
		ImGui::SameLine(col3Start);
		if (ImGui::Button(" Default ##GNSS-Receiver-Enabled"))
			ProgOptions::Instance()->GNSSModuleEnabled = true;
		
		ImGui::TextUnformatted("Verbose ");
		ImGui::SameLine();
		ImGui::TextDisabled(u8"\uf059");
		if (ImGui::IsItemHovered()) {
			ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::TextUnformatted("If checked, the GNSS receiver module will print out diagnostic and status info to the terminal periodically. "
			                       "This can be helpful for confirming that your receiver is working and for diagnosing issues.");
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
		ImGui::SameLine();
		ImGui::TextUnformatted(":");
		ImGui::SameLine(col2Start);
		ImGui::Checkbox("##GNSS-Receiver-Verbose", &(ProgOptions::Instance()->GNSSModuleVerbose));
		ImGui::SameLine(col3Start);
		if (ImGui::Button(" Default ##GNSS-Receiver-Verbose"))
			ProgOptions::Instance()->GNSSModuleVerbose = false;
		
		ImGui::TextUnformatted("Serial Port ");
		ImGui::SameLine();
		ImGui::TextDisabled(u8"\uf059");
		if (ImGui::IsItemHovered()) {
			ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::TextUnformatted("On *NIX platforms, this is the device path corresponding to the GNSS receiver's serial interface. On Windows, "
			                       "this is the port name for the serial device (e.g. COM3).");
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
		ImGui::SameLine();
		ImGui::TextUnformatted(":");
		ImGui::SameLine(col2Start);
		ImGui::PushItemWidth(sliderWidth);
		ImGui::InputText("##GNSS-Receiver-DevicePath", &(ProgOptions::Instance()->GNSSReceiverDevicePath));
		ImGui::PopItemWidth();
		ImGui::SameLine(col3Start);
		if (ImGui::Button(" Default ##GNSS-Receiver-DevicePath"))
			ProgOptions::Instance()->GNSSReceiverDevicePath = "/dev/ttyACM0"s;
		
		ImGui::TextUnformatted("Baud Rate ");
		ImGui::SameLine();
		ImGui::TextDisabled(u8"\uf059");
		if (ImGui::IsItemHovered()) {
			ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::TextUnformatted("Baud rate for serial comms, in bits/second. Note: If you are connecting your GNSS receiver over USB, "
			                       "this setting has no effect.");
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
		ImGui::SameLine();
		ImGui::TextUnformatted(":");
		ImGui::SameLine(col2Start);
		const char * items[] = { "9600", "14400", "19200", "38400", "57600", "115200", "128000", "256000" };
		int item_current = -1;
		if      (ProgOptions::Instance()->GNSSReceiverBaudRate == 9600)   item_current = 0;
		else if (ProgOptions::Instance()->GNSSReceiverBaudRate == 14400)  item_current = 1;
		else if (ProgOptions::Instance()->GNSSReceiverBaudRate == 19200)  item_current = 2;
		else if (ProgOptions::Instance()->GNSSReceiverBaudRate == 38400)  item_current = 3;
		else if (ProgOptions::Instance()->GNSSReceiverBaudRate == 57600)  item_current = 4;
		else if (ProgOptions::Instance()->GNSSReceiverBaudRate == 115200) item_current = 5;
		else if (ProgOptions::Instance()->GNSSReceiverBaudRate == 128000) item_current = 6;
		else if (ProgOptions::Instance()->GNSSReceiverBaudRate == 256000) item_current = 7;
		if (item_current >= 0) {
			ImGui::PushItemWidth(sliderWidth);
			ImExt::Style popupStyle(StyleVar::WindowPadding, Math::Vector2(4.0f, 4.0f));
			if (ImGui::Combo("##GNSS-BaudRate-Combo", &item_current, items, IM_ARRAYSIZE(items))) {
				switch (item_current) {
					case 0:  ProgOptions::Instance()->GNSSReceiverBaudRate = 9600;   break;
					case 1:  ProgOptions::Instance()->GNSSReceiverBaudRate = 14400;  break;
					case 2:  ProgOptions::Instance()->GNSSReceiverBaudRate = 19200;  break;
					case 3:  ProgOptions::Instance()->GNSSReceiverBaudRate = 38400;  break;
					case 4:  ProgOptions::Instance()->GNSSReceiverBaudRate = 57600;  break;
					case 5:  ProgOptions::Instance()->GNSSReceiverBaudRate = 115200; break;
					case 6:  ProgOptions::Instance()->GNSSReceiverBaudRate = 128000; break;
					case 7:  ProgOptions::Instance()->GNSSReceiverBaudRate = 256000; break;
					default: ProgOptions::Instance()->GNSSReceiverBaudRate = 9600;   break;
				}
			}
			ImGui::PopItemWidth();
		}
		else {
			ImGui::TextUnformatted((std::to_string(ProgOptions::Instance()->GNSSReceiverBaudRate) + " (Custom rate)"s).c_str());
		}
		ImGui::SameLine(col3Start);
		if (ImGui::Button(" Default ##GNSS-Receiver-BaudRate"))
			ProgOptions::Instance()->GNSSReceiverBaudRate = 9600;
		
		ImGui::TextUnformatted("Reset ");
		ImGui::SameLine();
		ImGui::TextDisabled(u8"\uf059");
		if (ImGui::IsItemHovered()) {
			ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::TextUnformatted("Changing the port or baud rate may not take effect immediately if the GNSS receiver module is already "
			                       "talking to a receiver. Resetting the module will force the new settings into effect immediately. "
			                       "Regardless of whether you reset the module or not, your setting will be saved and used on next program launch.");
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
		ImGui::SameLine();
		ImGui::TextUnformatted(":");
		ImGui::SameLine(col2Start);
		if (ImGui::Button("Reset GNSS receiver module", ImVec2(sliderWidth, 0)))
			GNSSReceiver::GNSSManager::Instance().Reset();

		// Guidance Module Settings   ******************************************************************************************************
		ImGui::Dummy(ImVec2(1, ImGui::GetFontSize()));
		ImGui::TextUnformatted("Guidance Module Setting:");
		ImGui::Dummy(ImVec2(1, 10));
		ImGui::TextUnformatted("Partitioning Method ");
		ImGui::SameLine();
		ImGui::TextDisabled(u8"\uf059");
		if (ImGui::IsItemHovered()) {
			ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::TextUnformatted("This selects the algorithm used to break a survey region into smaller, bite-sized pieces, which can "
			                       "then be dispatched to various drones during a survey mission. Setting takes effect after starting a new mission.\n\n"
			                       "Triangle Fusion: The region is decomposed into a collection of small triangles. Adjacent triangles are then "
			                       "iteratively merged into larger and larger regions until components of appropriate size are obtained. This is "
			                       "very general, but can result in 'pointier', or more irregular-shaped components.\n\n"
			                       "Iterated Cuts: Pieces of the survey region that are too large are repeatedly cut based on heuristics until pieces "
			                       "of appropriate size are obtained.");
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
		ImGui::SameLine();
		ImGui::TextUnformatted(":");
		ImGui::SameLine(col2Start);
		ImGui::PushItemWidth(sliderWidth);
		{
			ImExt::Style popupStyle(StyleVar::WindowPadding, Math::Vector2(4.0f, 4.0f));
			ImGui::Combo("## Partitioning-Method-Combo", &(ProgOptions::Instance()->SurveyRegionPartitioningMethod), "Triangle Fusion\0Iterated Cuts\0");
		}
		ImGui::PopItemWidth();
		ImGui::SameLine(col3Start);
		if (ImGui::Button(" Default ##Partitioning-Method"))
			ProgOptions::Instance()->SurveyRegionPartitioningMethod = 1;

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




