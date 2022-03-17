//This module supports UI themes
//Author: Bryan Poling
//Copyright (c) 2019 Sentek Systems, LLC. All rights reserved.

#pragma once

//System Includes
#include <vector>
#include <unordered_map>
#include <memory>

//External Includes
#include "../HandyImGuiInclude.hpp"

//To use, just create a ThemeSitter object for the theme you want to use. The theme will be in effect until the sitter is destroyed.
//You can stack themes by creating a new ThemeSitter object in a local scope - when it is detroyed your previous theme will be restored.
namespace Themes {
	enum class Theme {
		Dummy,  //Don't change anything
		Dark,
		Light
	};
	
	inline Themes::Theme StringToTheme(std::string const & S);
	inline std::string   ThemeToString(Themes::Theme theme);
	
	//Helpers for Displaying and Setting Theme options using ImGui
	inline std::string   GetThemeStrings_OneLine(void);
	inline int           ThemeToSelectionNum(Theme theme);
	inline Themes::Theme SelectionNumToTheme(int Sel);
	
	class ThemeSitter {
		public:
			ThemeSitter(Theme theme = Theme::Dummy) : m_theme(theme) { Load(); }
			~ThemeSitter() = default;
		
		private:
			Theme m_theme;
			ImExt::Style styleSitter;
			
			inline void Load(void);
	};
}

inline Themes::Theme Themes::StringToTheme(std::string const & S) {
	std::string s = Handy::ToLower(S);
	if (s == "dark")
		return Themes::Theme::Dark;
	else if (s == "light")
		return Themes::Theme::Light;
	else
		return Themes::Theme::Dummy;
}

inline std::string Themes::ThemeToString(Theme theme) {
	switch (theme) {
		case Theme::Dummy: return std::string("Dummy");
		case Theme::Dark:  return std::string("Dark");
		case Theme::Light: return std::string("Light");
		default:           return std::string("Dummy");
	}
}

inline std::string Themes::GetThemeStrings_OneLine(void) {
	using namespace std::string_literals;
	return "Dark\0Light\0"s;
}

inline int Themes::ThemeToSelectionNum(Themes::Theme theme) {
	switch (theme) {
		case Theme::Dark:  return 0;
		case Theme::Light: return 1;
		default:           return 0;
	}
}

inline Themes::Theme Themes::SelectionNumToTheme(int Sel) {
	switch (Sel) {
		case 0:  return Themes::Theme::Dark;
		case 1:  return Themes::Theme::Light;
		default: return Themes::Theme::Dummy;
	}
}

inline void Themes::ThemeSitter::Load(void) {
	//Common settings accross all themes
	styleSitter(StyleVar::GrabRounding,  4.0f);
	styleSitter(StyleVar::FrameRounding, 4.0f);
	styleSitter(StyleVar::FramePadding,  Math::Vector2(4.0f, 4.0f));
	styleSitter(StyleVar::ItemSpacing,   Math::Vector2(3.0f, 3.0f));
	styleSitter(StyleVar::ChildRounding, 4.0f);
	styleSitter(StyleVar::IndentSpacing, 30.0f);
	
	//Theme-specific settings
	if (m_theme == Theme::Dark) {
		styleSitter(StyleCol::WindowBg,             Math::Vector4(119.0f, 119.0f, 119.0f, 255.0f)/255.0f);
		styleSitter(StyleCol::FrameBg,              Math::Vector4(100.0f, 100.0f, 100.0f, 255.0f)/255.0f);
		styleSitter(StyleCol::FrameBgHovered,       Math::Vector4(119.0f, 119.0f, 119.0f, 255.0f)/255.0f);
		styleSitter(StyleCol::FrameBgActive,        Math::Vector4(119.0f, 119.0f, 119.0f, 255.0f)/255.0f);
		styleSitter(StyleCol::ChildBg,              Math::Vector4( 43.0f,  43.0f,  43.0f, 255.0f)/255.0f);
		styleSitter(StyleCol::MenuBarBg,            ImGui::GetStyle().Colors[ImGuiCol_WindowBg]);
		styleSitter(StyleCol::Button,               Math::Vector4(  0.0f, 122.0f, 145.0f, 255.0f)/255.0f);
		styleSitter(StyleCol::ButtonHovered,        Math::Vector4(  0.0f, 142.0f, 165.0f, 255.0f)/255.0f);
		styleSitter(StyleCol::ButtonActive,         Math::Vector4(  0.0f, 162.0f, 185.0f, 255.0f)/255.0f);
		styleSitter(StyleCol::ScrollbarGrab,        ImGui::GetStyle().Colors[ImGuiCol_Button]);
		styleSitter(StyleCol::ScrollbarGrabHovered, ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered]);
		styleSitter(StyleCol::ScrollbarGrabActive,  ImGui::GetStyle().Colors[ImGuiCol_ButtonActive]);
		styleSitter(StyleCol::ScrollbarBg,          Math::Vector4( 67.0f,  67.0f,  67.0f, 255.0f)/255.0f);
		styleSitter(StyleCol::Header,               Math::Vector4(0.35f, 0.35f, 0.35f, 1.00f));
		styleSitter(StyleCol::HeaderHovered,        Math::Vector4(0.31f, 0.31f, 0.31f, 1.00f));
		styleSitter(StyleCol::HeaderActive,         Math::Vector4(0.39f, 0.39f, 0.39f, 1.00f));
		styleSitter(StyleCol::SliderGrab,           ImGui::GetStyle().Colors[ImGuiCol_Button]);
		styleSitter(StyleCol::SliderGrabActive,     ImGui::GetStyle().Colors[ImGuiCol_ButtonActive]);
		styleSitter(StyleCol::Text,                 Math::Vector4(   1.0f,   1.0f,   1.0f,   1.0f));
		styleSitter(StyleCol::TextDisabled,         Math::Vector4(   1.0f,   1.0f,   1.0f,   0.5f));
		styleSitter(StyleCol::TextSelectedBg,       ImGui::GetStyle().Colors[ImGuiCol_Button]);
		styleSitter(StyleCol::Separator,            Math::Vector4(   1.0f,   1.0f,   1.0f,   0.082f));
		styleSitter(StyleCol::PopupBg,              Math::Vector4(  66.0f,  66.0f,  66.0f, 255.0f)/255.0f);
	}
	else if (m_theme == Theme::Light) {
		styleSitter(StyleCol::WindowBg,             Math::Vector4(0.96f, 0.96f, 0.96f, 1.00f));
		
		styleSitter(StyleCol::FrameBg,              Math::Vector4(0.85f, 0.85f, 0.85f, 0.40f));
		styleSitter(StyleCol::FrameBgHovered,       Math::Vector4(0.90f, 0.90f, 0.90f, 0.50f));
		styleSitter(StyleCol::FrameBgActive,        Math::Vector4(0.90f, 0.90f, 0.90f, 0.50f));
		
		styleSitter(StyleCol::ChildBg,              Math::Vector4(0.96f, 0.96f, 0.96f, 1.00f));
		styleSitter(StyleCol::MenuBarBg,            Math::Vector4(0.84f, 0.84f, 0.84f, 1.00f));
		styleSitter(StyleCol::ScrollbarGrab,        Math::Vector4(0.69f, 0.69f, 0.69f, 0.80f));
		styleSitter(StyleCol::ScrollbarGrabActive,  Math::Vector4(0.42f, 0.42f, 0.42f, 1.00f));
		styleSitter(StyleCol::ScrollbarBg,          Math::Vector4(0.69f, 0.69f, 0.69f, 0.53f));
		styleSitter(StyleCol::Header,               Math::Vector4(0.86f, 0.86f, 0.86f, 1.00f));
		styleSitter(StyleCol::HeaderHovered,        Math::Vector4(0.81f, 0.81f, 0.81f, 1.00f));
		styleSitter(StyleCol::HeaderActive,         Math::Vector4(0.69f, 0.69f, 0.69f, 1.00f));
		styleSitter(StyleCol::Text,                 Math::Vector4(0.00f, 0.00f, 0.00f, 1.00f));
		styleSitter(StyleCol::TextDisabled,         Math::Vector4(0.62f, 0.62f, 0.62f, 1.00f));
		styleSitter(StyleCol::TextSelectedBg,       Math::Vector4(0.26f, 0.59f, 0.98f, 0.35f));
		styleSitter(StyleCol::PopupBg,              Math::Vector4(0.95f, 0.95f, 0.95f, 1.00f));
		styleSitter(StyleCol::Border,               Math::Vector4(0.00f, 0.00f, 0.00f, 0.30f));
		styleSitter(StyleCol::BorderShadow,         Math::Vector4(0.00f, 0.00f, 0.00f, 0.00f));
		styleSitter(StyleCol::TitleBg,              Math::Vector4(0.96f, 0.96f, 0.96f, 1.00f));
		styleSitter(StyleCol::TitleBgActive,        Math::Vector4(0.79f, 0.79f, 0.79f, 1.00f));
		styleSitter(StyleCol::TitleBgCollapsed,     Math::Vector4(0.79f, 0.79f, 0.79f, 1.00f));
		
		styleSitter(StyleCol::Tab,                  Math::Vector4(0.83f, 0.83f, 0.83f, 1.00f));
		styleSitter(StyleCol::TabHovered,           Math::Vector4(0.58f, 0.76f, 0.97f, 1.00f));
		styleSitter(StyleCol::TabActive,            Math::Vector4(0.68f, 0.81f, 0.97f, 1.00f));
		
		styleSitter(StyleCol::Button,               Math::Vector4(0.68f, 0.81f, 0.97f, 1.00f));
		styleSitter(StyleCol::ButtonHovered,        Math::Vector4(0.58f, 0.76f, 0.97f, 1.00f));
		styleSitter(StyleCol::ButtonActive,         Math::Vector4(0.48f, 0.71f, 0.97f, 1.00f));
		
		styleSitter(StyleCol::CheckMark,            Math::Vector4(0.26f, 0.59f, 0.98f, 1.00f));
		styleSitter(StyleCol::SliderGrab,           Math::Vector4(0.68f, 0.81f, 0.97f, 1.00f));
		styleSitter(StyleCol::SliderGrabActive,     Math::Vector4(0.48f, 0.71f, 0.97f, 1.00f));
	}
}


	

