#pragma once

//System Includes
#include <tuple>
#include <vector>

//Eigen Includes
#include "../../../eigen/Eigen/Core"

//External Includes
#include "../../../imgui/imgui.h"

//Project Includes
#include "../Colormaps.hpp"
#include "../ImVecOps.hpp"
#include "../Journal.h"

namespace MyGui {
	//A non-selectable label with a header-colored background
	inline void HeaderLabel(const char * UnformattedText) {
		//Draw Queue Label with a header-colored background
		auto style = ImGui::GetStyle();
		ImDrawList * draw_list = ImGui::GetWindowDrawList();
		Math::Vector2 CursorStartPos = ImGui::GetCursorScreenPos();
		float availWidth = ImGui::GetContentRegionAvail().x;
		float xWinPadding = style.WindowPadding.x;
		Math::Vector2 RectULCorner = CursorStartPos - Math::Vector2(xWinPadding, 0.0f);
		Math::Vector2 RectLRCorner = CursorStartPos + Math::Vector2(availWidth + xWinPadding, ImGui::GetFontSize() + 2.0f*style.ItemInnerSpacing.y);
		draw_list->AddRectFilled(RectULCorner, RectLRCorner, ImColor(style.Colors[ImGuiCol_Header]), 0.0f);
		ImGui::SetCursorScreenPos(CursorStartPos + style.ItemInnerSpacing);
		ImGui::TextUnformatted(UnformattedText);
		ImGui::SetCursorScreenPos(Math::Vector2(CursorStartPos.x, RectLRCorner.y + style.ItemSpacing.y));
	}
	
	//Version of BeginMenu() that has a column for icons
	inline bool BeginMenu(const char* txticon, float XMargin, const char* label, bool enabled = true) {
		Math::Vector2 curPos = ImGui::GetCursorScreenPos();
		std::string spaces((size_t)std::ceil(XMargin / ImGui::CalcTextSize(" ").x), ' ');
		spaces += std::string(label);
		ImDrawList * winDrawList = ImGui::GetWindowDrawList();
		bool ret = ImGui::BeginMenu(spaces.c_str(), enabled);
		winDrawList->AddText(curPos, ImGui::ColorConvertFloat4ToU32(ImGui::GetStyleColorVec4(ImGuiCol_Text)), txticon);
		return ret;
	}
	
	//Version of MenuItem() that has a column for icons
	inline bool MenuItem(const char* txticon, float XMargin, const char* label, const char* shortcut = NULL, bool selected = false, bool enabled = true) {
		Math::Vector2 curPos = ImGui::GetCursorScreenPos();
		std::string spaces((size_t)std::ceil(XMargin / ImGui::CalcTextSize(" ").x), ' ');
		spaces += std::string(label);
		bool ret = ImGui::MenuItem(spaces.c_str(), shortcut, selected, enabled);
		ImGui::GetWindowDrawList()->AddText(curPos, ImGui::ColorConvertFloat4ToU32(ImGui::GetStyleColorVec4(ImGuiCol_Text)), txticon);
		return ret;
	}
}


