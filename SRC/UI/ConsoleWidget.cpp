//System Includes
#include <iostream>
#include <cmath>

//Project Includes
#include "ConsoleWidget.hpp"

ConsoleWidget::ConsoleWidget() {
	Handy::Console::SetOutputCallback([this](std::string const & text) {
		if (text.size() == 0)
			return;
	
		//Remove all carriage returns (convert to nix-style "\n" newlines) and add to line buffer
		std::string textLocalCopy = text;
		textLocalCopy.erase(std::remove(textLocalCopy.begin(), textLocalCopy.end(), '\r'), textLocalCopy.end());
		m_lineBuffer += textLocalCopy;
		
		//Add all complete lines in the line buffer to Items vector
		size_t posOfNewline = 0U;
		do {
			posOfNewline = m_lineBuffer.find_first_of('\n');
			if (posOfNewline != std::string::npos) {
				Items.push_back(m_lineBuffer.substr(0, posOfNewline));
				m_lineBuffer = m_lineBuffer.substr(posOfNewline + 1U);
				ScrollToBottomRequest = true;
			}
		}
		while (posOfNewline != std::string::npos);
		
		//Trim old items if we have exceeded the maximum allowable history length
		TrimItemsVector();
	});
	
	ClearLog();
	
	//Setup animation for widget show/hide (B state will be updated in GetWidgetHeight(), so just use 1 here)
	WidgetHeight = AnimatedVariable1D(0.0, 1.0, AnimationDuration);
}

ConsoleWidget::~ConsoleWidget() {
	//Un-register our callback for stream redirection
	Handy::Console::SetOutputCallback([](std::string const & text) { });
}

void ConsoleWidget::Draw() {
	if (ImGui::SmallButton("Clear")) 
		ClearLog(); 
	ImGui::SameLine();
	
	if (ImGui::SmallButton("Scroll To Bottom"))
		ScrollToBottomRequest = true;
	ImGui::SameLine();
	
	float CloseConsoleButtonWidth = ImGui::CalcTextSize(u8"\uf103  Hide Console  \uf103").x + 2.0f*ImGui::GetStyle().FramePadding.x;
	ImGui::Dummy(ImVec2(ImGui::GetContentRegionAvail().x - CloseConsoleButtonWidth - 10.0f, 1.0f));
	ImGui::SameLine();
	if (ImGui::SmallButton(u8"\uf103  Hide Console  \uf103") && (! IsClosedOrClosing()))
		AppearHideToggle();
	ImGui::Separator();
	
	ImGuiWindowFlags flags = IsTransitioning() ? ImGuiWindowFlags_NoScrollbar : ImGuiWindowFlags_HorizontalScrollbar;
	ImGui::BeginChild("ScrollingRegion", ImVec2(0,0/*-ImGui::GetItemsLineHeightWithSpacing()*/), false, flags);
	if (ImGui::BeginPopupContextWindow()) {
		if (ImGui::Selectable("Clear"))
			ClearLog();
		ImGui::EndPopup();
	}
	
	{
		ImExt::Font fnt(Fonts::Normal); //Set console font
		ImExt::Style consoleStyle(StyleVar::ItemSpacing, ImVec2(4,1)); //Tighten line spacing

		ImGuiListClipper clipper((int)Items.size());
		while (clipper.Step())
			for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; i++) {
				std::string const & item = Items[i];
				
				//Use red text for lines containing the word "Error"
				ImExt::Style lineStyle;
				std::string lowerCaseLine = Handy::ToLower(item);
				if (Handy::StringContains(lowerCaseLine, "error"))
					lineStyle(StyleCol::Text, Math::Vector4(1.0f,0.4f,0.4f,1.0f));
				
				if (item.size() <= 180U)
					ImGui::TextUnformatted(item.c_str());
				else
					ImGui::TextUnformatted((item.substr(0U, 177U) + "...").c_str());
			}

		if (ScrollToBottomRequest) {
			ImGui::SetScrollHereY();
			ScrollToBottomRequest = false;
		}
		
		clipper.End();
	}

	ImGui::EndChild();
}

void ConsoleWidget::TrimItemsVector(void) {
	size_t maxItemsSize = 5000U;
	if (Items.size() > maxItemsSize) {
		//Erase the oldest quarter of the buffer. We erase large chunks at a time since this will generally trigger reallocation.
		Items.erase(Items.begin(), Items.begin() + maxItemsSize/4U);
	}
}




