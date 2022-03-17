//This overlay is used to draw messages in a box on top of the map widget
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.

//System Includes
#include <algorithm>
#include <cctype>
#include <string>

//Project Includes
#include "MessageBoxOverlay.hpp"
#include "MapWidget.hpp"
#include "VisWidget.hpp"

//Returns whether or not a message begins with "Error". If true, HeadLength is populated with the number of chars in the "Error" portion
static bool IsErrorMessage(std::string const & Message, size_t & HeadLength) {
	if (Message.size() < 5U)
		return false;
	std::string head = Message.substr(0U, 5U);
	std::transform(head.begin(), head.end(), head.begin(), [](unsigned char c){ return std::tolower(c); });
	if (head == "error"s) {
		HeadLength = Message.find_first_not_of(" \t:-"s, 5U);
		if (HeadLength == std::string::npos)
			HeadLength = Message.size();
		return true;
	}
	else
		return false;
}

//Returns whether or not a message begins with "Warning". If true, HeadLength is populated with the number of chars in the "Warning" portion
static bool IsWarningMessage(std::string const & Message, size_t & HeadLength) {
	if (Message.size() < 7U)
		return false;
	std::string head = Message.substr(0U, 7U);
	std::transform(head.begin(), head.end(), head.begin(), [](unsigned char c){ return std::tolower(c); });
	if (head == "warning"s) {
		HeadLength = Message.find_first_not_of(" \t:-"s, 7U);
		if (HeadLength == std::string::npos)
			HeadLength = Message.size();
		return true;
	}
	else
		return false;
}

void MessageBoxOverlay::Draw_MessageBox(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
	std::scoped_lock lock(m_mutex);
	
	std::vector<int> tokens;
	tokens.reserve(m_messages.size());
	for (auto const & kv : m_messages) {
		if (! kv.second.empty())
			tokens.push_back(kv.first);
	}
	std::sort(tokens.begin(), tokens.end());
	if (! tokens.empty()) {
		//We need the height of the box before we draw anything so we know where to start the text.
		int numLines = 0;
		for (int token : tokens) {
			size_t newLines = std::count(m_messages.at(token).begin(), m_messages.at(token).end(), '\n');
			numLines += 1 + int(newLines);
		}
		float textHeight = numLines*ImGui::GetFontSize();
		
		MapWidget & map(MapWidget::Instance());
		Eigen::Vector2d MapULCorner_ScreenSpace = map.MapWidgetULCorner_ScreenSpace;
		Eigen::Vector2d MapLRCorner_ScreenSpace = MapULCorner_ScreenSpace + map.MapWidgetDims;
		
		float BoxMinX = float(MapULCorner_ScreenSpace(0)) + 6.0f*ImGui::GetStyle().FramePadding.x;
		float BoxMinY = float(MapLRCorner_ScreenSpace(1)) - textHeight - 12.0f*ImGui::GetStyle().FramePadding.y;
		ImVec2 TextMin(BoxMinX + 3.0f*ImGui::GetStyle().FramePadding.x, BoxMinY + 3.0f*ImGui::GetStyle().FramePadding.y);
		
		//Split the draw list - draw the text to the foreground first (keeping track of the max width) and then draw the box behind it.
		//This is kind of ugly, but if we want fancy coloring of words like "error" and "warning" then we have to do it this way or there
		//are edge cases that won't render correctly.
		DrawList->ChannelsSplit(2);
		DrawList->ChannelsSetCurrent(1); //Draw foreground first (text items)
		
		ImVec2 LineStart = TextMin;
		float maxX = TextMin.x;
		for (int token : tokens) {
			std::string line = m_messages.at(token);
			size_t HeadLength;
			float lineHeight = 0.0f;
			if (IsErrorMessage(line, HeadLength)) {
				ImVec2 headSize = ImGui::CalcTextSize(line.c_str(), line.c_str() + HeadLength);
				DrawList->AddText(LineStart, IM_COL32(255, 80, 80, 255), line.c_str(), line.c_str() + HeadLength);
				ImVec2 bodyStart(LineStart.x + headSize.x, LineStart.y);
				ImVec2 bodySize = ImGui::CalcTextSize(line.c_str() + HeadLength);
				DrawList->AddText(bodyStart, IM_COL32(255, 255, 255, 255), line.c_str() + HeadLength);
				maxX = std::max(maxX, bodyStart.x + bodySize.x);
				lineHeight = bodySize.y;
			}
			else if (IsWarningMessage(line, HeadLength)) {
				ImVec2 headSize = ImGui::CalcTextSize(line.c_str(), line.c_str() + HeadLength);
				DrawList->AddText(LineStart, IM_COL32(200, 200, 0, 255), line.c_str(), line.c_str() + HeadLength);
				ImVec2 bodyStart(LineStart.x + headSize.x, LineStart.y);
				ImVec2 bodySize = ImGui::CalcTextSize(line.c_str() + HeadLength);
				DrawList->AddText(bodyStart, IM_COL32(255, 255, 255, 255), line.c_str() + HeadLength);
				maxX = std::max(maxX, bodyStart.x + bodySize.x);
				lineHeight = bodySize.y;
			}
			else {
				ImVec2 textSize = ImGui::CalcTextSize(line.c_str());
				DrawList->AddText(LineStart, IM_COL32(255, 255, 255, 255), line.c_str());
				maxX = std::max(maxX, LineStart.x + textSize.x);
				lineHeight = textSize.y;
			}
			LineStart.y += lineHeight;
		}
		
		DrawList->ChannelsSetCurrent(0); //Draw background (box)
		ImVec2 BoxMin(BoxMinX, BoxMinY);
		ImVec2 BoxMax(maxX + 3.0f*ImGui::GetStyle().FramePadding.x, float(MapLRCorner_ScreenSpace(1)) - 6.0f*ImGui::GetStyle().FramePadding.y);
		DrawList->AddRectFilled(BoxMin, BoxMax, IM_COL32(70, 70, 70, 255), 5.0f, ImDrawCornerFlags_All);
		
		DrawList->ChannelsMerge();
	}
}

//Accessors
void MessageBoxOverlay::Reset() {
	std::scoped_lock lock(m_mutex);
	m_messages.clear();
}

int MessageBoxOverlay::GetAvailableToken(void) {
	std::scoped_lock lock(m_mutex);
	return m_nextAvailableToken++;
}

void MessageBoxOverlay::AddMessage(std::string const & Message, int Token) {
	std::scoped_lock lock(m_mutex);
	m_messages[Token] = Message;
}

bool MessageBoxOverlay::RemoveMessage(int Token) {
	std::scoped_lock lock(m_mutex);
	bool messageExists = (m_messages.count(Token) > 0U);
	m_messages.erase(Token);
	return messageExists;
}

