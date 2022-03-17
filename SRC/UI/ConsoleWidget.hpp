#pragma once

//System Includes
#include <vector>
#include <string>

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "../Journal.h"
#include "Animation.hpp"

// A simple console window, with scrolling.
class ConsoleWidget {
	static constexpr float ConsoleWidgetHeightInLines = 17.0f;
	static constexpr float AnimationDuration = 0.5f;
	
	std::string m_lineBuffer;
	std::vector<std::string> Items;
	bool ScrollToBottomRequest;
	AnimatedVariable1D WidgetHeight;
	
	void TrimItemsVector(void);

public:
	static ConsoleWidget & Instance() { static ConsoleWidget Widget; return Widget; }
	
	ConsoleWidget();
	~ConsoleWidget();
	
	void ScrollToBottom() { ScrollToBottomRequest = true; }
	void ClearLog() { Items.clear(); ScrollToBottomRequest = true; }

	void Draw();
	
	//Widget height and animation
	void  AppearHideToggle(void)  { WidgetHeight.Transition(); ScrollToBottomRequest = true; }
	bool  IsTransitioning(void)   { return WidgetHeight.IsTransitioning(); }
	bool  IsClosedOrClosing(void) { return (WidgetHeight.IsInStateA() || WidgetHeight.IsTransitioning_B_To_A()); }
	float GetWidgetHeight(void)   {
		WidgetHeight.SetB(ConsoleWidgetHeightInLines*ImGui::GetFontSize());
		return WidgetHeight.GetValue();
	}
};
