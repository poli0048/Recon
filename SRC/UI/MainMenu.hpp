//Handles the main menu in the primary UI window
//Author: Bryan Poling
//Copyright (c) 2019 Sentek Systems, LLC. All rights reserved. 
#pragma once

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "ReconUI.hpp"
#include "MapWidget.hpp"
#include "AboutWindow.hpp"
#include "SettingsWindow.hpp"
#include "ConsoleWidget.hpp"
#include "ModalDialogs.hpp"
#include "../Journal.h"
#include "MyGui.hpp"
#include "SimFiducialsWidget.hpp"

class MainMenu {
	public:
		MainMenu() = default;
		~MainMenu() = default;
		static MainMenu & Instance() { static MainMenu menu; return menu; }
		
		void Draw();
};

inline void MainMenu::Draw() {
	float labelMargin = 1.5f*ImGui::GetFontSize();
	
	if (ImGui::BeginMenuBar()) {
		if (ImGui::BeginMenu("File")) {
			if (MyGui::MenuItem(u8"\uf013", labelMargin, "Settings")) {
				SettingsWindow::Instance().Visible = true;
				ImExt::Window::FocusWindow("Settings");
			}
			
			if (MyGui::MenuItem(u8"\uf057", labelMargin, "Exit"))
				ReconUI::Instance().DrawLoopEnabled = false;
			ImGui::EndMenu();
		}
		
		if (ImGui::BeginMenu("View")) {
			ImGui::MenuItem("Draw Data", "", &(MapWidget::Instance().DrawDataTiles));
			ImGui::MenuItem("Draw Data Tooltip", "", &(MapWidget::Instance().DrawDataTooltip));
			
			std::string consoleText = ConsoleWidget::Instance().IsClosedOrClosing() ? "Open Console" : "Close Console";
			if (ImGui::MenuItem(consoleText.c_str()))
				ConsoleWidget::Instance().AppearHideToggle();
			
			if (ImGui::MenuItem("Show Simulation GCP Marker Tool"))
				SimFiducialsWidget::Instance().m_visible = true;
			
			if (ImGui::MenuItem("Show/Hide Demo Window"))
				ReconUI::Instance().show_demo_window = ! ReconUI::Instance().show_demo_window;
			ImGui::EndMenu();
		}
		
		if (ImGui::BeginMenu("Locations")) {
			for (size_t bookmarkIndex = 0U; bookmarkIndex < BookmarkManager::Instance().Bookmarks.size(); bookmarkIndex++) {
				LocationBookmark const & bookmark(BookmarkManager::Instance().Bookmarks[bookmarkIndex]);
				ImGui::PushID((int) bookmarkIndex);
				if (ImGui::Selectable(bookmark.Name.c_str(), false, 0, ImVec2(18.0f*ImGui::GetFontSize(),0)))
					MapWidget::Instance().StartAnimation(bookmark.MinLat, bookmark.MaxLat, bookmark.MinLon, bookmark.MaxLon);
				ImGui::SameLine(0.0f, 0.4f*ImGui::GetFontSize());
				float cursorX = ImGui::GetCursorPosX();
				ImGui::TextUnformatted("\uf057");
				ImGui::SameLine(cursorX);
				if (ImGui::InvisibleButton("DeleteBookmark", ImGui::GetItemRectSize()))
					DeleteBookmarkDialog::Instance().Show(bookmarkIndex);
				ImGui::PopID();
			}
			ImGui::Separator();
			if (ImGui::MenuItem("New Bookmark")) {
				Eigen::Vector2d LatBounds, LonBounds;
				MapWidget::Instance().GetCurrentLatLonBounds(LatBounds, LonBounds);
				NewBookmarkDialog::Instance().Show(LatBounds, LonBounds);
			}
			if (ImGui::MenuItem("Navigate To GPS Coords"))
				ZoomToCoordsDialog::Instance().Show();
			
			ImGui::EndMenu();
		}
		
		if (ImGui::BeginMenu("Help")) {
			if (MyGui::MenuItem(u8"\uf05a", labelMargin, "About")) {
				AboutWindow::Instance().Visible = true;
				ImExt::Window::FocusWindow("About");
			}
			ImGui::EndMenu();
		}
		
		ImGui::EndMenuBar();
	}
}


