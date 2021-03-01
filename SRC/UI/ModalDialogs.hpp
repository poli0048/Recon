//Singleton modal Dialog objects for the main window - these should be drawn from the top-level main window
//Author: Bryan Poling
//Copyright (c) 2019 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <string>
#include <system_error>

//External Includes
#include "../../../handycpp/Handy.hpp"
#include "../../../imgui/app/ImGuiApp.hpp"
#include "../../nativefiledialog/src/include/nfd.h"

//Project Includes
#include "BookmarkManager.hpp"
#include "MapWidget.hpp"
#include "../SurveyRegionManager.hpp"
#include "../Utilities.hpp"
#include "../OpenInFileManager.hpp"
#include "MyGui.hpp"

class DummyButtonStyle {
	public:
		ImExt::Style m_style;
		
		DummyButtonStyle() {
			Math::Vector4 dummyButtonColor = ImGui::GetStyle().Colors[ImGuiCol_Button];
			dummyButtonColor.x = 0.3f*dummyButtonColor.x + 0.2f;
			dummyButtonColor.y = 0.3f*dummyButtonColor.y + 0.2f;
			dummyButtonColor.z = 0.3f*dummyButtonColor.z + 0.2f;
			m_style(StyleCol::Button,        dummyButtonColor);
			m_style(StyleCol::ButtonHovered, dummyButtonColor);
			m_style(StyleCol::ButtonActive,  dummyButtonColor);
		}
};

class RedButtonStyle {
	public:
		ImExt::Style m_style;
		
		RedButtonStyle() {
			m_style(StyleCol::Button,        Math::Vector4(0.9f, 0.2f, 0.2f, 1.0f));
			m_style(StyleCol::ButtonHovered, Math::Vector4(1.0f, 0.3f, 0.3f, 1.0f));
			m_style(StyleCol::ButtonActive,  Math::Vector4(1.0f, 0.4f, 0.4f, 1.0f));
		}
};

class InformationDialog {
	private:
		bool showRequest = false;
		std::string m_text;
	
	public:
		static InformationDialog & Instance() { static InformationDialog Dialog; return Dialog; }
		void Show(std::string const & Text) {
			m_text = Text;
			showRequest = true;
		}
		void Draw() {
			if (showRequest && (! ImGui::IsPopupOpen("Information"))) {
				ImGui::OpenPopup("Information");
				showRequest = false;
			}
			if (ImGui::BeginPopupModal("Information", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
				ImGui::TextUnformatted(m_text.c_str());
				ImGui::Dummy(ImVec2(200,1)); //Make sure the dialog isn't ridiculously small
				
				if (ImGui::Button("Close"))
					ImGui::CloseCurrentPopup(); //Close the popup dialog
				
				ImGui::EndPopup();
			}
		}
};

class NewBookmarkDialog {
	private:
		bool m_showRequest = false;
		Eigen::Vector2d m_LatBounds;
		Eigen::Vector2d m_LonBounds;
		
		static size_t constexpr m_modalTextBufSize = 1000U;
		char m_modalTextBuf[m_modalTextBufSize];
	
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		static NewBookmarkDialog & Instance() { static NewBookmarkDialog Dialog; return Dialog; }
		void Show(Eigen::Vector2d const & LatBounds, Eigen::Vector2d const & LonBounds) {
			m_modalTextBuf[0] = 0;
			m_LatBounds = LatBounds;
			m_LonBounds = LonBounds;
			m_showRequest = true;
		}
		void Draw() {
			if (m_showRequest && (! ImGui::IsPopupOpen("New Bookmark"))) {
				ImGui::OpenPopup("New Bookmark");
				m_showRequest = false;
			}
			if (ImGui::BeginPopupModal("New Bookmark", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
				ImGui::TextUnformatted("Name:  ");
				ImGui::SameLine();
				if (ImGui::IsWindowAppearing())
					ImGui::SetKeyboardFocusHere();
				ImGui::PushItemWidth(std::min(25.0f*ImGui::GetFontSize(), 800.0f));
				ImGui::InputText("##BookmarkName", m_modalTextBuf, m_modalTextBufSize);
				ImGui::PopItemWidth();
				if (ImGui::Button("Save") || ImGui::IsKeyReleased(ImGui::GetKeyIndex(ImGuiKey_Enter))) {
					BookmarkManager::Instance().Bookmarks.emplace_back(std::string(m_modalTextBuf), m_LatBounds, m_LonBounds);
					ImGui::CloseCurrentPopup(); //Close the popup dialog
				}
				ImGui::SameLine();
				if (ImGui::Button("Cancel"))
					ImGui::CloseCurrentPopup(); //Close the popup dialog
				
				ImGui::EndPopup();
			}
		}
};

class DeleteBookmarkDialog {
	private:
		bool m_showRequest = false;
		size_t m_bookmarkIndex = 0U;
	
	public:
		static DeleteBookmarkDialog & Instance() { static DeleteBookmarkDialog Dialog; return Dialog; }
		void Show(size_t BookmarkIndex) {
			m_bookmarkIndex = BookmarkIndex;
			m_showRequest = true;
		}
		void Draw() {
			if (m_showRequest && (! ImGui::IsPopupOpen("Delete Bookmark"))) {
				ImGui::OpenPopup("Delete Bookmark");
				m_showRequest = false;
			}
			if (ImGui::BeginPopupModal("Delete Bookmark", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
				if (m_bookmarkIndex < BookmarkManager::Instance().Bookmarks.size()) {
					std::vector<LocationBookmark> & Bookmarks(BookmarkManager::Instance().Bookmarks);
					ImGui::Text("Delete bookmark '%s'?", Bookmarks[m_bookmarkIndex].Name.c_str());
					ImGui::Dummy(ImVec2(15.0f*ImGui::GetFontSize(), 0.5f*ImGui::GetFontSize()));
					if (ImGui::Button("Yes - Delete it") || ImGui::IsKeyReleased(ImGui::GetKeyIndex(ImGuiKey_Enter))) {
						Bookmarks.erase(Bookmarks.begin() + m_bookmarkIndex);
						ImGui::CloseCurrentPopup(); //Close the popup dialog
					}
					ImGui::SameLine();
					if (ImGui::Button("Cancel"))
						ImGui::CloseCurrentPopup(); //Close the popup dialog
				}
				else
					ImGui::CloseCurrentPopup();
				ImGui::EndPopup();
			}
		}
};

class ZoomToCoordsDialog {
	private:
		bool m_showRequest = false;
		static size_t constexpr m_modalTextBufSize = 1000U;
		char m_modalTextBuf_Lat[m_modalTextBufSize];
		char m_modalTextBuf_Lon[m_modalTextBufSize];
	
	public:
		static ZoomToCoordsDialog & Instance() { static ZoomToCoordsDialog Dialog; return Dialog; }
		void Show(void) {
			m_modalTextBuf_Lat[0] = 0;
			m_modalTextBuf_Lon[0] = 0;
			m_showRequest = true;
		}
		void Draw() {
			if (m_showRequest && (! ImGui::IsPopupOpen("Navigate To"))) {
				ImGui::OpenPopup("Navigate To");
				m_showRequest = false;
			}
			if (ImGui::BeginPopupModal("Navigate To", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
				float col2StartX = ImGui::CalcTextSize("Longitude:    ").x;
				
				ImGui::TextUnformatted("Latitude: ");
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("WGS84 Latitude, in decimal degrees.");
				ImGui::SameLine(col2StartX);
				if (ImGui::IsWindowAppearing())
					ImGui::SetKeyboardFocusHere();
				ImGui::PushItemWidth(std::min(20.0f*ImGui::GetFontSize(), 800.0f));
				ImGui::InputText("##LatitudeInputText", m_modalTextBuf_Lat, m_modalTextBufSize);
				ImGui::PopItemWidth();
				
				ImGui::TextUnformatted("Longitude: ");
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("WGS84 Longitude, in decimal degrees.");
				ImGui::SameLine(col2StartX);
				ImGui::PushItemWidth(std::min(20.0f*ImGui::GetFontSize(), 800.0f));
				ImGui::InputText("##LongitudeInputText", m_modalTextBuf_Lon, m_modalTextBufSize);
				ImGui::PopItemWidth();
				
				ImGui::Dummy(ImVec2(ImGui::GetFontSize(), 0.5f*ImGui::GetFontSize()));
				if (ImGui::Button("Go") || ImGui::IsKeyReleased(ImGui::GetKeyIndex(ImGuiKey_Enter))) {
					//Attempt conversion - if successful, start navigation animation
					try {
						double const PI = 3.14159265358979;
						double const eps = 0.00025;
						double lat = PI/180.0*std::stod(m_modalTextBuf_Lat);
						double lon = PI/180.0*std::stod(m_modalTextBuf_Lon);
						if (std::isfinite(lat) && std::isfinite(lon))
							MapWidget::Instance().StartAnimation(lat - eps, lat + eps, lon - eps, lon + eps);
					}
					catch (...) { }
					
					ImGui::CloseCurrentPopup(); //Close the popup dialog
				}
				ImGui::SameLine();
				if (ImGui::Button("Cancel"))
					ImGui::CloseCurrentPopup(); //Close the popup dialog
				
				ImGui::EndPopup();
			}
		}
};

class SelectActiveRegionDialog {
	private:
		bool m_showRequest = false;
		int m_selection = -1;
		std::vector<std::string> m_regionList;
		
		//Get an acceptable name for a new region
		std::string GetNewRegionName(void) {
			if (! Handy::Contains(m_regionList, "New Region"s))
				return "New Region"s;
			int num = 1;
			while (Handy::Contains(m_regionList, "New Region ("s + std::to_string(num) + ")"s))
				num++;
			return "New Region ("s + std::to_string(num) + ")"s;
		}
		
		//Call this for every item to render the context menu for that item (usually won't draw anything)
		//Returns true if the draw loop should abort
		void DrawContextMenu(bool Open, int itemIndex) {
			static size_t constexpr m_modalTextBufSize = 1000U;
			static char m_modalTextBuf_Name[m_modalTextBufSize];
			
			float labelMargin = 1.5f*ImGui::GetFontSize();
			std::string popupStrIdentifier("NoItem-ContextMenu");
			if (itemIndex >= 0)
				popupStrIdentifier = m_regionList[itemIndex] + "-ContextMenu"s;
			if (Open)
				ImGui::OpenPopup(popupStrIdentifier.c_str());
			
			if (ImGui::BeginPopup(popupStrIdentifier.c_str())) {
				bool confirmDeletePopupRequest = false;
				bool renameItemPopupRequest = false;
				if (MyGui::MenuItem(u8"\uf24d", labelMargin, "Duplicate", NULL, false, true)) {
					std::filesystem::path surveyRegionsFolderPath = Handy::Paths::ThisExecutableDirectory() / "Survey Regions";
					std::filesystem::path originalPath  = surveyRegionsFolderPath / (m_regionList[itemIndex] + ".region");
					std::filesystem::path duplicatePath = surveyRegionsFolderPath / (m_regionList[itemIndex] + " (copy).region");
					std::error_code ec;
					std::filesystem::copy_file(originalPath, duplicatePath, std::filesystem::copy_options::overwrite_existing, ec);
					if (! ec)
						m_regionList.insert(m_regionList.begin() + itemIndex + 1, duplicatePath.stem().string());
				}
				if (MyGui::MenuItem(u8"\uf02b", labelMargin, "Rename", NULL, false, true))
					renameItemPopupRequest = true;
				ImGui::Separator();
				if (MyGui::MenuItem(u8"\uf1f8", labelMargin, "Delete", NULL, false, true))
					confirmDeletePopupRequest = true;
				ImGui::EndPopup();
				
				if (confirmDeletePopupRequest)
					ImGui::OpenPopup(("Delete?##"s + popupStrIdentifier).c_str());
				if (renameItemPopupRequest) {
					size_t length = m_regionList[itemIndex].copy(m_modalTextBuf_Name, m_modalTextBufSize-1U);
					m_modalTextBuf_Name[length] = '\0';
					ImGui::OpenPopup(("Rename##"s + popupStrIdentifier).c_str());
				}
			}
			
			if (ImGui::BeginPopupModal(("Delete?##"s + popupStrIdentifier).c_str(), NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
				ImGui::Text("Are you sure?\nThis operation cannot be undone!\n\n");
				ImGui::Separator();
				if (ImGui::Button("Cancel"))
					ImGui::CloseCurrentPopup();
				float DeleteButtonWidth = ImGui::CalcTextSize(" Delete ").x + 2.0f*ImGui::GetStyle().FramePadding.x;
				ImGui::SameLine(ImGui::GetContentRegionAvail().x - DeleteButtonWidth);
				{
					RedButtonStyle styleSitter;
					if (ImGui::Button("Delete", ImVec2(DeleteButtonWidth, 0))) {
						std::filesystem::path surveyRegionsFolderPath = Handy::Paths::ThisExecutableDirectory() / "Survey Regions";
						std::filesystem::path filePath  = surveyRegionsFolderPath / (m_regionList[itemIndex] + ".region");
						std::error_code ec;
						std::filesystem::remove(filePath, ec);
						if (! ec) {
							m_regionList.erase(m_regionList.begin() + itemIndex);
							if (m_selection >= itemIndex)
								m_selection--;
						}
						ImGui::CloseCurrentPopup();
					}
				}
				ImGui::EndPopup();
			}
			
			if (ImGui::BeginPopupModal(("Rename##"s + popupStrIdentifier).c_str(), NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
				if (ImGui::IsWindowAppearing())
					ImGui::SetKeyboardFocusHere();
				ImGui::PushItemWidth(std::min(20.0f*ImGui::GetFontSize(), 800.0f));
				ImGui::InputText("##NameInput", m_modalTextBuf_Name, m_modalTextBufSize);
				ImGui::PopItemWidth();
				
				ImGui::Separator();
				if (ImGui::Button("Cancel", ImVec2(120, 0)))
					ImGui::CloseCurrentPopup();
				float RenameButtonWidth = ImGui::CalcTextSize(" Rename ").x + 2.0f*ImGui::GetStyle().FramePadding.x;
				ImGui::SameLine(ImGui::GetContentRegionAvail().x - RenameButtonWidth);
				
				bool NameIsOK = (isFilenameReasonable(m_modalTextBuf_Name) &&
				                ((! Handy::Contains(m_regionList, std::string(m_modalTextBuf_Name))) || (m_regionList[itemIndex] == m_modalTextBuf_Name)));
				if (! NameIsOK) {
					DummyButtonStyle styleSitter;
					ImGui::Button("Rename", ImVec2(RenameButtonWidth,0));
				}
				else {
					if ((ImGui::Button("Rename", ImVec2(RenameButtonWidth, 0))) || ImGui::IsKeyReleased(ImGui::GetKeyIndex(ImGuiKey_Enter))) {
						if (m_regionList[itemIndex] == m_modalTextBuf_Name)
							std::cerr << "Rename: No change needed.\r\n";
						else {
							std::filesystem::path surveyRegionsFolderPath = Handy::Paths::ThisExecutableDirectory() / "Survey Regions";
							std::filesystem::path originalPath = surveyRegionsFolderPath / (m_regionList[itemIndex] + ".region");
							std::filesystem::path newPath      = surveyRegionsFolderPath / (std::string(m_modalTextBuf_Name) + ".region");
							std::error_code ec;
							rename(originalPath, newPath, ec);
							if (! ec)
								m_regionList[itemIndex] = newPath.stem().string();
						}
						ImGui::CloseCurrentPopup();
					}
				}
				ImGui::EndPopup();
			}
		}
	
	public:
		static SelectActiveRegionDialog & Instance() { static SelectActiveRegionDialog Dialog; return Dialog; }
		void Show(void) {
			//Populate the files in the survey regions directory
			std::filesystem::path surveyRegionsFolderPath = Handy::Paths::ThisExecutableDirectory() / "Survey Regions";
			std::vector<std::filesystem::path> files = GetNormalFilesInDirectory(surveyRegionsFolderPath);
			m_regionList.clear();
			m_regionList.reserve(files.size());
			for (auto const & file : files) {
				std::string filename = file.stem().string(); //Without file extension
				if ((! filename.empty()) && (filename[0] != '.'))
					m_regionList.push_back(filename);
				
			}
			std::sort(m_regionList.begin(), m_regionList.end(),
			          [](std::string const & A, std::string const & B) -> bool { return StringNumberAwareCompare_LessThan(A, B); });
			
			//Clear the selection
			m_selection = -1;
			
			m_showRequest = true;
		}
		void Draw() {
			if (m_showRequest) {
				m_showRequest = false;
				if (! ImGui::IsPopupOpen("Select Active Survey Region"))
					ImGui::OpenPopup("Select Active Survey Region");
			}
			
			if (ImGui::BeginPopupModal("Select Active Survey Region", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize)) {
				if (ImGui::ListBoxHeader("##Survey Regions", ImVec2(45.0f*ImGui::GetFontSize(), 0.0f))) {
					bool clickProcessed = false;
					for (int itemIndex = 0; itemIndex < int(m_regionList.size()); itemIndex++) {
						if (ImGui::Selectable(m_regionList[itemIndex].c_str(), m_selection == itemIndex) || ImGui::IsItemClicked(1)) {
							m_selection = itemIndex;
							clickProcessed = true;
						}
						DrawContextMenu(ImGui::IsItemClicked(1), itemIndex);
					}
					ImGui::ListBoxFooter();
					if (ImGui::IsItemClicked(0) && (! clickProcessed))
						m_selection = -1;
				}
				
				if (ImGui::Button(" New Region ")) {
					m_selection = int(m_regionList.size());
					std::string newRegionName = GetNewRegionName();
					m_regionList.push_back(newRegionName);
					SurveyRegion newRegion(newRegionName); //Touch the new resource on disk
				}
				ImGui::SameLine();
				if (ImGui::Button(" Show Folder "))
					OpenInFileManager(Handy::Paths::ThisExecutableDirectory() / "Survey Regions");
				float CancelButtonWidth = ImGui::CalcTextSize(" Cancel ").x + 2.0f*ImGui::GetStyle().FramePadding.x;
				float NoneButtonWidth = ImGui::CalcTextSize(" Set No Active Region ").x + 2.0f*ImGui::GetStyle().FramePadding.x;
				float ActionButtonWidth = ImGui::CalcTextSize(" Make Selected Region Active ").x + 2.0f*ImGui::GetStyle().FramePadding.x;
				ImGui::SameLine(ImGui::GetContentRegionAvail().x - ActionButtonWidth - NoneButtonWidth - CancelButtonWidth - 2.0f*ImGui::GetStyle().ItemSpacing.x);
				if (ImGui::Button("Cancel", ImVec2(CancelButtonWidth,0)))
					ImGui::CloseCurrentPopup(); //Close the popup dialog
				ImGui::SameLine();
				if (ImGui::Button("Set No Active Region", ImVec2(NoneButtonWidth,0))) {
					SurveyRegionManager::Instance().SetActiveSurveyRegion(std::string());
					ImGui::CloseCurrentPopup(); //Close the popup dialog
				}
				ImGui::SameLine();
				if (m_selection < 0) {
					DummyButtonStyle styleSitter;
					ImGui::Button("Make Selected Region Active", ImVec2(ActionButtonWidth,0));
				}
				else {
					if (ImGui::Button("Make Selected Region Active", ImVec2(ActionButtonWidth,0))) {
						if ((m_selection >= 0) && (m_selection < int(m_regionList.size())))
							SurveyRegionManager::Instance().SetActiveSurveyRegion(m_regionList[m_selection]);
						ImGui::CloseCurrentPopup(); //Close the popup dialog
					}
				}
				
				ImGui::EndPopup();
			}
		}
};

inline void DrawModalDialogs(void) {
	InformationDialog::Instance().Draw();
	NewBookmarkDialog::Instance().Draw();
	DeleteBookmarkDialog::Instance().Draw();
	ZoomToCoordsDialog::Instance().Draw();
	SelectActiveRegionDialog::Instance().Draw();
}




