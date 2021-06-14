//Main Window and provider of outer draw loop - all other window draw loops are called
//from within our Draw member function.

//System Includes
#include <memory>

//Project Includes
#include "ReconUI.hpp"
#include "AboutWindow.hpp"
#include "SettingsWindow.hpp"
#include "MainMenu.hpp"
#include "MapWidget.hpp"
#include "VisWidget.hpp"
#include "VehicleControlWidget.hpp"
#include "CommandWidget.hpp"
#include "ConsoleWidget.hpp"
#include "StatusBar.hpp"
#include "../ProgOptions.hpp"
#include "ModalDialogs.hpp"
#include "TextureUploadFlowRestrictor.hpp"
#include "SimFiducialsWidget.hpp"

ReconUI::ReconUI() { }
void ReconUI::Preframe() { }
void ReconUI::Postframe() {
	TextureUploadFlowRestrictor::Instance().Reset();
}

void ReconUI::Draw() {
	static bool firstDrawPass = true;
	
	//Lock progOptions - this covers all access from the main thread in drawing the UI
	std::scoped_lock progOptionsLock(ProgOptions::Instance()->OptionsMutex);
	
	//Either open the Demo/Style window, or set the global theme
	std::unique_ptr<Themes::ThemeSitter> themeSitter;
	if (show_demo_window)
		ImGui::ShowDemoWindow(&show_demo_window);
	else
		themeSitter.reset(new Themes::ThemeSitter(ProgOptions::Instance()->UITheme));
	ImExt::Font fnt(Fonts::Normal); //Set Global font
	
	//Call Update methods for widgets that require periodic updates even when not being drawn
	MapWidget::Instance().Update();
	
	//Draw main window
	ImExt::Window::Options mainWinOpts;
	mainWinOpts.Flags = WindowFlags::NoTitleBar | WindowFlags::NoResize | WindowFlags::NoMove | WindowFlags::NoCollapse | WindowFlags::NoSavedSettings |
	                    WindowFlags::MenuBar | WindowFlags::NoDocking;
	mainWinOpts.POpen = &DrawLoopEnabled;
	mainWinOpts.Size(ImVec2(1300,750), Condition::Once);
	if (ImExt::Window window("Recon## Main Window", mainWinOpts); window.ShouldDrawContents()) {	
		DrawModalDialogs();
		
		MainMenu::Instance().Draw();
		
		ImGui::Columns(2, "Main Win H Columns");
		if (firstDrawPass)
			ImGui::SetColumnWidth(0, ImGui::GetWindowWidth()/4.0f);
		
		float childSpacing        = ImGui::GetStyle().ItemSpacing.y;
		float statusBarHeight     = ImGui::GetFontSize();
		float MainRegionHeight    = ImGui::GetContentRegionAvail().y - statusBarHeight - childSpacing;
		float VehiclesChildHeight = VehicleControlWidget::Instance().GetWidgetRecommendedHeight() + 2.0*ImGui::GetStyle().WindowPadding.y;
		float CommandChildHeight  = CommandWidget::Instance().GetWidgetRecommendedHeight() + 2.0*ImGui::GetStyle().WindowPadding.y;
		float ConsoleWidgetHeight = ConsoleWidget::Instance().GetWidgetHeight();
		bool  ConsoleVisible      = ConsoleWidgetHeight >= 1.0f;
		float MapChildtHeight     = ConsoleVisible ? (MainRegionHeight - ConsoleWidgetHeight - childSpacing) : MainRegionHeight;
		
		//Upper-Left Pane - Layer visibility and parameters
		float paneHeight = MainRegionHeight - VehiclesChildHeight - CommandChildHeight - 2.0f*childSpacing;
		ImGui::BeginChild("LayerListPane", ImVec2(0, paneHeight), true);
		VisWidget::Instance().Draw();
		ImGui::EndChild(); //LayerListPane
		
		//Middle-Left Pane - Connected vehicles
		ImGui::BeginChild("VehicleControlWidgetChildWin", ImVec2(0, VehiclesChildHeight), true, ImGuiWindowFlags_NoScrollbar);
		VehicleControlWidget::Instance().Draw();
		ImGui::EndChild();
		
		//Lower-Left Pane - Command Widget
		ImGui::BeginChild("CommandWidgetChildWin", ImVec2(0, CommandChildHeight), true, ImGuiWindowFlags_NoScrollbar);
		CommandWidget::Instance().Draw();
		ImGui::EndChild();
		
		ImGui::NextColumn();
		
		//Upper-right Pane
		ImGui::BeginChild("Map View", ImVec2(0, MapChildtHeight), true);
		MapWidget::Instance().Draw();
		ImGui::EndChild();
		
		//Lower-right Pane
		if (ConsoleVisible) {
			ImGuiWindowFlags flags = 0;
			if (ConsoleWidget::Instance().IsTransitioning())
				flags |= ImGuiWindowFlags_NoScrollbar;
			ImGui::BeginChild("Console", ImVec2(0, ConsoleWidgetHeight), true, flags);
			ConsoleWidget::Instance().Draw();
			ImGui::EndChild();
		}
		
		ImGui::NextColumn();
		ImGui::Columns(1);
		
		//Draw status bar
		StatusBar::Draw();
	}
	
	//Draw secondary singleton windows
	AboutWindow::Instance().Draw();
	SettingsWindow::Instance().Draw();
	SimFiducialsWidget::Instance().Draw();
	
	//Draw secondary non-singleton windows
	DrawChildren();
	
	firstDrawPass = false;
}


