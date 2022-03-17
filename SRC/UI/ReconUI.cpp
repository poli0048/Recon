//Main Window and provider of outer draw loop - all other window draw loops are called
//from within our Draw member function.

//System Includes
#include <memory>
#include <cmath>

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
#include "LiveFiducialsWidget.hpp"
#include "GNSSReceiverWindow.hpp"
#include "../Utilities.hpp"

#define PI 3.14159265358979

ReconUI::ReconUI() {
	gSoloud.init(); //Initialize the SoLoud engine
}

ReconUI::~ReconUI() {
	gSoloud.deinit(); //De-initialize the SoLoud engine
}

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
	mainWinOpts.Size(ImVec2(1400,850), Condition::Once);
	if (ImExt::Window window("Recon## Main Window", mainWinOpts); window.ShouldDrawContents()) {	
		DrawModalDialogs();
		
		MainMenu::Instance().Draw();
		
		ImGui::Columns(2, "Main Win H Columns");
		if (firstDrawPass)
			ImGui::SetColumnWidth(0, 21.0f*ImGui::GetFontSize());
		
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
	LiveFiducialsWidget::Instance().Draw();
	GNSSReceiverWindow::Instance().Draw();
	
	//Draw secondary non-singleton windows
	DrawChildren();
	
	//Draw cursor, if high contrast cursor is enabled
	if (ProgOptions::Instance()->HighContrastCursor) {
		ImGui::SetMouseCursor(ImGuiMouseCursor_None);
		ImGuiViewport * hoveredViewport = ImGui::FindViewportByID(ImGui::GetIO().MouseHoveredViewport);
		if (hoveredViewport != nullptr) {
			ImDrawList * drawlist = ImGui::GetForegroundDrawList(hoveredViewport);
			//For some reason, anti-aliasing doesn't work when accessing a viewport draw list directly
			//auto prevFlags = drawlist->Flags;
			//drawlist->Flags = (drawlist->Flags | ImDrawListFlags_AntiAliasedFill);
			
			Eigen::Matrix2d R;
			double theta = -20.0*PI/180.0;
			//double L = 60.0;
			double L = 3.0*ImGui::GetFontSize();
			R << cos(theta), -sin(theta),
			     sin(theta),  cos(theta);
			Eigen::Vector2d v(0, L);
			
			Eigen::Vector2d p1 = ImGui::GetMousePos();
			Eigen::Vector2d p2 = p1 + v;
			Eigen::Vector2d p3 = p1 + 0.8*R*v;
			Eigen::Vector2d p4 = p1 + R*R*v;
			drawlist->AddTriangleFilled(p1, p2, p3, IM_COL32_BLACK);
			drawlist->AddTriangleFilled(p1, p3, p4, IM_COL32_BLACK);
			
			double t = std::fmod(SecondsSinceT0Epoch(), 2.0) - 1.0;
			double s = 1.0 - t*t;
			//ImColor color(255, 255, int(255.0*s), 255); //Flash yellow and white
			//ImColor color(255, int(255.0*s), int(255.0*s), 255); //Flash red and white
			ImU32 color = IM_COL32_WHITE;
			
			Eigen::Vector2d w = 0.6*v;
			Eigen::Vector2d q1 = p1 + 0.2*L*R*Eigen::Vector2d(0,1);
			Eigen::Vector2d q2 = q1 + w;
			Eigen::Vector2d q3 = q1 + 0.8*R*w;
			Eigen::Vector2d q4 = q1 + R*R*w;
			drawlist->AddTriangleFilled(q1, q2, q3, color);
			drawlist->AddTriangleFilled(q1, q3, q4, color);
			
			//drawlist->Flags = prevFlags;
		}
	}
	
	firstDrawPass = false;
}


