//This window shows info from a connected GNSS receiver
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
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
#include "../Utilities.hpp"

class GNSSReceiverWindow {
	public:
		GNSSReceiverWindow() { }
		~GNSSReceiverWindow() = default;
		
		static GNSSReceiverWindow & Instance() { static GNSSReceiverWindow win; return win; }
	
		void Draw();
		
		bool Visible = false;
};

inline void GNSSReceiverWindow::Draw() {
	ImExt::Window::Options wOpts;
	wOpts.Flags = WindowFlags::NoCollapse | WindowFlags::NoSavedSettings | WindowFlags::NoDocking | WindowFlags::NoTitleBar | WindowFlags::NoResize;
	wOpts.POpen = &Visible;
	wOpts.Size(Math::Vector2(40.0f*ImGui::GetFontSize(), 40.0f*ImGui::GetFontSize()), Condition::Appearing);
	if (ImExt::Window window("GNSS Receiver", wOpts); window.ShouldDrawContents()) {
		ImExt::Style style(StyleVar::WindowPadding, Math::Vector2(20.0f));
		float col2Start = 10.0f*ImGui::GetFontSize();
		float col3Start = 19.0f*ImGui::GetFontSize();
		float col4Start = 31.0f*ImGui::GetFontSize();
		
		ImGui::BeginChild("GNSS Receiver Scrollable Region", ImVec2(0,0), true, ImGuiWindowFlags_AlwaysUseWindowPadding);
		ImGui::TextUnformatted("Receiver connected: ");
		ImGui::SameLine(col2Start);
		ImGui::TextUnformatted(GNSSReceiver::GNSSManager::Instance().IsConnected() ? "Yes" : "No");
		ImGui::NewLine();
		
		Eigen::Vector3d Pos_LLA;
		double   PosAccuracyHor  = 0.0;
		double   PosAccuracyVert = 0.0;
		double   PosAccuracy3D   = 0.0;
		uint32_t GPS_Week        = 0U;
		double   GPS_TOW         = std::nan("");
		std::chrono::time_point<std::chrono::steady_clock> posTimestamp, posAccuracyTimestamp, GPST_Timestamp;
		bool posValid = GNSSReceiver::GNSSManager::Instance().GetPosition_LLA(Pos_LLA, posTimestamp);
		bool posAccValid = GNSSReceiver::GNSSManager::Instance().GetPositionAccuracy(PosAccuracyHor, PosAccuracyVert, PosAccuracy3D, posAccuracyTimestamp);
		bool timeValid = GNSSReceiver::GNSSManager::Instance().GetGPSTime(GPS_Week, GPS_TOW, GPST_Timestamp);
		if (posValid) {
			double PI = 3.14159265358979;
			ImGui::TextUnformatted("WGS84 Latitude:");
			ImGui::SameLine(col2Start);
			ImGui::Text("%.6f\u00B0", Pos_LLA(0)*180.0/PI);
			
			if (posAccValid) {
				ImGui::SameLine(col3Start);
				ImGui::TextUnformatted("Position Accuracy (2D):");
				ImGui::SameLine(col4Start);
				ImGui::Text("%.2f meters", PosAccuracyHor);
			}
			
			ImGui::TextUnformatted("WGS84 Longitude:");
			ImGui::SameLine(col2Start);
			ImGui::Text("%.6f\u00B0", Pos_LLA(1)*180.0/PI);
			
			if (posAccValid) {
				ImGui::SameLine(col3Start);
				ImGui::TextUnformatted("Position Accuracy (Vertical):");
				ImGui::SameLine(col4Start);
				ImGui::Text("%.2f meters", PosAccuracyVert);
			}
			
			ImGui::TextUnformatted("WGS84 Altitude:");
			ImGui::SameLine(col2Start);
			ImGui::Text("%.2f meters", Pos_LLA(2));
			
			if (posAccValid) {
				ImGui::SameLine(col3Start);
				ImGui::TextUnformatted("Position Accuracy (3D):");
				ImGui::SameLine(col4Start);
				ImGui::Text("%.2f meters", PosAccuracy3D);
			}
			
			ImGui::TextUnformatted("Age of solution:");
			ImGui::SameLine(col2Start);
			double age = SecondsElapsed(posTimestamp);
			if (age < 1.0)
				ImGui::TextUnformatted("< 1 second");
			else
				ImGui::Text("%.1f seconds", age);
			
			if (timeValid) {
				ImGui::SameLine(col3Start);
				ImGui::TextUnformatted("GPS Week:");
				ImGui::SameLine(col4Start);
				ImGui::Text("%u", (unsigned int) GPS_Week);
			}
			
			if (ImGui::SmallButton("        Copy Position to Clipboard        ")) {
				std::ostringstream out;
				out << std::fixed << std::setprecision(6) << Pos_LLA(0)*180.0/PI << ", " << Pos_LLA(1)*180.0/PI << ", ";
				out << std::setprecision(2) << Pos_LLA(2);
				std::string clipboardText = out.str();
				ImGui::SetClipboardText(clipboardText.c_str());
			}
			
			if (timeValid) {
				ImGui::SameLine(col3Start);
				ImGui::TextUnformatted("GPS TOW:");
				ImGui::SameLine(col4Start);
				
				unsigned int hours = (unsigned int) std::floor(GPS_TOW / 3600.0);
				unsigned int minutes = (unsigned int) std::floor((GPS_TOW - 3600.0*double(hours)) / 60.0);
				double seconds = GPS_TOW - 3600.0*double(hours) - 60.0*double(minutes);
				
				ImGui::Text("%u:%u:%.0f (H:M:S)", hours, minutes, seconds);
			}
		}
		else {
			ImGui::TextUnformatted("Position:");
			ImGui::SameLine(col2Start);
			ImGui::TextUnformatted("No Solution Yet");
		}
		ImGui::NewLine();
		
		//Test button for timestamp-to-GPST conversion
		/*if (ImGui::Button("Convert Timestamp to GPST")) {
			std::cerr << "Current time:\r\n";
			std::cerr << "Week = " << (unsigned int) GPS_Week << " TOW = " << std::fixed << std::setprecision(6) << GPS_TOW << " s\r\n";
			
			std::chrono::time_point<std::chrono::steady_clock> tp = std::chrono::steady_clock::now();
			uint32_t tp_GPS_Week = 0U;
			double tp_GPS_TOW = std::nan("");
			if (GNSSReceiver::GNSSManager::Instance().TimestampToGPSTime(tp, tp_GPS_Week, tp_GPS_TOW)) {
				std::cerr << "Week = " << (unsigned int) tp_GPS_Week << " TOW = " << std::fixed << std::setprecision(6) << tp_GPS_TOW << " s\r\n";
			}
			else
				std::cerr << "Time conversion failure.\r\n";
		}*/
		
		int SatCount_GPS      = -1;
		int SatCount_SBAS     = -1;
		int SatCount_Galileo  = -1;
		int SatCount_BeiDou   = -1;
		int SatCount_IMES     = -1;
		int SatCount_QZSS     = -1;
		int SatCount_GLONASS  = -1;
		int SatsWithCodeLock  = -1;
		int SatsWithPhaseLock = -1;
		std::unordered_map<std::tuple<uint8_t,uint8_t>,uint8_t> CN0s;
		std::chrono::time_point<std::chrono::steady_clock> SigInfoTimestamp;
		if (GNSSReceiver::GNSSManager::Instance().GetSigInfo(SatCount_GPS,  SatCount_SBAS, SatCount_Galileo, SatCount_BeiDou,
		                                                     SatCount_IMES, SatCount_QZSS, SatCount_GLONASS, SatsWithCodeLock,
		                                                     SatsWithPhaseLock, CN0s, SigInfoTimestamp)) {
			ImGui::TextUnformatted("Sats with code lock:");
			ImGui::SameLine(col2Start);
			ImGui::Text("%d", SatsWithCodeLock);
			
			ImGui::TextUnformatted("Sats with phase lock:");
			ImGui::SameLine(col2Start);
			ImGui::Text("%d", SatsWithPhaseLock);
			
			ImGui::TextUnformatted("Usable sats:");
			ImGui::SameLine(col2Start);
			if (SatCount_GPS > 0) {
				if (SatCount_SBAS + SatCount_Galileo + SatCount_BeiDou + SatCount_IMES + SatCount_QZSS + SatCount_GLONASS > 0)
					ImGui::Text("%d GPS, ", SatCount_GPS);
				else
					ImGui::Text("%d GPS", SatCount_GPS);
				ImGui::SameLine();
			}
			if (SatCount_SBAS > 0) {
				if (SatCount_Galileo + SatCount_BeiDou + SatCount_IMES + SatCount_QZSS + SatCount_GLONASS > 0)
					ImGui::Text("%d SBAS, ", SatCount_SBAS);
				else
					ImGui::Text("%d SBAS", SatCount_SBAS);
				ImGui::SameLine();
			}
			if (SatCount_Galileo > 0) {
				if (SatCount_BeiDou + SatCount_IMES + SatCount_QZSS + SatCount_GLONASS > 0)
					ImGui::Text("%d Galileo, ", SatCount_Galileo);
				else
					ImGui::Text("%d Galileo", SatCount_Galileo);
				ImGui::SameLine();
			}
			if (SatCount_BeiDou > 0) {
				if (SatCount_IMES + SatCount_QZSS + SatCount_GLONASS > 0)
					ImGui::Text("%d BeiDou, ", SatCount_BeiDou);
				else
					ImGui::Text("%d BeiDou", SatCount_BeiDou);
				ImGui::SameLine();
			}
			if (SatCount_IMES > 0) {
				if (SatCount_QZSS + SatCount_GLONASS > 0)
					ImGui::Text("%d IMES, ", SatCount_IMES);
				else
					ImGui::Text("%d IMES", SatCount_IMES);
				ImGui::SameLine();
			}
			if (SatCount_QZSS > 0) {
				if (SatCount_GLONASS > 0)
					ImGui::Text("%d QZSS, ", SatCount_QZSS);
				else
					ImGui::Text("%d QZSS", SatCount_QZSS);
				ImGui::SameLine();
			}
			if (SatCount_GLONASS > 0) {
				ImGui::Text("%d GLONASS", SatCount_GLONASS);
				ImGui::SameLine();
			}
			ImGui::NewLine();
			ImGui::NewLine();
			
			//Pack C/N0's into a vector and sort (by GNSSID, then by SVID)
			std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> GNSSID_SVID_CN0_Vec;
			GNSSID_SVID_CN0_Vec.reserve(CN0s.size());
			for (auto const & kv : CN0s) {
				if (kv.second > 0U)
					GNSSID_SVID_CN0_Vec.push_back(std::make_tuple(std::get<0>(kv.first), std::get<1>(kv.first), kv.second));
			}
			std::sort(GNSSID_SVID_CN0_Vec.begin(), GNSSID_SVID_CN0_Vec.end());
			
			std::vector<uint8_t> CN0Vec;
			CN0Vec.reserve(GNSSID_SVID_CN0_Vec.size());
			for (auto const & item : GNSSID_SVID_CN0_Vec)
				CN0Vec.push_back(std::get<2>(item));
			
			std::vector<double> Yticks(GNSSID_SVID_CN0_Vec.size());
			std::vector<std::string> YLabels(GNSSID_SVID_CN0_Vec.size());
			std::vector<const char *> YLabels_c_str(GNSSID_SVID_CN0_Vec.size());
			for (int n = 0; n < int(GNSSID_SVID_CN0_Vec.size()); n++) {
				Yticks[n] = double(n);
				switch (std::get<0>(GNSSID_SVID_CN0_Vec[n])) {
					case 0:  YLabels[n] = "GPS - "s + std::to_string(std::get<1>(GNSSID_SVID_CN0_Vec[n]));     break;
					case 1:  YLabels[n] = "SBAS - "s + std::to_string(std::get<1>(GNSSID_SVID_CN0_Vec[n]));    break;
					case 2:  YLabels[n] = "Galileo - "s + std::to_string(std::get<1>(GNSSID_SVID_CN0_Vec[n])); break;
					case 3:  YLabels[n] = "BeiDou - "s + std::to_string(std::get<1>(GNSSID_SVID_CN0_Vec[n]));  break;
					case 4:  YLabels[n] = "IMES - "s + std::to_string(std::get<1>(GNSSID_SVID_CN0_Vec[n]));    break;
					case 5:  YLabels[n] = "QZSS - "s + std::to_string(std::get<1>(GNSSID_SVID_CN0_Vec[n]));    break;
					case 6:  YLabels[n] = "GLONASS - "s + std::to_string(std::get<1>(GNSSID_SVID_CN0_Vec[n])); break;
					default: YLabels[n] = "??? - "s + std::to_string(std::get<1>(GNSSID_SVID_CN0_Vec[n]));     break;
				}
				YLabels_c_str[n] = YLabels[n].c_str();
			}
			
			ImPlotStyle & plotStyle(ImPlot::GetStyle());
			float plotHeight = (Yticks.size() + 2U) * ImGui::GetTextLineHeightWithSpacing() + 2*(plotStyle.PlotPadding.y + plotStyle.LabelPadding.y);
			ImPlotFlags flags = ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText;
			ImPlot::SetNextAxesLimits(0.0, 52.0, -0.75, double(CN0Vec.size()) - 0.25, ImPlotCond_Always);
			if (ImPlot::BeginPlot("C/N0 (dBHz)", ImVec2(-1, plotHeight), flags)) {
				ImPlot::SetupAxes("C/N0", "Sat", ImPlotAxisFlags_NoLabel, ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoGridLines | ImPlotAxisFlags_NoTickMarks);
				ImPlot::SetupAxisTicks(ImAxis_X1, 10.0, 50.0, 9);
				ImPlot::SetupAxisTicks(ImAxis_Y1, &(Yticks[0]), int(Yticks.size()), &(YLabels_c_str[0]));

				ImVec2 plotPos_SS = ImPlot::GetPlotPos();
				ImVec2 plotDims_SS = ImPlot::GetPlotSize();
				float ymin = plotPos_SS.y;
				float ymax = plotPos_SS.y + plotDims_SS.y;
				float x0 = plotPos_SS.x;
				float x1 = ImPlot::PlotToPixels(35, 0).x;
				float x2 = ImPlot::PlotToPixels(42, 0).x;
				float x3 = plotPos_SS.x + plotDims_SS.x;
				
				ImDrawList * drawList = ImPlot::GetPlotDrawList();
				drawList->AddRectFilled(ImVec2(x0,ymin), ImVec2(x1,ymax), (ImU32)ImColor(255, 0, 0, 60));
				drawList->AddRectFilled(ImVec2(x1,ymin), ImVec2(x2,ymax), (ImU32)ImColor(255, 255, 0, 60));
				drawList->AddRectFilled(ImVec2(x2,ymin), ImVec2(x3,ymax), (ImU32)ImColor(0, 255, 0, 60));
				
				ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1,1,1,1));
				ImPlot::PlotBarsH<uint8_t>("##C/N0s", &(CN0Vec[0]), int(CN0Vec.size()), 0.67, 0, 0);
				ImPlot::PopStyleColor();
				ImPlot::EndPlot();
			}
			
		}
		else {
			ImGui::TextUnformatted("Signal Info:");
			ImGui::SameLine(col2Start);
			ImGui::TextUnformatted("Not Available");
		}
		
		ImGui::EndChild();
	}
}




