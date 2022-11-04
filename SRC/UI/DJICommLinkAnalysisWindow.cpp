//The DJI Comm Link Analysis Window shows useful info about packet throughput, jitter, and latency
//to help diagnose comm link problems between Recon and a DJI drone
//Author: Bryan Poling
//Copyright (c) 2022 Sentek Systems, LLC. All rights reserved.

//System Includes
#include <thread>

//Project Includes
#include "DJICommLinkAnalysisWindow.hpp"
#include "../Modules/DJI-Drone-Interface/DroneManager.hpp"

DJICommLinkAnalysisWindow::DJICommLinkAnalysisWindow() { }

void DJICommLinkAnalysisWindow::Draw() {
	ImExt::Window::Options wOpts;
	wOpts.Flags = WindowFlags::NoCollapse | WindowFlags::NoSavedSettings | WindowFlags::NoDocking | WindowFlags::NoTitleBar | WindowFlags::NoResize;
	wOpts.POpen = &Visible;
	wOpts.Size(Math::Vector2(40.0f*ImGui::GetFontSize(), 40.0f*ImGui::GetFontSize()), Condition::Appearing);
	if (ImExt::Window window("DJI Comm Link Analysis Tool", wOpts); window.ShouldDrawContents()) {
		ImExt::Style winPaddingStyle(StyleVar::WindowPadding, Math::Vector2(20.0f));
		
		ImGui::BeginChild("DJI Comm Link Analysis Scrollable Region", ImVec2(0,0), true, ImGuiWindowFlags_AlwaysUseWindowPadding);

		//Get vector of pointers to connected drones
		std::vector<DroneInterface::Drone *> drones;
		std::vector<std::string> droneSerials = DroneInterface::DroneManager::Instance().GetConnectedDroneSerialNumbers();
		for (std::string const & serial : droneSerials) {
			DroneInterface::Drone * dronePtr = DroneInterface::DroneManager::Instance().GetDrone(serial);
			if (dronePtr != nullptr)
				drones.push_back(dronePtr);
		}

		if (drones.empty())
			ImGui::TextUnformatted("DJI Comm Link Analysis Tool. No vehicles connected");
		else {
			//Draw real-time timeline of received packets for all connected drones   ********************************************************
			if (drones.size() == 1U)
				ImGui::TextUnformatted("DJI Comm Link Analysis Tool. Connected to 1 vehicle.");
			else
				ImGui::Text("DJI Comm Link Analysis Tool. Connected to %u vehicles. ", (unsigned int) drones.size());
			ImGui::Text("Live Packet Timeline");
			ImGui::SameLine();
			ImGui::TextDisabled(u8"\uf059");
			if (ImGui::IsItemHovered()) {
				ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
				ImGui::BeginTooltip();
				ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
				ImGui::TextUnformatted("Live view of packets received over the last 10 seconds from each connected drone. "
				                       "Telemetry packets are shown in green (light green for a core telemetry packet and dark "
				                       "green for an extended telemetry packet). Image packets are blue. Acknowledgement packets "
				                       "are purple. Message packets are pink. Any packet that fails its hash check or which was "
				                       "not decoded successfully is shown in red.");
				ImGui::PopTextWrapPos();
				ImGui::EndTooltip();
			}
			ImGui::SameLine();
			ImGui::TextUnformatted(":");

			std::vector<double> Xticks(11U);
			std::vector<std::string> XLabels(11U);
			std::vector<const char *> XLabels_c_str(11U);
			for (int n = 0; n < 11; n++) {
				Xticks[n]  = -1.0*double(n);
				XLabels[n] = std::to_string(n);
				XLabels_c_str[n] = XLabels[n].c_str();
			}

			std::vector<double> Yticks(drones.size());
			std::vector<std::string> YLabels(drones.size());
			std::vector<const char *> YLabels_c_str(drones.size());
			for (int n = 0; n < int(drones.size()); n++) {
				Yticks[n]  = double(n);
				YLabels[n] = drones[n]->GetDroneSerial();
				YLabels_c_str[n] = YLabels[n].c_str();
			}

			ImPlotStyle & plotStyle(ImPlot::GetStyle());
			float plotHeight = (drones.size() + 3U) * 1.5f * ImGui::GetTextLineHeightWithSpacing() + 2*(plotStyle.PlotPadding.y + plotStyle.LabelPadding.y);
			ImPlotFlags flags = ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText;
			ImPlot::SetNextAxesLimits(-10.0, 0.0, -0.75, double(drones.size()) - 0.25, ImPlotCond_Always);
			if (ImPlot::BeginPlot("Drone Data Feeds", ImVec2(-1, plotHeight), flags)) {
				ImPlot::SetupAxes("Time (s) - Right Edge is Now", "Drone #", 0, ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoGridLines | ImPlotAxisFlags_NoTickMarks);
				ImPlot::SetupAxisTicks(ImAxis_X1, &(Xticks[0]), int(Xticks.size()), &(XLabels_c_str[0]));
				ImPlot::SetupAxisTicks(ImAxis_Y1, &(Yticks[0]), int(Yticks.size()), &(YLabels_c_str[0]));

				ImVec2 plotPos_SS  = ImPlot::GetPlotPos();
				ImVec2 plotDims_SS = ImPlot::GetPlotSize();
				float ymin = plotPos_SS.y;
				float ymax = plotPos_SS.y + plotDims_SS.y;

				std::chrono::time_point<std::chrono::steady_clock> nowTime = std::chrono::steady_clock::now();
				ImDrawList * drawList = ImPlot::GetPlotDrawList();
				for (int n = 0; n < int(drones.size()); n++) {
					DroneInterface::Drone * drone = drones[n];
					std::vector<std::tuple<std::chrono::time_point<std::chrono::steady_clock>, int, bool>> packetLog;
					drone->GetMostRecentPacketLog(packetLog);
					for (auto const & item : packetLog) {
						auto timeStamp = std::get<0>(item);
						int  PID       = std::get<1>(item);
						bool decodeRes = std::get<2>(item);

						double secondsOld = SecondsElapsed(timeStamp, nowTime);
						if (secondsOld < 10.0) {
							//Draw mark on timeline to illustrate packet
							ImU32 color     = (ImU32) ImColor(255, 255, 255, 60);
							float width     = 10.0f; //Pixels wide
							float thickness = 0.9f;  //Bar height (0 to 1)
							if ((PID < 0) || (PID > 5) || (! decodeRes)) {
								//Packet is corrupt or at least has unrecognized PID
								color = (ImU32) ImColor(255, 0, 0, 255);
								width = 10.0f;
								thickness = 0.9f;
							}
							else if (PID == 0) {
								//Core telemetry packet
								color = (ImU32) ImColor(80, 255, 80, 128); //Light Green
								width = 6.0f;
								thickness = 0.4f;
							}
							else if (PID == 1) {
								//Extended telemetry packet
								color = (ImU32) ImColor(0, 255, 0, 128); //Dark Green
								width = 4.0f;
								thickness = 0.9f;
							}
							else if ((PID == 2) || (PID == 5)) {
								//Image packet or Compressed Image packet
								color = (ImU32) ImColor(0, 0, 255, 128); //Blue
								width = 6.0f;
								thickness = 0.8f;
							}
							else if (PID == 3) {
								//Ack packet
								color = (ImU32) ImColor(128, 0, 128, 128); //Purple
								width = 4.0f;
								thickness = 0.9f;
							}
							else if (PID == 4) {
								//Message packet
								color = (ImU32) ImColor(255, 192, 203, 128); //Pink
								width = 4.0f;
								thickness = 0.9f;
							}

							float xCenter   = ImPlot::PlotToPixels(-1.0*secondsOld, 0).x;
							float xMin = xCenter - width/2.0f;
							float xMax = xCenter + width/2.0f;
							float yMin = ImPlot::PlotToPixels(0, float(n) - thickness/2.0f).y;
							float yMax = ImPlot::PlotToPixels(0, float(n) + thickness/2.0f).y;
							drawList->AddRectFilled(ImVec2(xMin,yMin), ImVec2(xMax,yMax), color);
						}
					}
				}
				ImPlot::EndPlot();
			}

			ImGui::TextUnformatted("Jitter Histograms");
			ImGui::SameLine();
			ImGui::TextDisabled(u8"\uf059");
			if (ImGui::IsItemHovered()) {
				ImExt::Style tooltipStyle(StyleVar::WindowPadding, Math::Vector2(4.0f));
				ImGui::BeginTooltip();
				ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
				ImGui::TextUnformatted("Plots of the distribution of the time deltas (s) between consecutive telemetry packets. "
				                       "Bins cover the range from 0 to 5 seconds with a resolution of 0.1 seconds. Time deltas greater "
				                       "than 0.5 are accumulated in the last bin. Distributions do not reset - they reflect all "
				                       "packets received since program start from each drone.");
				ImGui::PopTextWrapPos();
				ImGui::EndTooltip();
			}
			ImGui::SameLine();
			ImGui::TextUnformatted(":");

			//Draw jitter histograms
			int layoutRows = (int(drones.size()) + 2) / 3;
			int layoutCols = (int(drones.size()) + layoutRows - 1) / layoutRows;

			ImGuiStyle & style = ImGui::GetStyle();
			float availWidth   = ImGui::GetContentRegionAvail().x;
			float availHeight  = ImGui::GetContentRegionAvail().y;
			float graphWidth   = (availWidth - (2.0f*style.FramePadding.x + style.ItemSpacing.x)*float(layoutCols - 1)) / float(layoutCols);
			float graphHeight  = (availHeight - (2.0f*style.FramePadding.y + style.ItemSpacing.y)*float(layoutRows - 1)) / float(layoutRows);
			graphHeight        = std::max(graphHeight, 100.0f);

			for (int row = 0; row < layoutRows; row++) {
				for (int col = 0; col < layoutCols; col++) {
					int droneIndex = row*layoutCols + col;
					if (droneIndex < (int) drones.size()) {
						//Draw jitter diagram for this drone
						DroneInterface::Drone * drone = drones[droneIndex];
						std::string serial = drone->GetDroneSerial();
						std::vector<double> coreTelemDist;
						std::vector<double> extendedTelemDist;
						drone->GetTelemetryDeltaTDistributions(coreTelemDist, extendedTelemDist);

						if (coreTelemDist.size() != extendedTelemDist.size()) {
							std::cerr << "Internal Error: Skipping plot since Telemetry deltaT dists have different sizes.\r\n";
							continue;
						}

						std::vector<double> extendedTelemDistNeg(extendedTelemDist.size());
						for (size_t n = 0U; n < extendedTelemDist.size(); n++)
							extendedTelemDistNeg[n] = -1.0 * extendedTelemDist[n];

						double maxDeltaT = double(coreTelemDist.size()) / 10.0;
						std::vector<double> Ts(coreTelemDist.size(), 0.0);
						for (size_t n = 0U; n < Ts.size(); n++)
							Ts[n] = 0.1*double(n + 1U);
						
						double maxValCoreTelem = *std::max_element(coreTelemDist.begin(), coreTelemDist.end());
						double maxValExtTelem  = *std::max_element(extendedTelemDist.begin(), extendedTelemDist.end());
						double maxVal          = std::max(maxValExtTelem, maxValCoreTelem);

						ImPlot::SetNextAxesLimits(0.0, maxDeltaT, -1.0*maxVal, maxVal, ImPlotCond_Always);
						std::string plotText = "Drone "s + serial + " Telemetry Jitter"s;
						//flags = ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText;
						flags = ImPlotFlags_NoMouseText;
						if (ImPlot::BeginPlot(plotText.c_str(), ImVec2(graphWidth, graphHeight), flags)) {
							ImPlot::SetupAxes("Inter-Packet Period (s)", "Freq", 0, ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoGridLines | ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels);
							ImPlot::SetupLegend(ImPlotLocation_SouthEast);
							ImPlot::SetNextFillStyle(ImVec4(0.0,0.0,1.0,1.0)); //Blue
							ImPlot::PlotBars("Core Telemetry", &(Ts[0]), &(coreTelemDist[0]), (int) Ts.size(), 0.1, 0);

							ImPlot::SetNextFillStyle(ImVec4(1.0,0.0,0.0,1.0)); //Red
							ImPlot::PlotBars("Ext Telemetry", &(Ts[0]), &(extendedTelemDistNeg[0]), (int) Ts.size(), 0.1, 0);

							ImPlot::EndPlot();
						}

						if (col + 1 < layoutCols)
							ImGui::SameLine();
					}
				}
			}
		}
		
		ImGui::EndChild();
	}
}




