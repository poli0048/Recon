//The Time Available overlay is used to render up-to-date time available functions on the map widget
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.

//External Includes
#include <algorithm>
#include <limits>

//Project Includes
#include "TimeAvailableOverlay.hpp"
#include "../Modules/Shadow-Propagation/ShadowPropagation.hpp"
#include "VisWidget.hpp"
#include "TextureUploadFlowRestrictor.hpp"
#include "MapWidget.hpp"
#include "../Maps/MapUtils.hpp"
#include "../Colormaps.hpp"

static void evaluateRGBFromColormap(uint16_t Value, uint16_t MinValue, uint16_t MaxValue, std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> const & cmap,
	                               uint8_t & Red, uint8_t & Green, uint8_t & Blue) {
		if (cmap.size() == 0U) {
			Red   = 0U;
			Green = 0U;
			Blue  = 0U;
			return;
		}
		else if ((cmap.size() == 1U) || (MaxValue <= MinValue) || (Value <= MinValue)) {
			Red   = std::get<0>(cmap[0]);
			Green = std::get<1>(cmap[0]);
			Blue  = std::get<2>(cmap[0]);
			return;
		}
		else if (Value >= MaxValue) {
			Red   = std::get<0>(cmap.back());
			Green = std::get<1>(cmap.back());
			Blue  = std::get<2>(cmap.back());
			return;
		}
		else {
			double t = double(Value - MinValue)/double(MaxValue - MinValue);
			double scaled_t = t*((double) (cmap.size() - 1));
			int interval = std::clamp((int) floor(scaled_t), 0, (int) cmap.size() - 2);
			std::tuple<uint8_t,uint8_t,uint8_t> c1 = cmap[interval];
			std::tuple<uint8_t,uint8_t,uint8_t> c2 = cmap[interval + 1];
			double s = scaled_t - (double) interval;
			
			Red   = (uint8_t) std::clamp(std::round((1.0 - s)*((double) std::get<0>(c1)) + s*((double) std::get<0>(c2))), 0.0, 255.0);
			Green = (uint8_t) std::clamp(std::round((1.0 - s)*((double) std::get<1>(c1)) + s*((double) std::get<1>(c2))), 0.0, 255.0);
			Blue  = (uint8_t) std::clamp(std::round((1.0 - s)*((double) std::get<2>(c1)) + s*((double) std::get<2>(c2))), 0.0, 255.0);
			return;
		}
	}

TimeAvailableOverlay::TimeAvailableOverlay() {
	m_callbackHandle = ShadowPropagation::ShadowPropagationEngine::Instance().RegisterCallback([this](ShadowPropagation::TimeAvailableFunction const & TAFun) {
		//Copy our visualization settings so we don't need to hold onto our mutex while we evaluate the texture
		m_mutex.lock();
		uint8_t alpha = (uint8_t) std::round(255.0f*m_Opacity/100.0f);
		m_mutex.unlock();
		
		//Set hard-coded vis parameters
		std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> const & cmap = Colormaps::GetColormap(Colormap::RedToBlue);
		uint16_t CmapMinVal = 0;  //Time available for low end of colormap (seconds)
		uint16_t CmapMaxVal = 10; //Time available for high end of colormap (seconds)
		
		std::vector<uint8_t> data(TAFun.TimeAvailable.rows * TAFun.TimeAvailable.cols * 4, 0);
		int index = 0;
		for (int row = 0; row < TAFun.TimeAvailable.rows; row++) {
			for (int col = 0; col < TAFun.TimeAvailable.cols; col++) {
				uint16_t TAFunVal = TAFun.TimeAvailable.at<uint16_t>(row, col);
				if (TAFunVal == std::numeric_limits<uint16_t>::max()) {
					data[index++] = 0;
					data[index++] = 0;
					data[index++] = 0;
					data[index++] = 0;
				}
				else {
					uint8_t R, G, B;
					evaluateRGBFromColormap(TAFunVal, CmapMinVal, CmapMaxVal, cmap, R, G, B);
					data[index++] = R;
					data[index++] = G;
					data[index++] = B;
					data[index++] = alpha;
				}
			}
		}
		TextureUploadFlowRestrictor::Instance().WaitUntilUploadIsAllowed();
		ImTextureID tex = ImGuiApp::Instance().CreateImageRGBA8888(&data[0], TAFun.TimeAvailable.cols, TAFun.TimeAvailable.rows);
		
		std::scoped_lock lock(m_mutex);
		m_TimeAvailableTexture = tex;
		UL_LL = TAFun.UL_LL;
		UR_LL = TAFun.UR_LL;
		LL_LL = TAFun.LL_LL;
		LR_LL = TAFun.LR_LL;
	});
}

void TimeAvailableOverlay::Draw_Overlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
	//Nothing to do if the overlay is disabled
	if (! VisWidget::Instance().LayerVisible_TimeAvailableOverlay)
		return;
	
	//Don't draw the overlay if the shadow propagation module isn't running
	if (! ShadowPropagation::ShadowPropagationEngine::Instance().IsRunning())
		return;
	
	std::scoped_lock lock(m_mutex);
	
	//Latch the current settings for the overlay - they will be used on the next evaluation
	m_Opacity = VisWidget::Instance().Opacity_TimeAvailableOverlay;
	
	//Convert the necessary corners to NM and then to screen space
	Eigen::Vector2d UL_NM = LatLonToNM(UL_LL);
	Eigen::Vector2d LR_NM = LatLonToNM(LR_LL);
	
	Eigen::Vector2d UL_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(UL_NM);
	Eigen::Vector2d LR_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(LR_NM);
	
	DrawList->AddImage(m_TimeAvailableTexture, UL_SS, LR_SS);
}

