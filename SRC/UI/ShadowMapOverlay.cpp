//The Shadow Map overlay is used to draw the current shadow map on the map widget
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//External Includes

//Project Includes
#include "ShadowMapOverlay.hpp"
#include "../Modules/Shadow-Detection/ShadowDetection.hpp"
#include "VisWidget.hpp"
#include "TextureUploadFlowRestrictor.hpp"
#include "MapWidget.hpp"
#include "../Maps/MapUtils.hpp"

ShadowMapOverlay::ShadowMapOverlay() {
	m_callbackHandle = ShadowDetection::ShadowDetectionEngine::Instance().RegisterCallback([this](ShadowDetection::InstantaneousShadowMap const & NewMap) {
		//Copy our visualization settings so we don't need to hold onto our mutex while we evaluate the texture
		m_mutex.lock();
		uint8_t red   = (uint8_t) std::round(255.0f*m_Color[0]);
		uint8_t green = (uint8_t) std::round(255.0f*m_Color[1]);
		uint8_t blue  = (uint8_t) std::round(255.0f*m_Color[2]);
		uint8_t alpha = (uint8_t) std::round(255.0f*m_Opacity/100.0f);
		m_mutex.unlock();
		
		std::vector<uint8_t> data(NewMap.Map.rows * NewMap.Map.cols * 4, 0);
		int index = 0;
		for (int row = 0; row < NewMap.Map.rows; row++) {
			for (int col = 0; col < NewMap.Map.cols; col++) {
				uint8_t shadowMapVal = NewMap.Map.at<uint8_t>(row, col);
				if ((shadowMapVal == 255) || (shadowMapVal <= 127)) {
					data[index++] = 0;
					data[index++] = 0;
					data[index++] = 0;
					data[index++] = 0;
				}
				else {
					data[index++] = red;
					data[index++] = green;
					data[index++] = blue;
					data[index++] = alpha;
				}
			}
		}
		TextureUploadFlowRestrictor::Instance().WaitUntilUploadIsAllowed();
		ImTextureID tex = ImGuiApp::Instance().CreateImageRGBA8888(&data[0], NewMap.Map.cols, NewMap.Map.rows);
		
		std::scoped_lock lock(m_mutex);
		m_shadowMapTexture = tex;
		UL_LL = NewMap.UL_LL;
		UR_LL = NewMap.UR_LL;
		LL_LL = NewMap.LL_LL;
		LR_LL = NewMap.LR_LL;
	});
}

void ShadowMapOverlay::Draw_Overlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds) {
	//Nothing to do if the overlay is disabled
	if (! VisWidget::Instance().LayerVisible_ShadowMapOverlay)
		return;
	
	//Don't draw the overlay if the shadow detection module isn't running
	if (! ShadowDetection::ShadowDetectionEngine::Instance().IsRunning())
		return;
	
	std::scoped_lock lock(m_mutex);
	
	//Latch the current settings for the overlay - they will be used on the next evaluation
	m_Opacity = VisWidget::Instance().Opacity_ShadowMapOverlay;
	m_Color   = VisWidget::Instance().ShadowMapColor;
	
	//Convert the necessary corners to NM and then to screen space
	Eigen::Vector2d UL_NM = LatLonToNM(UL_LL);
	Eigen::Vector2d LR_NM = LatLonToNM(LR_LL);
	
	Eigen::Vector2d UL_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(UL_NM);
	Eigen::Vector2d LR_SS = MapWidget::Instance().NormalizedMercatorToScreenCoords(LR_NM);
	
	DrawList->AddImage(m_shadowMapTexture, UL_SS, LR_SS);
}

