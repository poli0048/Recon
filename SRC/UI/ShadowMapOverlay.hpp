//The Shadow Map overlay is used to draw the current shadow map on the map widget
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "../EigenAliases.h"

class ShadowMapOverlay {
	private:
		std::mutex m_mutex;
		
		int m_callbackHandle;
		ImTextureID m_shadowMapTexture;
		
		Eigen::Vector2d UL_LL; //(Latitude, Longitude) of center of upper-left pixel, in radians
		Eigen::Vector2d UR_LL; //(Latitude, Longitude) of center of upper-right pixel, in radians
		Eigen::Vector2d LL_LL; //(Latitude, Longitude) of center of lower-left pixel, in radians
		Eigen::Vector2d LR_LL; //(Latitude, Longitude) of center of lower-right pixel, in radians
		
		//We latch the current values of the relavent layer settings in our Draw pass so they are available asynchronously
		//to the callback function that evaluates the texture.
		float m_Opacity; //0-100
		std::array<float, 3> m_Color; //Each item between 0 and 1
		
	public:
		 ShadowMapOverlay();
		~ShadowMapOverlay() = default;
		
		//Called in the draw loop for the map widget
		void Draw_Overlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
};




