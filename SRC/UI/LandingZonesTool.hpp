//The Landing Zone Paint Tool holds the state of and provides the draw calls for a tool used for
//editing safe landing zones.
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "../EigenAliases.h"

class LandingZonesTool {
	private:
		Eigen::Vector2d m_popupDims;
		bool m_popupOpen;

		void DrawTriangle(Eigen::Vector2d const & p_min, float scale, ImDrawList * DrawList);
		void Draw_DropDown(Eigen::Vector2d const & PopupULCorner, float PopupWidth);

		void DrawTool(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList);
		void ExecuteEdit(Eigen::Vector2d const & CursorPos_NM);
		
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		//Fields   ********************************************************************
		bool toolActive;
		
		int m_shape; //0=circle, 1=rectangle
		
		//Settings for circles
		float m_radius; //in meters
		
		//Settings for squares
		float m_lengthX;  //in meters
		float m_lengthY;  //in meters
		float m_angleDeg; //in degrees (special exception so ImGui can expose in degrees without a conversion)
		
		//Edit Mode
		int m_EditMode; //0 = Draw, 1 = Erase
		
		//Functions   *****************************************************************
		LandingZonesTool();
		~LandingZonesTool() = default;
		
		bool Draw_Button(void); //Returns true on transition from Inactive --> Active
		void Draw_Overlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
};




