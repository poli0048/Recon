//The Avoidance Zone Paint Tool holds the state of and provides the draw calls for a tool used for
//editing avoidance zones.
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//External Includes
#include "../../../handycpp/Handy.hpp"
#include "../../../imgui/app/ImGuiApp.hpp"

//Project Includes
#include "../EigenAliases.h"

class AvoidanceZonesTool {
	private:
		Math::Vector2 m_popupDims;
		bool m_popupOpen;

		void DrawTriangle(Math::Vector2 p_min, float scale, ImDrawList * DrawList);
		void Draw_DropDown(Math::Vector2 PopupULCorner, float PopupWidth);

		void DrawTool(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList);
		void ExecuteEdit(Eigen::Vector2d const & CursorPos_NM);
		
	public:
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
		AvoidanceZonesTool();
		~AvoidanceZonesTool() = default;
		
		bool Draw_Button(void); //Returns true on transition from Inactive --> Active
		void Draw_Overlay(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
};




