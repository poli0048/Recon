//The Survey Region Tool supports drawing and editing vector survey regions. A survey region
//is defined as a union of polygons with optional hole polygons (arbitrarily many in each polygon)
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "../EigenAliases.h"
#include "../SurveyRegionManager.hpp"

//When the tool is active, vertices show up as nodes that can be dragged around (and deleted).
class SurveyRegionsTool {
	//Struct for holding the "address" of a simple polygon component of a polygon collection
	struct SimplePolyAddress {
		int componentIndex = -1;
		int holeIndex = -1; //-1 for boundary, otherwise index of hole
		SimplePolyAddress() = default;
		SimplePolyAddress(int Comp, int Hole) : componentIndex(Comp), holeIndex(Hole) { }
		~SimplePolyAddress() = default;
	};
	
	//Struct for holding the "address" of a single vertex in a polygon collection
	struct VertexAddress {
		int componentIndex = -1;
		int holeIndex = -1; //-1 for boundary, otherwise index of hole
		int vertexIndex = -1;
		VertexAddress() = default;
		VertexAddress(int Comp, int Hole, int Vertex) : componentIndex(Comp), holeIndex(Hole), vertexIndex(Vertex) { }
		VertexAddress(SimplePolyAddress SimplePoly, int Vertex) : componentIndex(SimplePoly.componentIndex), holeIndex(SimplePoly.holeIndex), vertexIndex(Vertex) { }
		~VertexAddress() = default;
		void Reset(void) { componentIndex = -1; holeIndex = -1; vertexIndex = -1; }
		bool operator==(VertexAddress const & Other) const {
			return (componentIndex == Other.componentIndex) && (holeIndex == Other.holeIndex) && (vertexIndex == Other.vertexIndex);
		}
	};
	
	private:
		int m_MessageToken;
		Eigen::Vector2d m_popupDims;
		
		VertexAddress m_editNodeAddress;
		std::Evector<Eigen::Vector2d> m_editPolyLine_NM; //When dragging or creating new object, this is the working copy of the simple polygon under edit

		void DrawTriangle(Eigen::Vector2d const & p_min, float scale, ImDrawList * DrawList);
		void Draw_DropDown(Eigen::Vector2d const & PopupULCorner, float PopupWidth);
		
		void FinishNewPolyOrHole(SurveyRegion * surveyRegion);
		
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		//Fields   ********************************************************************
		bool toolActive;
		
		//The tool has more complex state than the raster tools.
		//0 = Default state (nothing going on)
		//1 = Dragging node
		//2 = Creating polygon or hole - not dragging node
		//3 = Creating polygon or hole - dragging node
		//4 = Deleting Polygon (awaiting click)
		//5 = Deleting Hole (awaiting click)
		int m_toolState = 0;
		
		//Functions   *****************************************************************
		SurveyRegionsTool();
		~SurveyRegionsTool() = default;
		
		bool Draw_Button(void); //Returns true on transition from Inactive --> Active
		void Draw_Overlay(Eigen::Vector2d const & CursorPos_ScreenSpace, Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
};




