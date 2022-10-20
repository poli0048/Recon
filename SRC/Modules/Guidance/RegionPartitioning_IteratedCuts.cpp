//This source file implements the Iterated Cuts survey region partitioning algorithm.
//Authors: Bryan Poling
//Copyright (c) 2022 Sentek Systems, LLC. All rights reserved.

//System Includes

//External Includes
#include "../../eigen/Eigen/LU"
#include "../../eigen/Eigen/QR"
#include "../../eigen/Eigen/Geometry"

//Project Includes
#include "Guidance.hpp"
#include "../../UI/VehicleControlWidget.hpp"
#include "../../Utilities.hpp"
#include "../../Maps/MapUtils.hpp"

#define PI 3.14159265358979323846

// *********************************************************************************************************************************
// *************************************************   Local Function Definitions   ************************************************
// *********************************************************************************************************************************

//Get the approximate area (in m^2) that can be flown in the given number of seconds, given the imaging requirements
static double FlightTimeToApproxArea(double TargetFlightTime, Guidance::ImagingRequirements const & ImagingReqs) {
	double rowSpacing = 2.0 * ImagingReqs.HAG * std::tan(0.5 * ImagingReqs.HFOV) * (1.0 - ImagingReqs.SidelapFraction);
	double coverageRate = rowSpacing * ImagingReqs.TargetSpeed; //m^2/s - not including first pass, which is more productive
	return TargetFlightTime * coverageRate;
}

//Convert an area from m^2 to squared NM units. This is approximate and depends on the NM y coordinate of our operating zone,
//since the distortion of the NM projection varies by latitude
static double Area_SquareMeters_To_NM(double Area_SqMeters, double ApproxY_NM) {
	double NMUnitsPerM = MetersToNMUnits(1.0, ApproxY_NM);
	double NMAreaUnitsPerSqMeter = NMUnitsPerM * NMUnitsPerM;
	return Area_SqMeters * NMAreaUnitsPerSqMeter;
}


// *********************************************************************************************************************************
// ************************************************   Public Function Definitions   ************************************************
// *********************************************************************************************************************************
namespace Guidance {
	//3 - Take a survey region, and break it into sub-regions of similar size that can all be flown in approximately the same flight time (argument).
	//    A good partition uses as few components as possible for a given target execution time. Hueristically, this generally means simple shapes.
	//    This function needs the target drone speed and imaging requirements because they impact what sized region can be flown in a given amount of time.
	//Arguments:
	//Region           - Input  - The input survey region to cover (polygon collection in NM coords)
	//Partition        - Output - A vector of sub-regions, each one a polygon collection in NM coords (typical case will have a single poly in each sub-region)
	//TargetFlightTime - Input  - The approx time (in seconds) a drone should be able to fly each sub-region in, given the max vehicle speed and sidelap
	//ImagingReqs      - Input  - Parameters specifying speed and row spacing (see definitions in struct declaration)
	void PartitionSurveyRegion_IteratedCuts(PolygonCollection const & Region, std::Evector<PolygonCollection> & Partition,
	                                        double TargetFlightTime, ImagingRequirements const & ImagingReqs) {
		//Get an approximate center point for the survey region - use it to convert approx flight time to approx component area
		Eigen::Vector4d AABB(std::nanf(""), std::nanf(""), std::nanf(""), std::nanf(""));
		for (Polygon const & poly : Region.m_components) {
	    		Eigen::Vector4d polyAABB = poly.m_boundary.GetAABB();
	    		if (std::isnan(AABB(0)))
				AABB = polyAABB;
	    		AABB(0) = std::min(AABB(0), polyAABB(0));
	    		AABB(1) = std::max(AABB(1), polyAABB(1));
	    		AABB(2) = std::min(AABB(2), polyAABB(2));
	    		AABB(3) = std::max(AABB(3), polyAABB(3));
    		}
    		double yCenter_NM = 0.5*AABB(2) + 0.5*AABB(3);
    		double compTargetArea_SqMeters = FlightTimeToApproxArea(TargetFlightTime, ImagingReqs);
    		double compTargetArea_NMUnits  = Area_SquareMeters_To_NM(compTargetArea_SqMeters, yCenter_NM);
    		std::cerr << "Comp approx area: " << compTargetArea_SqMeters << " m^2, " << compTargetArea_NMUnits << " squared NM units.\r\n";
		//Note: In at least one test the conversion to m^2 seems correct (agrees with Cheetah flight calc tool)
		//I have not checked the conversion to squared NM units, but we seem to be getting reasonable-looking results

    		//Now, iteratively cut any component of the region that is significantly larger than the target area.
		//Use hueristics to decide on the best number an orientation of cuts. Start by "flattening" the collection - that is,
		//breaking up the survey area into a standard vector of polygons. If the region is valid - the pieces should all be non-overlapping.
		std::Evector<Polygon> pieces = Region.m_components;
		int cutPassNum = 0;
		for (; cutPassNum < 10; cutPassNum++) {
			std::Evector<Polygon> newPieces;
			newPieces.reserve(std::max(size_t(3)*pieces.size(), size_t(10)));
			for (Polygon const & comp : pieces) {
				double compArea = comp.GetArea();
				if (compArea <= 1.5*compTargetArea_NMUnits)
					newPieces.push_back(comp);
				if (compArea > 1.5*compTargetArea_NMUnits) {
					//This component is too large and needs to be split - decide on cutting into 2 or 3 pieces
					int numCuts = (compArea > 2.5*compTargetArea_NMUnits) ? 2 : 1;

					//Find the longest axis and plan to cut the piece orthogonal to the longest axis
					//Eigen::Vector2d VMajor = comp.FindLongestAxis();
					//Eigen::Vector2d VCut = VMajor;

					//Find the shortest axis and plan to cut the piece along lines parallel to shortest axis
					Eigen::Vector2d VMinor = comp.FindShortestAxis();
					Eigen::Vector2d VCut(VMinor(1), -1.0*VMinor(0));

					//Decide where to place the cuts. We seem to have two options here... we can try to find places that
					//result in components of similar area, or we could use a heuristic to place the cuts assuming the piece
					//is nearly rectangular.
					double minDot = std::nanf("");
					double maxDot = std::nanf("");
					for (Eigen::Vector2d const & vert : comp.m_boundary.GetVertices()) {
						double dot = vert.dot(VCut);
						if (std::isnan(minDot) || (dot < minDot))
							minDot = dot;
						if (std::isnan(maxDot) || (dot > maxDot))
							maxDot = dot;
					}
					std::vector<double> cutDots(numCuts);
					for (int cutNum = 0; cutNum < numCuts; cutNum++)
						cutDots[cutNum] = minDot + double(cutNum + 1)/double(numCuts + 1)*(maxDot - minDot);
				
					//We want to make cuts along the lines x.dot(VCut) = c for each c in cutDots
					if (numCuts == 1) {
						std::Evector<Polygon> partsLow  = comp.IntersectWithHalfPlane(VCut, cutDots[0]);
						std::Evector<Polygon> partsHigh = comp.IntersectWithHalfPlane(-1.0*VCut, -1.0*cutDots[0]);
						newPieces.insert(newPieces.end(), partsLow.begin(),  partsLow.end());
						newPieces.insert(newPieces.end(), partsHigh.begin(), partsHigh.end());
					}
					else {
						//We are making 2 cuts
						std::Evector<Polygon> partsA = comp.IntersectWithHalfPlane(VCut, cutDots[0]);
						newPieces.insert(newPieces.end(), partsA.begin(), partsA.end());

						//Now take everything "above" the first cut line and cut with the second line
						std::Evector<Polygon> partsB = comp.IntersectWithHalfPlane(-1.0*VCut, -1.0*cutDots[0]);
						for (Polygon const & part : partsB) {
							std::Evector<Polygon> subPartsLow  = part.IntersectWithHalfPlane(VCut, cutDots[1]);
							std::Evector<Polygon> subPartsHigh = part.IntersectWithHalfPlane(-1.0*VCut, -1.0*cutDots[1]);
							newPieces.insert(newPieces.end(), subPartsLow.begin(),  subPartsLow.end());
							newPieces.insert(newPieces.end(), subPartsHigh.begin(), subPartsHigh.end());
						}
					}
				}
			}
			pieces.swap(newPieces);
			if (pieces.size() == newPieces.size())
				break;
		}
		std::cerr << "Iterated cuts algorithm finished after " << cutPassNum + 1 << " iterations.\r\n";

		//Pack into output data structure
		Partition.clear();
		Partition.reserve(pieces.size());
		for (Polygon const & comp : pieces)
			Partition.emplace_back(comp);

		MapWidget::Instance().m_guidanceOverlay.SetSurveyRegionPartition(Partition);

		//This is for testing purposes only - grab just the first simple ploygon
		/*Partition.clear();
		SimplePolygon const & simplyPoly(Region.m_components[0].m_boundary);
		Eigen::Vector4d compAABB = simplyPoly.GetAABB();

		std::cerr << "Original Polygon:\r\n" << simplyPoly << "\r\n\r\n";

		//Cut parallel to the x axis - half way up the AABB
		double yCut = 0.5*compAABB(2) + 0.5*compAABB(3);
		Eigen::Vector2d VCut(0.0, 1.0);
		std::Evector<SimplePolygon> piecesLow = simplyPoly.IntersectWithHalfPlane(VCut, yCut);
		std::cerr << "Num pieces below cut: " << piecesLow.size() << "\r\n";
		for (auto const & piece : piecesLow) {
			Partition.emplace_back();
			Partition.back().m_components.emplace_back(piece);
		}

		std::Evector<SimplePolygon> piecesHigh = simplyPoly.IntersectWithHalfPlane(-1.0*VCut, -1.0*yCut);
		std::cerr << "Num pieces above cut: " << piecesHigh.size() << "\r\n";
		for (auto const & piece : piecesHigh) {
			Partition.emplace_back();
			Partition.back().m_components.emplace_back(piece);
			std::cerr << piece << "\r\n\r\n";
		}*/

		//Testing purposes only - cut the first polygon
    		/*Partition.clear();
    		Polygon const & poly(Region.m_components[0]);
		//std::cerr << "Init Poly:\r\n" << poly << "\r\n\r\n";
    		Eigen::Vector4d compAABB = poly.m_boundary.GetAABB();
    		double yCut = 0.5*compAABB(2) + 0.5*compAABB(3);
    		Eigen::Vector2d VCut(0.0, 1.0);
    		std::Evector<Polygon> piecesLow = poly.IntersectWithHalfPlane(VCut, yCut);
    		std::cerr << "Num pieces below cut: " << piecesLow.size() << "\r\n";
    		for (auto const & piece : piecesLow) {
    			Partition.emplace_back();
    			Partition.back().m_components.push_back(piece);
    		}

    		std::Evector<Polygon> piecesHigh = poly.IntersectWithHalfPlane(-1.0*VCut, -1.0*yCut);
    		std::cerr << "Num pieces above cut: " << piecesHigh.size() << "\r\n";
    		for (auto const & piece : piecesHigh) {
    			Partition.emplace_back();
    			Partition.back().m_components.push_back(piece);
			//std::cerr << piece << "\r\n\r\n";
    		}*/
	}
}