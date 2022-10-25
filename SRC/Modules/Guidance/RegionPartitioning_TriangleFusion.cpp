//This source file implements the triangle fusion survey region partitioning algorithm.
//This was largely developed at Stanford University by Elaina Chai, Ben Choi, and Owen Queeglay.
//The code was cleaned up, refactored, and migrated to this source file by Bryan Poling at Sentek Systems, LLC
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
#define TOLERANCE 1e-10

// *********************************************************************************************************************************
// ***************************************************   Local Data Structures   ***************************************************
// *********************************************************************************************************************************

class TriangleAdjacencyMap {
public:
	struct Triangle {
		int id;
		int score;
	};
	std::vector<Triangle> nodes;
	std::vector<std::vector<int>> edges;
	std::vector<std::vector<int>> groups;
};

// *********************************************************************************************************************************
// *************************************************   Local Function Definitions   ************************************************
// *********************************************************************************************************************************
static bool PointsAreColinear(Eigen::Vector2d const & p1, Eigen::Vector2d const & p2, Eigen::Vector2d const & p3) {
	//To not be co-linear, all 3 points must be distinct.
	if ((p2 - p1).norm() < TOLERANCE)
		return true;
	if ((p3 - p1).norm() < TOLERANCE)
		return true;
	if ((p3 - p2).norm() < TOLERANCE)
		return true;

	//If the points are distinct, we can test orthogonality of the vector from 1 to 3 with the vector from 1 to 2
	Eigen::Vector2d v = p2 - p1;
	v.normalize();
	Eigen::Matrix2d R;
	R << 0, -1,
	     1,  0;
	Eigen::Vector2d w = R * v;
	return (fabs(w.dot(p3 - p1)) < TOLERANCE);
}

// Bisects a given triangle ABC by taking the midpoint of the largest side and cutting that in half
static void BisectTriangle(Triangle const & SourceTriangle, Triangle & SubTriangleADC, Triangle & SubTriangleABD) {
	Triangle tempTriangle = SourceTriangle;
	if ((SourceTriangle.m_pointA - SourceTriangle.m_pointB).norm() >= (SourceTriangle.m_pointA - SourceTriangle.m_pointC).norm() &&
	    (SourceTriangle.m_pointA - SourceTriangle.m_pointB).norm() >= (SourceTriangle.m_pointB - SourceTriangle.m_pointC).norm()) {
		tempTriangle.m_pointA = SourceTriangle.m_pointC;
		tempTriangle.m_pointB = SourceTriangle.m_pointA;
		tempTriangle.m_pointC = SourceTriangle.m_pointB;
	}
	else if ((SourceTriangle.m_pointA - SourceTriangle.m_pointC).norm() >= (SourceTriangle.m_pointA - SourceTriangle.m_pointB).norm() &&
	         (SourceTriangle.m_pointA - SourceTriangle.m_pointC).norm() >= (SourceTriangle.m_pointB - SourceTriangle.m_pointC).norm()) {
		tempTriangle.m_pointA = SourceTriangle.m_pointB;
		tempTriangle.m_pointB = SourceTriangle.m_pointC;
		tempTriangle.m_pointC = SourceTriangle.m_pointA;
	}

	Eigen::Vector2d m_pointD;
	m_pointD = (tempTriangle.m_pointB + tempTriangle.m_pointC) / 2;
	SubTriangleADC = tempTriangle;
	SubTriangleABD = tempTriangle;
	SubTriangleADC.m_pointB = m_pointD;
	SubTriangleABD.m_pointC = m_pointD;
}

// Like Bisect, but we bisect to the intersection
// Return false if no intersection. Do not use triangles
static bool BisectIntersection(Triangle const & SourceTriangle, Eigen::Vector2d const & Intersection, Triangle & subTriangleADC, Triangle & subTriangleABD) {
	Triangle tempTriangle = SourceTriangle;
	Eigen::Vector2d uniqueP1;
	Eigen::Vector2d uniqueP2;

	// Find the opposite corner
	bool pC = PointsAreColinear(SourceTriangle.m_pointA, SourceTriangle.m_pointB, Intersection);
	bool pB = PointsAreColinear(SourceTriangle.m_pointA, SourceTriangle.m_pointC, Intersection);
	bool pA = PointsAreColinear(SourceTriangle.m_pointC, SourceTriangle.m_pointB, Intersection);

	if (!(pC || pB || pA))
		return false;

	tempTriangle.m_pointA = Intersection;
	if (pC) {
		tempTriangle.m_pointB = SourceTriangle.m_pointC;
		uniqueP1 = SourceTriangle.m_pointA;
		uniqueP2 = SourceTriangle.m_pointB;
	}
	else if (pB) {
		tempTriangle.m_pointB = SourceTriangle.m_pointB;
		uniqueP1 = SourceTriangle.m_pointA;
		uniqueP2 = SourceTriangle.m_pointC;
	}
	else if (pA) {
		tempTriangle.m_pointB = SourceTriangle.m_pointA;
		uniqueP1 = SourceTriangle.m_pointB;
		uniqueP2 = SourceTriangle.m_pointC;
	}
	else
		return false;

	subTriangleADC = tempTriangle;
	subTriangleABD = tempTriangle;
	subTriangleADC.m_pointC = uniqueP1;
	subTriangleABD.m_pointC = uniqueP2;
	return true;
}

static SimplePolygon TriangleToSimplyPoly(Triangle const & T) {
	return SimplePolygon({T.m_pointA, T.m_pointB, T.m_pointC});
}

//Get the area (m^2) of a triangle whose coordinates are in NM. This is the area assuming each point is at a WGS84 altitude of 0.
//That is, the triangle is assumed to be right on the reference ellipsoid. For relatively small triangles, this will be a very close
//approximation to the area at different altitudes.
static double GetArea(Triangle const & T) {
	//Convert each point in the triangle to ECEF
	Eigen::Vector2d A_latlon = NMToLatLon(T.m_pointA);
	Eigen::Vector2d B_latlon = NMToLatLon(T.m_pointB);
	Eigen::Vector2d C_latlon = NMToLatLon(T.m_pointC);
	Eigen::Vector3d A_ECEF   = LLA2ECEF(Eigen::Vector3d(A_latlon(0), A_latlon(1), 0.0));
	Eigen::Vector3d B_ECEF   = LLA2ECEF(Eigen::Vector3d(B_latlon(0), B_latlon(1), 0.0));
	Eigen::Vector3d C_ECEF   = LLA2ECEF(Eigen::Vector3d(C_latlon(0), C_latlon(1), 0.0));

	Eigen::Vector3d ab_vector = A_ECEF - B_ECEF;
	Eigen::Vector3d ac_vector = B_ECEF - C_ECEF;
	return 0.5*(ac_vector.cross(ab_vector)).norm();
}

static bool insideColinear(LineSegment const lineA, Eigen::Vector2d pointB) {
	// Check if colinear; return false otherwise
	if (! PointsAreColinear(lineA.m_endpoint1, lineA.m_endpoint2, pointB))
		return false;

	// Check if pointB equals one of the end points
	// If true then it is an endpoint and thus NOT inside the line
	if (((lineA.m_endpoint1 - pointB).norm() < TOLERANCE) || ((lineA.m_endpoint2 - pointB).norm() < TOLERANCE))
		return false;

	// Now we check if pointB falls within lineA
	double normA1 = lineA.m_endpoint1.norm();
	double normA2 = lineA.m_endpoint2.norm();
	double normB  = pointB.norm();

	// Check if point B1 falls within lineA
	// if A1 < A2, then A1 < B1 < A2
	bool t3 = false;
	if (normA1 < normA2)
		t3 = (normB > normA1 + TOLERANCE) && (normB < normA2 - TOLERANCE);
	
	// Check in case A2 < A1
	else if (normA2 < normA1)
		t3 = (normB > normA2 + TOLERANCE) && (normB < normA1 - TOLERANCE);
	
	return t3;
}

// Handles the case where a vertex would otherwise cut the edge of another triangle
// Trying to decide if we cut triangle A
// Two points of triangle may intersect other
// Returns all intersecting points
static bool CheckIntersect(SimplePolygon const & PolyA, SimplePolygon const & PolyB, std::vector<Eigen::Vector2d> & FinalIntersect) {
	std::Evector<LineSegment> segmentA = PolyA.GetLineSegments();
	std::Evector<Eigen::Vector2d> pointsB = PolyB.GetVertices();

	for (int i=0; i < (int) segmentA.size(); i++) {
		for (int j=0; j < (int) pointsB.size(); j++) {
			if (insideColinear(segmentA[i], pointsB[j]))
				FinalIntersect.push_back(pointsB[j]);
		}
	}
	//if (FinalIntersect.size() > 0)
		//std::cerr << "Returning this number of points: " << FinalIntersect.size() << std::endl;
	return (FinalIntersect.size() > 0);
}


// Returns true if two polygons given share at least one side
static bool CheckAdjacency(SimplePolygon const & PolyA, SimplePolygon const & PolyB) {
	Eigen::Vector2d Intersection;
	bool IsInteriorThis;
	bool IsInteriorOther;

	for (LineSegment & main_line: PolyA.GetLineSegments()) {
		for (LineSegment & other_line : PolyB.GetLineSegments()) {
			if (!PointsAreColinear(main_line.m_endpoint1, main_line.m_endpoint2, other_line.m_endpoint1) ||
			    !PointsAreColinear(main_line.m_endpoint1, main_line.m_endpoint2, other_line.m_endpoint2))
				continue;
			if (main_line.ComputeInteriorIntersection(other_line, Intersection, IsInteriorThis, IsInteriorOther))
				return true;

			if (main_line.m_endpoint1 == other_line.m_endpoint1) {
				if (other_line.ContainsPoint(main_line.m_endpoint2) || main_line.ContainsPoint(other_line.m_endpoint2))
					return true;
			}
			if (main_line.m_endpoint1 == other_line.m_endpoint2) {
				if (other_line.ContainsPoint(main_line.m_endpoint2) || main_line.ContainsPoint(other_line.m_endpoint1))
					return true;
			}

			if (main_line.m_endpoint2 == other_line.m_endpoint1) {
				if (other_line.ContainsPoint(main_line.m_endpoint1) || main_line.ContainsPoint(other_line.m_endpoint2))
					return true;
			}
			if (main_line.m_endpoint2 == other_line.m_endpoint2) {
				if (other_line.ContainsPoint(main_line.m_endpoint1) || main_line.ContainsPoint(other_line.m_endpoint1))
					return true;
			}
		}
	}

	for (LineSegment & other_line: PolyA.GetLineSegments()) {
		for (LineSegment & main_line : PolyB.GetLineSegments()) {
			if(!PointsAreColinear(main_line.m_endpoint1, main_line.m_endpoint2, other_line.m_endpoint1) ||
			   !PointsAreColinear(main_line.m_endpoint1, main_line.m_endpoint2, other_line.m_endpoint2))
				continue;
			if (main_line.ComputeInteriorIntersection(other_line, Intersection, IsInteriorThis, IsInteriorOther))
				return true;

			if (main_line.m_endpoint1 == other_line.m_endpoint1) {
				if(other_line.ContainsPoint(main_line.m_endpoint2) || main_line.ContainsPoint(other_line.m_endpoint2))
					return true;
			}
			if (main_line.m_endpoint1 == other_line.m_endpoint2) {
				if(other_line.ContainsPoint(main_line.m_endpoint2) || main_line.ContainsPoint(other_line.m_endpoint1))
					return true;
			}

			if (main_line.m_endpoint2 == other_line.m_endpoint1) {
				if(other_line.ContainsPoint(main_line.m_endpoint1) || main_line.ContainsPoint(other_line.m_endpoint2))
					return true;
			}
			if (main_line.m_endpoint2 == other_line.m_endpoint2) {
				if(other_line.ContainsPoint(main_line.m_endpoint1) || main_line.ContainsPoint(other_line.m_endpoint1))
					return true;
			}
		}
	}

	return false;
}

//3.1 Helper Function -- Gets the "score" (estimated flight time) of a triangle based on a square.
static double GetTriangleScore(Triangle & triangle, Guidance::MissionParameters const & MissionParams) {
	//Row spacing = 2 * HAG * tan(0.5 * HFOV) * (1 - SidelapFraction) <In Meters>
	double row_spacing = 2 * MissionParams.HAG * tan(0.5 * MissionParams.HFOV) * (1 - MissionParams.SidelapFraction);
	double length = sqrt(2 * GetArea(triangle));
	return length * (length / row_spacing) / MissionParams.TargetSpeed;
}

//3.2 Helper Function -- Recursively bisects triangles until they score less than the targeted flight time.
static void PartitionSurveyRegionRec(Triangle & mainTriangle, std::Evector<Triangle> & allTriangles, Guidance::MissionParameters const & MissionParams) {
	if (GetTriangleScore(mainTriangle, MissionParams) > MissionParams.SubregionTargetFlightTime){
		//Split the triangle into two triangles
		Triangle subTriangleA;
		Triangle subTriangleB;
		BisectTriangle(mainTriangle, subTriangleA, subTriangleB);

		PartitionSurveyRegionRec(subTriangleA, allTriangles, MissionParams);
		PartitionSurveyRegionRec(subTriangleB, allTriangles, MissionParams);
	}
	else
		allTriangles.push_back(mainTriangle);
}

//3.3 Helper Function
static void PartitionTrianglesList(Triangle & mainTriangle, std::Evector<Triangle> & allTriangles, std::vector<Eigen::Vector2d> pointsList) {
	// Base case, just one point
	if (pointsList.size() == 1) {
		Triangle subTriangleA;
		Triangle subTriangleB;
		BisectIntersection(mainTriangle, pointsList[0], subTriangleA, subTriangleB);
		allTriangles.push_back(subTriangleA);
		allTriangles.push_back(subTriangleB);
	}
	// other base case: empty list
	else if (pointsList.empty())
		allTriangles.push_back(mainTriangle);

	else {
		Triangle subTriangleA;
		Triangle subTriangleB;
		std::vector<Eigen::Vector2d> pointsListA;
		std::vector<Eigen::Vector2d> pointsListB;
		BisectIntersection(mainTriangle, pointsList[0], subTriangleA, subTriangleB);

		for (int j = 1; j < (int) pointsList.size(); j++) {
			if (TriangleToSimplyPoly(subTriangleA).ContainsPoint(pointsList[j]))
				pointsListA.push_back(pointsList[j]);
			else
				pointsListB.push_back(pointsList[j]);
		}
		PartitionTrianglesList(subTriangleA, allTriangles,pointsListA);
		PartitionTrianglesList(subTriangleB, allTriangles,pointsListB);
	}
}

//Get the internal angle between two segments that share a point. You input points A, B, and C. The edges are assumed to be AB and BC. The interior
//is assumed to be on the left of these directed edges. The returned angle is in radians. A and B must be distinct. B and C must also be distinct.
//If either of these conditions is violated, we return 0.0. The output will be in the range [0, 2*PI]
static double getInternalAngle(Eigen::Vector2d const & A, Eigen::Vector2d const & B, Eigen::Vector2d const & C) {
	Eigen::Vector2d V1 = B - A;
	Eigen::Vector2d V2 = C - B;
	if (V1.norm() < 1e-6)
		return 0.0;
	if (V2.norm() < 1e-6)
		return 0.0;

	V1.normalize();
	V2.normalize();

	double x1 = V1(0);
	double y1 = V1(1);
	double x2 = V2(0);
	double y2 = V2(1);

	double dot = x1*x2 + y1*y2;
	double det = x1*y2 - y1*x2;
	double theta = atan2(det, dot);
	return (PI - theta);
}

//Try to remove an edge from a graph adjacency map. The edge need not exist, but IndexA and IndexB must be valid node indices.
static void RemoveEdgeFromAdjacencyMap(std::vector<std::vector<int>> & AdjacentNodes, int IndexA, int IndexB) {
	//Remove IndexB from the adjacent nodes vector for IndexA
	auto it = std::find(AdjacentNodes[IndexA].begin(), AdjacentNodes[IndexA].end(), IndexB);
	if (it != AdjacentNodes[IndexA].end())
		AdjacentNodes[IndexA].erase(it);
	//Remove IndexA from the adjacent nodes vector for IndexB
	it = std::find(AdjacentNodes[IndexB].begin(), AdjacentNodes[IndexB].end(), IndexA);
	if (it != AdjacentNodes[IndexB].end())
		AdjacentNodes[IndexB].erase(it);
}

static SimplePolygon TraceLineSegmentsToBuildSimplePloygon(std::Evector<LineSegment> segments) {
	//Build a collection of line segments that form the full polygon and break them at intersections so the resulting segments can only intersect at nodes.
	//segments = BreakAtIntersections(segments);

	//Note that breaking line segments happens exactly (the new interior node is exactly the same in the resulting segments).
	//Also, the segments extracted from the polygon exactly cover the boundary (nodes that are supposed to be the same are exacty the same).
	//The only error introduced is round-off error in computing the placement of interior nodes when segments intersect.

	SimplePolygon simplePoly;

	//Change our representation to a node graph
	std::Evector<Eigen::Vector2d> NodeLocations; NodeLocations.reserve(segments.size());
	std::vector<std::vector<int>> AdjacentNodes; AdjacentNodes.reserve(segments.size());
	for (auto const & segment : segments) {
		int endpoint1Index = -1;
		int endpoint2Index = -1;
		for (size_t n = 0U; n < NodeLocations.size(); n++) {
			if ((segment.m_endpoint1 - NodeLocations[n]).norm() < TOLERANCE)
				endpoint1Index = int(n);
			if ((segment.m_endpoint2 - NodeLocations[n]).norm() < TOLERANCE)
				endpoint2Index = int(n);
			if ((endpoint1Index >= 0) && (endpoint2Index >= 0))
				break;
		}

		if (endpoint1Index < 0) {
			endpoint1Index = (int) NodeLocations.size();
			NodeLocations.push_back(segment.m_endpoint1);
			AdjacentNodes.emplace_back();
		}
		if (endpoint2Index < 0) {
			endpoint2Index = (int) NodeLocations.size();
			NodeLocations.push_back(segment.m_endpoint2);
			AdjacentNodes.emplace_back();
		}

		AdjacentNodes[endpoint1Index].push_back(endpoint2Index);
		AdjacentNodes[endpoint2Index].push_back(endpoint1Index);
	}

	//Find an extremal node to begin traversal with - we choose the left-most node of those with min Y value
	double minY = std::numeric_limits<double>::infinity();
	for (Eigen::Vector2d point : NodeLocations)
		minY = std::min(minY, point(1));
	double minX = std::numeric_limits<double>::infinity();
	int extremalNodeIndex = 0;
	for (int index = 0; index < (int) NodeLocations.size(); index++) {
		Eigen::Vector2d point = NodeLocations[index];
		if (point(1) == minY) {
			if (point(0) < minX) {
				minX = point(0);
				extremalNodeIndex = index;
			}
		}
	}

	//We will populate a new polygonal object and store it as a sequence of node indices... starting with the one we just found.
	std::vector<int> trajectory;
	trajectory.reserve(NodeLocations.size());
	trajectory.push_back(extremalNodeIndex);

	//Choose the first edge to be the one closest to direction (1,0). All edges move up from this node because of how it was chosen
	{
		double maxInternalAngle = -1.0;
		int bextNextNodeIndex = -1;
		for (int nodeIndex : AdjacentNodes[extremalNodeIndex]) {
			Eigen::Vector2d A = NodeLocations[extremalNodeIndex] - Eigen::Vector2d(1.0, 0.0);
			Eigen::Vector2d B = NodeLocations[extremalNodeIndex];
			Eigen::Vector2d C = NodeLocations[nodeIndex];
			double internalAngle = getInternalAngle(A, B, C);
			if ((bextNextNodeIndex < 0) || (internalAngle > maxInternalAngle)) {
				maxInternalAngle = internalAngle;
				bextNextNodeIndex = nodeIndex;
			}
		}
		if (bextNextNodeIndex < 0) {
			std::cerr << "Internal Error in SimplePolygon::Sanitize - Failed to select second vertex.\r\n";
			return simplePoly;
		}
		trajectory.push_back(bextNextNodeIndex);
		RemoveEdgeFromAdjacencyMap(AdjacentNodes, extremalNodeIndex, bextNextNodeIndex);
	}

	//Now traverse edges, one at a time - removing each from the adjacency map once it's traversed. We always select the edge with maximal internal angle
	//with the last edge. This keeps us following the outermost boundary of the polygonal object. We are done when we get back to our first node.
	while (true) {
		int currentNodeIndex  = trajectory[trajectory.size() - 1U];
		int previousNodeIndex = trajectory[trajectory.size() - 2U];

		Eigen::Vector2d A = NodeLocations[previousNodeIndex];
		Eigen::Vector2d B = NodeLocations[currentNodeIndex];

		double maxInternalAngle = -1.0;
		int bextNextNodeIndex = -1;
		for (int nodeIndex : AdjacentNodes[currentNodeIndex]) {
			Eigen::Vector2d C = NodeLocations[nodeIndex];
			double internalAngle = getInternalAngle(A, B, C);
			if (internalAngle > maxInternalAngle) {
				maxInternalAngle = internalAngle;
				bextNextNodeIndex = nodeIndex;
			}
		}
		if (bextNextNodeIndex < 0) {
			std::cerr << "Internal Warning in SimplePolygon::Sanitize - Stuck at node with no more adjacent nodes. Closing from here.\r\n";
			trajectory.push_back(trajectory.front());
			break;
		}
		else {
			trajectory.push_back(bextNextNodeIndex);
			RemoveEdgeFromAdjacencyMap(AdjacentNodes, currentNodeIndex, bextNextNodeIndex);
		}

		//Test regular termination condition
		if (trajectory.front() == trajectory.back())
			break;
	}

	//Complete the object with the new vertices (remember that the first node is repeated at the end of trajectory so it should be removed)
	trajectory.pop_back();
	std::Evector<Eigen::Vector2d> vertices;
	vertices.reserve(trajectory.size());
	for (int index : trajectory)
		vertices.push_back(NodeLocations[index]);
	return SimplePolygon(vertices);
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
	//MissionParams    - Input  - Parameters specifying speed and row spacing (see definitions in struct declaration)
	void PartitionSurveyRegion_TriangleFusion(PolygonCollection const & Region, std::Evector<PolygonCollection> & Partition, MissionParameters const & MissionParams) {
        // *********************** MESHING SECTION **************************************************
        //Slice the polygon into triangles.
		std::Evector<Triangle> allTrianglesTriangulate;
		std::Evector<Triangle> allTriangles;
		TriangleAdjacencyMap map;
		std::vector<std::vector<Eigen::Vector2d>> intersectionsList;
		std::vector<Eigen::Vector2d> Intersection;
		for(Polygon shape: Region.m_components) {
			std::Evector<Triangle> subTriangles;
			shape.Triangulate(subTriangles);
			for (Triangle & tri: subTriangles) {
				allTrianglesTriangulate.push_back(tri);
			}
		}

		//Recursively slice triangles until they are smaller than the Target Flight Time.
		for (Triangle & triangle: allTrianglesTriangulate)
			PartitionSurveyRegionRec(triangle, allTriangles, MissionParams);

		allTrianglesTriangulate.clear();

		intersectionsList.resize(allTriangles.size());

		// This loop searches all triangles for vertices that cut an edge
		// A triangle may cut an edge 0, 1, or 2 times
		// All intersections are assembled into a list per triangle. We are trying to avoid duplicates
		for (int i = 0; i < (int) allTriangles.size(); i++) {
			for (int j = 0; j < (int) allTriangles.size(); j++) {
				Intersection.clear();
				if ((i != j) && (CheckIntersect(TriangleToSimplyPoly(allTriangles[i]), TriangleToSimplyPoly(allTriangles[j]), Intersection))) {
                    //std::cout<< Intersection.size() <<" intersections found!" <<std::endl;
					for (auto point: Intersection){
						bool isUnique = true;
                        // Checking list for existence of same point. If exists, not unique and do not add to list
						for (int k = 0; k < (int) intersectionsList[i].size(); k++){
							if ((intersectionsList[i][k] - point).norm() < TOLERANCE){
								isUnique = false;
							}
						}
						if (isUnique){
							//std::cout<< "Adding point! " << std::endl;
							intersectionsList[i].push_back(point);
						}
					}
					std::cout<< "Triangle " << std::to_string(i) << " has intersection with triangle " << j  << std::endl;
				}
			}
		}

		for (int i = 0; i < (int) intersectionsList.size(); i++) {
			if (!intersectionsList[i].empty()) {
				std::cout<< "Triangle " << std::to_string(i) << " has intersections " << intersectionsList[i].size() << std::endl;
			}
		}

		Triangle subTriangleA;
		Triangle subTriangleB;
		allTrianglesTriangulate.clear();
		for (int i = 0; i < (int) allTriangles.size(); i++) {
			if (intersectionsList[i].empty())
				allTrianglesTriangulate.push_back(allTriangles[i]);
			else {

                /*
                 *                 for (auto point: intersectionsList[i]){
                                    Triangle subTriangleA;
                                    Triangle subTriangleB;
                                    BisectIntersection(allTriangles[i], point, subTriangleA, subTriangleB);
                                    allTrianglesTriangulate.push_back(subTriangleA);
                                    allTrianglesTriangulate.push_back(subTriangleB);
                                }
                 */
				PartitionTrianglesList(allTriangles[i], allTrianglesTriangulate, intersectionsList[i]);
			}
		}

		std::vector<std::string> triangleLabels;

		for (int i = 0; i < (int) allTrianglesTriangulate.size(); i++)
			triangleLabels.push_back(std::to_string(i));
		//MapWidget::Instance().m_guidanceOverlay.ClearTriangleLabels();
		//MapWidget::Instance().m_guidanceOverlay.ClearTriangles();
		//MapWidget::Instance().m_guidanceOverlay.SetTriangles(allTrianglesTriangulate);
		//MapWidget::Instance().m_guidanceOverlay.SetTriangleLabels(triangleLabels);

		allTriangles = allTrianglesTriangulate; // ECHAI: Not great but I'm feeling lazy
		//return;
		//************************* END OF MESHING ****************************************
		// Up to this point, we are guaranteed that no vertex cuts another edge!

		//************************ ADJACENCY MAP CONSTRUCTION********************************
		//Add each triangle and its edges to the map.

		for (int i = 0; i < (int) allTriangles.size(); i++) {
			TriangleAdjacencyMap::Triangle tri;
			tri.id = i;
			tri.score = GetTriangleScore(allTriangles[i], MissionParams);
			map.nodes.push_back(tri);
			map.edges.push_back({});
			for (int j = 0; j < (int) allTriangles.size(); j++) {
				if ((i != j) && (CheckAdjacency(TriangleToSimplyPoly(allTriangles[i]), TriangleToSimplyPoly(allTriangles[j]))))
					map.edges[i].push_back(j);
			}
		}
		// ****************** END OF ADJACENCY MAP CONSTRUCTION ************************************

		// ****************** GROUP/PARITION CONSTRUCTION ********************************************
		//Combine triangles greedily until adding another adjacent triangle would go above the Target Flight Time.
		std::vector<int> groupScoreList;
		std::vector<int> assignedGroups(allTriangles.size(), -1);
		for (auto node: map.nodes) {
			std::vector<int> currentCandidateEdges;
			//If the node is already in a group, ignore it
			if (assignedGroups[node.id] != -1)
				continue;

			//Otherwise, create a group for the solo node.
			map.groups.push_back({node.id});
			groupScoreList.push_back(node.score);
			assignedGroups[node.id] = map.groups.size()-1;

			//Add in edges from the starting node of the group.
			for (auto item: map.edges[node.id])
				currentCandidateEdges.push_back(item);

			//While there are edges attached to this group, try to add them in.
			while (currentCandidateEdges.size() > 0) {
				int n = currentCandidateEdges[0];
				if (map.nodes[n].score + groupScoreList.back() < MissionParams.SubregionTargetFlightTime && assignedGroups[n] == -1) {
					map.groups.back().push_back(map.nodes[n].id);
					groupScoreList.back() += map.nodes[n].score;
					assignedGroups[n] = map.groups.size()-1;
					//Add edges to currentCandidateEdges
					for (auto item: map.edges[n]) {
						currentCandidateEdges.push_back(item);
					}
				}
				currentCandidateEdges.erase(currentCandidateEdges.begin());
			}

			//Now, create the new polygon & add it to its own private collection in the Evector of PolygonCollections.
			PolygonCollection collection;
			Polygon new_polygon;
			std::Evector<LineSegment> list_of_segments;
			for (auto node: map.groups.back()) {
				for (LineSegment line : TriangleToSimplyPoly(allTriangles[node]).GetLineSegments())
					list_of_segments.push_back(line);
			}
			new_polygon.m_boundary = TraceLineSegmentsToBuildSimplePloygon(list_of_segments);
			//new_polygon.m_boundary.CustomSanitize(list_of_segments);
			collection.m_components.push_back(new_polygon);
			Partition.push_back(collection);
		}

		//Set labels for the grouped shapes.
		std::vector<std::string> groupLabels;
		std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
		for (int n = 0; n < (int) map.groups.size(); n++)
			groupLabels.push_back(alphabet.substr(n, 1));

		//Display both the triangles and the grouped polygons on Recon.
		//MapWidget::Instance().m_guidanceOverlay.ClearPartitionLabels();
		//MapWidget::Instance().m_guidanceOverlay.ClearSurveyRegionPartition();
		MapWidget::Instance().m_guidanceOverlay.SetSurveyRegionPartition(Partition);
		MapWidget::Instance().m_guidanceOverlay.SetPartitionLabels(groupLabels);

		//Set labels for the individual triangles.
		//std::vector<std::string> triangleLabels;
		triangleLabels.clear();
		for (int i = 0; i < (int) map.nodes.size(); i++) {
			std::string fullLabel = alphabet[assignedGroups[i]] + std::to_string(i) + ":";
			for (int j = 0; j < (int) map.edges[i].size(); j++) {
				if (j != 0)
					fullLabel.append(",");
				fullLabel.append(std::to_string(map.edges[i][j]));
			}
			triangleLabels.push_back(fullLabel);
		}

		//MapWidget::Instance().m_guidanceOverlay.ClearTriangleLabels();
		//MapWidget::Instance().m_guidanceOverlay.ClearTriangles();
		MapWidget::Instance().m_guidanceOverlay.SetTriangles(allTriangles);
		MapWidget::Instance().m_guidanceOverlay.SetTriangleLabels(triangleLabels);

		//(NON-IMPACTING -- FOR DEVELOPER USE ONLY) Display polygon information on the terminal.
		std::cout<<"Target Score: " << MissionParams.SubregionTargetFlightTime << std::endl;
		std::cout<<"There are " << map.nodes.size() << " nodes in the graph. This should equal the number of triangles." << std::endl;
		std::cout<<"There are " << map.groups.size() << " groups in the graph." << std::endl;
		//Information on individual triangles.
		for (int i = 0; i < (int) map.nodes.size(); i++) {
			float value = (int)(map.nodes[i].score * 100 + .5);
			std::cout << "Edges of " << i <<" (Score " << (float)value / 100 << "; Group [" << alphabet[assignedGroups[i]] <<"]) : ";
			for (int j = 0; j < (int) map.edges[i].size(); j++) {
				std::cout << map.edges[i][j] << ", ";
			}
			std::cout << std::endl;
		}
		//Information on grouped triangles.
		for (int n = 0; n < (int) map.groups.size(); n++) {
			std::cout << "Group "<< alphabet.substr(n, 1) << " :";
			for (auto item: map.groups[n]){
				std::cout << " " << item;
			}
			std::cout << "." << std::endl;
		}
	}
}