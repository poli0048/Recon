//This source file implements the flight planning capabilities of the guidance module. Specifically, it provides the public function
//PlanMission() that takes a given region and plans a waypoint mission to cover the region with a survey flight that respects
//a given set of imaging constraints. The initial code for this task was developed by Elaina Chai at Stanford University. It has
//been refactored and farther developed by Bryan Poling at Sentek Systems, LLC.
//Authors: Bryan Poling, Elaina Chai
//Copyright (c) 2022 Sentek Systems, LLC. All rights reserved.

//System Includes

//External Includes
//#include "../../eigen/Eigen/LU"
//#include "../../eigen/Eigen/QR"
//#include "../../eigen/Eigen/Geometry"

//Project Includes
#include "Guidance.hpp"
//#include "../../UI/VehicleControlWidget.hpp"
#include "../../Utilities.hpp"
//#include "../../Maps/MapUtils.hpp"

#define PI 3.14159265358979323846

// *********************************************************************************************************************************
// *************************************************   Local Function Definitions   ************************************************
// *********************************************************************************************************************************

//Get a lower bound for the dot product of any point in a region with VHat, based on the regions AABB
static double FindLowerBoundProjectionOntoUnitVec(Eigen::Vector4d const & AABB, Eigen::Vector2d const & VHat) {
	Eigen::Vector2d p1(AABB(0), AABB(2));
	Eigen::Vector2d p2(AABB(0), AABB(3));
	Eigen::Vector2d p3(AABB(1), AABB(2));
	Eigen::Vector2d p4(AABB(1), AABB(3));

	double proj1 = p1.dot(VHat);
	double proj2 = p2.dot(VHat);
	double proj3 = p3.dot(VHat);
	double proj4 = p4.dot(VHat);

	return std::min(std::min(proj1, proj2), std::min(proj3, proj4));
}

//Get an upper bound for the dot product of any point in a region with VHat, based on the regions AABB
static double FindUpperBoundProjectionOntoUnitVec(Eigen::Vector4d const & AABB, Eigen::Vector2d const & VHat) {
	Eigen::Vector2d p1(AABB(0), AABB(2));
	Eigen::Vector2d p2(AABB(0), AABB(3));
	Eigen::Vector2d p3(AABB(1), AABB(2));
	Eigen::Vector2d p4(AABB(1), AABB(3));

	double proj1 = p1.dot(VHat);
	double proj2 = p2.dot(VHat);
	double proj3 = p3.dot(VHat);
	double proj4 = p4.dot(VHat);

	return std::max(std::max(proj1, proj2), std::max(proj3, proj4));
}

//Find the index of the segment that contains the closest endpoint to the point Pt. Return the index (or -1 if Segments is empty)
//and populate Endpoint with 1 or 2 to indicate which endpoint is the one closest to the point. If SegmentIndicesToIgnore is not
//null, segments with indices in the provided set will not be considered.
static int FindSegmentWithEndpointClosestToPoint(std::Evector<LineSegment> const & Segments, Eigen::Vector2d const & Pt, int & Endpoint,
                                                 std::unordered_set<int> * SegmentIndicesToIgnore) {
	int bestSegIndex = -1;
	double shortestDist = std::nan("");
	for (int n = 0U; n < (int) Segments.size(); n++) {
		if ((SegmentIndicesToIgnore != nullptr) && (SegmentIndicesToIgnore->count(n) > 0U))
			continue;
		double dist1 = (Segments[n].m_endpoint1 - Pt).norm();
		double dist2 = (Segments[n].m_endpoint2 - Pt).norm();
		if ((std::isnan(shortestDist)) || (dist1 < shortestDist)) {
			shortestDist = dist1;
			bestSegIndex = n;
			Endpoint = 1;
		}
		if ((std::isnan(shortestDist)) || (dist2 < shortestDist)) {
			shortestDist = dist2;
			bestSegIndex = n;
			Endpoint = 2;
		}
	}
	return bestSegIndex;
}

void PlanMission_Elaina(PolygonCollection const & Region, DroneInterface::WaypointMission & Mission, Guidance::MissionParameters const & MissionParams) {
	auto startTime = std::chrono::steady_clock::now();

	Mission.Waypoints.clear();
	Mission.LandAtLastWaypoint = false;
	Mission.CurvedTrajectory = true;
	std::Evector<Eigen::Vector2d> vertices_NM;
	std::Evector<PolygonCollection> Partition;

	std::Evector<std::tuple<LineSegment, float, Eigen::Vector3f>> lineSegments;
	std::Evector<std::tuple<LineSegment, float, Eigen::Vector3f>> hatchLinesVisualize;
	std::Evector<std::tuple<Eigen::Vector2d, float, Eigen::Vector3f>> circles;

	float lineThickness = 2.0f;
	float circleRadius  = 6.5f;
	Eigen::Vector3f lineColor  (0.0f, 0.8f, 0.0f);
	Eigen::Vector3f circleColor(0.8f, 0.0f, 0.0f);

	for (auto subregion: Region.m_components) {
		std::Evector<LineSegment> edges = subregion.m_boundary.GetLineSegments();
		SimplePolygon tempPolygon;

		// These variables are for the calculating the hatchlines
		std::vector<LineSegment> hatchLines;
		std::vector<LineSegment> hatchLines_NM;
		LineSegment longest_edge;
		double longest_side_length = 0.0;


		// Finding the longest edge of the simple polygon
		for (auto edge: edges){
			if (edge.GetLength() > longest_side_length){
				longest_side_length = edge.GetLength();
				longest_edge = edge;
			}
		}
		lineSegments.push_back(std::make_tuple(longest_edge, lineThickness, lineColor));
		// Step 1: Rotation
		// 1a: translation: I want to be able to view the rotation, which means this is a rotation around a point
		// Point (x1,y1) to be rotatated around point (x2,y2)
		// (xt, yt) = (x1,y1) - (x2,y2)
		// angle to x-axis given by arctan(x/y)
		// Point of rotation will be edge vertex closest to zero

		Eigen::Vector2d rotationPoint; // This will be the point of rotation.
		Eigen::Vector2d pointTranslated; // This will be the effective point to rotate

		if (longest_edge.m_endpoint1.norm() < longest_edge.m_endpoint2.norm()){
			rotationPoint = longest_edge.m_endpoint1;
			pointTranslated = longest_edge.m_endpoint2 - longest_edge.m_endpoint1;
		}
		else{
			rotationPoint = longest_edge.m_endpoint2;
			pointTranslated = longest_edge.m_endpoint1 - longest_edge.m_endpoint2;
		}
		double theta = atan2(pointTranslated(1), pointTranslated(0));

		// Calculating rotation matrix
		Eigen::Matrix2d rot2D;
		rot2D << cos(-theta), -sin(-theta),
		         sin(-theta),  cos(-theta);

		// Create rotated polygon
		vertices_NM.clear();
		for (auto vertex: subregion.m_boundary.GetVertices())
			vertices_NM.push_back((rot2D *(vertex-rotationPoint))+rotationPoint);

		tempPolygon.SetBoundary(vertices_NM); // Create our rotated polygon

		//********************** FOR PLOTTING PURPOSES *********************************

		//For visualization purposes, we are plotting original and rotated polygon
		Partition.emplace_back(); //Create a new element in the partition
		Partition.back().m_components.emplace_back(); //Add a component to the new element
		Partition.back().m_components.back().m_boundary.SetBoundary(subregion.m_boundary.GetVertices());

		Partition.emplace_back(); //Create a new element in the partition
		Partition.back().m_components.emplace_back(); //Add a component to the new element
		Partition.back().m_components.back().m_boundary.SetBoundary(vertices_NM);
		// ****************** END OF FOR PLOTTING PURPOSES ********************
		//****************** END OF POLYGON ROTATION **************************

		//****************** POLYGON FILL *************************************
		// Create bounding box containing our polygon
		Eigen::Vector4d bounding_box = tempPolygon.GetAABB(); //vector (XMin, XMax, YMin, YMax)
		Eigen::Vector2d MinMin_NM; // Store (XMin, YMin)
		Eigen::Vector2d MaxMax_NM; // Store (XMax, YMax)
		Eigen::Vector2d MinMin_ECEF; // Store (XMin, YMin), for debugging purposes
		Eigen::Vector2d MaxMax_ECEF; // Store (XMax, YMax), for debugging purposes

		MinMin_NM << bounding_box(0), bounding_box(2);
		MaxMax_NM << bounding_box(1), bounding_box(3);
		vertices_NM.clear();
		vertices_NM.emplace_back(MinMin_NM);
		vertices_NM.emplace_back(MaxMax_NM);

		// ****************** HATCH LINE GENERATION ****************************************
		// Fill the polygon with hatch lines
		// 1) Figure out spacing.
		//      1a) Convert bounding box to meters
		//      1b) Get straight line trajectories with overlap using Imaging requirements

		// 1a) Convert bounding box to meters

		// 1b) Get straight line trajectories based on imaging requirements
		// Start first hatch line half the row spacing from top of box, which is yMax
		// Build horizontal lines until y-value is less than yMin
		// All lines go from Xmin to Xmax
		double rowSpacing = 2 * MissionParams.HAG * tan(0.5 * MissionParams.HFOV) * (1 - MissionParams.SidelapFraction);
		double rowSpacing_NM = MetersToNMUnits(rowSpacing, MaxMax_NM(1));

		// This is for debugging. Useful to understand distances in meters
		double numHatchLines = ceil((MaxMax_ECEF(1) - MinMin_ECEF(1)- (rowSpacing/2))/rowSpacing);
		double y_init = MaxMax_ECEF(1)-(rowSpacing/2.0);
		numHatchLines = ceil((MaxMax_NM(1) - MinMin_NM(1)- (rowSpacing_NM/2))/rowSpacing_NM);
		y_init = MaxMax_NM(1)-(rowSpacing_NM/2.0);


		for (int i = 0; i<numHatchLines; i++){
			//InputSegments.emplace_back(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0));
			hatchLines.emplace_back(Eigen::Vector2d(MinMin_NM(0), y_init-(i*rowSpacing_NM)),
			                        Eigen::Vector2d(MaxMax_NM(0), y_init-(i*rowSpacing_NM)));
		}

		//********************* END OF HATCHLINE GENERATION ********************************************

		// ******************* GENERATE FRAGMENTED LINES FOR TRAJECTORIES ******************************
		// Now we break the hatchlines
		// For each hatchline
		// 2a) Break line where it intersects with polygon edges
		// 2b) Keep segment that is internal to polygon. Note that a line segment that might break in multiple places
		std::Evector<LineSegment> FragmentedHatchLines;

		for (auto currentHatchLine: hatchLines){
			std::vector<Eigen::Vector2d> allIntersections;
			bool IsInteriorA, IsInteriorB;

			// For each polygon edge, collect all intersections
			std::vector<double> m1_norm;
			for (auto edge: tempPolygon.GetLineSegments() ){
				Eigen::Vector2d Intersection;
				if (edge.ComputeInteriorIntersection(currentHatchLine, Intersection, IsInteriorA, IsInteriorB)){
					allIntersections.push_back(Intersection);
					m1_norm.push_back((Intersection-currentHatchLine.m_endpoint1).norm());
				}
			}
			// Sort intersections according to distance to one endpoint of hatchline
			std::vector<std::pair<double,int> >V;
			for (int i = 0; i < (int) m1_norm.size(); i++) {
				std::pair<double,int>P=std::make_pair(m1_norm[i],i);
				V.push_back(P);
			}
			sort(V.begin(),V.end());
			// V is now the intersections sorted by their norm distance to one end-point of hatchline

			// Now construct line segments, keeping only those inside the polygon
			// Guaranteed that line segment in each loop is possible shortest
			// Therefore it is either completely inside or outside polygon
			// Therefore the mid-point must be completely inside or outside polygon
			for (int i = 1; i < (int) V.size(); i++) {
				Eigen::Vector2d midPoint = (allIntersections[V[i-1].second]+allIntersections[(V[i].second)])/2;
				if(tempPolygon.ContainsPoint(midPoint)){
					FragmentedHatchLines.push_back(LineSegment(allIntersections[V[i-1].second],allIntersections[(V[i].second)]));
				}
			}
		}

		// ***************** FRAGMENTED LINES GENERATION COMPLETED *****************************

		// For each line segment
		//      1a) Populate waypoint vector with end points
		//              Shift end_points inwards by 0.5*rowSpacing_NM
		//              If line length is larger than rowspacing_NM, then the shift will result in a single point
		//                  Populate waypoint vector with just that single midpoint
		//              Otherwise, populate with m1 and m2
		//      1b) Create temp sorted list of norm and index of all remaining endpoints, see example code above
		//              The closest endpoint will be at index 0
		//              The associated line segment will be the next line
		//      1c) Undo the rotation of the points.
		//            NOTE WELL!!!! This function rotates around a point that is NOT the origin
		//            Therefore, the rotation operation is technically a translation, then rotation, then translation
		//

		//1a: Populate waypoint vector with end points
		for (auto & line: FragmentedHatchLines) {
			if (line.GetLength() < rowSpacing_NM) {
				LineSegment tempLine;
				tempLine.m_endpoint1 = (line.m_endpoint1 + line.m_endpoint2) / 2;
				tempLine.m_endpoint2 = tempLine.m_endpoint1;
				line = tempLine;
			}
			else {
				if (line.m_endpoint1[0] > line.m_endpoint2[0]) {
					line.m_endpoint1[0] -= 0.5*rowSpacing_NM;
					line.m_endpoint2[0] += 0.5*rowSpacing_NM;
				} else {
					line.m_endpoint1[0] += 0.5*rowSpacing_NM;
					line.m_endpoint2[0] -= 0.5*rowSpacing_NM;
				}
			}
		}

		//If there are no hatch lines, there is nothing left to do. Also, the code below assumes there is at least 1.
		if (FragmentedHatchLines.empty())
			return;


		//1b: Create temp sorted list of norm and index of all remaining endpoints, see example code above [WARNING: UNSORTED]
		std::vector<Eigen::Vector2d> orderedPoints;
		std::vector<bool> hasLineBeenUsed (FragmentedHatchLines.size(), false);
		bool isEndpointOne = true;
		int chosenLineIndex = 0;
		hasLineBeenUsed[0] = true;
		orderedPoints.push_back(FragmentedHatchLines[0].m_endpoint2);
		orderedPoints.push_back(FragmentedHatchLines[0].m_endpoint1);
		while (std::find(hasLineBeenUsed.begin(), hasLineBeenUsed.end(), false) != hasLineBeenUsed.end()) {
			Eigen::Vector2d currentEndpoint;
			Eigen::Vector2d chosenPoint;
			if(isEndpointOne) currentEndpoint = FragmentedHatchLines[chosenLineIndex].m_endpoint1;
			else currentEndpoint = FragmentedHatchLines[chosenLineIndex].m_endpoint2;
			chosenPoint = currentEndpoint;
			//Find the closest unused point to this one
			for (int x = 0; x < (int) hasLineBeenUsed.size(); x++) {
				if(hasLineBeenUsed[x])
					continue;
				if (chosenPoint == currentEndpoint){
					chosenLineIndex = x;
					isEndpointOne = false;
					chosenPoint = FragmentedHatchLines[x].m_endpoint1;
				}
				if ((chosenPoint - currentEndpoint).norm() > (FragmentedHatchLines[x].m_endpoint1 - currentEndpoint).norm()) {
					chosenLineIndex = x;
					isEndpointOne = false;
					chosenPoint = FragmentedHatchLines[x].m_endpoint1;
				}
				if ((chosenPoint - currentEndpoint).norm() > (FragmentedHatchLines[x].m_endpoint2 - currentEndpoint).norm()) {
					chosenLineIndex = x;
					isEndpointOne = true;
					chosenPoint = FragmentedHatchLines[x].m_endpoint2;
				}
			}
			hasLineBeenUsed[chosenLineIndex] = true;
			if (isEndpointOne) {
				orderedPoints.push_back(FragmentedHatchLines[chosenLineIndex].m_endpoint2);
				orderedPoints.push_back(FragmentedHatchLines[chosenLineIndex].m_endpoint1);
			}
			else {
				orderedPoints.push_back(FragmentedHatchLines[chosenLineIndex].m_endpoint1);
				orderedPoints.push_back(FragmentedHatchLines[chosenLineIndex].m_endpoint2);
			}
		}

		//1c: Undo Rotation of the points.
		rot2D << cos(theta), -sin(theta),
		sin(theta),  cos(theta);
		for (auto & line: FragmentedHatchLines) {
			line.m_endpoint1 = (rot2D *(line.m_endpoint1-rotationPoint))+rotationPoint;
			line.m_endpoint2 = (rot2D *(line.m_endpoint2-rotationPoint))+rotationPoint;
		}
		for (auto & point: orderedPoints) {
			point = (rot2D *(point-rotationPoint))+rotationPoint;
			Eigen::Vector2d LatLonPoint;
			LatLonPoint = NMToLatLon(point);
			DroneInterface::Waypoint newWayPoint;
			newWayPoint.Latitude = LatLonPoint[0];
			newWayPoint.Longitude = LatLonPoint[1];
			newWayPoint.RelAltitude = MissionParams.HAG;
			newWayPoint.Speed = MissionParams.TargetSpeed;
			newWayPoint.LoiterTime   = std::nanf("");
			newWayPoint.GimbalPitch   = std::nanf("");
			newWayPoint.CornerRadius = 5.0f;
			Mission.Waypoints.push_back(newWayPoint);
		}
	}
	//MapWidget::Instance().m_guidanceOverlay.SetSurveyRegionPartition(Partition);

	double runtime_ms = SecondsElapsed(startTime)*1000.0;
	std::cerr << "Runtime: " << runtime_ms << " ms.\r\n";
}


// *********************************************************************************************************************************
// ************************************************   Public Function Definitions   ************************************************
// *********************************************************************************************************************************
namespace Guidance {
	//4 - Take a region or sub-region and plan a trajectory to cover it at a given height that meets the specified imaging requirements. In this case we specify
	//    the imaging requirements using a maximum speed and sidelap fraction.
	//Arguments:
	//Region        - Input  - The input survey region or sub-region to cover (polygon collection in NM coords)
	//Mission       - Output - The planned mission that covers the input region
	//MissionParams - Input  - Parameters specifying speed and row spacing (see definitions in struct declaration)
	void PlanMission(PolygonCollection const & Region, DroneInterface::WaypointMission & Mission, MissionParameters const & MissionParams) {
		//PlanMission_Elaina(Region, Mission, MissionParams);
		//return;

		//Compute row spacing in meters
		auto startTime = std::chrono::steady_clock::now();
		double RowSpacing_m = 2.0 * MissionParams.HAG * std::tan(0.5 * MissionParams.HFOV) * (1.0 - MissionParams.SidelapFraction);

		//Lay down hatch lines separately for each disjoint component of the region.
		std::Evector<LineSegment> hatchLines;
		Eigen::Vector4d collectionAABB(std::nan(""), std::nan(""), std::nan(""), std::nan(""));
		for (Polygon const & comp : Region.m_components) {
			//Get the AABB and minor axis for the polygon
			Eigen::Vector4d AABB   = comp.GetAABB();
			Eigen::Vector2d VMinor = comp.FindShortestAxis();

			//If the polygon is empty, skip it
			if (std::isnan(AABB(0)))
				continue;

			//Update AABB of collection
			if (std::isnan(collectionAABB(0)))
				collectionAABB = AABB;
			collectionAABB(0) = std::min(collectionAABB(0), AABB(0));
			collectionAABB(1) = std::max(collectionAABB(1), AABB(1));
			collectionAABB(2) = std::min(collectionAABB(2), AABB(2));
			collectionAABB(3) = std::max(collectionAABB(3), AABB(3));

			//Convert row spacing to NM units based on center point of AABB
			double RowSpacing_NMUnits = MetersToNMUnits(RowSpacing_m, 0.5*AABB(2) + 0.5*AABB(3));

			//Lay down hatch lines orthogonal to the shortest axis to try and minimize the number of turns
			double minProj = FindLowerBoundProjectionOntoUnitVec(AABB, VMinor);
			double maxProj = FindUpperBoundProjectionOntoUnitVec(AABB, VMinor);
			for (double proj = minProj; proj <= maxProj; proj += RowSpacing_NMUnits) {
				//Lay down the hatch line X dot VMinor = proj
				std::Evector<LineSegment> segments = comp.ClipLine(VMinor, proj);
				hatchLines.insert(hatchLines.end(), segments.begin(), segments.end());
			}
		}

		//Now we need to traverse the hatch lines in a reasonable order. Start by finding the segment with endpoint
		//closest to the lower-left corner of the AABB (lowest)
		Eigen::Vector2d startRefPt(collectionAABB(0), collectionAABB(2));
		int startEndpoint = 0;
		int startSegIndex = FindSegmentWithEndpointClosestToPoint(hatchLines, startRefPt, startEndpoint, nullptr);

		std::Evector<Eigen::Vector2d> waypoints_NM;
		std::unordered_set<int> indicesOfUsedHatchLines;
		waypoints_NM.reserve(std::max(size_t(2)*hatchLines.size(), size_t(10)));
		if (startSegIndex >= 0) {
			if (startEndpoint == 1) {
				waypoints_NM.push_back(hatchLines[startSegIndex].m_endpoint1);
				waypoints_NM.push_back(hatchLines[startSegIndex].m_endpoint2);
			}
			else {
				waypoints_NM.push_back(hatchLines[startSegIndex].m_endpoint2);
				waypoints_NM.push_back(hatchLines[startSegIndex].m_endpoint1);
			}
			indicesOfUsedHatchLines.insert(startSegIndex);

			//We have seeded the mission with the first hatch line. Now greedily traverse remaining hatch lines
			while (indicesOfUsedHatchLines.size() < hatchLines.size()) {
				//Untraversed hatch lines remain
				int nextEndpoint = 0;
				int nextSegIndex = FindSegmentWithEndpointClosestToPoint(hatchLines, waypoints_NM.back(), nextEndpoint, &indicesOfUsedHatchLines);
				if (nextSegIndex < 0) {
					std::cerr << "Internal Error in PlanMission(): Hatch lines remain but we failed to select one. \r\n";
					break;
				}

				if (nextEndpoint == 1) {
					waypoints_NM.push_back(hatchLines[nextSegIndex].m_endpoint1);
					waypoints_NM.push_back(hatchLines[nextSegIndex].m_endpoint2);
				}
				else {
					waypoints_NM.push_back(hatchLines[nextSegIndex].m_endpoint2);
					waypoints_NM.push_back(hatchLines[nextSegIndex].m_endpoint1);
				}
				indicesOfUsedHatchLines.insert(nextSegIndex);
			}
		}

		//Build WaypointMission object from the collection of waypoints
		Mission.Waypoints.clear();
		Mission.Waypoints.reserve(waypoints_NM.size());
		Mission.LandAtLastWaypoint = false;
		Mission.CurvedTrajectory = true;
		for (Eigen::Vector2d const & waypoint_NM : waypoints_NM) {
			Eigen::Vector2d waypoint_LatLon = NMToLatLon(waypoint_NM);
			Mission.Waypoints.emplace_back();
			Mission.Waypoints.back().Latitude     = waypoint_LatLon(0);
			Mission.Waypoints.back().Longitude    = waypoint_LatLon(1);
			Mission.Waypoints.back().RelAltitude  = MissionParams.HAG;
			Mission.Waypoints.back().CornerRadius = 5.0f;
			Mission.Waypoints.back().Speed        = MissionParams.TargetSpeed;
			Mission.Waypoints.back().LoiterTime   = std::nanf("");
			Mission.Waypoints.back().GimbalPitch  = std::nanf("");
		}
		
		double runtime_ms = SecondsElapsed(startTime)*1000.0;
		std::cerr << "Runtime: " << runtime_ms << " ms.\r\n";
	}
}