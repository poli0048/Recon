//This module provides geometry utilities needed for working with Polygons
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.

//System Includes
#include <tuple>
#include <limits>

//External Includes
#include "HandyImGuiInclude.hpp"
#include "../../eigen/Eigen/LU"
#include "../../eigen/Eigen/Geometry"
//#include <opencv2/opencv.hpp>  //Only needed for debug and visualization
//#include <opencv2/core.hpp>    //Only needed for debug and visualization
//#include <opencv2/imgproc.hpp> //Only needed for debug and visualization
//#include <opencv2/highgui.hpp> //Only needed for debug and visualization

//Project Includes
#include "Polygon.hpp"
#include "Earcut.hpp"
#include "Maps/MapUtils.hpp"

#define PI 3.14159265358979
#define TOLERANCE 1e-10

// Local function declarations (static linkage)
static double getInternalAngle(Eigen::Vector2d const & A, Eigen::Vector2d const & B, Eigen::Vector2d const & C);
static void RemoveEdgeFromAdjacencyMap(std::vector<std::vector<int>> & AdjacentNodes, int IndexA, int IndexB);
static bool PointsAreColinear(Eigen::Vector2d const & p1, Eigen::Vector2d const & p2, Eigen::Vector2d const & p3);
static int PointIsInsidePolygonHelper_wn_PnPoly(Eigen::Vector2d const & P, std::Evector<Eigen::Vector2d> const & V);
static double PointIsInsidePolygonHelper_isLeft(Eigen::Vector2d const & P0, Eigen::Vector2d const & P1, Eigen::Vector2d const & P2);


// ************************************************************************************************************************************************
// ***********************************************************   LineSegment Definitions   ********************************************************
// ************************************************************************************************************************************************

//Test to see if the line segment is degenerate (endpoints are nearly equal)
bool LineSegment::IsDegenerate(void) const {
    return ((m_endpoint2 - m_endpoint1).norm() < TOLERANCE);
}

//Test to see if a point is approximately on the line segment
bool LineSegment::ContainsPoint(Eigen::Vector2d const & Point) const {
    Eigen::Vector2d projection = ProjectPoint(Point);
    return (Point - projection).norm() < TOLERANCE;
}

//Get segment length
double LineSegment::GetLength(void) const {
	return (m_endpoint2 - m_endpoint1).norm();
}

//Get the point on the line segment closest to the provided point
Eigen::Vector2d LineSegment::ProjectPoint(Eigen::Vector2d const & Point) const {
    //The closest point will be the orthogonal projection of the given point onto the line, if that projection is contained in the segment.
    //Otherwise, the closest point will be one of the endpoints.
    if (IsDegenerate())
        return m_endpoint1;

    //This line segment is not degenerate
    Eigen::Vector2d v1 = m_endpoint2 - m_endpoint1;
    v1.normalize(); //Unit vector pointing along line (1 to 2)
    double projectionOnLine = v1.dot(Point - m_endpoint1);
    if ((projectionOnLine >= 0.0) && (projectionOnLine <= (m_endpoint2 - m_endpoint1).norm()))
        return m_endpoint1 + projectionOnLine * v1;
    else {
        //The projection onto the line between endpoints 1 and 2 is not contained in this segment. Check the endpoints.
        if ((Point - m_endpoint1).norm() < (Point - m_endpoint2).norm())
            return m_endpoint1;
        else
            return m_endpoint2;
    }
}

//Returns the distance to the nearest end of the line segment
double LineSegment::GetDistanceToNearestEndpoint(Eigen::Vector2d const & Point) const {
    return std::min((Point - m_endpoint1).norm(), (Point - m_endpoint2).norm());
}

//Compute the intersection of two line segments. Returns true if they intersect and false otherwise.
//If this function returns false, the value of Intersection could be anything - don't use it.
bool LineSegment::ComputeIntersection(LineSegment const & Other, Eigen::Vector2d & Intersection) const {
    //If we can tell quickly that an intersection is impossible based on segment lengths and separation,
    //then return false and skip the work of solving a linear system.
    double totalLength = this->GetLength() + Other.GetLength();
    Eigen::Vector2d thisCenter  = 0.5*this->m_endpoint1 + 0.5*this->m_endpoint2;
    Eigen::Vector2d otherCenter = 0.5*Other.m_endpoint1 + 0.5*Other.m_endpoint2;
    double centerSep = (otherCenter - thisCenter).norm();
    if (centerSep > 0.5*totalLength)
        return false;

    Eigen::Matrix2d R;
    R << 0, -1,
         1,  0;

    if (this->IsDegenerate()) {
        Intersection = m_endpoint1;
        return Other.ContainsPoint(m_endpoint1);
    }
    else if (Other.IsDegenerate()) {
        Intersection = Other.m_endpoint1;
        return this->ContainsPoint(Other.m_endpoint1);
    }
    else {
        //Neither line segment is degenerate
        Eigen::Vector2d a  = m_endpoint1;
        Eigen::Vector2d b  = m_endpoint2;
        Eigen::Vector2d v1 = b - a;
        v1.normalize();                 //Unit vector pointing along line
        Eigen::Vector2d n1 = R*v1;      //Unit Normal vector to line 1

        Eigen::Vector2d c  = Other.m_endpoint1;
        Eigen::Vector2d d  = Other.m_endpoint2;
        Eigen::Vector2d v2 = d - c;
        v2.normalize();                 //Unit vector pointing along line
        Eigen::Vector2d n2 = R*v2;      //Unit Normal vector to line 2

        //Equation of line 1: v dot n1 = a dot n1
        //Equation of line 2: v dot n2 = c dot n2
        //The solution to this system is the intersection
        Eigen::Matrix2d A;
        A << n1(0), n1(1),
        n2(0), n2(1);
        Eigen::Vector2d rhs(a.dot(n1), c.dot(n2));

        Eigen::Matrix2d AInv;
        bool invertible;
        double determinant;
        A.computeInverseAndDetWithCheck(AInv, determinant, invertible);
        if (! invertible) {
            //This means the lines have the same slope. There is a fringe case where segments can intersect on a sub-segment instead of at a point.
            //In this case, we just return one point in the intersection.
            if (this->ContainsPoint(Other.m_endpoint1)) {
                Intersection = Other.m_endpoint1;
                return true;
            }
            if (this->ContainsPoint(Other.m_endpoint2)) {
                Intersection = Other.m_endpoint2;
                return true;
            }
            if (Other.ContainsPoint(this->m_endpoint1)) {
                Intersection = this->m_endpoint1;
                return true;
            }
            if (Other.ContainsPoint(this->m_endpoint2)) {
                Intersection = this->m_endpoint2;
                return true;
            }

            //If neither segment contains either endpoint of the other segment and the segments have the same slope, then they cannot intersect.
            return false;
        }

        //If we get this far, the lines intersect somewhere. Lets find the intersection.
        Eigen::Vector2d v = AInv * rhs;
        Intersection = v;

        //The segments intersect if v is on both line segments.
        double projLine1 = (v - a).dot(v1);
        double projLine2 = (v - c).dot(v2);
        return ((projLine1 >= 0.0) && (projLine1 <= (b - a).norm()) && (projLine2 >= 0.0) && (projLine2 <= (d - c).norm()));
    }
}

//Same as ComputeIntersection, but only returns true if the intersection is not an endpoint for at least one of the segments.
bool LineSegment::ComputeInteriorIntersection(LineSegment const & Other, Eigen::Vector2d & Intersection, bool & IsInteriorThis, bool & IsInteriorOther) const {
    if (ComputeIntersection(Other, Intersection)) {
        double dist1 = (Intersection - this->m_endpoint1).norm();
        double dist2 = (Intersection - this->m_endpoint2).norm();
        IsInteriorThis = ((dist1 > TOLERANCE) && (dist2 > TOLERANCE));

        dist1 = (Intersection - Other.m_endpoint1).norm();
        dist2 = (Intersection - Other.m_endpoint2).norm();
        IsInteriorOther = ((dist1 > TOLERANCE) && (dist2 > TOLERANCE));

        return (IsInteriorThis || IsInteriorOther);
    }
    else
        return false;
}

//Take two line segments and check for overlap and intersections. If the segments intersect at a point on the interior of at least one
//of the segments or overlap over a set of positive length (e.g. (0,0)-(2,0) and (1,0)-(3,0) have overlap (1,0)-(2,0)) then new segments
//are computed that cover the same path as the input segments but have the property that the computed segments can only intersect each
//other at endpoints. Note that when we say "covers the same path", we are only interested in paths of positive length. Isolated points
//are removed and not represented in the output segments. For instance, if the inputs are the segment (0,0)-(1,0) and the point (which
//is a degenerate line segment) (3,3), then the output will just be the segment (0,0)-(1,0). This function does not typically merge
//co-linear, overlapping segments. The only cases where it will is if one of the input or computed segments is so short that it is
//deemed degenerate. This is so that one can (relatively) safely sanitize pairs of conflicting segments in an iterative fashion
//without having to worry about oscillatory behavior.
//
//When the segments don't intersect or only intersect at endpoints, the function returns false.
//When modification is needed (split or merge), Dst is appended with the new segments that will replace both A and B and we return true.
bool LineSegment::SanitizeSegments(LineSegment const & A, LineSegment const & B, std::Evector<LineSegment> & Dst) {
    //If both segments are points, then they don't cover any length and they can both be dropped
    bool ADegen = A.IsDegenerate();
    bool BDegen = B.IsDegenerate();
    if (ADegen && BDegen)
        return true;
    else if (ADegen || BDegen) {
        //Exactly one of the segments is degenerate - If the degenerate segment is contained in the non-degenerate segment
        //it can be absorbed by the non-degenerate segment. If it isn't contained we can remove the degenerate segment since
        //we don't care about isolated points. In both cases the result is the same... we keep the non-degenerate segment
        //and drop the degenerate one.
        LineSegment const & nonDegenSegment(ADegen ? B : A);
        Dst.push_back(nonDegenSegment);
        return true;
    }
    else {
        //Neither segment is degenerate - this is the typical case

        //If we can quickly and cheaply rule out any intersection, do so now
        double totalLength = A.GetLength() + B.GetLength();
        Eigen::Vector2d ACenter = 0.5*A.m_endpoint1 + 0.5*A.m_endpoint2;
        Eigen::Vector2d BCenter = 0.5*B.m_endpoint1 + 0.5*B.m_endpoint2;
        double centerSep = (BCenter - ACenter).norm();
        if (centerSep > 0.5*totalLength)
            return false; //No intersection is possible

        //An intersection is possible
        Eigen::Vector2d a  = A.m_endpoint1;
        Eigen::Vector2d b  = A.m_endpoint2;
        Eigen::Vector2d v1 = b - a;
        v1.normalize();                        //Unit vector pointing along line
        Eigen::Vector2d n1(-1.0*v1(1), v1(0)); //Unit Normal vector to line 1

        Eigen::Vector2d c  = B.m_endpoint1;
        Eigen::Vector2d d  = B.m_endpoint2;
        Eigen::Vector2d v2 = d - c;
        v2.normalize();                        //Unit vector pointing along line
        Eigen::Vector2d n2(-1.0*v2(1), v2(0)); //Unit Normal vector to line 2

        //Equation of line 1: v dot n1 = a dot n1
        //Equation of line 2: v dot n2 = c dot n2
        //The solution to this system is the intersection, if one exists
        Eigen::Matrix2d M;
        M << n1(0), n1(1),
             n2(0), n2(1);
        Eigen::Vector2d rhs(a.dot(n1), c.dot(n2));

        Eigen::Matrix2d MInv;
        bool invertible;
        double determinant;
        M.computeInverseAndDetWithCheck(MInv, determinant, invertible);
        if (! invertible) {
            //The lines have the same slope to machine precision. If the segments intersect it could be at
            //a point or over a segment of positive length.
            if ((ACenter - BCenter).dot(n1) > TOLERANCE)
                //The line segments are parallel but not co-linear
                return false;
            else {
                //The line segments are co-linear - see if they intersect or overlap
                if (A.ContainsPoint(B.m_endpoint1) || A.ContainsPoint(B.m_endpoint2) ||
                    B.ContainsPoint(A.m_endpoint1) || B.ContainsPoint(A.m_endpoint2)) {
                    //The segments overlap
                	double alpha1 = (A.m_endpoint1 - ACenter).dot(v1);
                    double alpha2 = (A.m_endpoint2 - ACenter).dot(v1);
                    double alpha3 = (B.m_endpoint1 - ACenter).dot(v1);
                    double alpha4 = (B.m_endpoint2 - ACenter).dot(v1);

                    std::vector<double> alphas = {alpha1, alpha2, alpha3, alpha4};
                    std::vector<int> indices = {0, 1, 2, 3};
                    std::sort(indices.begin(), indices.end(), [&alphas](int i1, int i2) {return alphas[i1] < alphas[i2];});

                    std::Evector<Eigen::Vector2d> sortedPoints;
                    sortedPoints.reserve(4);
                    for (int index : indices) {
                    	switch (index) {
                    		case 0: sortedPoints.push_back(A.m_endpoint1); break;
                    		case 1: sortedPoints.push_back(A.m_endpoint2); break;
                    		case 2: sortedPoints.push_back(B.m_endpoint1); break;
                    		case 3: sortedPoints.push_back(B.m_endpoint2); break;
                    	}
                    }

                    double seg1Length = (sortedPoints[1] - sortedPoints[0]).norm();
                    double seg2Length = (sortedPoints[2] - sortedPoints[1]).norm();
                    double seg3Length = (sortedPoints[3] - sortedPoints[2]).norm();

                    if (((seg1Length < TOLERANCE) && (seg2Length < TOLERANCE)) ||
                        ((seg2Length < TOLERANCE) && (seg3Length < TOLERANCE)) ||
                        ((seg1Length < TOLERANCE) && (seg3Length < TOLERANCE))) {
                    	//All 3 segments or any 2 are too short. Combine into a single segment
                    	Dst.emplace_back(sortedPoints[0], sortedPoints[3]);
                    	return true;
                    }

                    //If we get here at most 1 segment is degenerate
                    if ((seg1Length >= TOLERANCE) && (seg2Length >= TOLERANCE) && (seg3Length >= TOLERANCE)) {
                    	Dst.emplace_back(sortedPoints[0], sortedPoints[1]);
                    	Dst.emplace_back(sortedPoints[1], sortedPoints[2]);
                    	Dst.emplace_back(sortedPoints[2], sortedPoints[3]);
                    	return true;
                    }

                    //If we get here then exactly 1 segment is degenerate. Since we are in a 2-segments in, 2-segments out
                    //situation, we need to check carefully whether our output is equivilant to the input.
                    LineSegment seg1, seg2;
                    if (seg1Length < TOLERANCE) {
                    	seg1 = LineSegment(sortedPoints[0], sortedPoints[2]);
                    	seg2 = LineSegment(sortedPoints[2], sortedPoints[3]);
                    }
                    else if (seg2Length < TOLERANCE) {
                    	Eigen::Vector2d midpoint = 0.5*sortedPoints[1] + 0.5*sortedPoints[2];
                    	seg1 = LineSegment(sortedPoints[0], midpoint);
                    	seg2 = LineSegment(midpoint, sortedPoints[3]);
                    }
                    else {
                    	seg1 = LineSegment(sortedPoints[0], sortedPoints[1]);
                    	seg2 = LineSegment(sortedPoints[1], sortedPoints[3]);
                    }
                    if ((AreEquivalent(seg1, A) && AreEquivalent(seg2, B)) || (AreEquivalent(seg1, B) && AreEquivalent(seg2, A)))
                    	return false;
                    else {
                    	Dst.push_back(seg1);
                    	Dst.push_back(seg2);
                    	return true;
                    }
                }
                else
                    return false; //The segments are co-linear but there is a gap between them
            }
        }

        //If we get this far, the lines intersect somewhere. Lets find the intersection.
        Eigen::Vector2d Intersection = MInv * rhs;

        //The segments intersect if the intersection point is on both line segments.
        double projLine1 = (Intersection - a).dot(v1);
        double projLine2 = (Intersection - c).dot(v2);
        if ((projLine1 >= 0.0) && (projLine1 <= (b - a).norm()) && (projLine2 >= 0.0) && (projLine2 <= (d - c).norm())) {
            //The segments intersect
            bool AInterior = (A.GetDistanceToNearestEndpoint(Intersection) > TOLERANCE);
            bool BInterior = (B.GetDistanceToNearestEndpoint(Intersection) > TOLERANCE);
            if (AInterior && BInterior) {
                Dst.emplace_back(A.m_endpoint1, Intersection);
                Dst.emplace_back(Intersection, A.m_endpoint2);
                Dst.emplace_back(B.m_endpoint1, Intersection);
                Dst.emplace_back(Intersection, B.m_endpoint2);
                return true;
            }
            else if (AInterior) {
                Dst.emplace_back(A.m_endpoint1, Intersection);
                Dst.emplace_back(Intersection, A.m_endpoint2);
                Dst.push_back(B);
                return true;
            }
            else if (BInterior) {
                Dst.emplace_back(B.m_endpoint1, Intersection);
                Dst.emplace_back(Intersection, B.m_endpoint2);
                Dst.push_back(A);
                return true;
            }
            else
                return false;
        }
        else
            return false; //The segments do not intersect
    }
}

//Returns true if the overlap between two segments contains at least one point that is not an endpoint of one of the segments
bool LineSegment::HasInteriorOverlap(LineSegment const & A, LineSegment const & B) {
	std::Evector<LineSegment> outSegments;
	return SanitizeSegments(A, B, outSegments);
}

bool LineSegment::AreEquivalent(LineSegment const & A, LineSegment const & B) {
	if (((A.m_endpoint1 - B.m_endpoint1).norm() < TOLERANCE) && ((A.m_endpoint2 - B.m_endpoint2).norm() < TOLERANCE))
		return true;
	if (((A.m_endpoint1 - B.m_endpoint2).norm() < TOLERANCE) && ((A.m_endpoint2 - B.m_endpoint1).norm() < TOLERANCE))
		return true;
	return false;
}


// ************************************************************************************************************************************************
// **********************************************************   SimplePolygon Definitions   *******************************************************
// ************************************************************************************************************************************************

//Return a vector of the line segments that make up the polygon - Note that this function doesn't require the polygon to be simple... it works
//even if we have self intersections, and so can be used in sanitizing.
std::Evector<LineSegment> SimplePolygon::GetLineSegments(void) const {
    std::Evector<LineSegment> segments;
    if (m_vertices.size() < 2U)
        return segments;
    for (size_t n = 0; n < m_vertices.size() - 1U; n++)
        segments.push_back(LineSegment(m_vertices[n], m_vertices[n+1]));
    segments.push_back(LineSegment(m_vertices.back(), m_vertices.front()));
    return segments;
}

// Bisects a given triangle ABC by taking the midpoint of the largest side and cuttting that in half
void Triangle::Bisect(Triangle & subTriangleADC, Triangle & subTriangleABD){
    Triangle tempTriangle;
    tempTriangle.m_pointA =  this->m_pointA;
    tempTriangle.m_pointB =  this->m_pointB;
    tempTriangle.m_pointC =  this->m_pointC;
    if (( this->m_pointA -  this->m_pointB).norm() >= ( this->m_pointA -  this->m_pointC).norm() && ( this->m_pointA -  this->m_pointB).norm() >= ( this->m_pointB -  this->m_pointC).norm()){
        tempTriangle.m_pointA =  this->m_pointC;
        tempTriangle.m_pointB =  this->m_pointA;
        tempTriangle.m_pointC =  this->m_pointB;
    }
    else if (( this->m_pointA -  this->m_pointC).norm() >= ( this->m_pointA -  this->m_pointB).norm() && ( this->m_pointA -  this->m_pointC).norm() >= ( this->m_pointB -  this->m_pointC).norm()){
        tempTriangle.m_pointA =  this->m_pointB;
        tempTriangle.m_pointB =  this->m_pointC;
        tempTriangle.m_pointC =  this->m_pointA;
    }
    Eigen::Vector2d m_pointD;
    m_pointD = (tempTriangle.m_pointB + tempTriangle.m_pointC) / 2;
    subTriangleADC = tempTriangle;
    subTriangleABD = tempTriangle;
    subTriangleADC.m_pointB = m_pointD;
    subTriangleABD.m_pointC = m_pointD;
}

// Converts a Triangle to a Simple Polygon
SimplePolygon Triangle::ToSimplePolygon(){
	return SimplePolygon({this->m_pointA, this->m_pointB, this->m_pointC});
}

// Converts a Triangle to a Polygon
Polygon Triangle::ToPolygon(){
	return Polygon(SimplePolygon({this->m_pointA, this->m_pointB, this->m_pointC}));
}

// Checks if two triangles have the same points in the same order
bool Triangle::isSameTriangle(Triangle const & otherTriangle) {
	return ((this->m_pointA == otherTriangle.m_pointA) &&
		   (this->m_pointB == otherTriangle.m_pointB) &&
		   (this->m_pointC == otherTriangle.m_pointC));
}

//Get the area (m^2) of a triangle whose coordinates are in NM. This is the area assuming each point is at a WGS84 altitude of 0.
//That is, the triangle is assumed to be right on the reference ellipsoid. For relatively small triangles, this will be a very close
//approximation to the area at different altitudes.
double Triangle::GetArea(){
	//Convert each point in the triangle to ECEF
	Eigen::Vector2d A_latlon = NMToLatLon(this->m_pointA);
	Eigen::Vector2d B_latlon = NMToLatLon(this->m_pointB);
	Eigen::Vector2d C_latlon = NMToLatLon(this->m_pointC);
	Eigen::Vector3d A_ECEF   = LLA2ECEF(Eigen::Vector3d(A_latlon(0), A_latlon(1), 0.0));
	Eigen::Vector3d B_ECEF   = LLA2ECEF(Eigen::Vector3d(B_latlon(0), B_latlon(1), 0.0));
	Eigen::Vector3d C_ECEF   = LLA2ECEF(Eigen::Vector3d(C_latlon(0), C_latlon(1), 0.0));

	Eigen::Vector3d ab_vector = A_ECEF - B_ECEF;
	Eigen::Vector3d ac_vector = B_ECEF - C_ECEF;
	return 0.5*(ac_vector.cross(ab_vector)).norm();
}

// Like Bisect, but we bisect to the intersection
// Return false if no intersection. Do not use triangles
bool Triangle::BisectIntersection(const Eigen::Vector2d & Intersection, Triangle & subTriangleADC, Triangle & subTriangleABD){
    Triangle tempTriangle;
    tempTriangle.m_pointA = this->m_pointA;
    tempTriangle.m_pointB = this->m_pointB;
    tempTriangle.m_pointC = this->m_pointC;
    Eigen::Vector2d uniqueP1;
    Eigen::Vector2d uniqueP2;

    // Find the opposite corner
    bool pC = PointsAreColinear(this->m_pointA, this->m_pointB, Intersection);
    bool pB = PointsAreColinear(this->m_pointA, this->m_pointC, Intersection);
    bool pA = PointsAreColinear(this->m_pointC, this->m_pointB, Intersection);

    if (!(pC || pB || pA)){
        return false;
    }
    tempTriangle.m_pointA = Intersection;
    if (pC){
        tempTriangle.m_pointB = this->m_pointC;
        uniqueP1 = this->m_pointA;
        uniqueP2 = this->m_pointB;
    }
    else if (pB){
        tempTriangle.m_pointB = this->m_pointB;
        uniqueP1 = this->m_pointA;
        uniqueP2 = this->m_pointC;
    }
    else if (pA){
        tempTriangle.m_pointB = this->m_pointA;
        uniqueP1 = this->m_pointB;
        uniqueP2 = this->m_pointC;
    }
    else{
        return false;
    }


    subTriangleADC = tempTriangle;
    subTriangleABD = tempTriangle;
    subTriangleADC.m_pointC = uniqueP1;
    subTriangleABD.m_pointC = uniqueP2;
    return true;
}

bool insideColinear(LineSegment const lineA, Eigen::Vector2d pointB){
    bool t1, t2, t3, t4, t5, t6;
    t3 = false;
    t4 = false;
    t5 = false;
    t6 = false;
    // Check if colinear; return false otherwise
    if (!PointsAreColinear(lineA.m_endpoint1, lineA.m_endpoint2, pointB)){
        return false;
    }
    // Check if pointB equals one of the end points
    // If true then it is an endpoint and thus NOT inside the line
    t1 = (lineA.m_endpoint1 - pointB).norm() < TOLERANCE || (lineA.m_endpoint2 - pointB).norm() < TOLERANCE;

    if (t1){
        //std::cout<< "  They share an entire line!" <<std::endl;
        return false;
    }

    // Now we check if pointB falls within lineA
    double normA1 = lineA.m_endpoint1.norm();
    double normA2 = lineA.m_endpoint2.norm();
    double normB = pointB.norm();


    // Check if point B1 falls within lineA
    // if A1 < A2, then A1 < B1 < A2
    if (normA1 < normA2){
        t3 = normB > normA1+TOLERANCE && normB < normA2-TOLERANCE;
    }
    // Check in case A2 < A1
    else if (normA2 < normA1){
        t3 = normB > normA2+TOLERANCE && normB < normA1-TOLERANCE;
    }
    if (t3){
        return true;
    }
    return false;

}
// Handles the case where a vertex would otherwise cut the edge of another triangle
// Trying to decide if we cut triangle A
// Two points of triangle may intersect other
// Returns all intersecting points
bool SimplePolygon::CheckIntersect(SimplePolygon const & otherPolygon, std::vector< Eigen::Vector2d> & FinalIntersect){
    std::Evector<LineSegment> segmentA = this->GetLineSegments();
    std::Evector<Eigen::Vector2d> pointsB = otherPolygon.GetVertices();

    for (int i=0; i < (int) segmentA.size(); i++){
        for (int j=0; j < (int) pointsB.size(); j++){
            //Eigen::Vector2d tempIntersection;

            if (insideColinear(segmentA[i], pointsB[j])){
                FinalIntersect.push_back(pointsB[j]);
            }
        }
    }
    if (FinalIntersect.size()> 0){
        //std::cout<<"Returning this number of points: "<<FinalIntersect.size() << std::endl;
        return true;
    }
    return false;
}


// Returns true if two polygons given share at least one side
bool SimplePolygon::CheckAdjacency(SimplePolygon const & otherPolygon){
    Eigen::Vector2d Intersection;
    bool IsInteriorThis;
    bool IsInteriorOther;

    for (LineSegment & main_line:  this->GetLineSegments()){
        for (LineSegment & other_line: otherPolygon.GetLineSegments()){
            if(!PointsAreColinear(main_line.m_endpoint1, main_line.m_endpoint2, other_line.m_endpoint1) || !PointsAreColinear(main_line.m_endpoint1, main_line.m_endpoint2, other_line.m_endpoint2)) continue;
            if (main_line.ComputeInteriorIntersection(other_line, Intersection, IsInteriorThis, IsInteriorOther)) return true;

            if (main_line.m_endpoint1 == other_line.m_endpoint1) {
                if(other_line.ContainsPoint(main_line.m_endpoint2) || main_line.ContainsPoint(other_line.m_endpoint2)) return true;
            }
            if (main_line.m_endpoint1 == other_line.m_endpoint2) {
                if(other_line.ContainsPoint(main_line.m_endpoint2) || main_line.ContainsPoint(other_line.m_endpoint1)) return true;
            }

            if (main_line.m_endpoint2 == other_line.m_endpoint1) {
                if(other_line.ContainsPoint(main_line.m_endpoint1) || main_line.ContainsPoint(other_line.m_endpoint2)) return true;
            }
            if (main_line.m_endpoint2 == other_line.m_endpoint2) {
                if(other_line.ContainsPoint(main_line.m_endpoint1) || main_line.ContainsPoint(other_line.m_endpoint1)) return true;
            }
        }
    }

    for (LineSegment & other_line:  this->GetLineSegments()){
        for (LineSegment & main_line: otherPolygon.GetLineSegments()){
            if(!PointsAreColinear(main_line.m_endpoint1, main_line.m_endpoint2, other_line.m_endpoint1) || !PointsAreColinear(main_line.m_endpoint1, main_line.m_endpoint2, other_line.m_endpoint2)) continue;
            if (main_line.ComputeInteriorIntersection(other_line, Intersection, IsInteriorThis, IsInteriorOther)) return true;

            if (main_line.m_endpoint1 == other_line.m_endpoint1) {
                if(other_line.ContainsPoint(main_line.m_endpoint2) || main_line.ContainsPoint(other_line.m_endpoint2)) return true;
            }
            if (main_line.m_endpoint1 == other_line.m_endpoint2) {
                if(other_line.ContainsPoint(main_line.m_endpoint2) || main_line.ContainsPoint(other_line.m_endpoint1)) return true;
            }

            if (main_line.m_endpoint2 == other_line.m_endpoint1) {
                if(other_line.ContainsPoint(main_line.m_endpoint1) || main_line.ContainsPoint(other_line.m_endpoint2)) return true;
            }
            if (main_line.m_endpoint2 == other_line.m_endpoint2) {
                if(other_line.ContainsPoint(main_line.m_endpoint1) || main_line.ContainsPoint(other_line.m_endpoint1)) return true;
            }
        }
    }
    return false;
}


//Get the point inside or on the edge of the polygon nearest to the provided point
Eigen::Vector2d SimplePolygon::ProjectPoint(Eigen::Vector2d const & Point) const {
    if (ContainsPoint(Point))
        return Point;

    //If we get here, the point is outside of the polygon or possibly on the boundary. The closest point will be on the boundary.
    return ProjectPointToBoundary(Point);
}

//Get the point on the boundary of the polygon that is nearest to the provided point
Eigen::Vector2d SimplePolygon::ProjectPointToBoundary(Eigen::Vector2d const & Point) const {
    std::Evector<LineSegment> segments = GetLineSegments();
    Eigen::Vector2d BestCandidatePoint = Point; //Default return value if we don't have any segments for some reason
    double distToBestCandidate = -1.0; //Negative indicates not initialized yet
    for (auto const & segment : segments) {
        Eigen::Vector2d candidate = segment.ProjectPoint(Point);
        double dist = (candidate - Point).norm();
        if ((distToBestCandidate < 0.0) || (dist < distToBestCandidate)) {
            BestCandidatePoint = candidate;
            distToBestCandidate = dist;
        }
    }
    return BestCandidatePoint;
}

//Get the point on the boundary of the polygon that is nearest to the provided point and get the index of the closest vertex
Eigen::Vector2d SimplePolygon::ProjectPointToBoundary(Eigen::Vector2d const & Point, size_t & ClosestVertexIndex) const {
    //Handle degenerate cases
    ClosestVertexIndex = 0U;
    if (m_vertices.empty())
        return Point;
    if (m_vertices.size() == 1U)
        return m_vertices[0];

    //Find the segment containing the closest point to the given point
    size_t BestStartVertexIndex = 0U;
    double BestSegmentProjectionDist = std::nan("");
    Eigen::Vector2d ProjectionToBestSegment = Eigen::Vector2d::Zero();
    for (size_t n = 0; n < m_vertices.size(); n++) {
        LineSegment segment(m_vertices[n], n + 1U < m_vertices.size() ? m_vertices[n + 1U] : m_vertices[0]);
        Eigen::Vector2d projection = segment.ProjectPoint(Point);
        double dist = (projection - Point).norm();
        if (std::isnan(BestSegmentProjectionDist) || (dist < BestSegmentProjectionDist)) {
            BestStartVertexIndex = n;
            BestSegmentProjectionDist = dist;
            ProjectionToBestSegment = projection;
        }
    }

    //Now check which endpoint of the segment containing the projection is closer to the given point
    size_t indexA = BestStartVertexIndex;
    size_t indexB = (indexA + 1U < m_vertices.size()) ? indexA + 1U : 0U;
    double distFromPointA = (Point - m_vertices[indexA]).norm();
    double distFromPointB = (Point - m_vertices[indexB]).norm();
    ClosestVertexIndex = (distFromPointA < distFromPointB) ? indexA : indexB;

    return ProjectionToBestSegment;
}

//Test to see if polygon contains a point in its interior (not especially stable on the boundary)
bool SimplePolygon::ContainsPoint(Eigen::Vector2d const & Point) const {
    // Test to see if a point is inside a polygon. Based on code from: http://geomalgorithms.com/a03-_inclusion.html. Copyright info:
    // Copyright 2000 softSurfer, 2012 Dan Sunday. This code may be freely used and modified for any purpose providing that this
    // copyright notice is included with it. SoftSurfer makes no warranty for this code, and cannot be held liable for any real or
    // imagined damage resulting from its use. Users of this code must verify correctness for their application.
    if (m_vertices.size() < 3U)
        return false;

    //Append first point to end and perform winding number test
    std::Evector<Eigen::Vector2d> V = m_vertices;
    V.push_back(V.front());
    return (PointIsInsidePolygonHelper_wn_PnPoly(Point, V) != 0);
}

//Get the X and Y bounds of the polygon (An Axis-Aligned Bounding Box). Returned as a vector (XMin, XMax, YMin, YMax).
Eigen::Vector4d SimplePolygon::GetAABB(void) const {
    if (m_vertices.empty())
        return Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);

    double XMin = m_vertices.front()(0);
    double XMax = m_vertices.front()(0);
    double YMin = m_vertices.front()(1);
    double YMax = m_vertices.front()(1);
    for (Eigen::Vector2d const & point : m_vertices) {
        XMin = std::min(XMin, point(0));
        XMax = std::max(XMax, point(0));
        YMin = std::min(YMin, point(1));
        YMax = std::max(YMax, point(1));
    }
    return Eigen::Vector4d(XMin, XMax, YMin, YMax);
}

//Compute the area enclosed by a polygon.
double SimplePolygon::GetArea(void) const {
    //This algorithm assumes that the polygon only intersects itself at nodes and always keeps the interior to the same side while traversing nodes.
    //It will typically give a wrong answer if these conditions are not met. Our requirements for a "simple polygon" satisfy these conditions,
    //so if the polygon is valid this should always work.
    if (m_vertices.size() < 3U)
        return 0.0;

    double signedArea = 0.0;
    for (size_t n = 0; n < m_vertices.size(); n++) {
        Eigen::Vector2d P0 = m_vertices[n];
        Eigen::Vector2d P1 = (n == m_vertices.size() - 1U) ? m_vertices[0] : m_vertices[n + 1U];

        double x0 = P0(0);
        double y0 = P0(1);
        double x1 = P1(0);
        double y1 = P1(1);
        signedArea += x0*y1 - x1*y0;
    }
    return 0.5*fabs(signedArea);
}

//Break edges of this simple polygon and another simple polygon into two pieces whenever they are found to intersect an edge of the other.
//This can modify both polygons. On completion the two polygons will have boundaries that only cross at vertices. It is still possible, however,
//for them to have overlapping boundary segments.
void SimplePolygon::FragmentIntersections(SimplePolygon & OtherPoly) {
    for (size_t n = 0U; n < this->m_vertices.size(); n++) {
        //Consider edge between vertices n and n+1 in this polygon
        Eigen::Vector2d this_endpoint1 = this->m_vertices[n];
        Eigen::Vector2d this_endpoint2 = (n + 1U == this->m_vertices.size()) ? this->m_vertices[0] : this->m_vertices[n + 1U];
        LineSegment edgeA(this_endpoint1, this_endpoint2);

        for (size_t m = 0U; m < OtherPoly.m_vertices.size(); m++) {
            //Consider the edge between vertices m and m+1 in the other polygon
            Eigen::Vector2d other_endpoint1 = OtherPoly.m_vertices[m];
            Eigen::Vector2d other_endpoint2 = (m + 1U == OtherPoly.m_vertices.size()) ? OtherPoly.m_vertices[0] : OtherPoly.m_vertices[m + 1U];
            LineSegment edgeB(other_endpoint1, other_endpoint2);

            Eigen::Vector2d intersection;
            bool IsInteriorA, IsInteriorB;
            if (edgeA.ComputeInteriorIntersection(edgeB, intersection, IsInteriorA, IsInteriorB)) {
                //There is an interior intersection between segments A and B - fracture the edges for which the intersection is an interior point
                if (IsInteriorA) {
                    this->m_vertices.insert(this->m_vertices.begin() + n + 1U, intersection);
                    edgeA.m_endpoint2 = intersection; //Update edgeA to be the first fragment of the previous edge
                }
                if (IsInteriorB) {
                    OtherPoly.m_vertices.insert(OtherPoly.m_vertices.begin() + m + 1U, intersection);
                    m++; //Skip test with the second fragment of edge B - it can't have a second interior intersection with edgeA
                }
            }
        }
    }
}

//Returns true if this polygon completely contains the given "other" polygon in its interior. Not stable if their boundaries touch.
bool SimplePolygon::Contains(SimplePolygon const & OtherPoly) const {
    //We contain the other polygon iff we contain all vertices of the other polygon and our boundaries have no intersections.
    for (auto const & vertex : OtherPoly.m_vertices) {
        if (! this->ContainsPoint(vertex))
            return false;
    }

    std::Evector<LineSegment> edgeSegmentsA = this->GetLineSegments();
    std::Evector<LineSegment> edgeSegmentsB = OtherPoly.GetLineSegments();
    for (auto const & edgeA : edgeSegmentsA) {
        for (auto const & edgeB : edgeSegmentsB) {
            Eigen::Vector2d Intersection;
            if (edgeA.ComputeIntersection(edgeB, Intersection))
                return false;
        }
    }

    return true;
}

//Returns true if this polygon intersects with the given "other" polygon. Not stable if the intersection has 0 area.
bool SimplePolygon::IntersectsWith(SimplePolygon const & OtherPoly) const {
    //We have an intersection if any vertex of either polygon is in the interior of the other, or if the boundaries instersect.
    for (auto const & vertex : OtherPoly.m_vertices) {
        if (this->ContainsPoint(vertex))
            return true;
    }
    for (auto const & vertex : this->m_vertices) {
        if (OtherPoly.ContainsPoint(vertex))
            return true;
    }

    //If we get here, niether polygon contains a vertex of the other. They can still intersect though if their boundaries cross
    std::Evector<LineSegment> edgeSegmentsA = this->GetLineSegments();
    std::Evector<LineSegment> edgeSegmentsB = OtherPoly.GetLineSegments();
    for (auto const & edgeA : edgeSegmentsA) {
        for (auto const & edgeB : edgeSegmentsB) {
            Eigen::Vector2d Intersection;
            if (edgeA.ComputeIntersection(edgeB, Intersection))
                return true;
        }
    }

    return false;
}

//TODO:
//Optimizations in history structure and updates in shadow detection module
//Final cleanup in shadow detection module + commit

//Trace the outline of the polygonal object represented by m_vertices. Compute new vertices from the outline that traces the object and keeps
//it's interior to the left. If m_vertices already defined a valid simple polygon and kept the interior to the left this should (I hope) leave
//the object essentially unchanged, except it might add vertices if the polygon touched itself at an isolated point, but the shape should be the same.
//
//This is actually more challenging than it seems and most simple approaches leave edge cases that give poor results, like clipping
//off large portions of the object. This function is rather complex, but most of the complexity is there to better handle strange edge
//cases. At a high level, the algorithm has several steps:
//1 - Build a collection of line segments representing the original object
//2 - Iteratively sanitize the line segments and replace elements as necessary to get segments that only intersect one another at endpoints
//3 - Build a graph where vertices are endpoints of segments and edges exist between vertices that are connected by a segment
//4 - Iteratively prune leaf nodes in the graph until none remain.
//5 - Select a vertex that we know is on the outer trace of the polygon and a first edge that keeps the interior to the left
//6 - Select the next node from those adjacent to the last one that maximizes the internal angle with the last segment.
//    Repeat this until we come back to the first node. We allow traversing edges multiple times at this stage.
//7 - The processing thus far does a good job of removing interior cruft and giving us an outline of the original shape, but
//    there may be multiple components that each independently contain area and which are only connected to each other through
//    chains of segments that enclose no area. These would not have been removed during leaf pruning since they aren't actually leaves.
//    In this state we may have an object that does not obey our rules for a simple polygon because there are self intersections.
//    We identify the largest component and prune off all smaller components.
void SimplePolygon::Sanitize(void) {
	//If the polygon is degenerate, there is no "good" answer here. Just leave it alone.
	if (m_vertices.size() < 3U)
		return;

	//Build a collection of line segments that form the full polygon and break them at intersections so the resulting segments can only intersect at nodes.
	std::Evector<LineSegment> segments = GetLineSegments();
	segments = SanitizeCollectionOfSegments(segments);

	//Note that when segments are sanitized, the new path should be nearly exactly the same as the original path. When a segment is
	//split into two segments, the point where the original segment was split will be an endpoint of both output line segments.
	//However, some new segments may contain endpoints that were not endpoints of the initial segment and there can be some round-off
	//error in the computation of these points.

	//Change our representation to a node graph
	std::Evector<Eigen::Vector2d> NodeLocations;        NodeLocations.reserve(segments.size());
	std::vector<std::unordered_set<int>> AdjacentNodes; AdjacentNodes.reserve(segments.size());
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

		AdjacentNodes[endpoint1Index].insert(endpoint2Index);
		AdjacentNodes[endpoint2Index].insert(endpoint1Index);
	}

	//Iteratively identify leaf nodes to prune from the graph until none remain
	std::unordered_set<int> indicesToPrune;
	while (true) {
		bool leavesFound = false;
		for (int n = 0; n < (int) AdjacentNodes.size(); n++) {
			if (indicesToPrune.count(n) > 0U)
				continue; //This node is already marked for deletion - skip it
			int numberOfAdjacentNodes = 0;
			for (int adjacentNodeIndex : AdjacentNodes[n]) {
				if (indicesToPrune.count(adjacentNodeIndex) == 0U)
					numberOfAdjacentNodes++;
			}
			//std::cerr << "Node " << n << ": Adjacent nodes: " << numberOfAdjacentNodes << "\r\n";
			if (numberOfAdjacentNodes <= 1) {
				indicesToPrune.insert(n);
				leavesFound = true;
			}
		}
		if (! leavesFound)
			break;
	}
	//if (! indicesToPrune.empty())
	//	std::cerr << "Pruning " << (int) indicesToPrune.size() << " leaf nodes.\r\n";
	//Re-build our graph data structures without the pruned nodes
	std::vector<int> oldIndexToNewIndex(NodeLocations.size());
	for (int oldIndex = 0; oldIndex < (int) NodeLocations.size(); oldIndex++) {
		//The new index is the old index minus the number of nodes with lower indices marked for deletion
		//If this node is marked for deletion, we use -1 for the new index
		if (indicesToPrune.count(oldIndex) > 0U)
			oldIndexToNewIndex[oldIndex] = -1;
		else {
			int numberOfNodesWithLowerIndicesMarkedForDeletion = 0;
			for (int index : indicesToPrune) {
				if (index < oldIndex)
					numberOfNodesWithLowerIndicesMarkedForDeletion++;
			}
			oldIndexToNewIndex[oldIndex] = oldIndex - numberOfNodesWithLowerIndicesMarkedForDeletion;
		}
	}
	{
		std::Evector<Eigen::Vector2d> NewNodeLocations;        NewNodeLocations.reserve(NodeLocations.size());
		std::vector<std::unordered_set<int>> NewAdjacentNodes; NewAdjacentNodes.reserve(NodeLocations.size());
		for (int oldIndex = 0; oldIndex < (int) NodeLocations.size(); oldIndex++) {
			if (oldIndexToNewIndex[oldIndex] >= 0) {
				NewNodeLocations.push_back(NodeLocations[oldIndex]);
				NewAdjacentNodes.emplace_back();
				for (int adjacentNodeOldIndex : AdjacentNodes[oldIndex]) {
					if (oldIndexToNewIndex[adjacentNodeOldIndex] >= 0)
						NewAdjacentNodes.back().insert(oldIndexToNewIndex[adjacentNodeOldIndex]);
				}
			}
		}
		NodeLocations.swap(NewNodeLocations);
		AdjacentNodes.swap(NewAdjacentNodes);
	}

	//If there is nothing left after pruning, then stop here
	if (NodeLocations.size() < 3U) {
		m_vertices.clear();
		return;
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
	//std::cerr << "NodeLocations.size(): " << NodeLocations.size() << "\r\n";
	//std::cerr << "AdjacentNodes.size(): " << AdjacentNodes.size() << "\r\n";
	//std::cerr << "Starting node: " << extremalNodeIndex << "\r\n";

	//We will populate a new polygonal object and store it as a sequence of node indices... starting with the one we just found.
	std::vector<int> trajectory;
	trajectory.reserve(NodeLocations.size());
	trajectory.push_back(extremalNodeIndex);

	//We keep track of the number of times each segment is traversed
	std::unordered_map<std::tuple<int,int>, int> edgeUseTallies; //<lowIndex, highIndex> --> number of times this edge is traversed

	//Choose the first edge to be the one closest to direction (1,0). All edges move up from this node because of how it was chosen
	{
		double maxInternalAngle = -1.0;
		int bextNextNodeIndex = -1;
		for (int nodeIndex : AdjacentNodes[extremalNodeIndex]) {
			//std::cerr << "Checking node with index " << nodeIndex << "\r\n";
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
			m_vertices.clear();
			return;
		}
		trajectory.push_back(bextNextNodeIndex);
		edgeUseTallies[std::make_tuple(std::min(extremalNodeIndex, bextNextNodeIndex), std::max(extremalNodeIndex, bextNextNodeIndex))]++;
	}

	//Now traverse edges, one at a time. In each step we select the next edge that maximizes the internal angle with the last edge.
	//This keeps us on the outermost boundary of the object. We are done when we get back to the node we started at.
	//We do not modify the graph while we go so we deliberately allow edges to be traversed multiple times. This is important
	//to prevent clipping if there are multiple components that are only connected by a single path. However, it also introduces
	//some risk. If we remove an edge from the graph in each step then we have a guaranteed stopping condition since we will eventually
	//run out of edges, even if something unexpected happens. However, since we don't do this we would be relying on a more theoretical
	//guarantee that we should eventually reach the starting node again. To be safe, we also have a failsafe to ensure that even if
	//there are some edge cases that we haven't thought of, this stage will always eventually finish. We want to make sure that our only
	//failure mode is a bad final polygon... not an infinite loop.
	while (true) {
		int currentNodeIndex  = trajectory[trajectory.size() - 1U];
		int previousNodeIndex = trajectory[trajectory.size() - 2U];

		if (AdjacentNodes[currentNodeIndex].size() == 1U) {
			//If there is only one option of where to go from here then the "choice" is easy
			int nextNodeIndex = *(AdjacentNodes[currentNodeIndex].begin());
			trajectory.push_back(nextNodeIndex);
			edgeUseTallies[std::make_tuple(std::min(currentNodeIndex, nextNodeIndex), std::max(currentNodeIndex, nextNodeIndex))]++;
		}
		else {
			//We have multiple options for where to go from here. Choose the segment with max internal angle
			Eigen::Vector2d A = NodeLocations[previousNodeIndex];
			Eigen::Vector2d B = NodeLocations[currentNodeIndex];

			double maxInternalAngle = -1.0;
			int bextNextNodeIndex = -1;
			for (int nodeIndex : AdjacentNodes[currentNodeIndex]) {
				//Never go directly back to the last node (remember all leaves have been pruned so this should never be necessary)
				//This test is important because going back in the exact direction we came from is the case where the internal
				//angle toggles between 0 and 2*pi, meaning round off error determines whether this looks like the best or worst
				//option. Thus, we need to manually exclude this option.
				if (nodeIndex == previousNodeIndex)
					continue;
				Eigen::Vector2d C = NodeLocations[nodeIndex];
				double internalAngle = getInternalAngle(A, B, C);
				if (internalAngle > maxInternalAngle) {
					maxInternalAngle = internalAngle;
					bextNextNodeIndex = nodeIndex;
				}
			}
			//Note: this case should never happen now since we don't edit the adjacency map as we go.
			//This should only be possible at a leaf node and we are supposed to have trimmed all leaf nodes already.
			if (bextNextNodeIndex < 0) {
				std::cerr << "Internal Error in SimplePolygon::Sanitize(). We seem to be stuck at a leaf node even though there ";
				std::cerr << "shouldn't be any. Closing from here - this will probably clip off some of the object.\r\n";
				std::cerr << "Last vertex: " << trajectory.back() << "\r\n";
				trajectory.push_back(trajectory.front());
				edgeUseTallies[std::make_tuple(std::min(currentNodeIndex, trajectory.front()), std::max(currentNodeIndex, trajectory.front()))]++;
				break;
			}
			else {
				trajectory.push_back(bextNextNodeIndex);
				edgeUseTallies[std::make_tuple(std::min(currentNodeIndex, bextNextNodeIndex), std::max(currentNodeIndex, bextNextNodeIndex))]++;
			}
		}

		//Test regular termination condition
		if (trajectory.front() == trajectory.back())
			break;

		//Test failsafe termination condition
		if (trajectory.size() > 2U*NodeLocations.size()) {
			std::cerr << "Internal error in SimplePolygon::Sanitize(): New trajectory is unreasonably long. This most likely ";
			std::cerr << "means that our outline trace is going in circles, which shouldn't happen. Closing from here - ";
			std::cerr << "this will probably clip off some of the object.\r\n";
			trajectory.push_back(trajectory.front());
			break;
		}
	}
	trajectory.pop_back(); //Remove the last node since there is an implicit connection between the last and first nodes

	//Since we now allow edges to be traversed multiple times the current trajectory is not guaranteed to meet our
	//requirements for a simple polygon. We need to identify the largest simple polygon component and prune off any
	//sections that are only connected to the main components through a single edge or chain of edges.
	//
	//There are some options for how to do this. We could throw out all nodes in our graph that were not traversed and then
	//do an all-node-to-all-node max flow computation. If two nodes have a max flow between them of 1 it means that all
	//paths between them go through a bottleneck, which means they are in different components, as we have defined them.
	//That is rather expensive though, so we do something else. We travel down the trajectory that we just found, node by node,
	//and assign a label to each node. When we traverse an edge that is only used one time in our trajectory we assign the same
	//label as the previous node. When we traverse an edge that is used multiple times in our trajectory we must be moving between
	//components so we change our current label based on the number of times we have traversed the edge so far. If it's our first
	//time we get a new, unused label. If it is our second use of the edge we are returning to a component and so we go back to the
	//label used by that component. When we are done, we select the label with the greatest number of vertices and prune all nodes
	//that were assigned a different label. We could also use area to pick the dominant component, but vertex count is faster.
	//
	//It seems that we should never see an edge used more than twice in a trajectory formed by a max-internal-angle trace so
	//we shouldn't need to worry about 3rd, 4th, or later crossings of an edge. I'm not certain though that this is guaranteed.
	//We treat later crossings the same as the second crossing, even though they probably shouldn't happen.
	{
		if (trajectory.empty()) {
			m_vertices.clear();
			return;
		}

		//We keep track of how many times each edge is traversed
		std::unordered_map<std::tuple<int,int>, int> edgeTracker; //<lowIndex, highIndex> --> number of times this edge is traversed
		std::unordered_map<int, int> numVerticesByLabel; //label --> tally

		std::vector<int> nodeLabels(trajectory.size());
		nodeLabels[0] = 0;
		numVerticesByLabel[0]++;
		for (size_t trajIndex = 1U; trajIndex < trajectory.size(); trajIndex++) {
			int lastIndex = trajectory[trajIndex - 1U];
			int thisIndex = trajectory[trajIndex];
			std::tuple<int, int> edgeKey(std::min(lastIndex, thisIndex), std::max(lastIndex, thisIndex));
			if (edgeUseTallies.at(edgeKey) == 1U)
				nodeLabels[trajIndex] = nodeLabels[trajIndex - 1U];
			else {
				//We are crossing an edge that is used multiple times
				if (edgeTracker[edgeKey] == 0U) {
					//This is our first crossing into a new component - get a new label
					int newLabel = 0;
					while (numVerticesByLabel.count(newLabel) > 0U)
						newLabel++;
					nodeLabels[trajIndex] = newLabel;
				}
				//If this isn't our first time traversing this edge, then we don't actually need to assign a label
				//to the current node since it already has one and we want to go back to using that label
			}
			edgeTracker[edgeKey]++;
			numVerticesByLabel[nodeLabels[trajIndex]]++;
		}

		int mainLabel = 0;
		int numVerticesInBiggestComponent = -1;
		for (auto const & kv : numVerticesByLabel) {
			int label = kv.first;
			int numVertices = kv.second;
			if ((numVerticesInBiggestComponent < 0) || (numVertices > numVerticesInBiggestComponent)) {
				mainLabel = label;
				numVerticesInBiggestComponent = numVertices;
			}
		}

		//We want to prune off all nodes that are not in the dominant group
		m_vertices.clear();
		m_vertices.reserve(trajectory.size());
		for (size_t trajIndex = 0U; trajIndex < trajectory.size(); trajIndex++) {
			if (nodeLabels[trajIndex] == mainLabel)
				m_vertices.push_back(NodeLocations[trajectory[trajIndex]]);
		}
	}
}


// ************************************************************************************************************************************************
// *************************************************************   Polygon Definitions   **********************************************************
// ************************************************************************************************************************************************

//Remove holes with no area
void Polygon::RemoveTrivialHoles(void) {
    size_t index = 0U;
    while (index < m_holes.size()) {
        if ((m_holes[index].NumVertices() <= 2U) || (m_holes[index].GetArea() <= 0.0))
            m_holes.erase(m_holes.begin() + index);
        else
            index++;
    }
}

//Remove holes with no vertices
void Polygon::RemoveEmptyHoles(void) {
    size_t index = 0U;
    while (index < m_holes.size()) {
        if (m_holes[index].Empty())
            m_holes.erase(m_holes.begin() + index);
        else
            index++;
    }
}

//Test to see if polygon contains a point
bool Polygon::ContainsPoint(Eigen::Vector2d const & Point) const {
    bool inHole = false;
    for (auto const & hole : m_holes) {
        if (hole.ContainsPoint(Point)) {
            inHole = true;
            break;
        }
    }
    return (m_boundary.ContainsPoint(Point) && (! inHole));
}

//Get the area of a *valid* polygon.
double Polygon::GetArea(void) const {
    double area = m_boundary.GetArea();
    for (auto const & hole : m_holes)
        area -= hole.GetArea();
    return area;
}

//Returns true if the contents of the polygon meet the geometry requirements - the holes must be completely within the interior and non-overlapping.
bool Polygon::IsValid(void) const {
    for (auto const & hole : m_holes) {
        if (! m_boundary.Contains(hole))
            return false;
    }
    for (size_t n = 0U; n < m_holes.size(); n++) {
        for (size_t m = n + 1U; m < m_holes.size(); m++) {
            if (m_holes[n].IntersectsWith(m_holes[m]))
                return false;
        }
    }

    return true;
}

//Returns true if this polygon intersects with the given "other" polygon. Not stable if the intersection has 0 area.
bool Polygon::IntersectsWith(Polygon const & OtherPoly) const {
    //We have a non-empty intersection if the boundary polygons intersect (neglecting holes) and it is not the case that the boundary of
    //one polygon is contained within a hole of the other polygon. Keep in mind that a Polygon, as we have defined it,
    //cannot have separate, disjoint components - this simplifies the logic here. If the boundary polygons don't intersect
    //as simple polygons then clearly there is no intersection. Thus, we restrict our focus to the case that they do
    //intersect (as simple polygons... that is, ignoring holes).
    //
    //Claim: It is not the case that one poly is completely in a hole of the other => Non-empty intersection
    //Prove by contradiction. Assume empty intersection.
    //Define the "simple closure" of a polygon to be the closure of the outer boundary simple polygon.
    //Again, if the simple closures of A and B don't intersect then clearly A and B don't intersect so we are only looking at
    //the case that the simple closures do intersect.
    //
    //Lemma: Some point in one poly (WLOG say A) must lie in the simple closure of the other (B).
    //Proof: Some point in the simple closure of A must lie in the simple closure of B. Call such a point P.
    //If P is in A, then we are done (P is in A and in the simple closure of B). Thus, let's assume P is not in A, which meens
    //it must be in a hole H, of A. Form a path from P to a point on the boundary of H, call that point PPrime. If this path stays
    //completely within the simple closure of B, then we are done since PPrime meets our criteria. If not, then the path intersects
    //the boundary of B at some other point before reaching the boundary of H. Call that point PCross. PCross is on the boundary of
    //B but still in the hole H. Traverse the boundary of B from PCross. If we intersect the boundary of the hole H, then that
    //intersection is in A and in the simple closure of B. If we make it back to PCross without ever hitting the boundary of H then the entire
    //simple closure of B is in the hole H, which means some point (in fact all points) in B are in the simple closure of A.
    //
    //With this lemma, we know some point in A, call it P, lies in the simple closure of B. If P is not in a hole of B then it is in
    //the intersection of A and B and we have our contradiction. Thus, P must lie in a hole, H, of B. We know that not every point in
    //A can lie in this hole because of our hypothesis. So some other point in A (call it PPrime) must lie outside of this hole.
    //By our definition of a Polygon (simple polygon boundary with non-intersecting holes completely in the interior) we know that there
    //must be a path from P to PPrime that stays completely within A. Since P is in H, PPrime is not, and H is defined by a simple polygon
    //then the path must intersect the boundary of H. The intersection is both in A and B and so is in the intersection, which gives us our
    //contradiction.

    if (! this->m_boundary.IntersectsWith(OtherPoly.m_boundary))
        return false;

    //Check to see if any hole of this polygon contains the outer boundary of the other polygon
    for (SimplePolygon const & hole : this->m_holes) {
        if (hole.Contains(OtherPoly.m_boundary))
            return false;
    }

    //Check to see if any hole of the other polygon contains the outer boundary of this polygon
    for (SimplePolygon const & hole : OtherPoly.m_holes) {
        if (hole.Contains(this->m_boundary))
            return false;
    }

    return true;
}

//Populate a vector of triangles exactly covering the same area as the polygon collection. Assumes object is valid. Uses Earcut algorithm.
//This function does not clear "Triangles" - it appends to it. This is so we can ask multiple objects to add their triangles to the same vector
void Polygon::Triangulate(std::Evector<Triangle> & Triangles) const {
    using Point = std::array<double, 2>;
    std::vector<std::vector<Point>> polygon;
    std::vector<Point> vertexVec;

    size_t totalVertices = m_boundary.NumVertices();
    for (auto const & hole : m_holes)
        totalVertices += hole.NumVertices();
    vertexVec.reserve(totalVertices);

    //The first item in the outer vector is the boundary polygon
    {
        std::Evector<Eigen::Vector2d> const & vertices(m_boundary.GetVertices());
        polygon.emplace_back();
        polygon.back().reserve(vertices.size());
        for (auto const & vertex : vertices) {
            polygon.back().push_back({vertex(0), vertex(1)});
            vertexVec.push_back({vertex(0), vertex(1)});
        }
    }

    //All subsequent items in the outer vector define holes
    for (auto const & hole : m_holes) {
        std::Evector<Eigen::Vector2d> const & vertices(hole.GetVertices());
        polygon.emplace_back();
        polygon.back().reserve(vertices.size());
        for (auto const & vertex : vertices) {
            polygon.back().push_back({vertex(0), vertex(1)});
            vertexVec.push_back({vertex(0), vertex(1)});
        }
    }

    //Run Earcut
    std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(polygon);

    //Pack results into "Triangles"
    //std::cerr << "Indices:\r\n";
    //for (auto const & n : indices)
    //	std::cerr << n << " - (" << (vertexVec[n])[0] << ", " << (vertexVec[n])[1] <<  ")\r\n";
    Triangles.reserve(Triangles.size() + (indices.size() / 3U));
    size_t n = 0U;
    while (n < indices.size()) {
        if (n + 2 >= indices.size()) {
            std::cerr << "Internal Error: Parsing Earcut results gave partial triangle.\r\n";
            break;
        }

        //We don't verify that indices contains only valid indices to save on time, but if Earcut gives us garbage we could segfault.
        Triangles.emplace_back();
        Triangles.back().m_pointA << (vertexVec[indices[n     ]])[0], (vertexVec[indices[n     ]])[1];
        Triangles.back().m_pointB << (vertexVec[indices[n + 1U]])[0], (vertexVec[indices[n + 1U]])[1];
        Triangles.back().m_pointC << (vertexVec[indices[n + 2U]])[0], (vertexVec[indices[n + 2U]])[1];

        n += 3U;
    }
}

// ************************************************************************************************************************************************
// ********************************************************   PolygonCollection Definitions   *****************************************************
// ************************************************************************************************************************************************

//Remove trivial holes from all components
void PolygonCollection::RemoveTrivialHoles(void) {
    for (auto & comp : m_components)
        comp.RemoveTrivialHoles();
}

//Remove empty holes from all components
void PolygonCollection::RemoveEmptyHoles(void) {
    for (auto & comp : m_components)
        comp.RemoveEmptyHoles();
}

//Remove trivial components
void PolygonCollection::RemoveTrivialComponents(void) {
    size_t index = 0U;
    while (index < m_components.size()) {
        if (m_components[index].IsTrivial())
            m_components.erase(m_components.begin() + index);
        else
            index++;
    }
}

//Remove components with empty boundaries
void PolygonCollection::RemoveEmptyComponents(void) {
    size_t index = 0U;
    while (index < m_components.size()) {
        if (m_components[index].m_boundary.Empty())
            m_components.erase(m_components.begin() + index);
        else
            index++;
    }
}

bool PolygonCollection::ContainsPoint(Eigen::Vector2d const & Point) const {
    for (auto const & poly : m_components) {
        if (poly.ContainsPoint(Point))
            return true;
    }
    return false;
}

//Get the area of a *valid* polygon collection
double PolygonCollection::GetArea(void) const {
    double area = 0.0;
    for (auto const & comp : m_components)
        area += comp.GetArea();
    return area;
}

//Returns true if the contents of the object meet the geometry requirements - all components are valid and disjoint from one another.
bool PolygonCollection::IsValid(void) const {
    for (auto const & comp : m_components) {
        if (! comp.IsValid())
            return false;
    }

    for (size_t n = 0U; n < m_components.size(); n++) {
        for (size_t m = n + 1U; m < m_components.size(); m++) {
            if (m_components[n].IntersectsWith(m_components[m]))
                return false;
        }
    }

    return true;
}

//Populate a vector of triangles exactly covering the same area as the polygon collection. Assumes object is valid. Uses Earcut algorithm
void PolygonCollection::Triangulate(std::Evector<Triangle> & Triangles) const {
    Triangles.clear();

    for (auto const & comp : m_components)
        comp.Triangulate(Triangles);
}


// ************************************************************************************************************************************************
// *********************************************************   Public Function Definitions   ******************************************************
// ************************************************************************************************************************************************

//Take a collection of line segments and break/merge as needed to get a new collection that covers the same path but for
//which segments can only intersect at vertices.
std::Evector<LineSegment> SanitizeCollectionOfSegments(std::Evector<LineSegment> const & InputSegments) {
	//First we build a graph where each node represents a segment and nodes are adjacent if the segments share a non-empty interior intersection
	std::vector<std::vector<int>> adjacencyMap(InputSegments.size());
	for (int n = 0; n < (int) InputSegments.size(); n++) {
		for (int m = n + 1; m < (int) InputSegments.size(); m++) {
			if (LineSegment::HasInteriorOverlap(InputSegments[n], InputSegments[m])) {
				adjacencyMap[n].push_back(m);
				adjacencyMap[m].push_back(n);
			}
		}
	}

	//Break the segments into clusters where there are no non-vertex intersections between segments in different clusters
	std::vector<std::unordered_set<int>> clusters;
	std::unordered_set<int> availableNodes;
	for (int n = 0; n < (int) InputSegments.size(); n++)
		availableNodes.insert(n);
	while (! availableNodes.empty()) {
		int seedNode = *(availableNodes.begin());
		clusters.emplace_back();
		clusters.back().insert(seedNode);
		availableNodes.erase(seedNode);
		while (true) {
			std::unordered_set<int> newNodes;
			for (int n : clusters.back()) {
				for (int m : adjacencyMap[n]) {
					if (clusters.back().count(m) == 0U)
						newNodes.insert(m);
				}
			}
			if (newNodes.empty())
				break;
			else {
				clusters.back().insert(newNodes.begin(), newNodes.end());
				for (int index : newNodes)
					availableNodes.erase(index);
			}
		}
	}

	//Now, within each cluster, iteratively sanitize pairs until there are no more internal overlaps. When done, copy the segments of
	//each sanitized cluster to a single vector of output segments.
	std::Evector<LineSegment> outputSegments;
	outputSegments.reserve(InputSegments.size());
	for (size_t clusterIndex = 0U; clusterIndex < clusters.size(); clusterIndex++) {
		//Sanitize cluster "clusterIndex"
		std::Evector<LineSegment> clusterSegments;
		clusterSegments.reserve(clusters[clusterIndex].size());
		for (int n : clusters[clusterIndex])
			clusterSegments.push_back(InputSegments[n]);
		if (clusterSegments.size() > 1U) {
			int numPasses = 0;
			for (; numPasses < 100; numPasses++) {
				std::Evector<LineSegment> newSegments;
				bool actionPerformed = false;
				for (size_t indexA = 0; indexA < clusterSegments.size(); indexA++) {
					for (size_t indexB = indexA + 1U; indexB < clusterSegments.size(); indexB++) {
						if (LineSegment::SanitizeSegments(clusterSegments[indexA], clusterSegments[indexB], newSegments)) {
							/*std::cerr << "Sanitization action performed.\r\n";
							std::cerr << "Original segments:\r\n";
							std::cerr << clusterSegments[indexA] << "\r\n";
							std::cerr << clusterSegments[indexB] << "\r\n";
							std::cerr << "New segments:\r\n";
							for (auto const & seg : newSegments)
								std::cerr << seg << "\r\n";
							std::cerr << "\r\n";*/

							//Replace original segments with new segments
							if (newSegments.empty()) {
								clusterSegments.erase(clusterSegments.begin() + indexB); //Erase element in last position first
								clusterSegments.erase(clusterSegments.begin() + indexA); //Erase element in first position second
							}
							else if (newSegments.size() == 1U) {
								clusterSegments[indexA] = newSegments[0];
								clusterSegments.erase(clusterSegments.begin() + indexB);
							}
							else {
								clusterSegments[indexA] = newSegments[0];
								clusterSegments[indexB] = newSegments[1];
								for (size_t n = 2U; n < newSegments.size(); n++)
									clusterSegments.push_back(newSegments[n]);
							}
							actionPerformed = true;
							break;
						}
					}
					if (actionPerformed)
						break;
				}

				//Stopping condition is going through a full loop without finding interior overlaps/intersections
				if (! actionPerformed)
					break;
			}
			if (numPasses >= 100) {
				std::cerr << "Warning in SanitizeCollectionOfSegments(): Excessive number of actions needed to sanitize cluster ";
				std::cerr << "of line segments. Possible oscillatory behavior in pairwise segment sanitization.";
			}
		}
		outputSegments.insert(outputSegments.end(), clusterSegments.begin(), clusterSegments.end());
	}

	return outputSegments;
}

// ************************************************************************************************************************************************
// ********************************************************   Internal Function Definitions   *****************************************************
// ************************************************************************************************************************************************

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

//Return true if the given 3 points are co-linear and false otherwise
//Note: It appears that we aren't using this right now... maybe keep it around though since we might want it later.
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

//Returns >0 if P2 is left of the infinite line from P0 to P1
//        =0 if P2 is on the line
//        <0 if P2 is right of the line
static double PointIsInsidePolygonHelper_isLeft(Eigen::Vector2d const & P0, Eigen::Vector2d const & P1, Eigen::Vector2d const & P2) {
    return ((P1.x() - P0.x()) * (P2.y() - P0.y()) - (P2.x() -  P0.x()) * (P1.y() - P0.y()));
}

//Winding number test for a point in a polygon
//Input:   P = a point,
//         V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//Output:  wn = the winding number (=0 if and only if P is outside)
static int PointIsInsidePolygonHelper_wn_PnPoly(Eigen::Vector2d const & P, std::Evector<Eigen::Vector2d> const & V) {
    int wn = 0; //The  winding number counter

    // loop through all edges of the polygon
    for (size_t i = 0; i < V.size() - 1U; i++) {
        //Edge from V[i]  to V[i+1]
        if (V[i].y() <= P.y()) {       // Start y <= P.y
            if (V[i+1].y() > P.y())   // An upward crossing
                //If P is left of edge, we have  a valid up intersect
                if (PointIsInsidePolygonHelper_isLeft(V[i], V[i+1], P) > 0)
                    ++wn;
        }
        else {                         // Start y > P.y (no test needed)
            if (V[i+1].y() <= P.y())  // A downward crossing
                //If P right of edge, we have  a valid down intersect
                if (PointIsInsidePolygonHelper_isLeft(V[i], V[i+1], P) < 0)
                    --wn;
        }
    }
    //std::cerr << "Winding Number: " << wn << "\r\n";
    return wn;
}

void SimplePolygon::CustomSanitize(std::Evector<LineSegment> segments) {
    //Build a collection of line segments that form the full polygon and break them at intersections so the resulting segments can only intersect at nodes.
    //segments = BreakAtIntersections(segments);

    //Note that breaking line segments happens exactly (the new interior node is exactly the same in the resulting segments).
    //Also, the segments extracted from the polygon exactly cover the boundary (nodes that are supposed to be the same are exacty the same).
    //The only error introduced is round-off error in computing the placement of interior nodes when segments intersect.

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
            m_vertices.clear();
            return;
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
    m_vertices.clear();
    m_vertices.reserve(trajectory.size());
    for (int index : trajectory)
        m_vertices.push_back(NodeLocations[index]);
}


