//This module provides geometry utilities needed for working with Polygons
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//System Includes
#include <tuple>
#include <limits>

//External Includes
#include "../../eigen/Eigen/LU"

//Project Includes
#include "Polygon.hpp"
#include "Earcut.hpp"

#include "Maps/MapUtils.hpp"
#include<Eigen/Geometry>

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

//Returns the distance to the nearset end of the line segment
double LineSegment::GetDistanceToNearestEndpoint(Eigen::Vector2d const & Point) const {
    return std::min((Point - m_endpoint1).norm(), (Point - m_endpoint2).norm());
}

//Compute the intersection of two line segments. Returns true if they intersect and false otherwise.
//If this function returns false, the value of Intersection could be anything - don't use it.
bool LineSegment::ComputeIntersection(LineSegment const & Other, Eigen::Vector2d & Intersection) const {
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
bool Triangle::isSameTriangle(Triangle const & otherTriangle){
    if(this->m_pointA == otherTriangle.m_pointA && this->m_pointB == otherTriangle.m_pointB && this->m_pointC == otherTriangle.m_pointC ) {
        return true;
    }
    return false;
}

//Get the area (m^2) of a triangle whose coordiantes are in NM
double Triangle::GetArea(){
    //Alt is unnecessary and will be removed
    //Each point of the triangle converted to LatLon
    Eigen::Vector2d a_latlon;
    Eigen::Vector2d b_latlon;
    Eigen::Vector2d c_latlon;
    a_latlon = NMToLatLon(this->m_pointA);
    b_latlon = NMToLatLon(this->m_pointB);
    c_latlon = NMToLatLon(this->m_pointC);

    //Constants for ABC to be converted to ECEF
    Eigen::Vector3d A_XYZ; A_XYZ << a_latlon(0), a_latlon(1), 0.0;
    Eigen::Vector3d B_XYZ; B_XYZ << b_latlon(0), b_latlon(1), 0.0;
    Eigen::Vector3d C_XYZ; C_XYZ << c_latlon(0), c_latlon(1), 0.0;

    //ECEF of ABC
    Eigen::Vector3d A_ECEF = LLA2ECEF(A_XYZ);
    Eigen::Vector3d B_ECEF = LLA2ECEF(B_XYZ);
    Eigen::Vector3d C_ECEF = LLA2ECEF(C_XYZ);

    //Vector between two points in LatLon
    Eigen::Vector3d ab_vector = A_ECEF - B_ECEF;
    Eigen::Vector3d ac_vector = B_ECEF - C_ECEF;
    return 0.5 * (ac_vector.cross(ab_vector)).norm();
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

    for (int i=0; i<segmentA.size(); i++){
        for (int j=0; j<pointsB.size(); j++){
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
bool SimplePolygon::CheckAdjacency (SimplePolygon const & otherPolygon){
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

//Test to see if polygon contains a point in its interior (not especially stable on the boundary)
bool SimplePolygon::ContainsPoint(Eigen::Vector2d const & Point) const {
    // Test to see if a point is inside a polygon. Based on code from: http://geomalgorithms.com/a03-_inclusion.html. Copyright info:
    // Copyright 2000 softSurfer, 2012 Dan Sunday. This code may be freely used and modified for any purpose providing that this
    // copyright notice is included with it. SoftSurfer makes no warranty for this code, and cannot be held liable for any real or
    // imagined damage resulting from its use. Users of this code must verify correctness for their application.
    if (m_vertices.size() < 3U)
        return false;

    //Append first point to end and peform winding number test
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

//Trace the outline of the polygonal object represented by m_vertices. Compute new vertices from the outline that traces the object and keeps
//it's interior to the left. If m_vertices already defined a valid simple polygon and kept the interior to the left this should (I hope) leave
//the object essentially unchanged, except it might add vertices if the polygon touched itself at an isolated point, but the shape should be the same.
void SimplePolygon::Sanitize(void) {
    //If the polygon is degenerate, there is no "good" answer here. Just leave it alone.
    if (m_vertices.size() < 3U)
        return;

    //Build a collection of line segments that form the full polygon and break them at intersections so the resulting segments can only intersect at nodes.
    std::Evector<LineSegment> segments = GetLineSegments();
    segments = BreakAtIntersections(segments);

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
    //We have a non-empty intersection if the boundery polygons intersect (neglecting holes) and it is not the case that the boundary of
    //one polygon is contained within a hole of the other polygon. Keep in mind that a Polygon, as we have defined it,
    //cannot have separate, disjoint components - this simplifies the logic here. If the boundary polygons don't intersect
    //as simple polygons then clearly there is no intersection. Thus, we restrict our focus to the case that they do
    //intersect (as simple polygons... that is, ignoring holes).
    //
    //Claim: It is not the case that one poly is comletely in a hole of the other => Non-empty intersection
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

//Take a collection of line segments and break all intersecting pairs of segments into fragments. The result is a collection of line segments covering
//the same path as the original collection, but with segments that (usually) only intersect at endpoints and not along edges. We say usually because there
//may be some odd behavior if parallel segments overlap over a length > 0.
//Tested over some relatively simple (but not completely trivial) test cases and this seems to be working correctly (1/29/2021)
std::Evector<LineSegment> BreakAtIntersections(std::Evector<LineSegment> const & InputSegments) {
    std::Evector<LineSegment> Segments = InputSegments;
    std::Evector<LineSegment> outputSegments;
    outputSegments.reserve(2U*InputSegments.size());
    while (! Segments.empty()) {
        //Create a vector containing a single segment and remove it from the Segments vector
        std::Evector<LineSegment> SegmentsA(1U, Segments.back());
        Segments.pop_back();

        //Create a container to hold fragments of the remaining segments when split by the segment we removed
        std::Evector<LineSegment> SegmentsB;
        SegmentsB.reserve(2U*Segments.size());

        //Go through each remaining segment and see if it intersects any fragment of the removed segment. If not, we just put in SegmentsB.
        //If so, we fracture both segments.
        for (size_t n = 0U; n < Segments.size(); n++) {
            LineSegment segmentB = Segments[n];
            bool intersectionFound = false;
            for (size_t m = 0U; m < SegmentsA.size(); m++) {
                LineSegment segmentA = SegmentsA[m];
                Eigen::Vector2d intersection;
                bool IsInteriorA, IsInteriorB;
                if (segmentA.ComputeInteriorIntersection(segmentB, intersection, IsInteriorA, IsInteriorB)) {
                    //There is an interior intersection between segments A and B
                    if (IsInteriorA) {
                        //We need to fracture the A segment
                        SegmentsA[m] = LineSegment(segmentA.m_endpoint1, intersection);
                        SegmentsA.push_back(LineSegment(intersection, segmentA.m_endpoint2));
                    }

                    if (IsInteriorB) {
                        SegmentsB.push_back(LineSegment(segmentB.m_endpoint1, intersection));
                        SegmentsB.push_back(LineSegment(intersection, segmentB.m_endpoint2));
                    }
                    else
                        SegmentsB.push_back(segmentB);

                    //The segments in the A vector or fragments of a single segment. If we intersected with one of them
                    //we canot possibly intersect another, so there is no need to continue checking fragments.
                    intersectionFound = true;
                    break;
                }
            }
            if (! intersectionFound)
                SegmentsB.push_back(segmentB);
        }

        //At this point, all of our original segments are covered by fragments held in SegmentsA and SegmentsB. Furthermore, none of
        //the fragments in SegmentsA have interior intersections with any of fragments in either SegmentsA or SegmentsB.
        outputSegments.insert(outputSegments.end(), SegmentsA.begin(), SegmentsA.end());
        Segments.swap(SegmentsB);
    }

    return outputSegments;
}
//omqueeg

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


