//This module provides some basic geometry utilities needed for working with Polygons
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <iostream>
#include <vector>

//Cereal Includes
#include "cereal/types/vector.hpp"
#include "cereal/types/string.hpp"
#include "cereal/archives/json.hpp"

//Project Includes
#include "EigenAliases.h"

//Class Declarations
class LineSegment;
class Triangle;
class SimplePolygon;
class Polygon;
class PolygonCollection;

//Cereal-based serialization for Eigen matrix types (Binary serialization)
namespace cereal {
	template <class Archive, class Derived> inline
		typename std::enable_if<traits::is_output_serializable<BinaryData<typename Derived::Scalar>, Archive>::value, void>::type
		save(Archive & ar, Eigen::PlainObjectBase<Derived> const & m) {
			typedef Eigen::PlainObjectBase<Derived> ArrT;
			if(ArrT::RowsAtCompileTime==Eigen::Dynamic) ar(m.rows());
			if(ArrT::ColsAtCompileTime==Eigen::Dynamic) ar(m.cols());
			ar(binary_data(m.data(),m.size()*sizeof(typename Derived::Scalar)));
		}

	template <class Archive, class Derived> inline
		typename std::enable_if<traits::is_input_serializable<BinaryData<typename Derived::Scalar>, Archive>::value, void>::type
		load(Archive & ar, Eigen::PlainObjectBase<Derived> & m) {
			typedef Eigen::PlainObjectBase<Derived> ArrT;
			Eigen::Index rows=ArrT::RowsAtCompileTime, cols=ArrT::ColsAtCompileTime;
			if(rows==Eigen::Dynamic) ar(rows);
			if(cols==Eigen::Dynamic) ar(cols);
			m.resize(rows,cols);
			ar(binary_data(m.data(),static_cast<std::size_t>(rows*cols*sizeof(typename Derived::Scalar))));
		}
	}

//Class for representing a line segment in 2D. This is primarily used internally, but it could be generally useful so we make it public.
//A LineSegment can be degenerate (endpoints are nearly identical) or non-degenerate. All methods should work in both cases. The only way
//a LineSegment can have an invalid state is if an endpoint has non-finite coordinates.
class LineSegment {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//The only fields are the two endpoints of the line segment
	Eigen::Vector2d m_endpoint1;
	Eigen::Vector2d m_endpoint2;

	LineSegment() : m_endpoint1(0.0, 0.0), m_endpoint2(0.0, 0.0) { }
	LineSegment(Eigen::Vector2d const & Endpoint1, Eigen::Vector2d const & Endpoint2) : m_endpoint1(Endpoint1), m_endpoint2(Endpoint2) { }
	~LineSegment() = default;

	//Test to see if the line segment is degenerate (endpoints are nearly equal)
	bool IsDegenerate(void) const;

	//Test to see if a point is approximately on the line segment
	bool ContainsPoint(Eigen::Vector2d const & Point) const;

	//Get segment length
	double GetLength(void) const;

	//Get the point on the line segment closest to the provided point
	Eigen::Vector2d ProjectPoint(Eigen::Vector2d const & Point) const;

	//Returns the distance from the given point to the nearset endpoint of the line segment
	double GetDistanceToNearestEndpoint(Eigen::Vector2d const & Point) const;

	//Test for intersection of 2 line segments and compute the intersection. Returns true if they intersect and false otherwise.
	bool ComputeIntersection(LineSegment const & Other, Eigen::Vector2d & Intersection) const;

	//Same as ComputeIntersection, but only returns true if the intersection is not an endpoint for at least one of the segments.
	bool ComputeInteriorIntersection(LineSegment const & Other, Eigen::Vector2d & Intersection, bool & IsInteriorThis, bool & IsInteriorOther) const;

	static bool SanitizeSegments(LineSegment const & A, LineSegment const & B, std::Evector<LineSegment> & Dst);

	static bool HasInteriorOverlap(LineSegment const & A, LineSegment const & B);

	static bool AreEquivalent(LineSegment const & A, LineSegment const & B);

	//Tell Cereal which members to serialize (must be public)
	template<class Archive> void serialize(Archive & archive) { archive(m_endpoint1, m_endpoint2); }

	//Clip line segment to polygon - this returns all sub-segments that lie inside the polygon
	//std::Evector<LineSegment> ClipToPolygon(SimplePolygon const & Poly) const;
	//std::Evector<LineSegment> ClipToPolygon(Polygon const & Poly) const;
};

//Stream Operator Definition - used to print contents in human-readable form
inline std::ostream & operator<<(std::ostream & Str, LineSegment const & v) {
	Str << "(" << v.m_endpoint1(0) << ", " << v.m_endpoint1(1) << ") ---- (" << v.m_endpoint2(0) << ", " << v.m_endpoint2(1) << ")";
	return Str;
}

//The Triangle class isn't meant to be used as a workhorse since it is geometrically a special case of a simple polygon.
//It is really just a super simple structure used for triangulation.
class Triangle {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::Vector2d m_pointA;
	Eigen::Vector2d m_pointB;
	Eigen::Vector2d m_pointC;

	//Tell Cereal which members to serialize (must be public)
	template<class Archive> void serialize(Archive & archive) { archive(m_pointA, m_pointB, m_pointC); }
};

//Stream Operator Definition - used to print contents in human-readable form
inline std::ostream & operator<<(std::ostream & Str, Triangle const & v) {
	Str << "Triangle: ";
	Str << "(" << v.m_pointA(0) << ", " << v.m_pointA(1) << ") ---- ";
	Str << "(" << v.m_pointB(0) << ", " << v.m_pointB(1) << ") ---- ";
	Str << "(" << v.m_pointC(0) << ", " << v.m_pointC(1) << ")";
	return Str;
}

//Class for representing a simple polygon. A simple polygon is defined by a single piecewise-linear, closed curve in the plane defining it's boundary.
//A simple polygon has no holes and it's boundary may only touch itself at isolated points. It does not overlap itself, the boundary does not cross itself,
//and stretches of boundary with positive length are not used more than once. A simple polygon may be convex or non-convex. If created with a sequence of
//points that do not define a simple polygon (e.g. edge segments intersect) the outline of the provided non-simple polygon will be used to define the object.
//This will, in many instances at least, yield reasonable and expected behavior since often self-intersections in polygons are accidents rather than deliberate.
//We don't attempt to support set operations on simple polygons (like union, intersection, and difference) because the results of these operations on simple
//poygons are not, in general, simple polygons themselves.
//Not every vector of points represents a valid simple polygon so if we exposed the vertices directly it would be possible to put an object into an
//invalid state. For simplicity we keep the vertices private and only allow them to be accessed through a const reference. The vertices can be set using
//SetBoundary(), which sanitizes the object and ensures it is in a valid state. Thus, all methods can assume the object is valid and should always work.
class SimplePolygon {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SimplePolygon() = default;
	SimplePolygon(std::Evector<Eigen::Vector2d> const & Vertices) { SetBoundary(Vertices); }
	~SimplePolygon() = default;

	void Clear(void) { m_vertices.clear(); }

	//Set the boundary of the polygon - this will sanitize the input if they don't define a simple polygon.
	void SetBoundary(std::Evector<Eigen::Vector2d> const & Vertices) {
	    m_vertices = Vertices;
	    Sanitize();
	}

	std::Evector<Eigen::Vector2d> const & GetVertices(void) const { return m_vertices; }
	size_t NumVertices(void) const { return m_vertices.size(); }
	bool Empty(void) const { return m_vertices.empty(); }

	//Return a vector of the line segments that make up the polygon
	std::Evector<LineSegment> GetLineSegments(void) const;

	//Get the point inside or on the edge of the polygon nearest to the provided point
	Eigen::Vector2d ProjectPoint(Eigen::Vector2d const & Point) const;

	//Get the point on the boundary of the polygon that is nearest to the provided point (and optionally get the closest vertex)
	Eigen::Vector2d ProjectPointToBoundary(Eigen::Vector2d const & Point) const;
	Eigen::Vector2d ProjectPointToBoundary(Eigen::Vector2d const & Point, size_t & ClosestVertexIndex) const;

	//Test to see if polygon contains a point in its interior (not stable on the boundary)
	bool ContainsPoint(Eigen::Vector2d const & Point) const;

	//Get the X and Y bounds of the polygon (An Axis-Aligned Bounding Box). Returned as a vector: (XMin, XMax, YMin, YMax).
	Eigen::Vector4d GetAABB(void) const;

	//Compute the area of the simple polygon (the area enclosed by its boundary)
	double GetArea(void) const;

	//Break edges of this simple polygon and another simple polygon into two pieces whenever they are found to intersect an edge of the other.
	//This can modify both polygons. On completion the two polygons will have boundaries that only cross at vertices. It is still possible, however,
	//for them to have overlapping boundary segments.
	void FragmentIntersections(SimplePolygon & OtherPoly);

	//Returns true if this polygon completely contains the given "other" polygon in its interior. Not stable if their boundaries touch.
	bool Contains(SimplePolygon const & OtherPoly) const;

	//Returns true if this polygon intersects with the given "other" polygon. Not stable if the intersection has 0 area.
	bool IntersectsWith(SimplePolygon const & OtherPoly) const;

	//Find the vector that minimizes the size of the orthogonal projection of the polygon onto the line defined by the vector
	Eigen::Vector2d FindShortestAxis(void) const;

	//Find the vector that maximizes the size of the orthogonal projection of the polygon onto the line defined by the vector
	Eigen::Vector2d FindLongestAxis(void) const;

	//Cut simple polygon with a half plane. Keep the portion satisfying X dot V <= P. This may result in 0 simple polygons
	//or arbitrarily many, depending on the geometry of the polygon and the chosen half plane. V must be a unit vector.
	std::Evector<SimplePolygon> IntersectWithHalfPlane(Eigen::Vector2d const & VArg, double P) const;

	//Cut a simple polygon along the line containing points A and B. Return all resulting pieces as simple polygons. This can
	//result in arbitrarily many pieces, depending on the object's geometry and the cut location
	std::Evector<SimplePolygon> CutAlongLine(Eigen::Vector2d const & A, Eigen::Vector2d const & B) const;

	//Populate a vector of triangles exactly covering the same area as the polygon collection. Uses Earcut algorithm.
	//This function does not clear "Triangles" - it appends to it. This is so we can ask multiple objects to add their triangles to the same vector
	void Triangulate(std::Evector<Triangle> & Triangles) const;

	//Clip the infinite line X dot V = P to the simple polygon. Return all interior segments. V must be a unit vector.
	std::Evector<LineSegment> ClipLine(Eigen::Vector2d const & VArg, double P) const;

	//Tell Cereal which members to serialize (must be public)
	template<class Archive> void serialize(Archive & archive) { archive(m_vertices); }

private:
	//Boundary vertices (final vertex implicitly connects to first). When traversed in order the polygon interior is to the left.
	//Private so that they cannot be externally modified to place the object into an invalid state. These should only be set by SetBoundary()
	std::Evector<Eigen::Vector2d> m_vertices;

	//If the current vertices do not represent a simple polygon, replace them with those defining a reasonable simple polygon
	//derived from the current object. We do this by tracing the outline of the non-simple polygonal object.
	void Sanitize(void);

	//Get the length of the projection of the polygon onto the line in the given direction (V must be a unit vector)
	double GetLengthOfProjection(Eigen::Vector2d const & V) const;

	//Find index of the first vertex that lies in the half plane (-1 if all vertices lie outside half plane)
	//The half plane is defined by X dot V <= P. V should be a unit vector.
	int FindFirstVertexInHalfPlane(Eigen::Vector2d const & V, double P) const;

	//Get a copy of the vertices, cycle-shifted (so they represent the same closed curve in the plane) so the given index comes first
	std::Evector<Eigen::Vector2d> CycleShiftVertices(size_t StartIndex) const;

	friend Polygon;
};

//Stream Operator Definition - used to print contents in human-readable form
inline std::ostream & operator<<(std::ostream & Str, SimplePolygon const & v) {
	Str << "Polygon vertices (in order - interior on left):\r\n";
	std::Evector<Eigen::Vector2d> const & vertices(v.GetVertices());
	for (auto const & node : vertices)
		Str << "(" << node(0) << ", " << node(1) << ")\r\n";
	return Str;
}

//Class for representing a more general polygon. For us, a polygon consists of a single outer boundary, which must be a simple polygon, and an arbitrary
//number of interior, non-overlapping holes, which are also simple polygons. Thus, our polygons can have as many holes as we like but self-intersections
//are still not allowed. As with SimplePolygon, we do not attempt to support set operations like union, difference, and intersection because the results of
//those operations on Polygons (as defined by this class) are not themselves polygons (in general).
//Unlike the SimplePolygon, it is much more complicated to try and sanitize arbitrary inputs to create a valid Polygon object from them. Thus,
//we allow for the possibility that a Polygon object can exist in either a valid or invalid state. Most methods assume the object is valid and should
//not be trusted if called on an invalid object. The IsValid() method is provided to check validity. This is not an especially fast method though, so
//only check after changing the object.
class Polygon {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SimplePolygon m_boundary;
	std::Evector<SimplePolygon> m_holes; //Holes may not overlap and they must be completely within the outer boundary

	Polygon() = default;
	Polygon(SimplePolygon const & Boundary) : m_boundary(Boundary) { }
	~Polygon() = default;

	//Simple query and sanitization methods
	void Clear(void) { m_boundary.Clear(); m_holes.clear(); }
	bool IsTrivial(void) const { return ((m_boundary.NumVertices() <= 2U) || (m_boundary.GetArea() <= 0.0)); }
	void RemoveTrivialHoles(void); //Remove holes with no area
	void RemoveEmptyHoles(void);   //Remove holes with no vertices

	//Test to see if polygon contains a point
	bool ContainsPoint(Eigen::Vector2d const & Point) const;

	//Get the X and Y bounds of the polygon (An Axis-Aligned Bounding Box). Returned as a vector: (XMin, XMax, YMin, YMax).
	Eigen::Vector4d GetAABB(void) const;

	//Get the area of a *valid* polygon.
	double GetArea(void) const;

	//Returns true if the contents of the polygon meet the geometry requirements - the holes must be completely within the interior and non-overlapping.
	bool IsValid(void) const;

	//Returns true if this polygon intersects with the given "other" polygon. Not stable if the intersection has 0 area.
	bool IntersectsWith(Polygon const & OtherPoly) const;

	//Find the vector that minimizes the size of the orthogonal projection of the polygon onto the line defined by the vector
	Eigen::Vector2d FindShortestAxis(void) const;
	
	//Find the vector that maximizes the size of the orthogonal projection of the polygon onto the line defined by the vector
	Eigen::Vector2d FindLongestAxis(void) const;

	//Cut polygon with a half plane. Keep the portion satisfying X dot V <= P. This may result in 0 polygons
	//or arbitrarily many, depending on the geometry of the polygon and the chosen half plane. V must be a unit vector.
	std::Evector<Polygon> IntersectWithHalfPlane(Eigen::Vector2d const & VArg, double P) const;

	//Cut polygon along the line containing points A and B. Return all resulting pieces as polygons. This can
	//result in arbitrarily many pieces, depending on the object's geometry and the cut location
	std::Evector<Polygon> CutAlongLine(Eigen::Vector2d const & A, Eigen::Vector2d const & B) const;

	//TODO: We could probably implement Contains() too if it would be useful... maybe later.

	//Populate a vector of triangles exactly covering the same area as the polygon collection. Assumes object is valid. Uses Earcut algorithm.
	//This function does not clear "Triangles" - it appends to it. This is so we can ask multiple objects to add their triangles to the same vector
	void Triangulate(std::Evector<Triangle> & Triangles) const;

	//Clip the infinite line X dot V = P to the polygon. Return all interior segments. V must be a unit vector.
	std::Evector<LineSegment> ClipLine(Eigen::Vector2d const & VArg, double P) const;

	//Tell Cereal which members to serialize (must be public)
	template<class Archive> void serialize(Archive & archive) { archive(m_boundary, m_holes); }
};

//Stream Operator Definition - used to print contents in human-readable form
inline std::ostream & operator<<(std::ostream & Str, Polygon const & v) {
	Str << "Polygon object is " << (v.IsValid() ? "valid" : "invalid") << ". Contains " << v.m_holes.size() << " holes.\r\n";
	Str << "Outer Boundary:\r\n" << v.m_boundary << "\r\n";
	for (SimplePolygon const & hole : v.m_holes)
		Str << "Hole:\r\n" << hole << "\r\n";
	return Str;
}

//Class for representing a collection of polygons. A polygon collection can exist in either a valid or invalid state.
//To be valid, each component must be valid, and no two components should overlap. The set represented by a polygon collection
//is the union of the sets represented by the disjoint components.
class PolygonCollection {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	std::Evector<Polygon> m_components;

	PolygonCollection() = default;
	PolygonCollection(Polygon const & Poly) : m_components(1, Poly) { }
	~PolygonCollection() = default;

	//Simple query and sanitization methods
	void Clear(void) { m_components.clear(); }
	void RemoveTrivialHoles(void);      //Remove trivial holes from all components
	void RemoveEmptyHoles(void);        //Remove empty holes from all components
	void RemoveTrivialComponents(void); //Remove trivial components
	void RemoveEmptyComponents(void);   //Remove components with empty boundaries

	//Test to see if polygon collection contains a point
	bool ContainsPoint(Eigen::Vector2d const & Point) const;

	//Get the X and Y bounds of the polygon collection (An Axis-Aligned Bounding Box). Returned as a vector: (XMin, XMax, YMin, YMax).
	Eigen::Vector4d GetAABB(void) const;

	//Get the area of a *valid* polygon collection
	double GetArea(void) const;

	//Returns true if the contents of the object meet the geometry requirements - all components are valid and disjoint from one another.
	bool IsValid(void) const;

	//Populate a vector of triangles exactly covering the same area as the polygon collection. Assumes object is valid. Uses Earcut algorithm
	void Triangulate(std::Evector<Triangle> & Triangles) const;

	//Return some vertex used in the boundary of some component of the poly collection (primarily used for fallback in some algorithms)
	Eigen::Vector2d GetSomeBoundaryVertex(void) const;

	//Tell Cereal which members to serialize (must be public)
	template<class Archive> void serialize(Archive & archive) { archive(m_components); }
};

//Stream Operator Definition - used to print contents in human-readable form
inline std::ostream & operator<<(std::ostream & Str, PolygonCollection const & v) {
	Str << "PolygonCollection object is " << (v.IsValid() ? "valid" : "invalid") << ". Contains " << v.m_components.size() << " components.\r\n";
	for (Polygon const & comp : v.m_components)
		Str << "Component:\r\n" << comp << "\r\n";
	return Str;
}

//Public Utility functions
//static bool PointsAreColinear(Eigen::Vector2d const & p1, Eigen::Vector2d const & p2, Eigen::Vector2d const & p3);
std::Evector<LineSegment> SanitizeCollectionOfSegments(std::Evector<LineSegment> const & InputSegments);

