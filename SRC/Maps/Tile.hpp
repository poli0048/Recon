#pragma once

//System Includes
#include <cstdint>
#include <vector>
#include <cmath>
#include <sstream>
#include <random>
#include <iostream>
#include <tuple>

//External Includes
#include "../../../handycpp/Handy.hpp"

namespace Maps {
	std::string assembleHERE_Tile_URL(int Row, int Col, int ZoomLevel, int Size, bool Hybrid);

	struct Tile {
		//The distance represented by one pixel (S) is given by
		//	S=C*cos(y)/2^(z+8)
		//where...
		//	C is the (equatorial) circumference of the Earth
		//	z is the zoom level
		//	y is the latitude of where you're interested in the scale.
		//	Make sure your calculator is in degrees mode, unless you want 
		// to express latitude in radians for some reason. C should be 
		// expressed in whatever scale unit you're interested in (miles,
		// meters, feet, smoots, whatever). Since the earth is actually 
		// ellipsoidal, there will be a slight error in this calculation. 
		// But it's very slight. (0.3% maximum error)

		int32_t Zoom = 0;
		int32_t Xi = 0;
		int32_t Yi = 0;

		Tile()             = default;
		Tile(Tile const &) = default;
		~Tile()            = default;
		Tile& operator=(Tile const & other) = default;

		Tile(int xi, int yi, int zoom)
			: Zoom(zoom),
			  Xi(xi),
			  Yi(yi)
		{ }

		std::string ToString() const {
			std::stringstream s;
			s << "Tile__Z_" << Zoom << "__X_" << Xi << "__Y_" << Yi;
			return s.str();
		}
		
		//Return the tile one zoom level lower covering this tile (will return negative zoom level if this has zoom 0)
		Tile GetParentTile() { return Tile(Xi/2, Yi/2, Zoom-1); }
		
		//Return the tiles one zoom level higher covering this tile
		std::vector<Tile> GetChildTiles() {
			std::vector<Tile> tiles;
			tiles.push_back(Tile(Xi*2,     Yi*2,     Zoom+1));
			tiles.push_back(Tile(Xi*2,     Yi*2 + 1, Zoom+1));
			tiles.push_back(Tile(Xi*2 + 1, Yi*2,     Zoom+1));
			tiles.push_back(Tile(Xi*2 + 1, Yi*2 + 1, Zoom+1));
			return tiles;
		}
	};

	inline bool operator==(Tile const & lhs, Tile const & rhs) {
		return 
			lhs.Zoom == rhs.Zoom && 
			lhs.Xi   == rhs.Xi   &&
			lhs.Yi   == rhs.Yi;
	}

	inline bool operator<(Tile const & lhs, Tile const & rhs) {
		if (lhs.Zoom < rhs.Zoom)  return true;
		if (lhs.Zoom > rhs.Zoom)  return false;

		if (lhs.Xi < rhs.Xi)  return true;
		if (lhs.Xi > rhs.Xi)  return false;

		if (lhs.Yi < rhs.Yi)  return true;
		if (lhs.Yi > rhs.Yi)  return false;

		return false;
	}
}

MAKE_HASHABLE(Maps::Tile, t.Zoom, t.Xi, t.Yi)


