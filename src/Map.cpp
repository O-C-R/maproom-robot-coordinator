//
//  Map.cpp
//  maproom-robot
//
//  Created by Christopher Anderson on 2/20/17.
//
//

#include "Map.h"

Map::Map(float width, float height): widthM(width), heightM(height) {}

void Map::loadSVG(const string filename) {
	// TODO

	// Load svg
	// Parse paths into MapPaths
	// Add to array
	// Rescale to size of map
}

bool hasNextPath() {
	// TODO

	return false;
}

MapPath* nextPath() {
	// TODO

	return NULL;
}
