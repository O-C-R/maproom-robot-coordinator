//
//  Map.h
//  maproom-robot
//
//  Created by Christopher Anderson on 2/20/17.
//
//

#ifndef Map_h
#define Map_h

#include "ofMain.h"

enum PathType {
	PATH_ROAD,
	PATH_BUILDING
};

typedef struct MapPathSegment {
	ofVec2f start, end;
} MapPathSegment;

typedef struct MapPath {
	int id;
	PathType type;
	vector<MapPathSegment> segments;
} MapPath;

class Map {
public:
	Map(float widthM, float heightM);
	void loadSVG(const string filename);

	bool hasNextPath();
	MapPath* nextPath();

private:
	float widthM, heightM;
	vector<MapPath> paths;
};

#endif
