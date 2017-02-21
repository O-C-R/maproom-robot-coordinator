//
//  Map.h
//  maproom-robot
//
//  Created by Christopher Anderson on 2/20/17.
//
//

#ifndef Map_h
#define Map_h

#include "ofxXmlSettings.h"
#include "ofMain.h"

enum roadTypes {
	major_road,
	minor_road,
    highway,
    aerialway,
    rail,
    path,
    ferry,
    etc
};



typedef struct MapPathSegment {
	ofVec2f start, end;
} MapPathSegment;

typedef struct MapPath {
	int id;
	roadTypes type;
	vector<MapPathSegment> segments;
} MapPath;


class Map {
public:
	Map(float widthM, float heightM);
	void loadMap(const string filename);

	bool hasNextPath();
	MapPath* nextPath();
    
    ofxXmlSettings currentMap;
    
    map<int, vector<MapPathSegment>> mapPathStore;
    
    void storePath(string type, float startX, float startY, float destX, float destY);
    
private:
	float widthM, heightM, widthSVG, heightSVG;
	vector<MapPath> paths;
};

#endif
