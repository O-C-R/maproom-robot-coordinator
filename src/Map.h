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
	FIRST,
    major_road,
	minor_road,
    highway,
    aerialway,
    rail,
    path,
    ferry,
    etc,
    LAST
};

typedef struct MapPathSegment {
	ofVec2f start, end;
} MapPathSegment;

typedef struct MapPath {
	int id;
	MapPathSegment segment;
} MapPath;

class Map {
public:
	Map(float widthM, float heightM, float offsetX, float offsetY);
	void loadMap(const string filename);
    
	bool checkNextPath();
    bool checkNextPath(int pathType);
    
	MapPath getNextPath();
    
    ofxXmlSettings currentMap;
    
    map<int, list<MapPathSegment>> mapPathStore;
    
    void storePath(string type, float startX, float startY, float destX, float destY);
    
private:
	float widthM, heightM, offsetX, offsetY, scaleX, scaleY;
    MapPath nextPath;
	vector<MapPath> paths;
};

#endif
