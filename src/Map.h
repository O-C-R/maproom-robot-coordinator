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
} pathSegment;

typedef struct MapPath {
	int id;
	pathSegment segment;
} MapPath;

class Map {
public:
	Map(float widthM, float heightM, float offsetX, float offsetY);
	void loadMap(const string filename);
    
	bool checkNextPath();
    bool checkNextPath(int pathType);
    
	MapPath getNextPath();
    
    ofxXmlSettings currentMap;
    
    map<int, list<pathSegment>> mapPathStore;
    
    void storePath(string type, float startX, float startY, float destX, float destY);
    void clearStore();
    
private:
	float widthM, heightM, offsetX, offsetY, scaleX, scaleY;
    MapPath nextPath;
	vector<MapPath> paths;
};

#endif
