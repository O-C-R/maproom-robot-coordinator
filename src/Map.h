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

typedef struct pathSegment {
	ofVec2f start, end;
	ofVec2f prescaleStart, prescaleEnd;
} pathSegment;

typedef struct MapPath {
    bool claimed, drawn;

    string type;
	pathSegment segment;
} MapPath;

class Map {
public:
	Map(float widthM, float heightM, float offsetX, float offsetY);
	void loadMap(const string filename);
	void rescaleMap(float widthM, float heightM, float offsetX, float offsetY);
    
	MapPath* nextPath(const ofVec2f &initial);
    
    ofxXmlSettings currentMap;
    
    map<string, list<MapPath>> mapPathStore;
    vector<string> pathTypes;
    
    void storePath(string type, float startX, float startY, float destX, float destY);
    void clearStore();
    
private:
	float widthM, heightM, offsetX, offsetY;
	float scaleX, scaleY;
	vector<MapPath> paths;

	ofVec2f svgExtentMin, svgExtentMax;
};

#endif
