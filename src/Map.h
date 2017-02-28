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
    
	MapPath* nextPath(const ofVec2f &initial);
    
    ofxXmlSettings currentMap;
    
    map<string, list<MapPath>> mapPathStore;
    vector<string> pathTypes;
    
    void storePath(string type, float startX, float startY, float destX, float destY);
    void clearStore();
    
private:
	float widthM, heightM, offsetX, offsetY, scaleX, scaleY;
	vector<MapPath> paths;
};

#endif
