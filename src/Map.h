//
//  Map.h
//  maproom-robot
//
//  Created by Christopher Anderson on 2/20/17.
//
//

#ifndef Map_h
#define Map_h
#include <regex>

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
    string getMostRecentMap(string path);
    
	MapPath* nextPath(const ofVec2f &initial, int robotId);
    
    ofxXmlSettings currentMap;
    
    map<string, vector<MapPath>> mapPathStore;
    vector<string> pathTypes;
    map<string, bool> activePaths;
    
    void storePath(string type, float startX, float startY, float destX, float destY);
    void clearStore();
    void optimizePaths();
    
    void setPathActive(string path, bool active);
    int getPathCount();
    int getActivePathCount();
    int getDrawnPaths();
    int getPathCount(string type);
    
    // string pathType, int robot_id
    map<string, int> pathAssignment;
private:
	float widthM, heightM, offsetX, offsetY, scaleX, scaleY;
	vector<MapPath> paths;
};

#endif
