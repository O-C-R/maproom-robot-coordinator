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
#include "Util.h"

typedef struct pathSegment {
	ofVec2f start, end;
	ofVec2f prescaleStart, prescaleEnd;
} pathSegment;

typedef struct MapPath {
	int id;
    bool claimed, drawn;

    string type;
	pathSegment segment;
} MapPath;

class Map {
public:
	Map(float widthM, float heightM, float offsetX, float offsetY, ofRectangle cropBox);
	void loadMap(const string filename);
	void rescaleMap(float widthM, float heightM, float offsetX, float offsetY);
    string getMostRecentMap(string path);
    
	MapPath* nextPath(const ofVec2f &initial, int robotId, float lastHeading, const set<string> &pathTypes);
    
    ofxXmlSettings currentMap;
    
    map<string, vector<MapPath>> mapPathStore;
    
    vector<string> pathTypes;
    map<string, bool> activePaths;
    
    void storePath(string type, float startX, float startY, float destX, float destY);
    void clearStore();
    void optimizePaths(float percent);
    
    void setPathActive(string path, bool active);
    int getPathCount();
    int getActivePathCount();
    int getDrawnPaths();
    int getPathCount(string type);
private:
	float widthM, heightM, offsetX, offsetY;
	float origOffsetX, origOffsetY;
	float scaleX, scaleY;
	vector<MapPath> paths;

	int storeCount, pathCount;

	ofVec2f svgExtentMin, svgExtentMax;
	ofRectangle cropBox;
};

#endif
