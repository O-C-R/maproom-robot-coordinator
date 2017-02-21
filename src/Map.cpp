//
//  Map.cpp
//  maproom-robot
//
//  Created by Christopher Anderson on 2/20/17.
//
//

#include "Map.h"

Map::Map(float width, float height): widthM(width), heightM(height) {}

void Map::storePath(string type, float startX, float startY, float destX, float destY) {
    int lineType;
    // be wary of strings with types of the same first two letters
    if (type.substr(0,2) == "ma") {
        lineType = major_road;
    } else if (type.substr(0,2) == "mi") {
        lineType = minor_road;
    } else if (type.substr(0,2) == "hi") {
        lineType = highway;
    } else if (type.substr(0,2) == "ae") {
        lineType = aerialway;
    } else if (type.substr(0,2) == "ra") {
        lineType = rail;
    } else if (type.substr(0,2) == "pa") {
        lineType = path;
    } else if (type.substr(0,2) == "fe") {
        lineType = ferry;
    } else {
        lineType = etc;
    }
    
    
    MapPathSegment segment;
    segment.start = ofVec2f(startX, startY);
    segment.end = ofVec2f(destX, destY);
    mapPathStore[lineType].push_back(segment);
}


void Map::loadMap(const string filename) {
    currentMap.loadFile(filename);
    widthSVG = stoi(ofToString(currentMap.getAttribute("svg", "width", "")));
    heightSVG = stoi(ofToString(currentMap.getAttribute("svg", "height", "")));
    
    currentMap.pushTag("svg");
    int firstLevel = currentMap.getNumTags("g");
    for (int i = 0; i < firstLevel; i++) {
        cout << "first_level id: " << currentMap.getAttribute("g", "id", "", i) << endl;
        currentMap.pushTag("g", i);
        int secondLevel = currentMap.getNumTags("g");
        for (int j = 0; j < secondLevel; j++) {
            string lineType = ofToString(currentMap.getAttribute("g", "id", "", j));
            currentMap.pushTag("g", j);
            int thirdLevel = currentMap.getNumTags("path");
            for (int k = 0; k < thirdLevel; k++) {
                string path = ofToString(currentMap.getAttribute("path", "d", "", k));
                int startIndex, endIndex;
                float move_x, move_y, dest_x, dest_y, last_x, last_y;
                int state = 1;

                for (int l = 0; l < path.size(); l++) {
                    switch (state) {
                        case 1: // M -> comma
                            if (path[l] == 'M') {
                                startIndex = l+1;
                            }
                            if (path[l] == ',') {
                                endIndex = l-1;
                                move_x = stof(path.substr(startIndex, endIndex));
                                startIndex = l+1;
                                state = 2;
                            }
                            break;
                        case 2: // comma -> L [after M]
                            if (path[l] == 'L') {
                                endIndex = l-1;
                                move_y = stof(path.substr(startIndex, endIndex));
                                startIndex = l+1;
                                state = 3;
                            }
                            break;
                        case 3: // L -> comma [after M -> comma]
                            if (path[l] == ',') {
                                endIndex = l-1;
                                dest_x = stof(path.substr(startIndex, endIndex));
                                startIndex = l+1;
                                state = 4;
                            }
                            break;
                        case 4: // comma -> (L || M) [after M -> comma -> L]
                            if (path[l] == 'L') {
                                endIndex = l-1;
                                dest_y = stof(path.substr(startIndex, endIndex));
                                storePath(lineType, move_x, move_y, dest_x, dest_y);
                                last_x = dest_x;
                                last_y = dest_y;
                                startIndex = l+1;
                                state = 5;
                                cout << "to state " << state << endl;
                            } else if (path[l] == 'M') {
                                endIndex = l-1;
                                dest_y = stof(path.substr(startIndex, endIndex));
                                storePath(lineType, move_x, move_y, dest_x, dest_y);
                                startIndex = l+1;
                                state = 1;
                            }
                            break;
                        case 5: // L -> comma [after L]
                            if (path[l] == ',') {
                                endIndex = l-1;
                                dest_x = stof(path.substr(startIndex, endIndex));
                                startIndex = l+1;
                                state = 6;
                            }
                            break;
                        case 6: // comma -> L || M [after L -> comma]
                            if (path[l] == 'L') {
                                endIndex = l-1;
                                dest_y = stof(path.substr(startIndex, endIndex));
                                storePath(lineType, last_x, last_y, dest_x, dest_y);
                                last_x = dest_x;
                                last_y = dest_y;
                                startIndex = l+1;
                                state = 5;
                            } else if (path[l] == 'M') {
                                endIndex = l-1;
                                dest_y = stof(path.substr(startIndex, endIndex));
                                storePath(lineType, last_x, last_y, dest_x, dest_y);
                                startIndex = l+1;
                                state = 1;
                            }
                            break;
                        default:
                            cout << "unknown state" << endl;
                            break;
                    }
                    // reaches end of string
                    if (l == path.size()-1) {
                        endIndex = l;
                        dest_y = stof(path.substr(startIndex, endIndex));
                        if (state == 6) {
                            storePath(lineType, last_x, last_y, dest_x, dest_y);
                        } else {
                            storePath(lineType, move_x, move_y, dest_x, dest_y);
                        }
                    }
                }
            }
            currentMap.popTag();
        }
        currentMap.popTag();
    }
    currentMap.popTag();

    
    // TODO
    
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
