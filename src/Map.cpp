//
//  Map.cpp
//  maproom-robot
//
//  Created by Christopher Anderson on 2/20/17.
//
//

#include "Map.h"

Map::Map(float width, float height, float offsetX, float offsetY): widthM(width), heightM(height), offsetX(offsetX), offsetY(offsetY) {}

int storeCount = 0;
int pathCount = 0;

void Map::storePath(string lineType, float startX, float startY, float destX, float destY) {
    // be wary of strings with types of the same first two letters

    pathSegment segment;
    segment.start = ofVec2f(scaleX*startX + offsetX, scaleY*startY + offsetY);
    segment.end = ofVec2f(scaleY*destX + offsetX, scaleY*destY + offsetY);
    
    // see if duplicate path exists already in store
    
    MapPath toStore = {false, lineType, segment};
    storeCount++;
    
    bool shouldStore = true;
    for (auto &path : mapPathStore[lineType]) {
        if (!shouldStore) {
            break;
        }
        if(path.segment.start == segment.start && path.segment.end == segment.end) {
            shouldStore = false;
        } else if (path.segment.end == segment.start && path.segment.start == segment.end) {
            shouldStore = false;
        }
    }
    if (shouldStore) {
        cout << "pathCount " << pathCount << " storeCount " << storeCount << endl;
        // check if pathype exists in pathtypes list
        if (!mapPathStore[lineType].size()) {
            pathTypes.push_back(lineType);
        }
        mapPathStore[lineType].push_back(toStore);
    }
}



void Map::clearStore() {
    for (int i=0; i < pathTypes.size(); i++) {
        mapPathStore[pathTypes[i]].clear();
    }
    pathTypes.clear();
}

void Map::loadMap(const string filename) {
    currentMap.loadFile(filename);
    scaleX = widthM / stoi(ofToString(currentMap.getAttribute("svg", "width", "")));
    scaleY = heightM / stoi(ofToString(currentMap.getAttribute("svg", "height", "")));
    
    clearStore();
    
    currentMap.pushTag("svg");
    int firstLevel = currentMap.getNumTags("g");
    for (int i = 0; i < firstLevel; i++) {
//        cout << "first_level id: " << currentMap.getAttribute("g", "id", "", i) << endl;
        currentMap.pushTag("g", i);
        int secondLevel = currentMap.getNumTags("g");
        for (int j = 0; j < secondLevel; j++) {
            string lineType = ofToString(currentMap.getAttribute("g", "id", "", j));
            currentMap.pushTag("g", j);
            int thirdLevel = currentMap.getNumTags("path");
            for (int k = 0; k < thirdLevel; k++) {
                string path = ofToString(currentMap.getAttribute("path", "d", "", k));
                int startIndex, endIndex, firstX, firstY;
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
                                // store incase there is a Z
                                firstX = move_x;
                                firstY = move_y;
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
                                pathCount++;
                                cout << "stored: " << move_x << " " << move_y << " " << dest_x << " " << dest_y << endl;
                                last_x = dest_x;
                                last_y = dest_y;
                                startIndex = l+1;
                                state = 5;
                            } else if (path[l] == 'M') {
                                endIndex = l-1;
                                dest_y = stof(path.substr(startIndex, endIndex));
                                storePath(lineType, move_x, move_y, dest_x, dest_y);
                                pathCount++;
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
                                cout << "stored: " << last_x << " " << last_y << " " << dest_x << " " << dest_y << endl;
                                pathCount++;
                                last_x = dest_x;
                                last_y = dest_y;
                                startIndex = l+1;
                                state = 5;
                            } else if (path[l] == 'M') {
                                endIndex = l-1;
                                dest_y = stof(path.substr(startIndex, endIndex));
                                storePath(lineType, last_x, last_y, dest_x, dest_y);
                                cout << "stored: " << last_x << " " << last_y << " " << dest_x << " " << dest_y << endl;
                                pathCount++;
                                startIndex = l+1;
                                state = 1;
                            } else if (path[l] == 'Z') {
                                endIndex = l-1;
                                dest_y = stof(path.substr(startIndex, endIndex));
                                storePath(lineType, dest_x, dest_y, firstX, firstY);
                                cout << "stored: " << dest_x << " " << dest_y << " " << firstX << " " << firstY << endl;
                                pathCount++;
                                // todo: add edge case for Z in the middle of the string
                                state = 1;
                            }
                            break;
                        default:
                            cout << "unknown state reached in Map.cpp" << endl;
                            break;
                    }
                    // reaches end of string
                    if (l == path.size()-1) {
                        endIndex = l;
                        dest_y = stof(path.substr(startIndex, endIndex));
                        if (state == 6) {
                            storePath(lineType, last_x, last_y, dest_x, dest_y);
                            cout << "stored: " << last_x << " " << last_y << " " << dest_x << " " << dest_y << endl;
                            pathCount++;
                        } else {
                            storePath(lineType, move_x, move_y, dest_x, dest_y);
                            cout << "stored: " << last_x << " " << last_y << " " << dest_x << " " << dest_y << endl;
                            pathCount++;
                        }
                    }
                }
            }
            currentMap.popTag();
        }
        currentMap.popTag();
    }
    currentMap.popTag();
}


bool Map::checkNextPath(ofVec2f initial) {

    MapPath *next;
    bool foundNext;
    float minDist = INFINITY;
    
    for (auto &path : pathTypes) {
        if (mapPathStore.find(path) != mapPathStore.end()) {
            for (auto &mapPath : mapPathStore[path]) {
                if (mapPath.drawn) {
                    continue;
                }
                
                ofVec2f startDir = mapPath.segment.start - initial;
                ofVec2f endDir = mapPath.segment.end - initial;
                float startDist = startDir.length();
                float endDist = endDir.length();
                
                if (startDist < minDist) {
                    minDist = startDist;
                    next = &mapPath;
                    foundNext = true;
                } else if (endDist < minDist) {
                    ofVec2f tmp = mapPath.segment.start;
                    mapPath.segment.start = mapPath.segment.end;
                    mapPath.segment.end = tmp;

                    minDist = endDist;
                    next = &mapPath;
                    foundNext = true;
                }
            }
        }
    }
    
    if (foundNext) {
        next->drawn = true;
        nextPath = *next;
        return true;
    } else {
        return false;
    }
}

bool Map::checkNextPath(ofVec2f initial, string pathType) {
    MapPath *next;
    bool foundNext;
    float minDist = INFINITY;

    
    if (mapPathStore.find(pathType) != mapPathStore.end()) {
        for (auto &mapPath : mapPathStore[pathType]) {
            if (mapPath.drawn) {
                continue;
            }
            
            ofVec2f startDir = mapPath.segment.start - initial;
            ofVec2f endDir = mapPath.segment.end - initial;
            float startDist = startDir.length();
            float endDist = endDir.length();
            
            if (startDist < minDist) {
                minDist = startDist;
                next = &mapPath;
                foundNext = true;
            } else if (endDist < minDist) {
                ofVec2f tmp = mapPath.segment.start;
                mapPath.segment.start = mapPath.segment.end;
                mapPath.segment.end = tmp;
                
                minDist = endDist;
                next = &mapPath;
                foundNext = true;
            }
        }
    }

    if (foundNext) {
        next->drawn = true;
        nextPath = *next;
        return true;
    } else {
        return false;
    }
}

MapPath Map::getNextPath() {
    return nextPath;
}

