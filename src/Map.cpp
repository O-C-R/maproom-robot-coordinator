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
    
    MapPath toStore = {false, false, lineType, segment};
    
    bool shouldStore = true;
	if ((segment.end - segment.start).length() < 0.005f) {
        shouldStore = false;
    }
    for (auto &path : mapPathStore[lineType]) {
        if (!shouldStore) {
            break;
        }
        
        const float kEpsilon = 0.001; // 1mm
        if (path.segment.start.distance(segment.start) < kEpsilon && path.segment.end.distance(segment.end) < kEpsilon) {
            shouldStore = false;
        } else if (path.segment.end.distance(segment.start) < kEpsilon && path.segment.start.distance(segment.end) < kEpsilon) {
            shouldStore = false;
        }
    }
    if (shouldStore) {
        if (!mapPathStore[lineType].size()) {
            pathTypes.push_back(lineType);
            activePaths[lineType] = true;
        }
        mapPathStore[lineType].push_back(toStore);
        storeCount++;
        cout << "stored " << storeCount << endl;
    }
}

void Map::setPathActive(string lineType, bool active) {
    if (mapPathStore.find(lineType) != mapPathStore.end()) {
        activePaths[lineType] = active;
    } else {
        cout << lineType << " is not in activePaths" << endl;
    }
}

int Map::getActivePathCount() {
    int count = 0;
    for (auto &i : mapPathStore) {
        if (activePaths[i.first]) {
            for (auto &j : i.second) {
                count++;
            }
        }
    }
    return count;
}

int Map::getDrawnPaths() {
    int count = 0;
    for (auto &i : mapPathStore) {
        if (activePaths[i.first]) {
            for (auto &j : i.second) {
                if (j.drawn) count++;
            }
        }
    }
    return count;
}

int Map::getPathCount() {
    int count = 0;
    for (auto type: pathTypes) {
        for (auto path: mapPathStore[type]) {
            count++;
        }
    }
    return count;
}

int Map::getPathCount(string type) {
    int count = 0;
    for (auto path: mapPathStore[type]) {
        count++;
    }
    return count;
}

void Map::optimizePaths() {
    const float kEpsilon = 0.001; // 1mm
    const float kAngleDiff = 0.01; // 0.1¼
    for (auto type: pathTypes) {
//        for (auto &path : mapPathStore[type]) {
        for (int i=0; i<mapPathStore[type].size(); i++) {
            pathSegment cur = mapPathStore[type][i].segment;
            for (int j=0; j<mapPathStore[type].size(); j++) {
                if (i != j) {
                    pathSegment tar = mapPathStore[type][j].segment;
                    if(cur.end.distance(tar.start) < kEpsilon || cur.start.distance(tar.end) < kEpsilon || cur.end.distance(tar.end) < kEpsilon || cur.start.distance(tar.start) < kEpsilon) {
                        float curAngle = cur.start.angle(cur.end);
                        float tarAngle = tar.start.angle(tar.end);
                        float diff = ofAngleDifferenceDegrees(curAngle, tarAngle);
                        if (abs(diff) < kAngleDiff) {
                            cout << "curAngle " << curAngle << endl;
                            cout << "otherAngle " << tarAngle << endl;
                            if (cur.end.distance(tar.start) < kEpsilon) {
                                mapPathStore[type][i].segment.end = tar.end;
                                mapPathStore[type][i].drawn = true;
                                mapPathStore[type].erase(mapPathStore[type].begin() + j);
                            } else if (cur.start.distance(tar.end) < kEpsilon) {
                                mapPathStore[type][i].segment.start = tar.start;
                                mapPathStore[type][i].drawn = true;
                                mapPathStore[type].erase(mapPathStore[type].begin() + j);
                            } else if (cur.end.distance(tar.end) < kEpsilon) {
                                mapPathStore[type][i].segment.start = tar.start;
                                mapPathStore[type][i].drawn = true;
                                mapPathStore[type].erase(mapPathStore[type].begin() + j);
                            } else {
                                mapPathStore[type][i].segment.end = tar.end;
                                mapPathStore[type][i].drawn = true;
                                mapPathStore[type].erase(mapPathStore[type].begin() + j);
                            }
                        }
                    } else if(cur.start.distance(tar.start) < kEpsilon) {
                        cout << "starts are equal i: " << i << " j: " << j << endl;
                    }
                }
            }
        }
    }
}



void Map::clearStore() {
    for (int i=0; i < pathTypes.size(); i++) {
        mapPathStore[pathTypes[i]].clear();
    }
    pathTypes.clear();
}

string Map::getMostRecentMap(string filePath) {
    // TODO: make this not absolute path, or change username
    ofDirectory dir(filePath);
    dir.allowExt("svg");
    dir.listDir();
    
    string mostRecentTime = "";
    string file = "";
    
    for(int i = 0; i < dir.size(); i++){
        string fileName = ofToString(dir.getPath(i));
        regex r("maproom-(.+)\\.svg");
        smatch m;
        regex_search(fileName, m, r);
        for(auto v: m) {
            if (ofToString(v).size() == 24) {
                if (mostRecentTime != "") {
                    mostRecentTime = v;
                    file = fileName;
                } else if(v.compare(mostRecentTime) > 0) {
                    mostRecentTime = v;
                    file = fileName;
                }
            }
        }
    }
    return file;
}

void Map::loadMap(const string filename) {
    currentMap.loadFile(filename);
    scaleX = widthM / stoi(ofToString(currentMap.getAttribute("svg", "width", "")));
    scaleY = heightM / stoi(ofToString(currentMap.getAttribute("svg", "height", "")));
    
    clearStore();
    
    currentMap.pushTag("svg");
    int firstLevel = currentMap.getNumTags("g");
    for (int i = 0; i < firstLevel; i++) {
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
                                pathCount++;
                                last_x = dest_x;
                                last_y = dest_y;
                                startIndex = l+1;
                                state = 5;
                            } else if (path[l] == 'M') {
                                endIndex = l-1;
                                dest_y = stof(path.substr(startIndex, endIndex));
                                storePath(lineType, last_x, last_y, dest_x, dest_y);
                                pathCount++;
                                startIndex = l+1;
                                state = 1;
                            } else if (path[l] == 'Z') {
                                endIndex = l-1;
                                dest_y = stof(path.substr(startIndex, endIndex));
                                storePath(lineType, dest_x, dest_y, firstX, firstY);
                                pathCount++;
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
                            pathCount++;
                        } else {
                            storePath(lineType, move_x, move_y, dest_x, dest_y);
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
//    cout << "pre optimize count: " << getPathCount() << endl;
//    cout << "optimize SVG" << endl;
//    optimizePaths();
//    cout << "post optimize count: " << getPathCount() << endl;
}

MapPath* Map::nextPath(const ofVec2f &pos) {
    MapPath *next = NULL;
    float minDist = INFINITY;
    
    int checkedPath = 0;
    
    for (auto &path : pathTypes) {
        if (mapPathStore.find(path) == mapPathStore.end()) {
            continue;
        }

        for (auto &mapPath : mapPathStore[path]) {
            if (mapPath.claimed || mapPath.drawn) {
                continue;
            }
            checkedPath++;
            
            float startDist = mapPath.segment.start.distance(pos);
            float endDist = mapPath.segment.end.distance(pos);

            if (startDist < minDist) {
                minDist = startDist;
                next = &mapPath;
            } else if (endDist < minDist) {
                ofVec2f tmp = mapPath.segment.start;
                mapPath.segment.start = mapPath.segment.end;
                mapPath.segment.end = tmp;

                minDist = endDist;
                next = &mapPath;
            }
        }
    }
	return next;
}

