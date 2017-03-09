//
//  Map.cpp
//  maproom-robot
//
//  Created by Christopher Anderson on 2/20/17.
//
//

#include "Map.h"


typedef int OutCode;

const int INSIDE = 0; // 0000
const int LEFT = 1;   // 0001
const int RIGHT = 2;  // 0010
const int BOTTOM = 4; // 0100
const int TOP = 8;    // 1000

// Compute the bit code for a point (x, y) using the clip rectangle
// bounded diagonally by (xmin, ymin), and (xmax, ymax)

// ASSUME THAT xmax, xmin, ymax and ymin are global constants.

OutCode ComputeOutCode(const ofVec2f &pt, const ofRectangle &rect) {
	OutCode code;

	const ofVec2f tl = rect.getTopLeft(), br = rect.getBottomRight();
	code = INSIDE;          // initialised as being inside of [[clip window]]

	if (pt.x < tl.x - 0.001)           // to the left of clip window
		code |= LEFT;
	else if (pt.x > br.x + 0.001)      // to the right of clip window
		code |= RIGHT;
	if (pt.y < tl.y - 0.001)           // below the clip window
		code |= TOP;
	else if (pt.y > br.y + 0.001)      // above the clip window
		code |= BOTTOM;

	return code;
}

// Cohen–Sutherland clipping algorithm clips a line from
// P0 = (x0, y0) to P1 = (x1, y1) against a rectangle with
// diagonal from (xmin, ymin) to (xmax, ymax).
bool CohenSutherlandLineClip(ofVec2f &pt1, ofVec2f &pt2, const ofRectangle &rect) {
	// compute outcodes for P0, P1, and whatever point lies outside the clip rectangle
	OutCode outcode0 = ComputeOutCode(pt1, rect);
	OutCode outcode1 = ComputeOutCode(pt2, rect);

	const ofVec2f tl = rect.getTopLeft(), br = rect.getBottomRight();

	while (true) {
		if (!(outcode0 | outcode1)) { // Bitwise OR is 0. Trivially accept and get out of loop
			return true;
		} else if (outcode0 & outcode1) { // Bitwise AND is not 0. (implies both end points are in the same region outside the window). Reject and get out of loop
			return false;
		} else {
			// failed both tests, so calculate the line segment to clip
			// from an outside point to an intersection with clip edge
			ofVec2f clip;

			// At least one endpoint is outside the clip rectangle; pick it.
			OutCode outcodeOut = outcode0 ? outcode0 : outcode1;

			// Now find the intersection point;
			// use formulas y = y0 + slope * (x - x0), x = x0 + (1 / slope) * (y - y0)
			if (outcodeOut & BOTTOM) {           // point is above the clip rectangle
				clip.x = pt1.x + (pt2.x - pt1.x) * (br.y - pt1.y) / (pt2.y - pt1.y);
				clip.y = br.y;
			} else if (outcodeOut & TOP) { // point is below the clip rectangle
				clip.x = pt1.x + (pt2.x - pt1.x) * (tl.y - pt1.y) / (pt2.y - pt1.y);
				clip.y = tl.y;
			} else if (outcodeOut & RIGHT) {  // point is to the right of clip rectangle
				clip.y = pt1.y + (pt2.y - pt1.y) * (br.x - pt1.x) / (pt2.x - pt1.x);
				clip.x = br.x;
			} else if (outcodeOut & LEFT) {   // point is to the left of clip rectangle
				clip.y = pt1.y + (pt2.y - pt1.y) * (tl.x - pt1.x) / (pt2.x - pt1.x);
				clip.x = tl.x;
			}

			// Now we move outside point to intersection point to clip
			// and get ready for next pass.
			if (outcodeOut == outcode0) {
				pt1.x = clip.x;
				pt1.y = clip.y;
				outcode0 = ComputeOutCode(pt1, rect);
				cout << "pt1 is now" << pt1 << " - " << outcode0 << endl;
			} else {
				pt2.x = clip.x;
				pt2.y = clip.y;
				outcode1 = ComputeOutCode(pt2, rect);
				cout << "pt2 is now" << pt2 << " - " << outcode1 << endl;
			}
		}
	}
	return true;
}


Map::Map(float width, float height, float offsetX, float offsetY, ofRectangle crop):
	widthM(width), heightM(height),
	offsetX(offsetX), offsetY(offsetY),
	origOffsetX(offsetX), origOffsetY(offsetY),
	svgExtentMin(10000), svgExtentMax(-10000),
	storeCount(0), pathCount(0),
	cropBox(crop)
{}

void Map::storePath(string lineType, float startX, float startY, float destX, float destY) {
    // be wary of strings with types of the same first two letters

    pathSegment segment;
	segment.prescaleStart = ofVec2f(startX, startY);
	segment.prescaleEnd = ofVec2f(destX, destY);
    
    MapPath toStore = {storeCount++, false, false, lineType, segment};

	if (!mapPathStore[lineType].size()) {
		pathTypes.push_back(lineType);
		activePaths[lineType] = true;
	}
	mapPathStore[lineType].push_back(toStore);

	if (segment.prescaleStart.x > svgExtentMax.x) {
		svgExtentMax.x = segment.prescaleStart.x;
	}
	if (segment.prescaleStart.y > svgExtentMax.y) {
		svgExtentMax.y = segment.prescaleStart.y;
	}
	if (segment.prescaleStart.x < svgExtentMin.x) {
		svgExtentMin.x = segment.prescaleStart.x;
	}
	if (segment.prescaleStart.y < svgExtentMin.y) {
		svgExtentMin.y = segment.prescaleStart.y;
	}

	if (segment.prescaleEnd.x > svgExtentMax.x) {
		svgExtentMax.x = segment.prescaleEnd.x;
	}
	if (segment.prescaleEnd.y > svgExtentMax.y) {
		svgExtentMax.y = segment.prescaleEnd.y;
	}
	if (segment.prescaleEnd.x < svgExtentMin.x) {
		svgExtentMin.x = segment.prescaleEnd.x;
	}
	if (segment.prescaleEnd.y < svgExtentMin.y) {
		svgExtentMin.y = segment.prescaleEnd.y;
	}
}

// TODO: reincorporate this somewhere
//bool shouldStore = true;
//if ((segment.end - segment.start).length() < 0.005f) {
//	shouldStore = false;
//}
//for (auto &path : mapPathStore[lineType]) {
//	if (!shouldStore) {
//		break;
//	}
//
//	const float kEpsilon = 0.001; // 1mm
//	if (path.segment.start.distance(segment.start) < kEpsilon && path.segment.end.distance(segment.end) < kEpsilon) {
//		shouldStore = false;
//	} else if (path.segment.end.distance(segment.start) < kEpsilon && path.segment.start.distance(segment.end) < kEpsilon) {
//		shouldStore = false;
//	}
//}

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

float getAngle(ofVec2f start, ofVec2f end) {
    return atan2(end.x - start.x, end.y - start.y)*180/3.14159;
}

void Map::optimizePaths(float percentDiff) {
    const float kEpsilon = 0.001; // 1mm
    int toRemoveCount = 0;
    for (auto type: pathTypes) {
        for (int i=0; i<mapPathStore[type].size(); i++) {
            pathSegment cur = mapPathStore[type][i].segment;

            for (int j=0; j<mapPathStore[type].size(); j++) {
                if (i != j) {
                    pathSegment tar = mapPathStore[type][j].segment;
                    bool endToStart = cur.end.distance(tar.start) < kEpsilon;
                    bool startToEnd = cur.start.distance(tar.end) < kEpsilon;
                    bool endToEnd = cur.end.distance(tar.end) == kEpsilon;
                    bool startToStart = cur.start.distance(tar.start) == kEpsilon;

                    if((endToStart || startToEnd || endToEnd || startToStart) && !mapPathStore[type][i].drawn && !mapPathStore[type][j].drawn && !mapPathStore[type][i].claimed && !mapPathStore[type][j].claimed) {
                        float curAngle;
                        float tarAngle;
                        float newAngle;
                        if (endToEnd) {
                            curAngle = getAngle(cur.end, cur.start);
                            tarAngle = getAngle(tar.start, tar.end);
                        } else if (endToStart) {
                            curAngle = getAngle(cur.end, cur.start);
                            tarAngle = getAngle(tar.end, tar.start);
                        } else if (startToStart) {
                            curAngle = getAngle(cur.start, cur.end);
                            tarAngle = getAngle(tar.end, tar.start);
                        } else if (startToEnd) {
                            curAngle = getAngle(cur.start, cur.end);
                            tarAngle = getAngle(tar.start, tar.end);
                        }
                        
                        
                        MapPath tmp = mapPathStore[type][i];
                        
                        float diff = ((curAngle+360) - (tarAngle+360));
                        if (abs(fmod((diff+360),360.0)) < percentDiff) {
//                            
//                            cout << "cur " <<  cur.start << "," << cur.end << endl;
//                            cout << "curAngle " << curAngle << endl;
//                            cout << "tar " <<  tar.start << "," << tar.end << endl;
//                            cout << "tarAngle " << tarAngle << endl;
//                            cout << "diff: " << diff << endl;
                            
                            if (startToStart) {
                                mapPathStore[type][i].segment.end = mapPathStore[type][j].segment.end;
                            } else if (startToEnd) {
                                mapPathStore[type][i].segment.start = mapPathStore[type][j].segment.start;
                            } else if (endToStart) {
                                mapPathStore[type][i].segment.end = mapPathStore[type][j].segment.end;
                            } else if (endToEnd) {
                                mapPathStore[type][i].segment.end = mapPathStore[type][j].segment.start;
                            }

//                            cout << "similar degrees" << endl;
                            mapPathStore[type][j].claimed = true;
                            toRemoveCount++;
//                            break;
                        }
                    }
                }
            }
        }
    }



    // remove all claimed path
    int removedPaths = 0;
    while(toRemoveCount > 0) {
        for (auto type: pathTypes) {
            for (int i=0; i<mapPathStore[type].size(); i++) {
                if (mapPathStore[type][i].claimed) {
                    mapPathStore[type].erase(mapPathStore[type].begin() + i);
                    toRemoveCount--;
                    removedPaths++;
                    break;
                }
                
            }
        }
    }
    cout << "removed paths " << removedPaths << endl;
}

void Map::clearStore() {
	activePaths.clear();
	mapPathStore.clear();
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
	currentMap.clear();
    currentMap.loadFile(filename);
	scaleX = 1.0;
	scaleY = 1.0;
	offsetX = 0.0;
	offsetY = 0.0;
	svgExtentMin = ofVec2f(10000);
	svgExtentMax = ofVec2f(-10000);

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

	rescaleMap(widthM, heightM, origOffsetX, origOffsetY);
}

void Map::rescaleMap(float width, float height, float newOffsetX, float newOffsetY) {
	widthM = width;
	heightM = height;

	float svgWidth = svgExtentMax.x - svgExtentMin.x;
	float svgHeight = svgExtentMax.y - svgExtentMin.y;

	scaleX = 1.0 / svgWidth * widthM;
	scaleY = 1.0 / svgHeight * heightM;
	const ofVec2f scale(scaleX, scaleY);

	offsetX = newOffsetX - svgExtentMin.x * scaleX;
	offsetY = newOffsetY - svgExtentMin.y * scaleY;
	const ofVec2f offset(offsetX, offsetY);

	for (auto &path : pathTypes) {
		if (mapPathStore.find(path) == mapPathStore.end()) {
			continue;
		}

		for (auto mapPathIt = mapPathStore[path].begin(); mapPathIt != mapPathStore[path].end(); /* no increment */)
		{
			MapPath &mapPath = *mapPathIt;

			if (mapPath.claimed || mapPath.drawn) {
				continue;
			}

			mapPath.segment.start = mapPath.segment.prescaleStart * scale + offset;
			mapPath.segment.end = mapPath.segment.prescaleEnd * scale + offset;

			bool success = CohenSutherlandLineClip(mapPath.segment.start, mapPath.segment.end, cropBox);

			static const float kEpsilon = 0.005;
			for (auto &otherMapPath : mapPathStore[path]) {
				if (otherMapPath.id == mapPath.id) {
					continue;
				}

				if ((mapPath.segment.start.distance(otherMapPath.segment.start) < kEpsilon && mapPath.segment.end.distance(otherMapPath.segment.end) < kEpsilon)
					|| (mapPath.segment.start.distance(otherMapPath.segment.end) < kEpsilon && mapPath.segment.end.distance(otherMapPath.segment.start) < kEpsilon)) {
					success = false;
					break;
				}
			}

			if (!success) {
				mapPathIt = mapPathStore[path].erase(mapPathIt);
			} else {
				++mapPathIt;
			}
		}
	}
    
//    cout << "pre optimize count: " << getPathCount() << endl;
//    cout << "optimize SVG" << endl;
//    float optPercent = 6;
//    optimizePaths(optPercent);
//    cout << "post optimize count: " << getPathCount() << endl;
}

MapPath* Map::nextPath(const ofVec2f &pos, int robotId, float lastHeading, const set<string> &pathTypes) {
    MapPath *next = NULL;
    vector<MapPath*> contenders;
    float minDist = INFINITY;
    
    int checkedPath = 0;
    for (auto &pathType : pathTypes) {
        if (mapPathStore.find(pathType) == mapPathStore.end()) {
            continue;
        }
    
        // check if path is active and that the robot id can draw it
		if (!activePaths[pathType] || pathTypes.find(pathType) == pathTypes.end()) {
            continue;
        }

        // check if robot has that
        for (auto &mapPath : mapPathStore[pathType]) {
            if (mapPath.claimed || mapPath.drawn) {
                continue;
            }
            checkedPath++;
            
            float startDist = mapPath.segment.start.distance(pos);
            float endDist = mapPath.segment.end.distance(pos);

            if (startDist < minDist) {
                minDist = startDist;
                contenders.clear();
                contenders.push_back(&mapPath);
            } else if (endDist < minDist) {
                minDist = endDist;
                contenders.clear();
                contenders.push_back(&mapPath);
            } else if (abs(endDist-minDist) < 0.0001) {
                contenders.push_back(&mapPath);
            } else if (abs(startDist-minDist) < 0.0001) {
                contenders.push_back(&mapPath);
            }
        }
    }
    if (contenders.size() > 1) {
        // pick path with heading closest to last path
        float minAngleDiff = INFINITY;
        for (int i=0; i<contenders.size(); i++) {
            MapPath p = *contenders[i];
            float angle = getAngle(p.segment.start, p.segment.end);
            float angleDiff = abs(fmod(((angle+360)-(lastHeading+360)), 360.0));
            if (angleDiff < minAngleDiff) {
                minAngleDiff = angleDiff;
                next = contenders[i];
            }
        }
        return next;
    } else if (contenders.size() == 1) {
        return contenders[0];
    } else {
     	return next;
    }
}

