//
//  PathPlanner.h
//  maproom-robot
//
//  Created by Christopher Anderson on 3/7/17.
//
//

#pragma once

#include "ofMain.h"
#include "VecExt.h"

typedef struct GridNode {
	int g, h;

	int onDrawPathForRobotId;
	int containsRobotId;
	int nextPathForRobotId;

	ofVec2i loc;
	GridNode *parent;

	GridNode(): g(0), h(0), onDrawPathForRobotId(-1), containsRobotId(-1), nextPathForRobotId(-1), parent(NULL) {};
} GridNode;

class PathPlanner {
public:
	int width, height;
	vector<vector<GridNode>> grid;

	PathPlanner(): width(0), height(0) {};
	PathPlanner(int _width, int _height) {
		width = _width;
		height = _height;
		grid.reserve(width);
		for (int x = 0; x < width; ++x) {
			grid.push_back(vector<GridNode>());
			grid[x].reserve(height);
			for (int y = 0; y < height; ++y) {
				grid[x].push_back(GridNode());
				grid[x][y].loc = ofVec2i(x, y);
			}
		}
	}

	GridNode* at(ofVec2i pt);
	float distance(ofVec2i pt1, ofVec2i pt2);
	vector<ofVec2i> findPath(ofVec2i origin, ofVec2i target);
};

