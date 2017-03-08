//
//  PathPlanner.cpp
//  maproom-robot
//
//  Created by Christopher Anderson on 3/7/17.
//
//

#include "PathPlanner.h"

float PathPlanner::distance(ofVec2i pt1, ofVec2i pt2) {
	return ofVec2f(pt1.x, pt1.y).distance(ofVec2f(pt2.x, pt2.y));
}

GridNode& PathPlanner::at(ofVec2i pt) {
	return grid[pt.x][pt.y];
}

vector<ofVec2i> PathPlanner::findPath(ofVec2i origin, ofVec2i target) {
	GridNode *start = &grid[origin.x][origin.y];
	GridNode *end = &grid[target.x][target.y];

	auto cmp = [](GridNode *left, GridNode *right) { return left->g + left->h < right->g + right->h; };
	priority_queue<GridNode*, std::vector<GridNode*>, decltype(cmp)> open(cmp);
	set<GridNode *> openSet;
	set<GridNode *> closed;

	open.push(start);
	openSet.insert(start);

	GridNode *current = NULL;
	while (open.size() > 0) {
		current = open.top();
		open.pop();
		openSet.erase(current);

		if (current == end) {
			break;
		}

		closed.insert(current);

		for (int x = -1; x <= 1; ++x) {
			for (int y = -1; y <= 1; ++y) {
				if (x == 0 && y == 0) continue;
				if (abs(x) == 1 && abs(y) == 1) continue;

				ofVec2i neighborLoc = current->loc + ofVec2i(x,y);
				if (!neighborLoc.inBounds(width, height)) continue;

				GridNode *neighbor = &grid[neighborLoc.x][neighborLoc.y];
//				if (neighbor->isObstacle) continue;

				float cost = current->g + 1;

				bool inOpen = openSet.find(neighbor) != openSet.end();
				bool inClosed = closed.find(neighbor) != closed.end();

				if (inOpen && cost < neighbor->g) {
					openSet.erase(neighbor);
				} else if (inClosed && cost < neighbor->g) {
					closed.erase(neighbor);
				} else if (!inOpen && !inClosed) {
					neighbor->g = cost;
					neighbor->h = distance(neighborLoc, target);
					neighbor->parent = current;

					open.push(neighbor);
					openSet.insert(neighbor);
				}
			}
		}
	}

	if (current == end) {
		vector<ofVec2i> ret;
		while (current != start) {
			ret.insert(ret.begin(), current->loc);
			current = current->parent;
		}
		return ret;
	} else {
		// No path to end
		return vector<ofVec2i>();
	}
}
