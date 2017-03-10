//
//  Util.h
//  maproom-robot
//
//  Created by Christopher Anderson on 3/3/17.
//
//

#ifndef Util_h
#define Util_h

#include "ofMain.h"

typedef int OutCode;

const int INSIDE = 0; // 0000
const int LEFT = 1;   // 0001
const int RIGHT = 2;  // 0010
const int BOTTOM = 4; // 0100
const int TOP = 8;    // 1000

static OutCode ComputeOutCode(const ofVec2f &pt, const ofRectangle &rect) {
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
static bool CohenSutherlandLineClip(ofVec2f &pt1, ofVec2f &pt2, const ofRectangle &rect) {
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


#endif /* Util_h */
