//
//  VecExt.h
//  maproom-robot
//
//  Created by Christopher Anderson on 3/7/17.
//
//

#ifndef VecExt_h
#define VecExt_h

class ofVec2i {
public:
	ofVec2i(int _x, int _y) {
		x = _x;
		y = _y;
	}

	ofVec2i(int _v = 0) {
		x = _v;
		y = _v;
	}

	ofVec2i(const ofVec2i &o) {
		x = o.x;
		y = o.y;
	}

	bool operator==(const ofVec2i &o) {
		return x == o.x && y == o.y;
	}

	ofVec2i operator+(const ofVec2i &r) {
		return { x + r.x, y + r.y };
	}

	bool inBounds(int w, int h) {
		return x >= 0 && y >= 0 && x < w && y < h;
	}

	int x,y;
};

class ThreePtRect {
public:
	ThreePtRect(const ofVec2f& ul, const ofVec2f& bl, const ofVec2f& br): ul(ul), bl(bl), br(br) {};

	bool contains(const ofVec2f pt) {

	}

	ofVec2f ul, bl, br;

};

#endif /* VecExt_h */
