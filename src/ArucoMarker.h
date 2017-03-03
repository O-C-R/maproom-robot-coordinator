//
//  ArucoMarker.h
//  maproom-robot
//
//  Created by Christopher Anderson on 2/28/17.
//
//

#ifndef ArucoMarker_h
#define ArucoMarker_h

#include "ofMain.h"

class ArucoMarker {
public:
	ArucoMarker();
	ArucoMarker(int id);

	void updateCamera(const ofVec2f &imPos, const ofVec2f &imUp);

	int id;

	// From openCV
	ofVec2f imgPos, upVec;
	float lastCameraUpdateTime;
	float cvFramerate;

	// Derived
	ofVec2f planePos;
	float glRot;
};

#endif
