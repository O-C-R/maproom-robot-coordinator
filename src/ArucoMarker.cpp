//
//  ArucoMarker.cpp
//  maproom-robot
//
//  Created by Christopher Anderson on 2/28/17.
//
//

#include "ArucoMarker.h"

ArucoMarker::ArucoMarker(int inId):
	id(inId),
	cvFramerate(0),
	lastCameraUpdateTime(0)
{}

ArucoMarker::ArucoMarker(): ArucoMarker(-1) {}

void ArucoMarker::updateCamera(const ofVec2f &imPos, const ofVec2f &imUp) {
	const float now = ofGetElapsedTimef();
	const float dt = now - lastCameraUpdateTime;

	imgPos = imPos;
	upVec = imUp;

	planePos = imPos;
	glRot = atan2(imUp.y, imUp.x);

	const float framerate = 1.0 / dt;
	cvFramerate += (framerate - cvFramerate) * 0.1;
	lastCameraUpdateTime = now;
}
