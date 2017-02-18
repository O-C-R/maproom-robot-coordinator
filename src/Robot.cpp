//
//  Robot.cpp
//  maproom-robot
//
//  Created by Christopher Anderson on 2/18/17.
//
//

#include "Robot.h"

static const float kHeartbeatTimeoutSec = 2.0f;
static const float kCameraTimeoutSec = 0.5f;

void Robot::setCommunication(const string &rIp, int rPort) {
	ip = rIp;
	port = rPort;

	socket.Create();
	socket.Connect(ip.c_str(), port);
	socket.SetNonBlocking(true);
}

void Robot::sendMessage(const string &message) {
	socket.Send(message.c_str(), message.length());
	lastMessage = message;
}

void Robot::sendHeartbeat() {
	sendMessage("MRHB");
	lastHeartbeatTime = ofGetElapsedTimef();
}

void Robot::updateCamera(const ofVec3f &newRvec, const ofVec3f &newTvec, const ofMatrix4x4 &cameraWorldInv) {
	rvec = newRvec;
	tvec = newTvec;

	// http://answers.opencv.org/question/110441/use-rotation-vector-from-aruco-in-unity3d/
	float angle = sqrt(rvec.x*rvec.x + rvec.y*rvec.y + rvec.z*rvec.z);
	ofVec3f axis(rvec.x, rvec.y, rvec.z);

	ofQuaternion rvecQuat;
	rvecQuat.makeRotate(angle / 3.14159 * 180.0, axis);

	mat.makeIdentityMatrix();
	mat.setRotate(rvecQuat);
	mat.setTranslation(tvec);
	mat = mat * cameraWorldInv;

	// Get position in world
	worldPos = mat.getTranslation();
	planePos.set(worldPos.x, worldPos.y);

	// Get z-axis rotation
	rot = mat.getRotate().getEuler().z;

	// Rotate so values are always in 0-360
	rot += 180.0;

	lastCameraUpdateTime = ofGetElapsedTimef();
}

void Robot::gotHeartbeat() {
	lastHeartbeatTime = ofGetElapsedTimef();
}

bool Robot::commsUp() {
	return ofGetElapsedTimef() - lastHeartbeatTime < kHeartbeatTimeoutSec;
}

bool Robot::cvDetected() {
	return ofGetElapsedTimef() - lastCameraUpdateTime < kCameraTimeoutSec;
}

void Robot::update() {
	if (lastHeartbeatTime < 0 || !commsUp()) {
		state = R_NO_CONN;
		cmdStop(msg);
	} else if (lastCameraUpdateTime < 0 || !cvDetected()) {
		state = R_NO_CONN;
		cmdStop(msg);
	} else if (state == R_START) {
		// TODO: implement
		cmdStop(msg);
	}

	sendMessage(msg);
}
