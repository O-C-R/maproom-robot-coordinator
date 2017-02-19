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

static const float kCalibrationWaitSec = 1.0f;
static const float kAngleWaitSec = 1.0f;

static const float kRotationTolerance = 2.0f;

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

void Robot::setState(RobotState newState) {
	state = newState;
	stateStartTime = ofGetElapsedTimef();
}

void Robot::update() {
	bool shouldSend = false;
	float elapsedStateTime = ofGetElapsedTimef() - stateStartTime;

	if (!commsUp()) {
		setState(R_NO_CONN);

		cmdStop(msg);
		shouldSend = true;
	} else if (!cvDetected()) {
		setState(R_NO_CONN);

		cmdStop(msg);
		shouldSend = true;
	} else if (state == R_NO_CONN) {
		// Now connected and seen!
		setState(R_START);

		cmdStop(msg);
		shouldSend = true;
	} else if (state == R_START) {
		// If we're started, immediately calibrate the angle

		setState(R_CALIBRATING_ANGLE);
	} else if (state == R_CALIBRATING_ANGLE) {
		// We're calibrating the angle for a bit

		cmdCalibrateAngle(msg, rot);
		shouldSend = true;

		if (elapsedStateTime >= kCalibrationWaitSec) {
			// We've calibrated enough
			setState(R_ROTATING_TO_ANGLE);
		}
	} else if (state == R_ROTATING_TO_ANGLE) {
		// We're rotating to a target angle

		if (abs(rot - targetRot) > kRotationTolerance) {
			// Too far from angle, keep moving.
			cmdRot(msg, rot, targetRot);
			shouldSend = true;
		} else {
			// Close enough to angle, wait to see if the robot stays close enough.
			setState(R_WAITING_ANGLE);
		}
	} else if (state == R_WAITING_ANGLE) {
		// We're waiting after issuing rotate commands

		if (abs(rot - targetRot) > kRotationTolerance) {
			// We're out of tolerance, try rotating again.
			setState(R_ROTATING_TO_ANGLE);
		} else if (elapsedStateTime > kAngleWaitSec) {
			// We've waited long enough, stop.
			setState(R_STOPPED);

			cmdStop(msg);
			shouldSend = true;
		}
	} else if (state == R_STOPPED) {
		// We're stopped. Stop.
		cmdStop(msg);
		shouldSend = true;
	}

	if (shouldSend) {
		sendMessage(msg);
	}
}
