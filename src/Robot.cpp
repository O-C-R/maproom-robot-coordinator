//
//  Robot.cpp
//  maproom-robot
//
//  Created by Christopher Anderson on 2/18/17.
//
//

#include "Robot.h"

static const float kPositionTolerance = 0.005f;
static const float kRotationTolerance = 3.0f;

static const float kHeartbeatTimeoutSec = 2.0f;
static const float kCameraTimeoutSec = 1.0f;

static const float kCalibrationWaitSec = 0.25f;
static const float kAngleWaitSec = 2.0f;

void cmdCalibrateAngle(char *buf, int measured) {
	sprintf(buf, "MRCAL%+06d\n", measured);
}

void cmdRot(char *buf, int target, int measured) {
	sprintf(buf, "MRROT%+06d%+06d\n", target, measured);
}

void cmdMove(char *buf, int angle, int magnitude, int measured) {
	sprintf(buf, "MRMOV%+06d%+06d%+06d\n", angle, magnitude, measured);
}

void cmdDraw(char *buf, int angle, int magnitude, int measured) {
	sprintf(buf, "MRDRW%+06d%+06d%+06d\n", angle, magnitude, measured);
}

void cmdStop(char *buf, bool penDown) {
    sprintf(buf, "MRSTP%d\n", penDown ? 1 : 0);
}

void cmdStop(char * buf) {
	cmdStop(buf, false);
}

inline ofVec2f vecPtToLine(const ofVec2f &pt, const ofVec2f &l1, const ofVec2f &l2) {
	const ofVec2f dir = (l2 - l1).normalize();
	const ofVec2f a = pt - l1;
	return (l1 + (pt - l1).dot(dir) * dir) - pt;
}

inline float constrainTo360(const float deg) {
	return fmod(deg + 360.0f, 360.0f);
}

inline float ofRadToRobotDeg(const float rad) {
	return fmod(ofRadToDeg(rad) + 360 - 90, 360.0);
}

inline float robotDegToOfRad(const float deg) {
	return ofDegToRad(deg + 90);
}

Robot::Robot(int rId, int mId, const string &n) :
	id(rId),
	markerId(mId),
	name(n),
	state(R_NO_CONN),
	enabled(true),
	planePos(0, 0),
	avgPlanePos(0, 0),
	slowAvgPlanePos(0, 0),
	allAvgPlanePos(0, 0),
	planeVel(0, 0),
	avgPlaneVel(0, 0),
	targetRot(0),
	targetPlanePos(0, 0),
	rot(0),
	avgRot(0),
	stateStartTime(0),
	lastCameraUpdateTime(-1000),
	lastHeartbeatTime(-1000),
	targetLinePID(2500.0, 0, 0)
{
	targetLinePID.setMaxIOutput(2000.0);
	targetLinePID.setOutputLimits(-100000000.0, 100000000.0);
}

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
	sendMessage("MRHB\n");
}

void Robot::calibrate() {
    cmdCalibrateAngle(msg, rot);
	sendMessage(msg);
}

void Robot::stop() {
	if (state != R_STOPPED) {
		setState(R_STOPPED);
	}

    cmdStop(msg);
	sendMessage(msg);
}

void Robot::start() {
	if (state != R_START) {
		setState(R_START);
	}
}

string Robot::stateString() {
	switch(state) {
		case R_START:
			return "R_START";
		case R_CALIBRATING_ANGLE:
			return "R_CALIBRATING_ANGLE";
		case R_ROTATING_TO_ANGLE:
			return "ROTATING_TO_ANGLE";
		case R_WAITING_ANGLE:
			return "R_WAITING_ANGLE";
		case R_READY_TO_POSITION:
			return "R_READY_TO_POSITION";
		case R_POSITIONING:
			return "R_POSITIONING";
		case R_WAIT_AFTER_POSITION:
			return "R_WAIT_AFTER_POSITION";
		case R_READY_TO_DRAW:
			return "R_WAITING_TO_DRAW";
		case R_DRAWING:
			return "R_DRAWING";
		case R_DONE_DRAWING:
			return "R_DONE_DRAWING";
		case R_STOPPED:
			return "R_STOPPED";
		case R_NO_CONN:
			return "R_NO_CONN";
		default:
			return "UNKNOWN_STATE";
	}
}

void Robot::updateCamera(const ofVec3f &newRvec, const ofVec3f &newTvec, const ofMatrix4x4 &cameraWorldInv) {
	rvec = newRvec;
	tvec = newTvec;

	// Convert from Rodrigues formulation of rotation matrix
	// http://answers.opencv.org/question/110441/use-rotation-vector-from-aruco-in-unity3d/
	float angle = sqrt(rvec.x*rvec.x + rvec.y*rvec.y + rvec.z*rvec.z);
	ofVec3f axis(rvec.x, rvec.y, rvec.z);

	ofQuaternion rvecQuat;
	rvecQuat.makeRotate(ofRadToDeg(angle), axis);

	mat.makeIdentityMatrix();
	mat.setRotate(rvecQuat);
	mat.setTranslation(tvec);
	mat = mat * cameraWorldInv;

	// Get position in world
	worldPos = mat.getTranslation();

	// Calculate plane positions, smoothed over time.
	ofVec2f newPlanePos(worldPos.x, worldPos.y);
	allAvgPlanePos += (newPlanePos - allAvgPlanePos) * 0.5;

	// Skip outliers
	if (newPlanePos.distance(allAvgPlanePos) > 0.01) {
		return;
	}

	const float now = ofGetElapsedTimef();
	const float dt = now - lastCameraUpdateTime;

	// Calculate delta velocity first
	if (dt > 0) {
		planeVel = (newPlanePos - planePos) / dt;
		avgPlaneVel += (avgPlaneVel - planeVel) * 0.25;
	}

	// Update positions and previous averages
	planePos.set(newPlanePos);
	avgPlanePos += (planePos - avgPlanePos) * 0.25;
	slowAvgPlanePos += (planePos - slowAvgPlanePos) * 0.075;

	// Get z-axis rotation
	// Rotation is right-handed, and goes from 0-360 starting on the +x axis
	static const ofVec3f upVec(0.0, 1.0, 0.0);
	ofVec3f angAxis = upVec * mat;
	ofVec2f axisVec = ofVec2f(angAxis.x, angAxis.y) - ofVec2f(worldPos.x, worldPos.y);
	float axisAngle = atan2(axisVec.y, axisVec.x);

	// Translate rotation into robot coordinates
	rot = ofRadToRobotDeg(axisAngle);
	avgRot += ofAngleDifferenceDegrees(avgRot, rot) * 0.1;
	avgRot = constrainTo360(avgRot);

	lastCameraUpdateTime = ofGetElapsedTimef();
}

void Robot::gotHeartbeat() {
	lastHeartbeatTime = ofGetElapsedTimef();
}

bool Robot::commsUp() {
    return true;
//	return ofGetElapsedTimef() - lastHeartbeatTime < kHeartbeatTimeoutSec;
}

bool Robot::cvDetected() {
	return ofGetElapsedTimef() - lastCameraUpdateTime < kCameraTimeoutSec;
}

void Robot::setState(RobotState newState) {
	cout << "State change " << stateString() << " -> ";
	state = newState;
	cout << stateString() << endl;

	stateStartTime = ofGetElapsedTimef();
}

void Robot::moveRobot(char *msg, bool drawing, bool &shouldSend) {
	// Vectors for movement - ideal and remaining
    const ofVec2f line = targetPlanePos - startPlanePos;
	const ofVec2f currentToEnd = targetPlanePos - planePos;

	// Calculate where and how fast we'd go to just get to the end
	const float distanceToEnd = currentToEnd.length();
	float forwardMag = ofMap(distanceToEnd, 0, 1, 50, 250, true);
	const ofVec2f currentToEndDir = ofVec2f(currentToEnd).normalize();
	vecToEnd = currentToEndDir * forwardMag;

	// Calculate how far we are from the line and how we should correct for that
	dirToLine = vecPtToLine(planePos, startPlanePos, targetPlanePos);
	const float distToLine = dirToLine.length();
	const double targetLinePIDOutput = targetLinePID.getOutput(distToLine, 0.0);
	backToLine = targetLinePIDOutput * ofVec2f(dirToLine).normalize() * -1.0;

	// Combine the two vectors
	movement = (vecToEnd + backToLine).normalize() * forwardMag;
	const float angle = ofRadToRobotDeg(atan2(movement.y, movement.x));
	const float mag = movement.length();

	// Send message
	if (drawing) {
		cmdDraw(msg, angle, mag, rot);
		shouldSend = true;
    } else {
        cmdMove(msg, angle, mag, rot);
		shouldSend = true;
    }
}

bool Robot::atRotation() {
	return ofAngleDifferenceDegrees(targetRot, avgRot) < kRotationTolerance;
}

bool Robot::inPosition(const ofVec2f &pos) {
	return targetPlanePos.distance(pos) < kPositionTolerance;
}

void Robot::navigateTo(const ofVec2f &target) {
	startPlanePos = avgPlanePos;
	targetPlanePos = target;
	targetLinePID.reset();

    setState(R_POSITIONING);
}

void Robot::drawLine(const ofVec2f &start, const ofVec2f &end) {
	startPlanePos = start;
	targetPlanePos = end;
	targetLinePID.reset();

	setState(R_DRAWING);
}

void Robot::update() {
	if (!enabled) {
		return;
	}

	bool shouldSend = false, mustSend = false;
	const float elapsedStateTime = ofGetElapsedTimef() - stateStartTime;

	if (!commsUp()) {
		if (state != R_NO_CONN) {
			cout << "Comms down, moving to NO_CONN" << endl;
			setState(R_NO_CONN);
		}

		cmdStop(msg);
		shouldSend = true;
	} else if (!cvDetected()) {
		if (state != R_NO_CONN) {
			cout << "CV down, moving to NO_CONN" << endl;
			setState(R_NO_CONN);
		}

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

		cmdStop(msg);
		sendMessage(msg);

		cmdCalibrateAngle(msg, avgRot);
		shouldSend = true;
        
		if (elapsedStateTime >= kCalibrationWaitSec) {
			// We've calibrated enough
            setState(R_ROTATING_TO_ANGLE);
		}
    } else if (state == R_ROTATING_TO_ANGLE) {
		// We're rotating to a target angle

		if (elapsedStateTime > 5.0f) {
			// Failsafe - don't get stuck here.
			cmdStop(msg);
			shouldSend = true;

			setState(R_CALIBRATING_ANGLE);
		} else if (!atRotation()) {
			// Too far from angle, keep moving.
			cmdRot(msg, targetRot, avgRot);
			shouldSend = true;
		} else {
			// Close enough to angle, wait to see if the robot stays close enough.
			setState(R_WAITING_ANGLE);
		}
	} else if (state == R_WAITING_ANGLE) {
		// We're waiting after issuing rotate commands

		if (elapsedStateTime > kAngleWaitSec) {
			// We've waited long enough, let's move on to positioning.
			cmdStop(msg, false);
			shouldSend = true;

			setState(R_READY_TO_POSITION);
		} else if (!atRotation()) {
			// We're out of tolerance, try calibrating again.
			setState(R_CALIBRATING_ANGLE);
		}
	} else if (state == R_READY_TO_POSITION) {
		// Pass - wait for controller to give the go-ahead.
		cmdStop(msg, false);
		shouldSend = true;
    } else if (state == R_POSITIONING) {
        // move in direction at magnitude
        if (inPosition(planePos)) {
            cmdStop(msg, false);
            mustSend = true;
            setState(R_WAIT_AFTER_POSITION);
        } else {
            // move is different from draw
            moveRobot(msg, false, shouldSend);
        }
    } else if (state == R_WAIT_AFTER_POSITION) {
        cmdStop(msg);
        shouldSend = true;

        if (elapsedStateTime > 0.5f && !inPosition(avgPlanePos)) {
			// Go back, we're out of position.
            setState(R_POSITIONING);
        } else if (elapsedStateTime > 1.5f) {
			// We've waited long enough, start drawing.
            setState(R_READY_TO_DRAW);
        }
    } else if (state == R_READY_TO_DRAW) {
        // Pass - wait to be sent to drawing.
		cmdStop(msg, false);
		shouldSend = true;
    } else if (state == R_DRAWING) {
        if (inPosition(planePos)) {
			cmdStop(msg);
			shouldSend = true;

            setState(R_DONE_DRAWING);
        } else {
            moveRobot(msg, true, shouldSend);
        }
    } else if (state == R_DONE_DRAWING) {
        cmdStop(msg);
        shouldSend = true;
    } else if (state == R_STOPPED) {
		// We're stopped. Stop.
		cmdStop(msg);
		shouldSend = true;
	}

	// Only send messages every so often to avoid hammering the Arduino.
	if (mustSend || (shouldSend && ofGetFrameNum() % 4 == 0)) {
		sendMessage(msg);
	}
}
