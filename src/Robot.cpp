//
//  Robot.cpp
//  maproom-robot
//
//  Created by Christopher Anderson on 2/18/17.
//
//

#include "Robot.h"

static const float kTolerance = 0.01f;

static const float kHeartbeatTimeoutSec = 2.0f;
static const float kCameraTimeoutSec = 0.5f;

static const float kPenMovementTime = 0.1f;

static const float kCalibrationWaitSec = 1.0f;
static const float kAngleWaitSec = 2.0f;

static const float kMoveWaitSec = 2.0f;

static const float kRotationToleranceStart = 10.0f;
static const float kRotationToleranceFinal = 3.0f;

const static float kSqSizeM = 0.25;
const static int kNumPositions = 5;

const static ofVec2f positions[] = {
	{ 0.0, 0.0 },
	{ -kSqSizeM, -kSqSizeM },
	{  kSqSizeM, -kSqSizeM },
	{  kSqSizeM,  kSqSizeM },
	{ -kSqSizeM,  kSqSizeM },
	{ 0.0, 0.0 }
};

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
    setState(R_STOPPED);
    cmdStop(msg, (penState == P_UP ? true : false));
	sendMessage(msg);
}

void Robot::start() {
    setState(R_START);
}

void Robot::testRotate(float angle) {
    targetRot = angle;
    setState(R_ROTATING_TO_ANGLE);
	cmdRot(msg, angle, rot);
	sendMessage(msg);
}

void Robot::testMove(float direction, float magnitude) {
    moveDir = direction;
    moveMag = magnitude;
//    setState(R_MOVING);

	cmdMove(msg, direction, magnitude, rot);
	sendMessage(msg);
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

	// Translate to the center of the marker
    
    planePos.set(worldPos.x, worldPos.y);
    
    // offset center of the robot
//    ofVec3f center = ofVec3f(kMarkerSizeM / 2.0, kMarkerSizeM / 2.0, 0.0)*mat;
//    planePos.set(center.x, center.y);
    
    
	// Get z-axis rotation
	ofVec3f angAxis = ofVec3f(0.0, 1.0, 0.0) * mat;
	ofVec2f axisVec = ofVec2f(angAxis.x, angAxis.y) - ofVec2f(worldPos.x, worldPos.y);
	float axisAngle = atan2(axisVec.y, axisVec.x);

	// rotation is right-handed, and goes from 0-360 starting on the +x axis
	rot = fmod(ofRadToDeg(axisAngle) + 360 - 90, 360.0);

	lastCameraUpdateTime = ofGetElapsedTimef();
}

void Robot::gotHeartbeat() {
	lastHeartbeatTime = ofGetElapsedTimef();
}

bool Robot::commsUp() {
    return true;
	return ofGetElapsedTimef() - lastHeartbeatTime < kHeartbeatTimeoutSec;
}

bool Robot::cvDetected() {
	return ofGetElapsedTimef() - lastCameraUpdateTime < kCameraTimeoutSec;
}

void Robot::setState(RobotState newState) {
	cout << "State change " << state << " to " << newState << endl;
	state = newState;
	stateStartTime = ofGetElapsedTimef();
}

void Robot::rotateToDraw(char *msg, ofVec2f target, bool &shouldSend) {
    ofVec2f dir = navState.end - navState.start;
//    dir.normalize();
    float rad = atan2(dir.y, dir.x);
    float heading = fmod(ofRadToDeg(rad) + 360, 360.0);
    
    // given heading, where do we rotate?
//    float idealOffset = fmod(heading - rot, 120.0);
//    float newAngle;
//    if (heading > rot) {
//        newAngle = fmod(rot - idealOffset, 360.0);
//    } else {
//        newAngle = fmod(rot + idealOffset, 360.0);
//    }
//    cout << "new angle: " << newAngle << endl;
    targetRot = heading;
    cout << "targetRot: " << targetRot << endl;
    shouldSend = true;
}

void Robot::moveRobot(char *msg, ofVec2f target, bool drawing, bool &shouldSend) {
	ofVec2f dir = target - planePos;
	float len = dir.length();
    
	dir.normalize();
	float mag = ofMap(len, 0, 1, 65, 250, true);
	float rad = atan2(dir.y, dir.x);
	float angle = fmod(ofRadToDeg(rad) + 360 - 90, 360.0);
    
    if (drawing) {
        cout << "DRAWING" << endl;
        cmdDraw(msg, angle, mag, rot);
    } else {
        cout << "MOVING" << endl;
        cmdMove(msg, angle, mag, rot);
    }
    
	shouldSend = true;
	cout << planePos << endl;
	cout << target << endl;
	cout << msg << endl;
}

bool Robot::inPosition(ofVec2f target) {
    ofVec2f dir = target - planePos;
    float len = dir.length();
    return (len < kTolerance ? true : false);
}

void Robot::setPathType(int pathType) {
    navState.pathType = pathType;
}

void Robot::startNavigation(ofVec2f start, ofVec2f end) {
    navState.start = start;
    navState.end = end;
    setState(R_POSITIONING);
}

void Robot::update() {
	bool shouldSend = false;

	float elapsedStateTime = ofGetElapsedTimef() - stateStartTime;
	float rotAngleDiff = ofAngleDifferenceDegrees(targetRot, rot);

	if (!enableMessages) {
		return;
	}

//	if (!commsUp()) {
//		if (state != R_NO_CONN) {
//			cout << "Comms down, moving to NO_CONN" << endl;
//			setState(R_NO_CONN);
//		}
//
//		cmdStop(msg, false);
//		shouldSend = true;
//	} else
    if (!cvDetected()) {
		if (state != R_NO_CONN) {
			cout << "CV down, moving to NO_CONN" << endl;
			setState(R_NO_CONN);
		}

		cmdStop(msg, false);
		shouldSend = true;
	} else if (state == R_NO_CONN) {
		// Now connected and seen!
		setState(R_START);

		cmdStop(msg, false);
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
            setState(R_POSITIONING);
		}
    } else if (state == R_ROTATING_TO_ANGLE) {
		// We're rotating to a target angle

		if (abs(rotAngleDiff) > kRotationToleranceFinal) {
			// Too far from angle, keep moving.
			cmdRot(msg, targetRot, rot);
			shouldSend = true;
		} else {
			// Close enough to angle, wait to see if the robot stays close enough.
			setState(R_WAITING_ANGLE);
		}
	} else if (state == R_WAITING_ANGLE) {
		// We're waiting after issuing rotate commands

		if (elapsedStateTime < kAngleWaitSec) {
			// Pass
		} else if (abs(rotAngleDiff) > kRotationToleranceFinal) {
			// We're out of tolerance, try rotating again.
			setState(R_ROTATING_TO_ANGLE);
		} else if (elapsedStateTime > kAngleWaitSec) {
			// We've waited long enough, stop.
			setState(R_STOPPED);

			cmdStop(msg, false);
			shouldSend = true;
		}
    } else if (state == R_POSITIONING) {
        cout << "STATE POSITIONING" << endl;
        // move in direction at magnitude
        if (inPosition(navState.start)) {
            cmdStop(msg, false);
            shouldSend = true;
            setState(R_WAITING_TO_ROTATE);
        } else {
            // move is different from draw
            moveRobot(msg, navState.start, false, shouldSend);
        }
    } else if (state == R_WAITING_TO_ROTATE) {
        cout << "WAITING TO ROTATE" << endl;
        cmdStop(msg, false);
        shouldSend = true;
        if (elapsedStateTime > 1.0f) {
//            setState(R_ROTATING_TO_DRAW);
            setState(R_WAITING_TO_DRAW);
        } else if (!inPosition(navState.start)) {
            setState(R_POSITIONING);
        }
    } else if (state == R_ROTATING_TO_DRAW) {
        cout << "ROTATING TO IDEAL ANGLE" << endl;
        ofVec2f dir = navState.end - navState.start;
        float rad = atan2(dir.y, dir.x);
        float heading = fmod(ofRadToDeg(rad) + 360 + 90, 360.0);
        targetRot = heading;
        cout << "targetRot: " << targetRot << endl;
        cmdRot(msg, targetRot, rot);
        shouldSend = true;
        rotAngleDiff = abs(fmod(ofAngleDifferenceDegrees(targetRot, rot), 360.0));
        cout << "rotAngleDiff: " << rotAngleDiff << endl;
        if (abs(rotAngleDiff) < kRotationToleranceStart) {
//            cmdStop(msg, false);
//            shouldSend = true;
            setRotating = false;
            setState(R_WAITING_DRAW_ANGLE);
        }
    }  else if (state == R_WAITING_DRAW_ANGLE) {
        if (!setRotating) {
            cmdRot(msg, targetRot, rot);
            shouldSend = true;
            setRotating = true;
        }
        cout << "WAITING FOR DRAW ANGLE" << endl;
        rotAngleDiff = abs(fmod(ofAngleDifferenceDegrees(targetRot, rot), 360.0));
        if (elapsedStateTime > 2.0f) {
            if (abs(rotAngleDiff) < kRotationToleranceFinal) {
                cmdStop(msg, false);
                shouldSend = true;
                setState(R_WAITING_TO_DRAW);
            } else {
                calibrate();
                if (elapsedStateTime > 2.25f) {
                    setState(R_ROTATING_TO_DRAW);
                }
            }
        } else if (abs(rotAngleDiff) > kRotationToleranceStart) {
            setState(R_ROTATING_TO_DRAW);
        }
    } else if (state == R_WAITING_TO_DRAW) {
        cout << "WAITING TO DRAW" << endl;
        // wait for okay to draw
        if (navState.drawReady) {
            if (elapsedStateTime > 1.0f) {
                setState(R_DRAWING);
                navState.drawReady = false;
            }
        }
    } else if (state == R_DRAWING) {
        cout << "DRAWING" << endl;
        if (inPosition(navState.end)) {
            setState(R_DONE_DRAWING);
        } else {
            moveRobot(msg, navState.end, true, shouldSend);
        }
    } else if (state == R_DONE_DRAWING) {
        cout << "DONE DRAWING" << endl;
        // we're stopped, pen is up
        cmdStop(msg, true);
        shouldSend = true;

		if (elapsedStateTime > 1.0f) {
			navState.readyForNextPath = true;
		}
    } else if (state == R_STOPPED) {
        cout << "STOPPED" << endl;
		// We're stopped. Stop.
		cmdStop(msg, false);
		shouldSend = true;
	}

	if (shouldSend) {
		shouldSend = ofGetFrameNum() % 4 == 0;
	}

	if (shouldSend && enableMessages) {
		sendMessage(msg);
	}
}
