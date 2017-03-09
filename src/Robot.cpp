//
//  Robot.cpp
//  maproom-robot
//
//  Created by Christopher Anderson on 2/18/17.
//
//

#include "Robot.h"

static const bool debugging = false;

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
	return constrainTo360(360 - ofRadToDeg(rad) + 90);
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
	minSpeed(100),
	maxSpeed(512),
	speedRamp(0.1),
	planeVel(0, 0),
	avgPlaneVel(0, 0),
	targetRot(0),
	targetPlanePos(0, 0),
	rot(0),
	avgRot(0),
	glRot(0),
	avgGlRot(),
	stateStartTime(0),
	lastCameraUpdateTime(-1000),
	cvFramerate(0),
	lastHeartbeatTime(-1000),
	targetLineKp(20000.0),
	targetLineKi(500),
	targetLineKd(0),
	targetLineMaxI(5000.0),
	targetLinePID(targetLineKp, targetLineKi, targetLineKd)
{
	targetLinePID.setMaxIOutput(targetLineMaxI);

}

void Robot::updatePID(float kp, float ki, float kd, float maxI) {
	targetLineKp = kp;
	targetLineKi = kd;
	targetLineKd = ki;
	targetLineMaxI = maxI;

	targetLinePID.setPID(kp, kd, ki);
	targetLinePID.setMaxIOutput(targetLineMaxI);
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
}

void Robot::start() {
	if (state != R_START) {
		setState(R_START);
	}
}

string Robot::stateDescription() {
	return stateString() + " (" + (commsUp() ? "CONN" : "DISCONN") + " " + (cvDetected() ? "SEEN" : "HIDDEN") + ")";
}

string Robot::positionString() {
	char buf[512];
	sprintf(buf, "(%+07.1f, %+07.1f) @ %03.1f%c (%.1f fps)",
			planePos.x * 100.0, planePos.y * 100.0,
			rot, char(176),
			cvFramerate);
	return buf;
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
		case R_WAITING_TO_POSITION:
			return "R_WAITING_TO_POSITION";
		case R_READY_TO_POSITION:
			return "R_READY_TO_POSITION";
		case R_POSITIONING:
			return "R_POSITIONING";
		case R_POSITIONING_WITH_FIELD:
			return "R_POSITIONING_WITH_FIELD";
		case R_WAIT_AFTER_POSITION:
			return "R_WAIT_AFTER_POSITION";
		case R_DONE_POSITIONING:
			return "R_DONE_POSITIONING";
		case R_WAITING_TO_DRAW:
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

void Robot::updateCamera(const ofVec2f &imPos, const ofVec2f &imUp) {
	imgPos = imPos;
	upVec = imUp;

	ofVec2f newPlanePos = imPos;
	glRot = atan2(imUp.y, imUp.x);
	rot = ofRadToRobotDeg(glRot);

	allAvgPlanePos += (newPlanePos - allAvgPlanePos) * 0.5;

	const float now = ofGetElapsedTimef();
	const float dt = now - lastCameraUpdateTime;

	// Calculate delta velocity first
	if (dt > 0) {
		planeVel = (newPlanePos - planePos) / dt;
		avgPlaneVel += (avgPlaneVel - planeVel) * 0.25;
	}

	// Update positions and previous averages
	planePos.set(newPlanePos);
	avgPlanePos += (planePos - avgPlanePos) * 0.5;

	avgRot += ofAngleDifferenceDegrees(avgRot, rot) * 0.5;
	avgRot = constrainTo360(avgRot);
	avgGlRot += ofAngleDifferenceRadians(avgGlRot, glRot) * 0.1;
	avgGlRot = fmod(avgGlRot +  3.1415964 * 2.0, 3.1415964 * 2.0);

	const float framerate = 1.0 / dt;
	cvFramerate += (framerate - cvFramerate) * 0.1;
	lastCameraUpdateTime = now;
}

void Robot::updateSimulation(float dt) {
	const float unitsPerSec = 0.05;
	if (state == R_POSITIONING || state == R_DRAWING) {
		// TODO: add noise?
		planePos += (targetPlanePos - startPlanePos).normalize() * dt * unitsPerSec;
	} else if (state == R_POSITIONING_WITH_FIELD) {
		planePos += fieldDirection * dt * unitsPerSec;
	}

	updateCamera(planePos, upVec);
}

void Robot::gotHeartbeat() {
	lastHeartbeatTime = ofGetElapsedTimef();
}

bool Robot::commsUp() {
	if (SIMULATING) {
		return true;
	} else {
		return ofGetElapsedTimef() - lastHeartbeatTime < kHeartbeatTimeoutSec;
	}
}

bool Robot::cvDetected() {
    if (debugging) return true;
	return ofGetElapsedTimef() - lastCameraUpdateTime < kCameraTimeoutSec;
}

void Robot::setState(RobotState newState) {
	if (newState == state) {
		return;
	}

	cout << "Robot " << id << ": " << stateString() << " -> ";
	state = newState;
	cout << stateString() << endl;

	// Reset PID on all state changes
	targetLinePID.reset();

	stateStartTime = ofGetElapsedTimef();
}

void Robot::moveWithGradient(char *msg, bool &shouldSend) {
	float xL = potentialField.fieldAtPoint(planePos + ofVec2f(-0.001f, 0));
	float xR = potentialField.fieldAtPoint(planePos + ofVec2f( 0.001f, 0));
	float yU = potentialField.fieldAtPoint(planePos + ofVec2f(0, -0.001f));
	float yD = potentialField.fieldAtPoint(planePos + ofVec2f(0,  0.001f));

	float xSlope = xR - xL;
	float ySlope = yD - yU;

	fieldDirection = ofVec2f(-xSlope, -ySlope).normalize();
	vecToEnd = fieldDirection * 20.0f;

	// TODO: send message
}

void Robot::moveRobot(char *msg, bool drawing, bool &shouldSend) {
	// Vectors for movement - ideal and remaining
    const ofVec2f line = targetPlanePos - startPlanePos;
	ofVec2f currentToEnd = targetPlanePos - planePos;

	// Calculate where and how fast we'd go to just get to the end
	const float distanceToEnd = currentToEnd.length();
	float forwardMag = ofMap(distanceToEnd, 0, speedRamp, minSpeed, maxSpeed, true);
	const ofVec2f currentToEndDir = (line.dot(currentToEnd) / line.lengthSquared() * line).normalize();
	vecToEnd = currentToEndDir * forwardMag;

	// Calculate how far we are from the line and how we should correct for that
	dirToLine = vecPtToLine(planePos, startPlanePos, targetPlanePos);
	const float distToLine = dirToLine.length();
	const double targetLinePIDOutput = targetLinePID.getOutput(distToLine, 0.0);
	backToLine = targetLinePIDOutput * ofVec2f(dirToLine).normalize() * -1.0;

	// Apply a weighting where the line following is stronger at the start
	backToLine *= ofMap(distanceToEnd, 0, 0.2, 0.5, 2.0, true);

	// Combine the two vectors
	movement = (vecToEnd + backToLine).normalize() * forwardMag;
	float angle = ofRadToRobotDeg(atan2(movement.y, movement.x));
	const float mag = movement.length();

	if (distanceToEnd < 0.02f) {
		// If we're really close to the end, just get there.
		currentToEnd.normalize();
		angle = ofRadToRobotDeg(atan2(currentToEnd.y, currentToEnd.x));
	}

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
	return abs(ofAngleDifferenceDegrees(targetRot, rot)) < kRotationTolerance;
}

bool Robot::inPosition(const ofVec2f &pos) {
	return targetPlanePos.distance(pos) < kPositionTolerance;
}

void Robot::navigateTo(const ofVec2f &target) {
	startPlanePos = avgPlanePos;
	targetPlanePos = target;

    setState(R_POSITIONING);
}

void Robot::navigateWithGradientTo(const ofVec2f &target) {
	potentialField.goal = target;
	startPlanePos = avgPlanePos;
	targetPlanePos = target;

	setState(R_POSITIONING_WITH_FIELD);
}

void Robot::updateObstacles(const vector<PotentialFieldObstacle> &obstacles) {
	potentialField.obstacles = obstacles;
}

void Robot::drawLine(const ofVec2f &start, const ofVec2f &end) {
	startPlanePos = start;
	targetPlanePos = end;

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

		if (elapsedStateTime < 0.5f) {
			cmdStop(msg);
			shouldSend = true;
		} else if (elapsedStateTime >= 3.0f) {
			// We've calibrated enough
            setState(R_ROTATING_TO_ANGLE);
		} else {
			cmdCalibrateAngle(msg, avgRot);
			shouldSend = true;
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

		if (elapsedStateTime > 1.0f && !atRotation()) {
			// We're out of tolerance, try calibrating again.
			setState(R_CALIBRATING_ANGLE);
		} else if (elapsedStateTime > kAngleWaitSec) {
			// We've waited long enough, let's move on to positioning.
			cmdStop(msg, false);
			shouldSend = true;

			setState(R_READY_TO_POSITION);
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
	} else if (state == R_POSITIONING_WITH_FIELD) {
		// move in direction at magnitude
		if (inPosition(planePos)) {
			cmdStop(msg, false);
			mustSend = true;
			setState(R_WAIT_AFTER_POSITION);
		} else {
			// move is different from draw
			moveWithGradient(msg, shouldSend);
		}
    } else if (state == R_WAIT_AFTER_POSITION) {
        cmdStop(msg);
        shouldSend = true;

        if (elapsedStateTime > 0.15f && !inPosition(avgPlanePos)) {
			// Go back, we're out of position.
            setState(R_POSITIONING_WITH_FIELD);
        } else if (elapsedStateTime > 0.3f) {
			// We've waited long enough, start drawing.
            setState(R_DONE_POSITIONING);
        }
    } else if (state == R_DONE_POSITIONING) {
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

void Robot::addPathType(const string &pathType) {
	pathTypes.insert(pathType);
}

void Robot::removePathType(const string &pathType) {
	pathTypes.erase(find(pathTypes.begin(), pathTypes.end(), pathType));
}
