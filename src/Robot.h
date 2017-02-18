//
//  Robot.h
//  maproom-robot
//
//  Created by Christopher Anderson on 2/18/17.
//
//

#ifndef Robot_h
#define Robot_h

#include "ofMain.h"
#include "ofxUDPManager.h"

typedef enum RobotState {
	R_START,
	R_CALIBRATING_ANGLE,
	R_ROTATING_TO_ANGLE,
	R_MOVING,
	R_DRAWING,
	R_STOPPED
} RobotState;

typedef enum PenState {
	P_UNKNOWN,
	P_UP,
	P_DOWN
} PenState;

void cmdCalibrateAngle(char *buf, int measured) {
	sprintf(buf, "MRCAL%+5d", measured);
}

void cmdRot(char *buf, int measured, int target) {
	sprintf(buf, "MRROT%+5d%+5d", measured, target);
}

void cmdMove(char *buf, int angle, int magnitude) {
	sprintf(buf, "MRMOV%+5d%+5d", angle, magnitude);
}

void cmdDraw(char *buf, int angle, int magnitude) {
	sprintf(buf, "MRDRW%+5d%+5d", angle, magnitude);
}

void cmdStop(char *buf) {
	sprintf(buf, "MRSTP");
}

class Robot {

public:
	// Required
	int id;
	string name;
	int markerId;

	// State machine
	RobotState state;
	PenState penState;

	// Targets
	float targetRot;
	ofVec2f targetPlanePos;

	// Communication
	string ip;
	int port;
	ofxUDPManager socket;
	string lastMessage;
	float lastHeartbeatTime;

	// Received from CV
	ofVec3f tvec, rvec;
	float lastUpdateTime;

	// Derived from CV
	ofMatrix4x4 mat;
	ofVec3f worldPos;
	ofVec2f planePos;
	float rot;

	Robot(int rId, int mId, const string &n) : id(rId), markerId(mId), name(n), state(R_START), penState(P_UNKNOWN) {}

	void setCommunication(const string &rIp, int rPort);

	void update(const ofVec3f &rvec, const ofVec3f &tvec, const ofMatrix4x4 &cameraWorldInv);

	void sendMessage(const string &message);
	void sendHeartbeat();
};

#endif