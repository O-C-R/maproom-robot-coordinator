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

static const float kMetersPerInch = 0.0254;
static const float kMarkerSizeIn = 5.0;
static const float kMarkerSizeM = kMarkerSizeIn * kMetersPerInch;

typedef enum RobotState {
	R_START,
	R_CALIBRATING_ANGLE,
	R_ROTATING_TO_ANGLE,
	R_WAITING_ANGLE,
	R_MOVING,
	R_DRAWING,
	R_STOPPED,
	R_NO_CONN
} RobotState;

typedef enum PenState {
	P_UNKNOWN,
	P_UP,
	P_DOWN
} PenState;

class Robot {

public:
	// Required
	int id;
	string name;
	int markerId;

	// State machine
	RobotState state;
	PenState penState;
	float stateStartTime;
	int positionIdx;

	// Targets
	float targetRot;
	ofVec2f targetPlanePos;
    
    // values to send
    float moveDir;
    float moveMag;

	// Communication
    bool enableMessages;
    string ip;
	int port;
	ofxUDPManager socket;
	string lastMessage;
	float lastHeartbeatTime;
	char msg[128];

	// Received from CV
	ofVec3f tvec, rvec;
	float lastCameraUpdateTime;

	// Derived from CV
	ofMatrix4x4 mat;
	ofVec3f worldPos;
	ofVec2f planePos;
	float rot;

	Robot(int rId, int mId, const string &n) :
		id(rId),
		markerId(mId),
		name(n),
		state(R_NO_CONN),
        enableMessages(false),
		penState(P_UNKNOWN),
		targetRot(120),
		targetPlanePos(0, 0),
		stateStartTime(0),
		lastCameraUpdateTime(-1000),
		lastHeartbeatTime(-1000),
		positionIdx(0)
	{}

	// Setup communication - must be called before sending any messages
	void setCommunication(const string &rIp, int rPort);
	void sendMessage(const string &message);
	void sendHeartbeat();
	void gotHeartbeat();

	// Update from CV
	void updateCamera(const ofVec3f &rvec, const ofVec3f &tvec, const ofMatrix4x4 &cameraWorldInv);

	// Update during loop
	void update();
	void setState(RobotState newState);

	// States
	void stateMove(char *msg, bool &shouldSend);

	// Query robot state
	bool commsUp();
	bool cvDetected();
    
    // Set robot commands
    void calibrate();
    void stop();
    
    // test commands
    void testRotate(float angle);
    void testMove(float direction, float magnitude);

};

#endif
