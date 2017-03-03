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
#include "MiniPID.h"

static const float kMetersPerInch = 0.0254;
static const float kMarkerSizeIn = 5.0;
static const float kMarkerSizeM = kMarkerSizeIn * kMetersPerInch;

typedef enum RobotState {
	R_NO_CONN,
	R_START,
	R_CALIBRATING_ANGLE,
	R_ROTATING_TO_ANGLE,
	R_WAITING_ANGLE,
	R_READY_TO_POSITION,
	R_POSITIONING,
    R_WAIT_AFTER_POSITION,
    R_READY_TO_DRAW,
	R_DRAWING,
    R_DONE_DRAWING,
	R_STOPPED
} RobotState;

typedef enum PenState {
	P_UNKNOWN,
	P_UP,
	P_DOWN
} PenState;

class Robot {

public:
	Robot(int rId, int mId, const string &n);

	// Setup communication - must be called before sending any messages
	void setCommunication(const string &rIp, int rPort);
	void sendMessage(const string &message);
	void sendHeartbeat();
	void gotHeartbeat();

	// Update from CV
	void updateCamera(const ofVec2f &imPos, const ofVec2f &imUp, const ofVec2f &opticalCenter, const ofVec2f &opticalScale);

	// Update during loop
	void update();
	void setState(RobotState newState);

	// States
	void moveRobot(char *msg, bool drawing, bool &shouldSend);
    bool inPosition(const ofVec2f &pos);
	bool atRotation();

	// Query robot state
	bool commsUp();
	bool cvDetected();
	string stateString();
	string stateDescription();
	string positionString();

    // Set robot commands
    void calibrate();
    void stop();
    void start();
    
    // nav states
	void navigateTo(const ofVec2f &target);
    void drawLine(const ofVec2f &start, const ofVec2f &end);
    
    // test commands
    void testRotate(float angle);
    void testMove(float direction, float magnitude);

// --------------------------------------
// -------------- DATA ------------------
// --------------------------------------

	// Required
	int id;
	string name;
	int markerId;
    
	// State machine
	RobotState state;
	float stateStartTime;

	// Targets
	float targetRot;
	ofVec2f startPlanePos, targetPlanePos;
	MiniPID targetLinePID;

	// Communication
	bool enabled;
	string ip;
	int port;
	ofxUDPManager socket;
	string lastMessage;
	float lastHeartbeatTime;
	char msg[128];

	// Received from CV
	ofVec3f imgPos, upVec;
	float lastCameraUpdateTime;
	float cvFramerate;
    
	// Derived from CV
	ofVec2f planePos, avgPlanePos, slowAvgPlanePos, allAvgPlanePos;
	ofVec2f planeVel, avgPlaneVel;
	float rot, avgRot;
	float glRot, avgGlRot;

	// debug: visualizing states
	ofVec2f dirToLine, backToLine, vecToEnd, movement;
    
    // used for map next path
    float lastHeading;
};

#endif
