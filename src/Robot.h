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
#include "VecExt.h"
#include "Map.h"
#include "Constants.h"

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
	R_WAITING_TO_POSITION,
	R_POSITIONING,
	R_POSITIONING_WITH_FIELD,
    R_WAIT_AFTER_POSITION,
    R_DONE_POSITIONING,
	R_WAITING_TO_DRAW,
	R_DRAWING,
    R_DONE_DRAWING,
	R_STOPPED
} RobotState;

typedef enum RobotPathState {
	RP_NONE,
	RP_WAITING
} RobotPathState;

typedef enum PenState {
	P_UNKNOWN,
	P_UP,
	P_DOWN
} PenState;

struct PotentialFieldObstacle {
	ofVec2f pos;
	float radius;
};

// Holy magic numbers batman
struct PotentialField {
	ofRectangle walls;
	vector<PotentialFieldObstacle> obstacles;
	ofVec2f goal;

	float fieldAtPoint(const ofVec2f &point) {
		float score = 0;

		for (auto &obstacle : obstacles) {
			float distanceToObstacle = point.distance(obstacle.pos);

			if (distanceToObstacle < obstacle.radius) {
				score += 1000;
			}

			score += 10.0 / pow(distanceToObstacle, 3.0);
		}

		if (!walls.inside(point)) {
			score += 1000.0 * pow(walls.getCenter().distance(point), 2.0) + 10000.0;
		} else {
			float distT = abs(walls.getTop() - point.y);
			float distB = abs(walls.getBottom() - point.y);
			float distL = abs(walls.getLeft() - point.x);
			float distR = abs(walls.getRight() - point.x);
			float minDist = min(distT, min(distB, min(distL, distR))) + 0.0000001;

			score += 0.1 * 1.0 / pow(minDist, 3.0);
		}


		float distanceToGoal = point.distance(goal) + 0.0000001;
		score -= 100.0 / pow(distanceToGoal, 2.0);

		return score;
	}
};

class Robot {

public:
	Robot(int rId, int mId, const string &n);

	// Setup communication - must be called before sending any messages
	void setCommunication(const string &rIp, int rPort);
	void sendMessage(const string &message);
	void sendHeartbeat();
	void gotHeartbeat();

	// Update from CV
	void updateCamera(const ofVec2f &imPos, const ofVec2f &imUp);

	// Update simulation
	void updateSimulation(float dt);

	// Update during loop
	void update();
	void setState(RobotState newState);
	void updatePID(float kp, float ki, float kd, float maxI);

	// States
	void moveRobot(char *msg, bool drawing, bool &shouldSend);
	void moveWithGradient(char *msg, bool &shouldSend);
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
	void navigateWithGradientTo(const ofVec2f &target);
	void updateObstacles(const vector<PotentialFieldObstacle> &obstacles);
	void drawLine(const ofVec2f &start, const ofVec2f &end);
    
    // test commands
    void testRotate(float angle);
    void testMove(float direction, float magnitude);

	// Paths
	void addPathType(const string &pathType);
	void removePathType(const string &pathType);

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
	ofVec2f fieldDirection;

	// Path planning
	MapPath *claimedMp;
	bool needsReplan;
	ofVec2f finalTarget;
	vector<ofVec2i> path;
	int pathIdx;
	vector<ofVec2i> drawPathMask;

	// PID
	float minSpeed, maxSpeed, speedRamp;
	MiniPID targetLinePID;
	float targetLineKp, targetLineKi, targetLineKd, targetLineMaxI;

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
	set<string> pathTypes;
    float lastHeading;

	PotentialField potentialField;
};

#endif
