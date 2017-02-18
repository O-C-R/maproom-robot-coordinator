#pragma once

#include "ofMain.h"
#include "ofxSVG.h"
#include "ofxDatGui.h"
#include "ofxOsc.h"
#include "ofxUDPManager.h"
#include "ofxJSON.h"

#define PORT 5100

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

typedef struct Robot {
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

	void setCommunication(const string &rIp, int rPort) {
		ip = rIp;
		port = rPort;

		socket.Create();
		socket.Connect(ip.c_str(), port);
		socket.SetNonBlocking(true);
	}

	void sendMessage(const string &message) {
		socket.Send(message.c_str(), message.length());
		lastMessage = message;
	}

	void sendHeartbeat() {
		sendMessage("MRHB");
	}
} Robot;

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

	void handleOSC();
	void commandRobots();

private:
	ofEasyCam cam;

	ofVec3f cameraPos;
	ofVec3f cameraLook;
	ofMatrix4x4 cameraToWorld;

	ofxUDPManager robot01;
	std::string lastMessage;
	float robotRotRolling;

	ofxOscReceiver oscReceiver;
	ofxJSONElement jsonMsg;

	map<int, Robot*> robotsById;
	map<int, Robot*> robotsByMarker;

	ofxDatGui *gui;
	ofxDatGuiToggle *robot01GoRot, *robot01GoPos;
	ofxDatGui2dPad *robotTargetPad, *robotPosPad;
	ofxDatGuiSlider *robotTargetRot, *robotCurRot;
	ofxDatGuiSlider *robotRotAccuracy, *robotPosAccuracy;
	ofxDatGuiSlider *robotMovStrength, *robotRotStrength;

	ofxSVG mapSvg;

};
