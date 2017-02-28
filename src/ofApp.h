#pragma once

#include "ofMain.h"
#include "ofxDatGui.h"
#include "ofxOsc.h"
#include "ofxUDPManager.h"
#include "ofxJSON.h"

#include "Robot.h"
#include "Map.h"

#define PORT 5100

typedef enum MaproomState {
	MR_RUNNING,
	MR_PAUSED,
	MR_STOPPED
} MaproomState;

typedef struct RobotGui {
	ofxDatGuiFolder *folder;

	ofxDatGuiLabel *stateLabel, *posLabel;
	ofxDatGuiToggle *enableToggle;

	ofxDatGuiButton *calibrateButton, *advanceButton;
	ofxDatGuiSlider *rotationAngleSlider;
} RobotGui;

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void exit();

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
    
        // DatGui events
        void onButtonEvent(ofxDatGuiButtonEvent e);
        void onToggleEvent(ofxDatGuiToggleEvent e);
        void onSliderEvent(ofxDatGuiSliderEvent e);
        void onTextInputEvent(ofxDatGuiTextInputEvent e);

	void setState(MaproomState newState);
	string stateString();
	void updateGui();

	void handleOSC();
	void receiveFromRobots();
	void commandRobots();
    
    void loadMap(const string &name);
    void loadNextPath(Robot* r);

	void unclaimPath(int robotId);

private:
	ofEasyCam cam;

	ofVec3f cameraPos;
	ofVec3f cameraLook;
	ofMatrix4x4 cameraToWorld, cameraToWorldInv;

	ofxOscReceiver oscReceiver;
	ofxJSONElement jsonMsg;

	ofxUDPManager robotReceiver;
	char robotMessage[1024];

	map<int, Robot*> robotsById;
	map<int, Robot*> robotsByMarker;
	map<int, MapPath*> robotPaths;

	map<int, vector<ofVec2f>> robotPositions;
	map<int, int> robotPositionsCount;
	map<int, int> robotPositionsIdx;

	ofxDatGui *gui;
	ofxDatGuiLabel *stateLabel;
	ofxDatGuiButton *startButton, *pauseButton, *stopButton;
	map<int, RobotGui> robotGuis;
    
    Map *currentMap;

	MaproomState state;
	float stateStartTime;
};
