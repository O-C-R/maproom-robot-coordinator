#pragma once 

#include "ofMain.h"
#include "ofxDatGui.h"
#include "ofxOsc.h"
#include "ofxUDPManager.h"
#include "ofxJSON.h"

#include "Constants.h"
#include "Robot.h"
#include "Map.h"
#include "ArucoMarker.h"

#define PORT 5100

typedef enum MaproomState {
	MR_RUNNING,
	MR_PAUSED,
	MR_STOPPED
} MaproomState;

typedef enum RPiState {
	RPI_UNKNOWN = -1,
	RPI_TRACKING = 0,
	RPI_FLASHLIGHT = 1,
	N_RPI_STATES = 2
} RPiState;

typedef struct RobotGui {
	ofxDatGuiFolder *folder;

	ofxDatGuiLabel *stateLabel, *posLabel;
	ofxDatGuiLabel *lastMessageLabel;
	ofxDatGuiToggle *enableToggle;

	ofxDatGuiButton *calibrateButton, *advanceButton;
	ofxDatGuiSlider *rotationAngleSlider;
} RobotGui;

typedef struct PathGui {
	string pathType;
	ofxDatGuiFolder *folder;
    ofxDatGuiToggle *toggle;

    vector<ofxDatGuiToggle *> robotSelect;
//    ofxDatGuiDropdown *drawOptions;
} PathGui;

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

	void setState(MaproomState newState);
	string stateString();
	void updateGui();
    
	void handleOSC();
	void receiveFromRobots();
	void commandRobots();
	void sendRobotsToCorners();

	void loadMap(const string &newMapPath);
	void setupMapGui();
    
	void unclaimPath(int robotId);
    
    // path gui
    // int dropdown_index, int robot_id
    map <int, int> dropDownToRobotId;


private:
	ofEasyCam cam;

	ofxOscReceiver oscReceiver;
	ofxOscSender oscToRPi;
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
    ofxDatGuiLog *guiLogger;
    ofxDatGui *pathGui;
	ofxDatGuiLabel *stateLabel, *pathLabel, *drawnPathLabel, *pathStatusLabel;
	ofxDatGuiButton *startButton, *pauseButton, *stopButton;
	ofxDatGuiLabel *rpiStateLabel;
	ofxDatGuiDropdown *rpiStateDropdown;
	ofxDatGuiFolder *robotConstantsFolder;
	ofxDatGuiSlider *kpSlider, *kiSlider, *kdSlider, *kMaxISlider;
	ofxDatGuiSlider *minSpeedSlider, *maxSpeedSlider, *speedRampSlider;
	map<int, RobotGui> robotGuis;
	map<string, PathGui> pathGuis;

	map<int, ArucoMarker> markersById;

	string mapPath;
    Map *currentMap;

	MaproomState state;
	float stateStartTime;

	RPiState rpiState;
	float rpiLastStateMessageTime;
};
