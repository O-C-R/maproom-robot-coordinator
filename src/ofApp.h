#pragma once

#include "ofMain.h"
#include "ofxDatGui.h"
#include "ofxOsc.h"
#include "ofxUDPManager.h"
#include "ofxJSON.h"

#include "Robot.h"
#include "Map.h"

#define PORT 5100

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
    
        // DatGui events
        void onButtonEvent(ofxDatGuiButtonEvent e);
        void onToggleEvent(ofxDatGuiToggleEvent e);
        void onSliderEvent(ofxDatGuiSliderEvent e);
        void onTextInputEvent(ofxDatGuiTextInputEvent e);
    
    void updateGui();
	void handleOSC();
	void receiveFromRobots();
	void commandRobots();
    void robotConductor();

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

	ofxDatGui *gui;
    
    Map *currentMap;
};
