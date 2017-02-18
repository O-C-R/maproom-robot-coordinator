#pragma once

#include "ofMain.h"
#include "ofxSVG.h"
#include "ofxDatGui.h"
#include "ofxOsc.h"
#include "ofxUDPManager.h"
#include "ofxJSON.h"

#include "Robot.h"

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

	void handleOSC();
	void commandRobots();

private:
	ofEasyCam cam;

	ofVec3f cameraPos;
	ofVec3f cameraLook;
	ofMatrix4x4 cameraToWorld, cameraToWorldInv;

	ofxOscReceiver oscReceiver;
	ofxJSONElement jsonMsg;

	map<int, Robot*> robotsById;
	map<int, Robot*> robotsByMarker;

//	ofxDatGui *gui;

	ofxSVG mapSvg;

};
