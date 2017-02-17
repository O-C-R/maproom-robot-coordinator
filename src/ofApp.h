#pragma once

#include "ofMain.h"
//#include "ofxCv.h"
#include "ofxSVG.h"
#include "ofxDatGui.h"
#include "ofxOsc.h"
#include "ofxUDPManager.h"
#include "ofxJSON.h"

#define PORT 5100

//#include <opencv2/aruco/charuco.hpp>
typedef struct {
	int markerIdx;

	ofVec3f tvec, rvec;
	ofMatrix4x4 mat;

	ofVec3f worldPos;
	float rot;
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
	map<int, Robot> robots;

	ofxDatGui *gui;
	ofxDatGuiToggle *robot01GoRot, *robot01GoPos;
	ofxDatGui2dPad *robotTargetPad, *robotPosPad;
	ofxDatGuiSlider *robotTargetRot, *robotCurRot;
	ofxDatGuiSlider *robotRotAccuracy, *robotPosAccuracy;
	ofxDatGuiSlider *robotMovStrength, *robotRotStrength;

	ofxSVG mapSvg;

};
