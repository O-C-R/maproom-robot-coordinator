#include "ofApp.h"


static const int kFrameWait = 3;
static const int kNumCornersCalibration = 15;
static const int kImgWidth = 1980;
static const int kImgHeight = 1080;

static const int kRobot01Idx = 23;
static const float kMarkerSize = 0.2032f;

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetVerticalSync(true);
	ofSetBackgroundColor(0);

	cam.setDistance(3);
	cam.enableMouseInput();
	cam.setTarget(ofVec3f(0.0));
	cam.setNearClip(0.01);

	// on the pillar
//	cameraPos = ofVec3f(0., 0., 2.03);
//	cameraLook = ofVec3f(0.0, 0.82, 0.0);
//	cameraToWorld.makeLookAtMatrix(cameraPos, cameraLook, ofVec3f(0.0, 0.0, 1.0));

	// on the map hanging thing
	cameraPos = ofVec3f(0., 0., 2.845);
	cameraLook = ofVec3f(0.0, 0.0, 0.0);

	cameraToWorld.makeIdentityMatrix();
	ofQuaternion cameraRotation;
	cameraRotation.makeRotate(180, ofVec3f(1.0, 0.0, 0.0));
	cameraToWorld.setRotate(cameraRotation);
	cameraToWorld.setTranslation(cameraPos);

	mapSvg.load("map.svg");
	cout << "mapSvg " << mapSvg.getWidth() << "x" << mapSvg.getHeight() << " w/ " << mapSvg.getNumPath() << endl;

	gui = new ofxDatGui( ofxDatGuiAnchor::TOP_RIGHT );
	robot01GoRot = gui->addToggle("robot01GoRot", false);
	robot01GoPos = gui->addToggle("robot01GoPos", false);
	robotCurRot = gui->addSlider("robotCurRot", -180, 180);
	robotCurRot->setValue(0);
	robotTargetRot = gui->addSlider("robotTargetRot", -180, 180);
	robotTargetRot->setValue(0);
	robotTargetPad = gui->add2dPad("robotTargetPad", ofRectangle(ofVec2f(-2), ofVec2f(2)));
	robotPosPad = gui->add2dPad("robotPosPad", ofRectangle(ofVec2f(-2), ofVec2f(2)));
	robotRotAccuracy = gui->addSlider("robotRotAccuracy", 0, 30);
	robotRotAccuracy->setValue(5);
	robotPosAccuracy = gui->addSlider("robotPosAccuracy", 0, 1);
	robotPosAccuracy->setValue(0.1);
	robotMovStrength = gui->addSlider("robotMovStrength", 0, 1000);
	robotMovStrength->setValue(200);
	robotRotStrength = gui->addSlider("robotRotStrength", 0, 1000);
	robotRotStrength->setValue(70);

	// Connect to first robot
	robot01.Create();
	robot01.Connect("192.168.1.70",5111);
	robot01.SetNonBlocking(true);
	robotRotRolling = 0;

	// Listen for messages from camera
	oscReceiver.setup( PORT );
}

//--------------------------------------------------------------
void ofApp::update(){
	handleOSC();
	commandRobots();
}

void ofApp::handleOSC() {
	while( oscReceiver.hasWaitingMessages() )
	{
		// get the next message
		ofxOscMessage m;
		oscReceiver.getNextMessage( &m );

		// check for mouse moved message
		if ( m.getAddress() == "/cv" )
		{
			string msg = m.getArgAsString(0);
			jsonMsg.parse(msg);
			int nIds = jsonMsg["ids"].size();
			for (int i = 0; i < nIds; ++i) {
				int idx = jsonMsg["ids"][i].asInt();
				ofVec3f rvec(jsonMsg["rvecs"][i][0].asFloat(), jsonMsg["rvecs"][i][1].asFloat(), jsonMsg["rvecs"][i][2].asFloat());
				ofVec3f tvec(jsonMsg["tvecs"][i][0].asFloat(), jsonMsg["tvecs"][i][1].asFloat(), jsonMsg["tvecs"][i][2].asFloat());

				Robot r;
				r.markerIdx = idx;
				r.rvec = rvec;
				r.tvec = tvec;

				// http://answers.opencv.org/question/110441/use-rotation-vector-from-aruco-in-unity3d/
				float angle = sqrt(rvec.x*rvec.x + rvec.y*rvec.y + rvec.z*rvec.z);
				ofVec3f axis(rvec.x, rvec.y, rvec.z);

				ofQuaternion rot;
				rot.makeRotate(angle / 3.14159 * 180.0, axis);

				r.mat.makeIdentityMatrix();
				r.mat.setRotate(rot);
				r.mat.setTranslation(ofVec3f(tvec.x, tvec.y, tvec.z));
				r.mat = r.mat * cameraToWorld.getInverse();

				r.worldPos = r.mat.getTranslation();
				r.rot = r.mat.getRotate().getEuler().z;

				// Update GUI
				robotPosPad->setPoint(ofVec2f(r.worldPos.x, r.worldPos.y));
				robotCurRot->setValue(r.rot);

				robots[idx] = r;
			}
		}
	}
}

void ofApp::commandRobots() {
	for (auto &p : robots) {
		int idx = p.first;
		Robot &r = p.second;

		if (idx == kRobot01Idx) {
			bool didWrite = false;
			char *buf = (char *)malloc(50 * sizeof(char));
			ofVec2f pos2d = ofVec2f(r.worldPos.x, r.worldPos.y);
			ofVec2f target2d = robotTargetPad->getPoint();

			float targetRot = robotTargetRot->getValue();
			float rotAccuracy = robotRotAccuracy->getValue();
			float posAccuracy = robotPosAccuracy->getValue();
			float rotStrength = robotRotStrength->getValue();
			float movStrength = robotMovStrength->getValue();

			robotRotRolling += (r.rot - robotRotRolling) * 0.05;

			if (robot01GoRot->getChecked() && (r.rot > targetRot + rotAccuracy || r.rot < targetRot - rotAccuracy)) {
				int angleVal = robotRotRolling > targetRot ? 500 - rotStrength : 500 + rotStrength;

				sprintf((char *)buf, "MR01ROT%04d\n", angleVal);
				didWrite = true;
			} else if (robot01GoPos->getChecked() && pos2d.distance(target2d) > posAccuracy) {
				ofVec2f go = target2d - pos2d;
				go = go.normalize() * movStrength;

				int goX = 500 - go.x;
				int goY = 500 - go.y;

				sprintf((char *)buf, "MR01MOV%04d%04d\n", goX, goY);
				didWrite = true;
			} else {
				sprintf((char *)buf, "MR01SET050005000500\n");
				didWrite = true;
			}

			if (didWrite) {
				lastMessage = buf;
				cout << "SENDING " << buf << endl;
				robot01.Send(buf, strlen((char *)buf));
			}
			free(buf);
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	stringstream posstr;

	cam.begin();

	ofSetColor(255);
	ofDrawAxis(10.0);

	{
		ofPushMatrix();
		ofPushStyle();
		ofVec3f pos = ofVec3f(0.0) * cameraToWorld;
		ofSetColor(255, 255, 255);
		ofDrawIcoSphere(pos, 0.02);
		ofPopStyle();
		ofPopMatrix();
	}

	{
		ofPushMatrix();
		ofPushStyle();
		ofVec2f target = robotTargetPad->getPoint();
		ofVec3f pos(target.x, target.y, 0.0);
		ofSetColor(255, 0, 0);
		ofDrawIcoSphere(pos, 0.02);
		ofPopStyle();
		ofPopMatrix();
	}

	posstr << "lastMessage: " << lastMessage << endl;

	ofVec3f corner1(0.0),
		corner2(kMarkerSize, kMarkerSize, 0.0),
		corner3(kMarkerSize, 0.0, 0.0),
		corner4(0.0, kMarkerSize, 0.0),
		up(0.0, 0.0, kMarkerSize);

	for (map<int, Robot>::iterator it = robots.begin(); it != robots.end(); ++it) {
		int idx = it->first;
		Robot &r = it->second;

		ofVec3f c1 = corner1 * r.mat,
			c2 = corner2 * r.mat,
			c3 = corner3 * r.mat,
			c4 = corner4 * r.mat,
			u = up * r.mat;

		posstr << idx << ": " << r.tvec << endl;
		posstr << idx << ": " << c1*100.0 << " @ " << r.rot << endl;

		ofPushMatrix();
		ofPushStyle();

		// Option 1: draw axes
//		ofMultMatrix(r.mat);
//		ofDrawAxis(1.0);

		// Option 2: draw a robot
		ofSetColor(255, 255, 255);
		ofDrawLine(c1, c3);
		ofDrawLine(c1, c4);
		ofDrawLine(c2, c3);
		ofDrawLine(c2, c4);
		ofSetColor(255, 255, 0);
		ofDrawLine(c1, c2);
		ofSetColor(0, 255, 255);
		ofDrawLine(c1, u);
		ofPopStyle();
		ofPopMatrix();
	}
	cam.end();

	ofSetColor(255, 255, 255);
	ofDrawBitmapString(posstr.str(), 10, 15);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
