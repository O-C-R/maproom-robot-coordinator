#include "ofApp.h"

char udpMessage[1024];

// values for GUI


string guiRobotName;
string guiRotationStatus;



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

	cameraToWorldInv = cameraToWorld.getInverse();

	mapSvg.load("map.svg");
	cout << "mapSvg " << mapSvg.getWidth() << "x" << mapSvg.getHeight() << " w/ " << mapSvg.getNumPath() << endl;

	Robot *r01 = new Robot(1, 23, "Delmar");
	robotsById[r01->id] = r01;
	robotsByMarker[r01->markerId] = r01;
	r01->setCommunication("192.168.1.70", 5111);

//	Robot *r02 = new Robot(2, 24, "Sarah");
//	robotsById[r02->id] = r02;
//	robotsByMarker[r02->markerId] = r02;
//	r02->setCommunication("192.168.1.71", 5111);
    
    // set up GUI
    gui = new ofxDatGui( ofxDatGuiAnchor::TOP_RIGHT );
    gui->addHeader("St. Louis Maproom Console", false);
    gui->addBreak();
    
    for (map<int, Robot*>::iterator it = robotsById.begin(); it != robotsById.end(); ++it) {
        int robotId = it->first;
        Robot &r = *it->second;
        gui->addLabel("Robot ID: " + ofToString(r.id));
        gui->addLabel("Robot Name: " + ofToString(r.name.c_str()));
        
        float startingAngle = 0;
        float startingDir = 0;
        float startingMag = 0;
        r.targetRot = startingAngle;
        r.moveDir = startingDir;
        r.moveMag = startingMag;
        
        gui->addToggle("Messages Enabled " + ofToString(r.id), false);
        gui->addButton("Stop: " + ofToString(r.id));
        gui->addButton("Calibrate: " + ofToString(r.id));
        gui->addSlider("Rotation Angle: " + ofToString(r.id), 0, 360, startingAngle);
        gui->addButton("Rotate: " + ofToString(r.id));
        gui->addSlider("Move Dir: " + ofToString(r.id), 0, 360, startingDir);
        gui->addSlider("Move Mag: " + ofToString(r.id), 0, 600, startingMag);
        gui->addButton("Move: " + ofToString(r.id));
        gui->addBreak();
    }
    
    gui->onToggleEvent(this, &ofApp::onToggleEvent);
    gui->onButtonEvent(this, &ofApp::onButtonEvent);
    gui->onSliderEvent(this, &ofApp::onSliderEvent);

	// Listen for messages from camera
	oscReceiver.setup( PORT );

	// Listen for messages from the robots

	robotReceiver.Create();
	robotReceiver.Bind(5101);
	robotReceiver.SetNonBlocking(true);
    
}

//--------------------------------------------------------------
void ofApp::update(){
	handleOSC();
	receiveFromRobots();
	commandRobots();
}

void ofApp::receiveFromRobots() {
	int nChars = robotReceiver.Receive(robotMessage, 1024);
	if (nChars > 0) {
		robotMessage[nChars] = 0;

		if (robotMessage[0] == 'R' && robotMessage[1] == 'B') {
			char val = robotMessage[4];
			robotMessage[4] = 0;
			int robotId = atoi(robotMessage + 2);
			robotMessage[4] = val;

			if (robotsById.find(robotId) == robotsById.end()) {
				cout << "Unknown robot: " << robotId << endl;
				return;
			}

			Robot &r = *robotsById[robotId];

			if (robotMessage[4] == 'H' && robotMessage[5] == 'B') {
				r.gotHeartbeat();
			} else {
				cout << "Got unknown message from robot " << robotId << ": " << robotMessage + 4 << endl;
			}
		} else {
			cout << "Unknown robot message: " << robotMessage << endl;
		}
	}
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
				int markerId = jsonMsg["ids"][i].asInt();
				ofVec3f rvec(jsonMsg["rvecs"][i][0].asFloat(), jsonMsg["rvecs"][i][1].asFloat(), jsonMsg["rvecs"][i][2].asFloat());
				ofVec3f tvec(jsonMsg["tvecs"][i][0].asFloat(), jsonMsg["tvecs"][i][1].asFloat(), jsonMsg["tvecs"][i][2].asFloat());

				if (robotsByMarker.find(markerId) == robotsByMarker.end()) {
					// Erroneous marker
					continue;
				}

				robotsByMarker[markerId]->updateCamera(rvec, tvec, cameraToWorldInv);
			}
		}
	}
}

void ofApp::commandRobots() {
	char *buf = (char *)malloc(128 * sizeof(char));

	for (auto &p : robotsById) {
		int id = p.first;
		Robot &r = *p.second;
		r.update();
	}

	free(buf);
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

	ofVec3f corner1(0.0),
		corner2(kMarkerSizeM, kMarkerSizeM, 0.0),
		corner3(kMarkerSizeM, 0.0, 0.0),
		corner4(0.0, kMarkerSizeM, 0.0),
		center(kMarkerSizeM / 2.0, kMarkerSizeM / 2.0, 0.0),
		up(0.0, 0.0, kMarkerSizeM);

	char buf[1024];

	for (map<int, Robot*>::iterator it = robotsById.begin(); it != robotsById.end(); ++it) {
		int robotId = it->first;
		Robot &r = *it->second;

		// Debug output

		sprintf(buf, "%d - %s %s - posCm = (%+07.1f, %+07.1f) - heightCm = %+04.1f - rotDeg = %03.1f (%s)",
				r.id,
				r.commsUp() ? "CONNECTED" : "DISCONNECTED",
				r.cvDetected() ? "SEEN" : "HIDDEN",
				r.planePos.x * 100.0, r.planePos.y * 100.0,
				r.worldPos.z * 100.0,
				r.rot,
				r.name.c_str());
		posstr << buf << endl;
        
        

		ofVec3f c1 = corner1 * r.mat;
		ofVec3f c2 = corner2 * r.mat;
		ofVec3f c3 = corner3 * r.mat;
		ofVec3f c4 = corner4 * r.mat;
		ofVec3f cen = center * r.mat;
		ofVec3f u = up * r.mat;

		ofPushMatrix();
		ofPushStyle();

		// Option 1: draw axes
//		ofMultMatrix(r.mat);
//		ofDrawAxis(1.0);

		// Option 2: draw a robot
		ofSetColor(255, 255, 0);
		ofDrawLine(c1, c3);
		ofSetColor(255, 0, 0);
		ofDrawLine(c1, c4);
		ofSetColor(255, 255, 255);
		ofDrawLine(c2, c3);
		ofDrawLine(c2, c4);
		ofSetColor(0, 255, 255);
		ofDrawLine(c1, u);

		ofSetColor(255, 255, 255);
		ofDrawSphere(cen, kMarkerSizeM / 4.0);

		ofPopStyle();
		ofPopMatrix();
	}
	cam.end();

	ofSetColor(255, 255, 255);
	ofDrawBitmapString(posstr.str(), 10, 15);
}

void ofApp::onToggleEvent(ofxDatGuiToggleEvent e)
{
    for (map<int, Robot*>::iterator it = robotsById.begin(); it != robotsById.end(); ++it) {
        int robotId = it->first;
        Robot &r = *it->second;
        if (e.target->is("messages enabled " + ofToString(r.id))) {
            r.enableMessages = e.checked;
        };
    }
}


void ofApp::onButtonEvent(ofxDatGuiButtonEvent e)
{
    for (map<int, Robot*>::iterator it = robotsById.begin(); it != robotsById.end(); ++it) {
        int robotId = it->first;
        Robot &r = *it->second;
        
        if (e.target->is("stop: " + ofToString(r.id))) {
            r.stop();
            cout << "STOPPING " << ofToString(r.id) << endl;
        } else if (e.target->is("calibrate: " + ofToString(r.id))) {
            r.calibrate();
            cout << "CALIBRATING " << ofToString(r.id) << endl;
        } else if (e.target->is("move: " + ofToString(r.id))) {
            cout << "MOVING " << ofToString(r.id) << endl;
            r.testMove(r.moveDir, r.moveMag);
        } else if (e.target->is("rotate: " + ofToString(r.id))) {
            cout << "ROTATING " << ofToString(r.id) << endl;
            r.testRotate(r.targetRot);
        }
    }
}

void ofApp::onSliderEvent(ofxDatGuiSliderEvent e)
{
    for (map<int, Robot*>::iterator it = robotsById.begin(); it != robotsById.end(); ++it) {
        int robotId = it->first;
        Robot &r = *it->second;
        
        if (e.target->is("rotation angle: " + ofToString(r.id))) {
            cout << "set rotation angle of  " << e.value << endl;
            r.targetRot = e.value;
        } else if (e.target->is("move dir: " + ofToString(r.id))) {
            cout << "move dir set to " << e.value << endl;
            r.moveDir = e.value;
        } else if (e.target->is("move mag: " + ofToString(r.id))) {
            cout << "move mag set to " << e.value << endl;
            r.moveMag = e.value;
        }
    }
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
