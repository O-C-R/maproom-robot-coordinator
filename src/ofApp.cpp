#include "ofApp.h"

//static const string currentFile = "maproom-2017-02-26T04-23-37.025Z.svg";
static const string currentFile = "test_case.svg";
static const float kMarkerSize = 0.2032f;

static const float MAP_W = 1.0f;
static const float MAP_H = 1.0f;
static const float OFFSET_X = -0.5f;
static const float OFFSET_Y = -0.5f;

static char udpMessage[1024];
static char buf[1024];

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetVerticalSync(true);
	ofSetBackgroundColor(0);

	state = MR_STOPPED;

	cam.setDistance(3);
	cam.enableMouseInput();
	cam.setTarget(ofVec3f(0.0));
	cam.setNearClip(0.01);

	// on the pillar
    //	cameraPos = ofVec3f(0., 0., 2.03);
    //	cameraLook = ofVec3f(0.0, 0.82, 0.0);
    //	cameraToWorld.makeLookAtMatrix(cameraPos, cameraLook, ofVec3f(0.0, 0.0, 1.0));

	// on the map hanging thing
//	cameraPos = ofVec3f(0., 0., 2.845);
//	cameraLook = ofVec3f(0.0, 0.0, 0.0);

	// on the wall in the airbnb
	cameraPos = ofVec3f(0., 0., 2.4384);
	cameraLook = ofVec3f(0.0, 1.0668, 0.0);
	ofVec3f up = (cameraLook - cameraPos).rotate(-90, ofVec3f(1.0, 0.0, 0.0)).normalize();
	cameraToWorld.makeLookAtMatrix(cameraPos, cameraLook, ofVec3f(0.0, 0.0, 1.0));

	cameraToWorld.makeIdentityMatrix();
	ofQuaternion cameraRotation;
	cameraRotation.makeRotate(180, ofVec3f(1.0, 0.0, 0.0));
	cameraToWorld.setRotate(cameraRotation);
	cameraToWorld.setTranslation(cameraPos);

	cameraToWorldInv = cameraToWorld.getInverse();

	Robot *r01 = new Robot(1, 24, "Delmar");
	robotsById[r01->id] = r01;
	robotsByMarker[r01->markerId] = r01;
	r01->setCommunication("192.168.7.72", 5111);

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
        gui->addToggle("Messages Enabled " + ofToString(r.id), true);
        gui->addButton("Stop: " + ofToString(r.id));
        gui->addButton("Start: " + ofToString(r.id));
        gui->addButton("Calibrate: " + ofToString(r.id));
        gui->addButton("Next Path: " + ofToString(r.id));
        gui->addSlider("Rotation Angle: " + ofToString(r.id), 0, 360, r.rot);
        gui->addButton("Rotate: " + ofToString(r.id));
        gui->addButton("Start Drawing: " + ofToString(r.id));
        gui->addBreak();
    }
    gui->addButton("Reload Map");
    
    gui->onToggleEvent(this, &ofApp::onToggleEvent);
    gui->onButtonEvent(this, &ofApp::onButtonEvent);
    gui->onSliderEvent(this, &ofApp::onSliderEvent);

	// Listen for messages from camera
	oscReceiver.setup( PORT );

	// Listen for messages from the robots

	robotReceiver.Create();
	robotReceiver.Bind(5101);
	robotReceiver.SetNonBlocking(true);
    
    currentMap = new Map(MAP_W, MAP_H, OFFSET_X, OFFSET_Y);
    loadMap(currentFile);
}

void ofApp::exit() {
	for (auto &p : robotsById) {
		int id = p.first;
		Robot &r = *p.second;
		r.stop();
	}
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
	for (auto &p : robotsById) {
		int id = p.first;
		Robot &r = *p.second;

		// Determine draw state
		if (state == MR_STOPPED) {
			r.stop();
		} else if (r.state == R_STOPPED && state == MR_RUNNING) {
			r.start();
		} else if (r.state == R_READY_TO_POSITION && state == MR_RUNNING) {
			MapPath *mp = currentMap->nextPath(r.avgPlanePos);
			robotPaths[id] = mp;

			if (mp != NULL) {
				mp->claimed = true;
				r.navigateTo(mp->segment.start);
			} else {
				cout << "No more paths to draw!" << endl;
			}
		} else if (r.state == R_READY_TO_DRAW && state == MR_RUNNING) {
			if (robotPaths.find(id) == robotPaths.end()) {
				// Error!
				r.stop();
			} else {
				MapPath *mp = robotPaths[id];
				if (mp == NULL) {
					// error!
					r.stop();
				} else {
					r.drawLine(mp->segment.start, mp->segment.end);
				}
			}
		} else if (r.state == R_DONE_DRAWING) {
			if (robotPaths.find(id) == robotPaths.end()) {
				// Error!
				r.stop();
			} else {
				MapPath *mp = robotPaths[id];
				if (mp == NULL) {
					// error!
					r.stop();
				} else {
					mp->drawn = true;
					robotPaths.erase(id);

					// TODO: make this a function on robot specifically
					r.setState(R_READY_TO_POSITION);
				}
			}
		}

		// Robot process its own loop.
		r.update();

		// Record robot location
		if (robotPositionsCount.find(id) == robotPositionsCount.end()) {
			robotPositions[id].reserve(1000);
			robotPositionsCount[id] = 0;
			robotPositionsIdx[id] = 0;
		}
		robotPositions[id][robotPositionsIdx[id]] = r.avgPlanePos;
		if (robotPositionsCount[id] < 1000) {
			robotPositionsCount[id]++;
		}
		robotPositionsIdx[id] = (robotPositionsIdx[id] + 1) % 1000;
	}
}

void ofApp::loadNextPath(Robot* r) {
//    MapPath nextPath = currentMap->getNextPath();
//    float lenToStart = (nextPath.segment.start - r->navState.end).length();
//    float lenToEnd = (nextPath.segment.end - r->navState.end).length();
//    if (lenToStart < lenToEnd) {
//        pathsDrawn.push_back(nextPath);
//    } else {
//        ofVec2f tmp = nextPath.segment.start;
//        nextPath.segment.start = nextPath.segment.end;
//        nextPath.segment.end = tmp;
//        pathsDrawn.push_back(nextPath);
//    }
//    r->startNavigation(nextPath.segment.start, nextPath.segment.end);
}

//--------------------------------------------------------------
void ofApp::draw(){
	stringstream posstr;

	string maproomStateStr;
	switch(state) {
		case MR_STOPPED:
			maproomStateStr = "MR_STOPPED";
			break;
		case MR_RUNNING:
			maproomStateStr = "MR_RUNNING";
			break;
		case MR_PAUSED:
			maproomStateStr = "MR_PAUSED";
			break;
		default:
			maproomStateStr = "UNKNOWN_STATE";
	}
	sprintf(buf, "Maproom: %s", maproomStateStr.c_str());
	posstr << buf << endl;

	cam.begin();

	ofSetColor(255);
	ofDrawAxis(10.0);

	{
		ofPushMatrix();
		ofPushStyle();
		ofVec3f pos = ofVec3f(0.0) * cameraToWorld;
		ofSetColor(255, 255, 255);
        ofNoFill();
		ofDrawIcoSphere(pos, 0.02);
		ofPopStyle();
		ofPopMatrix();
	}
    
    // offset projection
    ofVec3f corner1(-kMarkerSizeM/2.0, -kMarkerSizeM/2.0),
    corner2(kMarkerSizeM/2.0, kMarkerSizeM/2.0, 0.0),
    corner3(kMarkerSizeM/2.0, -kMarkerSizeM/2.0, 0.0),
    corner4(-kMarkerSizeM/2.0, kMarkerSizeM/2.0, 0.0),
    center(0.0, 0.0, 0.0),
    up(0.0, 0.0, kMarkerSizeM);
    
    // draw all paths
    for (auto &i : currentMap->mapPathStore) {
        for (auto &j : i.second) {
			if (j.drawn) {
				ofSetColor(255, 255, 255);
			} else if (j.claimed) {
				ofSetColor(200, 200, 0);
			} else {
				ofSetColor(50, 50, 50);
			}

            ofDrawLine(j.segment.start, j.segment.end);
        }
    }

	for (auto &rPos : robotPositions) {
		if (rPos.first == 0) {
			ofSetColor(255, 0, 0);
		} else if (rPos.first == 1) {
			ofSetColor(255, 255, 0);
		} else if (rPos.first == 2) {
			ofSetColor(0, 255, 255);
		}

		for (int i = 0; i < robotPositionsCount[rPos.first] - 1; ++i) {
			if (abs(i - robotPositionsIdx[rPos.first]) < 2) {
				continue;
			}

			ofDrawLine(rPos.second[i], rPos.second[i+1]);
		}
	}

	for (map<int, Robot*>::iterator it = robotsById.begin(); it != robotsById.end(); ++it) {
		int robotId = it->first;
		Robot &r = *it->second;

        // draw current path for the robot:
		if (r.state == R_DRAWING) {
			ofSetColor(255, 255, 0);
		} else {
			ofSetColor(0, 0, 255);
		}
        ofDrawLine(r.startPlanePos, r.targetPlanePos);
        
		// Debug output
		sprintf(buf, "%d - %s %s %s - posCm = (%+07.1f, %+07.1f) - rotDeg = %03.1f (%s)",
            r.id,
			r.stateString().c_str(),
            r.commsUp() ? "CONNECTED" : "DISCONNECTED",
            r.cvDetected() ? "SEEN" : "HIDDEN",
            r.avgPlanePos.x * 100.0, r.avgPlanePos.y * 100.0,
            r.avgRot,
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
//		ofSetColor(255, 255, 0);
//		ofDrawLine(c1, c3);
//		ofSetColor(255, 0, 0);
//		ofDrawLine(c1, c4);
//		ofSetColor(255, 255, 255);
//		ofDrawLine(c2, c3);
//		ofDrawLine(c2, c4);
//		ofSetColor(0, 255, 255);
//		ofDrawLine(c1, u);

        // draw arrows
        
//        float proj = 0.1;
//        float rad = ofDegToRad(r.headingAngle);


//        ofVec2f projPos;
//        projPos.x = r.planePos.x + cos(r.headingRad)*proj;
//        projPos.y = r.planePos.y + sin(r.headingRad)*proj;
//        ofSetColor(150, 150, 150);
//        ofDrawLine(r.planePos, projPos);
//        
//        projPos.x = r.planePos.x + cos(r.idealRad)*proj;
//        projPos.y = r.planePos.y + sin(r.idealRad)*proj;
//        ofSetColor(0, 255, 0);
//        ofDrawLine(r.planePos, projPos);
//        
//        projPos.x = r.planePos.x + cos(r.counterRad)*proj;
//        projPos.y = r.planePos.y + sin(r.counterRad)*proj;
//        ofSetColor(255, 0, 0);
//        ofDrawLine(r.planePos, projPos);

//		ofSetColor(255, 255, 255);
//		ofDrawSphere(cen, kMarkerSizeM / 16.0);

		ofSetColor(255, 0, 0);
		ofDrawLine(r.planePos, r.planePos + r.vecToEnd / 1000.0f);

		ofSetColor(0, 255, 0);
		ofDrawLine(r.planePos, r.planePos + r.backToLine / 1000.0f);

		ofSetColor(90, 90, 90);
		ofDrawLine(r.planePos, r.planePos + r.dirToLine);

		ofSetColor(255, 255, 0);
		ofDrawLine(r.planePos, r.planePos + r.movement / 1000.0f);
        
		ofPopStyle();
		ofPopMatrix();
        
        ofPushStyle();
        ofNoFill();
        ofSetColor(0,255,255);
        ofDrawCircle(r.avgPlanePos.x, r.avgPlanePos.y, 0.03);
        ofSetColor(255,0,255);
        ofDrawCircle(r.planePos.x, r.planePos.y, 0.03);
        ofPopStyle();
	}
	cam.end();
    
	ofSetColor(255, 255, 255);
	ofDrawBitmapString(posstr.str(), 10, 15);
}

void ofApp::loadMap(string mapName) {
    currentMap->loadMap(mapName);
    
    for (auto &p : robotsById) {
        int id = p.first;
        Robot &r = *p.second;
    }
}

void ofApp::onToggleEvent(ofxDatGuiToggleEvent e)
{
    for (map<int, Robot*>::iterator it = robotsById.begin(); it != robotsById.end(); ++it) {
        int robotId = it->first;
        Robot &r = *it->second;
        if (e.target->is("messages enabled " + ofToString(r.id))) {
            r.enabled = e.checked;
        }
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
        } else if (e.target->is("start: " + ofToString(r.id))) {
            r.start();
        } else if (e.target->is("next path: " + ofToString(r.id))) {
//            if(currentMap->checkNextPath(r.navState.end)) {
//                loadNextPath(&r);
//            } else {
//                cout << "no more paths" << endl;
//            }
        }
    }
    if (e.target->is("reload map")) {
        cout << "RELOADING MAP " << endl;
        loadMap(currentFile);
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
//            r.moveDir = e.value;
        } else if (e.target->is("move mag: " + ofToString(r.id))) {
            cout << "move mag set to " << e.value << endl;
//            r.moveMag = e.value;
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'g') {
		state = MR_RUNNING;
	} else if (key == ' ') {
		state = MR_PAUSED;
	} else if (key == 'x') {
		state = MR_STOPPED;
	}
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
