#include "ofApp.h"

static const string currentFile = "maproom-2017-02-26T04-23-37.025Z.svg";
static const float kMarkerSize = 0.2032f;

static const bool ROBOTS_DRAW = true;

static const float MAP_W = 1.3f;
static const float MAP_H = 1.3f;
static const float OFFSET_X = 1.0f;
static const float OFFSET_Y = 0.7f;

char udpMessage[1024];

Map *currentMap;

vector<MapPath> pathsDrawn;
vector<vector<ofVec2f>> robotPaths; // paths for reach robot, indexed by ID
bool getNextPath = false;


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

	Robot *r01 = new Robot(2, 24, "Delmar");
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
        gui->addButton("Start: " + ofToString(r.id));
        gui->addButton("Calibrate: " + ofToString(r.id));
        gui->addButton("Next Path: " + ofToString(r.id));
        gui->addSlider("Rotation Angle: " + ofToString(r.id), 0, 360, startingAngle);
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

//--------------------------------------------------------------
void ofApp::update(){
	handleOSC();
    robotConductor();
	receiveFromRobots();
	commandRobots();
}

void ofApp::robotConductor() {
    // robot conductor responsibilities: control pen, send new directions, give the okay to start drawing
    for (auto &p : robotsById) {
        int id = p.first;
        Robot &r = *p.second;
        // todo refactor getNextPath to work for both robots
        if(r.getInitial || (ROBOTS_DRAW && (r.state == R_STOPPED || r.state == R_DONE_DRAWING) && r.navState.readyForNextPath)) {
            r.getInitial = false;
            r.navState.readyForNextPath = false;
            if (currentMap->checkNextPath(r.planePos)) {
            
            } else {
                r.stop();
                cout << "No more paths to draw! (of type: " << ofToString(r.navState.pathType) << endl;
            }
        }
        // look at state machine
        if (r.state == R_WAITING_TO_DRAW) {
            // todo: calibrate and do more things
            r.navState.drawReady = true;
        } else if (r.state == R_DRAWING) {
//            ofVec2f current = ofVec2f(r.planePos.x, r.planePos.y);
//            if (robotPaths.size()) {
//                if (robotPaths[r.id].size()) {
//                    ofVec2f last = robotPaths[r.id].back();
//                    ofVec2f diff = current - last;
//                    float len = diff.length();
//                    if (len > 0.001) {
//                        robotPaths[r.id].push_back(current);
//                    }
//                } else {
//                    robotPaths[r.id].push_back(current);
//                }
//            }
//            robotPaths[r.id].push_back(current);
        }
    }
    getNextPath = false;
    
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

void ofApp::loadNextPath(Robot* r) {
    MapPath nextPath = currentMap->getNextPath();
    float lenToStart = (nextPath.segment.start - r->navState.end).length();
    float lenToEnd = (nextPath.segment.end - r->navState.end).length();
    if (lenToStart < lenToEnd) {
        pathsDrawn.push_back(nextPath);
    } else {
        ofVec2f tmp = nextPath.segment.start;
        nextPath.segment.start = nextPath.segment.end;
        nextPath.segment.end = tmp;
        pathsDrawn.push_back(nextPath);
    }
    r->startNavigation(nextPath.segment.start, nextPath.segment.end);

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
     
    char buf[1024];
    

    ofPushStyle();
    
    // draw all paths
    for (auto &i : currentMap->mapPathStore) {
        for (auto &j : i.second) {
            ofSetColor(0, 90, 0);
            ofDrawCircle(j.segment.start, 0.006);
            ofSetColor(90, 0, 0);
            ofDrawCircle(j.segment.end, 0.006);
            ofSetColor(90, 90, 90);
            ofDrawLine(j.segment.start, j.segment.end);
        }
    }
    
    // draw current paths / paths drawn
    ofSetColor(95, 255, 255);
    for (int path = 0; path < pathsDrawn.size(); path++) {
        ofVec3f lineStart = ofVec3f(pathsDrawn[path].segment.start[0], pathsDrawn[path].segment.start[1], 0);
        ofVec3f lineEnd = ofVec3f(pathsDrawn[path].segment.end[0], pathsDrawn[path].segment.end[1], 0);
        ofDrawLine(lineStart, lineEnd);
    }
    ofPopStyle();
    
	for (map<int, Robot*>::iterator it = robotsById.begin(); it != robotsById.end(); ++it) {
		int robotId = it->first;
		Robot &r = *it->second;
        
        
        // draw current path for the robot:
        
        ofPushStyle();
        ofSetColor(255, 255, 0);
        ofDrawLine(r.navState.start, r.navState.end);
        // start = green
        // end = red
        ofSetColor(0, 255, 0);
        ofDrawCircle(r.navState.start.x, r.navState.start.y, 0.02);
        ofSetColor(255, 0, 0);
        ofDrawCircle(r.navState.end.x, r.navState.end.y, 0.02);
        ofPopStyle();
        // draw total paths for the robot:
        
        ofSetColor(247, 0, 255);
        if (robotPaths.size()) {
            for (int i=0; i<robotPaths[r.id].size()-1; i++) {
                ofVec2f curr = robotPaths[r.id][i];
                ofVec2f next = robotPaths[r.id][i+1];
                ofVec2f diff = curr - next;
                if (diff.length() < 0.1) {
                    ofDrawLine(curr, next);
                }
            }
        }
        
        ofPopStyle();
        
        
		// Debug output

		sprintf(buf, "%d - %s %s - %+04.1f - posCm = (%+07.1f, %+07.1f) - heightCm = %+04.1f - rotDeg = %03.1f (%s)",
            r.id,
            r.commsUp() ? "CONNECTED" : "DISCONNECTED",
            r.cvDetected() ? "SEEN" : "HIDDEN",
            r.dist * 100.0,
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
        
        // draw arrows
        
        float proj = 0.5;
//        float rad = ofDegToRad(r.headingAngle);
        ofVec2f projPos;
        projPos.x = r.planePos.x + cos(r.headingRad)*proj;
        projPos.y = r.planePos.y + sin(r.headingRad)*proj;
        ofSetColor(150, 150, 150);
        ofDrawLine(r.planePos, projPos);
        
        projPos.x = r.planePos.x + cos(r.idealRad)*proj;
        projPos.y = r.planePos.y + sin(r.idealRad)*proj;
        ofSetColor(0, 255, 0);
        ofDrawLine(r.planePos, projPos);
        
        projPos.x = r.planePos.x + cos(r.counterRad)*proj;
        projPos.y = r.planePos.y + sin(r.counterRad)*proj;
        ofSetColor(255, 0, 0);
        ofDrawLine(r.planePos, projPos);

		ofSetColor(255, 255, 255);
		ofDrawSphere(cen, kMarkerSizeM / 4.0);

        
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
    pathsDrawn.clear();
    getNextPath = false;
    
    for (auto &p : robotsById) {
        int id = p.first;
        Robot &r = *p.second;
        r.getInitial = true;
    }
}

void ofApp::onToggleEvent(ofxDatGuiToggleEvent e)
{
    for (map<int, Robot*>::iterator it = robotsById.begin(); it != robotsById.end(); ++it) {
        int robotId = it->first;
        Robot &r = *it->second;
        if (e.target->is("messages enabled " + ofToString(r.id))) {
            r.enableMessages = e.checked;
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
        } else if (e.target->is("move: " + ofToString(r.id))) {
            cout << "MOVING " << ofToString(r.id) << endl;
            r.testMove(r.moveDir, r.moveMag);
        } else if (e.target->is("rotate: " + ofToString(r.id))) {
            cout << "ROTATING " << ofToString(r.id) << endl;
            r.testRotate(r.targetRot);
        } else if (e.target->is("start drawing: " + ofToString(r.id))) {
            r.navState.readyForNextPath = true;
        } else if (e.target->is("start: " + ofToString(r.id))) {
            r.start();
        } else if (e.target->is("next path: " + ofToString(r.id))) {
            if(currentMap->checkNextPath(r.navState.end)) {
                loadNextPath(&r);
            } else {
                cout << "no more paths" << endl;
            }
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
