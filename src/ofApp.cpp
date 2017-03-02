#include "ofApp.h"

static const bool debugging = true;

static const string currentFile = "test_case.svg";
static const float kMarkerSize = 0.2032f;

static const string filePath = "/Users/dqgorelick/Downloads/";

static const float MAP_W = 1.0f;
static const float MAP_H = 1.0f;
static const float OFFSET_X = -0.0f;
static const float OFFSET_Y = -0.3f;

static const int kNumPathsToSave = 10000;

static char udpMessage[1024];
static char buf[1024];

//--------------------------------------------------------------

void ofApp::setup() {
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

	Robot *r02 = new Robot(2, 24, "Sarah");
	robotsById[r02->id] = r02;
	robotsByMarker[r02->markerId] = r02;
	r02->setCommunication("192.168.1.71", 5111);
    
    // set up GUI
    gui = new ofxDatGui( ofxDatGuiAnchor::TOP_RIGHT );
	gui->setTheme(new ofxDatGuiThemeMidnight());

    gui->addHeader("St. Louis Maproom Console", false);
	stateLabel = gui->addLabel(stateString());

	startButton = gui->addButton("Start");
	pauseButton = gui->addButton("Pause");
	stopButton = gui->addButton("Stop");

	startButton->onButtonEvent([this](ofxDatGuiButtonEvent e) {
		setState(MR_RUNNING);
	});
	pauseButton->onButtonEvent([this](ofxDatGuiButtonEvent e) {
		setState(MR_PAUSED);
	});
	stopButton->onButtonEvent([this](ofxDatGuiButtonEvent e) {
		setState(MR_STOPPED);
	});

	gui->addBreak();
    
    for (map<int, Robot*>::iterator it = robotsById.begin(); it != robotsById.end(); ++it) {
        int robotId = it->first;
        Robot &r = *it->second;
		RobotGui &rGui = robotGuis[robotId];

		rGui.folder = gui->addFolder(ofToString(r.name.c_str()) + " (" + ofToString(r.id) + ")", ofColor::white);
		rGui.folder->expand();

        rGui.stateLabel = rGui.folder->addLabel(r.stateDescription());
		rGui.posLabel = rGui.folder->addLabel(r.positionString());
		rGui.lastMessageLabel = rGui.folder->addLabel("");
		rGui.folder->addBreak();
		rGui.kp = rGui.folder->addSlider("kp", 0, 10000);
		rGui.kp->setValue(2500.0);
		rGui.ki = rGui.folder->addSlider("ki", 0, 100);
		rGui.ki->setValue(0.0);
		rGui.kd = rGui.folder->addSlider("kd", 0, 10000);
		rGui.kd->setValue(0.0);
		ofxDatGuiSlider *kiMax = rGui.folder->addSlider("kiMax", 0, 10000);
		kiMax->setValue(0.0);
        rGui.advanceButton = rGui.folder->addButton("Skip path");
        
        // event listeners
		rGui.kp->onSliderEvent([&r](ofxDatGuiSliderEvent e) {
			r.targetLinePID.setP(e.value);
		});
		rGui.ki->onSliderEvent([&r](ofxDatGuiSliderEvent e) {
			r.targetLinePID.setI(e.value);
		});
		rGui.kd->onSliderEvent([&r](ofxDatGuiSliderEvent e) {
			r.targetLinePID.setD(e.value);
		});
		kiMax->onSliderEvent([&r](ofxDatGuiSliderEvent e) {
			r.targetLinePID.setMaxIOutput(e.value);
		});
        rGui.advanceButton->onButtonEvent([&r](ofxDatGuiButtonEvent e) {
            r.setState(R_DONE_DRAWING);
        });

		gui->addBreak();
    }
    

    ofxDatGuiButton *reloadMapButton = gui->addButton("Reset Map");
	reloadMapButton->onButtonEvent([this](ofxDatGuiButtonEvent e) {
		for (auto &i : currentMap->mapPathStore) {
			for (auto &j : i.second) {
				j.claimed = false;
				j.drawn = false;
			}
		}
	});
    
    ofxDatGuiButton *loadNewMapButton = gui->addButton("Load New Map");
    loadNewMapButton->onButtonEvent([this](ofxDatGuiButtonEvent e) {
        currentMap->clearStore();
        string mostRecent = currentMap->getMostRecentMap(filePath);
        if (mostRecent.size()) {
            cout << "loading from " << mostRecent << endl;
            loadMap(mostRecent);
        } else {
            loadMap("test.svg");
        }
    });
    
	gui->addFRM();

	// Listen for messages from camera
	oscReceiver.setup( PORT );

	// Listen for messages from the robots

	robotReceiver.Create();
	robotReceiver.Bind(5101);
	robotReceiver.SetNonBlocking(true);
    
    currentMap = new Map(MAP_W, MAP_H, OFFSET_X, OFFSET_Y);
    
    string mostRecent = currentMap->getMostRecentMap(filePath);
    if (mostRecent.size()) {
        cout << "loading from " << mostRecent << endl;
        loadMap(mostRecent);
    } else {
        loadMap("test.svg");
    }
	setState(MR_STOPPED);
    
    
    // set up paths gui
    pathGui = new ofxDatGui( ofxDatGuiAnchor::TOP_LEFT );
    pathGui->setTheme(new ofxDatGuiThemeMidnight());
    
    pathGui->addHeader("Paths GUI");
    pathLabel = pathGui->addLabel("Total Active Paths: " + ofToString(currentMap->getActivePathCount()));
    pathStatusLabel = pathGui->addLabel("");
    drawnPathLabel = pathGui->addLabel("");
    
    pathGui->addBreak();
    

    int dropdown_index;
    vector<string> opts;
    for (auto &p : robotsById) {
        int id = p.first;
        Robot &r = *p.second;
        
        opts.push_back("ROBOT ID " + ofToString(id) + " - " + ofToString(r.name));
        dropDownToRobotId[dropdown_index] = id;
        dropdown_index++;
    }

    for (int i=0; i<currentMap->pathTypes.size(); i++) {
        PathGui &pGui = pathGuis[i];
        
        string pathName = currentMap->pathTypes[i];
        
        pGui.togglePath = pathGui->addToggle("PATH: " + ofToString(pathName) + " \t\t- " + ofToString(currentMap->getPathCount(pathName)));
        pGui.togglePath->setChecked(true);
        
        if (opts.size()) {
            pGui.drawOptions = pathGui->addDropdown("Select Robot:", opts);
            // defaults to first robot for all paths
            // TODO: divy up paths differently?
            pGui.drawOptions->select(0);
            pathAssignment[pathName] = dropDownToRobotId[0];
            pGui.drawOptions->onDropdownEvent([this, pathName, pGui](ofxDatGuiDropdownEvent e) {
                pathAssignment[pathName] = dropDownToRobotId[e.child];
            });
        }
        
        pathGui->addBreak();
        
        pGui.togglePath->onToggleEvent([this, pathName](ofxDatGuiToggleEvent e) {
            currentMap->setPathActive(pathName, e.checked);
        });
    }
    
    pathGui->addFooter();
}

void ofApp::exit() {
	for (auto &p : robotsById) {
		int id = p.first;
		Robot &r = *p.second;
		r.stop();
	}
}

void ofApp::setState(MaproomState newState) {
	startButton->setEnabled(newState != MR_RUNNING);
	pauseButton->setEnabled(newState != MR_PAUSED);
	stopButton->setEnabled(newState != MR_STOPPED);

	static const ofColor enabled(50, 50, 100), disabled(50, 50, 50);
	startButton->setBackgroundColor(newState == MR_RUNNING ? enabled : disabled);
	pauseButton->setBackgroundColor(newState == MR_PAUSED ? enabled : disabled);
	stopButton->setBackgroundColor(newState == MR_STOPPED ? enabled : disabled);

	state = newState;
	stateStartTime = ofGetElapsedTimef();
}

//--------------------------------------------------------------
void ofApp::update(){
	handleOSC();
	receiveFromRobots();
	commandRobots();
	updateGui();
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
				ofMatrix3x3 rmat(jsonMsg["rmats"][i][0][0].asFloat(), jsonMsg["rmats"][i][0][1].asFloat(), jsonMsg["rmats"][i][0][2].asFloat(),
								 jsonMsg["rmats"][i][1][0].asFloat(), jsonMsg["rmats"][i][1][1].asFloat(), jsonMsg["rmats"][i][1][2].asFloat(),
								 jsonMsg["rmats"][i][2][0].asFloat(), jsonMsg["rmats"][i][2][1].asFloat(), jsonMsg["rmats"][i][2][2].asFloat());

				if (robotsByMarker.find(markerId) == robotsByMarker.end()) {
					// Erroneous marker
					continue;
				}

				robotsByMarker[markerId]->updateCamera(rvec, tvec, rmat, cameraToWorldInv);
			}
		}
	}
}

void ofApp::unclaimPath(int robotId) {
	if (robotPaths.find(robotId) == robotPaths.end()) {
		MapPath *mp = robotPaths[robotId];
		if (mp != NULL && mp->claimed && !mp->drawn) {
			mp->claimed = false;
		}
		robotPaths.erase(robotId);
	}
}

void ofApp::commandRobots() {
	for (auto &p : robotsById) {
		int id = p.first;
		Robot &r = *p.second;

		// Determine draw state
		if (state == MR_STOPPED) {
			unclaimPath(id);
			r.stop();
		} else if (r.state == R_STOPPED && state == MR_RUNNING) {
			unclaimPath(id);
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
                    if (debugging) {
                        r.planePos = mp->segment.end;
                        r.avgPlanePos = mp->segment.end;
                        r.slowAvgPlanePos = mp->segment.end;
                    }
					// TODO: make this a function on robot specifically
					r.setState(R_READY_TO_POSITION);
				}
			}
		}

		// Robot process its own loop.
		r.update();

		// Update robot gui
		if (robotGuis.find(id) != robotGuis.end()) {
			RobotGui &gui = robotGuis[id];

			gui.stateLabel->setLabel(r.stateDescription());
			gui.posLabel->setLabel(r.positionString());
			gui.lastMessageLabel->setLabel(r.lastMessage.length() > 0 ? r.lastMessage : "Unknown");

		}


		if (ofGetElapsedTimef() > 3.0f) {
			// Record robot location
			if (robotPositionsCount.find(id) == robotPositionsCount.end()) {
				robotPositions[id].reserve(kNumPathsToSave);
				robotPositionsCount[id] = 0;
				robotPositionsIdx[id] = 0;
			}
			robotPositions[id][robotPositionsIdx[id]] = r.avgPlanePos;
			if (robotPositionsCount[id] < kNumPathsToSave) {
				robotPositionsCount[id]++;
			}
			robotPositionsIdx[id] = (robotPositionsIdx[id] + 1) % kNumPathsToSave;
		}
	}
}

void ofApp::updateGui() {
	char buf[1024];
	sprintf(buf, "%s (%.2f)", stateString().c_str(), ofGetElapsedTimef() - stateStartTime);
	stateLabel->setLabel(buf);
    
    pathLabel->setLabel("Total Active Paths: " + ofToString(currentMap->getActivePathCount()));
    
    int activePaths = currentMap->getActivePathCount();
    int drawnPaths = currentMap->getDrawnPaths();
    int pathsLeft = activePaths - drawnPaths;
    float percentage = (activePaths > 0 ? drawnPaths / activePaths : 100);
    
    sprintf(buf, "Drawn (active) Paths: %d", drawnPaths);
    pathStatusLabel->setLabel(buf);
    
    sprintf(buf, "Active Paths Remaining %d, percentage drawn: (%.2f)", drawnPaths, percentage);
    drawnPathLabel->setLabel(buf);
    
    static const ofColor enabled(50, 50, 100), disabled(50, 50, 50);
    
    for (int i=0; i<currentMap->pathTypes.size(); i++) {
        PathGui &pGui = pathGuis[i];
        bool pathActive = currentMap->activePaths[currentMap->pathTypes[i]];
        pGui.togglePath->setBackgroundColor(pathActive ? enabled : disabled);
        pGui.drawOptions->setBackgroundColor(pathActive ? enabled : disabled);
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

string ofApp::stateString() {
	switch(state) {
		case MR_STOPPED:
			return "MR_STOPPED";
		case MR_RUNNING:
			return "MR_RUNNING";
		case MR_PAUSED:
			return "MR_PAUSED";
		default:
			return "UNKNOWN_STATE";
	}
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
//	sprintf(buf, "Maproom: %s", maproomStateStr.c_str());
//	posstr << buf << endl;

	cam.begin();

	ofSetColor(255);
	ofDrawAxis(1.0);

//	{
//		ofPushMatrix();
//		ofPushStyle();
//		ofVec3f pos = ofVec3f(0.0) * cameraToWorld;
//		ofSetColor(255, 255, 255);
//        ofNoFill();
//		ofDrawIcoSphere(pos, 0.02);
//		ofPopStyle();
//		ofPopMatrix();
//	}

    // offset projection
    ofVec3f corner1(-kMarkerSizeM/2.0, -kMarkerSizeM/2.0),
    corner2(kMarkerSizeM/2.0, kMarkerSizeM/2.0, 0.0),
    corner3(kMarkerSizeM/2.0, -kMarkerSizeM/2.0, 0.0),
    corner4(-kMarkerSizeM/2.0, kMarkerSizeM/2.0, 0.0),
    center(0.0, 0.0, 0.0),
    up(0.0, 0.0, kMarkerSizeM);
    
    // draw all paths
    ofSetLineWidth(2.0);
    for (auto &i : currentMap->mapPathStore) {
        for (auto &j : i.second) {
			if (j.drawn) {
				ofSetColor(255, 255, 255);
			} else if (j.claimed) {
				ofSetColor(200, 0, 200);
			} else if(!currentMap->activePaths[i.first]) {
                ofSetColor(0, 0, 0);
            } else {
				ofSetColor(50, 50, 50);
			}
            
            ofDrawLine(j.segment.start, j.segment.end);
        }
    }
    ofSetLineWidth(1.0);

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
		if (r.state == R_DRAWING || debugging) {
			ofSetColor(255, 255, 0);
		} else {
			ofSetColor(0, 0, 255);
		}
        ofDrawLine(r.startPlanePos, r.targetPlanePos);
        
		// Debug output
//		sprintf(buf, "%d - %s %s %s - posCm = (%+07.1f, %+07.1f) - rotDeg = %03.1f (%s)",
//            r.id,
//			r.stateString().c_str(),
//            r.commsUp() ? "CONNECTED" : "DISCONNECTED",
//            r.cvDetected() ? "SEEN" : "HIDDEN",
//            r.avgPlanePos.x * 100.0, r.avgPlanePos.y * 100.0,
//            r.avgRot,
//            r.name.c_str());
//		posstr << buf << endl;

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

		ofPopStyle();
		ofPopMatrix();

		ofSetColor(255, 0, 0);
		ofDrawLine(r.planePos, r.planePos + r.vecToEnd / 1000.0f);

		ofSetColor(0, 255, 0);
		ofDrawLine(r.planePos, r.planePos + r.backToLine / 1000.0f);

		ofSetColor(90, 90, 90);
		ofDrawLine(r.planePos, r.planePos + r.dirToLine);

		ofSetColor(255, 255, 0);
		ofDrawLine(r.planePos, r.planePos + r.movement / 1000.0f);
        
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

void ofApp::loadMap(const string &mapName) {
    currentMap->loadMap(mapName);
    
    for (auto &p : robotsById) {
        int id = p.first;
        Robot &r = *p.second;
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'g') {
		setState(MR_RUNNING);
	} else if (key == ' ') {
		setState(MR_PAUSED);
	} else if (key == 'x') {
		setState(MR_STOPPED);
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
