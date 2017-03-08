#include "ofApp.h"

static const string currentFile = "test.svg";
static const string filePath = "/Users/anderson/Downloads/";

static const string kRPiHost = "127.0.0.1";
static const int kRPiPort = 5300;

static const float kMapWidthM = 1.0f;
static const float kMapHeightM = 1.0f;
static const float kMapOffsetXM = -kMapWidthM / 2.0;
static const float kMapOffsetYM = -kMapHeightM / 2.0;
static const ofRectangle kCropBox(ofVec2f(-0.45, -0.4), ofVec2f(0.45, 0.4));

static const bool debugging = false;

static const float kRobotSafetyDiameterM = 0.75f;

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

//	Robot *r01 = new Robot(1, 30, "Delmar");
//	robotsById[r01->id] = r01;
//	robotsByMarker[r01->markerId] = r01;
//	r01->setCommunication("192.168.7.70", 5111);

	Robot *r02 = new Robot(2, 26, "Archie");
	robotsById[r02->id] = r02;
	robotsByMarker[r02->markerId] = r02;
	r02->setCommunication("192.168.7.73", 5111);

//	Robot *r03 = new Robot(3, 26, "Camille");
//	robotsById[r03->id] = r03;
//	robotsByMarker[r03->markerId] = r03;
//	r03->setCommunication("192.168.7.71", 5111);

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

	rpiStateDropdown = gui->addDropdown("RPi State", { "Tracking", "Flashlight" });
	rpiStateDropdown->select(0);
	rpiStateDropdown->onDropdownEvent([this](ofxDatGuiDropdownEvent e) {
		int index = e.target->getSelected()->getIndex();
		ofxOscMessage m;
		m.setAddress("/state");
		m.addIntArg(index);
		oscToRPi.sendMessage(m, false);
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
		rGui.kp->setValue(r.targetLineKp);
		rGui.ki = rGui.folder->addSlider("ki", 0, 100);
		rGui.ki->setValue(r.targetLineKi);
		rGui.kd = rGui.folder->addSlider("kd", 0, 10000);
		rGui.kd->setValue(r.targetLineKd);
		rGui.kMaxI = rGui.folder->addSlider("kMaxI", 0, 10000);
		rGui.kMaxI->setValue(r.targetLineMaxI);
		rGui.folder->addBreak();
		rGui.minSpeed = rGui.folder->addSlider("minSpeed", 0, 1024);
		rGui.minSpeed->setValue(r.minSpeed);
		rGui.minSpeed->onSliderEvent([&r](ofxDatGuiSliderEvent e) {
			r.minSpeed = e.value;
		});
		rGui.maxSpeed = rGui.folder->addSlider("maxSpeed", 0, 1024);
		rGui.maxSpeed->setValue(r.maxSpeed);
		rGui.maxSpeed->onSliderEvent([&r](ofxDatGuiSliderEvent e) {
			r.maxSpeed = e.value;
		});
		rGui.speedRamp = rGui.folder->addSlider("speedRamp", 0, 1);
		rGui.speedRamp->setValue(r.speedRamp);
		rGui.speedRamp->onSliderEvent([&r](ofxDatGuiSliderEvent e) {
			r.speedRamp = e.value;
		});
        rGui.advanceButton = rGui.folder->addButton("Skip path");

		auto pidListener = [&rGui, &r](ofxDatGuiSliderEvent e) {
			r.updatePID(rGui.kp->getValue(),
						rGui.ki->getValue(),
						rGui.kd->getValue(),
						rGui.kMaxI->getValue());
		};

        // event listeners
		rGui.kp->onSliderEvent(pidListener);
		rGui.ki->onSliderEvent(pidListener);
		rGui.kd->onSliderEvent(pidListener);
		rGui.kMaxI->onSliderEvent(pidListener);

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

	oscToRPi.setup(kRPiHost, kRPiPort);

	// Listen for messages from the robots

	robotReceiver.Create();
	robotReceiver.Bind(5101);
	robotReceiver.SetNonBlocking(true);

    currentMap = new Map(kMapWidthM, kMapHeightM, kMapOffsetXM, kMapOffsetYM, kCropBox);

    string mostRecent = currentMap->getMostRecentMap(filePath);
    if (mostRecent.size()) {
        cout << "loading from " << mostRecent << endl;
        loadMap(mostRecent);
    } else {
        loadMap(currentFile);
    }

	setState(MR_STOPPED);
    
    // silence logger:
    guiLogger->quiet();
    
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
        
        // note: don't change the order of label. datGuiDropdown event handler is broken so we have to parse the string to get the selected ID
        opts.push_back(ofToString(id) + " ID ROBOT - " + ofToString(r.name));
        dropDownToRobotId[dropdown_index] = r.id;
        cout << r.id << endl;
        cout << "dropDownToRobotId[dropdown_index] " << dropDownToRobotId[dropdown_index] << endl;
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
            int initial = 0;
            pGui.drawOptions->select(initial);
            currentMap->pathAssignment[pathName] = dropDownToRobotId[initial];
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
    receiveGuiUpdates();
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
				ofVec2f pos(jsonMsg["pos"][i][0].asFloat(), jsonMsg["pos"][i][1].asFloat());
				ofVec2f up(jsonMsg["up"][i][0].asFloat(), jsonMsg["up"][i][1].asFloat());
				ofVec2f rawPos(jsonMsg["raw_pos"][i][0].asFloat(), jsonMsg["raw_pos"][i][1].asFloat());
				ofVec2f rawUp(jsonMsg["raw_up"][i][0].asFloat(), jsonMsg["raw_up"][i][1].asFloat());

				if (robotsByMarker.find(markerId) != robotsByMarker.end()) {
					robotsByMarker[markerId]->updateCamera(pos, up);
				}

				if (markersById.find(markerId) == markersById.end()) {
					markersById[markerId] = ArucoMarker(markerId);
				}
				markersById[markerId].updateCamera(rawPos, rawUp);
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
			MapPath *mp = currentMap->nextPath(r.avgPlanePos, r.id, r.lastHeading);
			robotPaths[id] = mp;

			if (mp != NULL) {
				if (mp->segment.start.distance(r.avgPlanePos) > mp->segment.end.distance(r.avgPlanePos)) {
					ofVec2f tmp = mp->segment.start;
					mp->segment.start = mp->segment.end;
					mp->segment.end = tmp;
				}

				mp->claimed = true;
				r.navigateTo(mp->segment.start);
                r.lastHeading = atan2(mp->segment.end.x - mp->segment.start.x, mp->segment.end.y - mp->segment.start.y)*180/3.14159;
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

void ofApp::receiveGuiUpdates() {
    // event handler for dropdown is broken so this is a hack
    // https://www.bountysource.com/issues/31521059-ofxdatguidropdown-not-collapsing-after-select-when-added-to-ofxdatgui
    
    for (int i=0; i<currentMap->pathTypes.size(); i++) {
        PathGui &pGui = pathGuis[i];
        ofxDatGuiDropdownOption selected = *pGui.drawOptions->getSelected();
        
        int selectedRobot = atoi(ofToString(selected.getLabel()[0]).c_str());
        currentMap->pathAssignment[currentMap->pathTypes[i]] = selectedRobot;
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
    float percentage = (activePaths > 0 ? float(drawnPaths) / float(activePaths) : 100);
    
    sprintf(buf, "Drawn (active) Paths: %d", drawnPaths);
    pathStatusLabel->setLabel(buf);
    
    sprintf(buf, "Active Paths Remaining %d, percentage drawn: %.2f%%", drawnPaths, percentage);
    drawnPathLabel->setLabel(buf);
    
    static const ofColor enabled(50, 50, 100), disabled(50, 50, 50);
    
    for (int i=0; i<currentMap->pathTypes.size(); i++) {
        PathGui &pGui = pathGuis[i];
        bool pathActive = currentMap->activePaths[currentMap->pathTypes[i]];
        pGui.togglePath->setBackgroundColor(pathActive ? enabled : disabled);
        pGui.drawOptions->setBackgroundColor(pathActive ? enabled : disabled);
    }
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

	cam.begin();

	ofSetColor(255);
	ofDrawAxis(1.0);
    
	// draw all paths

    for (auto &i : currentMap->mapPathStore) {
        for (auto &j : i.second) {
            if (j.drawn) {
                ofSetColor(255, 255, 255, 100);
            } else if (j.claimed) {
                ofSetColor(200, 0, 200, 100);
            } else if(!currentMap->activePaths[i.first]) {
                ofSetColor(0, 0, 0);
            } else {
                ofSetColor(90, 90, 90);
            }
            
            ofDrawLine(j.segment.start, j.segment.end);
        }
    }
    ofSetLineWidth(1.0);

	// Draw historical positions

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

	// Draw all found markers
	ofPushStyle();
	ofSetColor(255, 127, 0);
	ofNoFill();
	for (auto &m : markersById) {
		// Skip markers that are already on robots
		if (robotsByMarker.find(m.first) != robotsByMarker.end()) {
			continue;
		}

		ArucoMarker &marker = m.second;

		ofPushMatrix();

		ofDrawCircle(marker.planePos.x, marker.planePos.y, 0.02);
		ofDrawBitmapString("m" + ofToString(m.first), ofVec3f(marker.planePos.x, marker.planePos.y, 0));
		ofPopMatrix();
	}
	ofPopStyle();

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
		ofSetColor(255,255,255);
		ofDrawBitmapString(ofToString(r.id), r.planePos.x, r.planePos.y);
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
