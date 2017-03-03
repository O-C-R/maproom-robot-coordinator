#include "ofApp.h"

//static const string currentFile = "maproom-2017-02-26T04-23-37.025Z.svg";
//static const string currentFile = "test_case.svg";
static const string currentFile = "maproom-2017-03-02T19-03-12.993Z.svg";
static const float kMarkerSize = 0.2032f;

//static const float kMapWidthM = -2.72f;
//static const float kMapHeightM = -2.72f;
static const float kMapWidthM = -2.4f;
static const float kMapHeightM = -2.4f;
static const float kMapOffsetXM = -kMapWidthM / 2.0 - 0.15;
static const float kMapOffsetYM = -kMapHeightM / 2.0 - 0.05;

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

	Robot *r01 = new Robot(1, 23, "Delmar");
	robotsById[r01->id] = r01;
	robotsByMarker[r01->markerId] = r01;
	r01->setCommunication("192.168.7.70", 5111);

	Robot *r02 = new Robot(2, 24, "Archie");
	robotsById[r02->id] = r02;
	robotsByMarker[r02->markerId] = r02;
	r02->setCommunication("192.168.7.72", 5111);

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
        rGui.enableToggle = rGui.folder->addToggle("Enabled", true);
        rGui.calibrateButton = rGui.folder->addButton("Calibrate");
        rGui.advanceButton = rGui.folder->addButton("Skip path");
        rGui.rotationAngleSlider = rGui.folder->addSlider("Rotation Angle: " + ofToString(r.id), 0, 360, r.rot);

		rGui.enableToggle->onToggleEvent([&r](ofxDatGuiToggleEvent e)  {
			r.enabled = e.checked;
		});
		rGui.calibrateButton->onButtonEvent([&r](ofxDatGuiButtonEvent e) {
			r.calibrate();
		});
		rGui.advanceButton->onButtonEvent([&r](ofxDatGuiButtonEvent e) {
			r.setState(R_DONE_DRAWING);
		});
		rGui.rotationAngleSlider->onSliderEvent([&r](ofxDatGuiSliderEvent e) {
			r.targetRot = e.value;
		});

		rGui.folder->addBreak();

		rGui.kp = rGui.folder->addSlider("kp", 0, 10000);
		rGui.kp->setValue(2500.0);
		rGui.ki = rGui.folder->addSlider("ki", 0, 100);
		rGui.ki->setValue(0.0);
		rGui.kd = rGui.folder->addSlider("kd", 0, 10000);
		rGui.kd->setValue(0.0);
		ofxDatGuiSlider *kiMax = rGui.folder->addSlider("kiMax", 0, 10000);
		kiMax->setValue(0.0);
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


	gui->addFRM();

	// Listen for messages from camera
	oscReceiver.setup( PORT );

	// Listen for messages from the robots

	robotReceiver.Create();
	robotReceiver.Bind(5101);
	robotReceiver.SetNonBlocking(true);
    
    currentMap = new Map(kMapWidthM, kMapHeightM, kMapOffsetXM, kMapOffsetYM);
    loadMap(currentFile);

	setState(MR_STOPPED);
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
				ofVec2f pos(jsonMsg["pos"][i][0].asFloat(), jsonMsg["pos"][i][1].asFloat());
				ofVec2f up(jsonMsg["up"][i][0].asFloat(), jsonMsg["up"][i][1].asFloat());

				if (robotsByMarker.find(markerId) != robotsByMarker.end()) {
					robotsByMarker[markerId]->updateCamera(pos, up);
				}

				if (markersById.find(markerId) == markersById.end()) {
					markersById[markerId] = ArucoMarker(markerId);
				}
				markersById[markerId].updateCamera(pos, up);
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
				ofSetColor(255, 255, 255);
			} else if (j.claimed) {
				ofSetColor(200, 0, 200);
			} else {
				ofSetColor(50, 50, 50);
			}

            ofDrawLine(j.segment.start, j.segment.end);
        }
    }

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
		if (r.state == R_DRAWING) {
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
