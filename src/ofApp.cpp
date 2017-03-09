#include "ofApp.h"

static const string kDefaultMapPath = "test.svg";
static const string kDownloadPath = "/Users/anderson/Downloads/";

static const string kRPiHost = "192.168.7.52";
static const int kRPiPort = 5300;

static const float kMapWidthM = 1.0f;
static const float kMapHeightM = 1.0f;
static const float kMapOffsetXM = -kMapWidthM / 2.0;
static const float kMapOffsetYM = -kMapHeightM / 2.0;
static const ofRectangle kCropBox(ofVec2f(-0.45, -0.4), ofVec2f(0.45, 0.4));
static const ofRectangle kSafetyBox(ofVec2f(-0.5, -0.45), ofVec2f(0.5, 0.45));

static const bool debugging = true;

static const float kRobotSafetyDiameterM = 0.15f;
static const float kRobotOuterSafetyDiameterM = 0.25f;

static const int kNumPathsToSave = 10000;

static const ofVec2i kNumGridPtsPerAxis(10, 10);
static const ofVec2f kGridStart = kCropBox.position;
static const ofVec2f kCellSize(kCropBox.width / kNumGridPtsPerAxis.x, kCropBox.height / kNumGridPtsPerAxis.y);

static char udpMessage[1024];
static char buf[1024];

// centered
inline ofVec2f pathPtToPlanePt(ofVec2i pathPt) {
	return ofVec2f(pathPt.x * kCellSize.x, pathPt.y * kCellSize.y) + kCellSize / 2.0 + kGridStart;
}

inline ofVec2i planePtToPathPt(ofVec2f planePt) {
	return ofVec2i(floor((planePt.x - kGridStart.x) / kCropBox.width * kNumGridPtsPerAxis.x),
				   floor((planePt.y - kGridStart.y) / kCropBox.height * kNumGridPtsPerAxis.y));
}

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
	r01->setCommunication("192.168.7.74", 5111);
	robotIds.push_back(r01->id);
	r01->potentialField.walls = kSafetyBox;

	Robot *r02 = new Robot(2, 26, "Camille");
	robotsById[r02->id] = r02;
	robotsByMarker[r02->markerId] = r02;
	r02->setCommunication("192.168.7.73", 5111);
	robotIds.push_back(r02->id);
	r02->potentialField.walls = kSafetyBox;

#if SIMULATING
	r01->planePos = ofVec2f(-0.35);
	r02->planePos = ofVec2f(0.35);
	r01->upVec = ofVec2f(0.0, 1.0);
	r02->upVec = ofVec2f(0.0, 1.0);
#endif

//	Robot *r03 = new Robot(3, 24, "Archie");
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
		rGui.kp = rGui.folder->addSlider("kp", 0, 30000);
		rGui.kp->setValue(r.targetLineKp);
		rGui.ki = rGui.folder->addSlider("ki", 0, 2000);
		rGui.ki->setValue(r.targetLineKi);
		rGui.kd = rGui.folder->addSlider("kd", 0, 50);
		rGui.kd->setValue(r.targetLineKd);
		rGui.kMaxI = rGui.folder->addSlider("kMaxI", 0, 20000);
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
		ofFileDialogResult openFileResult = ofSystemLoadDialog("Select a map svg!");
		if (openFileResult.bSuccess){
			cout << "User loaded: " << openFileResult.getPath() << endl;
			loadMap(openFileResult.getPath());
		} else {
			cout << "No map selected" << endl;
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
	pathGui = NULL;

	string mostRecent = currentMap->getMostRecentMap(kDownloadPath);
	if (mostRecent.size() > 0) {
		cout << "Found a recent path in downloads: " << mostRecent << endl;
		mapPath = mostRecent;
	} else {
		mapPath = kDefaultMapPath;
	}
	loadMap(mapPath);

	setState(MR_STOPPED);
}

void ofApp::loadMap(const string &newMapPath) {
	mapPath = newMapPath;
	currentMap->loadMap(mapPath);

	for (auto &p : robotsById) {
		int id = p.first;
		Robot &r = *p.second;

		r.stop();
		r.pathTypes.clear();
		for (auto pathType : currentMap->pathTypes) {
			r.addPathType(pathType);
		}
	}

	setupMapGui();
}

void ofApp::setupMapGui() {
	// silence logger:
	guiLogger->quiet();

	// set up paths gui
	if (pathGui) {
		delete pathGui;
	}

	pathGui = new ofxDatGui( ofxDatGuiAnchor::TOP_LEFT );
	pathGui->setTheme(new ofxDatGuiThemeMidnight());

	pathGui->addHeader("Paths GUI");
	pathLabel = pathGui->addLabel("Total Active Paths: " + ofToString(currentMap->getActivePathCount()));
	pathStatusLabel = pathGui->addLabel("");
	drawnPathLabel = pathGui->addLabel("");

	pathGui->addBreak();

	pathGuis.clear();
	for (auto &pathType : currentMap->pathTypes) {
		PathGui pGui;
		pGui.pathType = pathType;

		pGui.toggle = pathGui->addToggle("PATH: " + ofToString(pathType) + " \t\t- " + ofToString(currentMap->getPathCount(pathType)));
		pGui.toggle->setChecked(true);
		pGui.toggle->onToggleEvent([&pathType, this](ofxDatGuiToggleEvent e) {
			currentMap->setPathActive(pathType, e.checked);
		});

		pGui.folder = pathGui->addFolder("Robot Select");
		for (auto &pair : robotsById) {
			Robot &r = *pair.second;

			ofxDatGuiToggle *t = pGui.folder->addToggle(r.name + " (" + ofToString(r.id) + ")");
			t->setChecked(true);
			t->onToggleEvent([&r, &pathType](ofxDatGuiToggleEvent e) {
				if (e.checked) {
					r.addPathType(pathType);
				} else {
					r.removePathType(pathType);
				}
			});
		}

		pathGui->addBreak();
		pathGuis[pathType] = pGui;
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
void ofApp::update() {
#if SIMULATING
	for (auto &p : robotsById) {
		p.second->updateSimulation(ofGetLastFrameTime());
	}
#endif
	handleOSC();
	receiveFromRobots();
	planRobotPaths();
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

void ofApp::planRobotPaths() {
//	PathPlanner pathPlanner(kNumGridPtsPerAxis.x, kNumGridPtsPerAxis.y);
//
//	// First, add robot positions
//	for (int robotId : robotIds) {
//		Robot &r = *robotsById[robotId];
//
//		r.needsReplan = false;
//		ofVec2i currentCell = planePtToPathPt(r.planePos);
//		if (pathPlanner.at(currentCell)->containsRobotId >= 0) {
//			cout << "Cell contains two robots!!!" << endl;
//		} else {
//			pathPlanner.at(currentCell)->containsRobotId = robotId;
//		}
//	}
//
//	// Next, add robot drawing
//	for (int robotId : robotIds) {
//		Robot &r = *robotsById[robotId];
//
//		if (r.state == R_WAITING_TO_DRAW) {
//			for (ofVec2i &pt : r.drawPathMask) {
//				if (pathPlanner.at(pt)->onDrawPathForRobotId >= 0 && pathPlanner.at(pt)->onDrawPathForRobotId != robotId) {
//					r.needsReplan = true;
//				} else {
//					pathPlanner.at(pt)->onDrawPathForRobotId = robotId;
//				}
//			}
//		} else if (r.state == R_DRAWING) {
//			for (ofVec2i &pt : r.drawPathMask) {
//				if (pathPlanner.at(pt)->onDrawPathForRobotId >= 0 && pathPlanner.at(pt)->onDrawPathForRobotId != robotId) {
//					r.stop();
//					cout << "Going to draw into another robot!!" << endl;
//				} else {
//					pathPlanner.at(pt)->onDrawPathForRobotId = robotId;
//				}
//			}
//		}
//	}
//
//		//			static const int linear = 50;
//		//			static const int radial = 8;
//		//			ofVec2f cur = r.startPlanePos;
//		//			ofVec2f target = r.targetPlanePos;
//		//			ofVec2f diff = target - cur / (linear - 1);
//		//			for (int i = 0; i < linear + 1; ++i) {
//		//				for (int j = 0; j < radial; ++j) {
//		//					ofVec2f rad = cur + ofVec2f(0.25f).rotate(360.0 * j / radial);
//		//					ofVec2i currentCell = planePtToPathPt(rad);
//		//
//		//					if (pathPlanner.at(currentCell).onDrawPathForRobotId >= 0 ||
//		//						pathPlanner.at(currentCell).containsRobotId >= 0) {
//		//						cout << "Drawing path contains a robot or another robot's drawing path!" << endl;
//		//					}
//		//
//		//					pathPlanner.at(currentCell).onDrawPathForRobotId = robotId;
//		//				}
//		//				cur += diff;
//		//			}
//
//	// Add next paths
//	for (int robotId : robotIds) {
//		Robot &r = *robotsById[robotId];
//
//		if (r.state == R_POSITIONING) {
//			ofVec2i nextCell = r.path[r.pathIdx];
//			if (pathPlanner.at(nextCell)->containsRobotId >= 0) {
//				if (pathPlanner.at(nextCell)->containsRobotId > robotId) {
//					r.setState(R_WAITING_TO_POSITION);
//				} else {
//					r.needsReplan = true;
//				}
//			} else if (pathPlanner.at(nextCell)->nextPathForRobotId >= 0) {
//				r.needsReplan = true;
//			} else {
//				pathPlanner.at(nextCell)->nextPathForRobotId = robotId;
//			}
//		}
//	}
//
//	// Find paths
////	for (auto &p : robotsById) {
////		int id = p.first;
////		Robot &r = *p.second;
////
////		if (r.state == R_READY_TO_POSITION) {
////			MapPath *mp = currentMap->nextPath(r.avgPlanePos, r.id, r.lastHeading, r.pathTypes);
////			robotPaths[id] = mp;
////
////			if (r.path.size() > 0) {
////				// We're on a path, keep going.
////				if (r.pathIdx == r.path.size() - 1) {
////					// We made it to the end of the path, go to the real target
////					r.navigateTo(mp->segment.start);
////				} else {
////					r.navigateTo(pathPtToPlanePt(r.path[r.pathIdx]));
////				}
////			} else {
////				// TODO: find a path!
////			}
////		} else if (r.state == R_DONE_POSITIONING) {
////			if (r.path.size() > 0) {
////				if (r.pathIdx == r.path.size() - 1) {
////					MapPath *mp = robotPaths[id];
////					if (mp == NULL) {
////						r.stop();
////						r.setState(R_READY_TO_POSITION);
////					} else {
////						r.drawLine(mp->segment.start, mp->segment.end);
////					}
////				} else {
////					r.setState(R_READY_TO_POSITION);
////				}
////			} else {
////				cout << "Error: finished positioning without a path somehow" << endl;
////			}
////		} else if (r.state == R_DONE_DRAWING) {
////			if (robotPaths.find(id) == robotPaths.end()) {
////				// Error!
////				r.stop();
////			} else {
////				MapPath *mp = robotPaths[id];
////				if (mp == NULL) {
////					// error!
////					r.stop();
////				} else {
////					mp->drawn = true;
////					robotPaths.erase(id);
////					if (debugging) {
////						r.planePos = mp->segment.end;
////						r.avgPlanePos = mp->segment.end;
////						r.slowAvgPlanePos = mp->segment.end;
////					}
////					// TODO: make this a function on robot specifically
////					r.setState(R_READY_TO_POSITION);
////				}
////			}
////		}
////
////		ofVec2i current = planePtToPathPt(r.planePos);
////		ofVec2i target = planePtToPathPt(r.finalTarget);
////
////		vector<ofVec2i> path = pathPlanner.findPath(current, target);
////		if (path.size() > 0) {
////			r.path = path;
////		} else {
////			// No path!
////		}
////	}
//
//	drawnPathPlanner = pathPlanner;
}

void ofApp::commandRobots() {
	for (auto &p : robotsById) {
		int id = p.first;
		Robot &r = *p.second;

		vector<PotentialFieldObstacle> o;
		for (auto &p2 : robotsById) {
			int id2 = p2.first;
			if (id == id2) continue;

			o.push_back({ p2.second->planePos, 0.1f });
		}
		r.updateObstacles(o);

//		for (auto &p2 : robotsById) {
//			int id2 = p2.first;
//			if (id == id2) continue;
//
//			Robot &r2 = *p2.second;
//
//			float dist = r.planePos.distance(r2.planePos);
//			if (dist < kRobotSafetyDiameterM) {
//				// Too close! Stop entirely.
//				r.stop();
//				r2.stop();
//				cout << "Stopping both robots, way too close " << dist << endl;
//			} else if (dist < kRobotOuterSafetyDiameterM) {
//				// Noone is drawing
//				ofVec2f oneToTwo = r2.planePos - r.planePos;
//				ofVec2f midpoint = oneToTwo / 2.0 + r.planePos;
//				oneToTwo.normalize();
//
//				bool success = false;
//
//				ofVec2f r2left = r2.planePos + oneToTwo.rotate(-90) * 0.25f;
//				ofVec2f r2right = r2.planePos + oneToTwo.rotate(90) * 0.25f;
//				if (!success && r2.state != R_DRAWING) {
//					if (kSafetyBox.inside(r2left)) {
//						unclaimPath(id);
//						unclaimPath(id2);
//						r2.navigateTo(r2left);
//						r.stop();
//						success = true;
//					} else if (kSafetyBox.inside(r2right)) {
//						unclaimPath(id);
//						unclaimPath(id2);
//						r2.navigateTo(r2right);
//						r.stop();
//						success = true;
//					}
//				}
//
//				if (!success && r.state != R_DRAWING) {
//					ofVec2f r1left = r.planePos + oneToTwo.rotate(90) * 0.25f;
//					ofVec2f r1right = r.planePos + oneToTwo.rotate(-90) * 0.25f;
//					if (kSafetyBox.inside(r1left)) {
//						unclaimPath(id);
//						unclaimPath(id2);
//						r.navigateTo(r1left);
//						r2.stop();
//						success = true;
//					} else if (kSafetyBox.inside(r1right)) {
//						unclaimPath(id);
//						unclaimPath(id2);
//						r.navigateTo(r1right);
//						r2.stop();
//						success = true;
//					}
//				}
//
//				// Couldn't force robots to renavigate around each other
//				if (!success) {
//					r.stop();
//					r2.stop();
//					cout << "Stopping both robots " << dist << endl;
//				}
//			}
//		}

		// Determine draw state
		if (state == MR_STOPPED) {
			unclaimPath(id);
			r.stop();
		} else if (r.state == R_STOPPED && state == MR_RUNNING) {
			unclaimPath(id);
			r.start();
		} else if (r.state == R_READY_TO_POSITION && state == MR_RUNNING) {
			MapPath *mp = currentMap->nextPath(r.avgPlanePos, r.id, r.lastHeading, { "major_road" });
			robotPaths[id] = mp;

			if (mp != NULL) {
				if (mp->segment.start.distance(r.avgPlanePos) > mp->segment.end.distance(r.avgPlanePos)) {
					ofVec2f tmp = mp->segment.start;
					mp->segment.start = mp->segment.end;
					mp->segment.end = tmp;
				}

				mp->claimed = true;
				r.navigateWithGradientTo(mp->segment.start);
                r.lastHeading = atan2(mp->segment.end.x - mp->segment.start.x, mp->segment.end.y - mp->segment.start.y)*180/3.14159;
			} else {
				cout << "No more paths to draw!" << endl;
			}
		} else if (r.state == R_DONE_POSITIONING && state == MR_RUNNING) {
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
    float percentage = (activePaths > 0 ? float(drawnPaths) / float(activePaths) : 1.0);
    
    sprintf(buf, "Drawn (active) Paths: %d", drawnPaths);
    pathStatusLabel->setLabel(buf);
    
    sprintf(buf, "Active Paths Remaining %d, percentage drawn: %.1f%%", drawnPaths, percentage * 100);
    drawnPathLabel->setLabel(buf);
    
//    static const ofColor enabled(50, 50, 100), disabled(50, 50, 50);
//    
//    for (int i=0; i<currentMap->pathTypes.size(); i++) {
//        PathGui &pGui = pathGuis[i];
//        bool pathActive = currentMap->activePaths[currentMap->pathTypes[i]];
//        pGui.togglePath->setBackgroundColor(pathActive ? enabled : disabled);
//        pGui.drawOptions->setBackgroundColor(pathActive ? enabled : disabled);
//    }
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
				continue;
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
//
//	ofPushStyle();
//	ofNoFill();
//	for (int x = 0; x < drawnPathPlanner.width; ++x) {
//		for (int y = 0; y < drawnPathPlanner.height; ++y) {
//			GridNode &g = *drawnPathPlanner.at(ofVec2i(x,y));
//			ofVec2f planePt = pathPtToPlanePt(g.loc);
//
//			if (g.containsRobotId >= 0) {
//				ofSetColor(255, 255, 255);
//			} else if (g.onDrawPathForRobotId >= 0) {
//				ofSetColor(255, 0, 0);
//			} else if (g.nextPathForRobotId >= 0) {
//				ofSetColor(255, 255, 0);
//			} else {
//				ofSetColor(40, 40, 40);
//			}
//			ofDrawRectangle(planePt - kCellSize * 0.8 / 2.0, kCellSize.x * 0.8, kCellSize.y * 0.8);
//		}
//	}
//	ofPopStyle();

	ofPushStyle();
	ofNoFill();
	ofSetColor(255);
	PotentialField pf = robotsById[1]->potentialField;//{ kSafetyBox, { { robotsById[2]->planePos, 0.1 } }, ofVec2f(0) };
	float gridSize = kSafetyBox.getWidth() / 100.0;
	for (float x = kSafetyBox.getLeft() - 0.5; x < kSafetyBox.getRight() + 0.5; x += gridSize) {
		for (float y = kSafetyBox.getTop() - 0.5; y < kSafetyBox.getBottom() + 0.5; y += gridSize) {
			float h = pf.fieldAtPoint(ofVec2f(x,y));
			ofDrawRectangle(x, y, h / 10000.0, gridSize, gridSize);
		}
	}
	ofPopStyle();

	for (map<int, Robot*>::iterator it = robotsById.begin(); it != robotsById.end(); ++it) {
		int robotId = it->first;
		Robot &r = *it->second;

		if (robotId == 1) {
			ofSetColor(255, 255, 0);
		} else if (robotId == 2) {
			ofSetColor(0, 255, 255);
		} else if (robotId == 3) {
			ofSetColor(255, 0, 0);
		}
		for (auto &waypoint : r.path) {
//			float gridSize = 1.0 / float(kNumGridPtsPerAxis);
//			ofVec2f planePt = ofVec2f(waypoint.x, waypoint.y) * gridSize + ofVec2f(gridSize / 2.0) - 0.5;

//			ofDrawRectangle(planePt - ofVec2f(gridSize * 0.4), gridSize * 0.8, gridSize * 0.8);
		}

		ofDrawSphere(r.potentialField.goal, 0.02);

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
