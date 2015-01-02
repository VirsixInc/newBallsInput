#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
  angle = 0;
  amtOfPlayers = 4;
  sender.setup("localhost", 9999);
  receiver.setup(7600);
  lastTime = 0;

	kinect.setRegistration(true);
	kinect.init();
	kinect.open();		// opens first available kinect
  camWidth = 320;
  camHeight = 240;
  kinWidth = kinect.getWidth();
  kinHeight = kinect.getHeight();
  r0.allocate(camWidth,camHeight);
  g0.allocate(camWidth,camHeight);
  b0.allocate(camWidth,camHeight);
  r1.allocate(camWidth,camHeight);
  g1.allocate(camWidth,camHeight);
  b1.allocate(camWidth,camHeight);
	grayImage.allocate(camWidth, camHeight);
	grayImageDiff.allocate(camWidth, camHeight);
  grayResizeImg.allocate(kinect.getWidth(), kinect.getHeight());
	manipImage.allocate(camWidth, camHeight);
  colImg.allocate(camWidth, camHeight);
  warpedColImg.allocate(camWidth, camHeight);
  colImgNoCont.allocate(camWidth, camHeight);
  colorResizeImg.allocate(kinect.getWidth(), kinect.getHeight());
	ofSetFrameRate(60);

  gui.addSlider("First Col Thresh", targetColThresh, 0, 255);
  gui.addSlider("Depth Thresh", depthThresh, 0, 255);
  gui.addSlider("Range", range, 0, 20);
  gui.addSlider("Kin Angle", angle, -30, 30);
  gui.addSlider("minContArea", minContArea, 0, 20);
  gui.addSlider("maxContArea", maxContArea, 0, 2000);
  gui.addToggle("Configured",configured);
  gui.addToggle("Color Configured",colorConfig);
  gui.addToggle("Save Points",savePts);
  gui.addToggle("Disable Timer",timerEngaged);
  gui.addToggle("Reset Pts",resetPts);
  gui.addToggle("Save Background",saveBk);
  gui.loadFromXML();

  src[0] = ofPoint(0,0);
  src[1] = ofPoint(camWidth,0);
  src[2] = ofPoint(0,camHeight);
  src[3] = ofPoint(camWidth,camHeight);
  dest[0] = ofPoint(0,0);
  dest[1] = ofPoint(camWidth,0);
  dest[2] = ofPoint(0,camHeight);
  dest[3] = ofPoint(camWidth,camHeight);

  players[0].ballColor = ofColor(255,255,0);
  players[1].ballColor = ofColor(255,0,255);
  players[2].ballColor = ofColor(0,255,255);
  players[3].ballColor = ofColor(0,255,0);
  ofImage imageToLoad;
  imageToLoad.loadImage("backgroundToDiff.png");
  grayImageDiff.setFromPixels(imageToLoad.getPixelsRef());;
}

//--------------------------------------------------------------
void testApp::update() {

  kinect.update();
  colorContourFinder.setThreshold(targetColThresh);
  colorContourFinder.setMinAreaRadius(minContArea);
  colorContourFinder.setMaxAreaRadius(maxContArea);
  if(kinect.isFrameNew()) {
    if(configured){
      grayResizeImg.setFromPixels(kinect.getDepthPixelsRef());
      manipImage.scaleIntoMe(grayResizeImg);
      grayImage.warpIntoMe(manipImage, dest, src);

      colorResizeImg.setFromPixels(kinect.getPixelsRef());
      colImg.scaleIntoMe(colorResizeImg);
      warpedColImg.warpIntoMe(colImg, dest, src);
      colManipImg.setFromPixels(warpedColImg.getPixelsRef());
      if(saveBk){
        saveBk = false;
        grayImageDiff.setFromPixels(grayImage.getPixels(), camWidth, camHeight);
        ofImage imageToSave;
        imageToSave.setFromPixels(grayImageDiff.getPixelsRef());
        imageToSave.saveImage("backgroundToDiff.png");
      }

      grayImage.absDiff(grayImageDiff);
      for(int i=0; i<grayImage.getWidth()*grayImage.getHeight();i++){
        int tempPixel = grayImage.getPixels()[i];
        int xPos = i%camWidth;
        int yPos = i/camWidth;
        int index = colManipImg.getPixelsRef().getPixelIndex(xPos, yPos);
        if(tempPixel == 0){
          colManipImg.setColor(index, ofColor::black);
          continue;
        }
        if(!(ofInRange(tempPixel, depthThresh, depthThresh+range))){
          colManipImg.setColor(index, ofColor::black);
          tempPixel = 0;
        }else{
          tempPixel = 255;
        }
        grayImage.getPixels()[i] = tempPixel;
      }
      warpedColImg.setFromPixels(colManipImg.getPixelsRef());
      contours.findContours(grayImage, minContArea, maxContArea, 8, true);

      if(receiver.hasWaitingMessages()){
        ofxOscMessage m;
        receiver.getNextMessage(&m);
        ofLogNotice("GOT A MESSAGE");
        colImgNoCont.setFromPixels(warpedColImg.getPixelsRef());
        ofxCvBlob blob = contours.blobs.at(0);
        checkForColor(colImgNoCont, blob.centroid.x, blob.centroid.y);
      }
      if(configured && (timerEngaged && timeSinceLastSend + 1.0 < ofGetElapsedTimef())){
        for(int j = 0;j<contours.nBlobs;j++){
          ofxCvBlob blob = contours.blobs.at(j);
          ofxOscMessage m;
          m.setAddress("/checkColor");
          m.addFloatArg(1.0);
          sender.sendMessage(m);
          timeSinceLastSend = ofGetElapsedTimef();
        }
      }
      if(resetPts){
        resetPoints();
        resetPts = false;
      }
      if(savePts){
        savePoints();
        savePts = false;
      }
    }
  }
}
void testApp::checkForColor(ofxCvColorImage imageInQuestion, float x, float y){
  for(int i = 0;i<4;i++){
    colorContourFinder.setTargetColor(players[i].ballColor,ofxCv::TRACK_COLOR_HSV);
    colorContourFinder.findContours(imageInQuestion);
    if(colorContourFinder.size()>0){
      ofLogNotice("PLAYER FOUND: " + ofToString(i));
      players[i].ballFound = true;
      ofxOscMessage m;
      m.setAddress("/shoot");
      m.addFloatArg(x);
      m.addFloatArg(y);
      m.addIntArg(i);
      sender.sendMessage(m);
    }
  }
}
//--------------------------------------------------------------
void testApp::draw() {
  ofSetHexColor(0xff0000);

  ofSetColor(255, 255, 255);
  kinect.draw(0,0,camWidth,camHeight);
  warpedColImg.draw(camWidth, 0, camWidth, camHeight);
  grayImage.draw(camWidth*2, 0, camWidth, camHeight);
  colImgNoCont.draw(0,camHeight,camWidth,camHeight);
  if(configured){
    colorContourFinder.draw();
    contours.draw();
  }else{
    ofxXmlSettings settings;
    for(int j=0; j<4;j++){
      ofSetColor(255,255,255);
      settings.loadFile("points.xml");
      settings.pushTag("positions");
      settings.pushTag("position", j);
      if(selectedCorner < 0){
        if(settings.getValue("X", -1) != -1){
          dest[j].x = settings.getValue("X", -1);
        }
        if(settings.getValue("Y", -1) != -1){
          dest[j].y = settings.getValue("Y", -1);
        }
      }
      
      switch(j){
        case 0:
          ofSetColor(0,0,255);
          break;
        case 1:
          ofSetColor(255,0,0);
          break;
        case 2:
          ofSetColor(255,255,0);
          break;
        case 3:
          ofSetColor(0,255,0);
          break;
      }
      ofCircle(dest[j].x, dest[j].y, 3);
      settings.popTag();
      settings.popTag();
    }

  }
  gui.draw();
  for(int i = 0;i<amtOfPlayers;i++){
    ofSetColor(255,255,255);
    ofSetColor(players[i].ballColor);
    ofCircle(camWidth+50*i, camHeight+100, 30);
  }
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.close();
}
void testApp::resetPoints(){
  ofxXmlSettings settings;
  settings.addTag("positions");
  settings.pushTag("positions");
  for(int i = 0;i<4;i++){
    if(dest[i].x != -1 && dest[i].y != -1){
      settings.addTag("position");
      settings.pushTag("position", i);
      settings.setValue("X", i*10);
      settings.setValue("Y", i*10);
      settings.popTag();
    }
  }
}
void testApp::savePoints(){
  ofxXmlSettings settings;
  settings.addTag("positions");
  settings.pushTag("positions");
  for(int i = 0;i<4;i++){
    if(dest[i].x != -1 && dest[i].y != -1){
      settings.addTag("position");
      settings.pushTag("position", i);
      settings.setValue("X", dest[i].x);
      settings.setValue("Y", dest[i].y);
      settings.popTag();
    }
  }
  settings.popTag();
  settings.saveFile("points.xml");
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
  switch(key){
    case '1':
      id = key-49;
      idSet = true;
      break;
    case '2':
      id = key-49;
      idSet = true;
      break;
    case '3':
      id = key-49;
      idSet = true;
      break;
    case '4':
      id = key-49;
      idSet = true;
      break;
    case '[':
			angle++;
			//if(angle>30) angle=30;
			break;
			
		case ']':
			angle--;
			//if(angle<-30) angle=-30;
			break;
    case OF_KEY_LEFT:
      gui.toggleDraw();
    }
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
  if (selectedCorner > -1){
    dest[selectedCorner].x = x;
    dest[selectedCorner].y = y;
  }
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

  selectedCorner = -1;
  float smallestDist  = 999999;
  float clickRadius = 10;

  for (int j = 0; j < 4; j++){
    ofPoint inputPt;
    inputPt.x = dest[j].x;
    inputPt.y = dest[j].y;
    inputPt.z = 0;
    float len = sqrt( (inputPt.x - x) * (inputPt.x - x) +
              (inputPt.y - y) * (inputPt.y - y));
    if (len < clickRadius && len < smallestDist){
      selectedCorner  = j;
      smallestDist = len;
    }
  } 
  if(selectedCorner == -1 && idSet){
    idSet = false;
    colorJustAcquired = warpedColImg.getPixelsRef().getColor(x, y);
    ofLogNotice(ofToString(x) + "  " + ofToString(y));
    players[id].ballColor = colorJustAcquired;
    colorContourFinder.setTargetColor(players[id].ballColor,ofxCv::TRACK_COLOR_HSV);
  }
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
