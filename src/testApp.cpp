#include "testApp.h"

const int depthImageAverageTime = 20;

//--------------------------------------------------------------
void testApp::setup() {
    amtOfPlayers = 4;
    votesReq = 5;
    sender.setup("localhost", 9999);
    receiver.setup(7600);
    
    kinect.setRegistration(true); // Syncs up depth and RGB
    kinect.init();
    kinect.open();                // Opens first available kinect
    
    lastTime = 0;
    
    camWidth = 320;
    camHeight = 240;
    kinWidth = kinect.getWidth();
    kinHeight = kinect.getHeight();
    
    colImg.allocate(camWidth, camHeight);
    grayImage.allocate(camWidth, camHeight);
    grayForColor.allocate(camWidth, camHeight);
    grayImageDiff.allocate(camWidth, camHeight);
    temp_depth.allocate(kinect.width, kinect.height);
    temp_scale.allocate(camWidth, camHeight);
    imgToCheck.allocate(camWidth,camHeight);
    
    edge.allocate(camWidth, camHeight, OF_IMAGE_GRAYSCALE);
    warpedColImg.allocate(camWidth, camHeight);
    colImgNoCont.allocate(camWidth, camHeight);
    
    ofSetFrameRate(60);
    
    src[0] = ofPoint(0,0);
    src[1] = ofPoint(camWidth,0);
    src[2] = ofPoint(camWidth,camHeight);
    src[3] = ofPoint(0,camHeight);
    
    dest[0] = ofPoint(0,0);
    dest[1] = ofPoint(camWidth,0);
    dest[2] = ofPoint(camWidth,camHeight);
    dest[3] = ofPoint(0,camHeight);
    
    state = ConfigBackground;
    timer = 0;
    whiteScreen = false;
    
    players[0].ballColor = ofColor(255,0,0);
    players[1].ballColor = ofColor(0,255,0);
    players[2].ballColor = ofColor(0,255,0);
    players[3].ballColor = ofColor(0,255,0);
    
    gui.addSlider("First Col Thresh", targetColThresh, 0, 255);
    gui.addSlider("Depth Thresh", depthThresh, 0, 255);
    gui.addSlider("Range", range, 0, 20);
    gui.addSlider("minContArea", minContArea, 0, 1000);
    gui.addSlider("maxContArea", maxContArea, 0, 2000);
    gui.addSlider("minPartEffect", minPartEffect, 0, 10000);
    gui.addSlider("maxPartEffect", maxPartEffect, 0, 20000);
    gui.addSlider("partThresh", partThresh, 0, 255);
    gui.addToggle("Configured", configured);
    gui.addToggle("Color Configured", colorConfig);
    gui.addToggle("Disable Timer", timerEngaged);
    gui.addToggle("Save Background", saveBk);
    gui.addToggle("Flip", flip);
    gui.addSlider("minVariationDistance", minVariationDistance, 0.01, 1000.0);
    gui.addSlider("lifeTime", lifeTime, 0, 150);
    gui.loadFromXML();
    
    ballTracker.Init(minVariationDistance, lifeTime);
    
    partEffectFinder.setTargetColor(ofColor::white,ofxCv::TRACK_COLOR_HSV);
}

//--------------------------------------------------------------
void testApp::update() {
  colorContourFinder.setThreshold(targetColThresh);
  colorContourFinder.setMinArea(minContArea);
  colorContourFinder.setMaxArea(maxContArea);
    partEffectFinder.setMinArea(minPartEffect); // TODO tweak. Seems good tho.
    partEffectFinder.setMaxArea(maxPartEffect); // TODO tweak. Seems good tho.
    partEffectFinder.setThreshold(partThresh); // TODO tweak. Seems good tho.
    
    ballTracker.SetMinDistance(minVariationDistance);
    ballTracker.SetLifeTime(lifeTime);
    ballTracker.Update();
    
    CheckOSCMessage();
    
    kinect.update();
    if(kinect.isFrameNew()) {
        
        UpdateImages();
        if (saveBk)
          SaveBackground();
        
        switch(state) {
            case Config:
                break;
                
            case ConfigScreen:
                ConfigureScreen();
                break;
                
            case ConfigBackground:
              SaveBackground();
              SendMessage("/config/done");
              state = ConfigColors;

              ThresholdImages();
              break;
            case ConfigColors:
              SendMessage("/config/done");
              state = Main;
              break;
                
            case Main:
              if(configured) {
                ThresholdImages();

                contours.findContours(grayImage, minContArea, maxContArea, 8, true);
                partEffectFinder.findContours(colImgNoCont);
                if(contours.nBlobs > 0) {
                  ofxCvBlob blob = contours.blobs.at(0);
                  if(!whiteScreen && timeSinceLastWhiteFound + 0.5 < ofGetElapsedTimef()) {
                    partEffectFinder.findContours(colImgNoCont);
                    if(partEffectFinder.size() > 0){
                      ofxCvColorImage tmpColCont;
                      tmpColCont.setFromPixels(warpedColImg.getPixelsRef());
                      tmpColCont.setROI(blob.boundingRect);
                      tmpColCont.setFromPixels(tmpColCont.getRoiPixelsRef());
                      //imgToCheck.scaleIntoMe(tmpColCont);
                      imgToCheck.setFromPixels(tmpColCont.getRoiPixelsRef());
                      imgToCheck.convertToRange(1,200);
                      checkForColor(imgToCheck, hitPoint);
                      timeSinceLastWhiteFound = ofGetElapsedTimef();
                      whiteScreen = true;
                    }
                  }else{
                    whiteScreen = false;
                  }
                  if(timerEngaged && timeSinceLastSend + 0.5 < ofGetElapsedTimef()) {
                    hitPoint = blob.centroid;
                    //imgToCheck.drawROI(0,camHeight);
                    SendHitMessage("/checkColor", hitPoint, 0);
                    ofLogNotice("Blob found");
                    timeSinceLastSend = ofGetElapsedTimef();
                  }
                }
              }
              break;
        }
        
    }
    
}

//--------------------------------------------------------------
void testApp::UpdateImages() {
    colImg.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);

    warpedColImg.scaleIntoMe(colImg);
    temp_color.setFromPixels(warpedColImg.getPixelsRef());
    warpedColImg.warpIntoMe(temp_color, dest, src);
    
    temp_depth.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
    temp_scale.scaleIntoMe(temp_depth);
    grayImage.warpIntoMe(temp_scale, dest, src); //Temp stuff. Gotta clean it up
}

//--------------------------------------------------------------
void testApp::ThresholdImages() {
    grayImage.absDiff(grayImageDiff);
    unsigned char* grayPixels = grayImage.getPixels();
    //grayForColor.setFromPixels(grayPixels.getPixels());

    unsigned char* colorPixels = warpedColImg.getPixels();
    colImgNoCont.setFromPixels(warpedColImg.getPixelsRef());
    for (int i = 0, n = grayImage.getWidth() * grayImage.getHeight(); i < n; i++) {
        if(ofInRange(grayPixels[i],depthThresh,depthThresh+range)){
            grayPixels[i] = 255;
        }else{
            grayPixels[i] = 0;
            colorPixels[i*3] = colorPixels[i*3+1] = colorPixels[i*3+2] = 0;
        }
    }
    grayImage = grayPixels;
    warpedColImg = colorPixels;
}

//--------------------------------------------------------------
void testApp::ConfigureScreen() {
    timer++;
    colorContourFinder.setTargetColor(ofColor::white);
    colorContourFinder.setMinArea(100); // TODO tweak. Seems good tho.
    colorContourFinder.setThreshold(100); // TODO tweak. Seems good tho.
    colorContourFinder.resetMaxArea();
    colorContourFinder.findContours(warpedColImg);
    
    //TODO cant just give up like this.
    if(colorContourFinder.size() < 1) {
        ofLogNotice("Less than 1 contour(s) found");
        return;
    }
    if(colorContourFinder.size() > 1) {
        ofLogNotice("More than 1 contour(s) found");
        return;
    }
    
    if(timer > 20) { //Delay to let front end change scenes
        const std::vector<ofPoint> contPts = colorContourFinder.getPolyline(0).getVertices();
        get_corners(contPts, &contCorners);

        dest[0] = contCorners.tl;//ofPoint(rect.x, rect.y);
        dest[1] = contCorners.tr;//ofPoint(rect.x + rect.width, rect.y);
        dest[2] = contCorners.br;//ofPoint(rect.x + rect.width, rect.y + rect.height);
        dest[3] = contCorners.bl;//ofPoint(rect.x, rect.y + rect.height);
        
        ofxXmlSettings settings;
        settings.addTag("positions");
        settings.pushTag("positions");
        for(int i = 0; i < 4; i++){
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
       
        SendMessage("/config/cornerParsed");
        state = ConfigBackground;
    }
}

//--------------------------------------------------------------
void testApp::checkForColor(ofxCvColorImage imageInQuestion, ofPoint ptToFire){
  for(int i = 0;i<amtOfPlayers;i++){
    colorContourFinder.setTargetColor(players[i].ballColor, ofxCv::TRACK_COLOR_HSV);
    colorContourFinder.findContours(imageInQuestion);
    if(colorContourFinder.size() > 0){
        ofLogNotice("PLAYER FOUND: " + ofToString(i));
        players[i].ballFound = true;
        SendHitMessage("/shoot", ptToFire, i);
        break;
    }
  }
}
//--------------------------------------------------------------
void testApp::SaveBackground() {
    saveBk = false;
    unsigned char* pixels = grayImage.getPixels();
    grayImageDiff.setFromPixels(pixels, camWidth, camHeight);
    //timer = 0;
    //state = ConfigBackground;
}

//--------------------------------------------------------------
void testApp::draw() {
    ofSetHexColor(0xff0000);
    
    ofSetColor(255, 255, 255);
    colImgNoCont.draw(0,0,camWidth,camHeight);
    kinect.draw(camWidth*2,camHeight,camWidth,camHeight);
    partEffectFinder.draw();
    imgToCheck.draw(0, camHeight);
    warpedColImg.draw(camWidth*2,0);

    grayImage.draw(camWidth, 0, camWidth, camHeight);
    grayImageDiff.draw(camWidth, camHeight, camWidth, camHeight);

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
    for(int i = 0;i < amtOfPlayers;i++){
        ofSetColor(255,255,255);
        ofSetColor(players[i].ballColor);
        ofCircle(camWidth+50*i, 2*camHeight+100, 30);
    }
}

//--------------------------------------------------------------
void testApp::CheckOSCMessage() {
    while(receiver.hasWaitingMessages()){
        // get the next message
        ofxOscMessage m;
        if(receiver.getNextMessage(&m)) {
            string addr = m.getAddress();
            if(addr == "/config/corner") {
                state = ConfigScreen;
                timer = 0;
                dest[0] = ofPoint(0,0);
                dest[1] = ofPoint(camWidth,0);
                dest[2] = ofPoint(camWidth,camHeight);
                dest[3] = ofPoint(0,camHeight);
            } else if(addr == "/startGame") {
                state = Main;
            } else if(addr == "/readyCheck") {
                whiteScreen = true;
            }
        }
    }
}

//--------------------------------------------------------------
void testApp::SendMessage(string message) {
    ofxOscMessage m;
    m.setAddress(message);
    sender.sendMessage(m);
}

//--------------------------------------------------------------
void testApp::SendHitMessage(string message, ofPoint pos, int player) {
    if (player == -1)
        return;
    
    ofxOscMessage m;
    m.setAddress(message);
    float x = pos.x / camWidth;
    float y = pos.y / camHeight;
    if(flip) {
        x = 1-x;
        y = 1-y;
    }
    m.addFloatArg(x);
    m.addFloatArg(y);
    m.addIntArg(player);
    sender.sendMessage(m);
    ofLogNotice(
                "ID:" + ofToString(player) + "   X:"
                + ofToString(x) + "   Y:"
                + ofToString(y));
    timeSinceLastSend = ofGetElapsedTimef();
}


//--------------------------------------------------------------
void testApp::exit() {
    kinect.close();
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
        case OF_KEY_LEFT:
            gui.toggleDraw();
    }
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
    
    selectedCorner = -1;
    if(selectedCorner == -1 && idSet){
        idSet = false;
        colorJustAcquired = warpedColImg.getPixelsRef().getColor(x, y);
        ofLogNotice(ofToString(x) + "  " + ofToString(y));
        players[id].ballColor = colorJustAcquired;
        colorContourFinder.setTargetColor(players[id].ballColor,ofxCv::TRACK_COLOR_HSV);
    }
}
