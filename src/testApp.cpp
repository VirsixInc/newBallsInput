#include "testApp.h"

const int depthImageAverageTime = 20;

//--------------------------------------------------------------
void testApp::setup() {
    amtOfPlayers = 4;
//    votesReq = 5;
    sender.setup("localhost", 9999);
    receiver.setup(7600);
    
    kinect.setRegistration(true); // Syncs up depth and RGB
    kinect.init();
    kinect.open();                // Opens first available kinect
    
//    lastTime = 0;
    
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
    
//    edge.allocate(camWidth, camHeight, OF_IMAGE_GRAYSCALE);
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
    
    state = Config;
    timer = 0;
//    whiteScreen = false;
    
    players[0].ballColor = ofColor(255,0,0);
    players[1].ballColor = ofColor(0,255,0);
    players[2].ballColor = ofColor(0,255,0);
    players[3].ballColor = ofColor(0,255,0);
    
#if __linux__
    ofLogNotice("Linux OS. Moving _settings.xml");
    
    
    ofFile f;
    if(false == f.doesFileExist("/home/dodgeball/_settings.xml", false)) {
        f.open("_settings.xml");
//        f.open("_settings.xml", ofFile::ReadWrite);
        f.copyTo("/home/dodgeball/_settings.xml", false);
    }
    
    gui.setup();
    vector<ofxSimpleGuiPage *> pages = gui.getPages();
    for(int i = 0; i < pages.size(); i++) {
        gui.page(i).setXMLName("/home/dodgeball/_settings.xml");
    }
    
#endif
    
    //gui.addSlider("First Col Thresh", targetColThresh, 0, 255);
	gui.addSlider("configThreshold", configThreshold, 0, 255);
    gui.addSlider("Depth Thresh", depthThresh, 0, 255);
    gui.addSlider("Range", range, 0, 80);
    gui.addSlider("minContArea", minContArea, 0, 1000);
    gui.addSlider("maxContArea", maxContArea, 0, 2000);
    gui.addSlider("minPartEffect", minPartEffect, 0, 10000);
    gui.addSlider("maxPartEffect", maxPartEffect, 0, 20000);
    gui.addSlider("partThresh", partThresh, 0, 255);
    gui.addSlider("minVariationDistance", minVariationDistance, 0.01, 1000.0);
    gui.addSlider("velSmoothRate", velSmoothRate, 0.0, 1.0);
    gui.addSlider("lifeTime", lifeTime, 0, 150);
    gui.addToggle("Configured", configured);
    gui.addToggle("Color Configured", colorConfig);
    gui.addToggle("Disable Timer", timerEngaged);
    gui.addToggle("Save Background", saveBk);
    gui.addToggle("Flip", flip);
    gui.loadFromXML();
    
    ballTracker.init(&lifeTime, &minVariationDistance, &minContArea, &maxContArea, &velSmoothRate);

    partEffectFinder.setTargetColor(ofColor::white, ofxCv::TRACK_COLOR_RGB);
}

//--------------------------------------------------------------
void testApp::update() {
  colorContourFinder.setThreshold(targetColThresh);
  colorContourFinder.setMinArea(minContArea);
  colorContourFinder.setMaxArea(maxContArea);
    partEffectFinder.setMinArea(minPartEffect);
    partEffectFinder.setMaxArea(maxPartEffect);
    partEffectFinder.setThreshold(partThresh);
    
    CheckOSCMessage();
    
    kinect.update();
    if(kinect.isFrameNew()) {
        
        UpdateImages();
        if (saveBk)
          SaveBackground();
        
        switch(state) {
            case Config:
                SendMessage("/config/start");
                state = ConfigScreen;
                timer = 0;
                break;
                
            case ConfigScreen:
                ConfigureScreen();
                break;
                
            case ConfigBackground:
              SaveBackground();
              SendMessage("/config/done");
              state = Main;

              ThresholdImages();
              break;
                
            case Main:
              if(configured) {
                ThresholdImages();
                  
                labels.clear();
                rects.clear();
                velocities.clear();
                ballTracker.track(grayImage, &rects, &labels, &velocities);

                for(int i = 0; i < labels.size(); i++) {
                  if(!ballTracker.colorTracked(labels[i])) {
                    partEffectFinder.findContours(colImgNoCont);
                    if(partEffectFinder.size() > 0){
                      hitPoint = rects[i].getCenter();
                      ofxCvColorImage tmpColCont;
                      tmpColCont.setFromPixels(warpedColImg.getPixelsRef());
                      tmpColCont.setROI(rects[i]);
                      tmpColCont.setFromPixels(tmpColCont.getRoiPixelsRef());

                      imgToCheck.setFromPixels(tmpColCont.getRoiPixelsRef());
//                      imgToCheck.convertToRange(1,200);
                      checkForColor(imgToCheck, hitPoint);
                    }
                  }
                  if(!ballTracker.depthTracked(labels[i])) {
                    hitPoint = rects[i].getCenter();
                    SendHitMessage("/checkColor", hitPoint, 0);
//                    ofLogNotice("Blob found");
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
    colorContourFinder.setMinArea(300); // TODO tweak. Seems good tho.
    colorContourFinder.setThreshold(configThreshold); // TODO tweak. Seems good tho.
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
    
    if(timer > 10) { //Delay to let front end change scenes
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
       
//        SendMessage("/config/done");
        state = ConfigBackground;
    }
}

//--------------------------------------------------------------
void testApp::checkForColor(ofxCvColorImage imageInQuestion, ofPoint ptToFire) {
    //-----------------------Average color method---------------------------
    int r, g, b; //Could *theoretically* be an issue with int maxing out depending on blob size
    r = g = b = 0;

    int samples = 0;
    for(int i = 0, n = imageInQuestion.width; i < n; i++) {
        for(int j = 0, n2 = imageInQuestion.height; j < n2; j++) {
            ofColor color = imageInQuestion.getPixelsRef().getColor(i, j);
            if(color != ofColor::black) {
                r += color.r;
                g += color.g;
                b += color.b;
                samples++;
            }
        }
    }
    
    if(samples == 0) //Stop division by zero. *Shouldn't* happen
        samples = 1;
    
    ofColor averageColor = ofColor(r/samples, g/samples, b/samples);
    
    int color = 0;
    for(int i = 1; i < 2; i++) {
        int curColorDist = sqrt(
                                pow(players[color].ballColor.r - averageColor.r, 2.0f)
                                + pow(players[color].ballColor.g - averageColor.g, 2.0f)
                                + pow(players[color].ballColor.b - averageColor.b, 2.0f)
                                );
        int otherColorDist = sqrt(
                                  pow(players[i].ballColor.r - averageColor.r, 2.0f)
                                  + pow(players[i].ballColor.g - averageColor.g, 2.0f)
                                  + pow(players[i].ballColor.b - averageColor.b, 2.0f)
                                  );
        if(otherColorDist < curColorDist)
            color = i;
        
    }
    if(color == 1)
        color = 3;
    ofLogNotice(ofToString(color) + "=color");
    SendHitMessage("/shoot", ptToFire, color);
    
    //TODO Should do something for min acceptable color difference
    //----------------------------------------------------------------------
    
//    //-----------------------Contour finder method--------------------------
//  for(int i = 0;i<amtOfPlayers;i++){
//    colorContourFinder.setTargetColor(players[i].ballColor,ofxCv::TRACK_COLOR_HSV);
//    colorContourFinder.findContours(imageInQuestion);
//    if(colorContourFinder.size() > 0){
//        ofLogNotice("PLAYER FOUND: " + ofToString(i));
//        players[i].ballFound = true;
//        SendHitMessage("/shoot", ptToFire, i);
//        break;
//    }
//  }
//    //----------------------------------------------------------------------
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
    ofSetColor(ofColor::white);
    
    kinect.draw(0, 0, camWidth, camHeight);
    colImgNoCont.draw(camWidth, 0, camWidth, camHeight);
    imgToCheck.draw(camWidth*2, 0);
    grayImage.draw(0, camHeight, camWidth, camHeight);
    warpedColImg.draw(camWidth, camHeight, camWidth, camHeight);
    
    ofSetColor(30, 255, 30);
    colorContourFinder.draw();
//    autoConfigurator.draw();
    
    ofSetColor(3, 3, 255);
    ballTracker.draw();
    
    ofSetColor(ofColor::white);
    partEffectFinder.draw();

    if(configured){
        
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
            if(addr == "/config/start") {
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
//    ofLogNotice(
//                "ID:" + ofToString(player) + "   X:"
//                + ofToString(x) + "   Y:"
//                + ofToString(y));
//    timeSinceLastSend = ofGetElapsedTimef();
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
    
//    selectedCorner = -1;
//    if(selectedCorner == -1 && idSet){
//        idSet = false;
//        colorJustAcquired = warpedColImg.getPixelsRef().getColor(x, y);
//        ofLogNotice(ofToString(x) + "  " + ofToString(y));
//        players[id].ballColor = colorJustAcquired;
//        colorContourFinder.setTargetColor(players[id].ballColor,ofxCv::TRACK_COLOR_HSV);
//    }
}
