#include "testApp.h"

const int depthImageAverageTime = 20;

//--------------------------------------------------------------
void testApp::setup() {
    sender.setup("localhost", 9999);
    receiver.setup(7600);
    
    kinect.setRegistration(true); // Syncs up depth and RGB
    kinect.init();
    kinect.open();                // Opens first available kinect
    
    camWidth = 320;
    camHeight = 240;
    
    colImg.allocate(camWidth, camHeight);
    grayImage.allocate(camWidth, camHeight);
    grayForColor.allocate(camWidth, camHeight);
    grayImageDiff.allocate(camWidth, camHeight);
    temp_depth.allocate(kinect.width, kinect.height);
    temp_scale.allocate(camWidth, camHeight);
    imgToCheck.allocate(camWidth,camHeight);
    
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
    
    
    players[0].ballColor = ofColor(255,0,0);
    players[1].ballColor = ofColor(0,255,0);

//    partThreshLevels[0] = 90;
//    partThreshLevels[1] = 100;
//    partThreshLevels[2] = 110;
//    partThreshLevels[3] = 115;
//    partThreshLevels[4] = 120;
//    partThreshLevels[5] = 130;
    
#if __linux__
    ofLogNotice("Linux OS. Moving _settings.xml");

    ofFile f;
    if(false == f.doesFileExist("/home/dodgeball/_settings.xml", false)) {
        f.open("_settings.xml");
        f.copyTo("/home/dodgeball/_settings.xml", false);
    }
    
    gui.setup();
    vector<ofxSimpleGuiPage *> pages = gui.getPages();
    for(int i = 0; i < pages.size(); i++) {
        gui.page(i).setXMLName("/home/dodgeball/_settings.xml");
    }
    
#endif
//    gui.setAlignRight(true);
    gui.addSlider("configThreshold", configThreshold, 0, 255);
    gui.addSlider("Depth Thresh", depthThresh, 0, 255);
    gui.addSlider("Range", range, 0, 80);
//    gui.addSlider("minContArea", minContArea, 0, 1000);
//    gui.addSlider("maxContArea", maxContArea, 0, 2000);
//    gui.addSlider("minPartEffect", minPartEffect, 0, 10000);
//    gui.addSlider("maxPartEffect", maxPartEffect, 0, 20000);
//    gui.addSlider("minVaryDist", minVariationDistance, 0.01, 1000.0);
//    gui.addSlider("velSmoothRate", velSmoothRate, 0.0, 1.0);
//    gui.addSlider("lifeTime", lifeTime, 0, 150);
    gui.addSlider("colorSamples", colorSamples, 0, 4);
//    gui.addToggle("Configured", configured);
    gui.addToggle("Save Background", saveBk);
    gui.addToggle("Flip", flip);
    gui.setAlignRight(true);
//    gui.config->fullColor = 
    gui.loadFromXML();

    velSmoothRate = 0.775f; //Hardcoding for production version. Values still changeable
    minContArea = 260;
    maxContArea = 1260;
    minVariationDistance = 65.0f;
    lifeTime = 19;
    configured = true;

    ballTracker.init(&lifeTime, &minVariationDistance, &minContArea, &maxContArea, &velSmoothRate, &colorSamples);

//    partEffectFinder.setTargetColor(ofColor::white, ofxCv::TRACK_COLOR_RGB);
    
    autoConfigurator.init(camWidth, camHeight, &configThreshold);
    
    SendMessage("/config/start");
    kinectTimeout = 0;
}

//--------------------------------------------------------------
void testApp::update() {
    CheckOSCMessage();
    
    kinect.update();
    if(kinect.isFrameNew()) {
        kinectTimeout = 0;
//        partEffectFinder.setMinArea(minPartEffect);
//        partEffectFinder.setMaxArea(maxPartEffect);
//        partEffectFinder.setThreshold(partThreshLevels[partThreshLevel]);
        
        if (!autoConfigurator.isConfigured()) {
            dest[0] = ofPoint(0,0);
            dest[1] = ofPoint(camWidth,0);
            dest[2] = ofPoint(camWidth,camHeight);
            dest[3] = ofPoint(0,camHeight);
            
            UpdateImages();
            
            autoConfigurator.configure(warpedColImg);
            
            if (autoConfigurator.isConfigured()) { // Config done
                SendMessage("/config/done");
                autoConfigurator.getCorners(dest);
                UpdateImages();
                SaveBackground();
                saveBk = true;
            }
        } else {
            UpdateImages();
            
            if (saveBk)
                SaveBackground();
            
            ThresholdImages();
            
            if(configured) {
                labels.clear();
                rects.clear();
                velocities.clear();
                grayImage.blur(1);
                ballTracker.track(grayImage, &rects, &labels, &velocities);
                
                if(labels.size() > 0) {
                    for(int i = 0; i < labels.size(); i++) {
                        if(!ballTracker.colorTracked(labels[i])) {
                            ofxCvColorImage tmpColCont;
                            tmpColCont.setFromPixels(warpedColImg.getPixelsRef());
                            tmpColCont.setROI(rects[i]);
                            tmpColCont.setFromPixels(tmpColCont.getRoiPixelsRef());
                                
                            imgToCheck.setFromPixels(tmpColCont.getRoiPixelsRef());
                            ofPoint hitPoint = rects[i].getCenter();
                            int color = checkForColor(imgToCheck, hitPoint);
                            
                            if(ballTracker.colorFound(labels[i], color)) {
                                ballTracker.setColorTracked(labels[i]);
                                SendHitMessage("/shoot", hitPoint, color);
                            }
                        }
//                        if(!ballTracker.depthTracked(labels[i])) {
//                            hitPoint = rects[i].getCenter();
//                            SendHitMessage("/checkColor", hitPoint, 0);
//                        }
                    }
                }
            }
        }
    } else {
        kinectTimeout++;
        if(kinectTimeout == 2000) {
            SendMessage("/config/noKinect");
        }
        if(kinectTimeout > 2000 && kinectTimeout % 100 == 0) {
            kinect.open();
            if(kinect.isConnected()) {
                SendMessage("/config/kinectFound");
            }
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
    grayImage.warpIntoMe(temp_scale, dest, src); // Temp stuff. Gotta clean it up
}

//--------------------------------------------------------------
void testApp::ThresholdImages() {
    colImgNoCont.setFromPixels(warpedColImg.getPixelsRef());
    
    unsigned char* grayPixels = grayImage.getPixels();
    unsigned char* diffPixels = grayImageDiff.getPixels();
    unsigned char* colorPixels = warpedColImg.getPixels();
    for (int i = 0, n = grayImage.getWidth() * grayImage.getHeight(); i < n; i++) {
        if(grayPixels[i] < diffPixels[i]) {
            grayPixels[i] = 0;
            colorPixels[i*3] = colorPixels[i*3+1] = colorPixels[i*3+2] = 0;
        }
    }
    grayImage = grayPixels;
    grayImage.absDiff(grayImageDiff);
    grayPixels = grayImage.getPixels();
    
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
int testApp::checkForColor(ofxCvColorImage imageInQuestion, ofPoint ptToFire) {
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
    if(color == 1) // Lazy. Cuz in Unity 3 is green
        color = 3;
    ofLogNotice(ofToString(color) + "=color");
    
    return color;
}
//--------------------------------------------------------------
void testApp::SaveBackground() {
    saveBk = false;
    unsigned char* pixels = grayImage.getPixels();
    grayImageDiff.setFromPixels(pixels, camWidth, camHeight);
}

//--------------------------------------------------------------
void testApp::draw() {
    ofSetColor(ofColor::white);
    
    kinect.draw(0, 0, camWidth, camHeight);
    colImgNoCont.draw(camWidth, 0, camWidth, camHeight);
    imgToCheck.draw(camWidth*2, 0);
    grayImage.draw(0, camHeight, camWidth, camHeight);
    warpedColImg.draw(camWidth, camHeight, camWidth, camHeight);
    
//    ofSetColor(ofColor::white);
//    partEffectFinder.draw();

    ofSetColor(30, 255, 30);
    autoConfigurator.draw();
    
    ofSetColor(30, 30, 255);
    ballTracker.draw();
    
//    ofSetColor(ofColor::white);
//    partEffectFinder.draw();
    
    if(!configured){
        ofSetColor(0, 0, 255);
        ofCircle(dest[0].x, dest[0].y, 3);
        ofDrawBitmapString("TL", dest[0]);
        
        ofSetColor(255, 0, 0);
        ofCircle(dest[1].x, dest[1].y, 3);
        ofDrawBitmapString("TR", dest[1]);
        
        ofSetColor(255, 255, 0);
        ofCircle(dest[2].x, dest[2].y, 3);
        ofDrawBitmapString("BR", dest[2]);
        
        ofSetColor(0, 255, 0);
        ofCircle(dest[3].x, dest[3].y, 3);
        ofDrawBitmapString("BL", dest[3]);
    }
    
    gui.draw();
    for(int i = 0;i < amtOfPlayers;i++){
        ofSetColor(255,255,255);
        ofSetColor(players[i].ballColor);
        ofCircle(camWidth+60*i, 2*camHeight+100, 30);
    }
}

//--------------------------------------------------------------
void testApp::CheckOSCMessage() {
    while(receiver.hasWaitingMessages()) {
        // get the next message
        ofxOscMessage m;
        if(receiver.getNextMessage(&m)) {
            string addr = m.getAddress();
            if(addr == "/config/start") {
                autoConfigurator.reconfigure();
            }
        }
    }
}

//--------------------------------------------------------------
void testApp::SendMessage(string message) {
    ofLogNotice("Message sent: " + message);
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
}


//--------------------------------------------------------------
void testApp::exit() {
    kinect.close();
}
//--------------------------------------------------------------
void testApp::keyPressed (int key) {
    switch(key) {
        case OF_KEY_LEFT:
            gui.toggleDraw();
            break;
//        case OF_KEY_F1:
//            autoConfigurator.configDone = true;
//            break;
    }
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}
