#include "testApp.h"

const int depthImageAverageTime = 20;

//--------------------------------------------------------------
void testApp::setup() {
    //    angle = 0;
    amtOfPlayers = 4;
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
    grayImageDiff.allocate(camWidth, camHeight);
    meanGrayImage.allocate(camWidth, camHeight);
    
    edge.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    warpedColImg.allocate(camWidth, camHeight);
    colImgNoCont.allocate(camWidth, camHeight);
    
    ofSetFrameRate(60);
    
    src[0] = ofPoint(0,0);
    src[1] = ofPoint(camWidth,0);
    src[2] = ofPoint(0,camHeight);
    src[3] = ofPoint(camWidth,camHeight);
    
    dest[0] = ofPoint(0,0);
    dest[1] = ofPoint(camWidth,0);
    dest[2] = ofPoint(0,camHeight);
    dest[3] = ofPoint(camWidth,camHeight);
    
    state = ConfigBackground;
    timer = 0;
    whiteScreen = false;
    
    players[0].ballColor = ofColor(255,255,0);
    players[1].ballColor = ofColor(255,0,255);
    players[2].ballColor = ofColor(0,255,255);
    players[3].ballColor = ofColor(0,255,0);
    
    gui.addSlider("First Col Thresh", targetColThresh, 0, 255);
    gui.addSlider("Depth Thresh", depthThresh, 0, 255);
    gui.addSlider("Range", range, 0, 20);
    gui.addSlider("minContArea", minContArea, 0, 20);
    gui.addSlider("maxContArea", maxContArea, 0, 2000);
    gui.addToggle("Configured", configured);
    gui.addToggle("Color Configured", colorConfig);
    gui.addToggle("Disable Timer", timerEngaged);
    gui.addToggle("Save Background", saveBk);
    gui.addToggle("Flip", flip);
    gui.loadFromXML();
    
}

//--------------------------------------------------------------
void testApp::update() {
    
    CheckOSCMessage();
    
    kinect.update();
    if(kinect.isFrameNew()) {
        colorContourFinder.setThreshold(targetColThresh);
        colorContourFinder.setMinAreaRadius(minContArea);
        colorContourFinder.setMaxAreaRadius(maxContArea);
        
        UpdateImages();
        
        switch(state) {
            case Config:
                break;
                
            case ConfigScreen:
                ConfigureScreen();
                break;
                
            case ConfigBackground:
            { //Scope-compiler fix
                unsigned char* oldPixels = meanGrayImage.getPixels();
                unsigned char* newPixels = grayImage.getPixels();
                for (int i = 0; i < grayImage.getWidth() * grayImage.getHeight(); i++) {
                    oldPixels[i] = max(oldPixels[i], newPixels[i]);
                }
                meanGrayImage = oldPixels;
            }
                
                timer++;
                if(timer > depthImageAverageTime) {
                    grayImageDiff = meanGrayImage;
                    SendMessage("/config/done");
                    state = ConfigColors;
                    timer = 0;
                }
                ThresholdImages();
                break;
                
            case ConfigColors:
                SendMessage("/config/done");
                state = Main;
                break;
                
            case Main:
                if(configured) {
                    grayImage.absDiff(grayImageDiff);
                    ThresholdImages();
                    
                    if (saveBk)
                        SaveBackground();
                    
                    //                    grayResizeImg.setFromPixels(kinect.getDepthPixelsRef());
                    //                    manipImage.scaleIntoMe(grayResizeImg);
                    //                    grayImage.warpIntoMe(manipImage, dest, src);
                    
                    //                    colorResizeImg.setFromPixels(kinect.getPixelsRef());
                    //                    colImg.scaleIntoMe(colorResizeImg);
                    //                    warpedColImg.warpIntoMe(colImg, dest, src);
                    //                    colManipImg.setFromPixels(warpedColImg.getPixelsRef());
                    //                    if(saveBk){
                    //                        saveBk = false;
                    //                        grayImageDiff.setFromPixels(grayImage.getPixels(), camWidth, camHeight);
                    //                        ofImage imageToSave;
                    //                        imageToSave.setFromPixels(grayImageDiff.getPixelsRef());
                    //                        imageToSave.saveImage("backgroundToDiff.png");
                    //                    }
                    
                    //                    grayImage.absDiff(grayImageDiff);
                    //                    for(int i=0; i<grayImage.getWidth()*grayImage.getHeight();i++){
                    //                        int tempPixel = grayImage.getPixels()[i];
                    //                        int xPos = i%camWidth;
                    //                        int yPos = i/camWidth;
                    //                        int index = colManipImg.getPixelsRef().getPixelIndex(xPos, yPos);
                    //                        if(tempPixel == 0){
                    //                            colManipImg.setColor(index, ofColor::black);
                    //                            continue;
                    //                        }
                    //                        if(!(ofInRange(tempPixel, depthThresh, depthThresh+range))){
                    //                            colManipImg.setColor(index, ofColor::black);
                    //                            tempPixel = 0;
                    //                        }else{
                    //                            tempPixel = 255;
                    //                        }
                    //                        grayImage.getPixels()[i] = tempPixel;
                    //                    }
                    //                    warpedColImg.setFromPixels(colManipImg.getPixelsRef());
                    
                    contours.findContours(grayImage, minContArea, maxContArea, 8, true);
                    
                    if(contours.nBlobs > 0) {
                        if(timerEngaged && timeSinceLastSend + 1.0 < ofGetElapsedTimef()) {
                            SendMessage("/checkColor");
                            timeSinceLastSend = ofGetElapsedTimef();
                        }
                    }
                    
                    if(whiteScreen) {
                        colImgNoCont.setFromPixels(warpedColImg.getPixelsRef());
                        ofxCvBlob blob = contours.blobs.at(0);
                        checkForColor(colImgNoCont, blob.centroid.x, blob.centroid.y);
                    }
                    
                    //                    if(receiver.hasWaitingMessages()){
                    //                        ofxOscMessage m;
                    //                        receiver.getNextMessage(&m);
                    //                        ofLogNotice("GOT A MESSAGE");
                    //                        colImgNoCont.setFromPixels(warpedColImg.getPixelsRef());
                    //                        ofxCvBlob blob = contours.blobs.at(0);
                    //                        checkForColor(colImgNoCont, blob.centroid.x, blob.centroid.y);
                    //                    }
                    //                    if(configured && (timerEngaged && timeSinceLastSend + 1.0 < ofGetElapsedTimef())){
                    //                        for(int j = 0;j<contours.nBlobs;j++){
                    //                            ofxCvBlob blob = contours.blobs.at(j);
                    //                            ofxOscMessage m;
                    //                            m.setAddress("/checkColor");
                    //                            m.addFloatArg(1.0);
                    //                            sender.sendMessage(m);
                    //                            timeSinceLastSend = ofGetElapsedTimef();
                    //                        }
                    //                    }
                }
                break;
        }
        
    }
    
}

//--------------------------------------------------------------
void testApp::UpdateImages() {
    colImg.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
    warpedColImg.warpIntoMe(colImg, src, dest);
    
    //    depthImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
    ofxCvGrayscaleImage temp_depth;
    temp_depth.allocate(kinect.width, kinect.height);
    temp_depth.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
    
    grayImage.warpIntoMe(temp_depth, src, dest); //Temp stuff. Gotta clean it up
    //    grayImage.blur(3);
}

//--------------------------------------------------------------
void testApp::ThresholdImages() {
    //depthImage.absDiff(depthImageDiff);
    unsigned char* grayPixels = grayImage.getPixels();
    for (int i = 0, n = grayImage.getWidth() * grayImage.getHeight(); i < n; i++) {
        if(ofInRange(grayPixels[i], depthThresh, depthThresh+range)){
            grayPixels[i] = 255;
        }else{
            grayPixels[i] = 0;
        }
    }
}

//--------------------------------------------------------------
void testApp::ConfigureScreen() {
    timer++;
    colorContourFinder.setTargetColor(ofColor::white);
    colorContourFinder.setMinArea(800); // TODO tweak. Seems good tho.
    colorContourFinder.setThreshold(120); // TODO tweak. Seems good tho.
    colorContourFinder.resetMaxArea();
    colorContourFinder.findContours(colImg);
    
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
        vector<cv::Vec4i> lines;
        
        cv::Rect rect = colorContourFinder.getBoundingRect(0);
        
        ofPoint box[4];
        box[0] = ofPoint(rect.x - 4, rect.y - 4);
        box[1] = ofPoint(rect.x + rect.width + 4, rect.y - 4);
        box[2] = ofPoint(rect.x + rect.width + 4, rect.y + rect.height + 4);
        box[3] = ofPoint(rect.x - 4, rect.y + rect.height + 4);
        
        
        ofImage cropped;
        cropped.setFromPixels(colImg.getPixelsRef());
        cropped.crop(rect.x - 4, rect.y - 4, rect.width + 8, rect.height + 8);
        cropped.update();
        
        ofPixels gray;
        ofxCv::convertColor(cropped, gray, CV_RGB2GRAY);
        
        ofxCv::blur(gray, 5);
        
        ofxCv::Canny(gray, edge, 100, 100);
        edge.update();
        
        cv::HoughLinesP(ofxCv::toCv(edge), lines, 1, CV_PI/180, 70, 150, 10); //TODO fiiiiix
        ofLogNotice("Lines: " + ofToString(lines.size()));
        
        std::vector<cv::Point2f> corners;
        for (int i = 0; i < lines.size(); i++) {
            for (int j = i+1; j < lines.size(); j++) {
                cv::Point2f pt = ComputeIntersect(lines[i], lines[j]);
                if (pt.x >= 0 && pt.y >= 0)
                    corners.push_back(pt);
            }
        }
        
        std::vector<cv::Point2f> approx;
        
        if(corners.size() > 0) {
            cv::approxPolyDP(cv::Mat(corners), approx, cv::arcLength(cv::Mat(corners), true) * 0.02, true);
        }
        
        if (approx.size() != 4) {
            std::cout << "Hough Lines didnt work " + ofToString(approx.size()) << std::endl;
            dest[0] = ofPoint(rect.x, rect.y);
            dest[1] = ofPoint(rect.x + rect.width, rect.y);
            dest[2] = ofPoint(rect.x + rect.width, rect.y + rect.height);
            dest[3] = ofPoint(rect.x, rect.y + rect.height);
            
            SendMessage("/config/cornerParsed");
            state = ConfigBackground;
        } else {
            // Get mass center
            cv::Point2f center(0,0);
            for (int i = 0; i < corners.size(); i++)
                center += corners[i];
            
            center *= (1. / corners.size());
            SortCorners(corners, center);
            
            corners[0].x += box[0].x + 4; //TODO make sure this works
            corners[0].y += box[0].y + 4;
            
            corners[1].x += box[1].x - 4;
            corners[1].y += box[1].y + 4;
            
            corners[2].x += box[1].x + 4;
            corners[2].y += box[1].y - 4;
            
            corners[3].x += box[1].x - 4;
            corners[3].y += box[1].y - 4;
            
            for(int i = 0; i < 4; i++)
                dest[i] = ofxCv::toOf(corners[i]);
            
            SendMessage("/config/cornerParsed");
            state = ConfigBackground;
            
            timer = 0;
        }
        
        // Get depth
        depthThresh = 0;
        
        // Index *should* be right. Hopefully the dest corners arent parsed out by threshold image
        for(int i = 0; i < 4; i++) {
            int index = dest[i].x * grayImage.width + dest[i].y;
            depthThresh += grayImage.getPixels()[index];
        }
        
        depthThresh /= 4;
        depthThresh += 15; // A little buffer to account for the screen waving
    }
}

void testApp::checkForColor(ofxCvColorImage imageInQuestion, float x, float y){
    for(int i = 0;i<4;i++){
        colorContourFinder.setTargetColor(players[i].ballColor,ofxCv::TRACK_COLOR_HSV);
        colorContourFinder.findContours(imageInQuestion);
        if(colorContourFinder.size() > 0){
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
        ofCircle(camWidth+50*i, camHeight+100, 30);
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
            } else if(addr == "/startGame") {
                state = Main;
            } else if(addr == "/readyCheck") {
                whiteScreen = true;
            }
        }
    }
}

cv::Point2f testApp::ComputeIntersect(cv::Vec4i a, cv::Vec4i b) {
    int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3];
    int x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];
    
    if (float d = ((float)(x1-x2) * (y3-y4)) - ((y1-y2) * (x3-x4))) {
        cv::Point2f pt;
        pt.x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;
        pt.y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d;
        return pt;
    }
    else
        return cv::Point2f(-1, -1);
}

void testApp::SortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center) {
    std::vector<cv::Point2f> top, bot;
    
    for (int i = 0; i < corners.size(); i++) {
        if (corners[i].y < center.y)
            top.push_back(corners[i]);
        else
            bot.push_back(corners[i]);
    }
    
    cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
    cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
    cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
    cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];
    
    corners.clear();
    corners.push_back(tl);
    corners.push_back(tr);
    corners.push_back(bl);
    corners.push_back(br);
}

//--------------------------------------------------------------
void testApp::SendMessage(string message) {
    ofxOscMessage m;
    m.setAddress(message);
    sender.sendMessage(m);
}

//--------------------------------------------------------------
void testApp::SendHitMessage(ofPoint pos, int player) {
    if (player == -1)
        return;
    
    ofxOscMessage m;
    m.setAddress("/shoot");
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
    
    //    sentMessage message;
    //    message.pos = pos;
    //    message.timeToRemove = timeSinceLastSend + 0.5f;
    //    sentMessages.push_back(message);
}

//--------------------------------------------------------------
void testApp::SaveBackground() {
    saveBk = false;
    unsigned char* pixels = grayImageDiff.getPixels();
    for(int i = 0; i < camWidth * camHeight; i++) {
        pixels[i] = 0;
    }
    grayImageDiff = pixels;
    timer = 0;
    state = ConfigBackground;
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
            //        case '[':
            //            angle++;
            //            //if(angle>30) angle=30;
            //            break;
            //
            //        case ']':
            //            angle--;
            //            //if(angle<-30) angle=-30;
            //            break;
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
