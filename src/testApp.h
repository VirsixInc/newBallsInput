#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"
#include "ofxXmlSettings.h"
#include "ofxMSAInteractiveObject.h"
#include "ofxSimpleGuiToo.h"
#include "corners.h"
#include "BallTracker.h"

struct ballPlayer{
//    int vote;
    ofColor ballColor;
    bool ballFound;
//    int voteId;
};

class testApp : public ofBaseApp {
    
public:
    enum State {
        ConfigBackground, Main, ConfigScreen, Config
    };
    void setup();
    void update();
    void draw();
    void exit();
    
    void keyPressed(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    //	void mouseReleased(int x, int y, int button);
    //	void windowResized(int w, int h);
    
private:
    void UpdateImages();
    void ConfigureScreen();
    void ThresholdImages();
    void checkForColor(ofxCvColorImage imageInQuestion, ofPoint ptToFire);
    void resetPoints();
    void savePoints();
    
    void SaveBackground();
    
    void CheckOSCMessage();
    
    void SendMessage(string message);
    void SendHitMessage(string message, ofPoint pos, int player);
    
//    cv::Point2f ComputeIntersect(cv::Vec4i a, cv::Vec4i b);
//    void SortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center);

    Corners contCorners;
    
    State state;
    float threshold;
    
    ofxCv::ContourFinder colorContourFinder, partEffectFinder;
    ofxCv::TrackingColorMode trackingColorMode;
    
    ofxOscSender sender;
    ofxOscReceiver receiver;
    
    ofxKinect kinect;
    
    ofxCvGrayscaleImage grayImageDiff, grayImage, grayForColor, temp_depth, temp_scale;
    ofPoint hitPoint;
    //ofxCvGrayscaleImage firstThresh, secondThresh;//meanGrayImage;
    
    ofxCvColorImage colImgNoCont, colImg, warpedColImg, temp_color, imgToCheck;
    
//    ofImage edge; // TEMP
    
    ofxCvContourFinder contours, colorContours;
    
    ofColor colorJustAcquired, colorJustAcquired2;
    
    ofxSimpleGuiToo gui;
    
    // BallTracker variables
    BallTracker ballTracker;
    vector<ofRectangle> rects;
    vector<unsigned int> labels;
    vector<ofVec2f> velocities;
    
    float minVariationDistance;
    int lifeTime;
    float velSmoothRate;
    int minContArea, maxContArea;
    //----------------------
    
//    typedef std::pair<ofRectangle, unsigned int> rectSeenPair;
    std::map<unsigned int, ofRectangle> labelMap;
    
    bool passedColor, idSet, saveBk, resetPts, timerEngaged, configured, savePts, colorConfig;
    
    ballPlayer players[4];
    long amtOfWhitePixels, hue, sat, bri;
    int colorToConfig[3];
    int targetColThresh,hueRange, satRange, briRange;
    int id, configId, round;
    int camWidth, camHeight, kinWidth, kinHeight;
    int range, objThresh, depthThresh, secondColThresh;
    int minPartEffect, maxPartEffect, partThresh;
    int port;
    int amtOfPlayers, votesReq;
    float timeSinceLastSend, timeSinceLastWhiteFound, lastTime, prevRow;
    
    
    
    bool whiteScreen;
    bool flip;
    
    unsigned int timer;
    
    ofPoint src[4];
    ofPoint dest[4];
    
    int selectedCorner;
};
