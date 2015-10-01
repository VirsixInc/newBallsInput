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
#include "AutoConfigurator.h"

struct ballPlayer{
//    int vote;
    ofColor ballColor;
    bool ballFound;
//    int voteId;
};

class testApp : public ofBaseApp {
    
public:
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
    void resetPoints();
    void savePoints();
    
    int checkForColor(ofxCvColorImage imageInQuestion, ofPoint ptToFire);
    
    void SaveBackground();
    
    void CheckOSCMessage();
    
    void SendMessage(string message);
    void SendHitMessage(string message, ofPoint pos, int player);
    
//    cv::Point2f ComputeIntersect(cv::Vec4i a, cv::Vec4i b);
//    void SortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center);

    Corners contCorners;
    
//    State state;
//    float threshold;
    
//    ofxCv::ContourFinder partEffectFinder;
    
    ofxOscSender sender;
    ofxOscReceiver receiver;
    
    ofxKinect kinect;
    
    ofxCvGrayscaleImage grayImageDiff, grayImage, grayForColor, temp_depth, temp_scale;
    
    ofxCvColorImage colImgNoCont, colImg, warpedColImg, temp_color, imgToCheck;
    
    ofxSimpleGuiToo gui;
    
    // BallTracker variables
    BallTracker ballTracker;
    vector<ofRectangle> rects;
    vector<unsigned int> labels;
    vector<ofVec2f> velocities;
 
	int colorSamples;   
    float minVariationDistance;
    int lifeTime;
    float velSmoothRate;
    int minContArea, maxContArea;
    //----------------------
    
    //AutoConfigurator stuff
    AutoConfigurator autoConfigurator;
    //----------------------
    
    bool saveBk, resetPts, configured;
    
    static const int amtOfPlayers = 2;
    ballPlayer players[amtOfPlayers];

    int camWidth, camHeight;
    int range, depthThresh;
//    int minPartEffect, maxPartEffect, partThreshLevel;
    int configThreshold;
//    int partThreshLevels[6];
    
    int kinectTimeout;
    
    bool flip;
 	
	bool needsConfiguring;   
    ofPoint src[4];
    ofPoint dest[4];
};
