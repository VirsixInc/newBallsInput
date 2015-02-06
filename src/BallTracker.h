/*
 Utilizes OfxCv::ContourFinder and its RectTracker to track shapes across a grayscaled image.
 
 Run init(...) and pass it pointers to variables (meant for OfxGui sliders) in setup.
 
 Call track(...) in your update loop and pass it a grayscale image. It will return a vector or ofRectangles.
 
 Call Draw() if you want to see a visualization of the contour and velocity.
 
 set LOGINFO to 0 if you dont want logs
 
 */

#ifndef _Ball_Tracker_H_
#define _Ball_Tracker_H_

#define LOGINFO 0

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinect.h"

class BallTracker {
public:
    
    BallTracker() { }
    ~BallTracker() { }
    
    void init(int* p_persistence, float* p_maxDistance, int* p_minContArea, int* p_maxContArea, float*p_velSmoothRate) {
        persistence = p_persistence;
        maxDistance = p_maxDistance;
        minContArea = p_minContArea;
        maxContArea = p_maxContArea;
        velSmoothRate = p_velSmoothRate;
        
        contourFinder.setUseTargetColor(false);
        
        sideBalls = ballCounter = configCounter = 0;
    }
    
    void track(ofxCvGrayscaleImage gray, vector<ofRectangle>* rects, vector<unsigned int>* labels, vector<ofVec2f>* velocities) {
        ofxCv::RectTracker& tracker = contourFinder.getTracker();
        
        if(persistence == NULL || maxDistance == NULL || minContArea == NULL || maxContArea == NULL) {
            ofLogNotice("Must call init!");
            return;
        }
        
        tracker.setPersistence(*persistence);
        tracker.setMaximumDistance(*maxDistance);
        contourFinder.setFindHoles(true);
        contourFinder.setMinArea(*minContArea);
        contourFinder.setMaxArea(*maxContArea);
        
        gray.blur(1);
        
        contourFinder.findContours(gray);
        
        updateVelocity();
        updateColorTracked();
        
#if LOGINFO
        logInfo();
#endif
        
        for(int i = 0; i < contourFinder.size(); i++) {
            int label = contourFinder.getLabel(i);
            cv::Rect_<int> rect = tracker.getCurrent(label);
            rects->push_back(ofxCv::toOf(rect));
            labels->push_back(label);
            velocities->push_back(velocityMap[label]);
        }
        
        checkConfigIntegrity();
    }
    
    bool depthTracked(unsigned int label) {
        vector<unsigned int> newLabels = contourFinder.getTracker().getNewLabels();
        for(int i = 0; i < newLabels.size(); i++) {
            if(label == newLabels[i]) {
                return false;
            }
        }
        return true;
    }
    
    bool colorTracked(unsigned int label) {
        if(colorCheckedMap.count(label) > 0) {
            if(colorCheckedMap[label] == false) {
                return false;
            }
            return true;
        } else {
            ofLogNotice("Bad usage? (colorTracked func)");
            return true;
        }
    }
    
    void setColorTracked(unsigned int label) {
        if(colorCheckedMap.count(label) > 0) {
            colorCheckedMap[label] = true;
        } else {
            ofLogNotice("Bad usage? (trackColor func)");
        }
    }
    
    bool badConfig() {
        ofLogNotice("total balls: " + ofToString(ballCounter));
        ofLogNotice("sideBalls: " + ofToString(sideBalls));
        if(ballCounter > 10 || sideBalls > 5) { // TODO tweak me
            return true;
        }
        return false;
    }
    
    void draw() {
        ofSetColor(ofColor::blue);
        contourFinder.draw();
        
        ofxCv::RectTracker& tracker = contourFinder.getTracker();
        ofSetColor(ofColor::teal);
        for(int i = 0; i < contourFinder.size(); i++) {
            int label = contourFinder.getLabel(i);
            ofVec2f pos = ofxCv::toOf(tracker.getCurrent(label)).getCenter();
            ofVec2f vel = velocityMap[label];
            ofLine(pos, pos + (vel * 10));
        }
    }

private:
    void checkConfigIntegrity() {
        configCounter++;
        if(configCounter > 60) { // Sample every 20 frames for bad stuff
            ballCounter = sideBalls = 0;
        }
        
        if(contourFinder.size() > 0)
            ballCounter += contourFinder.size();
        
        ofxCv::RectTracker& tracker = contourFinder.getTracker();
        for(int i = 0; i < contourFinder.size(); i++) {
            int label = contourFinder.getLabel(i);
            ofVec2f pos = ofxCv::toOf(tracker.getCurrent(label)).getCenter();
            if(pos.x < 20 || pos.x > 320 - 20) { //FIXME hardcoding camWidth
                sideBalls++;
            }
        }
    }
    
    void updateVelocity() {
        for(int i = 0; i < contourFinder.size(); i++) {
            ofVec2f cur = ofxCv::toOf(contourFinder.getVelocity(i));
            unsigned int label = contourFinder.getLabel(i);
            
            if(velocityMap.count(label) > 0) {
                ofVec2f& smooth = velocityMap[label];
                smooth.x = ofLerp(smooth.x, cur.x, *velSmoothRate);
                smooth.y = ofLerp(smooth.y, cur.y, *velSmoothRate);
            } else {
                velocityMap[label] = cur;
            }
        }
        
        vector<unsigned int> deadLabels = contourFinder.getTracker().getDeadLabels();
        for(int i = 0 ; i < deadLabels.size(); i++) {
            velocityMap.erase(deadLabels[i]);
        }
    }
    
    void updateColorTracked() {
        for(int i = 0; i < contourFinder.size(); i++) {
            unsigned int label = contourFinder.getLabel(i);
            
            if(colorCheckedMap.count(label) > 0) {
               // Nothing (I think)
            } else {
                colorCheckedMap[label] = false;
            }
        }
        
        vector<unsigned int> deadLabels = contourFinder.getTracker().getDeadLabels();
        for(int i = 0 ; i < deadLabels.size(); i++) {
            colorCheckedMap.erase(deadLabels[i]);
        }
    }
    
    void logInfo() {
        for(int i = 0; i < contourFinder.size(); i++) {
            ofxCv::RectTracker& tracker = contourFinder.getTracker();

            int label = contourFinder.getLabel(i);
            int age = contourFinder.getTracker().getAge(label);
            const cv::Rect& current = tracker.getSmoothed(label);
            
            ofVec2f curPos = ofxCv::toOf(current).getCenter();
            
            string notice = "curLabels: " + ofToString(tracker.getCurrentLabels().size())
            + " newLabels: " + ofToString(tracker.getNewLabels().size())
            + "|prevLabels: " + ofToString(tracker.getPreviousLabels().size())
            + "|deadLabels: " + ofToString(tracker.getDeadLabels().size())
            + "|label: " + ofToString(label)
            + "|age: " + ofToString(age)
            + "|pos: " + ofToString(curPos)
            + "|vel: " + ofToString(velocityMap[label]);
            
            ofLogNotice(ofToString(notice));
        }
    }
    
    int* persistence;
    float* maxDistance;
    int* minContArea;
    int* maxContArea;
    float* velSmoothRate;
    
    unsigned int ballCounter, sideBalls, configCounter;
    
    std::map<unsigned int, ofVec2f> velocityMap;
    std::map<unsigned int, bool> colorCheckedMap;
    
    ofxCv::ContourFinder contourFinder;
};

#endif
