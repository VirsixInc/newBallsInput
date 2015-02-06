#ifndef _AUTO_CONFIGURATOR_H_
#define _AUTO_CONFIGURATOR_H_

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "corners.h"

const unsigned int FRONT_END_DELAY_TIME = 20;
const unsigned int RECONFIGURE_TIME = 3600;

class AutoConfigurator {
public:
    AutoConfigurator() {
        delayTimer = 0;
    }
    ~AutoConfigurator() { }
    
    void init(/*ofPoint p_corners[4],*/ int p_camWidth, int p_camHeight) {
        //corners = p_corners;
        camWidth = p_camWidth;
        camHeight = p_camHeight;
        configured = false;
        threshold = 100;
    }
    
    //void reconfigure() {
    //    delayTimer = 0;
    //    configured = false;
    //}
    
    void configure(ofxCvColorImage image) {
        delayTimer++;
        
        if (delayTimer > FRONT_END_DELAY_TIME) {
            
//            corners[0] = ofPoint(0, 0);
//            corners[1] = ofPoint(camWidth, 0);
//            corners[2] = ofPoint(0, camHeight);
//            corners[3] = ofPoint(camWidth, camHeight);
            
            contourFinder.setTargetColor(ofColor::white);
            contourFinder.setMinArea(600); // TODO tweak. Seems good tho.
            contourFinder.setThreshold(threshold); // TODO tweak. Seems good tho.
            contourFinder.resetMaxArea();
            
            contourFinder.findContours(image);
            
            //TODO cant just give up like this.
            if (contourFinder.size() < 1) {
                ofLogNotice("Less than 1 contour(s) found");
                threshold++;
                if(threshold > 120)
                    threshold = 90;
                return;
            }
            if (contourFinder.size() > 1) {
                ofLogNotice("More than 1 contour(s) found");
                threshold--;
                if(threshold < 90)
                    threshold = 110;
                return;
            }
            
            const std::vector<ofPoint> contPts = contourFinder.getPolyline(0).getVertices();
            get_corners(contPts, &contCorners);
            
            corners[0] = contCorners.tl;
            corners[1] = contCorners.tr;
            corners[2] = contCorners.br;
            corners[3] = contCorners.bl;
            
            configured = true;
            delayTimer = 0;
        }
    }
    
    bool updateTimer() {
        configTimer++;
        
        if(configTimer > RECONFIGURE_TIME) {
            reconfigure();
            return true;
        }
        return false;
    }
    
    void resetTimer() {
        configTimer = 0;
    }
    
    bool isConfigured() {
        return configured;
    }
    
    void reconfigure() {
        delayTimer = 0;
        configTimer = 0;
        configured = false;
    }
    
    void getCorners(ofPoint* dest) {
        dest[0] = corners[0];
        dest[1] = corners[1];
        dest[2] = corners[2];
        dest[3] = corners[3];
    }
    
    void draw() {
        contourFinder.draw();
    }
    
private:
    Corners contCorners;
    
    bool configured;
    ofPoint corners[4];
    
    unsigned int delayTimer, configTimer;
    
    int camWidth, camHeight;
    
    ofxCv::ContourFinder contourFinder;
    
    int threshold;
};

#endif