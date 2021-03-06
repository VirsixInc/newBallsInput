#ifndef _AUTO_CONFIGURATOR_H_
#define _AUTO_CONFIGURATOR_H_

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "corners.h"

const unsigned int FRONT_END_DELAY_TIME = 20;
//const unsigned int RECONFIGURE_TIME = 3600;

class AutoConfigurator {
public:
    AutoConfigurator() {
        delayTimer = 0;
//        configDone = false;
    }
    ~AutoConfigurator() { }
    
    void init(int p_camWidth, int p_camHeight, int* p_threshold) {
        camWidth = p_camWidth;
        camHeight = p_camHeight;
        configured = false;
        threshold = p_threshold;
    }
    
    void configure(ofxCvColorImage image) {
        delayTimer++;
        
        if (delayTimer > FRONT_END_DELAY_TIME) {
            contourFinder.setTargetColor(ofColor::white);
            contourFinder.setMinArea(2000); // TODO tweak. Seems good tho.
            contourFinder.setThreshold(*threshold);
            contourFinder.resetMaxArea();
            
            contourFinder.findContours(image);
            
            //TODO cant just give up like this.
            if (contourFinder.size() < 1) {
                ofLogNotice("Less than 1 contour(s) found");
//                threshold++;
//                if(threshold > 120)
//                    threshold = 90;
                return;
            }
            if (contourFinder.size() > 1) {
                ofLogNotice("More than 1 contour(s) found");
//                threshold--;
//                if(threshold < 90)
//                    threshold = 110;
                return;
            }
            
            const std::vector<ofPoint> contPts = contourFinder.getPolyline(0).getVertices();
            get_corners(contPts, &contCorners);
            
            corners[0] = contCorners.tl;
            corners[1] = contCorners.tr;
            corners[2] = contCorners.br;
            corners[3] = contCorners.bl;
            
//            if(configDone == true) {
//                configDone = false;
            configured = true;
            delayTimer = 0;
//            }
        }
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
    
//    bool configDone;
    
private:
    Corners contCorners;
    
    bool configured;
    ofPoint corners[4];
    
    unsigned int delayTimer, configTimer;
    
    int camWidth, camHeight;
    
    ofxCv::ContourFinder contourFinder;
    
    int* threshold;
};

#endif