#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"
#include "ofxXmlSettings.h"
#include "ofxMSAInteractiveObject.h"
#include "ofxSimpleGuiToo.h"

struct ballPlayer{
  int vote;
  ofColor ballColor;
  bool ballFound;
  int voteId;
};
class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
  void checkForColor(ofxCvColorImage imageInQuestion, float x, float y);
  void resetPoints();
  void savePoints();
	
  ofxCv::ContourFinder colorContourFinder;
  float threshold;
  ofxCv::TrackingColorMode trackingColorMode;

  ofxOscSender sender;
  ofxOscReceiver receiver;

	ofxKinect kinect;
	ofxCvFloatImage floatImg;
	ofxCvGrayscaleImage manipImage, grayImageDiff, grayImage, grayResizeImg; // grayscale depth image
  ofxCvGrayscaleImage hueImg, satImg, briImg;
  ofxCvColorImage colImgNoCont, colImg, warpedColImg, colorResizeImg;
  ofxCvGrayscaleImage r0, g0, b0, r1, g1, b1;
  ofImage colManipImg;
	ofxCvContourFinder contours, colorContours;

  ofColor colorJustAcquired, colorJustAcquired2;

  ofxSimpleGuiToo gui;

  bool passedColor, idSet, saveBk, resetPts, timerEngaged, configured, savePts, colorConfig;

  ballPlayer players[4];
  int voteCount;
  long amtOfWhitePixels, hue, sat, bri;
  int colorToConfig[3];
  int targetColThresh,hueRange, satRange, briRange;
  int id, configId, round;
  int camWidth, camHeight, kinWidth, kinHeight;
  int range, objThresh, depthThresh, secondColThresh;
  int minContArea, maxContArea;
  int angle, port; 
  int amtOfPlayers;
  float timeSinceLastSend, lastTime, prevRow;

  ofPoint src[4];
  ofPoint dest[4];

  int selectedCorner;
};
