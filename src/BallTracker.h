#ifndef BALL_TRACKER_H
#define BALL_TRACKER_H

#include <vector>
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

using namespace std;

class BallData {
public:
    BallData(ofPoint p_pos, int p_lifeTime) {
        pos = p_pos;
        lifeTime = p_lifeTime;
        colorChecked = false;
    }
    ofPoint pos;
    int lifeTime;
    bool colorChecked;
};

class BallTracker {
public:
    BallTracker() { }
    
    void Init(float p_minVariationDistance, int p_lifeTime) {
        minVariationDistance = p_minVariationDistance;
        lifeTime = p_lifeTime;
    }
    
    void Update() {
        for(int i = 0; i < balls.size(); i++) {
            balls[i].lifeTime--;
            if(balls[i].lifeTime < 0) {
                balls.erase(balls.begin()+i);
            }
        }
    }
    
    bool UseBlob(ofxCvBlob blob, bool checkColor) {
        BallData data(blob.centroid, lifeTime);

        for(int i = 0; i < balls.size(); i++) {
            if(checkColor) {
                if(balls[i].colorChecked == false) {
                    if(balls[i].pos.distance(data.pos) > minVariationDistance) {
                        balls[i].colorChecked = true;
                        AddBall(data);
                        return true;
                    }
                }
            } else {
                if(balls[i].pos.distance(data.pos) > minVariationDistance) {
                    AddBall(data);
                    return true;
                }
            }
        }
        return false;
    }
    
    void SetMinDistance(float p_minVariationDistance) {
        minVariationDistance = p_minVariationDistance;
    }
    
    void SetLifeTime(int p_lifeTime) {
        lifeTime = p_lifeTime;
    }

    void AddBall(BallData data) {
        balls.push_back(data);
    }
    
private:
    
    vector<BallData> balls;
    
    float minVariationDistance;
    int lifeTime;
};

#endif // BALLOBJECT_H
