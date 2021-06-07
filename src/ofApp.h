#pragma once

#include "ofMain.h"
#include "ode/ode.h"
#include "ofxGui.h"
class ofApp : public ofBaseApp{

public:
    void setup();
    void update();
    void draw();
    void exit();


    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);

    /* These variables needed for ode files */
    dWorldID world;
    dSpaceID space;
    dSpaceID car_space;

    dGeomID ground;
    dGeomID box[10000];
    dGeomID sphere[10000];
    dGeomID ball;
    dGeomID turret[10000];
    dGeomID targetID;

    vector<dGeomID> target;

    dBodyID body[10000];
    dBodyID cannon[10000];
    dBodyID targetBody[10000];
    dJointID joint[100000];
    dJointID cannonTurret;
    dJointGroupID contactgroup;
    dJointGroupID cannongroup;
    dReal speed,steer;
    dMass m;
    ofEasyCam cam;

    ofxGuiGroup settings;
    ofxButton practice;
    ofxButton reaction;
    ofxButton timed;
    ofxButton quit;
    ofxLabel score;
    ofParameter<float> endTime;

    int shot;
    int misses;
    int scoreval;
    int keys[65536];
    int type;
    int rand;
    int timer;
    int randomTarget;
    int currentTarget;
    unsigned int val;

    bool hit;
    bool showGui;
    bool timerEnd;
    float  accuracy;
    float startTime;
    float maxTime;

    void cameraControl();
    void game();
    void drawCannon(const dReal*pos_ode, const dMatrix3 rot_ode, const dReal*sides_ode);
    void drawTurret(const dReal*pos_od1e, const dMatrix3 rot_ode, dReal len, dReal rad);
    void drawBox(const dReal*pos_ode, const dQuaternion rot_ode, const dReal*sides_ode);
    void drawSph(const dReal*pos_ode, const dQuaternion rot_ode, const double radius);
    void drawCyl(const dReal*pos_ode, const dQuaternion rot_ode, dReal len, dReal rad);
    void collide (dGeomID o1, dGeomID o2);


    vector<ofTexture> planet;
    ofTexture planets;
    ofPlanePrimitive bg[6];
    vector<ofTexture> background;
    ofTexture backgrounds;
    ofSpherePrimitive sph;

};

/* ODE requires a global function to use as the collision callback; this
 * function, combined with the ofApp pointer, allows us to put the collision
 * code within myApp. Look at the .cpp for details of how this works.
 */
static void nearCallback (void *, dGeomID o1, dGeomID o2);
extern ofApp *myApp;


