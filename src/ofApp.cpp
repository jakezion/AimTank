#include "ofApp.h"
#include "ofxGui.h"

#define LENGTH 3.5        // chassis length
#define WIDTH 2.5        // chassis width
#define HEIGHT 1.0        // chassis height
#define RADIUS 0.5        // wheel radius
#define STARTZ 1.0        // starting height of chassis
#define CMASS 1            // chassis mass
#define WMASS 0.5            // wheel mass
#define COMOFFSET -5        // center of mass offset
#define FMAX 25            // car engine fmax
#define ROWS 1            // rows of cars
#define COLS 1            // columns of cars
#define ITERS 20        // number of iterations
#define DISABLE_THRESHOLD 0.008    // maximum velocity (squared) a body can have and be disabled
#define DISABLE_STEPS 10    // number of steps a box has to have been disable-able before it will be disabled
#define CANNON_BALL_MASS 50    // mass of the cannon ball
#define CANNON_BALL_RADIUS 0.4
#define CLENGTH 2   //cannon length
#define CWIDTH 2  //cannon width
#define CHEIGHT 2 //cannon height
#define TARGET_RADIUS 3.5 //targets radius


static const dVector3  yunit = {0, 1, 0}, zunit = {0, 0, 1};

dReal cannon_angle=0,cannon_elevation=-1.2;

void ofApp::setup(){

        // create world
        dInitODE2(0);
        world = dWorldCreate();
        space = dHashSpaceCreate (0);
        contactgroup = dJointGroupCreate (0);
        dWorldSetGravity (world,0,0,-0.5);
        ground = dCreatePlane (space,0,0,1,0);

        ofEnableTextureEdgeHack();
        ofSetVerticalSync(true);
        ofEnableSmoothing();
        ofSetFrameRate(60);
        ofDisableArbTex();
        //initialise variables
        timerEnd = false;
        maxTime = 120000;
        timer = 120;
        int i;
        hit = false;
        scoreval = 0;
        misses = 0;
        accuracy = 0;
        shot = 0;
        rand = 1;
        speed=0, steer=0;


    // chassis body
    body[0] = dBodyCreate (world);
    dBodySetPosition (body[0],0,0,STARTZ);
    dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
    dMassAdjust (&m,CMASS);
    dBodySetMass (body[0],&m);
    box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
    dGeomSetBody (box[0],body[0]);

    // wheel bodies
    for (i=1; i<=4; i++) {
      body[i] = dBodyCreate (world);
      dQuaternion q;
      dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
      dBodySetQuaternion (body[i],q);
      dMassSetSphere (&m,1,RADIUS);
      dMassAdjust (&m,WMASS);
      dBodySetMass (body[i],&m);
      sphere[i-1] = dCreateSphere (0,RADIUS);
      dGeomSetBody (sphere[i-1],body[i]);
    }
    dBodySetPosition (body[1],0.5*LENGTH,WIDTH*0.5,STARTZ-HEIGHT*0.20);
    dBodySetPosition (body[2],0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.20);
    dBodySetPosition (body[3],-0.5*LENGTH, WIDTH*0.5,STARTZ-HEIGHT*0.20);
    dBodySetPosition (body[4],-0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.20);

    // front and back wheel hinges
    for (i=0; i<4; i++) {
      joint[i] = dJointCreateHinge2 (world,0);
      dJointAttach (joint[i],body[0],body[i+1]);
      const dReal *a = dBodyGetPosition (body[i+1]);
      dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
      dJointSetHinge2Axes (joint[i], zunit, yunit);
    }
    // set joint suspension
    for (i=0; i<4; i++) {
      dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
      dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
    }

    // lock back wheels along the steering axis
    for (i=2; i<4; i++) {
      // set stops to make sure wheels always stay in alignment
      dJointSetHinge2Param (joint[i],dParamLoStop,0);
      dJointSetHinge2Param (joint[i],dParamHiStop,0);
    }

    //cannon setup
    cannon[0] = dBodyCreate (world);
    dBodySetPosition (cannon[0],0,0,STARTZ+1.5);
    turret[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
    dGeomSetBody (turret[0],cannon[0]);

    //turret setup
    cannon[1] = dBodyCreate (world);
    dBodySetPosition (cannon[1],0,0,STARTZ+1.5);
    turret[1] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
    dGeomSetBody (turret[1],cannon[1]);

    // create car space and add it to the top level space
    //add cannon to car space so they act as the same body
    car_space = dSimpleSpaceCreate (space);
    dSpaceSetCleanup (car_space,0);
    dSpaceAdd (car_space,box[0]);
    dSpaceAdd (car_space,sphere[0]);
    dSpaceAdd (car_space,sphere[1]);
    dSpaceAdd (car_space,sphere[2]);
    dSpaceAdd (car_space,sphere[3]);
    dSpaceAdd (car_space,turret[0]);
    dSpaceAdd (car_space,turret[1]);

    //cannon ball setup
    body[5] = dBodyCreate(world);
    dMassSetSphere(&m, CANNON_BALL_MASS, CANNON_BALL_RADIUS);
    dMassAdjust(&m, CANNON_BALL_MASS);
    dBodySetMass(body[5], &m);
    ball = dCreateSphere(space, CANNON_BALL_RADIUS); //space not 0
    dGeomSetBody(ball, body[5]);
    dBodyDisable(body[5]);
    dGeomDisable(ball);
    dBodySetPosition(body[5],0,0,-10000);

    //set up key inputs for arrow keys
    for(int j=0; j<65536; j++) keys[j]=0;

    //target setup
    for(unsigned int k = 0; k < 50 ;k++){
        targetBody[k] = dBodyCreate(world);
        dBodySetPosition(targetBody[k],ofRandom(-200,200), ofRandom(-200,200), ofRandom(10,70));
        target.push_back(dCreateSphere(space, TARGET_RADIUS)); //CHECK
        dGeomSetBody(target[k], targetBody[k]);
        dBodySetGravityMode(targetBody[k],0);
        sph.set(TARGET_RADIUS*1.05,128);

    }
    //textures for targets
    for(int i=1; i<=12;i++){
        if(!ofLoadImage(planets, to_string(i)+".png")) { std::cerr << "Failed to load planet"+to_string(i)+"texture." << std::endl; }
       planets.setTextureWrap(GL_REPEAT, GL_REPEAT);

        planet.push_back(planets);
    }

    //setup faces for skybox

       ofLoadImage(backgrounds, "positive_z.png");
       backgrounds.setTextureWrap(GL_REPEAT, GL_REPEAT);
       background.push_back(backgrounds);
       ofLoadImage(backgrounds, "positive_y.png");
       backgrounds.setTextureWrap(GL_REPEAT, GL_REPEAT);
       background.push_back(backgrounds);
       ofLoadImage(backgrounds, "negative_x.png");
       backgrounds.setTextureWrap(GL_REPEAT, GL_REPEAT);
       background.push_back(backgrounds);
       ofLoadImage(backgrounds, "negative_z.png");
       backgrounds.setTextureWrap(GL_REPEAT, GL_REPEAT);
       background.push_back(backgrounds);
       ofLoadImage(backgrounds, "negative_y.png");
       backgrounds.setTextureWrap(GL_REPEAT, GL_REPEAT);
       background.push_back(backgrounds);
       ofLoadImage(backgrounds, "positive_x.png");
       backgrounds.setTextureWrap(GL_REPEAT, GL_REPEAT);
       background.push_back(backgrounds);

    dAllocateODEDataForThread(dAllocateMaskAll);

    //setup GUI buttons and initial positioning
    settings.setup("Options");
    settings.add(practice.setup("Practice"));
    settings.add(timed.setup("Timed"));
    settings.add(reaction.setup("Reaction"));
    settings.add(quit.setup("Quit"));
    settings.setSize(800,600);
    settings.setWidthElements(800);

}




void ofApp::cameraControl(){
    //cannon matrix aiming system
    dMatrix3 R2, R3, R4;
    dRFromAxisAndAngle(R2, 0, 0, 1, cannon_angle);
    dRFromAxisAndAngle(R3, 0, 1, 0, -cannon_elevation);
    dMultiply0(R4, R2, R3, 3, 3, 3);
    float tPos[3];
    const dReal *cpos = dBodyGetPosition(body[0]);
    tPos[0] = cpos[0];
    tPos[1] = cpos[1];
    tPos[2] = cpos[2];
    dQuaternion q;
    dQfromR(q, R4);
    //cam.disableMouseInput();
    //camera set to look at tank whilst rotating based on cannons current orientation
    ofVec3f upVector;
    upVector.set(0,0,1);
    cam.setAutoDistance(false);
    cam.setNearClip(0.01);
    cam.setPosition(tPos[0]-20,tPos[1],tPos[2]-35);
    cam.setOrientation({90,0,0});
    cam.rotateDeg(270,0,0,1);
    cam.rotateAround({glm::quat(q[0],q[1],q[2],q[3])},{tPos[0],tPos[1],tPos[2]});
    cam.lookAt({tPos[0],tPos[1],tPos[2]},upVector);
    cam.setUpAxis(upVector);
}






void ofApp::update(){
    //calls camera
    cameraControl();
    /*
    if statements to check what button has been pressed,
    then resets all target bodies so they're new compared
    to the previous game mode
    */
    if(practice){
        for (unsigned int i = 0; i< target.size(); i++){
            dGeomEnable(target[i]);
            dBodyEnable(dGeomGetBody(target[i]));
            dBodySetPosition(targetBody[i],ofRandom(-200,200), ofRandom(-200,200), ofRandom(10,70));
        }
    }
    if(timed){
        for (unsigned int i = 0; i< target.size(); i++){
            dGeomEnable(target[i]);
            dBodyEnable(dGeomGetBody(target[i]));
         dBodySetPosition(targetBody[i],ofRandom(-200,200), ofRandom(-200,200), ofRandom(10,70));
        }
    }
    if(reaction){
        for (unsigned int i = 0; i< target.size(); i++){
            dGeomEnable(target[i]);
            dBodyEnable(dGeomGetBody(target[i]));
         dBodySetPosition(targetBody[i],ofRandom(-200,200), ofRandom(-200,200), ofRandom(10,70));
        }
    }

    //key inputs
    if(timerEnd == false){
    if(keys[OF_KEY_LEFT]==1) steer -= 0.25;
    if(keys[OF_KEY_RIGHT]==1) steer += 0.25;
    if(keys[OF_KEY_UP]==1) speed += 0.36;
    if(keys[OF_KEY_DOWN]==1) speed -= 0.36;

    //if key released slowly deccelerate
    if(!keys[OF_KEY_LEFT] && !keys[OF_KEY_RIGHT]) steer *= 0.9;
    if(!keys[OF_KEY_UP] && !keys[OF_KEY_DOWN])  speed *= 0.99; //0.99
    } else {
            steer = 0;
            speed = 0;
    }
    // motor
    dJointSetHinge2Param (joint[0],dParamVel2,-speed);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);
    // steering: Conencted to turn angle
    for (int i=0; i<2;i++){

        dReal v = steer - dJointGetHinge2Angle1 (joint[i]);
        if (v > 0.1) v = 0.1;
        if (v < -0.1) v = -0.1;
        v *= 10.0;
        dJointSetHinge2Param (joint[i],dParamVel,v);
        dJointSetHinge2Param (joint[i],dParamFMax,0.2);
        dJointSetHinge2Param (joint[i],dParamLoStop,-0.75);
        dJointSetHinge2Param (joint[i],dParamHiStop,0.75);
        dJointSetHinge2Param (joint[i],dParamFudgeFactor,0.1);

    }

    dSpaceCollide (space,0,&nearCallback);
    /*if statement to check if target has been collided with
     * if true, disable target and add a point to score
     * */
    if(hit){
       dBodySetPosition(dGeomGetBody(targetID),20,20,-10000);
       dGeomDisable(targetID);
       dBodyDisable(dGeomGetBody(targetID));
       hit = false;
       scoreval++;
    } else {
    dWorldStep (world,0.09);
    }
    // remove all contact joints
    dJointGroupEmpty (contactgroup);

}



void ofApp::game(){
    if (timerEnd==false){
    // chassis
    ofSetColor(ofColor::forestGreen);
    const dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
    const dReal *cpos = dBodyGetPosition(body[0]);
    drawBox(cpos, dBodyGetQuaternion(body[0]), sides);
    // wheels
    ofSetColor(ofColor::limeGreen);
    for (int i=1; i<=4; i++) {
        drawCyl(dBodyGetPosition(body[i]), dBodyGetQuaternion(body[i]),0.4f,RADIUS);
    }
    //cannon positioning
    dMatrix3 R2, R3, R4;
    dRFromAxisAndAngle(R2, 0, 0, 1, 3.15+cannon_angle);
    dRFromAxisAndAngle(R3, 0, 1, 0, cannon_elevation);
    dMultiply0(R4, R2, R3, 3, 3, 3);
    float tPos[3];
    const dReal csides[3] = {CLENGTH, CWIDTH, CHEIGHT};
    tPos[0] = cpos[0];
    tPos[1] = cpos[1];
    tPos[2] = cpos[2] + 1.5;
    ofSetColor(ofColor::limeGreen);
    drawCannon(tPos,  R2, csides);
    for (int i = 0; i < 3; i++) tPos[i] += 1.5* R4[i * 4 + 2 ];
    ofSetColor(ofColor::forestGreen);
    drawTurret(tPos, R4,3,0.5);
    ofSetColor(255);
    float cannon_pos[3];
    //cannonball out of bounds
    const dReal *bPos = dBodyGetPosition(body[5]);
    cannon_pos[0] = bPos[0];
    cannon_pos[1] = bPos[1];
    cannon_pos[2] = bPos[2];
    if (cannon_pos[0] >= 210 || cannon_pos[0] <= -210 || cannon_pos[1] >= 210 || cannon_pos[1] <= -210 || cannon_pos[2] >= 210){
        cannon_pos[0] = 0; cannon_pos[1] = 0; cannon_pos[2] = -10000;
    }
    drawSph(cannon_pos,dBodyGetRotation(body[5]) , CANNON_BALL_RADIUS);
 if(type == 1 || type == 2){
     //spawning of targets for practice and timed
            for(unsigned int i =0; i < target.size(); i++){
                float pose[3];
                const dReal *pos = dBodyGetPosition(targetBody[i]);
                pose[0] = pos[0];
                pose[1] = pos[1];
                pose[2] = pos[2];
                sph.setPosition(pose[0],pose[1],pose[2]);
                ofSetColor(255,255,255,0);
                drawSph(dBodyGetPosition(targetBody[i]), dBodyGetQuaternion(targetBody[i]),TARGET_RADIUS);
                if (rand == 10){
                    rand = 0;
                }
                ofSetColor(255);
                planet[rand].bind();
                sph.draw();
                planet[rand].unbind();
                rand ++;
            }
 }
 if(type == 3){
      //spawning of targets for reaction, requires single target spawned at a time
         float wait = ofGetElapsedTimeMillis()/1000 % 10;
         if(wait != 0.0){
         for(unsigned int i =0; i < target.size(); i++){
            if(!(target[i] == target[currentTarget])){
                dGeomDisable(target[i]);
                dBodyDisable(dGeomGetBody(target[i]));
            }
         }
         float pose[3];
         const dReal *pos = dBodyGetPosition(targetBody[currentTarget]);
         pose[0] = pos[0];
         pose[1] = pos[1];
         pose[2] = pos[2];
         sph.setPosition(pose[0],pose[1],pose[2]);
         ofSetColor(255,255,255,0);
         drawSph(dBodyGetPosition(targetBody[currentTarget]), dBodyGetQuaternion(targetBody[currentTarget]),TARGET_RADIUS);
         ofSetColor(255);
         planet[10].bind();
         sph.draw();
         planet[10].unbind();
         } else {
             //sets 1 target to current target and draws it, rest are disabled
               randomTarget = ofRandom(target.size());
               currentTarget = randomTarget;
               for(unsigned int i =0; i < target.size(); i++){
                      dGeomEnable(target[i]);
                      dBodyEnable(dGeomGetBody(target[i]));
               }
            }
        }
    }
}

void ofApp::draw(){
    ofBackground(0);
    //resets variables when a button is pressed
    if (practice){
        type = 1;
        scoreval = 0;
        shot = 0;
        accuracy=0;
        timer = maxTime/1000;
        settings.setSize(200,30);
        settings.setWidthElements(200);
        settings.setPosition(5,20);
        timerEnd = false;
    }
    if (timed){
        type = 2;
        scoreval = 0;
        shot = 0;
        accuracy=0;
        settings.setSize(200,30);
        settings.setWidthElements(200);
        settings.setPosition(5,45);
        timer = maxTime/1000;
        timerEnd = false;
        ofResetElapsedTimeCounter();
    }
    if (reaction){
        type = 3;
        scoreval = 0;
        shot = 0;
        accuracy=0;
        settings.setSize(200,30);
        settings.setPosition(5,45);
        settings.setWidthElements(200);
        timer = maxTime/1000;
        timerEnd = false;
        ofResetElapsedTimeCounter();
    }
    if (quit){
           ofExit();
    }
    ofEnableDepthTest();
    ofPushMatrix();
    cam.begin();
   //skybox draw to correct position
   for (int i=0;i<6;i++){
       bg[i].set(420,420);
   }

   for (int i=0;i<=1;i++){
      bg[i].setOrientation({90,0,0});
   }
   for (int i=2;i<=3;i++){
      bg[i].setOrientation({0,90,0});
   }
   for (int i=4;i<=5;i++){
      bg[i].setOrientation({0,0,90});
   }

   bg[0].setGlobalPosition(0,210,0);
   bg[1].setGlobalPosition(0,-210,0);
   bg[2].setGlobalPosition(210,0,0);
   bg[3].setGlobalPosition(-210,0,0);
   bg[4].setGlobalPosition(0,0,210);
   bg[5].setGlobalPosition(0,0,-210);

   for (int i=0;i<6;i++){
       background[i].bind();
       bg[i].draw();
       background[i].unbind();
   }

  ofDrawAxis(0);

    if(type == 1 || type == 2 || type == 3){
        game();
       }
    cam.end();
    ofPopMatrix();
    ofDisableDepthTest();
    //timer check for timed game modes
   float timerBar = ofMap(timer,0.0,maxTime/1000,0,1,true);
       if (timer == 0){
           //if timer 0 display statistics
           timerEnd = true;
              if (type == 2 || type == 3){
                   accuracy = (float(scoreval)/shot)*(100.0);
                   if (shot > scoreval){
                   misses = shot-scoreval;
                  }
            ofDrawBitmapString("Total Shots: " + ofToString(shot), ofGetWidth()/2-50, ofGetHeight()/2-30);
            ofDrawBitmapString("Total Hits: " + ofToString(scoreval), ofGetWidth()/2-50, ofGetHeight()/2-15);
            ofDrawBitmapString("Total Misses: " + ofToString(misses), ofGetWidth()/2-50, ofGetHeight()/2);
            ofDrawBitmapString("Accuracy: " + ofToString(accuracy)+"%", ofGetWidth()/2-50, ofGetHeight()/2+15);

            cam.setPosition(0,0,20);
            cam.setOrientation({0,0,1});
              }
       } else{
           if (type == 2 || type == 3){
                //if timer not 0, change timer bar colour based on time remaining
           ofDrawBitmapString("Time Remaining: " + ofToString(timer), 5, 30);
           if(timer > 100) ofSetColor(ofColor::limeGreen);
           if (timer <= 100) ofSetColor(ofColor::yellowGreen);
           if (timer <= 80) ofSetColor(ofColor::yellow);
           if (timer <= 60) ofSetColor(ofColor::orange);
           if (timer <= 30) ofSetColor(ofColor::red);
           ofFill();
           ofRect(5,35,200*timerBar, 10);
           ofSetColor(ofColor::white);
           timer = (maxTime - ofGetElapsedTimeMillis())/1000;
           }
           if(type ==1 || type == 2){
            ofDrawBitmapString("Hits: " + ofToString(scoreval), 5, 15);
           } else if (type == 3){
            ofDrawBitmapString("Hits: " + ofToString(scoreval), 5, 15);
           }
       }
       //draw GUI above camera
       if(type == NULL){
           settings.setPosition((ofGetWidth()/2)-400,ofGetHeight()/2);
       }
    settings.draw();
}





void ofApp::collide(dGeomID o1, dGeomID o2)
{
  int i,n;
  //checks for every target if it is currently colliding with the cannonball
 for(auto x: target){
      if(o2 == x){
          //if true, send placeholder value of target to update loop to be disabled
          targetID = x;
          hit = true;
      }
//checks if any body is colliding with another
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnected(b1, b2))
      return;
}
  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
        dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.5;
      contact[i].surface.soft_cfm = 0.3;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
                    dGeomGetBody(contact[i].geom.g1),
                    dGeomGetBody(contact[i].geom.g2));
    }
  }
}





void ofApp::keyPressed(int key){

    keys[key] = 1;

    switch(key) {
    case 'a':
         case 'A':
        //cannon moves left
        cannon_angle += 0.05;
        break;
    case 'd':
        case 'D':
          //cannon moves left
        cannon_angle -= 0.05;
        break;
    case 'w':
         case 'W':
          //cannon moves up
        cannon_elevation += 0.1;
        break;
    case 's':
         case 'S':
          //cannon moves down
          cannon_elevation -= 0.1;
        break;
       case 'x':
       case 'X': {
        //sets angular and linear velocity for cannonball based on position and angle of turret
        shot++;
         dBodyEnable(body[5]);
         dGeomEnable(ball);
         dMatrix3 R2, R3, R4;
         dRFromAxisAndAngle(R3, 0, 1, 0, cannon_elevation);
         dRFromAxisAndAngle(R2, 0, 0, 1, 3.15+cannon_angle);
         dMultiply0(R4, R2, R3, 3, 3, 3);
         float tPos[3];
         const dReal *cpos = dBodyGetPosition(body[0]);
         tPos[0] = cpos[0];
         tPos[1] = cpos[1];
         tPos[2] = cpos[2] +1.5;
         for (int i = 0; i < 3; i++) tPos[i] += R4[i * 4 + 2];
         dBodySetPosition(body[5],tPos[0], tPos[1], tPos[2]);
         dReal force = 10;
         dBodySetLinearVel(body[5], force * R4[2], force * R4[6], force * R4[10]);
         dBodySetAngularVel(body[5], 1, 1, 0);
         break;
       }
    case ' ':
        //stop tank
      speed = 0;
      steer = 0;
        break;
    }
}

void ofApp::drawCannon(const dReal*pos_ode, const dMatrix3 rot_ode, const dReal*sides_ode)
{
    //same design drawBox but orientation displayed using matrix value of cannon for accurate rotation
    ofBoxPrimitive b;
    dQuaternion q;
    dQfromR(q, rot_ode);
    b.setScale(glm::vec3(0.01*sides_ode[0],0.01*sides_ode[1],0.01*sides_ode[2]));
    b.setGlobalOrientation(glm::quat(q[0],q[1],q[2],q[3]));
    b.setGlobalPosition(glm::vec3(pos_ode[0],pos_ode[1],pos_ode[2]));
    b.draw();

}

void ofApp::drawTurret(const dReal*pos_ode, const dMatrix3 rot_ode, dReal len, dReal rad){
        //same design drawCyl but orientation displayed using matrix value of cannon for accurate rotation
    ofCylinderPrimitive c;
    dQuaternion q;
    dQfromR(q, rot_ode);
    ofQuaternion rot_of(glm::quat(q[0],q[1], q[2], q[3]));
    ofQuaternion fix_cy; fix_cy.makeRotate(180,0,1,1);
    ofQuaternion rot_final = fix_cy * rot_of;
    c.setScale(glm::vec3(rad/60.0,len/80.0,rad/60.0));
    c.setGlobalOrientation(rot_final);
    c.setGlobalPosition(glm::vec3(pos_ode[0],pos_ode[1],pos_ode[2]));

    c.draw();
}

void ofApp::drawBox(const dReal*pos_ode, const dQuaternion rot_ode, const dReal*sides_ode)
{
        //draws a box
    ofBoxPrimitive b;
    b.setScale(glm::vec3(0.01*sides_ode[0],0.01*sides_ode[1],0.01*sides_ode[2]));
    b.setGlobalOrientation(glm::quat(rot_ode[0],rot_ode[1],rot_ode[2],rot_ode[3]));
    b.setGlobalPosition(glm::vec3(pos_ode[0],pos_ode[1],pos_ode[2]));
    b.draw();
}

void ofApp::drawCyl(const dReal*pos_ode, const dQuaternion rot_ode, dReal len, dReal rad)
{
     //draws a cylinder
    ofCylinderPrimitive c;
    ofQuaternion rot_of(rot_ode[1], rot_ode[2], rot_ode[3], rot_ode[0]);
    ofQuaternion fix_cy; fix_cy.makeRotate(90,1,0,0);
    ofQuaternion rot_final = fix_cy * rot_of;
    c.setScale(glm::vec3(rad/60.0,len/80.0,rad/60.0));
    c.setGlobalOrientation(rot_final);
    c.setGlobalPosition(glm::vec3(pos_ode[0],pos_ode[1],pos_ode[2]));
    c.draw();
}




void ofApp::drawSph(const dReal*pos_ode, const dQuaternion rot_ode, const double radius)
{
     //draws a sphere
    ofSpherePrimitive s;
    s.setRadius(radius);
    s.setGlobalOrientation(glm::quat(rot_ode[0],rot_ode[1],rot_ode[2],rot_ode[3]));
    s.setGlobalPosition(glm::vec3(pos_ode[0],pos_ode[1],pos_ode[2]));
    s.draw();
}

//--------------------------------------------------------------
void ofApp::exit() {
    //destroys all geometrys and closes ode
    dGeomDestroy (box[0]);
    dGeomDestroy (sphere[0]);
    dGeomDestroy (sphere[1]);
    dGeomDestroy (sphere[2]);
    dGeomDestroy (sphere[3]);
    dGeomDestroy (ball);
    for (auto i: target){
    dGeomDestroy (i);
    //dBodyDestroy(dGeomGetBody(i));
    }
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
        dCloseODE();
}
static void nearCallback (void *, dGeomID o1, dGeomID o2) {
    myApp->collide(o1,o2);
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    keys[key]=0;
}


//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
