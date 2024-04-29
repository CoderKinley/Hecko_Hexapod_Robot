#include <math.h>
#include <SPI.h>
#include <Wire.h>

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver hexapodLeg1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver hexapodLeg2 = Adafruit_PWMServoDriver(0x41);

#define servoMIN 150
#define servoMAX 600


#include "vector.h"
#include "Helper.h"
#include "Setup.h"
#include "RC.h"

//  //////////////////////////    CONFIGURATION   ///////////////////////////
 
//   ____________________________(((((((((((((((____________________________    
//   ____________________________(((((((((((((((((___________________________    
//   ____________________________(((((((((((((((((___________________________    
//   ___________________________((((((((((((((((((#__________________________    
//   ___________________________(((((((((((((((((((__________________________    
//   ________________________&(((((((((((((((((((((((________________________    
//   _______&((((((_______#(((((((((((((((((((((((((((((#_______((((((_______    
//   ______#((((((((((((((((((((((((/,.......,/((((((((((((((((((((((((#_____    
//   _____(((((((((((((((((((((,...................*(((((((((((((((((((((____    
//   ____(((((((((((((((((((,.........................*(((((((((((((((((((___    
//   ___((((((((((((((((((...............................((((((((((((((((((__    
//   _%(((((((((((((((((,.........(#############(........./(((((((((((((((((_    
//   __((((((((((((((((........(###################(........#(((((((((((((((_    
//   ____(((((((((((((......./########(.....(########,.......##(((((((((((___    
//   ______((((((((((.......(######............,######/.......###(((((((_____    
//   ________(((((((/....../#####/...............(#####,......#####((#_______    
//   _________((((((.......#####(.................######......,######________    
//   _________((((((.......#####...................#####.......######________    
//   _________((((((.......#####...................#####*******######________    
//   _________((((((.......#####/.................(#####******/######________    
//   ________(((((((/......##### ............... #####/******########_______    
//   ______#(((((((((.......#######.............######(*******#########%_____    
//   ____(((((((((((((.......(########*...../########(*******#############___    
//   __((((((((((((((((........#####################********################_    
//   _&(((((((((((((((((..........###############*********/#################_    
//   ___((((((((((((((((((.........**********************##################__    
//   ____(((((((((((((((((((......********************/###################___    
//   _____((((((((((((((((((((#..******************/#####################____    
//   ______(((((((((((((((((((((####/*********(#########################_____    
//   _______%((((((_______((((((((#######################______%######_______    
//   _________________________((((((#################________________________    
//   ___________________________((((((#############__________________________    
//   ___________________________((((((((###########__________________________    
//   ____________________________((((((((#########___________________________    
//   ____________________________(((((((((((######___________________________    
//   _____________________________((((((((((((###____________________________       

//  //////////////////////////    CONFIGURATION   ///////////////////////////

const int legPins[6][3] = {
  {0, 1, 2},  // Leg 1: Coxa, Femur, Tibia // 1ST PCA
  {3, 4, 5},  // Leg 2: Coxa, Femur, Tibia
  {6, 7, 8},   // Leg 3: Coxa, Femur, Tibia
  {0, 1, 2},  // Leg 4: Coxa, Femur, Tibia // 2ND PCA which is chained
  {3, 4, 5},  // Leg 5: Coxa, Femur, Tibia
  {6, 7, 8}   // Leg 6: Coxa, Femur, Tibia
};

enum State {
  Initialize,
  Stand,
  Car,
  Crab,
  Calibrate,
};

enum LegState {
  Propelling,
  Lifting,
  Standing,
  Reset
};

enum Gait {
  Tri,
  Wave,
  Ripple,
  Bi,
  Quad,
  Hop
};

int totalGaits = 6;
Gait gaits[6] = { Tri, Wave, Ripple, Bi, Quad, Hop };

float points = 1000;
int cycleProgress[6];
LegState legStates[6];
int standProgress = 0;

State currentState = Initialize;
Gait currentGait = Tri;
Gait previousGait = Tri;
int currentGaitID = 0;

float standingDistanceAdjustment = 0;

float distanceFromGroundBase = -60;
float distanceFromGround = 0;
float previousDistanceFromGround = 0;

float liftHeight = 80;
float landHeight = 70;
float strideOvershoot = 10;
float distanceFromCenter = 100;

float crabTargetForwardAmount = 0;
float crabForwardAmount = 0;

// just vector point initialization for the joystick movement
Vector2 joy1TargetVector = Vector2(0, 0);
float joy1TargetMagnitude = 0;

Vector2 joy1CurrentVector = Vector2(0, 0);
float joy1CurrentMagnitude = 0;

Vector2 joy2TargetVector = Vector2(0, 0);
float joy2TargetMagnitude = 0;

Vector2 joy2CurrentVector = Vector2(0, 0);
float joy2CurrentMagnitude = 0;

unsigned long timeSinceLastInput = 0;

float landingBuffer = 15;

long elapsedTime = 0;
long loopStartTime = 0;

void setup() {
  Serial.begin(9600);
  
  // for wireless communication
  RC_Setup();
  
  // for PCA servo drive
  hexapodLeg1.begin();
  hexapodLeg1.setPWMFreq(60);

  hexapodLeg2.begin();
  hexapodLeg2.setPWMFreq(60);

  stateInitialize();
  delay(2000);
}

void loop() {
  bool connected = RC_GetDataPackage();  // this is for the radio data
  if (connected) {

    // the analogRead is later fetched from the radio control
    double joy1xval = map(joy1x, 0, 1023, -100, 100);
    double joy1yval = map(joy1y, 0, 1023, -100, 100);
    double joy2xval = map(joy2x, 0, 1023, 100, -100);
    double joy2yval = map(joy2y, 0, 1023, 100, -100);

    joy1TargetVector = Vector2(joy1xval, joy1yval);
    joy1TargetMagnitude = constrain(calculateHypotenuse(abs(joy1xval), abs(joy1yval)), 0, 100);

    joy2TargetVector = Vector2(joy2xval, joy2yval);
    joy2TargetMagnitude = constrain(calculateHypotenuse(abs(joy2xval), abs(joy2yval)), 0, 100);

    int slider1 = map(liPo1, 0, 1023, 100, 0);
    previousDistanceFromGround = distanceFromGround;

    // distance from the ground determined by the slider 1 value
    distanceFromGround = distanceFromGroundBase + slider1 * -1.7;  

    // this is the distance from the centre of the body
    distanceFromCenter = 180; 

  } 

  joy1CurrentVector = lerp(joy1CurrentVector, joy1TargetVector, 0.08);  // ..> still gives the 100 to -100 ranged values
  joy1CurrentMagnitude = lerp(joy1CurrentMagnitude, joy1TargetMagnitude, 0.08);

  joy2CurrentVector = lerp(joy2CurrentVector, joy2TargetVector, 0.12);
  joy2CurrentMagnitude = lerp(joy2CurrentMagnitude, joy2TargetMagnitude, 0.12);

  // some sort of gait selection
  previousGait = currentGait;

  switch ((int)gait_count) {
    case 0:
      currentGaitID = 0;
      currentGait = gaits[currentGaitID];
      break;
    case 1:
      currentGaitID = 1;
      currentGait = gaits[currentGaitID];
      break;
    case 2:
      currentGaitID = 2;
      currentGait = gaits[currentGaitID];
      break;
    case 3:
      currentGaitID = 3;
      currentGait = gaits[currentGaitID];
      break;
    case 4:
      currentGaitID = 4;
      currentGait = gaits[currentGaitID];
      break;
    case 5:
      currentGaitID = 5;
      currentGait = gaits[currentGaitID];
      break;

    default:
      break;
  }

  // bool carStates = true;
  if (abs(joy1CurrentMagnitude) >= 10 || abs(joy2CurrentMagnitude) >= 10) {
    carState();
    timeSinceLastInput = millis();
    return;
  }

  if (abs(timeSinceLastInput - millis()) > 5) {
    standingState();
    return;
  }
}

// Initialize the robot
void stateInitialize() {
  moveToPos(0, Vector3(160, 0, 0));
  moveToPos(1, Vector3(160, 0, 0));
  moveToPos(2, Vector3(160, 0, 0));
  moveToPos(3, Vector3(160, 0, 0));
  moveToPos(4, Vector3(160, 0, 0));
  moveToPos(5, Vector3(160, 0, 0));
  delay(1000);

  moveToPos(0, Vector3(225, 0, 115));
  moveToPos(1, Vector3(225, 0, 115));
  moveToPos(2, Vector3(225, 0, 115));
  moveToPos(3, Vector3(225, 0, 115));
  moveToPos(4, Vector3(225, 0, 115));
  moveToPos(5, Vector3(225, 0, 115));
  delay(500);
}

void resetMovementVectors() {
  joy1CurrentVector = Vector2(0, 0);
  joy1CurrentMagnitude = 0;

  joy2CurrentVector = Vector2(0, 0);
  joy2CurrentMagnitude = 0;
}

void setCycleStartPoints(int leg) {
  cycleStartPoints[leg] = currentPoints[leg];
}

void setCycleStartPoints() {
  for (int i = 0; i < 6; i++) {
    cycleStartPoints[i] = currentPoints[i];
  }
}

// calculate the inverse kinematics
void inverseKinematics(int leg, Vector3 pos) {
  float x = pos.x;
  float y = pos.y;
  float z = pos.z;

  // Serial.print("X----> ");
  // Serial.println(x);
  // Serial.print("Y----> ");
  // Serial.println(y);
  // Serial.print("Z----> ");
  // Serial.println(z);

  float o1 = offsets[leg].x;
  float o2 = offsets[leg].y;
  float o3 = offsets[leg].z;

  // Inverse Kinematics second formula
  float theta1 = atan2(y, x) * (180 / PI) + o1;  // base angle
  float l = sqrt(x * x + y * y);                 // x and y extension
  float l1 = l - a1;
  float h = sqrt(l1 * l1 + z * z);

  float phi1 = acos(constrain((pow(h, 2) + pow(a2, 2) - pow(a3, 2)) / (2 * h * a2), -1, 1));
  float phi2 = atan2(z, l1);
  float theta2 = (phi1 + phi2) * 180 / PI + o2;
  float phi3 = acos(constrain((pow(a2, 2) + pow(a3, 2) - pow(h, 2)) / (2 * a2 * a3), -1, 1));
  float theta3 = 180 - (phi3 * 180 / PI) + o3;
  // theta3 = -abs(theta3);


  // The theta are added because of initial or actual servo setup offsets
  // float angle1 = 45 + theta1;
  // float angle2 = 67 + theta2;
  // float angle3 = 135 + theta3;
  targetRot = Vector3(theta1, theta2, theta3);
  return;
}

int angleToMicroseconds(double angle) {
  double val = servoMIN + (((servoMAX - servoMIN) / 180.0) * angle);
  return (int)val;
}

// making the leg to move to the desired location
void moveLegs(int legIndex, int coxa, int femur, int tibia) {
  if (legIndex <= 2 && legIndex >= 0){
    for (int i = 0; i < 3; i++) {
      hexapodLeg1.setPWM(legPins1[legIndex][i], 0, (i == 0) ? angleToMicroseconds(coxa) : (i == 1) ? angleToMicroseconds(femur) : angleToMicroseconds(tibia));
    }
  }
  else if (legIndex > 2 && legIndex < 6){
    for (int i = 0; i < 3; i++) {
      hexapodLeg2.setPWM(legPins1[legIndex][i], 0, (i == 0) ? angleToMicroseconds(coxa) : (i == 1) ? angleToMicroseconds(femur) : angleToMicroseconds(tibia));
    }
  }
}

// inverse kinematics calculation
void moveToPos(int leg, Vector3 pos) {
  currentPoints[leg] = pos;
  float dis = Vector3(0, 0, 0).distanceTo(pos);

  if (dis > legLength) {
    print_value("Point impossible to reach", pos, false);
    print_value("Distance", dis, true);
    return;
  }

  // calculate the leg inverse kinematics
  inverseKinematics(leg, pos);

  int coxaMicroseconds = angleToMicroseconds(targetRot.x);
  int femurMicroseconds = angleToMicroseconds(targetRot.y);
  int tibiaMicroseconds = angleToMicroseconds(targetRot.z);
  
  // Legs are moving due to this block of code here
  
  switch (leg) {
    case 0:
      moveLegs(0, targetRot.x, targetRot.y, targetRot.z);
      break;

    case 1:
      moveLegs(1, targetRot.x, targetRot.y, targetRot.z);                                                                                   
      break;

    case 2:
      moveLegs(2, targetRot.x, targetRot.y, targetRot.z);
      break;

    case 3:
      moveLegs(5, targetRot.x, targetRot.y, targetRot.z);
      break;

    case 4:
      moveLegs(4, targetRot.x, targetRot.y, targetRot.z);
      break;

    case 5:
      moveLegs(3, targetRot.x, targetRot.y, targetRot.z);
      break;

    default:
      break;
  }
  return;
}

// Standing mode --> when there is no control signal from the remote control ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void standingState() {
  bool moveAllAtOnce = false;
  bool highLift = false;
  setCycleStartPoints();
  standingEndPoint = Vector3(distanceFromCenter, 0, distanceFromGround + standingDistanceAdjustment);
  standLoops = 2;
  // We only set the starting, inbetween, and ending points one time, which is when we enter the standing state.
  if (currentState == Calibrate || currentState == Initialize) moveAllAtOnce = true;
  if (currentState != Stand) {

    set3HighestLeg();
    standLoops = 0;
    standProgress = 0;
    memcpy(standingStartPoints, currentPoints, sizeof(currentPoints[0]) * 6);
    currentState = Stand;

    // Calculate the inbetween and ending points
    for (int i = 0; i < 6; i++) {
      Vector3 inBetweenPoint = standingStartPoints[i];
      inBetweenPoint.x = (inBetweenPoint.x + standingEndPoint.x) / 2;
      inBetweenPoint.y = (inBetweenPoint.y + standingEndPoint.y) / 2;

      inBetweenPoint.z = ((inBetweenPoint.z + standingEndPoint.z) / 2);
      if (abs(inBetweenPoint.z - standingEndPoint.z) < 50) inBetweenPoint.z += 50;
      if (highLift) inBetweenPoint.z += 150;

      standingInBetweenPoints[i] = inBetweenPoint;

      SCPA[i][0] = standingStartPoints[i];
      SCPA[i][1] = standingInBetweenPoints[i];
      SCPA[i][2] = standingEndPoint;
    }

    for (int i = 0; i < 6; i++) {
      legStates[i] = Standing;
    }
  }

  //update distance from ground constantly
  for (int i = 0; i < 6; i++) {
    SCPA[i][2] = standingEndPoint;
  }

  //readjusting. This takes about a second
  while (standLoops < 2) {
    standProgress += 25;
    if (highLift) {
      standProgress += 40 - 50 * ((float)standProgress / points);
    }

    float t = (float)standProgress / points;
    if (t > 1) {
      t = 1;
    }

    if (moveAllAtOnce) {
      for (int i = 0; i < 6; i++) {
        moveToPos(i, GetPointOnBezierCurve(SCPA[i], 3, t));
      }

      if (standProgress > points) {
        standProgress = 0;
        standLoops = 2;
      }
    }

    else {
      for (int i = 0; i < 3; i++) {
        if (currentLegs[i] != -1) {
          moveToPos(currentLegs[i], GetPointOnBezierCurve(SCPA[currentLegs[i]], 3, t));
        }
      }

      if (standProgress > points) {
        standProgress = 0;
        standLoops++;
        set3HighestLeg();
      }
    }
  }

  //constantly move to the standing end position
  for (int i = 0; i < 6; i++) {
    moveToPos(i, GetPointOnBezierCurve(SCPA[i], 3, 1));
  }
  return;
}

void set3HighestLeg() {
  currentLegs[0] = -1;
  currentLegs[1] = -1;
  currentLegs[2] = -1;
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 6; i++) {  //go through the legs
      //if the leg is already on the list of current legs, skip it
      if (currentLegs[0] == i || currentLegs[1] == i || currentLegs[2] == i) continue;

      //if the leg is already in position, dont add it
      if (currentPoints[i] == standingEndPoint) continue;

      //if the legs z is greater than the leg already there, add it
      if (currentLegs[j] == -1 || currentPoints[i].z > currentPoints[currentLegs[j]].z) {
        currentLegs[j] = i;
      }
    }
  }
}

// Walk like a car yo!!!!!----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void carState() {
  // the 100 are all the rc.sliderLeftValues
  leftSlider = (int)map(liPo2, 0, 1023, 0, 100);
  globalSpeedMultiplier = (leftSlider + 10.0) * 0.01;
  globalRotationMultiplier = map(leftSlider, 0, 100, 40, 130) * 0.01;

  if (currentState != Car || previousGait != currentGait) {
    currentState = Car;

    //Initialize Leg States
    for (int i = 0; i < 6; i++) {
      legStates[i] = Reset;
    }

    Serial.println(currentGait);
    switch (currentGait) {
      case Tri:
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 2);
        cycleProgress[2] = 0;
        cycleProgress[3] = (points / 2);
        cycleProgress[4] = 0;
        cycleProgress[5] = (points / 2);

        pushFraction = 3.1 / 6.0;
        speedMultiplier = 1;
        strideLengthMultiplier = 1.2;
        liftHeightMultiplier = 1.1;
        maxStrideLength = 240;
        maxSpeed = 200;
        break;

      case Wave:
        //Offsets
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 6);
        cycleProgress[2] = (points / 6) * 2;
        cycleProgress[3] = (points / 6) * 5;
        cycleProgress[4] = (points / 6) * 4;
        cycleProgress[5] = (points / 6) * 3;

        //Percentage Time On Ground
        pushFraction = 5.0 / 6.0;

        speedMultiplier = 0.40;
        strideLengthMultiplier = 2;
        liftHeightMultiplier = 1.3;
        maxStrideLength = 150;
        maxSpeed = 160;
        break;

      case Ripple:
        //Offsets
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 6) * 4;
        cycleProgress[2] = (points / 6) * 2;
        cycleProgress[3] = (points / 6) * 5;
        cycleProgress[4] = (points / 6);
        cycleProgress[5] = (points / 6) * 3;

        //Percentage Time On Ground
        pushFraction = 3.2 / 6.0;

        speedMultiplier = 1;
        strideLengthMultiplier = 1.3;
        liftHeightMultiplier = 1;
        maxStrideLength = 220;
        maxSpeed = 200;
        break;

      case Bi:
        //Offsets
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 3);
        cycleProgress[2] = (points / 3) * 2;
        cycleProgress[3] = 0;
        cycleProgress[4] = (points / 3);
        cycleProgress[5] = (points / 3) * 2;

        //Percentage Time On Ground
        pushFraction = 2.1 / 6.0;

        speedMultiplier = 4;
        strideLengthMultiplier = 1;
        liftHeightMultiplier = 1.8;
        maxStrideLength = 230;
        maxSpeed = 130;
        break;

      case Quad:
        //Offsets
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 3);
        cycleProgress[2] = (points / 3) * 2;
        cycleProgress[3] = 0;
        cycleProgress[4] = (points / 3);
        cycleProgress[5] = (points / 3) * 2;

        //Percentage Time On Ground
        pushFraction = 4.1 / 6.0;


        speedMultiplier = 1;
        strideLengthMultiplier = 1.2;
        liftHeightMultiplier = 1.8;
        maxStrideLength = 220;
        maxSpeed = 200;
        break;

      case Hop:
        //Offsets
        cycleProgress[0] = 0;
        cycleProgress[1] = 0;
        cycleProgress[2] = 0;
        cycleProgress[3] = 0;
        cycleProgress[4] = 0;
        cycleProgress[5] = 0;

        //Percentage Time On Ground
        pushFraction = 3 / 6.0;

        speedMultiplier = 1;
        strideLengthMultiplier = 1.6;
        liftHeightMultiplier = 2.5;
        maxStrideLength = 240;
        maxSpeed = 200;
        break;
    }
  }

  for (int i = 0; i < 6; i++) {
    tArray[i] = (float)cycleProgress[i] / points;
  }

  forwardAmount = joy1CurrentMagnitude;
  turnAmount = joy2CurrentVector.x;

  moveToPos(0, getGaitPoint(0, pushFraction));
  moveToPos(1, getGaitPoint(1, pushFraction));
  moveToPos(2, getGaitPoint(2, pushFraction));
  moveToPos(3, getGaitPoint(3, pushFraction));
  moveToPos(4, getGaitPoint(4, pushFraction));
  moveToPos(5, getGaitPoint(5, pushFraction));

  float progressChangeAmount = (max(abs(forwardAmount), abs(turnAmount)) * speedMultiplier) * globalSpeedMultiplier;

  progressChangeAmount = constrain(progressChangeAmount, 0, maxSpeed * globalSpeedMultiplier);

  for (int i = 0; i < 6; i++) {
    cycleProgress[i] += progressChangeAmount;

    if (cycleProgress[i] >= points) {
      cycleProgress[i] = cycleProgress[i] - points;
    }
  }
}

// calculatin the next points for the locomotion of the robot-----------------------------------------------------------------------------------------------------------------------------------------------------------
Vector3 getGaitPoint(int leg, float pushFraction) {
  float rotateStrideLength = joy2CurrentVector.x * globalRotationMultiplier;
  Vector2 v = joy1CurrentVector * Vector2(1, strideLengthMultiplier);
  v.y = constrain(v.y, -maxStrideLength / 2, maxStrideLength / 2);
  v = v * globalSpeedMultiplier;

  float weightSum = abs(forwardAmount) + abs(turnAmount);
  float t = tArray[leg];

  //Propelling state
  if (t < pushFraction) {
    // Serial.println("Propelling");
    if (legStates[leg] != Propelling) setCycleStartPoints(leg);
    legStates[leg] = Propelling;

    ControlPoints[0] = cycleStartPoints[leg];
    ControlPoints[1] = Vector3(v.x * strideMultiplier[leg] + distanceFromCenter, -v.y * strideMultiplier[leg], distanceFromGround).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(distanceFromCenter, 0));
    ControlPointsAmount = 2;
    Vector3 straightPoint = GetPointOnBezierCurve(ControlPoints, ControlPointsAmount, mapFloat(t, 0, pushFraction, 0, 1));

    RotateControlPoints[0] = cycleStartPoints[leg];
    RotateControlPoints[1] = { distanceFromCenter + 40, 0, distanceFromGround };
    RotateControlPoints[2] = { distanceFromCenter, rotateStrideLength, distanceFromGround };
    RotateControlPointsAmount = 3;
    Vector3 rotatePoint = GetPointOnBezierCurve(RotateControlPoints, RotateControlPointsAmount, mapFloat(t, 0, pushFraction, 0, 1));
    return (straightPoint * abs(forwardAmount) + rotatePoint * abs(turnAmount)) / weightSum;
  }

  //Lifting
  else {
    // Serial.println("Lifting...");
    if (legStates[leg] != Lifting) setCycleStartPoints(leg);
    legStates[leg] = Lifting;

    ControlPoints[0] = cycleStartPoints[leg];
    ControlPoints[1] = cycleStartPoints[leg] + Vector3(0, 0, liftHeight * liftHeightMultiplier);
    ControlPoints[2] = Vector3(-v.x * strideMultiplier[leg] + distanceFromCenter, (v.y + strideOvershoot) * strideMultiplier[leg], distanceFromGround + landHeight).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(distanceFromCenter, 0));
    ControlPoints[3] = Vector3(-v.x * strideMultiplier[leg] + distanceFromCenter, v.y * strideMultiplier[leg], distanceFromGround).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(distanceFromCenter, 0));
    ControlPointsAmount = 4;
    Vector3 straightPoint = GetPointOnBezierCurve(ControlPoints, ControlPointsAmount, mapFloat(t, pushFraction, 1, 0, 1));

    RotateControlPoints[0] = cycleStartPoints[leg];
    RotateControlPoints[1] = cycleStartPoints[leg] + Vector3(0, 0, liftHeight * liftHeightMultiplier);
    RotateControlPoints[2] = { distanceFromCenter + 40, 0, distanceFromGround + liftHeight * liftHeightMultiplier };
    RotateControlPoints[3] = { distanceFromCenter, -(rotateStrideLength + strideOvershoot), distanceFromGround + landHeight };
    RotateControlPoints[4] = { distanceFromCenter, -rotateStrideLength, distanceFromGround };
    RotateControlPointsAmount = 5;
    Vector3 rotatePoint = GetPointOnBezierCurve(RotateControlPoints, RotateControlPointsAmount, mapFloat(t, pushFraction, 1, 0, 1));
    return (straightPoint * abs(forwardAmount) + rotatePoint * abs(turnAmount)) / weightSum;
  }
}

// Calculating the Bazier points for the trajectory planning----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Calculating the Bezier curve points
Vector2 GetPointOnBezierCurve(Vector2* points, int numPoints, float t) {
  Vector2 pos;
  for (int i = 0; i < numPoints; i++) {
    float b = binomialCoefficient(numPoints - 1, i) * pow(1 - t, numPoints - 1 - i) * pow(t, i);
    pos.x += b * points[i].x;
    pos.y += b * points[i].y;
  }
  return pos;
}

Vector3 GetPointOnBezierCurve(Vector3* points, int numPoints, float t) {
  Vector3 pos;
  for (int i = 0; i < numPoints; i++) {
    float b = binomialCoefficient(numPoints - 1, i) * pow(1 - t, numPoints - 1 - i) * pow(t, i);
    pos.x += b * points[i].x;
    pos.y += b * points[i].y;
    pos.z += b * points[i].z;
  }
  return pos;
}

int binomialCoefficient(int n, int k) {
  int result = 1;
  for (int i = 1; i <= k; i++) {
    result *= (n - (k - i));
    result /= i;
  }
  return result;
}
