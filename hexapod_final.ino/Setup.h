// some of the setup functions

// Pins mapping for leg servos under driver 1
const int legPins1[6][3] = {
  {0, 1, 2},  // Leg 1: Coxa, Femur, Tibia // 1ST PCA
  {3, 4, 5},  // Leg 2: Coxa, Femur, Tibia
  {6, 7, 8},   // Leg 3: Coxa, Femur, Tibia
  {0, 1, 2},  // Leg 4: Coxa, Femur, Tibia // 2ND PCA
  {3, 4, 5},  // Leg 5: Coxa, Femur, Tibia
  {6, 7, 8}   // Leg 6: Coxa, Femur, Tibia
};

const Vector3 offsets1 = {90,66,-90};
const Vector3 offsets2 = {90,66,-90};
const Vector3 offsets3 = {90,66,-90}; 
const Vector3 offsets4 = {90,66,-90};
const Vector3 offsets5 = {90,66,-90};
const Vector3 offsets6 = {90,66,-90};
const Vector3 offsets[6] = {offsets1, offsets2, offsets3, offsets4, offsets5, offsets6};

const float a1 = 40;  //Coxa Length
const float a2 = 80; //Femur Length
const float a3 = 150; //Tibia Length   

float legLength = a1+a2+a3;

Vector3 currentPoints[6];
Vector3 cycleStartPoints[6];

Vector3 currentRot(180, 0, 180);
Vector3 targetRot(180, 0, 180);

float strideMultiplier[6] = {1, 1, 1, -1, -1, -1};
float rotationMultiplier[6] = {-1, 0, 1, -1, 0 , 1};

Vector3 ControlPoints[10];
Vector3 RotateControlPoints[10];

//Standing Control Points Array
Vector3 SCPA[6][10];

Vector3 standingStartPoints[6];      //the points the legs are at in the beginning of the standing state
Vector3 standingInBetweenPoints[6];  //the middle points of the bezier curves that the legs will follow to smoothly transition to the end points
Vector3 standingEndPoint;

int currentLegs[3] = { -1, -1, -1 };
int standLoops = 0;


// Car state variable initialization
float forwardAmount;
float turnAmount;
float  tArray[6];
int ControlPointsAmount = 0;
int RotateControlPointsAmount = 0;
float pushFraction = 3.0/6.0; // Percentage of time on the ground compared to that of the lift phase
float speedMultiplier = 0.5;
float strideLengthMultiplier = 1.5;
float liftHeightMultiplier = 1.0;
float maxStrideLength = 200;
float maxSpeed = 100;
float legPlacementAngle = 56;

int leftSlider = 50;
float globalSpeedMultiplier = 0.55;
float globalRotationMultiplier = 0.55;

