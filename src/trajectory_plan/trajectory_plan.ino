#include <Servo.h>
#include <math.h>

const int Joint1Pin = 2;
const int Joint2Pin = 3;
const int Joint3Pin = 4;

const int Joint1Offset = 7;
const int Joint2Offset = 14;
const int Joint3Offset = -12;

typedef struct Joints {
  float t1;
  float t2;
  float t3;
} Joints;

typedef struct Position {
  float x;
  float y;
  float z;
} Position;

typedef struct Coefficients{
  float a0;
  float a1; 
  float a2;
  float a3;
} Coefficients;

// Link Length
const int L1 = 9.6;
const int L2 = 16.7;

// Positon and Joints Preset
const Joints INIT_JOINTS = {90, 90, 90};
const Position INIT_POSITION = {0, 16.7, -9.6};
const Position TARGETS[] = {{5, 10, 0}, {0, 25, 0}};
const int TARGET_NUMBER = 2;

// Time Presets
const float DURATION = 5000;
const int LOOP_DURATION = 100;

Servo s1, s2, s3;

Joints inverseKinematics(Position pos){
  float t1 = atan2(pos.y, pos.x);

  float r2 = pow(pos.x,2) + pow(pos.y,2);
  float R2 = r2 + pow(pos.z,2);

  float t2 = acos(constrain((R2 + L1*L1 - L2*L2)/(2*L1*sqrt(R2)), -1, 1)) - atan2(pos.z, sqrt(r2));
  float t3 = M_PI - acos(constrain((L1*L1 + L2*L2 - R2)/(2*L1*L2), -1, 1));

  Joints result = {t1, t2, t3};
  return result;
}

void standarlise(Joints &joints) {
  joints.t1 = (joints.t1 * 180 / M_PI + Joint1Offset);
  joints.t2 = (joints.t2 * 180 / M_PI + Joint2Offset);
  joints.t3 = (joints.t3 * 180 / M_PI + Joint3Offset);
}

Coefficients calculateCubicCoefficients(float u0, float uf) {
  float a0, a1, a2, a3;
  float t0 = 0, tfSec = DURATION/1000.0;
  
  a0 = u0;
  a1 = 0;
  a2 = (3.0 * (uf - u0)/pow(tfSec, 2));
  a3 = - (2.0 * (uf - u0)/pow(tfSec, 3));
  Coefficients co = {a0, a1, a2, a3};

  return co;
}


float calculateNextPos(Coefficients co, int tMSec) {
  float tSec = tMSec / 1000.0; 
  float result = co.a0 
                 + co.a1 * tSec
                 + co.a2 * pow(tSec, 2) 
                 + co.a3 * pow(tSec, 3);

  return result;
}

void moveRobot(Coefficients coeX, Coefficients coeY, Coefficients coeZ) {
  for(int t = 0; t <= DURATION; t += LOOP_DURATION) {
    float nextPosX = calculateNextPos(coeX, t);
    float nextPosY = calculateNextPos(coeY, t);
    float nextPosZ = calculateNextPos(coeZ, t);
    Position nextPos = {nextPosX, nextPosY, nextPosZ};

    Joints joints = inverseKinematics(nextPos);
    standarlise(joints);

    s1.write(joints.t1);
    s2.write(joints.t2);
    s3.write(joints.t3);
    
    delay(LOOP_DURATION);
  }
}

void goFromTo(Position targetPos, Position initPos){
  Coefficients coeX = calculateCubicCoefficients(initPos.x, targetPos.x);
  Coefficients coeY = calculateCubicCoefficients(initPos.y, targetPos.y);
  Coefficients coeZ = calculateCubicCoefficients(initPos.z, targetPos.z);

  moveRobot(coeX, coeY, coeZ);
}

void setup() {
  Serial.begin(9600);

  s1.attach(Joint1Pin);
  s2.attach(Joint2Pin);
  s3.attach(Joint3Pin);

  s1.write(INIT_JOINTS.t1+Joint1Offset);
  s2.write(INIT_JOINTS.t2+Joint2Offset);
  s3.write(INIT_JOINTS.t3+Joint3Offset);

  // Wait for 5s as required
  delay(5000);
}

void loop() {
  // Loop through the target
  for(int targetIndex=0; targetIndex < TARGET_NUMBER; targetIndex++){
    // Go to target
    goFromTo(INIT_POSITION, TARGETS[targetIndex]);
    // Go back from target
    goFromTo(TARGETS[targetIndex], INIT_POSITION);
  }

  //Stop the loop
  while(true){}
}
