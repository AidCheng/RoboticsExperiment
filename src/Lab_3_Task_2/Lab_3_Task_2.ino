#include <Servo.h>
#include <math.h>

const int Joint1Pin = 2;
const int Joint2Pin = 3;
const int Joint3Pin = 4;

const int Joint1Offset = 7;
const int Joint2Offset = 14;
const int Joint3Offset = -12;

const float L1 = 9.6;
const float L2 = 16.7;

Servo s1, s2, s3;

typedef struct Position {
  float x;
  float y; 
  float z;
} Position;

typedef struct Joints {
  float t1;
  float t2;
  float t3;
} Joints;

Joints calculateJoint(Position pos) {
  float t1 = atan2(pos.y, pos.x);

  float r2 = pow(pos.x,2) + pow(pos.y,2);
  float R2 = r2 + pow(pos.z,2);

  float t2 = acos(constrain((R2 + L1*L1 - L2*L2)/(2*L1*sqrt(R2)), -1, 1)) - atan2(pos.z, sqrt(r2));
  float t3 = M_PI - acos(constrain((L1*L1 + L2*L2 - R2)/(2*L1*L2), -1, 1));

  Joints result = {t1, t2, t3};
  return result;
}

void standarlise(Joints &joints){
  joints.t1 = (joints.t1 * 180 / M_PI + Joint1Offset);
  joints.t2 = (joints.t2 * 180 / M_PI + Joint2Offset);
  joints.t3 = (joints.t3 * 180 / M_PI + Joint3Offset);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  s1.attach(Joint1Pin);
  s2.attach(Joint2Pin);
  s3.attach(Joint3Pin);

  s1.write(0+Joint1Offset);
  s2.write(90+Joint2Offset);
  s3.write(90+Joint3Offset);

  delay(2000);
  
  Position position = {15,0,0};
  for(int i = 0; i < 5; i++){
    // update position
    position.x += i * 2;
    Joints jointAngles = calculateJoint(position);
    
    // Calibration
    standarlise(jointAngles);

    // Constrain
    jointAngles.t1 = constrain(jointAngles.t1, 0, 90);
    jointAngles.t2 = constrain(jointAngles.t2, 0, 90);
    jointAngles.t3 = constrain(jointAngles.t3, 0, 90);


    // Write
    s1.write(jointAngles.t1);
    s2.write(jointAngles.t2);
    s3.write(jointAngles.t3);
  
    delay(1000);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
