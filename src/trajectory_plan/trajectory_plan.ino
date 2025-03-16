#include <Servo.h>
#include <math.h>

const int Joint1Pin = 2;
const int Joint2Pin = 3;
const int Joint3Pin = 4;

const int Joint1Offset = 7;
const int Joint2Offset = 14;
const int Joint3Offset = -12;

Servo s1, s2, s3;

typedef struct Joints {
  float t1;
  float t2;
  float t3;
} Joints;

void setup() {

}

void loop() {

  //update per 0.1s
  delay(100);
}
