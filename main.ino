#include <Servo.h>
#include <math.h>

#define Y 10 // TODO: determine the default height
#define ll 20; // length of the lower section of the crane. The variable name is "LL", but lower case. TODO: determine the actual length of the section
#define lh 30; // length of the higher section of the crane. The variable name is "LH", but lower case. TODO: determine the actual length of the section

Servo craneBase;
int craneBasePos = 0; // TODO: calculate
int craneBasePin = A2;
Servo craneLowerJoint;
int craneLowerJointPos = 0; // TODO: calculate
int craneLowerJointPin = A3;
Servo craneMiddleJoint;
int craneMiddleJointPos = 0; // TODO: calculate
int craneMiddleJointPin = A4;
Servo craneClaw;
int craneClawPos = 0; // crane is not clenched by default
int craneClawPin = A5;

int craneXCoord = 10; // TODO: calculate the right starting position in terms of x, y is always constant

Servo wheelDirection;
int wheelMotorPin = 3;

Servo cameraX;
int cameraXPos = 90;
int cameraXPin = A0;
Servo cameraY;
int cameraYPos = 90;
int cameraYPin = A1;

double distance(double x, double y) {
  return sqrt(x*x + y*y);
}

double radToDeg(double rad) {
  return rad * 57296 / 1000;
}

void setup() {
  Serial1.begin(115200); // ethernet cable or WiFi

  // crane stuff
  // attach servos to respective pins
  craneBase.attach(craneBasePin);
  craneLowerJoint.attach(craneLowerJointPin);
  craneMiddleJoint.attach(craneMiddleJointPin);
  craneClaw.attach(craneClawPin);
  // TODO: resting position of the crane
  craneClaw.write(craneClawPos);
  
  // camera stuff
  // attach servos to respective pins
  cameraX.attach(cameraXPin);
  cameraY.attach(cameraYPin);
  // move camera servos to middle position
  cameraX.write(cameraXPos);
  cameraY.write(cameraYPos);

  // clear the input buffer - read the input while there is some, the input is ignored
  while (Serial1.available()) { 
     Serial1.read();
  }
}

void loop() {
  // while there is input available, consume it
  if(Serial1.available() > 0) {
    char command = Serial1.read(); // the input

    switch(command) {
      // camera navigation(i - up, j - left, k - down, l - right, m - start position)
      case 'i':
        cameraYPos = cameraYPos + 5;
        cameraY.write(cameraYPos);
        break;
      case 'j':
        cameraXPos = cameraXPos + 5;
        cameraX.write(cameraXPos);
        break;
      case 'k':
        cameraYPos = cameraYPos - 5;
        cameraY.write(cameraYPos);
        break;
      case 'l':
        cameraXPos = cameraXPos - 5;
        cameraX.write(cameraXPos);
        break;
      // rotate the base of the crane(f - clockwise, h - counterclockwise) TODO: refactor respectivly
      case 'f':
        craneBasePos = craneBasePos + 5;
        craneBase.write(craneBasePos);
        break;
      case 'h':
        craneBasePos = craneBasePos - 5;
        craneBase.write(craneBasePos);
        break;
      // navigate crane(t - forward, g - back)
      case 't':
        craneXCoord = craneXCoord + 3; // TODO: determine the right number, insted of 3
        double dist = distance(craneXCoord, Y);
        double alpha = acos( (sq(ll) + sq(lh) - sq(dist)) / ( 2 * ll * lh ) );
        alpha = radToDeg(alpha);
        double gamma = 180 - alpha; // the value of the middle joint angle in degrees
        double theta1 = atan(Y, craneXCoord);
        double theta2 = acos( (sq(dist) + sq(ll) - sq(lh)) / ( 2 * dist * ll ) );
        theta1 = radToDeg(theta1);
        theta2 = radToDeg(theta2);
        double theta = theta1 + theta2; // the value of the lower joint angle in degrees
        // first move the middle joint in order to prevent the tip of the crane hitting the ground
        craneMiddleJointPos = gamma;
        craneMiddleJoint.write(craneMiddleJointPos);
        craneLowerJointPos = theta;
        craneLowerJoint.write(craneLowerJointPos); 
        break;
      case 'g':
        craneXCoord = craneXCoord - 3; // TODO: determine the right number, insted of 3
        double dist = distance(craneXCoord, Y);
        double alpha = acos( (sq(ll) + sq(lh) - sq(dist)) / ( 2 * ll * lh ) );
        alpha = radToDeg(alpha);
        double gamma = 180 - alpha; // the value of the middle joint angle in degrees
        double theta1 = atan(Y, craneXCoord);
        double theta2 = acos( (sq(dist) + sq(ll) - sq(lh)) / ( 2 * dist * ll ) );
        theta1 = radToDeg(theta1);
        theta2 = radToDeg(theta2);
        double theta = theta1 + theta2; // the value of the lower joint angle in degrees
        // first move the lower joint in order to prevent the tip of the crane hitting the ground
        craneLowerJointPos = theta;
        craneLowerJoint.write(craneLowerJointPos); 
        craneMiddleJointPos = gamma;
        craneMiddleJoint.write(craneMiddleJointPos);
        break;
      // return the crane to the resting position - r
      case 'r':
        craneBasePos = 0; // TODO: to be calculated
        craneBase.write(craneBasePos); 
        craneLowerJointPos = 0; // TODO: to be calculated
        craneLowerJoint.write(craneLowerJointPos); 
        craneMiddleJointPos = 0; // TODO: to be calculated
        craneMiddleJoint.write(craneMiddleJointPos);
        craneClawPos = 0;
        craneClaw.write(craneClawPos);
        break;
      
    }
  }

}
