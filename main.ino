#include <Servo.h>
#include <math.h>

#define Y 10 // TODO: determine the default height
#define ll 20 // length of the lower section of the crane. The variable name is "LL", but lower case. TODO: determine the actual length of the section
#define lh 30 // length of the higher section of the crane. The variable name is "LH", but lower case. TODO: determine the actual length of the section
#define gearRatio 4 // the ratio between the gear of continous servos and the gears on the crane, the ratio is 1:4=gearServo:gearCrane. TODO: check the actual ratio

typedef int bool;
#define true 1
#define false 0

bool craneManual = false;
bool motorRunning = false; // variable that determines whether the DC wheel motors should stop, based on the input and the current state (true if the motors were running in the last iteration of the loop() function)
bool wheelsTurning = false; // variable that determines whether the wheel servo should return to the forward position, based on the input and the current state (true if the vehicle was turning in the last iteration of the loop() function)

Servo craneBase;
int craneBasePos = 0; // TODO: calculate
int craneBasePin = A0;
Servo craneLowerJoint;
int craneLowerJointPos = 0; // TODO: calculate
int craneLowerJointPin = A1;
Servo craneMiddleJoint;
int craneMiddleJointPos = 0; // TODO: calculate
int craneMiddleJointPin = A2;
Servo craneClaw;
int craneClawPos = 0; // crane is not clenched by default
int craneClawPin = A3;

float craneXCoord = 10; // TODO: calculate the right starting position in terms of x, y is always constant
float craneYCoord = Y; // at initialization, set the starting height of the crane to be Y

Servo wheelDirection;
int wheelDirectionPin = A3;
int wheelDirectionPos = 90;
int wheelDirectionIterationCounter = 0;
int wheelMotorPin = 3;
int wheelIterationCounter = 0;
const int iterationMax = 50;

Servo cameraX;
int cameraXPos = 90;
int cameraXPin = A4;
Servo cameraY;
int cameraYPos = 90;
int cameraYPin = A5;

double distance(double x, double y) {
  return sqrt(x*x + y*y);
}

double radToDeg(double rad) {
  return rad * 57296 / 1000;
}

void calculateCraneXAndY() {
  craneXCoord = cos(craneLowerJointPos) * ll + cos(craneLowerJointPos + craneMiddleJointPos) * lh;

  if(craneManual) {
    craneYCoord = sin(craneLowerJointPos) * ll + sin(craneLowerJointPos + craneMiddleJointPos) * lh;
  } else {
    craneYCoord = Y;
  }
}

void moveCraneAutomatic(int xDelta, bool forward) {
  craneXCoord = craneXCoord + xDelta;
  double dist = distance(craneXCoord, Y);
  double alpha = acos( (sq(ll) + sq(lh) - sq(dist)) / ( 2 * ll * lh ) );
  alpha = radToDeg(alpha);
  double gamma = 180 - alpha; // the value of the middle joint angle in degrees
  double theta1 = atan2(Y, craneXCoord);
  double theta2 = acos( (sq(dist) + sq(ll) - sq(lh)) / ( 2 * dist * ll ) );
  theta1 = radToDeg(theta1);
  theta2 = radToDeg(theta2);
  double theta = theta1 + theta2; // the value of the lower joint angle in degrees

  if(forward) {
    // first move the middle joint in order to prevent the tip of the crane hitting the ground
    craneMiddleJointPos = gamma;
    if(craneMiddleJointPos > 180) {
    	craneMiddleJointPos = 180;
    }
    craneMiddleJoint.write(craneMiddleJointPos*gearRatio);
    craneLowerJointPos = theta;
    if(craneLowerJointPos > 180) {
    	craneLowerJointPos = 180;
    }
    craneLowerJoint.write(craneLowerJointPos*gearRatio);
  } else {
    // first move the lower joint in order to prevent the tip of the crane hitting the ground
    craneLowerJointPos = theta;
    if(craneLowerJointPos > 180) {
    	craneLowerJointPos = 180;
    }
    craneLowerJoint.write(craneLowerJointPos*gearRatio);
    craneMiddleJointPos = gamma;
    if(craneMiddleJointPos > 180) {
    	craneMiddleJointPos = 180;
    }
    craneMiddleJoint.write(craneMiddleJointPos*gearRatio);
  }
}

void setup() {
  Serial.begin(9600); // ethernet cable or WiFi (115200)

  // DC motor wheel stuff
  pinMode(wheelMotorPin, OUTPUT);
  wheelDirection.attach(wheelDirectionPin);
  wheelDirection.write(wheelDirectionPos);

  // crane stuff
  // attach servos to respective pins
  craneBase.attach(craneBasePin);
  craneLowerJoint.attach(craneLowerJointPin);
  craneMiddleJoint.attach(craneMiddleJointPin);
  // craneClaw.attach(craneClawPin); // TODO: connect the claw to the PWM pin, this pin is taken by the wheel servo
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
  // while (Serial.available()) {
  //    Serial.read();
  // }
  Serial.flush();
}

void loop() {
  // while there is input available, consume it
  if(Serial.available() > 0) {
    char command = Serial.read(); // the input

    if(motorRunning && command != 'w' && command != 's') {
      wheelIterationCounter++;
      if(wheelIterationCounter > iterationMax) {
        wheelIterationCounter = 0;
      	motorRunning = false;
      	analogWrite(wheelMotorPin, 0);
      }
    } else if(wheelsTurning && command != 'a' && command != 'd') {
      wheelDirectionIterationCounter++;
      if(wheelDirectionIterationCounter > iterationMax) {
        wheelDirectionIterationCounter = 0;
        wheelsTurning = false;
        wheelDirectionPos = 90;
        wheelDirection.write(wheelDirectionPos);
      }
    }

    switch(command) {
      // camera navigation(i - up, j - left, k - down, l - right, m - start position)
      case 'i':
        Serial.print("i");
        cameraYPos = cameraYPos + 5;
      	if(cameraYPos > 180) {
        	cameraYPos = 180;
      	}
        cameraY.write(cameraYPos);
        break;
      case 'j':
        Serial.print("j");
        cameraXPos = cameraXPos + 5;
      	if(cameraXPos > 180) {
        	cameraXPos = 180;
      	}
        cameraX.write(cameraXPos);
        break;
      case 'k':
        Serial.print("k");
        cameraYPos = cameraYPos - 5;
      	if(cameraYPos < 0) {
        	cameraYPos = 0;
      	}
        cameraY.write(cameraYPos);
        break;
      case 'l':
        Serial.print("l");
        cameraXPos = cameraXPos - 5;
      	if(cameraXPos < 0) {
        	cameraXPos = 0;
      	}
        cameraX.write(cameraXPos);
        break;
      case 'm':
        Serial.print("m");
        cameraXPos = 90;
        cameraYPos = 90;
        cameraX.write(cameraXPos);
        cameraY.write(cameraYPos);
        break;
      // rotate the base of the crane(f - clockwise, h - counterclockwise), manual control mode does not change anything. TODO: refactor respectivly
      case 'f':
        Serial.print("f");
        craneBasePos = craneBasePos + 5;
      	if(craneBasePos > 180) {
        	craneBasePos = 180;
      	}
        craneBase.write(craneBasePos);
        break;
      case 'h':
        Serial.print("h");
        craneBasePos = craneBasePos - 5;
      	if(craneBasePos < 0) {
        	craneBasePos = 0;
      	}
        craneBase.write(craneBasePos);
        break;
      // navigate crane(t - forward, g - back), in case of manual control, move the lower joint up (t) or down (g)
      case 't':
        Serial.print("t");
        if(craneManual) {
          craneLowerJointPos += 3; // TODO: to be calculated how much to move the servo
          if(craneLowerJointPos > 180) {
        	craneLowerJointPos = 180;
      	  }
          craneLowerJoint.write(craneLowerJointPos*gearRatio);
          calculateCraneXAndY();
        } else {
          moveCraneAutomatic(3, true); // TODO: determine the right number, insted of 3
        }
        break;
      case 'g':
        Serial.print("g");
        if(craneManual) {
          craneLowerJointPos -= 3; // TODO: to be calculated how much to move the servo
          if(craneLowerJointPos < 0) {
        	craneLowerJointPos = 0;
      	  }
          craneLowerJoint.write(craneLowerJointPos*gearRatio);
          calculateCraneXAndY();
        } else {
          moveCraneAutomatic(-3, false); // TODO: determine the right number, insted of 3
        }
        break;
      // used to move the middle joint of the crane in manual mode
      case 'v':
        Serial.print("v");
        if(craneManual) {
          craneMiddleJointPos += 3; // TODO: to be calculated how much to move the servo
          if(craneMiddleJointPos > 180) {
        	   craneMiddleJointPos = 180;
      	  }
          craneMiddleJoint.write(craneMiddleJointPos*gearRatio);
        }
        break;
      case 'b':
        Serial.print("b");
        if(craneManual) {
          craneMiddleJointPos -= 3; // TODO: to be calculated how much to move the servo
          if(craneMiddleJointPos < 0) {
        	   craneMiddleJointPos = 0;
      	  }
          craneMiddleJoint.write(craneMiddleJointPos*gearRatio);
        }
        break;
      // return the crane to the resting position - r
      case 'r':
        Serial.print("r");
        craneBasePos = 0; // TODO: to be calculated
        craneBase.write(craneBasePos);
        craneLowerJointPos = 0; // TODO: to be calculated
        craneLowerJoint.write(craneLowerJointPos);
        craneMiddleJointPos = 0; // TODO: to be calculated
        craneMiddleJoint.write(craneMiddleJointPos);
        craneClawPos = 0;
        craneClaw.write(craneClawPos);
        break;
      // toggle crane manual and crane automatic modes
      case 'y':
        Serial.print("y");
        craneManual = craneManual ^ true;
        calculateCraneXAndY();
        break;
      // wheel stuff, controls are: w - forwards (turn the motor in positive direction), s - backwards (move the motor in negative direction), a - left (in combination with w/s), d - right (in combination with w/s)
      case 'w':
      	wheelIterationCounter = 0;
      	motorRunning = true;
      	analogWrite(wheelMotorPin, 255);
      	break;
      case 's':
      	wheelIterationCounter = 0;
      	motorRunning = true;
      	analogWrite(wheelMotorPin, 255);
      	break;
      case 'a':
      	wheelDirectionIterationCounter = 0;
      	wheelsTurning = true;
        wheelDirectionPos = wheelDirectionPos + 5;
      	if(wheelDirectionPos > 180) {
        	wheelDirectionPos = 180;
      	}
        wheelDirection.write(wheelDirectionPos);
        break;
      case 'd':
      	wheelDirectionIterationCounter = 0;
      	wheelsTurning = true;
      	wheelDirectionPos = wheelDirectionPos - 5;
      	if(wheelDirectionPos < 0) {
        	wheelDirectionPos = 0;
      	}
        wheelDirection.write(wheelDirectionPos);
      	break;
    }
  } else if(motorRunning) {
    wheelIterationCounter++;
    Serial.println(wheelIterationCounter);
    if(wheelIterationCounter > iterationMax) {
      wheelIterationCounter = 0;
      motorRunning = false;
	  analogWrite(wheelMotorPin, 0);
    }
  } else if(wheelsTurning) {
    wheelDirectionIterationCounter++;
    Serial.println(wheelDirectionIterationCounter);
    if(wheelDirectionIterationCounter > iterationMax) {
      wheelDirectionIterationCounter = 0;
      wheelsTurning = false;
      wheelDirectionPos = 90;
	  wheelDirection.write(wheelDirectionPos);
    }
  }

}
