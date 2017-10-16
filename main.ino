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

const int ultrasonicEchoPin = 9;
const int ultrasonicTrigPin = 10;

Servo craneBase;
int craneBasePos = 0; // TODO: calculate
const int craneBaseStop = 97; // TODO: check the exact value
int craneBasePin = A0;
Servo craneLowerJoint;
int craneLowerJointPos = 0; // TODO: calculate
const int craneLowerJointStop = 97; // TODO: check the exact value
int craneLowerJointPin = A1;
Servo craneMiddleJoint;
int craneMiddleJointPos = 0; // TODO: calculate
const int craneMiddleJointStop = 97; // TODO: check the exact value
int craneMiddleJointPin = A2;
Servo craneClaw;
int craneClawPos = 0; // crane is not clenched by default
int craneClawPin = A3;

float craneXCoord = 10; // TODO: calculate the right starting position in terms of x, y is always constant
float craneYCoord = Y; // at initialization, set the starting height of the crane to be Y

Servo wheelDirection;
int wheelDirectionPin = A3;
int wheelDirectionPos = 90;

int wheelLeftMotorPinOne = 3;
int wheelLeftMotorPinTwo = 5;
int wheelRightMotorPinOne = 6;
int wheelRightMotorPinTwo = 11;

int wheelIterationCounter = 0;
int wheelDirectionIterationCounter = 0;
const int iterationMax = 1000;

int moveDirection = 0;

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

void readUltrasonic() {
  // Clears the trigPin
  digitalWrite(ultrasonicTrigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(ultrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  int ultrasonicDuration = pulseIn(ultrasonicEchoPin, HIGH);
  
  // Calculating the distance
  double ultrasonicDistance = ultrasonicDuration*0.034/2; // in cm
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(ultrasonicDistance);
}

void moveContinuousServo(Servo servo, int toSpeed) {
  int current = servo.read();
  if(current < toSpeed) {
    for(int i = current + 1; i <= toSpeed; i++) {
      servo.write(i);
      delay(15);
    }
  } else if(current > toSpeed) {
    for(int i = current - 1; i >= toSpeed; i--) {
      servo.write(i);
      delay(15);
    }
  }
}

void moveCraneAutomatic(int xDelta, bool forward) {
  craneXCoord = craneXCoord + xDelta;
  double dist = distance(craneXCoord, Y);
  double alpha = acos( (sq(ll) + sq(lh) - sq(dist)) / ( 2 * ll * lh ) );
  alpha = radToDeg(alpha);
  int gamma = (int) 180 - alpha; // the value of the middle joint angle in degrees
  double theta1 = atan2(Y, craneXCoord);
  double theta2 = acos( (sq(dist) + sq(ll) - sq(lh)) / ( 2 * dist * ll ) );
  theta1 = radToDeg(theta1);
  theta2 = radToDeg(theta2);
  int theta = (int) theta1 + theta2; // the value of the lower joint angle in degrees

  if(forward) {
    // first move the middle joint in order to prevent the tip of the crane hitting the ground
    if(craneMiddleJointPos - gamma < 0) {
      moveContinuousServo(craneMiddleJoint, 180);
      delay(((gamma - craneMiddleJointPos) % 3)*15);
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
      craneMiddleJointPos = gamma;
    } else if(craneMiddleJointPos - gamma > 0) {
      moveContinuousServo(craneMiddleJoint, 0);
      delay(((craneMiddleJointPos - gamma) % 3)*15);
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
      craneMiddleJointPos = gamma;
    }

    if(craneLowerJointPos - theta < 0) {
      moveContinuousServo(craneLowerJoint, 180);
      delay(((theta - craneLowerJointPos) % 3) * 15);
      moveContinuousServo(craneLowerJoint, craneLowerJointStop);
      craneLowerJointPos = theta;
    } else if(craneLowerJointPos - theta > 0) {
      moveContinuousServo(craneLowerJoint, 0);
      delay(((craneLowerJointPos - theta) % 3)*15);
      moveContinuousServo(craneLowerJoint, craneLowerJointStop);
      craneLowerJointPos = theta;
    }
  } else {
    // first move the lower joint in order to prevent the tip of the crane hitting the ground
    if(craneMiddleJointPos - gamma < 0) {
      moveContinuousServo(craneMiddleJoint, 180);
      delay(((gamma - craneMiddleJointPos) % 3)*15);
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
      craneMiddleJointPos = gamma;
    } else if(craneMiddleJointPos - gamma > 0) {
      moveContinuousServo(craneMiddleJoint, 0);
      delay(((craneMiddleJointPos - gamma) % 3)*15);
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
      craneMiddleJointPos = gamma;
    }

    if(craneLowerJointPos - theta < 0) {
      moveContinuousServo(craneLowerJoint, 180);
      delay(((theta - craneLowerJointPos) % 3) * 15);
      moveContinuousServo(craneLowerJoint, craneLowerJointStop);
      craneLowerJointPos = theta;
    } else if(craneLowerJointPos - theta > 0) {
      moveContinuousServo(craneLowerJoint, 0);
      delay(((craneLowerJointPos - theta) % 3)*15);
      moveContinuousServo(craneLowerJoint, craneLowerJointStop);
      craneLowerJointPos = theta;
    }
  }
}

void setup() {
  Serial.begin(9600); // ethernet cable or WiFi (115200)

  // ultrasonic sensor stuff
  pinMode(ultrasonicTrigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(ultrasonicEchoPin, INPUT); // Sets the echoPin as an Input

  // DC motor wheel stuff
  pinMode(wheelLeftMotorPinOne, OUTPUT);
  pinMode(wheelLeftMotorPinTwo, OUTPUT);
  pinMode(wheelRightMotorPinOne, OUTPUT);
  pinMode(wheelRightMotorPinTwo, OUTPUT);
  
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
  // read the ultrasonic sensor on every loop() iteration.
//  readUltrasonic();
  
  // while there is input available, consume it
  if(Serial.available() > 0) {
    char command = Serial.read(); // the input

    if(motorRunning && command != 'w' && command != 's') {
      wheelIterationCounter++;
      if(wheelIterationCounter > iterationMax) {
        wheelIterationCounter = 0;
      	motorRunning = false;
      	digitalWrite(wheelLeftMotorPinOne, LOW);
        digitalWrite(wheelLeftMotorPinTwo, LOW);
        digitalWrite(wheelRightMotorPinOne, LOW);
        digitalWrite(wheelRightMotorPinTwo, LOW);
        moveDirection = 0;
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
        moveContinuousServo(craneBase, 180);
        delay(15);
        moveContinuousServo(craneBase, craneBaseStop);
        craneBasePos -= 3;
        break;
      case 'h':
        Serial.print("h");
        moveContinuousServo(craneBase, 0);
        delay(15);
        moveContinuousServo(craneBase, craneBaseStop);
        craneBasePos -= 3;
        break;
      // navigate crane(t - forward, g - back), in case of manual control, move the lower joint up (t) or down (g)
      case 't':
        Serial.print("t");
        if(craneManual) {
          moveContinuousServo(craneLowerJoint, 180);
          delay(15);
          moveContinuousServo(craneLowerJoint, craneLowerJointStop);
          craneLowerJointPos += 3; // TODO: test the exact value
          calculateCraneXAndY();
        } else {
          moveCraneAutomatic(3, true); // TODO: determine the right number, insted of 3
        }
        break;
      case 'g':
        Serial.print("g");
        if(craneManual) {
          moveContinuousServo(craneLowerJoint, 0);
          delay(15);
          moveContinuousServo(craneLowerJoint, craneLowerJointStop);
          craneLowerJointPos -= 3; // TODO: test the exact value
          calculateCraneXAndY();
        } else {
          moveCraneAutomatic(-3, false); // TODO: determine the right number, insted of 3
        }
        break;
      // used to move the middle joint of the crane in manual mode
      case 'v':
        Serial.print("v");
        if(craneManual) {
          moveContinuousServo(craneMiddleJoint, 180);
          delay(15);
          moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
          craneMiddleJointPos += 3; // TODO: test the exact value
        }
        break;
      case 'b':
        Serial.print("b");
        if(craneManual) {
          moveContinuousServo(craneMiddleJoint, 0);
          delay(15);
          moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
          craneMiddleJointPos -= 3; // TODO: test the exact value
        }
        break;
      // return the crane to the resting position - r
      case 'r':
        Serial.print("r");
        if(craneBasePos > 0) {
          moveContinuousServo(craneBase, 0);
          delay((craneBasePos % 3) * 15);
          moveContinuousServo(craneBase, craneBaseStop);
          craneBasePos = 0;
        } else if(craneBasePos < 0) {
          moveContinuousServo(craneBase, 180);
          delay((craneBasePos % 3) * 15);
          moveContinuousServo(craneBase, craneBaseStop);
          craneBasePos = 0;
        }
        if(craneLowerJointPos > 0) {
          moveContinuousServo(craneLowerJoint, 180);
          delay((craneLowerJointPos % 3) * 15);
          moveContinuousServo(craneLowerJoint, craneLowerJointStop);
          craneLowerJointPos = 0;
        } else if(craneLowerJointPos > 0) {
          moveContinuousServo(craneLowerJoint, 0);
          delay((craneLowerJointPos % 3) * 15);
          moveContinuousServo(craneLowerJoint, craneLowerJointStop);
          craneLowerJointPos = 0;
        }
        if(craneMiddleJointPos > 0) {
          moveContinuousServo(craneMiddleJoint, 180);
          delay((craneMiddleJointPos % 3) * 15);
          moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
          craneMiddleJointPos = 0;
        } else if(craneMiddleJointPos < 0) {
          moveContinuousServo(craneMiddleJoint, 0);
          delay((craneMiddleJointPos % 3) * 15);
          moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
          craneMiddleJointPos = 0;
        }
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
        if(moveDirection < 0) {
          motorRunning = false;
          digitalWrite(wheelLeftMotorPinOne, LOW);
          digitalWrite(wheelLeftMotorPinTwo, LOW);
          digitalWrite(wheelRightMotorPinOne, LOW);
          digitalWrite(wheelRightMotorPinTwo, LOW);
        } else {
          motorRunning = true;
          Serial.println("going forward");
          digitalWrite(wheelLeftMotorPinOne, HIGH);
          digitalWrite(wheelLeftMotorPinTwo, LOW);
          digitalWrite(wheelRightMotorPinOne, HIGH);
          digitalWrite(wheelRightMotorPinTwo, LOW);
        }
        moveDirection = 1;
      	break;
      case 's':
      	wheelIterationCounter = 0;
        if(moveDirection > 0) {
          motorRunning = false;
          digitalWrite(wheelLeftMotorPinOne, LOW);
          digitalWrite(wheelLeftMotorPinTwo, LOW);
          digitalWrite(wheelRightMotorPinOne, LOW);
          digitalWrite(wheelRightMotorPinTwo, LOW);
        } else {
          motorRunning = true;
          Serial.println("going backwards");
          digitalWrite(wheelLeftMotorPinOne, LOW);
          digitalWrite(wheelLeftMotorPinTwo, HIGH);
          digitalWrite(wheelRightMotorPinOne, LOW);
          digitalWrite(wheelRightMotorPinTwo, HIGH);
        }
        moveDirection = -1;
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
	    digitalWrite(wheelLeftMotorPinOne, LOW);
      digitalWrite(wheelLeftMotorPinTwo, LOW);
      digitalWrite(wheelRightMotorPinOne, LOW);
      digitalWrite(wheelRightMotorPinTwo, LOW);
      moveDirection = 0;
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
