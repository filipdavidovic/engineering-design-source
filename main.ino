#include <Servo.h>
#include <math.h>

#define Y 10 // TODO: determine the default height
#define ll 20 // length of the lower section of the crane. The variable name is "LL", but lower case. TODO: determine the actual length of the section
#define lh 30 // length of the higher section of the crane. The variable name is "LH", but lower case. TODO: determine the actual length of the section
#define gearRatio 4 // the ratio between the gear of continous servos and the gears on the crane, the ratio is 1:4=gearServo:gearCrane. TODO: check the actual ratio
#define timeSampleServo 150

typedef int bool;
#define true 1
#define false 0

bool craneManual = false;
bool motorRunning = false; // variable that determines whether the DC wheel motors should stop, based on the input and the current state (true if the motors were running in the last iteration of the loop() function)
bool wheelsTurning = false; // variable that determines whether the wheel servo should return to the forward position, based on the input and the current state (true if the vehicle was turning in the last iteration of the loop() function)
bool gear = true; // vehicle gear for manouvering, there exist two gears, slow and fast

const int ultrasonicEchoPin = 9;
const int ultrasonicTrigPin = 10;

Servo craneBase;
int craneBasePos = 0; // TODO: calculate
const int craneBaseStop = 97; // TODO: check the exact value
int craneBasePin = A0;
Servo craneLowerJoint;
int craneLowerJointPos = 0; // TODO: calculate
const int craneLowerJointStop = 96; // TODO: check the exact value
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
int cameraXPos = 63;
int cameraXPin = A4;
Servo cameraY;
int cameraYPos = 0;
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
  // Prints the distance on the Serial1 Monitor
  Serial1.print("Distance: ");
  Serial1.println(ultrasonicDistance);
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
      Serial1.println("moving the middle joint forward");
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop+4);
      delay(((gamma - craneMiddleJointPos) / 5) * timeSampleServo);
      
      Serial1.println(((gamma - craneMiddleJointPos) / 5) * timeSampleServo);
      
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
      craneMiddleJointPos = gamma;
    } else if(craneMiddleJointPos - gamma > 0) {
      Serial1.println("moving the middle joint backward");
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop - 4);
      delay(((craneMiddleJointPos - gamma) / 5) * timeSampleServo);

      Serial1.println(((gamma - craneMiddleJointPos) / 5) * timeSampleServo);
      
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
      craneMiddleJointPos = gamma;
    }

    if(craneLowerJointPos - theta < 0) {
      Serial1.println("moving the lower joint forward");
      moveContinuousServo(craneLowerJoint, craneLowerJointStop + 4);
      delay(((theta - craneLowerJointPos) / 5) * timeSampleServo);
      moveContinuousServo(craneLowerJoint, craneLowerJointStop);
      craneLowerJointPos = theta;
    } else if(craneLowerJointPos - theta > 0) {
      Serial1.println("moving the lower joint backward");
      moveContinuousServo(craneLowerJoint, craneLowerJointStop - 4);
      delay(((craneLowerJointPos - theta) / 5) * timeSampleServo);
      moveContinuousServo(craneLowerJoint, craneLowerJointStop);
      craneLowerJointPos = theta;
    }
  } else {
    // first move the lower joint in order to prevent the tip of the crane hitting the ground
    if(craneLowerJointPos - theta < 0) {
      Serial1.println("moving the lower joint forward");
      moveContinuousServo(craneLowerJoint, craneLowerJointStop + 4);
      delay(((theta - craneLowerJointPos) / 5) * timeSampleServo);
      moveContinuousServo(craneLowerJoint, craneLowerJointStop);
      craneLowerJointPos = theta;
    } else if(craneLowerJointPos - theta > 0) {
      Serial1.println("moving the lower joint backward");
      moveContinuousServo(craneLowerJoint, craneLowerJointStop - 4);
      delay(((craneLowerJointPos - theta) / 5) * timeSampleServo);
      moveContinuousServo(craneLowerJoint, craneLowerJointStop);
      craneLowerJointPos = theta;
    }

    if(craneMiddleJointPos - gamma < 0) {
      Serial1.println("moving the middle joint forward");
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop+4);
      delay(((gamma - craneMiddleJointPos) / 5) * timeSampleServo);
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
      craneMiddleJointPos = gamma;
    } else if(craneMiddleJointPos - gamma > 0) {
      Serial1.println("moving the middle joint backward");
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop - 4);
      delay(((craneMiddleJointPos - gamma) / 5) * timeSampleServo);
      moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
      craneMiddleJointPos = gamma;
    }
  }
}

void setup() {
  Serial1.begin(115200); // ethernet cable or WiFi (115200)

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
  craneBase.write(craneBaseStop);
  craneLowerJoint.attach(craneLowerJointPin);
  craneLowerJoint.write(craneLowerJointStop);
  craneMiddleJoint.attach(craneMiddleJointPin);
  craneMiddleJoint.write(craneMiddleJointStop);
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
  // while (Serial1.available()) {
  //    Serial1.read();
  // }
  Serial1.flush();
}

void loop() {
  // read the ultrasonic sensor on every loop() iteration.
//  readUltrasonic();
  
  // while there is input available, consume it
  if(Serial1.available() > 0) {
    char command = Serial1.read(); // the input

    if(motorRunning && command != 'w' && command != 's') {
      wheelIterationCounter++;
      if(wheelIterationCounter > iterationMax) {
        wheelIterationCounter = 0;
      	motorRunning = false;
      	analogWrite(wheelLeftMotorPinOne, 0);
        analogWrite(wheelLeftMotorPinTwo, 0);
        analogWrite(wheelRightMotorPinOne, 0);
        analogWrite(wheelRightMotorPinTwo, 0);
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
        cameraYPos = cameraYPos + 5;
      	if(cameraYPos > 180) {
        	cameraYPos = 180;
      	}
        Serial1.print("i: ");
        Serial1.println(cameraYPos);
        cameraY.write(cameraYPos);
        break;
      case 'j':
        cameraXPos = cameraXPos + 5;
      	if(cameraXPos > 180) {
        	cameraXPos = 180;
      	}
        Serial1.print("j: ");
        Serial1.println(cameraXPos);
        cameraX.write(cameraXPos);
        break;
      case 'k':
        cameraYPos = cameraYPos - 5;
      	if(cameraYPos < 0) {
        	cameraYPos = 0;
      	}
        Serial1.print("k: ");
        Serial1.println(cameraYPos);
        cameraY.write(cameraYPos);
        break;
      case 'l':
        cameraXPos = cameraXPos - 5;
      	if(cameraXPos < 0) {
        	cameraXPos = 0;
      	}
        Serial1.print("l: ");
        Serial1.println(cameraXPos);
        cameraX.write(cameraXPos);
        break;
      case 'm':
        Serial1.print("m");
        cameraXPos = 63;
        cameraYPos = 0;
        cameraX.write(cameraXPos);
        cameraY.write(cameraYPos);
        break;
      // rotate the base of the crane(f - clockwise, h - counterclockwise), manual control mode does not change anything. TODO: refactor respectivly
      case 'f':
        Serial1.print("f");
        if(craneBasePos > 0) {
          craneBase.write(craneBaseStop-4);
          delay(timeSampleServo);
          craneBase.write(craneBaseStop);
          craneBasePos -= 5;
        }
        if(craneBasePos < 0) {
          craneBasePos = 0;
        }
        break;
      case 'h':
        Serial1.print("h");
        if(craneBasePos < 180) {
          craneBase.write(craneBaseStop+4);
          delay(timeSampleServo);
          craneBase.write(craneBaseStop);
          craneBasePos += 5;
        }
        if(craneBasePos > 360) {
          craneBasePos = 360;
        }
        break;
      // navigate crane(t - forward, g - back), in case of manual control, move the lower joint up (t) or down (g)
      case 't':
        Serial1.print("t");
        if(craneManual) {
          moveContinuousServo(craneLowerJoint, craneLowerJointStop + 4);
          delay(timeSampleServo);
          moveContinuousServo(craneLowerJoint, craneLowerJointStop);
          craneLowerJointPos += 5; // TODO: test the exact value
          calculateCraneXAndY();
        } else {
          moveCraneAutomatic(3, true); // TODO: determine the right number, insted of 3
        }
        break;
      case 'g':
        Serial1.print("g");
        if(craneManual) {
          if(craneLowerJointPos > 0) {
            moveContinuousServo(craneLowerJoint, craneLowerJointStop - 4);
            delay(timeSampleServo);
            moveContinuousServo(craneLowerJoint, craneLowerJointStop);
            craneLowerJointPos -= 5; // TODO: test the exact value
          }
          if(craneLowerJointPos < 0) {
            craneLowerJointPos = 0;
          }
          calculateCraneXAndY();
        } else {
          moveCraneAutomatic(-3, false); // TODO: determine the right number, insted of 3
        }
        break;
      // used to move the middle joint of the crane in manual mode
      case 'v':
        Serial1.print("v");
        if(craneManual) {
          if(craneMiddleJointPos < 180) {
            moveContinuousServo(craneMiddleJoint, craneMiddleJointStop + 4);
            delay(timeSampleServo);
            moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
            craneMiddleJointPos += 5; // TODO: test the exact value
          }
          if(craneMiddleJointPos > 180) {
            craneMiddleJointPos = 180;
          }
        }
        break;
      case 'b':
        Serial1.print("b");
        if(craneManual) {
          if(craneMiddleJointPos > 0) {
            moveContinuousServo(craneMiddleJoint, craneMiddleJointStop - 4);
            delay(timeSampleServo);
            moveContinuousServo(craneMiddleJoint, craneMiddleJointStop);
            craneMiddleJointPos -= 5; // TODO: test the exact value
          }
          if(craneMiddleJointPos < 0) {
            craneMiddleJointPos = 0;
          }
        }
        break;
      // return the crane to the resting position - r
      case 'r':
        Serial1.print("r");
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
        craneManual = craneManual ^ true;
        if(craneManual) {
          Serial1.print("You have the manual control of the crane.");
        } else {
          Serial1.print("Crane is controlled automatically.");
        }
        calculateCraneXAndY();
        break;
      // wheel stuff, controls are: w - forwards (turn the motor in positive direction), s - backwards (move the motor in negative direction), a - left (in combination with w/s), d - right (in combination with w/s)
      case 'w':
      	wheelIterationCounter = 0;
      	motorRunning = true;
        if(gear) {
          if(moveDirection < 0) {
            motorRunning = false;
            analogWrite(wheelLeftMotorPinOne, 0);
            analogWrite(wheelLeftMotorPinTwo, 0);
            analogWrite(wheelRightMotorPinOne, 0);
            analogWrite(wheelRightMotorPinTwo, 0);
          } else {
            motorRunning = true;
            analogWrite(wheelLeftMotorPinOne, 50);
            analogWrite(wheelLeftMotorPinTwo, 0);
            analogWrite(wheelRightMotorPinOne, 50);
            analogWrite(wheelRightMotorPinTwo, 0);
          }
        } else {
          if(moveDirection < 0) {
            motorRunning = false;
            analogWrite(wheelLeftMotorPinOne, 0);
            analogWrite(wheelLeftMotorPinTwo, 0);
            analogWrite(wheelRightMotorPinOne, 0);
            analogWrite(wheelRightMotorPinTwo, 0);
          } else {
            motorRunning = true;
            analogWrite(wheelLeftMotorPinOne, 255);
            analogWrite(wheelLeftMotorPinTwo, 0);
            analogWrite(wheelRightMotorPinOne, 255);
            analogWrite(wheelRightMotorPinTwo, 0);
          }
        }        
        moveDirection = 1;
      	break;
      case 's':
      	wheelIterationCounter = 0;
        if(gear) {
          if(moveDirection > 0) {
            motorRunning = false;
            analogWrite(wheelLeftMotorPinOne, 0);
            analogWrite(wheelLeftMotorPinTwo, 0);
            analogWrite(wheelRightMotorPinOne, 0);
            analogWrite(wheelRightMotorPinTwo, 0);
          } else {
            motorRunning = true;
            analogWrite(wheelLeftMotorPinOne, 0);
            analogWrite(wheelLeftMotorPinTwo, 50);
            analogWrite(wheelRightMotorPinOne, 0);
            analogWrite(wheelRightMotorPinTwo, 50);
          }
        } else {
          if(moveDirection > 0) {
            motorRunning = false;
            analogWrite(wheelLeftMotorPinOne, 0);
            analogWrite(wheelLeftMotorPinTwo, 0);
            analogWrite(wheelRightMotorPinOne, 0);
            analogWrite(wheelRightMotorPinTwo, 0);
          } else {
            motorRunning = true;
            analogWrite(wheelLeftMotorPinOne, 0);
            analogWrite(wheelLeftMotorPinTwo, 255);
            analogWrite(wheelRightMotorPinOne, 0);
            analogWrite(wheelRightMotorPinTwo, 255);
          }
        }
        moveDirection = -1;
      	break;
      case 'a':
      	wheelDirectionIterationCounter = 0;
      	wheelsTurning = true;
        wheelDirectionPos = wheelDirectionPos + 10;
      	if(wheelDirectionPos > 180) {
        	wheelDirectionPos = 180;
      	}
        wheelDirection.write(wheelDirectionPos);
        break;
      case 'd':
      	wheelDirectionIterationCounter = 0;
      	wheelsTurning = true;
      	wheelDirectionPos = wheelDirectionPos - 10;
      	if(wheelDirectionPos < 0) {
        	wheelDirectionPos = 0;
      	}
        wheelDirection.write(wheelDirectionPos);
      	break;
      // switch motor gears
      case 'z':
        gear = gear ^ true;
        if(gear) {
          Serial1.println("The vehicle is in the slow gear operation mode.");
        } else {
          Serial1.println("The vehicle is in the fast gear operation mode.");
        }
        break;
    }
  } else if(motorRunning) {
    wheelIterationCounter++;
//    Serial1.println(wheelIterationCounter);
    delay(1);
    if(wheelIterationCounter > iterationMax) {
      wheelIterationCounter = 0;
      motorRunning = false;
	    analogWrite(wheelLeftMotorPinOne, 0);
      analogWrite(wheelLeftMotorPinTwo, 0);
      analogWrite(wheelRightMotorPinOne, 0);
      analogWrite(wheelRightMotorPinTwo, 0);
      moveDirection = 0;
    }
  } else if(wheelsTurning) {
    wheelDirectionIterationCounter++;
    delay(1);
    if(wheelDirectionIterationCounter > iterationMax) {
      wheelDirectionIterationCounter = 0;
      wheelsTurning = false;
      wheelDirectionPos = 90;
	  wheelDirection.write(wheelDirectionPos);
    }
  }

}
