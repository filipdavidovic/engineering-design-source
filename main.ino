#include <Servo.h>
#include <math.h>
#include <Stepper.h>

#define ll 20 // length of the lower section of the crane. The variable name is "LL", but lower case. TODO: determine the actual length of the section
#define lh 30 // length of the higher section of the crane. The variable name is "LH", but lower case. TODO: determine the actual length of the section

#define timeSampleServo 100

#define craneBasePinOne 3 // pin one for the stepper motor mounted on the base of the crane
#define craneBasePinTwo 4 // pin two for the stepper motor mounted on the base of the crane
#define craneBasePinThree 5 // pin three for the stepper motor mounted on the base of the crane
#define craneBasePinFour 6 // pin four for the stepper motor mounted on the base of the crane

typedef int bool;
#define true 1
#define false 0

bool readUltrasonic = false;
bool motorRunning = false; // variable that determines whether the DC wheel motors should stop, based on the input and the current state (true if the motors were running in the last iteration of the loop() function)
bool wheelsTurning = false; // variable that determines whether the wheel servo should return to the forward position, based on the input and the current state (true if the vehicle was turning in the last iteration of the loop() function)
bool gearSlow = true; // vehicle gear for manouvering, there exist two gears, slow and fast

const int ultrasonicEchoPin = 9;
const int ultrasonicTrigPin = 10;

double craneBasePos = 0;

Servo craneLowerJoint;
int craneLowerJointPos = 130; 
int craneLowerJointPin = A1;

Servo craneMiddleJoint;
int craneMiddleJointPos = 90; 
const int craneMiddleJointStop = 90;
const int craneMiddleJointSpeed = 10;
int craneMiddleJointPin = A2;

Servo craneClawRotation;
int craneClawRotationPos = 30;
const int craneClawRotationPin = 12;

Servo craneClawClench;
bool craneClawClenchPos = 0; // crane is not clenched by default
const int craneClawClenchPin = 13;

Servo wheelDirection;
const int wheelDirectionPin = A3;
int wheelDirectionPos = 90;

int wheelLeftMotorPinOne = 2;
int wheelLeftMotorPinTwo = 7;
int wheelRightMotorPinOne = 8;
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

void blinkHeadlight(int pin) {
  digitalWrite(pin, HIGH);
  delay(500);
  digitalWrite(pin, LOW);
}

void writeServoSmooth(Servo servo,int current, int deg, bool negative) {
  for(int i = 1; i <= deg; i++) {
    if(negative) {
      servo.write(current - i);
    } else {
      servo.write(current + i);
    }
    delay(20);
  }
}

void stepper(int n, bool dir){
  int steps = 0;
  for (int i = 0; i < n; i++){
    switch(steps){
     case 0:
       digitalWrite(craneBasePinOne, LOW); 
       digitalWrite(craneBasePinTwo, LOW);
       digitalWrite(craneBasePinThree, LOW);
       digitalWrite(craneBasePinFour, HIGH);
       break; 
     case 1:
       digitalWrite(craneBasePinOne, LOW); 
       digitalWrite(craneBasePinTwo, LOW);
       digitalWrite(craneBasePinThree, HIGH);
       digitalWrite(craneBasePinFour, HIGH);
       break; 
     case 2:
       digitalWrite(craneBasePinOne, LOW); 
       digitalWrite(craneBasePinTwo, LOW);
       digitalWrite(craneBasePinThree, HIGH);
       digitalWrite(craneBasePinFour, LOW);
       break; 
     case 3:
       digitalWrite(craneBasePinOne, LOW); 
       digitalWrite(craneBasePinTwo, HIGH);
       digitalWrite(craneBasePinThree, HIGH);
       digitalWrite(craneBasePinFour, LOW);
       break; 
     case 4:
       digitalWrite(craneBasePinOne, LOW); 
       digitalWrite(craneBasePinTwo, HIGH);
       digitalWrite(craneBasePinThree, LOW);
       digitalWrite(craneBasePinFour, LOW);
       break; 
     case 5:
       digitalWrite(craneBasePinOne, HIGH); 
       digitalWrite(craneBasePinTwo, HIGH);
       digitalWrite(craneBasePinThree, LOW);
       digitalWrite(craneBasePinFour, LOW);
       break; 
     case 6:
       digitalWrite(craneBasePinOne, HIGH); 
       digitalWrite(craneBasePinTwo, LOW);
       digitalWrite(craneBasePinThree, LOW);
       digitalWrite(craneBasePinFour, LOW);
       break; 
     case 7:
       digitalWrite(craneBasePinOne, HIGH); 
       digitalWrite(craneBasePinTwo, LOW);
       digitalWrite(craneBasePinThree, LOW);
       digitalWrite(craneBasePinFour, HIGH);
       break; 
     default:
       digitalWrite(craneBasePinOne, LOW); 
       digitalWrite(craneBasePinTwo, LOW);
       digitalWrite(craneBasePinThree, LOW);
       digitalWrite(craneBasePinFour, LOW);
       break; 
    }
    steps = setDirection(steps, dir);
    delay(5);
  }
} 

int setDirection(int steps, bool dir){
  if(dir){ 
    steps++;
  }
  
  if(!dir){ 
    steps--; 
  }
  
  if(steps > 7){
    steps=0;
  }
  
  if(steps < 0){
    steps=7; 
  }

  return steps;
}

void readUltrasonicFN() {
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
  Serial1.println("The vehicle is in the slow gear operation mode.");
  
  wheelDirection.attach(wheelDirectionPin);
  wheelDirection.write(wheelDirectionPos);

  // crane stuff
  // attach servos to respective pins
  craneLowerJoint.attach(craneLowerJointPin);
  craneLowerJoint.write(craneLowerJointPos); // 180-degree
  craneMiddleJoint.attach(craneMiddleJointPin);
  craneMiddleJoint.write(craneMiddleJointStop); // continuous
  craneClawClench.attach(craneClawClenchPin);
  craneClawClench.write(70); // 180-degree
  craneClawRotation.attach(craneClawRotationPin);
  craneClawRotation.write(craneClawRotationPos); // 180-degree

  // camera stuff
  // attach servos to respective pins
  cameraX.attach(cameraXPin);
  cameraY.attach(cameraYPin);
  // move camera servos to middle position
  cameraX.write(cameraXPos);
  cameraY.write(cameraYPos);
  
  Serial1.flush();
}

void loop() {
  // read the ultrasonic sensor on every loop() iteration.
  if(readUltrasonic) {
    readUltrasonicFN();
  }
  
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
      // camera navigation(o - up, k - left, l - down, ; - right, m - start position)
      case 'o':
        cameraYPos = cameraYPos + 5;
      	if(cameraYPos > 180) {
        	cameraYPos = 180;
      	}
        Serial1.print("i: ");
        Serial1.println(cameraYPos);
        cameraY.write(cameraYPos);
        break;
      case 'k':
        cameraXPos = cameraXPos + 5;
      	if(cameraXPos > 180) {
        	cameraXPos = 180;
      	}
        Serial1.print("j: ");
        Serial1.println(cameraXPos);
        cameraX.write(cameraXPos);
        break;
      case 'l':
        cameraYPos = cameraYPos - 5;
      	if(cameraYPos < 0) {
        	cameraYPos = 0;
      	}
        Serial1.print("k: ");
        Serial1.println(cameraYPos);
        cameraY.write(cameraYPos);
        break;
      case ';':
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
      // rotate the base of the crane(r - clockwise, f - counterclockwise), manual control mode does not change anything.
      case 'r':
        Serial1.print("Crane base: ");
        stepper(2048, true);
        craneBasePos += 22.5;
        if(craneBasePos > 360) {
          craneBasePos -= 360;
        } else if(craneBasePos < 0) {
          craneBasePos += 360;
        }
        Serial1.println(craneBasePos);
        break;
      case 'f':
        Serial1.print("Crane base: ");
        stepper(2048, false);
        craneBasePos -= 22.5;
        if(craneBasePos > 360) {
          craneBasePos -= 360;
        } else if(craneBasePos < 0) {
          craneBasePos += 360;
        }
        Serial1.println(craneBasePos);
        break;
      // move the lower joint up (t) or down (g)
      case 't':
        Serial1.print("Lower Joint: ");
        craneLowerJointPos += 10;
        if(craneLowerJointPos > 180) {
          craneLowerJointPos = 180;
          Serial1.println("At its maximum!");
        }
        craneLowerJoint.write(craneLowerJointPos);
        Serial1.println(craneLowerJointPos);
        break;
      case 'g':
        Serial1.print("Lower Joint: ");
        if(craneLowerJointPos <  0) {
          craneLowerJointPos = 0;
          craneLowerJoint.write(craneLowerJointPos);
          Serial1.println(craneLowerJointPos);
        } else if(craneLowerJointPos != 0) {
          writeServoSmooth(craneLowerJoint, craneLowerJointPos, 10, true);
          craneLowerJointPos -= 10;
          Serial1.println(craneLowerJointPos);
        } else {
          Serial1.println("At its minimum!");
        }
        break;
      // used to move the middle joint up (y) and down (h)
      case 'y':
        Serial1.print("Middle Joint: ");
        if(craneMiddleJointPos < 140) {
          craneMiddleJoint.write(craneMiddleJointStop + craneMiddleJointSpeed);
          delay(timeSampleServo);
          craneMiddleJoint.write(craneMiddleJointStop);
          craneMiddleJointPos += craneMiddleJointSpeed;
          Serial1.println(craneMiddleJointPos);
        }
        if(craneMiddleJointPos >= 140) {
          craneMiddleJointPos = 140;
          Serial1.println("At its maximum!");
        }
        break;
      case 'h':
        Serial1.print("Middle Joint: ");
        if(craneMiddleJointPos > -10) {
          craneMiddleJoint.write(craneMiddleJointStop - craneMiddleJointSpeed);
          delay(timeSampleServo);
          craneMiddleJoint.write(craneMiddleJointStop);
          craneMiddleJointPos -= craneMiddleJointSpeed;
          Serial1.println(craneMiddleJointPos);
        } else if(craneMiddleJointPos <= -10) {
          craneMiddleJointPos = -10;
          Serial1.println("At its minimum!");
        }
        break;
      case 'u':
        Serial1.print("Crane Claw Rotation, Clockwise: ");
        if(craneClawRotationPos > 120) {
          craneClawRotationPos = 120;
          craneClawRotation.write(craneClawRotationPos);
        } else if(craneClawRotationPos != 120) {
          writeServoSmooth(craneClawRotation, craneClawRotationPos, 10, false);
          craneClawRotationPos += 10;
        }
//        craneClawRotation.write(craneClawRotationPos);
        Serial1.println(craneClawRotationPos);
        break;
      case 'j':
        Serial1.print("Crane Claw Rotation, Counterclockwise: ");
        if(craneClawRotationPos <  30) {
          craneClawRotationPos = 30;
          craneClawRotation.write(craneClawRotationPos);
        } else if(craneClawRotationPos != 30) {
          writeServoSmooth(craneClawRotation, craneClawRotationPos, 10, true);
          craneClawRotationPos -= 10;
        }
        Serial1.println(craneClawRotationPos);
        break;
      case 'n':
        craneClawClenchPos = craneClawClenchPos ^ true;
        if(craneClawClenchPos) {
          writeServoSmooth(craneClawClench, 110, 40, false);
          Serial1.println("Clenching the claw.");
        } else {
          writeServoSmooth(craneClawClench, 70, 40, true);
          Serial1.println("Unclenching the claw.");
        }
        break;
      // return the crane to the resting position - b
      case 'b':
        Serial1.println("Reset crane.");
        if(craneBasePos >= 180) {
          stepper(256 * ((360 - craneBasePos) / 22.5), true);
          craneBasePos = 0;
        } else if(craneBasePos != 0) {
          stepper(256 * (craneBasePos / 22.5), false);
          craneBasePos = 0;
        }
        
        craneLowerJointPos = 90;
        craneLowerJoint.write(craneLowerJointPos);
        
        if(craneMiddleJointPos > 90) {
          craneMiddleJoint.write(craneMiddleJointStop - craneMiddleJointSpeed);
          delay((craneMiddleJointPos - 90) / craneMiddleJointSpeed * timeSampleServo);
          craneMiddleJoint.write(craneMiddleJointStop);
          craneMiddleJointPos = 90;
        } else if(craneMiddleJointPos < 90) {
          craneMiddleJoint.write(craneMiddleJointStop + craneMiddleJointSpeed);
          delay((90 - craneMiddleJointPos) / craneMiddleJointSpeed * timeSampleServo);
          craneMiddleJoint.write(craneMiddleJointStop);
          craneMiddleJointPos = 90;
        }

        craneClawRotationPos = 30;
        craneClawRotation.write(craneClawRotationPos);
        break;
      // wheel stuff, controls are: w - forwards (turn the motor in positive direction), s - backwards (move the motor in negative direction), a - left (in combination with w/s), d - right (in combination with w/s)
      case 'w':
        Serial1.println("should be moving");
      	wheelIterationCounter = 0;
      	motorRunning = true;
        if(gearSlow) {
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
        if(gearSlow) {
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
      // switch motor gears (q)
      case 'z':
        gearSlow = gearSlow ^ true;
        if(gearSlow) {
          Serial1.println("The vehicle is in the slow gear operation mode.");
        } else {
          Serial1.println("The vehicle is in the fast gear operation mode.");
        }
        break;
      case 'x':
        readUltrasonic = readUltrasonic ^ true;
        if(readUltrasonic) {
          Serial1.println("The ROD is fetching data from the ultrasonic sensor.");
        } else {
          Serial1.println("The ROD is NOT fetching data from the ultrasonic sensor.");
        }
    }
  } else if(motorRunning) {
    wheelIterationCounter++;
//    Serial1n.println(wheelIterationCounter);
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
