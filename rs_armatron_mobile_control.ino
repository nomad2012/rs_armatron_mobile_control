#include <AFMotor.h>
#include <Servo.h>

// constants won't change. Used here to 
// set pin numbers:
const int p1ArmDn = 53;
const int p1ArmUp = 51;
const int p1WheelRBk = 49;
const int p1WheelRFw = 47;
const int p1WheelLBk = 45;
const int p1WheelLFw = 43;
const int p1WristDn = 41;
const int p1WristUp = 39;
const int p1Jaw = 37;
const int p1WristRot = 35;
const int p1MotorSense = 2;
const int pSwitch01 = 22;

const int pCoinSense = 52;
const int pServo1 = 10;
const int pServo2 = 9;

int ms = 0;
int dr = 0;
int motCur = 0; // Motor current sense

// Timers for each motor
unsigned long t1ArmTimer = 0;
unsigned long t1WheelLTimer = 0;
unsigned long t1WheelRTimer = 0;
unsigned long t1WristTimer = 0;
unsigned long t1JawTimer = 0;
unsigned long switch01Timer = 0;
unsigned long loadBallTimer = 0;
unsigned long coinSenseTimer = 0;

int ib = 0;   // for incoming serial data

Servo servo1; // servo in coin box / makerbot
Servo servo2; // gumball lift arm release servo

void setup() {
  // Set the first Armatron control pins:
  pinMode(p1ArmDn, OUTPUT);
  pinMode(p1ArmUp, OUTPUT);
  pinMode(p1WheelRBk, OUTPUT);
  pinMode(p1WheelRFw, OUTPUT);
  pinMode(p1WheelLBk, OUTPUT);
  pinMode(p1WheelLFw, OUTPUT);
  pinMode(p1WristDn, OUTPUT);
  pinMode(p1WristUp, OUTPUT);
  pinMode(p1Jaw, OUTPUT);
  pinMode(p1WristRot, OUTPUT);

  // Set them all low so we don't have a run-away motor
  digitalWrite(p1ArmDn, LOW);
  digitalWrite(p1ArmUp, LOW);
  digitalWrite(p1WheelRBk, LOW);
  digitalWrite(p1WheelRFw, LOW);
  digitalWrite(p1WheelLBk, LOW);
  digitalWrite(p1WheelLFw, LOW);
  digitalWrite(p1WristDn, LOW);
  digitalWrite(p1WristUp, LOW);
  digitalWrite(p1Jaw, LOW);
  digitalWrite(p1WristRot, LOW);
  
  // Set up current sensing
  pinMode(p1MotorSense, INPUT);

  // Set up the push-button switch
  pinMode(pSwitch01, INPUT);
  digitalWrite(pSwitch01, HIGH);
  
  // Set up the coin sensor
  pinMode(pCoinSense, INPUT);
  digitalWrite(pCoinSense, HIGH);
  
  // Set up servo
  servo1.attach(pServo1);
  servo2.attach(pServo2);
  delay(500);
  servo2.write(1); 
/*  // test Drop the ball
  servo1.write(130);
  // reset the carrier
  delay(1000);
  servo1.write(90);
  delay(1000);
  servo1.write(130);
*/  
  
  Serial.begin(115200);
  Serial.println("Initializing Armatron #1...");
  delay(100); // Let things settle
  Serial.println("Ready Master...");
}

void loop() {
  // Check for movement that should stop
  mvArm(0, 0);
  mvWrist(0, 0);
  mvWheelL(0, 0);
  mvWheelR(0, 0);
  mvWristJaw(0, 0);
  
  if (loadBallTimer > 0) {  // Needed to keep things going
    loadBall();
  }
  
  unsigned long curTime = millis();

  // See if coin sensed
  int coinSensed = digitalRead(pCoinSense);
  //Serial.print("coin sense = ");
  //Serial.print(coinSensed);
  
  if (coinSensed == LOW) {
    //Serial.println(" ----> Dispense Ball");
    servo1.write(85);
    coinSenseTimer = curTime + 5000;
  } else if (coinSenseTimer < curTime) {
    //Serial.println("RESET SERVO");
    servo1.write(130);
  }
  //delay(15);
  
  // See if the switch has been pressed
  int switch01 = digitalRead(pSwitch01);
  
  if (switch01 == LOW) {
//    Serial.print("It's low: ");
//    Serial.println(switch01);
    if (switch01Timer < curTime) { // No recent switch presses
      Serial.println("Button pressed, doing stuff...");
      loadBall();
      switch01Timer = curTime + 10000;
    }
  }
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    ib = Serial.read();
    
    // If we see a "0" or a "1"
    if (ib >= 48 && ib <= 49) {  // Move the Arm
      int ab = ib - 48;
      mvArm(ab,1000);
    }
    // If we see a "2" or a "3"
    if (ib >= 50 && ib <= 51) {  // Move the Wrist
      int wb = ib - 50;
      mvWrist(wb,1000);
    }
    // If we see a "4" or a "5"
    if (ib >= 52 && ib <= 53) {  // Move the Left Tire
      int wl = ib - 52;
      mvWheelL(wl,200);
    }
    // If we see a "6" or a "7"
    if (ib >= 54 && ib <= 55) {  // Move the Right Tire
      int wr = ib - 54;
      mvWheelR(wr,200);
    }
    // If we see a "8" or a "9"
    if (ib >= 56 && ib <= 57) {  // Move the Jaw
      int wj = ib - 56;
      mvWristJaw(wj,1000);
    }
    
    // "a" releases gumball lift arm
    if (ib == 97) {
      servo2.write(90);
    }
    
    // "z" resets gumball servo
    if (ib == 122) {
      servo2.write(1);
    }
    
    // "c" manually triggers coin servo
    if (ib == 99) {
      servo1.write(85);
      coinSenseTimer = curTime + 5000;
    }
    
    // say what you got:
    Serial.print("ASCII In: ");
    Serial.println(ib);
  }

}

int lb1 = 0;

int loadBall () {
  unsigned long curTime = millis();  
  mvArm(0,0);
  mvWrist(0,0);
  mvWheelL(0,0);
  mvWheelR(0,0);
  if (loadBallTimer == 0) { // Begin arm movement
    mvArm(1, 1050);
    loadBallTimer = curTime + 6000;
  } else if ((curTime + 5000) > loadBallTimer && loadBallTimer > curTime) { // Done lifting, move the wrist
    if (lb1 == 0) {
// not needed   mvWrist(1,600);
      lb1 = 1;
    }
  } else if (loadBallTimer < curTime) { // Done
    Serial.println("Finished loading ball");
    delay(2000);
    mvArm(0, 1000);
    mvWrist(0,530);
    
    lb1 = 0;
    loadBallTimer = 0;
    return 1;
  }
  return 0;
}

/*   Returns 1 if the arm is moving, 0 if it's not */
  int mvArm(int dir, unsigned long dist) {
  
  unsigned long curTime = millis();
  unsigned long mvInverval;

  if (dist != 0) {
    if (dir == 0) { // Move down
      mvInverval = dist * 3.55;
      digitalWrite(p1ArmUp, LOW);
      digitalWrite(p1ArmDn, HIGH);
    } else { // Move up
      mvInverval = dist * 3.5;
      digitalWrite(p1ArmDn, LOW);
      digitalWrite(p1ArmUp, HIGH);
    }
    t1ArmTimer = curTime + mvInverval;
    
    Serial.print("Moving arm ");
    Serial.print(dir);
    Serial.print(", ");
    Serial.print(dist);
    Serial.print(".\n");

  } else if (dist == 0 && t1ArmTimer < curTime) { // Check the timer to see if we should stop
    digitalWrite(p1ArmUp, LOW);
    digitalWrite(p1ArmDn, LOW);
    return 0;
  } else if (dist == 0 && t1ArmTimer > curTime){ // We are moving... see if we hit something...
    chkMotData("Arm motor", p1ArmUp, p1ArmDn);
    return 1;
  }
  return 0;
}

/*   Returns 1 if the wrist is moving, 0 if it's not */
int mvWrist(int dir, unsigned long dist) {
  
  unsigned long curTime = millis();
  unsigned long mvInverval;

  if (dist != 0) {
    if (dir == 0) { // Move down
      mvInverval = dist * 4.45;
      digitalWrite(p1WristUp, LOW);
      digitalWrite(p1WristDn, HIGH);
    } else { // Move up
      mvInverval = dist * 3.6;
      digitalWrite(p1WristDn, LOW);
      digitalWrite(p1WristUp, HIGH);
    }
    t1WristTimer = curTime + mvInverval;
    
    Serial.print("Moving wrist ");
    Serial.print(dir);
    Serial.print(", ");
    Serial.print(dist);
    Serial.print(".\n");

  } else if (dist == 0 && t1WristTimer < curTime) { // Check the timer to see if we should stop
    digitalWrite(p1WristUp, LOW);
    digitalWrite(p1WristDn, LOW);
    return 0;
  } else if (dist == 0 && t1WristTimer > curTime){ // We are moving... see if we hit something...
    chkMotData("Wrist motor", p1WristUp, p1WristDn);
    return 1;
  }
  return 0;
}

/*  Returns 1 if the wheel is moving, 0 if it's not */
int mvWheelL(int dir, unsigned long dist) {
  
  unsigned long curTime = millis();
  unsigned long mvInverval;

  if (dist != 0) {
    if (dir == 0) { // Move back
      mvInverval = dist * 6;
      digitalWrite(p1WheelLFw, LOW);
      digitalWrite(p1WheelLBk, HIGH);
    } else { // Move forward
      mvInverval = dist * 3;
      digitalWrite(p1WheelLBk, LOW);
      digitalWrite(p1WheelLFw, HIGH);
    }
    t1WheelLTimer = curTime + mvInverval;
    
    Serial.print("Moving L Wheel ");
    Serial.print(dir);
    Serial.print(", ");
    Serial.print(dist);
    Serial.print(".\n");

  } else if (dist == 0 && t1WheelLTimer < curTime) { // Check the timer to see if we should stop
    digitalWrite(p1WheelLBk, LOW);
    digitalWrite(p1WheelLFw, LOW);
    return 0;
  } else if (dist == 0 && t1WheelLTimer > curTime){ // We are moving... see if we hit something...
    chkMotData("L Wheel motor", p1WheelLBk, p1WheelLFw);
    return 1;
  }
  return 0;
}

/*  Returns 1 if the wheel is moving, 0 if it's not */
int mvWheelR(int dir, unsigned long dist) {
  
  unsigned long curTime = millis();
  unsigned long mvInverval;

  if (dist != 0) {
    if (dir == 0) { // Move back
      mvInverval = dist * 6;
      digitalWrite(p1WheelRFw, LOW);
      digitalWrite(p1WheelRBk, HIGH);
    } else { // Move forward
      mvInverval = dist * 3;
      digitalWrite(p1WheelRBk, LOW);
      digitalWrite(p1WheelRFw, HIGH);
    }
    t1WheelRTimer = curTime + mvInverval;
    
    Serial.print("Moving R Wheel ");
    Serial.print(dir);
    Serial.print(", ");
    Serial.print(dist);
    Serial.print(".\n");

  } else if (dist == 0 && t1WheelRTimer < curTime) { // Check the timer to see if we should stop
    digitalWrite(p1WheelRBk, LOW);
    digitalWrite(p1WheelRFw, LOW);
    return 0;
  } else if (dist == 0 && t1WheelRTimer > curTime){ // We are moving... see if we hit something...
    chkMotData("R Wheel motor", p1WheelRBk, p1WheelRFw);
    return 1;
  }
  return 0;
}

/*  Returns 1 if the wheel is moving, 0 if it's not */
int mvWristJaw(int dir, unsigned long dist) {
  
  unsigned long curTime = millis();
  unsigned long mvInverval;

  if (dist != 0) {
    if (dir == 0) { // Wrist
      mvInverval = dist * 3.7;
      digitalWrite(p1Jaw, LOW);
      digitalWrite(p1WristRot, HIGH);
    } else { // Jaw
      mvInverval = dist * 3;
      digitalWrite(p1WristRot, LOW);
      digitalWrite(p1Jaw, HIGH);
    }
    t1JawTimer = curTime + mvInverval;
    
    Serial.print("Moving Wrist/Jaw ");
    Serial.print(dir);
    Serial.print(", ");
    Serial.print(dist);
    Serial.print(".\n");

  } else if (dist == 0 && t1JawTimer < curTime) { // Check the timer to see if we should stop
    digitalWrite(p1Jaw, LOW);
    digitalWrite(p1WristRot, LOW);
    return 0;
  } else if (dist == 0 && t1JawTimer > curTime){ // We are moving... see if we hit something...
    chkMotData("WristJaw motor", p1WristRot, p1Jaw);
    return 1;
  }
  return 0;
}

int rotateWrist(unsigned long dist) {
  return mvWristJaw(0, dist);
}

int mvJaw(unsigned long dist) {
  return mvWristJaw(1, dist);
}

void chkMotData (char mot[20], int pin1, int pin2) {
  return; // Doesn't work
 int motCurNow = analogRead(p1MotorSense);
  if (motCur != motCurNow) { // New value, print it out
   Serial.print(mot);
   Serial.print(" sense: ");
   Serial.print(motCurNow);
   Serial.println();
   motCur = motCurNow;
 } 
}

void ms1Print (){
  int ms = analogRead(p1MotorSense);
  Serial.print("Motor 1: ");
  Serial.print(ms);
  Serial.println(); 
}

void chkMot1 () {
  ms = analogRead(p1MotorSense);
  dr = 1;
}
