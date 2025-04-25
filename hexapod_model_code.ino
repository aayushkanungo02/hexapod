
// Push-Up and Down code

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver PCA1(0x40), PCA2(0x41);
#define SERVOMIN 150
#define SERVOMAX 600

int angleToPulse(int angle) {
    int pulse = map(constrain(angle, 0, 180), 0, 180, SERVOMIN, SERVOMAX);
    Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" -> Pulse: "); Serial.println(pulse);
    return pulse;
}

void setServoPositions(Adafruit_PWMServoDriver &PCA, const int (*pins)[2], size_t size) {
    for (size_t i = 0; i < size; i++) {
        int pulse = angleToPulse(pins[i][1]);
        PCA.setPWM(pins[i][0], 0, pulse);
        Serial.print("Servo "); Serial.print(pins[i][0]);
        Serial.print(" set to pulse "); Serial.println(pulse);
    }
}

const int leftStand[][2] = {{0, 45}, {8, 135}, {3, 90}, {4, 45}, {5, 135}, {6, 90}, {13, 45}, {14, 135}, {15, 90}};
const int rightStand[][2] = {{0, 45}, {1, 135}, {2, 90}, {3, 45}, {4, 135}, {5, 90}, {7, 45}, {12, 135}, {13, 90}};
const int leftSit[][2] = {{0, 90}, {8, 90}, {3, 90}, {4, 90}, {5, 90}, {6, 90}, {13, 90}, {14, 90}, {15, 90}};
const int rightSit[][2] = {{0, 90}, {1, 90}, {2, 90}, {3, 90}, {4, 90}, {5, 90}, {7, 90}, {12, 90}, {13, 90}};

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing PCA9685 boards...");
    PCA1.begin(); PCA1.setPWMFreq(50);
    PCA2.begin(); PCA2.setPWMFreq(50);
    delay(10);
    Serial.println("PCA9685 boards initialized.");
}

void loop() {
    Serial.println("Standing up...");
    setServoPositions(PCA1, leftStand, 9);
    setServoPositions(PCA2, rightStand, 9);
    delay(3000);

    Serial.println("Sitting down...");
    setServoPositions(PCA1, leftSit, 9);
    setServoPositions(PCA2, rightSit, 9);
    delay(3000);
}



// Inverse Kinematics

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver PCA1(0x40), PCA2(0x41);

#define SERVOMIN 150
#define SERVOMAX 600
#define L1 50   // Coxa length in mm
#define L2 80   // Femur length
#define L3 100  // Tibia length

int angleToPulse(int angle) {
  int pulse = map(constrain(angle, 0, 180), 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

void setServo(Adafruit_PWMServoDriver &PCA, int pin, int angle) {
  PCA.setPWM(pin, 0, angleToPulse(angle));
}

// Inverse Kinematics function
void inverseKinematics(float x, float y, float z, float &theta1, float &theta2, float &theta3) {
  // θ1 (Coxa angle)
  theta1 = atan2(y, x) * 180.0 / PI;

  // Horizontal distance from coxa to foot
  float horizontalDist = sqrt(x * x + y * y) - L1;

  // Distance from femur joint to foot
  float r = sqrt(horizontalDist * horizontalDist + z * z);

  // Check if within reachable workspace
  if (r > (L2 + L3)) r = L2 + L3;

  // θ2 (Femur angle)
  float alpha = atan2(z, horizontalDist);
  float beta = acos((L2*L2 + r*r - L3*L3) / (2 * L2 * r));
  theta2 = (alpha + beta) * 180.0 / PI;

  // θ3 (Tibia angle)
  theta3 = (acos((L2*L2 + L3*L3 - r*r) / (2 * L2 * L3)) * 180.0 / PI) - 90.0;
}


void moveLeg(Adafruit_PWMServoDriver &PCA, int coxaPin, int femurPin, int tibiaPin, float x, float y, float z) {
  float theta1, theta2, theta3;
  inverseKinematics(x, y, z, theta1, theta2, theta3);

  setServo(PCA, coxaPin, theta1 + 90);  
  setServo(PCA, femurPin, theta2);
  setServo(PCA, tibiaPin, theta3);
}

void setup() {
  Serial.begin(115200);
  PCA1.begin(); PCA1.setPWMFreq(50);
  PCA2.begin(); PCA2.setPWMFreq(50);
  delay(10);
}

void loop() {
  
  moveLeg(PCA1, 0, 1, 2, 60, 60, -50);
  delay(1000);
}



// Forward Motion code


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver PCA1(0x40), PCA2(0x41);

#define SERVOMIN 150
#define SERVOMAX 600

int angleToPulse(int angle) {
  return map(constrain(angle, 0, 180), 0, 180, SERVOMIN, SERVOMAX);
}

void setServo(Adafruit_PWMServoDriver &PCA, int pin, int angle) {
  if (pin >= 0) PCA.setPWM(pin, 0, angleToPulse(angle));
}


const int L1[] = {0, 1, -1};  
const int L2[] = {3, 4, 5};      
const int L3[] = {6, 13, 14};   


const int R1[] = {0, 1, 2};    
const int R2[] = {3, 4, 5};     
const int R3[] = {6, 7, 8};     

void liftLeg(Adafruit_PWMServoDriver &PCA, const int leg[3]) {
  setServo(PCA, leg[1], 60); 
  setServo(PCA, leg[2], 60);
}

void dropLeg(Adafruit_PWMServoDriver &PCA, const int leg[3]) {
  setServo(PCA, leg[1], 90); 
  setServo(PCA, leg[2], 90); 
}

void moveCoxaForward(Adafruit_PWMServoDriver &PCA, const int leg[3], int offset) {
  setServo(PCA, leg[0], 90 - offset); 
}

void moveCoxaBackward(Adafruit_PWMServoDriver &PCA, const int leg[3], int offset) {
  setServo(PCA, leg[0], 90 + offset); 
}

void tripodStepA() {

  liftLeg(PCA1, L1); moveCoxaForward(PCA1, L1, 20);
  liftLeg(PCA1, L3); moveCoxaForward(PCA1, L3, 20);
  liftLeg(PCA2, R2); moveCoxaForward(PCA2, R2, 20);

  delay(400);

  // Drop Tripod A
  dropLeg(PCA1, L1); dropLeg(PCA1, L3); dropLeg(PCA2, R2);

  // Push body forward using Tripod B: L2, R1, R3
  moveCoxaBackward(PCA1, L2, 20);
  moveCoxaBackward(PCA2, R1, 20);
  moveCoxaBackward(PCA2, R3, 20);

  delay(400);
}

void tripodStepB() {
  
  liftLeg(PCA1, L2); moveCoxaForward(PCA1, L2, 20);
  liftLeg(PCA2, R1); moveCoxaForward(PCA2, R1, 20);
  liftLeg(PCA2, R3); moveCoxaForward(PCA2, R3, 20);

  delay(400);

  dropLeg(PCA1, L2); dropLeg(PCA2, R1); dropLeg(PCA2, R3);

  
  moveCoxaBackward(PCA1, L1, 20);
  moveCoxaBackward(PCA1, L3, 20);
  moveCoxaBackward(PCA2, R2, 20);

  delay(400);
}

void setup() {
  Serial.begin(115200);
  PCA1.begin(); PCA1.setPWMFreq(50);
  PCA2.begin(); PCA2.setPWMFreq(50);
  delay(10);

  int neutralCoxa = 90, neutralFemur = 90, neutralTibia = 90;
  for (int i = 0; i < 3; i++) {
    setServo(PCA1, L1[i], (L1[i] >= 0 ? (i == 0 ? neutralCoxa : neutralFemur) : 0));
    setServo(PCA1, L2[i], (i == 0 ? neutralCoxa : (i == 1 ? neutralFemur : neutralTibia)));
    setServo(PCA1, L3[i], (i == 0 ? neutralCoxa : (i == 1 ? neutralFemur : neutralTibia)));

    setServo(PCA2, R1[i], (i == 0 ? neutralCoxa : (i == 1 ? neutralFemur : neutralTibia)));
    setServo(PCA2, R2[i], (i == 0 ? neutralCoxa : (i == 1 ? neutralFemur : neutralTibia)));
    setServo(PCA2, R3[i], (i == 0 ? neutralCoxa : (i == 1 ? neutralFemur : neutralTibia)));
  }
}

void loop() {
  tripodStepA();
  tripodStepB();
}


