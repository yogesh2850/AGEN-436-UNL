// Motor A (Right Motor)
const int motorRightIn1 = 2; 
const int motorRightIn2 = 4; 
const int motorRightEn = 9;  

// Motor B (Left Motor)
const int motorLeftIn1 = 7; 
const int motorLeftIn2 = 8; 
const int motorLeftEn = 10; 


// 3 UR sensors
const int trigPin_1 = 5;
const int echoPin_1 = 6;
const int trigPin_2 = 11;
const int echoPin_2 = 3;
const int trigPin_3 = 13;
const int echoPin_3 = 12;


// Distance variables
float distanceFront, distanceRight, distanceLeft;

// Constants for thresholds
const float obstacleThreshold = 25;  // Td from front wall
const float targetSideDistance = 20; // d from side walls

// Speed constants
const int baseSpeed = 65;  // for moving straight
const int minSpeed = 0;    // Min speed 
const int maxSpeed = 95;   // Max speed


void setup() {
  pinMode(motorRightIn1, OUTPUT);
  pinMode(motorRightIn2, OUTPUT);
  pinMode(motorRightEn, OUTPUT);
  pinMode(motorLeftIn1, OUTPUT);
  pinMode(motorLeftIn2, OUTPUT);
  pinMode(motorLeftEn, OUTPUT);
  
  pinMode(trigPin_1, OUTPUT);
  pinMode(echoPin_1, INPUT);
  pinMode(trigPin_2, OUTPUT);
  pinMode(echoPin_2, INPUT);
  pinMode(trigPin_3, OUTPUT);
  pinMode(echoPin_3, INPUT); 
  Serial.begin(9600);  

  stopMotors();
  delay(1000);  
}

void loop() {
  distanceFront = getDistance(trigPin_1, echoPin_1);
  distanceRight = getDistance(trigPin_2, echoPin_2);
  distanceLeft = getDistance(trigPin_3, echoPin_3);
  
  // Print distance for debugging
  Serial.print("Distance Front: "); Serial.println(distanceFront);
  //delay(2000);
  Serial.print("Distance Right: "); Serial.println(distanceRight);
  //delay(1000);
  Serial.print("Distance Left: "); Serial.println(distanceLeft);
  //delay(1000);


  // Speed Adjustment
  int motorSpeed = calculateSpeed(distanceFront);
  
  if (distanceFront < obstacleThreshold) {
    stopMotors();
  } else {
    if (distanceRight < targetSideDistance) {
      steerLeft();
    } else if (distanceLeft < targetSideDistance) {
      steerRight();
    } else {
      goStraight(motorSpeed);  
    }
  }
}


void stopMotors() {
  digitalWrite(motorRightIn1, LOW);
  digitalWrite(motorRightIn2, LOW);
  digitalWrite(motorLeftIn1, LOW);
  digitalWrite(motorLeftIn2, LOW);
  analogWrite(motorRightEn, 0);  // Stop
  analogWrite(motorLeftEn, 0);  // Stop 
}


void goStraight(int speed) {
  digitalWrite(motorRightIn1, HIGH);
  digitalWrite(motorRightIn2, LOW);
  digitalWrite(motorLeftIn1, HIGH);
  digitalWrite(motorLeftIn2, LOW);
  analogWrite(motorRightEn, speed); 
  analogWrite(motorLeftEn, speed);  
}


void steerLeft() {
  digitalWrite(motorRightIn1, HIGH);
  digitalWrite(motorRightIn2, LOW);
  
  int leftSpeed;
  int rightSpeed = maxSpeed;  // max speed
  leftSpeed = map(distanceRight, 0, 2*targetSideDistance, minSpeed, maxSpeed); // Map distance to speed
  
  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  
  analogWrite(motorRightEn, rightSpeed);
  analogWrite(motorLeftEn, leftSpeed); 
}


void steerRight() {
  digitalWrite(motorRightIn1, HIGH);
  digitalWrite(motorRightIn2, LOW);
  
  int rightSpeed = map(distanceLeft, 0, 2*targetSideDistance, minSpeed, maxSpeed); // Map distance to speed
  int leftSpeed = maxSpeed;  // give max speed
  
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
  
  analogWrite(motorRightEn, rightSpeed);
  analogWrite(motorLeftEn, leftSpeed);
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0343) / 2;  
}


int calculateSpeed(float distance) {
  if (distance < obstacleThreshold) {
    stopMotors();
    return 0; 
  } else {
    return baseSpeed; 
  }
}