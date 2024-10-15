// Motor A (Right Motor)
const int motorAIn1 = 2; 
const int motorAIn2 = 4; 
const int motorAEn = 9;  

// Motor B (Left Motor)
const int motorBIn1 = 7; 
const int motorBIn2 = 8; 
const int motorBEn = 10; 

// 3 UR sensors
const int trigPin_1 = 5;
const int echoPin_1 = 6;
const int trigPin_2 = 11;
const int echoPin_2 = 3;
const int trigPin_3 = 13;
const int echoPin_3 = 12;

float distanceFront, distanceRight, distanceLeft;

const float obstacleThreshold = 30.0;  // front sensor  gap
const float targetSideDistance = 19.0; // side wall gap

float kp = 3; 
float ki = 0.5;  
float kd = 0; 
float prevError = 0;
float integral = 0;
float error = 0;
float derivative = 0;
float pidOutput = 0;
float speedBase = 210; // base speed 
float speedAdjustment = 0;
float minSpeed = 0;

unsigned long currentTime, previousTime;
float elapsedTime;

void setup() {
  pinMode(motorAIn1, OUTPUT);
  pinMode(motorAIn2, OUTPUT);
  pinMode(motorAEn, OUTPUT);
  pinMode(motorBIn1, OUTPUT);
  pinMode(motorBIn2, OUTPUT);
  pinMode(motorBEn, OUTPUT);
  
  pinMode(trigPin_1, OUTPUT);
  pinMode(echoPin_1, INPUT);
  pinMode(trigPin_2, OUTPUT);
  pinMode(echoPin_2, INPUT);
  pinMode(trigPin_3, OUTPUT);
  pinMode(echoPin_3, INPUT); 

  Serial.begin(9600);  

  stopMotors();
  analogWrite(motorAEn, speedBase); 
  analogWrite(motorBEn, speedBase); 

  // time for PID
  previousTime = millis();
}

void loop() {
    distanceFront = getDistance(trigPin_1, echoPin_1);
    distanceRight = getDistance(trigPin_2, echoPin_2);
    distanceLeft = getDistance(trigPin_3, echoPin_3);
    
    Serial.print("Distance Front: "); Serial.println(distanceFront);
    Serial.print("Distance Right: "); Serial.println(distanceRight);
    Serial.print("Distance Left: "); Serial.println(distanceLeft);

    // logic
    if (distanceFront < obstacleThreshold) {
        stopMotors();  
    } 
    else {
        
        if (distanceRight < targetSideDistance) {
            error = targetSideDistance - distanceRight;  
            applyPID(true);  
        } 
        else if (distanceLeft < targetSideDistance) {
            error = targetSideDistance - distanceLeft;  
            applyPID(false);  
        } 
        else {
            goStraight();  
        }
    }
}

void applyPID(bool isRight) {
    currentTime = millis();
    elapsedTime = (float)(currentTime - previousTime) / 1000;  // time in sec

    integral += error * elapsedTime;
    derivative = (error - prevError) / elapsedTime;
    pidOutput = (kp * error) + (ki * integral) + (kd * derivative);

    speedAdjustment = constrain(pidOutput, -speedBase, speedBase);  // limit

    if (isRight) {
        analogWrite(motorBEn, min(speedBase + speedAdjustment, minSpeed));  
        analogWrite(motorAEn, speedBase);  
    } else {
        analogWrite(motorBEn, speedBase);  
        analogWrite(motorAEn, min(speedBase + speedAdjustment, minSpeed));  
    }

    // saving
    prevError = error;
    previousTime = currentTime;
}

void stopMotors() {
  digitalWrite(motorAIn1, LOW);
  digitalWrite(motorAIn2, LOW);
  digitalWrite(motorBIn1, LOW);
  digitalWrite(motorBIn2, LOW);
  analogWrite(motorAEn, 0);  
  analogWrite(motorBEn, 0);  
}

void goStraight() {
  digitalWrite(motorAIn1, HIGH);
  digitalWrite(motorAIn2, LOW);
  digitalWrite(motorBIn1, HIGH);
  digitalWrite(motorBIn2, LOW);
  analogWrite(motorAEn, speedBase); 
  analogWrite(motorBEn, speedBase); 
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
