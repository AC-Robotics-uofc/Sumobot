//      __  ___     __ __      _ __________
//     /  |/  /____/ //_/___ _(_)__  /__  /
//    / /|_/ / ___/ ,< / __ `/ /  / /  / /
//   / /  / / /  / /| / /_/ / /  / /  / /
//  /_/  /_/_/  /_/ |_\__,_/_/  /_/  /_/
//
// Main code for my sumobot! 
// This sumobot is equipped with:
// - 2x IR sensors (Disabled in software)
// - 1x HC-SR04 sensor
// - 1x L298N motor driver
// - 2x banana motors
// - 1x Arduino


#include <Arduino.h>    // For normal functions
#include <Ultrasonic.h> // For HC-SR04 

// Photoresistor
int photoR = A0;
int blackDetected = 590; // Change this value when calibrating

// HC-SR04 pins
Ultrasonic UltraSensor(13, 12); // Initialize Trig and Echo pins of ultrasonic sensor

// IR Sensors
bool enableIR = false; // Set to true if you want to enable IR sensors
int IRR = 2;
int IRL = 3;  

// L298N pins
int MLForward = 10;
int MLBackward = 11;
int MRForward = 8;
int MRBackward = 9;
int MLEnabler = 5;
int MREnabler = 6;

String currentStat = "";


// Function to check if the robot is on the boundary
bool isOnBlack() {
  if(analogRead(photoR) >= blackDetected) {
    return true;
  } else {
    return false;
  }
}

// Function to check ultrasonic sensor
bool ultrasonicCheck() {
  double distance = UltraSensor.read(CM);
  if(distance <= 50) {
    return true;
  } else {
    return false;
  }
}

// Funtion to check right IR sensor
bool checkRightIR() {
  if(digitalRead(IRR) ==  1) {
    return true;
  } else {
    return false;
  }
}

// Funtion to check left IR sensor
bool checkLeftIR() {
  if(digitalRead(IRL) ==  1) {
    return true;
  } else {
    return false;
  }
}

// Function to move backward
void MBack() {
  Serial.println("Backward");
  digitalWrite(MLForward, LOW);
  digitalWrite(MLBackward, HIGH);
  digitalWrite(MRForward, LOW);
  digitalWrite(MRBackward, HIGH);
}

// Function to move forward
void MFore() {
  Serial.println("Forward");
  digitalWrite(MLForward, HIGH);
  digitalWrite(MLBackward, LOW);
  digitalWrite(MRForward, HIGH);
  digitalWrite(MRBackward, LOW);
}

// Function to move right
void MRight() {
  Serial.println("Turning Right");
  digitalWrite(MLForward, HIGH);
  digitalWrite(MLBackward, LOW);
  digitalWrite(MRForward, LOW);
  digitalWrite(MRBackward, HIGH);
}

// Function to move left
void MLeft() {
  Serial.println("Turning Left");
  digitalWrite(MLForward, LOW);
  digitalWrite(MLBackward, HIGH);
  digitalWrite(MRForward, HIGH);
  digitalWrite(MRBackward, LOW);
}

// Function to stop moving
void MStop() {
  Serial.println("Stopping");
  digitalWrite(MLForward, LOW);
  digitalWrite(MLBackward, LOW);
  digitalWrite(MRForward, LOW);
  digitalWrite(MRBackward, LOW);
}

// Function to set speed of motors
void MSpeed(int speed) {
  // Serial.print("Speed to ");
  // Serial.println(speed);
  analogWrite(MREnabler, speed);
  analogWrite(MLEnabler, speed);
}

void setup() {
  // Initialize Serial at 9600 Baud
  Serial.begin(9600);
  Serial.println("Starting up!");

  Serial.println("Initializing Photoresistor pin!");
  pinMode(photoR, INPUT_PULLUP);

  Serial.println("Initializing Motor pins!");
  // Left motors
  pinMode(MLForward, OUTPUT);
  pinMode(MLBackward, OUTPUT);
  pinMode(MLEnabler, OUTPUT);

  // Right motors
  pinMode(MRForward, OUTPUT);
  pinMode(MRBackward, OUTPUT);
  pinMode(MREnabler, OUTPUT);

  // Set motors to full speed
  analogWrite(MLEnabler, 100);
  analogWrite(MREnabler, 100);

  Serial.println("Initializing IR Sensor pins!");
  pinMode(IRR, INPUT_PULLUP);
  pinMode(IRL, INPUT_PULLUP);

  Serial.println("Finished initializing!");
}

void loop() {
  if(isOnBlack() ==  true) { // If black is detected
    // Serial.print("Moving Back");
    MSpeed(255);
    MBack();
    delay(500);
    MStop();
  }
  if(ultrasonicCheck() !=  0) { // If something is detected
    currentStat = "Murderer Mode";
    MSpeed(200);
    MFore();
    delay(100);
  } else {  // If nothing is detected
    currentStat = "Sus Mode";
    MSpeed(100);
    if(enableIR ==  true) {
      if(checkRightIR() ==  true) {       // Check right, and move right if there's anything
        MRight();
        delay(200);
        MStop();
      } else if(checkLeftIR() ==  true) { // Check left, and move left if there's anything
        MLeft();
        delay(200);
        MStop();
      } else {  // If there isn't anything, move forward at a slow rate
        MSpeed(80);
        MFore();
      }
    } else {
      MLeft();
      // MBack();
      delay(70);
      MStop();
      delay(100);
    }
  }

  // DEBUGGING
  Serial.print("PhotoR: ");
  Serial.print(analogRead(photoR)); // Send raw data instead of function, for calibration purposes
  Serial.print(", Ultrasonic: ");
  Serial.print(ultrasonicCheck());
  if(enableIR ==  true) {
    Serial.print(", IRR: ");
    Serial.print(checkRightIR());
    Serial.print(", IRL: ");
    Serial.print(checkLeftIR());
  }
  Serial.print(", Status: ");
  Serial.println(currentStat);
}