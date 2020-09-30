#include <Wire.h>

const uint8_t MPU = 0b1101000;
const float GYRO_WEIGHT = 0.96;
const float ACCELEROMETER_SENSITIVITY = 1.0 / 16384;
const float GYROSCOPE_SENSITIVITY = 1.0 / 131.0;
const uint8_t YawDriftSampleCount = 10;
float YawDrift = 0.0;
int16_t Gyro[3];
int16_t Accel[3];
float Pitch = 0.0, Yaw = 0.0;

const uint8_t stepsPerRevolution = 200;
uint8_t stepperOnePins[] = {8, 9, 10, 11};
uint8_t stepperTwoPins[] = {2, 3, 4, 5};
uint8_t stepperOneOut = 0b0011;
uint8_t stepperTwoOut = 0b0011;

float targetAngle = 15;
const int MIN_STEP_DELAY_MS = 2;
const float MAX_PITCH_OFFSET = 5.0;
const float P = MIN_STEP_DELAY_MS / MAX_PITCH_OFFSET;

uint64_t PastTime = 0, DeltaTime = 0;

void setup(){
  Serial.begin(9600); //serial communication at 9600 bauds
  SetupIMU();
  for(int i = 0; i < 4; ++i){
    pinMode(stepperOnePins[i], OUTPUT);
    pinMode(stepperTwoPins[i], OUTPUT);
  }
  PastTime = millis();
  GetAccelerationInput();
  Pitch = GetAccelerometerPitch();
}

void loop(){
  DeltaTime = millis() - PastTime;
  PastTime = PastTime + DeltaTime;
  GetGyroInput();
  GetAccelerationInput();
  CalculatePitchAndYaw();
  StepMotors();
  delay(GetMSBeforeNextStep());
}
float GetAccelerometerPitch(){
  return atan2f(Accel[0], Accel[1]) * 180.0 / 3.14159;
}
void CalculatePitchAndYaw(){
  // http://www.pieter-jan.com/node/11
  Yaw += (Gyro[0] - YawDrift) * GYROSCOPE_SENSITIVITY * DeltaTime / 1000.0;
  Pitch += Gyro[2] * GYROSCOPE_SENSITIVITY * DeltaTime / 1000.0;
  int forceMagnitude = abs(Accel[0]) + abs(Accel[1]) + abs(Accel[2]);
  forceMagnitude *= ACCELEROMETER_SENSITIVITY;
  if(abs(forceMagnitude) < 2.0){
    // If there is not a lot of force
    float p = GetAccelerometerPitch();
    Pitch = Pitch * GYRO_WEIGHT + p * (1.0 - GYRO_WEIGHT);
  }
}
void SetupIMU(){
  Wire.beginTransmission(0b1101000);          // Start the communication by using address of MPU
  Wire.write(0x6B);                           // Access the power management register
  Wire.write(0b00000000);                     // Set sleep = 0
  Wire.endTransmission();                     // End the communication
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);                           // Access the gyro configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);                           // Access the accelerometer configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();

  CalibrateYawDrift();
}
void CalibrateYawDrift(){
  long YawDriftSum = 0;
  for(int i = 0; i < YawDriftSampleCount; ++i){
    GetGyroInput();
    YawDriftSum += Gyro[0];
  }
  YawDrift = YawDriftSum / YawDriftSampleCount;
}
void GetAccelerationInput() {
  Wire.beginTransmission(MPU); 
  Wire.write(0x3B); 
  Wire.endTransmission();
  Wire.requestFrom(MPU,(uint8_t)6); 
  while(Wire.available() < 6);
  Accel[0] = Wire.read()<<8|Wire.read(); 
  Accel[1] = Wire.read()<<8|Wire.read(); 
  Accel[2] = Wire.read()<<8|Wire.read();
}
void GetGyroInput(){
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU, (uint8_t)6);
  while(Wire.available() < 6);
  Gyro[0] = Wire.read() << 8 | Wire.read();
  Gyro[1] = Wire.read() << 8 | Wire.read();
  Gyro[2] = Wire.read() << 8 | Wire.read();
}
int GetMSBeforeNextStep(){
  float error = abs(targetAngle - Pitch);
  float result = error * P;
  float reciprocal = 1.0 / max(0.01, result); // max is to avoid dividing by zero
  return max(MIN_STEP_DELAY_MS, reciprocal);
}
void StepMotors(){
  if(Pitch < targetAngle){
    Step();
  } else {
    ReverseStep();
  }
  for(int i = 0; i < 4; i++){
    digitalWrite(stepperOnePins[i], stepperOneOut & (1 << i));
    digitalWrite(stepperTwoPins[i], stepperTwoOut & (1 << i));
  }
}
void Step(){
  stepperOneOut = ((stepperOneOut << 1) | (stepperOneOut >> 3)) & 0b1111;
  stepperTwoOut = ((stepperTwoOut >> 1) | (stepperTwoOut << 3)) & 0b1111;
}
void ReverseStep(){
  stepperOneOut = ((stepperOneOut >> 1) | (stepperOneOut << 3)) & 0b1111;
  stepperTwoOut = ((stepperTwoOut << 1) | (stepperTwoOut >> 3)) & 0b1111;
}
