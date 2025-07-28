#include <Wire.h>
#include <Servo.h>
#include "CRSFforArduino.hpp"

/* STM32F405 PID Controller for Drone
  Implements a PID controller for a drone using an STM32F405 board.
  It reads gyro data from an MPU6050, applies PID control to adjust motor speeds,
  and uses CRSF for RC channel input.

  The PID parameters can be adjusted for tuning the drone's flight characteristics.
  Also includes a simple TPA (Throttle PID Attenuation) implementation to reduce
  PID gains at higher throttle levels.
*/

// Servo objects for each ESC (Electronic Speed Controller)
Servo esc1, esc2, esc3, esc4;

// Motor pin definitions
#define MOTOR1 5
#define MOTOR2 6
#define MOTOR3 9
#define MOTOR4 10

// IMU (Gyro) variables for pitch, roll, yaw rates
float RatePitch, RateRoll, RateYaw;
// Calibration offsets for gyro rates
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber = 0;

// Filtered gyro values using Exponential Moving Average (EMA)
float FilteredRatePitch = 0, FilteredRateRoll = 0, FilteredRateYaw = 0;
const float EMA_ALPHA = 0.1; // EMA smoothing factor

// Timing variables for main loop
uint32_t LoopTimer;
const float TimeStep = 0.004; // 250Hz loop time (4ms)

// PID variables
float throttle = 1000; // Throttle value from RC
float DesiredRateRoll = 0, DesiredRatePitch = 0, DesiredRateYaw = 0; // Target rates from RC
bool armed = false; // Arming state

// PID error and state variables for each axis
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0;
float PrevItermRateRoll = 0, PrevItermRatePitch = 0, PrevItermRateYaw = 0;
float PIDReturn[3]; // Stores PID output, error, and I-term for reuse
float OutputRoll, OutputPitch, OutputYaw;

// PID tuning parameters for Roll, Pitch, and Yaw axes
float PRateRoll = 0.4, IRateRoll = 1.25, DRateRoll = 0.015;
float PRatePitch = 0.4, IRatePitch = 1.25, DRatePitch = 0.015;
float PRateYaw = 2.0, IRateYaw = 12.0, DRateYaw = 0.0;

// Debug variables (unused now, can be deleted if desired)
float debugPterm, debugIterm, debugDterm;
float debugPtermPitch, debugItermPitch, debugDtermPitch;
float debugPtermYaw, debugItermYaw, debugDtermYaw;

CRSFforArduino *crsf = nullptr;

// Exponential Moving Average function for filtering
float applyEMA(float newValue, float prevEMA, float alpha) {
  return alpha * newValue + (1.0 - alpha) * prevEMA;
}

// Callback function for receiving RC channels via CRSF
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels) {
  if (rcChannels->failsafe) return;

  int rawThrottle = crsf->rcToUs(crsf->getChannel(3));
  int rawRoll     = crsf->rcToUs(crsf->getChannel(1));
  int rawPitch    = crsf->rcToUs(crsf->getChannel(2));
  int rawYaw      = crsf->rcToUs(crsf->getChannel(4));
  int rawArm      = crsf->rcToUs(crsf->getChannel(5));

  // Arming logic:
  // Arm only when arm switch is flipped high and throttle is low at that moment.
  // Stay armed until arm switch is flipped low.
  static bool prevArmSwitchHigh = false;
  bool armSwitchHigh = rawArm > 1500;

  if (!armed) {
    // Only arm if switch is flipped high and throttle is low
    if (armSwitchHigh && !prevArmSwitchHigh && rawThrottle <= 1050) {
      armed = true;
    }
  } else {
    // Disarm if switch is flipped low
    if (!armSwitchHigh) {
      armed = false;
    }
  }
  prevArmSwitchHigh = armSwitchHigh;

  if (armed) {
    throttle = constrain(rawThrottle, 1000, 2000);
    DesiredRateRoll  = map(rawRoll,  1000, 2000, -150, 150);
    DesiredRatePitch = map(rawPitch, 1000, 2000, -150, 150);
    DesiredRateYaw   = map(rawYaw,   1000, 2000, -150, 150);
  } else {
    throttle = 1000;
    DesiredRateRoll = DesiredRatePitch = DesiredRateYaw = 0;
  }
}

// Gyro signal reading and conversion from MPU6050
void gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = -(float)GyroZ / 65.5;
}

// PID equation implementation for calculating PID output
void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * TimeStep / 2;
  Iterm = constrain(Iterm, -400, 400); // Limit I-term to prevent windup
  float Dterm = D * (Error - PrevError) / TimeStep;
  float PIDOutput = Pterm + Iterm + Dterm;
  PIDOutput = constrain(PIDOutput, -400, 400);
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

// PID reset function to clear previous errors and I-term
void reset_pid() {
  PrevErrorRateRoll = PrevItermRateRoll = 0;
  PrevErrorRatePitch = PrevItermRatePitch = 0;
  PrevErrorRateYaw = PrevItermRateYaw = 0;
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  // MPU6050 initialization and configuration
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(); delay(10);
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); delay(10);
  Wire.beginTransmission(0x68); Wire.write(0x19); Wire.write(0x03); Wire.endTransmission(); delay(10);
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); delay(10);

  // Gyro calibration routine
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw;
    delay(1);
  }

  RateCalibrationRoll  /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw   /= 2000;

  FilteredRateRoll  = RateCalibrationRoll;
  FilteredRatePitch = RateCalibrationPitch;
  FilteredRateYaw   = RateCalibrationYaw;

  // ESC initialization and calibration
  esc1.attach(MOTOR1); esc2.attach(MOTOR2); esc3.attach(MOTOR3); esc4.attach(MOTOR4);
  esc1.writeMicroseconds(1000); esc2.writeMicroseconds(1000); esc3.writeMicroseconds(1000); esc4.writeMicroseconds(1000);
  delay(3000);

  // CRSF initialization for RC input
  crsf = new CRSFforArduino();
  if (!crsf->begin()) {
    while (1) delay(10);
  }
  crsf->setRcChannelsCallback(onReceiveRcChannels);

  LoopTimer = micros();
}

// Throttle PID Attenuation (TPA) parameters
const int TPA_BREAKPOINT = 1300;
const float TPA_FACTOR_AT_MAX = 0.65;

void loop() {
  crsf->update();
  gyro_signals();

  // Rate calculation and filtering
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  FilteredRateRoll  = applyEMA(RateRoll,  FilteredRateRoll,  EMA_ALPHA);
  FilteredRatePitch = applyEMA(RatePitch, FilteredRatePitch, EMA_ALPHA);
  FilteredRateYaw   = applyEMA(RateYaw,   FilteredRateYaw,   EMA_ALPHA);

  // Throttle PID Attenuation (TPA) scaling
  float tpa_scale = 1.0;
  if (throttle > TPA_BREAKPOINT) {
    float t = (throttle - TPA_BREAKPOINT) / (2000.0 - TPA_BREAKPOINT);
    tpa_scale = 1.0 - t * (1.0 - TPA_FACTOR_AT_MAX);
  }

  // Roll PID
  ErrorRateRoll = DesiredRateRoll - FilteredRateRoll;
  float scaledP = PRateRoll * tpa_scale;
  float scaledD = DRateRoll * tpa_scale;

  pid_equation(ErrorRateRoll, scaledP, IRateRoll, scaledD, PrevErrorRateRoll, PrevItermRateRoll);
  OutputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  // Pitch PID
  ErrorRatePitch = DesiredRatePitch - FilteredRatePitch;
  float scaledP_Pitch = PRatePitch * tpa_scale;
  float scaledD_Pitch = DRatePitch * tpa_scale;

  pid_equation(ErrorRatePitch, scaledP_Pitch, IRatePitch, scaledD_Pitch, PrevErrorRatePitch, PrevItermRatePitch);
  OutputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  // Yaw PID
  ErrorRateYaw = DesiredRateYaw - FilteredRateYaw;
  float scaledP_Yaw = PRateYaw * tpa_scale;
  float scaledD_Yaw = DRateYaw * tpa_scale;

  pid_equation(ErrorRateYaw, scaledP_Yaw, IRateYaw, scaledD_Yaw, PrevErrorRateYaw, PrevItermRateYaw);
  OutputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  // Motor output calculation and ESC signal sending
  if (armed) {
    float M1 = throttle - OutputRoll + OutputPitch + OutputYaw;
    float M2 = throttle - OutputRoll - OutputPitch - OutputYaw;
    float M3 = throttle + OutputRoll + OutputPitch - OutputYaw;
    float M4 = throttle + OutputRoll - OutputPitch + OutputYaw;

    M1 = constrain(M1, 1025, 2000);
    M2 = constrain(M2, 1025, 2000);
    M3 = constrain(M3, 1025, 2000);
    M4 = constrain(M4, 1025, 2000);

    esc1.writeMicroseconds((int)M1);
    esc2.writeMicroseconds((int)M2);
    esc3.writeMicroseconds((int)M3);
    esc4.writeMicroseconds((int)M4);
  } else {
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    reset_pid();
  }

  // Loop timing control
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}

