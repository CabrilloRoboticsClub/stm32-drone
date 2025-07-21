#include <Wire.h>
#include <Servo.h>
#include "CRSFforArduino.hpp"

Servo esc1, esc2, esc3, esc4;
#define MOTOR1 5
#define MOTOR2 6
#define MOTOR3 9
#define MOTOR4 10

// IMU variables
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber = 0;

// Filtered values
float FilteredRatePitch = 0, FilteredRateRoll = 0, FilteredRateYaw = 0;
const float EMA_ALPHA = 0.1; // Lower = smoother, higher = more responsive

// Timing
uint32_t LoopTimer;
const float TimeStep = 0.004; // 250Hz

// PID variables
float throttle = 1000;
float DesiredRateRoll = 0, DesiredRatePitch = 0, DesiredRateYaw = 0;
bool armed = false;

float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0;
float PrevItermRateRoll = 0, PrevItermRatePitch = 0, PrevItermRateYaw = 0;
float PIDReturn[3];
float OutputRoll, OutputPitch, OutputYaw;

float PRateRoll = 0.5, IRateRoll = 3.2, DRateRoll = 0.02;
float PRatePitch = 0.5, IRatePitch = 3.2, DRatePitch = 0.02;
float PRateYaw = 2.0, IRateYaw = 12.0, DRateYaw = 0.0;

// Debug variables for PID terms
float debugPterm, debugIterm, debugDterm;

// CRSF
CRSFforArduino *crsf = nullptr;

float applyEMA(float newValue, float prevEMA, float alpha) {
  return alpha * newValue + (1.0 - alpha) * prevEMA;
}

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels) {
  if (rcChannels->failsafe) return;

  int rawThrottle = crsf->rcToUs(crsf->getChannel(3)); // T
  int rawRoll     = crsf->rcToUs(crsf->getChannel(1)); // A
  int rawArm      = crsf->rcToUs(crsf->getChannel(5)); // Aux1

  armed = rawArm > 1500;

  if (armed) {
    throttle = constrain(rawThrottle, 1000, 2000);
    DesiredRateRoll = map(rawRoll, 1000, 2000, -150, 150);  // scale to deg/sec
  } else {
    throttle = 1000;
    DesiredRateRoll = 0;
  }
}

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
  RateYaw   = (float)GyroZ / 65.5;
}

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * TimeStep / 2;
  Iterm = constrain(Iterm, -400, 400);
  float Dterm = D * (Error - PrevError) / TimeStep;
  float PIDOutput = Pterm + Iterm + Dterm;
  PIDOutput = constrain(PIDOutput, -400, 400);
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid() {
  PrevErrorRateRoll = PrevItermRateRoll = 0;
  PrevErrorRatePitch = PrevItermRatePitch = 0;
  PrevErrorRateYaw = PrevItermRateYaw = 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  // MPU6050 init
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(); delay(10);
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); delay(10);
  Wire.beginTransmission(0x68); Wire.write(0x19); Wire.write(0x03); Wire.endTransmission(); delay(10);
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); delay(10);

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  FilteredRateRoll = RateCalibrationRoll;
  FilteredRatePitch = RateCalibrationPitch;
  FilteredRateYaw = RateCalibrationYaw;

  esc1.attach(MOTOR1); esc2.attach(MOTOR2); esc3.attach(MOTOR3); esc4.attach(MOTOR4);
  esc1.writeMicroseconds(1000); esc2.writeMicroseconds(1000); esc3.writeMicroseconds(1000); esc4.writeMicroseconds(1000);
  delay(3000);

  crsf = new CRSFforArduino();
  if (!crsf->begin()) {
    Serial.println("CRSF init failed");
    while (1) delay(10);
  }
  crsf->setRcChannelsCallback(onReceiveRcChannels);

  LoopTimer = micros();

  Serial.println("Throttle,RateRoll,DesiredRoll,PID,Pterm,Iterm,Dterm,TPA");

}

const int TPA_BREAKPOINT = 1200;
const float TPA_FACTOR_AT_MAX = 0.45;

void loop() {
  crsf->update();
  gyro_signals();

  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  FilteredRateRoll  = applyEMA(RateRoll,  FilteredRateRoll,  EMA_ALPHA);
  FilteredRatePitch = applyEMA(RatePitch, FilteredRatePitch, EMA_ALPHA);
  FilteredRateYaw   = applyEMA(RateYaw,   FilteredRateYaw,   EMA_ALPHA);

  // Calculate TPA scale for P and D
  float tpa_scale = 1.0;
  if (throttle > TPA_BREAKPOINT) {
    float t = (throttle - TPA_BREAKPOINT) / (2000.0 - TPA_BREAKPOINT);
    tpa_scale = 1.0 - t * (1.0 - TPA_FACTOR_AT_MAX);  // linear scale from 1 to 0.45
  }

  // Roll PID
  ErrorRateRoll = DesiredRateRoll - FilteredRateRoll;
  float scaledP = PRateRoll * tpa_scale;
  float scaledD = DRateRoll * tpa_scale;

  // Calculate PID terms for debug before calling pid_equation
  debugPterm = scaledP * ErrorRateRoll;
  debugIterm = PrevItermRateRoll + IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * TimeStep / 2;
  debugIterm = constrain(debugIterm, -400, 400);
  debugDterm = scaledD * (ErrorRateRoll - PrevErrorRateRoll) / TimeStep;

  pid_equation(
    ErrorRateRoll,
    scaledP,
    IRateRoll,
    scaledD,
    PrevErrorRateRoll,
    PrevItermRateRoll
  );
  OutputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  // TODO: You can add similar PID calls and debug prints for Pitch and Yaw if needed

  if (armed) {
    float M1 = throttle - OutputRoll - OutputPitch - OutputYaw;
    float M2 = throttle - OutputRoll + OutputPitch + OutputYaw;
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

  //Serial.print("Throttle: "); Serial.print(throttle);
  //Serial.print(" | RateRoll (filtered): "); Serial.print(FilteredRateRoll);
  //Serial.print(" | Desired: "); Serial.print(DesiredRateRoll);
  //Serial.print(" | PID: "); Serial.print(OutputRoll);

  //Serial.print(" | Pterm: "); Serial.print(scaledP, 4);
  //Serial.print(" | Dterm: "); Serial.print(scaledD, 4);
  //Serial.print(" | TPA: "); Serial.println(tpa_scale, 4);

  Serial.print(throttle); Serial.print(",");
  Serial.print(FilteredRateRoll); Serial.print(",");
  Serial.print(DesiredRateRoll); Serial.print(",");
  Serial.print(OutputRoll); Serial.print(",");
  Serial.print(scaledP); Serial.print(",");
  Serial.print(scaledD); Serial.print(",");
  Serial.println(tpa_scale, 4);


  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
