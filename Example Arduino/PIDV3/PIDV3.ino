#include <Wire.h>
#include <Servo.h>

Servo esc1, esc2, esc3, esc4;
#define MOTOR1 5
#define MOTOR2 6
#define MOTOR3 9
#define MOTOR4 10

// IMU variables
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber = 0;

// Timing
uint32_t LoopTimer;
const float TimeStep = 0.004; // 250Hz

// Serial input
float throttle = 1000;
float DesiredRateRoll = 0, DesiredRatePitch = 0, DesiredRateYaw = 0;

// PID variables
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0;
float PrevItermRateRoll = 0, PrevItermRatePitch = 0, PrevItermRateYaw = 0;
float PIDReturn[3];
float OutputRoll, OutputPitch, OutputYaw;

// PID constants
float PRateRoll = 0.6, IRateRoll = 3.5, DRateRoll = 0.03;
float PRatePitch = 0.6, IRatePitch = 3.5, DRatePitch = 0.03;
float PRateYaw = 2.0, IRateYaw = 12.0, DRateYaw = 0.0;

void readSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    double values[4]; int index = 0;
    char* ptr = strtok((char*)input.c_str(), ",");

    while (ptr && index < 4) {
      values[index++] = atof(ptr);
      ptr = strtok(NULL, ",");
    }

    if (index == 4) {
      throttle = constrain((int)values[0], 1000, 2000);
      DesiredRateRoll = values[1];
      DesiredRatePitch = values[2];
      DesiredRateYaw = values[3];
    } else {
      Serial.println("Invalid input! Format: throttle,roll_rate,pitch_rate,yaw_rate");
    }
  }
}

void gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x43);  // Start with Gyro X
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
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
  PrevErrorRateRoll = 0; PrevItermRateRoll = 0;
  PrevErrorRatePitch = 0; PrevItermRatePitch = 0;
  PrevErrorRateYaw = 0; PrevItermRateYaw = 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  // MPU6050 setup
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // Power management register
  Wire.write(0x00);  // Wake up
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // DLPF config
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);  // Gyro config
  Wire.write(0x08);
  Wire.endTransmission();

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

  esc1.attach(MOTOR1);
  esc2.attach(MOTOR2);
  esc3.attach(MOTOR3);
  esc4.attach(MOTOR4);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(3000);

  LoopTimer = micros();
}

void loop() {
  readSerialInput();
  gyro_signals();

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Roll PID
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  OutputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  // // Pitch PID
  // ErrorRatePitch = DesiredRatePitch - RatePitch;
  // pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  // OutputPitch = PIDReturn[0];
  // PrevErrorRatePitch = PIDReturn[1];
  // PrevItermRatePitch = PIDReturn[2];

  // // Yaw PID
  // ErrorRateYaw = DesiredRateYaw - RateYaw;
  // pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  // OutputYaw = PIDReturn[0];
  // PrevErrorRateYaw = PIDReturn[1];
  // PrevItermRateYaw = PIDReturn[2];

  if (throttle > 1050) {
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
    // Emergency stop
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    reset_pid();
  }

  // Debug output
  Serial.print("Throttle: "); Serial.print(throttle);
  Serial.print(" | RateRoll: "); Serial.print(RateRoll);
  Serial.print(" | Desired: "); Serial.print(DesiredRateRoll);
  Serial.print(" | PID: "); Serial.println(OutputRoll);

  while (micros() - LoopTimer < 4000); // 250 Hz
  LoopTimer = micros();
}
