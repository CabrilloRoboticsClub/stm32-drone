#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Servo.h>

// PID Setup
float RateRoll, RatePitch, RateYaw;
float DesiredRateRoll = 0.0;
float ErrorRateRoll, PrevErrorRateRoll = 0, PrevItermRateRoll = 0;
float OutputRoll;
float PIDReturn[3];

// Rate PID constants
float PRateRoll = 0.6;
float IRateRoll = 3.5;
float DRateRoll = 0.02;

float TimeStep = 0.004; // 250Hz

uint32_t LoopTimer;

// Angle PID Setup
float rawRollAngle = 0.0;       // raw IMU roll in degrees
float filteredRollAngle = 0.0;  // filtered roll angle
float alpha = 0.1;              // EMA smoothing factor

float targetRollAngle = 0.0;    // user input in degrees
bool angleMode = false;

// Angle PID state variables
float PrevErrorAngleRoll = 0;
float PrevItermAngleRoll = 0;
float OutputAnglePID = 0;

// Angle PID constants
float PAngleRoll = 15;
float IAngleRoll = 0.0;
float DAngleRoll = 1;

// IMU Setup
Adafruit_BNO08x bno08x;

// ESC Setup
Servo esc1, esc2, esc3, esc4;
#define MOTOR1 5
#define MOTOR2 6
#define MOTOR3 9
#define MOTOR4 10

float throttle = 1000;
float T1, T2, T3, T4;

// Serial Input
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

      if (angleMode) {
        targetRollAngle = constrain(values[1], -45.0, 45.0); // safety clamp
      } else {
        DesiredRateRoll = values[1]; // °/s
      }
    } else {
      Serial.println("Invalid input! Format: throttle,roll_target,extra1,extra2");
    }
  }
}

// PID function
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

void readIMU() {
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
    bno08x.enableReport(SH2_ROTATION_VECTOR);
  }

  sh2_SensorValue_t sensorValue;
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      RateRoll = sensorValue.un.gyroscope.x * 180.0 / PI;
    } else if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      // Get raw roll angle from quaternion
      float qr = sensorValue.un.rotationVector.real;
      float qi = sensorValue.un.rotationVector.i;
      float qj = sensorValue.un.rotationVector.j;
      float qk = sensorValue.un.rotationVector.k;

      rawRollAngle = atan2(2.0 * (qj * qk + qi * qr), -qi*qi - qj*qj + qk*qk + qr*qr) * RAD_TO_DEG;

      // EMA filter
      filteredRollAngle = alpha * rawRollAngle + (1 - alpha) * filteredRollAngle;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("PID + ESC + Serial + Angle Mode with IMU Filtering Init");

  Wire.begin();
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1);
  }
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
  bno08x.enableReport(SH2_ROTATION_VECTOR);
  delay(100);
  LoopTimer = micros();

  esc1.attach(MOTOR1);
  esc2.attach(MOTOR2);
  esc3.attach(MOTOR3);
  esc4.attach(MOTOR4);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(3000);

  filteredRollAngle = 0.0;
}

void loop() {
  readSerialInput();
  readIMU();

  if (throttle > 1000) {
    if (angleMode) {
      float angleError = targetRollAngle - filteredRollAngle;
      pid_equation(angleError, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
      OutputAnglePID = PIDReturn[0];
      PrevErrorAngleRoll = PIDReturn[1];
      PrevItermAngleRoll = PIDReturn[2];

      DesiredRateRoll = constrain(OutputAnglePID, -100, 100);
    }

    ErrorRateRoll = DesiredRateRoll - RateRoll;
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    OutputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    T1 = constrain(throttle - OutputRoll, 1025, 2000);
    T2 = constrain(throttle - OutputRoll, 1025, 2000);
    T3 = constrain(throttle + OutputRoll, 1025, 2000);
    T4 = constrain(throttle + OutputRoll, 1025, 2000);

    esc1.writeMicroseconds((int)T1);
    esc2.writeMicroseconds((int)T2);
    esc3.writeMicroseconds((int)T3);
    esc4.writeMicroseconds((int)T4);
  } else {
    // Stop motors immediately if throttle too low
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
  }

  // Debug output
  Serial.print("Roll (filtered): "); Serial.print(filteredRollAngle);
  Serial.print(" | Target Angle: "); Serial.print(targetRollAngle);
  Serial.print(" | DesiredRate: "); Serial.print(DesiredRateRoll);
  Serial.print(" | Rate: "); Serial.print(RateRoll);
  Serial.print(" | PID: "); Serial.println(OutputRoll);

  while (micros() - LoopTimer < 4000); // 250Hz
  LoopTimer = micros();
}

