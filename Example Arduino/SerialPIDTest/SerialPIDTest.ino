#include "ArduPID.h"

ArduPID YAWController;
ArduPID ROLLController;
ArduPID PITCHController;

double YAW_P = 50.0;  // Proportional Gain
double YAW_I = 0.00;  // Integral Gain
double YAW_D = 50.0;   // Derivative Gain

double ROLL_P = 50.0;   // Derivative Gain
double ROLL_I = 0.00;   // Derivative Gain
double ROLL_D = 50.0;   // Derivative Gain

double PITCH_P = 50.0;      // Proportional Gain
double PITCH_I = 0.00;      // Integral Gain
double PITCH_D = 50.0;       // Derivative Gain

double YAWoutput;
double ROLLoutput;
double PITCHoutput;


#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1

// Euler orientation stored
struct euler_t {
  double yaw;    // YAW    + Angle is LEFT ROTATION <-- We will mostly ignore this for now
  double pitch;  // PITCH  + Angle is FORWARD ROTATION
  double roll;   // ROLL:  + Angle is RIGHT ROTATION

} ypr;

// Target states orientation stored
struct target_euler_t {
  int throttle = 0.0;
  double yaw = 0.0;  // Note this one is in (rads/s), not an abs location (+ Rotation is LEFT)
  double pitch = 0.0;
  double roll = 0.0;
} target_states;

double yaw_rotation = 0.0;


Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}


#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

// Defining Motor's pin association
#define MOTOR1 5   // Back Right
#define MOTOR2 6   // Front Right
#define MOTOR3 9   // Back Left
#define MOTOR4 10  // Front Left

double T1;
double T2;
double T3;
double T4;

double Unmapped_T1 = 0;
double Unmapped_T2 = 0;
double Unmapped_T3 = 0;
double Unmapped_T4 = 0;



// Defining Motor's spin direction. (This is relevent for YAW control)
// 1 Means motors are rotating inwards (props in), -1 means rotating out (props out)
#define MOTOR_DIRECTION 1

double throttle;

void setup() {
  // ESC SETUP (Note: ESC Calibration must be done before hand, see MotorDrive.ino)
  delay(1000);  // Wait for ESC to power up

  // Attach ESCs to their respective pins
  esc1.attach(MOTOR1);
  esc2.attach(MOTOR2);
  esc3.attach(MOTOR3);
  esc4.attach(MOTOR4);

  // Send min throttle
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(3000);  // Wait for ESC to register min throttle

  // Open Serial Port (For Testing Only)
  Serial.begin(9600);
  Serial.println("ESC initialized.");

  // IMU SETUP
  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);
  if (!bno08x.enableReport(SH2_GYRO_INTEGRATED_RV)) {
    Serial.println("Could not enable gyroscope report");
    while (1)
      ;
  }


  Serial.println("Reading events");
  delay(100);

  YAWController.begin(&yaw_rotation, &YAWoutput, &target_states.yaw, YAW_P, YAW_I, YAW_D);
  YAWController.setOutputLimits(-200, 200);

  PITCHController.begin(&ypr.pitch, &PITCHoutput, &target_states.pitch,PITCH_P, PITCH_I, PITCH_D);
  PITCHController.setOutputLimits(-200.0, 200.0);

  ROLLController.begin(&ypr.roll, &ROLLoutput, &target_states.roll, ROLL_P, ROLL_I, ROLL_D);
  ROLLController.setOutputLimits(-200.0, 200.0);
}
// Functions for IMU Computation
void quaternionToEuler(double qr, double qi, double qj, double qk, euler_t* ypr, bool degrees = false) {

  double sqr = sq(qr);
  double sqi = sq(qi);
  double sqj = sq(qj);
  double sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void loop() {

  // Thorttle
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read full line
    input.trim();                                 // Remove whitespace/newlines

    // Split input into values
    double values[4];  // Array for throttle, yaw, pitch, roll
    int index = 0;
    char* ptr = strtok((char*)input.c_str(), ",");  // Tokenize string

    while (ptr != NULL && index < 4) {
      values[index] = atof(ptr);  // Convert to double
      ptr = strtok(NULL, ",");
      index++;
    }

    // Ensure we received exactly 4 values
    if (index == 4) {
      target_states.throttle = values[0];
      target_states.yaw = values[1];
      target_states.pitch = values[2];
      target_states.roll = values[3];

      // Validate throttle range
      if (target_states.throttle <= 1000 || target_states.throttle > 2000) {
        Serial.println("Invalid throttle! Enter a number between 1000 and 2000.");
      } else {
        Serial.print("Throttle: ");
        Serial.print(target_states.throttle);
        Serial.print(", Yaw: ");
        Serial.print(target_states.yaw);
        Serial.print(", Pitch: ");
        Serial.print(target_states.pitch);
        Serial.print(", Roll: ");
        Serial.println(target_states.roll);
      }
    } else {
      Serial.println("Invalid input! Format: throttle,yaw,pitch,roll");
    }
  }


  // IMU
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);

      case SH2_GYRO_INTEGRATED_RV:
        // float x = sensorValue.un.gyroIntegratedRV.angVelX;
        // float y = sensorValue.un.gyroIntegratedRV.angVelY;
        yaw_rotation = sensorValue.un.gyroIntegratedRV.angVelZ;

        // Serial.print("Angular Velocity (rad/s) - X: ");
        // Serial.print(x, 5);
        // Serial.print(", Y: ");
        // Serial.print(y, 5);
        Serial.print("Yaw Velocity (rad/s) - Z: ");
        Serial.print(yaw_rotation, 5);  // use println to add a newline

        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }

    if (target_states.throttle != 0) {
      YAWController.compute();
      PITCHController.compute();
      ROLLController.compute();
    }
    // For Debugging!
    static long last = 0;
    long now = micros();
    Serial.print(now - last);
    Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);
    Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);
    Serial.print("\t");
    Serial.print(ypr.pitch);
    Serial.print("\t");  // There is some wierd stuff with Pitch here
    Serial.print(ypr.roll);
    Serial.print("\t|\t");

    Serial.print(target_states.throttle);
    Serial.print("\t");
    Serial.print(target_states.yaw);
    Serial.print("\t");
    Serial.print(target_states.pitch);
    Serial.print("\t");  // There is some wierd stuff with Pitch here
    Serial.print(target_states.roll);
    Serial.print("\t|\t");
    Serial.print(YAWoutput);
    Serial.print("\t");
    Serial.print(PITCHoutput);
    Serial.print("\t");
    Serial.print(ROLLoutput);
    Serial.print("\t|\t");

    // Motor Powers!
    //1 Back Right
    T1 = (target_states.throttle + PITCHoutput - ROLLoutput - YAWoutput);

    //2 Front Right
    T2 = (target_states.throttle - PITCHoutput - ROLLoutput + YAWoutput);

    //3 Back Left
    T3 = (target_states.throttle + PITCHoutput + ROLLoutput - YAWoutput);

    //4 Front Left
    T4 = (target_states.throttle - PITCHoutput + ROLLoutput + YAWoutput);


    if (target_states.throttle != 1000) {

      T1 = constrain(T1, 1025, 2000);
      T2 = constrain(T2, 1025, 2000);
      T3 = constrain(T3, 1025, 2000);
      T4 = constrain(T4, 1025, 2000);

    } else {

      T1 = 1000;
      T2 = 1000;
      T3 = 1000;
      T4 = 1000;

    }

    esc1.writeMicroseconds(T1);
    esc2.writeMicroseconds(T2);
    esc3.writeMicroseconds(T3);
    esc4.writeMicroseconds(T4);

    Serial.print(T1);
    Serial.print("\t");
    Serial.print(T2);
    Serial.print("\t");
    Serial.print(T3);
    Serial.print("\t");
    Serial.println(T4);
  }
}
