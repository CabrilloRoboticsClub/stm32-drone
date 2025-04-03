#include "ArduPID.h"

ArduPID ROLLController;
ArduPID PITCHController;

double p = 0.5;  // Proportional Gain
double i = 0.0; // Integral Gain
double d = 0.0; // Derivative Gain
double ROLLoutput;
double PITCHoutput;

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1

// This is where our Euler orientation will be stored
// YAW    + Angle is LEFT ROTATION (NOT ABSOLUTE <-- We should only care about the 𐤃YAW here) 
// PITCH  + Angle is FORWARD ROTATION
// ROLL:  + Angle is RIGHT ROTATION
struct euler_t {
  double yaw;
  double pitch;
  double roll;
} ypr;

struct target_euler_t {
  int throttle = 1000;
  double yaw;
  double pitch = 0.0;
  double roll = 0.0;
} target_states;


Adafruit_BNO08x  bno08x(BNO08X_RESET);
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
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}


#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

// Defining Motor's pin association
#define MOTOR1 5  // Back Right
#define MOTOR2 6  // Front Right
#define MOTOR3 9  // Back Left
#define MOTOR4 10 // Front Left

double T1 = 1000;
double T2 = 1000;
double T3 = 1000;
double T4 = 1000;


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

  Serial.println("Reading events");
  delay(100);

  PITCHController.begin(&ypr.pitch, &PITCHoutput, &target_states.pitch, p, i, d);
  PITCHController.setOutputLimits(-15.0, 15.0);

  ROLLController.begin(&ypr.roll, &ROLLoutput, &target_states.roll, p, i, d);
  ROLLController.setOutputLimits(-15.0, 15.0);


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

void loop() {
  
  // Thorttle
    if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Read full line
    input.trim();  // Remove whitespace/newlines

    // Split input into values
    double values[4]; // Array for throttle, yaw, pitch, roll
    int index = 0;
    char *ptr = strtok((char*)input.c_str(), ","); // Tokenize string

    while (ptr != NULL && index < 4) {
      values[index] = atof(ptr); // Convert to double
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
      if (target_states.throttle < 1000 || target_states.throttle > 2000) {
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
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }

    PITCHController.compute();
    ROLLController.compute();
    
    // For Debugging!
    static long last = 0;
    long now = micros();
    Serial.print(now - last);             Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t"); // There is some wierd stuff with Pitch here
    Serial.print(ypr.roll);               Serial.print("\t|\t");
    
    Serial.print(target_states.throttle); Serial.print("\t");
    Serial.print(target_states.yaw);      Serial.print("\t");
    Serial.print(target_states.pitch);    Serial.print("\t"); // There is some wierd stuff with Pitch here
    Serial.print(target_states.roll);     Serial.print("\t|\t");
    Serial.print(PITCHoutput/100);        Serial.print("\t");
    Serial.print(ROLLoutput / 100);       Serial.print("\t|\t");


  	// Motor Powers!
    //1 Back Right
    T1 = target_states.throttle + target_states.throttle*(PITCHoutput/100) - target_states.throttle*(ROLLoutput/100);

    //2 Front Right
    T2 = target_states.throttle - target_states.throttle*(PITCHoutput/100) - target_states.throttle*(ROLLoutput/100);

    //3 Back Left
    T3 = target_states.throttle + target_states.throttle*(PITCHoutput/100) + target_states.throttle*(ROLLoutput/100);

    //4 Front Left
    T4 = target_states.throttle - target_states.throttle*(PITCHoutput/100) + target_states.throttle*(ROLLoutput/100);
    if (target_states.throttle != 1000){
      esc1.writeMicroseconds(T1);  
      esc2.writeMicroseconds(T2);
      esc3.writeMicroseconds(T3);
      esc4.writeMicroseconds(T4);
    }
    else{
      esc1.writeMicroseconds(1000);  
      esc2.writeMicroseconds(1000);
      esc3.writeMicroseconds(1000);
      esc4.writeMicroseconds(1000);
    }
    Serial.print(T1);                     Serial.print("\t");
    Serial.print(T2);                     Serial.print("\t");
    Serial.print(T3);                     Serial.print("\t");
    Serial.println(T4);

 
  }
}
