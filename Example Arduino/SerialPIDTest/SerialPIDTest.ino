#include "ArduPID.h"
#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Servo.h>

// ========== PID Setup ==========
ArduPID PITCHController;

double PITCH_P = 1.5;
double PITCH_I = 0.0;
double PITCH_D = 400.0;

double PITCHoutput;

// ========== Orientation Data ==========
struct euler_t {
  double yaw;
  double pitch;
  double roll;
} ypr;

struct target_euler_t {
  int throttle = 0;
  double pitch = 0.0;
} target_states;

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable report");
  }
}

// ========== ESC + Motor Setup ==========
Servo esc1, esc2;

#define MOTOR_FRONT 5
#define MOTOR_BACK 6

double T1; // Back
double T2; // Front

// ========== Setup ==========
void setup() {
  delay(1000);

  esc1.attach(MOTOR_BACK);
  esc2.attach(MOTOR_FRONT);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  delay(3000);

  Serial.begin(9600);
  Serial.println("2-Motor Pitch Control Init");

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");
  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);

  PITCHController.begin(&ypr.pitch, &PITCHoutput, &target_states.pitch, PITCH_P, PITCH_I, PITCH_D);
  PITCHController.setOutputLimits(-200.0, 200.0);
}

// ========== Quaternion to Euler ==========
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

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rv, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rv->real, rv->i, rv->j, rv->k, ypr, degrees);
}

// ========== Main Loop ==========
void loop() {
  // Serial input format: throttle,pitch
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    double values[2];
    int index = 0;
    char* ptr = strtok((char*)input.c_str(), ",");

    while (ptr != NULL && index < 2) {
      values[index++] = atof(ptr);
      ptr = strtok(NULL, ",");
    }

    if (index == 2) {
      target_states.throttle = values[0];
      target_states.pitch = values[1];

      if (target_states.throttle <= 1000 || target_states.throttle > 2000) {
        Serial.println("Invalid throttle! Enter a number between 1000 and 2000.");
      } else {
        Serial.print("Throttle: ");
        Serial.print(target_states.throttle);
        Serial.print(", Pitch: ");
        Serial.println(target_states.pitch);
      }
    } else {
      Serial.println("Invalid input! Format: throttle,pitch");
    }
  }

  // Read IMU data
  if (bno08x.wasReset()) {
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
    }

    if (target_states.throttle != 0) {
      PITCHController.compute();
    }

    // Debug Print
    Serial.print("Pitch: ");
    Serial.print(ypr.pitch);
    Serial.print("\tTarget: ");
    Serial.print(target_states.pitch);
    Serial.print("\tOutput: ");
    Serial.println(PITCHoutput);

    // Calculate Motor Outputs
    T1 = target_states.throttle - PITCHoutput; // Back
    T2 = target_states.throttle + PITCHoutput; // Front

    // Clamp to ESC range
    T1 = constrain(T1, 1000, 2000);
    T2 = constrain(T2, 1000, 2000);

    esc1.writeMicroseconds((int)T1);
    esc2.writeMicroseconds((int)T2);
  }
}

