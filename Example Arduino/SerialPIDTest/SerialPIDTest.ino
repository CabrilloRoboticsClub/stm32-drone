#include "ArduPID.h"
#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Servo.h>

// ========== PID Setup ==========
ArduPID ROLLController;

double ROLL_P = 2.5;
double ROLL_I = 0.0;
double ROLL_D = 1000.0;

double ROLLoutput;

// ========== Orientation Data & Filtering ==========
struct euler_t {
  double yaw;
  double pitch;
  double roll;
} ypr;

// Filtered values and constants
struct euler_t yprFiltered;
double alpha = 0.2; // EMA smoothing factor (0<alpha<1)

struct target_euler_t {
  int throttle = 0;
  double roll = 0.0;
  int extra1 = 0;
  int extra2 = 0;
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
Servo esc1, esc2, esc3, esc4;

#define MOTOR1 5    // Back Right motor
#define MOTOR2 6    // Front Right motor
#define MOTOR3 9    // Back Left motor
#define MOTOR4 10   // Front Left motor

double T1, T2, T3, T4;

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

inline void applyFilter(euler_t* raw, euler_t* filt) {
  filt->yaw   = alpha * raw->yaw   + (1 - alpha) * filt->yaw;
  filt->pitch = alpha * raw->pitch + (1 - alpha) * filt->pitch;
  filt->roll  = alpha * raw->roll  + (1 - alpha) * filt->roll;
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rv, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rv->real, rv->i, rv->j, rv->k, ypr, degrees);
}

// ========== Setup ==========
void setup() {
  target_states.throttle = 1000;
  delay(1000);

  esc1.attach(MOTOR1);
  esc2.attach(MOTOR2);
  esc3.attach(MOTOR3);
  esc4.attach(MOTOR4);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(3000);

  Serial.begin(9600);
  Serial.println("4-Motor Roll Control Init with IMU Filtering");

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  bno08x.hardwareReset();
  Serial.println("BNO08x Found!");
  setReports(reportType, reportIntervalUs);

  // Initialize filtered values to zero
  yprFiltered.yaw = yprFiltered.pitch = yprFiltered.roll = 0.0;

  // Initialize PID controller for roll using filtered roll
  ROLLController.begin(&yprFiltered.roll, &ROLLoutput, &target_states.roll, ROLL_P, ROLL_I, ROLL_D);
  ROLLController.setOutputLimits(-200.0, 200.0);
}

// ========== Main Loop ==========
void loop() {
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
      target_states.throttle = (int)values[0];
      target_states.roll = values[1];
      target_states.extra1 = (int)values[2];
      target_states.extra2 = (int)values[3];

      if (target_states.throttle < 1000 || target_states.throttle > 2000) {
        Serial.println("Invalid throttle! Enter 1000-2000.");
      } else {
        Serial.print("Throttle: "); Serial.print(target_states.throttle);
        Serial.print(", Roll: "); Serial.print(target_states.roll);
        Serial.print(", Extra1: "); Serial.print(target_states.extra1);
        Serial.print(", Extra2: "); Serial.println(target_states.extra2);
      }
    } else {
      Serial.println("Invalid input! Format: throttle,roll,extra1,extra2");
    }
  }

  if (bno08x.wasReset()) setReports(reportType, reportIntervalUs);

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      applyFilter(&ypr, &yprFiltered);
    }

    if (target_states.throttle != 0) {
      ROLLController.compute();
    }

    // Debug Print
    Serial.print("Filtered Roll: "); Serial.print(yprFiltered.roll);
    Serial.print(" Target: "); Serial.print(target_states.roll);
    Serial.print(" Output: "); Serial.print(ROLLoutput);

    // Actively return extra values
    Serial.print(" Extra1: "); Serial.print(target_states.extra1);
    Serial.print(" Extra2: "); Serial.println(target_states.extra2);

    // Motor mixing
    T1 = constrain(target_states.throttle + ROLLoutput, 1025, 2000);
    T2 = constrain(target_states.throttle + ROLLoutput, 1025, 2000);
    T3 = constrain(target_states.throttle - ROLLoutput, 1025, 2000);
    T4 = constrain(target_states.throttle - ROLLoutput, 1025, 2000);

    if (target_states.throttle != 1000) {
      esc1.writeMicroseconds((int)T1);
      esc2.writeMicroseconds((int)T2);
      esc3.writeMicroseconds((int)T3);
      esc4.writeMicroseconds((int)T4);
    } else {
      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
      esc3.writeMicroseconds(1000);
      esc4.writeMicroseconds(1000);
    }
  }
}

