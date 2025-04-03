#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

// Defining Motor's pin association
#define MOTOR1 = 5  // Back Right
#define MOTOR2 = 6  // Front Right
#define MOTOR3 = 9  // Back Left
#define MOTOR4 = 10 // Front Left

// Defining Motor's spin direction for yaw control
// 1 Means motors are rotating inwards (props in), -1 means rotating out (props out)
#define MOTOR_DIRECTION = 1


void setup() {
  Serial.begin(9600);
  delay(1000);  // Wait for ESCs to power up

  // Attach ESCs to their respective pins
  esc1.attach(MOTOR1);
  esc2.attach(MOTOR2);
  esc3.attach(MOTOR3);
  esc4.attach(MOTOR4);

  // ESC Calibration: 1000us min, and 2000us max
  Serial.println("Calibrating ESCs...");
  Serial.println("Step 1: Sending maximum throttle (2000 µs).");
  Serial.println("Please turn on your ESCs *now* if required.");
  Serial.println("Press Enter to continue if ESC has \"angrily beeped\"...");

  // Step 1: Send max throttle
  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  esc4.writeMicroseconds(2000);

  // Wait for user input to proceed
  while (Serial.available() == 0) {
    // Do nothing, wait for user input
  }
  Serial.read();  // Clear the buffer

  Serial.println("Continuing calibration...");

  // Step 2: Send min throttle
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(3000);  // Wait for ESC to register min throttle

  Serial.println("ESCs initialized. Enter a throttle value between 1000-2000.");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove whitespace/newlines
    
    if (input.length() > 0) {
      int value = input.toInt();
      if (value >= 1000 && value <= 2000) {
        esc1.writeMicroseconds(value);
        esc2.writeMicroseconds(value);
        esc3.writeMicroseconds(value);
        esc4.writeMicroseconds(value);
        Serial.print("ESC throttle set to: ");
        Serial.println(value);
      } else {
        Serial.println("Invalid value! Enter a number between 1000 and 2000.");
      }
    }
  }
}
