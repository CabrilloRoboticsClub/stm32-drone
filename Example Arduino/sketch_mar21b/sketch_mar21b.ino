// #define TEST_PIN 23

// void setup() {
//   pinMode(TEST_PIN, OUTPUT); // Set pin TEST_PIN as an output
// }

// void loop() {
//   digitalWrite(TEST_PIN, HIGH); // Turn the LED on
//   delay(100);            // Wait for 1 second
//   digitalWrite(TEST_PIN, LOW);  // Turn the LED off
// //   delay(100);            // Wait for 1 second
// // }
// #include <Servo.h>

// Servo esc;  // Create a Servo object to control the ESC

// void setup() {
//   esc.attach(23);  // Attach the ESC to pin 23 (PWM-capable pin)
//   Serial.begin(9600);  // Start serial communication at 9600 baud rate
//   delay(2000);  // Wait for ESC to initialize
  
//   // You can calibrate the ESC if needed
//   esc.writeMicroseconds(1000);  // Minimum throttle (1ms pulse)
//   delay(3000);
//   esc.writeMicroseconds(2000);  // Maximum throttle (2ms pulse)
//   delay(3000);

//   esc.writeMicroseconds(1000);  // Neutral throttle (1.5ms pulse)
//   delay(3000);
//   Serial.println("ESC initialized. Send values between 1000 and 2000 for throttle control.");
// }

// void loop() {
//   if (Serial.available() > 0) {  // Check if data is available to read from serial
//     int value = Serial.parseInt();  // Read the integer from serial input
//     if (value >= 1000 && value <= 2000) {  // Ensure the value is within the valid range
//       esc.writeMicroseconds(value);  // Set the ESC throttle
//       Serial.print("ESC throttle set to: ");
//       Serial.println(value);  // Print the current throttle value
//     } else {
//       Serial.println("Invalid value! Please enter a number between 1000 and 2000.");
//     }
//   }
// }



#include <Servo.h>

Servo esc1, esc2, esc3, esc4;  // Create Servo objects to control the ESCs

void setup() {
  esc1.attach(9);  // Attach ESCs to their respective pins
  esc2.attach(10);
  esc3.attach(23);
  esc4.attach(24);
  
  Serial.begin(9600);  // Start serial communication at 9600 baud rate
  delay(2000);  // Wait for ESCs to initialize

  // ESC calibration sequence
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(3000);
  
  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  esc4.writeMicroseconds(2000);
  delay(3000);
  
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(3000);
  
  Serial.println("ESCs initialized. Send values between 1000 and 2000 for throttle control.");
}

void loop() {
  if (Serial.available() > 0) {  // Check if data seInt();  // Read the integer from serial input
    if (value >= 1000 && vais available to read from serial
    int value = Serial.parlue <= 2000) {  // Ensure the value is within the valid range
      esc1.writeMicroseconds(value);
      esc2.writeMicroseconds(value);
      esc3.writeMicroseconds(value);
      esc4.writeMicroseconds(value);
      Serial.print("ESC throttle set to: ");
      Serial.println(value);  // Print the current throttle value
    } else {
      Serial.println("Invalid value! Please enter a number between 1000 and 2000.");
    }
  }
}
