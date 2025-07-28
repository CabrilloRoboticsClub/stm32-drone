#include <STM32_PWM.h>

// STM32F405 Adafruit Feather LED Blink Test

// This sketch blinks the onboard LED on the STM32F405 Feather board.
// It uses the onboard LED connected to pin D13.

#define LED_PIN D13 // Onboard LED on STM32F405 Feather

void setup() {
    pinMode(LED_PIN, OUTPUT); // Set LED pin as output
}

void loop() {
    digitalWrite(LED_PIN, HIGH); // Turn LED on
    delay(100); // Wait 500ms
    digitalWrite(LED_PIN, LOW);  // Turn LED off
    delay(100); // Wait 500ms
}

