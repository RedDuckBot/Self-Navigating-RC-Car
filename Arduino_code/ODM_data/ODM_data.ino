#include <Wire.h>

// Define the digital pins to which the encoder is connected
const int encoderChannelA = 2;
const int encoderChannelB = 3;

// Variables for encoder counting
float encoderCount = 0.0;
unsigned long lastTime = 0;    // Store the last time the count was read
unsigned long deltaTime = 100; // Set the time interval for RPM calculation (in milliseconds)

void setup() {
  // Initialize the serial communication for debugging
  Serial.begin(115200);

  // Set the encoder channel pins as inputs
  pinMode(encoderChannelA, INPUT);
  pinMode(encoderChannelB, INPUT);
  
  // Attach interrupt service routines to the pins for detecting changes
  attachInterrupt(digitalPinToInterrupt(encoderChannelA), encoderISR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderChannelB), encoderISR, INPUT_PULLUP);
}

void loop() {
  unsigned long currentTime = millis();
  // Calculate RPM at regular intervals
  if (currentTime - lastTime >= deltaTime) {
    Serial.println(encoderCount);
    
    encoderCount = 0; // Reset the count

    lastTime = currentTime;

  }

}

// Interrupt Service Routine (ISR) for the encoder
void encoderISR() {
  encoderCount++;
}
