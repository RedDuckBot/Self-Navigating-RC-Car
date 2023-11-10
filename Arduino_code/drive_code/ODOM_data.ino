#define ENCODER_PPR 100 //For one revolution of input shaft for motor 100 pulses is created on on channel

/*This sketch sends serial data related to magneto and encoder sensors*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// Define the digital pins to which the encoder is connected
const int encoderChannelA = 2;
const int encoderChannelB = 3;

// Variables for encoder counting
float encoderCount = 0.0;
unsigned long lastTime = 0;    // Store the last time the count was read
unsigned long deltaTime = 1000; // Set the time interval for RPM calculation (in milliseconds)

float distance = 0.0000000;
//float R = 0.01 * (1/30) * (17/70) * (15/39);  //Gear train ratios
float R = 0.000015568;  // R / 2

float wheel_radius = 0.0402;  //Meters

void setup() {
  // Initialize the serial communication for debugging
  Serial.begin(115200);

  // Set the encoder channel pins as inputs
  pinMode(encoderChannelA, INPUT);
  pinMode(encoderChannelB, INPUT);

  // Attach interrupt service routines to the pins for detecting changes
  attachInterrupt(digitalPinToInterrupt(encoderChannelA), encoderISR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderChannelB), encoderISR, INPUT_PULLUP);
  /*
   if (!mag.begin()) {
    Serial.println("Didn't find the LSM303 sensor. Check wiring.");
    while(1);
  }*/
}

void loop() {
  unsigned long currentTime = millis();

  // Calculate RPM at regular intervals
  if (currentTime - lastTime >= deltaTime) {
      sensors_event_t event;
      mag.getEvent(&event);
    // Read the encoder count
    noInterrupts(); // Disable interrupts to prevent data corruption
    float currentCount = encoderCount;

    // Calculate RPM, heading (relative to magnetic north) and current distance traveled
    float rpm = (float) ((currentCount * 60) / 5000);
    float temp_distance = encoderCount * R * 2.0 * PI * wheel_radius;
    float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / PI;
    
    //Serial.print("encoderCount ");
    //Serial.println(encoderCount);

    //Serial.print("temp_distance");
    //Serial.println(temp_distance);

    //Update total distance
    distance = distance + temp_distance;

     if (heading < 0) {
    heading = 360 + heading;
    }

    Serial.println(heading);

    // Display RPM
    //Serial.print("RPM: ");
    Serial.println(rpm);

    //Display Distance
    //Serial.print("Distance: ");
    Serial.println(distance);

    //Serial.println("\n\n");
    encoderCount = 0; // Reset the count


    lastTime = currentTime;
    interrupts(); // Re-enable interrupts
  }
}

// Interrupt Service Routine (ISR) for the encoder
void encoderISR() {
  encoderCount++;
}
