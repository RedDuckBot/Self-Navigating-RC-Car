#include <Servo.h>

//L298N pins
#define enA 6
#define in1 12
#define in2 13

//Pulse width range
#define MIN 981 //(steering turned farthest left)
#define MAX 1776 //(steering turned farthest right)
#define MIN_ANGLE 0
#define MAX_ANGLE 80

Servo steeringServo; //rotate steering linkage

int steerPin = 10;

void setup() {
  Serial.begin(112500);
  // init servo
  steeringServo.attach(steerPin, MIN, MAX);
  steeringServo.write(40);
  // init motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  char steer_angle, motor_speed;

  // wait for drive data
  while(!Serial.available());
  steer_angle = Serial.read();
  motor_speed = Serial.read();

  if(motor_speed != 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, motor_speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    steer_angle = 0;
  }
  steeringServo.write(map(steer_angle, -100, 100, MIN_ANGLE, MAX_ANGLE));

}


























