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

int ang;
int lin;




void setup() {
  Serial.begin(115200);
  steeringServo.attach(steerPin,MIN,MAX);
  steeringServo.write(40); //center wheels
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  if(Serial.available() > 0){
    String data = Serial.readStringUntil('\n');
    sscanf(data.c_str(), "%d,%d", &ang, &lin);

    if(lin == 0){
      stop();
    } else {
      motion(lin);
    }
    steer(ang);
  }
}

void steer(int dir){
  steeringServo.write(map(dir,-100,100,MIN_ANGLE,MAX_ANGLE));
}

void motion(int speed){
  if (speed > 0){ //drive forwards
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, lin);
  } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, -lin);
  }
}

void stop(){
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      delay(250);

}
