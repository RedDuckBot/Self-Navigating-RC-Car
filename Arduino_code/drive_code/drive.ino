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
  Serial.begin(115200 );
  steeringServo.attach(steerPin,MIN,MAX);
  steeringServo.write(40); //wheels centered
  //Initialize motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  char incoming_msg;
  int steer_angle, motor_speed;

  //Wait on drive data
  //Get a command by reading one char
  while (!Serial.available());
  incoming_msg = Serial.read();

  if (incoming_msg == 'F' || incoming_msg == 'B') {
    //Move forward or backward 
    motor_speed = Serial.parseInt();
    motion(motor_speed,incoming_msg);
   } 

  if (incoming_msg == 'S') {
    //Update steering angle
    steer_angle = Serial.parseInt();
    control_servo(steer_angle);
  }

   if (incoming_msg == 'D') {
    //Disabled mode (MacNav stationary)
    control_servo(0);
    applyBrake();
   }
   
 }

//Parameter --angle is associated with controller input value
void control_servo(int angle)
{
  int new_angle;
  
  if ((angle > -0.20) && (angle < 0.20))
  {
     steeringServo.write(40);
     return;
  }
  new_angle = map(angle,-100,100,MIN_ANGLE,MAX_ANGLE);
  steeringServo.write(new_angle);
}

//Parameter ---pwmSpeed is controller input
//Parameter ---command is forward or backward
void motion(int pwmSpeed, char command)
{
  if (command == 'F') {
    //Move forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    //Move backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(enA, pwmSpeed);
  delay(250);
}

void applyBrake()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
 delay(250);
}
