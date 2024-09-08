#include <Arduino.h>
#include <Servo.h>

const int servoPin = 2;

Servo servo;

const int enaPin = 3;
const int in1Pin = 7;
const int in2Pin = 8;

int current_speed = 0;
int set_speed = 0;
int acc_time = 3;
int last_acc_time = 0;


int middle = 70; // +55 -55
int degree_max = 125;
int degree_min = 15;
int current_degree = 0;
int set_degree = 0;

void drive(int speed)
{
  if (speed > 255)
  {
    speed = 255;
  }
  else if (speed < -255)
  {
    speed = -255;
  }
  if (millis() - last_acc_time < acc_time)
  {
    return;
  }
  last_acc_time = millis();
  if (abs(speed - current_speed) > 1)
  {
    current_speed = current_speed + (speed - current_speed) / fabs(speed - current_speed) * 1;
  }
  else if (speed == 0)
  {
    current_speed = 0;
  }
  if (current_speed > 0)
  {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  else if (current_speed < 0)
  {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  }
  else
  {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
  analogWrite(enaPin, abs(current_speed));
}

void steer(int angle)
{
  angle = angle + middle;
  if (angle > degree_max)
  {
    angle = degree_max;
  }
  else if (angle < degree_min)
  {
    angle = degree_min;
  }
  servo.write(angle);
}

void setup()
{
  pinMode(enaPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  servo.attach(servoPin);

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(enaPin, 0);
  Serial.begin(9600);
  
}

int a = 0;

void loop()
{
  Serial.println(set_degree);
  if (Serial.available() > 0)
  {
    a = Serial.read();
    if (a == '1')
    {
      set_degree++;
    }
    else if (a == '2')
    {
      set_degree--;
    }
  }
  steer(set_degree);
  drive(set_speed);
}
