#include <Arduino.h>
#include <Servo.h>

const int servoPin = 2;

Servo servo;

const int enaPin = 11;
const int in1Pin = 5;
const int in2Pin = 6;

int current_speed = 0;
int set_speed = 0;
unsigned long acc_time = 20;
unsigned long last_acc_time = 0;

int middle = 70; // +55 -55
int degree_max = 125;
int degree_min = 15;
int current_degree = 0;
int set_degree = 0;

#define BUFFER_SIZE 64

char ringBuffer[BUFFER_SIZE];
int head = 0;
int tail = 0;

void drive(int speed)
{
  if (speed > 200)
  {
    speed = 200;
  }
  else if (speed < -200)
  {
    speed = -200;
  }
  if (millis() < last_acc_time + acc_time)
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

void parseMessage(char *msg)
{
  char cmd[3]; // To store the 2-char command
  int value = 0;

  sscanf(msg, "%1s", cmd);

  // skip whitespace
  char *beg = ++msg;

  while (*beg == ' ')
  {
    beg++;
  }

  char *end = beg;

  while (*end != '\0')
  {
    end++;
  }

  Serial.println(cmd);
  value = atoi(beg);
  Serial.println(value);

  if (cmd[0] == 'd')
  {
    set_speed = value;
  }
  else if (cmd[0] == 's')
  {
    set_degree = value;
  }
}

void processMessage()
{
  // Message extraction from ring buffer
  char message[BUFFER_SIZE];
  int index = 0;
  while (tail != head)
  {
    char currentChar = ringBuffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;

    if (currentChar == '\n')
    { // End of message
      break;
    }

    message[index++] = currentChar;
  }

  message[index] = '\0'; // Null-terminate the message string
  Serial.println(message);

  // Parse the extracted message
  parseMessage(message);
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
  while (Serial.available() > 0)
  {
    char incomingByte = Serial.read();
    Serial.print(incomingByte);
    ringBuffer[head] = incomingByte;
    head = ++head % BUFFER_SIZE; // Move the head and wrap it around

    // If head meets tail, it means buffer overflow, so move tail forward
    if (head == tail)
    {
      tail = ++tail % BUFFER_SIZE;
    }

    // Check for the end of the message (newline '\n')
    if (incomingByte == '\n')
    {
      processMessage();
    }
  }
  steer(set_degree);
  drive(set_speed);
}
