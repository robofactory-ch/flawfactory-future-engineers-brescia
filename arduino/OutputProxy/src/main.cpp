#include <Arduino.h>

#define BUFFER_SIZE 64 // Size of the ring buffer

char ringBuffer[BUFFER_SIZE]; // The ring buffer
int head = 0;                 // Points to the next byte to be written
int tail = 0;                 // Points to the next byte to be read

void parseMessage(char *msg)
{
  char cmd[3];         // To store the 2-char command
  float args[10];      // To store argument values
  char arg_labels[10]; // To store argument labels
  int arg_count = 0;

  // Scan the 2-char command
  sscanf(msg, "%2s", cmd);

  char *p = msg + 3; // Start parsing arguments after CMD

  // Parse all arguments (e.g., X, Y, Z with corresponding float values)
  while (*p != '\0')
  {
    char label;
    float value;

    if (sscanf(p, "%c%f", &label, &value) == 2)
    {
      arg_labels[arg_count] = label;
      args[arg_count] = value;
      arg_count++;
    }

    // Move pointer to the next argument
    while (*p != ' ' && *p != '\0')
    {
      p++;
    }
  }

  // Output parsed message for debugging
  Serial.print("CMD: ");
  Serial.println(cmd);

  for (int i = 0; i < arg_count; i++)
  {
    Serial.print(arg_labels[i]);
    Serial.print(": ");
    Serial.println(args[i]);
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

  // Parse the extracted message
  parseMessage(message);
}

void setup()
{
  Serial.begin(9600); // Initialize serial communication
}

void loop()
{
  // Read serial data and store it in the ring buffer
  while (Serial.available() > 0)
  {
    char incomingByte = Serial.read();
    ringBuffer[head] = incomingByte;
    head = (head + 1) % BUFFER_SIZE; // Move the head and wrap it around

    // If head meets tail, it means buffer overflow, so move tail forward
    if (head == tail)
    {
      tail = (tail + 1) % BUFFER_SIZE;
    }

    // Check for the end of the message (newline '\n')
    if (incomingByte == '\n')
    {
      processMessage();
    }
  }
}
