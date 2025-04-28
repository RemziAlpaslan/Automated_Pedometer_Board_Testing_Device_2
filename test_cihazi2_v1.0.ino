#include <ESP32Servo.h>

Servo servo1;  
Servo servo2;

// Control, feedback and switch pins
int servoPin1 = 15   ; // GPIO pin for the servo control
int feedbackPin1 = 35; // GPIO pin for the feedback (must be an ADC capable pin)
int servoPin2 = 2   ; // GPIO pin for the servo control
int feedbackPin2 = 34; // GPIO pin for the feedback (must be an ADC capable pin)
int switchPin1 = 4; // GPIO pin for the first switch
int switchPin2 = 16 ; // GPIO pin for the second switch

// Calibration values
int minMicSec = 500;
int maxMicSec = 2500;
int minFeedback = 200;
int maxFeedback = 3500;
int minDegree = 0;
int maxDegree = 270;
int tolerance = 75; // max feedback measurement error

int degree1;
int degree2;

int switchState1;
int switchState2;



int analogToPWM(int analogPin)
{
  return map(analogRead(analogPin), minFeedback, maxFeedback, minMicSec, maxMicSec);
}

int degreeToPWM(int degree)
{
  return map(degree, minDegree, maxDegree, minMicSec, maxMicSec);
}

int analogToDegree(int analogPin)
{
  return map(analogRead(analogPin), minFeedback, maxFeedback, minDegree, maxDegree);
}

void delayMicSec(unsigned int us) {
  unsigned long startMicros = micros();
  while (micros() - startMicros < us) {
    // Sadece bekle
  }
}

// Function to move the servos to a specific position with speed control
void goToPosWithSpeed(Servo &servo1, Servo &servo2, int analogPin1, int analogPin2, int pos1, int pos2)
{
  int currentPos1 = analogToPWM(analogPin1);
  int currentPos2 = analogToPWM(analogPin2);
  int target1 = degreeToPWM(pos1);
  int target2 = degreeToPWM(pos2);
  
  while (abs(analogToPWM(analogPin1) - target1) > tolerance || abs(analogToPWM(analogPin2) - target2) > tolerance)
  {
    if (currentPos1 < target1) currentPos1++;
    if (currentPos1 > target1) currentPos1--;
    if (currentPos2 < target2) currentPos2++;
    if (currentPos2 > target2) currentPos2--;
    
    servo1.writeMicroseconds(currentPos1);
    servo2.writeMicroseconds(currentPos2);

    delayMicSec(100);
  }
}

void setup() 
{ 
  Serial.begin(9600);
  servo1.attach(servoPin1,500,2500);
  servo2.attach(servoPin2,500,2500);

  // Set switch pins as input
  pinMode(switchPin1, INPUT_PULLUP);
  pinMode(switchPin2, INPUT_PULLUP);
} 

void loop()
{

  // Read the state of the switches
  switchState1 = digitalRead(switchPin1);
  switchState2 = digitalRead(switchPin2);

  if(switchState1==LOW)
  {
    degree1 = 225;
  }
  else {
    degree1 = 135;
  }

  if(switchState2==LOW)
  {
    degree2 = 225;
  }
  else
  {
    degree2 = 135;
  }

  goToPosWithSpeed(servo1, servo2, feedbackPin1, feedbackPin2, degree1, degree2);
  Serial.println("********************************");
  Serial.println(analogToDegree(feedbackPin1));
  Serial.println(analogToDegree(feedbackPin2));
  delay(1000);

  goToPosWithSpeed(servo1, servo2, feedbackPin1, feedbackPin2, 135, 135);
  Serial.println("********************************");
  Serial.println(analogToDegree(feedbackPin1));
  Serial.println(analogToDegree(feedbackPin2));
  delay(300);

  // Read the state of the switches
  switchState1 = digitalRead(switchPin1);
  switchState2 = digitalRead(switchPin2);

  if(switchState1==LOW)
  {
    degree1 = 45;
  }
  else
  {
    degree1 = 135;
  }

  if(switchState2==LOW)
  {
    degree2 = 45;
  }
  else
  {
    degree2 = 135;
  }

  

  goToPosWithSpeed(servo1, servo2, feedbackPin1, feedbackPin2, degree1, degree2);
  Serial.println("********************************");
  Serial.println(analogToDegree(feedbackPin1));
  Serial.println(analogToDegree(feedbackPin2));

  delay(1000);

  goToPosWithSpeed(servo1, servo2, feedbackPin1, feedbackPin2, 135, 135);
  Serial.println("********************************");
  Serial.println(analogToDegree(feedbackPin1));
  Serial.println(analogToDegree(feedbackPin2));
  delay(300);
  

 
}
