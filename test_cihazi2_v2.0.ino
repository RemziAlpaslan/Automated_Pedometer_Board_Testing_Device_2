
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

Servo servo1;  
Servo servo2;

int lcdColumns = 16;
int lcdRows = 2;

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

#define BUZZER_PIN 27 // ilk buzzer için pin

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

int currentTime1 = 0;
int currentTime2 = 0;
int seconds1 = 0;
int minutes1 = 0;
int seconds2 = 0;
int minutes2 = 0;

bool buzzerstate = false;

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

  pinMode(BUZZER_PIN, OUTPUT);   // buzzer pini için çıkış modu
  
  lcd.init();
  lcd.backlight();
} 

void loop()
{

  if (digitalRead(switchPin1) == LOW){
    degree1 = 225;
  }
  else {
    degree1 = 135;
  }
  
  // İkinci buton kontrolü
  if (digitalRead(switchPin2) == LOW){
    degree2 = 225;
  }
  else {
    degree2 = 135;
  }

  unsigned long startTime = millis(); // Şu anki zamanı al

  goToPosWithSpeed(servo1, servo2, feedbackPin1, feedbackPin2, degree1, degree2);
  /*
  Serial.println("********************************");
  Serial.println(analogToDegree(feedbackPin1));
  Serial.println(analogToDegree(feedbackPin2));
  */
  delay(1000);

  goToPosWithSpeed(servo1, servo2, feedbackPin1, feedbackPin2, 135, 135);
  /*
  Serial.println("********************************");
  Serial.println(analogToDegree(feedbackPin1));
  Serial.println(analogToDegree(feedbackPin2));
  */
  delay(300);

  unsigned long stopTime = millis(); // Şu anki zamanı al

  currentTime1 +=(stopTime - startTime);
  currentTime2 +=(stopTime - startTime);

  if (digitalRead(switchPin1) == HIGH){
    currentTime1=0;
    digitalWrite(BUZZER_PIN, LOW);
  }
  if (digitalRead(switchPin2) == HIGH){
    currentTime2=0;
    digitalWrite(BUZZER_PIN, LOW);
  }

  seconds1 = (currentTime1/1000)%60 ;
  minutes1 = (currentTime1/60000)%60;

  seconds2 = (currentTime2/1000)%60 ;
  minutes2 = (currentTime2/60000)%60;

  lcd.setCursor(0, 0);
  if (minutes1 < 10) lcd.print("0");
  lcd.print(minutes1);
  lcd.print(":");
  if (seconds1 < 10) lcd.print("0");
  lcd.print(seconds1);
  

  lcd.setCursor(11, 0);
  if (minutes2 < 10) lcd.print("0");
  lcd.print(minutes2);
  lcd.print(":");
  if (seconds2 < 10) lcd.print("0");
  lcd.print(seconds2);

  if (currentTime1>=3600000 || currentTime2>=3600000)
  {
    buzzerstate = !buzzerstate;
    if (buzzerstate){
    digitalWrite(BUZZER_PIN, HIGH);}
    else{
    digitalWrite(BUZZER_PIN, LOW);}
  }

  if (digitalRead(switchPin1) == LOW){
    degree1 = 45;
  }
  else {
    degree1 = 135;
  }
  
  // İkinci buton kontrolü
  if (digitalRead(switchPin2) == LOW){
    degree2 = 45;
  }
  else {
    degree2 = 135;
  }

  startTime = millis(); // Şu anki zamanı al

  goToPosWithSpeed(servo1, servo2, feedbackPin1, feedbackPin2, degree1, degree2);
  /*
  Serial.println("********************************");
  Serial.println(analogToDegree(feedbackPin1));
  Serial.println(analogToDegree(feedbackPin2));
  */
  delay(1000);

  goToPosWithSpeed(servo1, servo2, feedbackPin1, feedbackPin2, 135, 135);
  /*
  Serial.println("********************************");
  Serial.println(analogToDegree(feedbackPin1));
  Serial.println(analogToDegree(feedbackPin2));
  */
  delay(300);

  stopTime = millis(); // Şu anki zamanı al

  currentTime1 +=(stopTime - startTime);
  currentTime2 +=(stopTime - startTime);

  if (digitalRead(switchPin1) == HIGH){
    currentTime1=0;
    digitalWrite(BUZZER_PIN, LOW);
  }
  if (digitalRead(switchPin2) == HIGH){
    currentTime2=0;
    digitalWrite(BUZZER_PIN, LOW);
  }

  seconds1 = (currentTime1/1000)%60 ;
  minutes1 = (currentTime1/60000)%60;

  seconds2 = (currentTime2/1000)%60 ;
  minutes2 = (currentTime2/60000)%60;

  lcd.setCursor(0, 0);
  if (minutes1 < 10) lcd.print("0");
  lcd.print(minutes1);
  lcd.print(":");
  if (seconds1 < 10) lcd.print("0");
  lcd.print(seconds1);
  

  lcd.setCursor(11, 0);
  if (minutes2 < 10) lcd.print("0");
  lcd.print(minutes2);
  lcd.print(":");
  if (seconds2 < 10) lcd.print("0");
  lcd.print(seconds2);

  if (currentTime1>=3600000 || currentTime2>=3600000)
  {
    buzzerstate = !buzzerstate;
    if (buzzerstate){
    digitalWrite(BUZZER_PIN, HIGH);}
    else{
    digitalWrite(BUZZER_PIN, LOW);}
  }
}
