
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

Servo servo1;  
Servo servo2;

LiquidCrystal_I2C lcd(0x27, 16, 2);


// Control, feedback and switch pins
#define SERVO_PIN_1 15   // GPIO pin for the first servo control
#define SERVO_PIN_2 2    // GPIO pin for the second servo control
#define SWITCH_PIN_1 4   // GPIO pin for the first switch
#define SWITCH_PIN_2 16  // GPIO pin for the second switch
#define FEEDBACK_PIN_1 35 // GPIO pin for the feedback (must be an ADC capable pin)
#define FEEDBACK_PIN_2 34 // GPIO pin for the feedback (must be an ADC capable pin)
#define BUZZER_PIN 27 // ilk buzzer için pin

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

int miliseconds1;
int miliseconds2;


bool buzzerstate = false;

int analogToPWM(int analogPin){return map(analogRead(analogPin), minFeedback, maxFeedback, minMicSec, maxMicSec);}

int degreeToPWM(int degree){return map(degree, minDegree, maxDegree, minMicSec, maxMicSec);}

// Function to move the servos to a specific position with speed control
void goToPos(Servo &servo1, Servo &servo2, int analogPin1, int analogPin2, int pos1, int pos2)
{
  int currentPos1 = analogToPWM(analogPin1);
  int currentPos2 = analogToPWM(analogPin2);
  int target1 = degreeToPWM(pos1);
  int target2 = degreeToPWM(pos2);
  
  while (abs(analogToPWM(analogPin1) - target1) > tolerance || abs(analogToPWM(analogPin2) - target2) > tolerance)
  {
    
    servo1.writeMicroseconds(target1);
    servo2.writeMicroseconds(target2);

  }
}



void lcdTimer(LiquidCrystal_I2C &lcd, unsigned long miliseconds1, unsigned long miliseconds2) {

  if(miliseconds1 < 2500 )
    {miliseconds1 = 0;}

  if(miliseconds2 < 2500 )
    {miliseconds2 = 0;}

  int seconds1 = (miliseconds1 / 1500) % 60;
  int minutes1 = (miliseconds1 / 60000) % 60;

  int seconds2 = (miliseconds2 / 1500) % 60;
  int minutes2 = (miliseconds2 / 60000) % 60;

  
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
  }


void lcdPos(LiquidCrystal_I2C &lcd, unsigned long degree1, unsigned long degree2) {
  
    lcd.setCursor(0, 1);
    if (degree1 < 100) lcd.print("0");
    lcd.print(degree1);

    lcd.setCursor(11, 1);
    if (degree2 < 100) lcd.print("0");
    lcd.print(degree2);
  }


void setup() 
{ 
  Serial.begin(9600);

  servo1.attach(SERVO_PIN_1,500,2500);
  servo2.attach(SERVO_PIN_2,500,2500);

  // Set switch pins as input
  pinMode(SWITCH_PIN_1, INPUT_PULLUP);
  pinMode(SWITCH_PIN_2, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);   // buzzer pini için çıkış modu
  
  lcd.init();
  lcd.backlight();
} 

void loop()
{

  if (digitalRead(SWITCH_PIN_1) == LOW){
    degree1 = 225;
  }
  else {
    degree1 = 135;
  }
  
  // İkinci buton kontrolü
  if (digitalRead(SWITCH_PIN_2) == LOW){
    degree2 = 225;
  }
  else {
    degree2 = 135;
  }

  unsigned long startTime = millis(); // Şu anki zamanı al

  // Servo kontrolü
  goToPos(servo1, servo2, FEEDBACK_PIN_1, FEEDBACK_PIN_2, degree1, degree2);
  // Lcd ye zamanın yazdırılması
  lcdTimer(lcd, miliseconds1+(millis()-startTime), miliseconds2+(millis()-startTime));
  lcdPos(lcd, degree1, degree2);
  delay(1500);

  unsigned long stopTime = millis(); // Şu anki zamanı al
  

  miliseconds1 +=(stopTime - startTime);
  miliseconds2 +=(stopTime - startTime);

  if (digitalRead(SWITCH_PIN_1) == HIGH){
    miliseconds1=0;
    digitalWrite(BUZZER_PIN, LOW);
  }
  if (digitalRead(SWITCH_PIN_2) == HIGH){
    miliseconds2=0;
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Lcd ye zamanın yazdırılması
  lcdTimer(lcd, miliseconds1, miliseconds2);

  startTime = millis(); // Şu anki zamanı al

  // Servo kontrolü
  goToPos(servo1, servo2, FEEDBACK_PIN_1, FEEDBACK_PIN_2, 135, 135);
  // Lcd ye zamanın yazdırılması
  lcdTimer(lcd, miliseconds1+(millis()-startTime), miliseconds2+(millis()-startTime));
  lcdPos(lcd, 135, 135);
  delay(1500);

  stopTime = millis(); // Şu anki zamanı al
  
  miliseconds1 +=(stopTime - startTime);
  miliseconds2 +=(stopTime - startTime);

  if (digitalRead(SWITCH_PIN_1) == HIGH){
    miliseconds1=0;
    digitalWrite(BUZZER_PIN, LOW);
  }
  if (digitalRead(SWITCH_PIN_2) == HIGH){
    miliseconds2=0;
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Lcd ye zamanın yazdırılması
  lcdTimer(lcd, miliseconds1, miliseconds2);
  

  // Buzzerın 1 saatin sonunda yanıp sönmesi için gerekli kod parçası
  if (miliseconds1>=3600000 || miliseconds2>=3600000)
  {
    buzzerstate = !buzzerstate;
    digitalWrite(BUZZER_PIN, buzzerstate);
  }

  if (digitalRead(SWITCH_PIN_1) == LOW){
    degree1 = 45;
  }
  else {
    degree1 = 135;
  }
  
  // İkinci buton kontrolü
  if (digitalRead(SWITCH_PIN_2) == LOW){
    degree2 = 45;
  }
  else {
    degree2 = 135;
  }

  startTime = millis(); // Şu anki zamanı al

  // Servo kontrolü
  goToPos(servo1, servo2, FEEDBACK_PIN_1, FEEDBACK_PIN_2, degree1, degree2);
  // Lcd ye zamanın yazdırılması
  lcdTimer(lcd, miliseconds1+(millis()-startTime), miliseconds2+(millis()-startTime));
  lcdPos(lcd, degree1, degree2);
  delay(1500);
  
  stopTime = millis(); // Şu anki zamanı al

  miliseconds1 +=(stopTime - startTime);
  miliseconds2 +=(stopTime - startTime);

  if (digitalRead(SWITCH_PIN_1) == HIGH){
    miliseconds1=0;
    digitalWrite(BUZZER_PIN, LOW);
  }
  if (digitalRead(SWITCH_PIN_2) == HIGH){
    miliseconds2=0;
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Lcd ye zamanın yazdırılması
  lcdTimer(lcd, miliseconds1, miliseconds2);
  

  startTime = millis(); // Şu anki zamanı al

  // Servo kontrolü
  goToPos(servo1, servo2, FEEDBACK_PIN_1, FEEDBACK_PIN_2, 135, 135);
  // Lcd ye zamanın yazdırılması
  lcdTimer(lcd, miliseconds1+(millis()-startTime), miliseconds2+(millis()-startTime));
  lcdPos(lcd, 135, 135);
  delay(1500);

  stopTime = millis(); // Şu anki zamanı al
  
  miliseconds1 +=(stopTime - startTime);
  miliseconds2 +=(stopTime - startTime);

  if (digitalRead(SWITCH_PIN_1) == HIGH){
    miliseconds1=0;
    digitalWrite(BUZZER_PIN, LOW);
  }
  if (digitalRead(SWITCH_PIN_2) == HIGH){
    miliseconds2=0;
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Lcd ye zamanın yazdırılması
  lcdTimer(lcd, miliseconds1, miliseconds2);
  

  // Buzzerın 1 saatin sonunda yanıp sönmesi için gerekli kod parçası
  if (miliseconds1>=3600000 || miliseconds2>=3600000)
  {
    buzzerstate = !buzzerstate;
    digitalWrite(BUZZER_PIN, buzzerstate);
  }
}
