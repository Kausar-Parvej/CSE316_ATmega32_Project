#include <MPU6050_light.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // Include the LiquidCrystal_I2C library

int lcdAddress = 0x27;

MPU6050 mpu(Wire);
LiquidCrystal_I2C lcd(lcdAddress, 16, 2);

unsigned long time = 0;

#define PADDLE_LEFT 4
#define PADDLE_RIGHT 5
#define BUZZER 6
#define SCORE_INPUT 8
#define LIFE_COUNT 9
#define RESET 10


float angleX , angleY , angleZ;
int currentScore = 0;
int highestScore = 0;
int lifeCount = 3;

int pin_8_prevValue = 0 ;
int pin_9_prevValue = 0 ;
int change = 0;

void setup(){

  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();        //for calibration

  //PIN CONFIGURATIONS
  pinMode(PADDLE_LEFT, OUTPUT);
  pinMode(PADDLE_RIGHT, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(SCORE_INPUT, INPUT);
  pinMode(LIFE_COUNT, INPUT);
  pinMode(RESET, INPUT);
  inGameDisplay();
  digitalWrite(PADDLE_RIGHT, LOW);
  digitalWrite(PADDLE_RIGHT, LOW);
}

void inGameDisplay(){
    lcd.clear();                     
    lcd.print("Score:");
    lcd.print(currentScore);
    lcd.setCursor(0,1); 
    lcd.print("Life Left: ");
    lcd.print(lifeCount);
}


void gameEndDisplay(){
    lcd.clear();                     
    lcd.setCursor(0,0);  
    lcd.print("Game Over!");
    lcd.setCursor(0,1); 
    lcd.print("Score:");
    lcd.print(currentScore);

    lcd.print(" Best:");
    lcd.print(highestScore);
}


void manageScore(){
    if(lifeCount>-1) {
      change = 1;
      currentScore++;
      digitalWrite(BUZZER,HIGH);
      delay(60);
      digitalWrite(BUZZER,LOW);
    }
    if(currentScore>highestScore) highestScore = currentScore ;
}

void manageLife(){
      change = 1;
      if(lifeCount>-1) {
        tone(BUZZER,200);
        delay(130);
        noTone(BUZZER);
        lifeCount--;
      }
      if(lifeCount==-1) {
          tone(BUZZER, 900);
          delay(1000);
          tone(BUZZER, 200);
          delay(1000);
          tone(BUZZER,900);
          delay(1000);
          noTone(BUZZER);
          lifeCount--;
      }
}



void loop(){


  int pinValue1 = digitalRead(LIFE_COUNT);
  int pinValue = digitalRead(SCORE_INPUT);
  int resetPinValue = digitalRead(RESET);

  if(pinValue>0){
    manageScore();
    delay(40);
  }

  if(pinValue1>0){
    manageLife();
    delay(40);
  }

  mpu.update();
  angleX = mpu.getAngleX();
  
  
  
  if(1)                         // print data every 3ms
  {            

    if (angleX > 10) {
        digitalWrite(PADDLE_LEFT, HIGH); // Set the pin to HIGH
        delay(1);
        digitalWrite(PADDLE_LEFT , LOW);
    }
    

    if (angleX < -10) {
        digitalWrite(PADDLE_RIGHT, HIGH); // Set the pin to HIGH
        delay(1);
        digitalWrite(PADDLE_RIGHT,LOW);
    }
    


    if(resetPinValue>0){
        change = 1;
        lifeCount = 3;
        currentScore = 0;
    }

    
    time=millis();
    if(time>60){
      if(change == 1 && lifeCount>-1){
        inGameDisplay();
      } else if (change == 1){
        gameEndDisplay();
      }
      time = 0;
    }

    change = 0; 
  }
}





