#include <PID_v1.h>
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
#include "Arduino.h"
#include <Wire.h>

#define PIN_INPUT A0
#define PIN_SETPOINT A1
#define PIN_OUTPUT 6

RTC_DS3231 rtc;

float V_OC;
float I_SC;

float irradiance [24][2];
int timeSetPoint;
char typeTime;
int positionIrradiance = 0;

unsigned long lastSend = 0;
unsigned long lastChange = 0;

//Define Variables we'll be connecting to
double Setpoint, Input, Output; // these are just variables for storing values

float Kp=0.10; //Initial Proportional Gain 
float Ki=0.8; //Initial Integral Gain 
float Kd=0; //Initial Differential Gain 

File file;

PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);


void setupRTC(){
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    //Serial.flush();
    abort();
  }

  if (rtc.lostPower()) {
      Serial.println("RTC lost power, let's set the time!");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void setupSD(){
  int count = 0;
  int count_line = 0;
  int count_column = 0;
  if (!SD.begin(4)) {
    Serial.println("initialization SD  failed!");
    while (1);
  }
  file = SD.open("002.csv");
  if(file){
    Serial.println("test.txt:");

    while (file.available()) {
      String numberString = "";
      static char input[8];
      static uint8_t i;
      char c = file.read();
      if ( (c != '\r' && c != ';' && i < 15) || c == '.'){
          input[i++] = c;
      }else{
        input[i] = '\0';
        i = 0;
        numberString = input;
        float number = numberString.toFloat();
        if(count%2 == 0){
          irradiance[count_line][count_column] =  number;
          count_column++;
        }else{
          irradiance[count_line][count_column] =  number;
          count_column = 0;
          count_line++;
        }
        count++;
      }
    }
  }else{
    Serial.println("Erro abrir irradiance.csv");
  }
}

void configureTime(){
  bool configuredTime = false;
  while(!configuredTime){
     while (Serial.available()) {
        static char input[5];
        static uint8_t i;
        char c = Serial.read();
        if (c != '\r' && c != ';' && c > 47 && c < 58){
          input[i++] = c;
        }else{
          if(c == 'm' || c == 's' || c == 'h'){
            typeTime = c;
            configuredTime =true;
          }else{
            input[i] = '\0';
            i = 0;
            if(timeSetPoint == 0)
              timeSetPoint = atoi(input);
          }
      }
     }
  }
  Serial.println(timeSetPoint);
  Serial.println(typeTime);
}

void changeSetPoint(){
  if(typeTime = 's'){
     if((millis()-lastChange) > (timeSetPoint * 1000)){
      if(positionIrradiance < 24){
        Setpoint = convertToPwm(irradiance[positionIrradiance++][1]);
        Serial.println(Setpoint);
      }
      lastChange = millis();
    }
  }else if(typeTime = 'm'){
    if((millis()-lastChange) > (timeSetPoint* 60 * 1000)){
      if(positionIrradiance < 24){
        Setpoint = convertToPwm(irradiance[positionIrradiance++][1]);
        Serial.println(Setpoint);
      }
      lastChange = millis();
    }
  }else if(typeTime = 'h'){
     if((millis()-lastChange) > (timeSetPoint * 60 * 60 * 1000)){
      if(positionIrradiance < 24){
        Setpoint = convertToPwm(irradiance[positionIrradiance++][1]);
        Serial.println(Setpoint);
      }
      lastChange = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Input = analogRead(PIN_INPUT);
  Setpoint = 150;
  myPID.SetMode(AUTOMATIC);
  setupSD();
  setupRTC();
  configureTime();
  lastChange = millis();
  changeSetPoint();
  Setpoint = convertToPwm(irradiance[0][1]);
}

float getIrradiance(int input){
  V_OC = (float(input)/ float(1023))*3.3;;
  I_SC = (V_OC/float(470))/float(0.1);
  return ((I_SC * 1000) / (0.0419));
}

int convertToPwm(float irradiance){
  float current = ((irradiance * 0.0419)/1000);
  float voltage = current * 0.1 * 470; 
  int irradiance_pwm = (voltage/3.3)*float(1023);
  return irradiance_pwm;
}

float averageIrradiance(){
  float sum = 0.0;
  for (int i = 0 ; i < 4 ; i++)
    sum += analogRead(PIN_INPUT);
  return  ((float) sum) / 4 ;  
}

void loop() {
  Input = averageIrradiance();
  //Setpoint = analogRead(PIN_SETPOINT);
  changeSetPoint();
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);

  if(millis() - lastSend > 100){
//    Serial.println(convertToPwm(1675.72));
//    Serial.println(getIrradiance(1023)) ;
//    lastSend = millis();
//    Serial.println("Reference,y,pwm");
//    Serial.print(getIrradiance(Setpoint));
//    Serial.print(",");
//    Serial.print(getIrradiance(Input));
//    Serial.print(",");
//    Serial.println(Output);
  }
}
