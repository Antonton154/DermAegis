#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht31 = Adafruit_SHT31();
//Hardware pin definitions
int UVOUT = A0;    //Output from the sensor
int REF_3V3 = A1;  //3.3V power on the Arduino board

int PIN_MQ7 = A2;

#include "MQ135.h"
const int PIN_MQ135 = A3;
#define RZERO 61.47
MQ135 mq135_sensor(PIN_MQ135, RZERO);

bool previousState = false;
bool currentState = false;

int UVI = 0;

unsigned long time = 0;

#include <SoftwareSerial.h>  //Create software serial object to communicate with SIM800L
SoftwareSerial GSM(9, 8);    //SIM800L Tx & Rx is connected to Arduino #9 & #8
SoftwareSerial esp(4, 5);
char phonenumber[] = "+84972073092";  //change +92 with country code and 3378655465 with phone number to sms
char inchar;                          // Will hold the incoming character from the GSM shield

float t = 25.0;
float h = 60.0;

String data[7] = 0;

#define VOLTAGE_AT_0_MG_M3 0.2
/* Sensor voltage at 0.5mg/m^3 */
#define VOLTAGE_AT_0_5_MG_M3 3.6
/* As the ADC and the sensor outputs are both linear we can simply scale the output 
*  voltage of the sensor to convert from volts to mg/m^3. */
#define SCALLING_FACTOR 0.5 / (VOLTAGE_AT_0_5_MG_M3 - VOLTAGE_AT_0_MG_M3)
#define LED_PIN 11
/* Sensors output pin */
#define SENSOR_PIN A6


void setup() {
  Serial.begin(9600);

  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(SENSOR_PIN, INPUT);

  esp.begin(9600);

  // GSM.begin(9600);     //Begin serial communication with Arduino and SIM800L

  // Serial.println("Initializing....");
  // initModule("AT", "OK", 1000);                 //Once the handshake test is successful, it will back to OK
  // initModule("ATE1", "OK", 1000);               //this command is used for enabling echo
  // initModule("AT+CPIN?", "READY", 1000);        //this command is used to check whether SIM card is inserted in GSM Module or not
  // initModule("AT+CMGF=1", "OK", 1000);          //Configuring TEXT mode
  // initModule("AT+CNMI=2,2,0,0,0", "OK", 1000);  //Decides how newly arrived SMS messages should be handled
  // Serial.println("Initialized Successfully");

  if (!sht31.begin(0x44)) {  // Set to 0x45 for alternate I2C address
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  } else {
    Serial.println("SHT oke");
  }
}

void loop() {
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);

  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;

  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  data[0] = String(t, 2);
  data[1] = String(h, 2);
  data[2] = String(uvIntensity, 2);

  UVI = round(uvIntensity / 2.5);

  data[3] = String(UVI, 2);

  int COValue = analogRead(PIN_MQ7);
  
  data[4] = String(COValue, 2);

  float correctedPPM = mq135_sensor.getCorrectedPPM(t, h);

  data[5] = String(correctedPPM, 2);

  // float DustDensity = 0.2;
  int Result;
  float SensorVoltage;
  float DustDensity;
  digitalWrite(LED_PIN, LOW);
  delayMicroseconds(280);

  /* Read the sensors analogue output */
  Result = analogRead(SENSOR_PIN);
  digitalWrite(LED_PIN, HIGH);
  SensorVoltage = Result * (5.00 / 1024);
  DustDensity = (SensorVoltage - VOLTAGE_AT_0_MG_M3) * SCALLING_FACTOR;;
  // Serial.println(SensorVoltage);

  data[6] = String(max(0, DustDensity), 2);

  currentState = checkState(UVI, COValue, correctedPPM, DustDensity);
  switch (currentState) {
    case true:
      if (previousState == false) {
        if (GSM.available() > 0) {
          inchar = GSM.read();
          Serial.print(inchar);
          // sendSMS(phonenumber, "Warning! The area is not safe for your health.");
          Serial.println("Warning sent.");
          delay(100);
        }
        previousState = true;
      }
      break;
    case false:
      if (previousState == true) {
        previousState = false;
      }
      break;
  }

  if (millis() - time > 3000) {
    String temp = "";
    for (int i = 0 ; i  < 7; i ++) {
      temp += data[i];
      temp+=";"
    }
    temp += "\n";
    Serial.print(temp);
    esp.print(temp);
  }

  delay(1000);
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead) {
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0; x < numberOfReadings; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

void sendSMS(char *number, char *msg) {
  GSM.print("AT+CMGS=\"");
  GSM.print(number);
  GSM.println("\"\r\n");  //AT+CMGS=”Mobile Number” <ENTER> - Assigning recipient’s mobile number
  delay(500);
  GSM.println(msg);  // Message contents
  delay(500);
  GSM.write(byte(26));  //Ctrl+Z  send message command (26 in decimal).
  delay(3000);
}

void initModule(String cmd, char *res, int t) {
  while (1) {
    Serial.println(cmd);
    GSM.println(cmd);
    delay(100);
    while (GSM.available() > 0) {
      if (GSM.find(res)) {
        Serial.println(res);
        delay(t);
        return;
      } else {
        Serial.println("Error");
      }
    }
    delay(t);
  }
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool checkState(int UVI, int COValue, float correctedPPM, float dust) {
  if (UVI >= 5 || COValue >= 170 || correctedPPM >= 1000) {
    return true;
  }
  return false;
}