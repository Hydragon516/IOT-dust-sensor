/**
 * \file IOT_dust_sensor.ino
 * \author Hydragon516 <hydragon516@gmail.com>
 * \date 22 January 2019
 *
 * \brief Send and run MIDI signals to Arduino through MIDI communications.
 *
 * \section License
 * The author has placed this work in the Public Domain, thereby relinquishing
 * all copyrights. Everyone is free to use, modify, republish, sell or give away
 * this work without prior consent from anybody.
 */

#define BLYNK_PRINT Serial
#define ESP8266_BAUD 9600
 
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <SoftwareSerial.h>
 
BlynkTimer timer;
SoftwareSerial EspSerial(2, 3);
 
char auth[] = "*****";
char ssid[] = "*****";
char pass[] = "*****";
 
int pinDrive = A0;                                       // Sensor IR LED Drive
int pinADC = A1;                                         // Reading Sensor Output
int pinLED = A2;                                         // LED Indicattion of Dust Density 
boolean pinState = 1;
 
int result;
 
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;                                    // Pulse Interval (0.28 + 0.04 + 9.68 = 10 ms)
 
unsigned long measureInterval = 20;                      // Time Interval of Measurement (ms)
unsigned long blinkInterval = 3000;                      // Time Interval for LED Indicator
 
unsigned long prevMeasure;                               // Previous Time of Measure
unsigned long prevBlink;                                 // Previous Time of LED ON or OFF
 
float voltageADC = 0.0;                                  // Instantaneous Voltage (ADC)
float smoothADC = 0.8;                                   // Moving Average Voltage
float voltageClean = 0.8;                                // Voltage for Clean Air (Zero Setting)
 
ESP8266 wifi(&EspSerial);
 
void setup() {
  pinMode(pinDrive, OUTPUT);
  pinMode(pinLED, OUTPUT);     
 
  Serial.begin(9600);
  delay(10);
  EspSerial.begin(ESP8266_BAUD);
  delay(10);
  Blynk.begin(auth, wifi, ssid, pass);
  timer.setInterval(1000L, myTimerEvent1);
  prevMeasure = millis();
  prevBlink = millis();
}
 
void myTimerEvent1()
{
  Blynk.virtualWrite(V1, result);
}
 
void loop() {
  Blynk.run();
  timer.run();
 
  if(millis() - prevMeasure > measureInterval) {
    prevMeasure = millis();
    digitalWrite(pinDrive, LOW);                         // Pulse ON
    delayMicroseconds(samplingTime);                     // 0.28 ms 
    
    voltageADC = analogRead(pinADC) / 1023.0 * 5.0;      // ADC to Voltage Convertion
    
    delayMicroseconds(deltaTime);                        // 0.04 ms
    digitalWrite(pinDrive, HIGH);                        // Pulse OFF
    delayMicroseconds(sleepTime);                        // 9.68 ms
 
    smoothADC = voltageADC * 0.005 + smoothADC * 0.995;  // Exponential Moving Average
 
    result = smoothADC * 171.9;
    
    Serial.print("* V_instant = ");
    Serial.print(voltageADC, 4);
    Serial.print(",   * V_smooth = ");
    Serial.println(smoothADC, 4);
  }
  
  blinkInterval = long(50.0 / (smoothADC - voltageClean));
  
  if(blinkInterval > 5000) {
    blinkInterval = 5000;
    pinState = 1;
  }
  
  if(millis() - prevBlink > blinkInterval) {
    prevBlink = millis();
    digitalWrite(pinLED, pinState);
    pinState = !pinState;
  }
}
