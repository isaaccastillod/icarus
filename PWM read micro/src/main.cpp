#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "ESC.h"
#define LED_PIN (13)              // Pin for the LED 
#define SPEED_MIN (1000)                                  // Set the Minimum Speed in microseconds
#define SPEED_MAX (2000)                                  // Set the Minimum Speed in microseconds

ESC myESC (6, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
int oESC; 

const int groundpin = A4;             // analog input pin 4 -- ground
const int powerpin = A5;              // analog input pin 5 -- voltage
const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)
int pin = 7;
unsigned long duration;
// Servo ESC;


void setup() {
  Serial.begin(9600);
  pinMode(pin, INPUT);
  pinMode(groundpin, OUTPUT);
  pinMode(powerpin, OUTPUT);
  digitalWrite(groundpin, LOW);
  digitalWrite(powerpin, HIGH);
  digitalWrite(LED_PIN, HIGH);    // LED High while signal is High (can be removed)
  myESC.calib();
  myESC.arm();   
  Serial.print("ESC armed");
  digitalWrite(LED_PIN, LOW);     // LED Low when the calibration is done (can be removed)
}

void loop() {
  duration = pulseIn(pin, HIGH);
  Serial.print("PWM: ");
  Serial.print(duration);
  Serial.print("\t");
  myESC.speed(duration);
  // print the sensor values:
  Serial.print("X: ");
  Serial.print(analogRead(xpin));
  // print a tab between values:
  Serial.print("\t");
  Serial.print("Y: ");
  Serial.print(analogRead(ypin));
  // print a tab between values:
  Serial.print("\t");
  Serial.print("Z: ");  //600 - 404
  Serial.print(analogRead(zpin));
  Serial.println();
  // delay before next reading:
  //  delay(100);
}

// void setup() {
//   pinMode(LED_PIN, OUTPUT);                               // LED Visual Output
//   myESC.arm();                                            // Send the Arm value so the ESC will be ready to take commands
//   digitalWrite(LED_PIN, HIGH);                            // LED High Once Armed
//   delay(5000);                                            // Wait for a while
// }

// void loop() {
//   for (oESC = SPEED_MIN; oESC <= SPEED_MAX; oESC += 1) {  // goes from 1000 microseconds to 2000 microseconds
//     myESC.speed(oESC);                                    // tell ESC to go to the oESC speed value
//     delay(10);                                            // waits 10ms for the ESC to reach speed
//   }
//   delay(1000);
//   for (oESC = SPEED_MAX; oESC >= SPEED_MIN; oESC -= 1) {  // goes from 2000 microseconds to 1000 microseconds
//     myESC.speed(oESC);                                    // tell ESC to go to the oESC speed value
//     delay(10);                                            // waits 10ms for the ESC to reach speed  
//    }
//   delay(5000);                                            // Wait for a while befor restart
// }

