#include <Arduino.h>
#include <math.h>

const int groundpin = A4;             // analog input pin 4 -- ground
const int powerpin = A5;              // analog input pin 5 -- voltage
const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)
int pin = 7;
unsigned long duration;

void setup() {
  Serial.begin(9600);
  pinMode(pin, INPUT);
  pinMode(groundpin, OUTPUT);
  pinMode(powerpin, OUTPUT);
  digitalWrite(groundpin, LOW);
  digitalWrite(powerpin, HIGH);
}

void loop() {
  double x = analogRead(xpin);

  duration = pulseIn(pin, HIGH);
  Serial.print("PWM: ");
  Serial.print(duration);
  Serial.print("\t");
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
  delay(100);
}

int* filter(float x, float y, float z) {
  return 0;
}

void calibration() {

}

double tilt(double x, double y, double z) { // -arctan[(-sin(ax)*cos(az) + cos(ax)*sin(ay)*sin(az)) / (cos(ax)*cos(ay))]
  return atan((-sin(x)*cos(z) + cos(x)*sin(y)*sin(z) )/ (cos(x)*cos(y)) );  
}

double pitch(double x, double y, double z) {
  //pitch = arcsin[ cos(ax)*sin(ay)*cos(az) + sin(ax)*sin(az) ]
  return asin(cos(x)*sin(y)*cos(z) + sin(x)*sin(z));
} 

double roll(double x, double y, double z) {
  //roll = -arctan[(-cos(ax)*sin(az) + sin(ax)*sin(ay)*cos(az)) / (cos(ay)*cos(az))]
  return -atan(() / (cos(y)));
} 

