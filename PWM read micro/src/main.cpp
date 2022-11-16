//BNO055 Dependencies
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_SPIDevice.h>

#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_SPIDevice.h> 
#include "ESC.h"
#define LED_PIN (13)              // Pin for the LED 
#define SPEED_MIN (1000)                                  // Set the Minimum Speed in microseconds
#define SPEED_LOW (1200)  
#define SPEED_MID (1500) 
#define SPEED_MAX (2000)                                  // Set the Minimum Speed in microseconds

ESC myESC1 (11, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC myESC2 (10, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
int oESC;                                                 // Variable for the speed sent to the ESC


const int groundpin = A4;             // analog input pin 4 -- ground
const int powerpin = A5;              // analog input pin 5 -- voltage
const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)
int pin = 7;
unsigned long duration;


// ------------------------------------------------
// BNO055 ReadData

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void ESC_startup() {
  pinMode(LED_PIN, OUTPUT);                               // LED Visual Output  
  Serial.println("5 seconds to plug in battery");
  delay(4000);
  Serial.println("1 second to plug in battery");
  delay(1000);
  Serial.println("Arming now");
  myESC1.arm();                                            // Send the Arm value so the ESC will be ready to take commands
  Serial.println("Finished arming ESC1");
  myESC2.arm();  
  Serial.println("Finished arming ESC2");
  digitalWrite(LED_PIN, HIGH);                            // LED High Once Armed
  Serial.println("Waiting");
  delay(5000);                                            // Wait for a while
  myESC1.speed(SPEED_MIN);                                    // my calibration sequence
  myESC2.speed(SPEED_MIN); 
  delay(2000); 
  Serial.println("Drone armed");
}

void setup(void)
{
  Serial.begin(115200);
  ESC_startup();
  Serial.println("Orientation Sensor Test"); Serial.println("");

  // while(!Serial);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

void loop(void)
{
  myESC1.speed(SPEED_LOW);                                    // tell ESC to go to the oESC speed value
  myESC2.speed(SPEED_LOW); 
  Serial.println("Cycle Done");
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  // printEvent(&orientationData);
  // printEvent(&angVelocityData);
  // printEvent(&linearAccelData);
  // printEvent(&magnetometerData);
  // printEvent(&accelerometerData);
  // printEvent(&gravityData);

  // int8_t boardTemp = bno.getTemp();
  // Serial.println();
  // Serial.print(F("temperature: "));
  // Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  Serial.print(" Orient=");
  Serial.print("Orient:");
  x = orientationData.gyro.x;
  y = orientationData.gyro.y;
  z = orientationData.gyro.z;
  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.println(mag);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}


