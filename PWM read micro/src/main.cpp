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
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "ESC.h"
#define LED_PIN (13)              // Pin for the LED 
#define SPEED_MIN (1000)                                  // Set the Minimum Speed in microseconds
#define SPEED_LOW (1200)  
#define SPEED_MID (1500) 
#define SPEED_MAX (2000)                                  // Set the Minimum Speed in microseconds
// Right Motor
ESC myESC1 (11, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
// Left Motor
ESC myESC2 (10, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
int oESC;                                                 // Variable for the speed sent to the ESC


const int groundpin = A4;             // analog input pin 4 -- ground
const int powerpin = A5;              // analog input pin 5 -- voltage
const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)
int pin = 7;
int L_speed = SPEED_LOW;
int R_speed = SPEED_LOW;
unsigned long duration;

struct PID_
{
  double kP;
  double kD;
  double kI;
  double lastDerivative;
};

PID_ pitchPD;

void initPID() {
  pitchPD.kD = 0.25;
  pitchPD.kP = 0.25;
  pitchPD.kI = 0;
  pitchPD.lastDerivative = 0;
}

double pitchPID(double desiredPitch, double currPitch, double dt, PID_ pid) {
    double error = desiredPitch - currPitch;
    double derivative = error / dt;
    double output = pid.kP * error + pid.kD * (derivative - pid.lastDerivative);
    pid.lastDerivative = derivative;
    return output;
}



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
  initPID();

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void loop(void)
{
  myESC1.speed(R_speed);                                    // tell ESC to go to the oESC speed value
  myESC2.speed(L_speed); 
  Serial.println("Cycle Done");
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  

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
 

  //Control loop
  // if (y < -5) {
  //   R_speed++;
  //   L_speed--;
  // } else if (y > 5) {
  //   R_speed--;
  //   L_speed++;
  // }
  L_speed -= pitchPID(0, y, 1, pitchPD);
  R_speed += pitchPID(0, y, 1, pitchPD);

  if (R_speed < SPEED_MID - 100) {
    R_speed = SPEED_MID - 100;
  }
  if (L_speed < SPEED_MID - 100) {
    L_speed = SPEED_MID - 100;
  }
  if (R_speed > SPEED_MID + 100) {
    R_speed = SPEED_MID + 100;
  }
  if (L_speed > SPEED_MID + 100) {
    L_speed = SPEED_MID + 100;
  }

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}


