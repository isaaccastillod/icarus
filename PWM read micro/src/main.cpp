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
// int pin = 7;
int throttle_pin = 7;
int throttle_speed = SPEED_MIN;

double W1_offset = 0;
double W2_offset = 0;
double W3_offset = 0;
double W4_offset = 0;

double L_offset = 0;
double R_offset = 0;

long int prev_t = 0;
long int curr = 0;
long int dt = 0;

struct Angles 
{
  double x;
  double y;
  double z;
};

struct PID_
{
  double kP;
  double kD;
  double kI;
  double lastDerivative;
};

PID_ pitchPID;
PID_ yawPID;

Angles eulerAng;

void initPIDs() {
  pitchPID.kD = 0.3;
  pitchPID.kP = 0.5;
  pitchPID.kI = 0;
  pitchPID.lastDerivative = 0;
  yawPID.kD = 0.3;
  yawPID.kP = 0.5;
  yawPID.kI = 0;
  yawPID.lastDerivative = 0;

}

double pd(double desiredAngle, double currAngle, double dt, PID_ pid) {
  double error = desiredAngle - currAngle;
    double derivative = error / dt;
    double output = pid.kP * error + pid.kD * (derivative - pid.lastDerivative);
    pid.lastDerivative = derivative;
    return output;
}



// ------------------------------------------------
// Read gyro from BNO055
void read_gyro(sensors_event_t sensor) {
  eulerAng.x = sensor.gyro.x;
  eulerAng.y = sensor.gyro.y;
  eulerAng.z = sensor.gyro.z;
  Serial.print("\tx= ");
  Serial.print(eulerAng.x);
  Serial.print(" |\ty= ");
  Serial.print(eulerAng.y);
  Serial.print(" |\tz= ");
  Serial.println(eulerAng.z);
}


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
  pinMode(throttle_pin, INPUT);

  // while(!Serial);
  initPIDs();
  //dumb values, easy to spot problem with gyro
  eulerAng.x = -1000000;
  eulerAng.y = -1000000;
  eulerAng.z = -1000000; 

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
  throttle_speed = pulseIn(throttle_pin, HIGH);
  // Serial.println(throttle_speed);  
  int tot_R_speed = throttle_speed + R_offset;
  int tot_L_speed = throttle_speed + L_offset;

  // int tot_W1 = throttle_speed + W1_offset;
  // int tot_W2 = throttle_speed + W2_offset;
  // int tot_W3 = throttle_speed + W3_offset;
  // int tot_W4 = throttle_speed + W4_offset;

  //DO NOT REMOVE, SAFETY
  if (tot_R_speed > SPEED_MID || tot_L_speed > SPEED_MID) {
    myESC1.speed(SPEED_MIN);                                    // tell ESC to go to the oESC speed value
    myESC2.speed(SPEED_MIN); 
  } else {    
    myESC1.speed(tot_R_speed);                                    // tell ESC to go to the oESC speed value
    myESC2.speed(tot_L_speed);
  }  
  Serial.print("\ttot_R_speed= ");
  Serial.print(tot_R_speed);
  Serial.print(" |\ttot_L_speed= ");
  Serial.println(tot_L_speed);
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);  

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.println();
  // Serial.print("Calibration: Sys=");
  // Serial.print(system);
  
  // Serial.print(" Orient=");
  // Serial.print("Orient:");
  read_gyro(orientationData);

  curr = millis();
  dt = curr - prev_t;
  prev_t = curr;
  

  //Control loop
  // if (y < -5) {
  //   R_speed++;
  //   L_speed--;
  // } else if (y > 5) {
  //   R_speed--;
  //   L_speed++;
  // }

  L_offset = -pd(0, eulerAng.y, dt, pitchPID);
  R_offset = pd(0, eulerAng.y, dt, pitchPID);


  // Four motor controller
  // (currently ignoring roll for simplicity)
  // W1_offset = pd(0, eulerAng.y, dt, pitchPID) - pd(0, eulerAng.z, dt, yawPID);
  // W2_offset = pd(0, eulerAng.y, dt, pitchPID) + pd(0, eulerAng.z, dt, yawPID);
  // W3_offset = -pd(0, eulerAng.y, dt, pitchPID) - pd(0, eulerAng.z, dt, yawPID); 
  // W4_offset = -pd(0, eulerAng.y, dt, pitchPID) + pd(0, eulerAng.z, dt, yawPID);

  if (R_offset < -200) {
    R_offset = -200;
  }
  if (L_offset < -200) {
    L_offset = -200;
  }
  if (R_offset > 200) {
    R_offset = 200;
  }
  if (L_offset > 200) {
    L_offset = 200;
  }
  Serial.print(" |\tR_offset= ");
  Serial.print(R_offset);
  Serial.print(" |\tL_offset= ");
  Serial.print(L_offset);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}