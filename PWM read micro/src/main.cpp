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
#define FRONT_LEFT (11)
#define FRONT_RIGHT (10)
#define BACK_LEFT (6)
#define BACK_RIGHT (5)
#define INPUT_RS0 (18)
#define INPUT_RS1 (19)
#define INPUT_THROTTLE (20)
#define INPUT_YAW (21)
ESC myESC1 (FRONT_RIGHT, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC myESC2 (FRONT_LEFT, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC myESC3 (BACK_LEFT, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC myESC4 (BACK_RIGHT, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
bool calibrated = false;

const int groundpin = A4;             // analog input pin 4 -- ground
const int powerpin = A5;              // analog input pin 5 -- voltage
const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)
int throttle_speed = SPEED_MIN;
int desired_yaw = 1500;
int desired_pitch = 1500;
int desired_roll = 1500;
int rs0 = 1500;
int rs1 = 1500;

double level_x = 0;
double level_y = 0;
double level_z = 0;
double W1_offset = 0;
double W2_offset = 0;
double W3_offset = 0;
double W4_offset = 0;

double L_offset = 0;
double R_offset = 0;

long int init_t = 0;
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
PID_ rollPID;

Angles eulerAng;

void initPIDs() {
  pitchPID.kD = 1.3;
  pitchPID.kP = 0.04;
  pitchPID.kI = 18; //0
  pitchPID.lastDerivative = 0;
  rollPID.kD = pitchPID.kD; //oscillation
  rollPID.kP = pitchPID.kP; //magnitude of difference between motors
  rollPID.kI = pitchPID.kI ; //0
  rollPID.lastDerivative = 0;
  yawPID.kD = 4; //oscillation
  yawPID.kP = 0.02; //magnitude of difference between motors
  yawPID.kI = 0 ; //0
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
uint16_t BNO055_SAMPLERATE_DELAY_MS = 1; //originally 100
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void ESC_startup() {
  pinMode(LED_PIN, OUTPUT);                              // LED Visual Output    
  Serial.println("3 second to plug in battery");
  delay(3000);
  Serial.println("Arming now");
  myESC1.arm();                                            // Send the Arm value so the ESC will be ready to take commands
  Serial.println("Finished arming ESC1");
  myESC2.arm();  
  Serial.println("Finished arming ESC2");
  myESC3.arm();                                            // Send the Arm value so the ESC will be ready to take commands
  Serial.println("Finished arming ESC3");
  myESC4.arm();  
  Serial.println("Finished arming ESC4");
  digitalWrite(LED_PIN, HIGH);                            // LED High Once Armed
  Serial.println("Waiting");
  delay(5000);                                            // Wait for a while
  myESC1.speed(SPEED_MIN);                                    // my calibration sequence
  myESC2.speed(SPEED_MIN); 
  myESC3.speed(SPEED_MIN);                                    // my calibration sequence
  myESC4.speed(SPEED_MIN); 
  delay(2000); 
  Serial.println("Drone armed");
}

void custom_calibration() {
  level_x = eulerAng.x;
  level_y = eulerAng.y;
  level_z = eulerAng.z;
  Serial.println("ground calibrated");
}

void setup(void)
{
  Serial.begin(115200);
  ESC_startup();
  Serial.println("Orientation Sensor Test"); Serial.println("");
  pinMode(INPUT_YAW, INPUT);
  pinMode(INPUT_RS0, INPUT);
  pinMode(INPUT_RS1, INPUT);
  pinMode(INPUT_THROTTLE, INPUT);
  Serial.println("Sensor Test finished");
  // while(!Serial);
  initPIDs();
  Serial.println("initPIDs finished");
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
  Serial.println("Setup finished");
}

void loop(void)
{
  throttle_speed = pulseIn(INPUT_THROTTLE, HIGH);
  desired_yaw = pulseIn(INPUT_YAW, HIGH);
  rs0 = pulseIn(INPUT_RS0, HIGH);
  rs1 = pulseIn(INPUT_RS1, HIGH);
  desired_pitch = rs0 + (1500 - rs1);
  desired_roll = rs1 - (1500 - rs0);
  
  // Serial.println(throttle_speed);  

  
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

  if (!calibrated) {
    custom_calibration();
    calibrated = true;
  }

  // L_offset = -pd(0, eulerAng.y, dt, pitchPID);
  // R_offset = pd(0, eulerAng.y, dt, pitchPID);

  // Serial.print(" |\teulerAng.y= ");
  // Serial.print(eulerAng.y);
  // Serial.print(" |\teulerAng.z= ");
  // Serial.print(eulerAng.z);

  // Four motor controller
  // (currently ignoring roll for simplicity)
  W1_offset = pd(level_y, eulerAng.y, dt, pitchPID)+pd(level_z, eulerAng.z, dt, rollPID)+pd(level_x, eulerAng.x, dt, rollPID); //front left
  W2_offset = pd(level_y, eulerAng.y, dt, pitchPID)-pd(level_z, eulerAng.z, dt, rollPID)-pd(level_x, eulerAng.x, dt, rollPID); //front right
  W3_offset = -pd(level_y, eulerAng.y, dt, pitchPID) + pd(level_z, eulerAng.z, dt, rollPID)-pd(level_x, eulerAng.x, dt, rollPID); //back left
  W4_offset = -pd(level_y, eulerAng.y, dt, pitchPID) - pd(level_z, eulerAng.z, dt, rollPID)+pd(level_x, eulerAng.x, dt, rollPID);; //back right

    // esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    // esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    // esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    // esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)    

  Serial.print(" |\tW1_offset= ");
  Serial.print(W1_offset);
  Serial.print(" |\tW2_offset= ");
  Serial.print(W2_offset);
  Serial.print(" |\tW3_offset= ");
  Serial.print(W3_offset);
  Serial.print(" |\tW4_offset= ");
  Serial.print(W4_offset);

  int tot_W1 = throttle_speed + W2_offset; //front left
  int tot_W2 = throttle_speed + W1_offset; //front right
  int tot_W3 = throttle_speed + W3_offset; //back left
  int tot_W4 = throttle_speed + W4_offset; //back right

  int tot_FR_speed = tot_W2;
  int tot_FL_speed = tot_W1;
  int tot_BR_speed = tot_W4;
  int tot_BL_speed = tot_W3; 

  //DO NOT REMOVE, SAFETY
  if (tot_FR_speed > SPEED_MID || tot_FL_speed > SPEED_MID || tot_BL_speed > SPEED_MID || tot_BR_speed > SPEED_MID) {
    myESC1.speed(SPEED_MIN);                                    // tell ESC to go to the oESC speed value
    myESC2.speed(SPEED_MIN); 
    myESC3.speed(SPEED_MIN);                                    // tell ESC to go to the oESC speed value
    myESC3.speed(SPEED_MIN); 
  } else {    
    myESC1.speed(tot_FR_speed);                                    // tell ESC to go to the oESC speed value
    myESC2.speed(tot_FL_speed);
    myESC3.speed(tot_BR_speed);                                    // tell ESC to go to the oESC speed value
    myESC4.speed(tot_BL_speed);
  }  
  // Serial.print("\ttot_FR_speed= ");
  // Serial.print(tot_FR_speed);
  // Serial.print(" |\ttot_FL_speed= ");
  // Serial.println(tot_FL_speed);
  // Serial.print("\ttot_BR_speed= ");
  // Serial.print(tot_BR_speed);
  // Serial.print(" |\ttot_BL_speed= ");
  // Serial.println(tot_BL_speed);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}