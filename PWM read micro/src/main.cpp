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
#define BACK_LEFT (11)
#define BACK_RIGHT (10)
#define FRONT_LEFT (6)
#define FRONT_RIGHT (5)
#define INPUT_RS0 (18)
#define INPUT_RS1 (19)
#define INPUT_THROTTLE (20)
#define INPUT_YAW (21)
ESC myESC1 (FRONT_RIGHT, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC myESC2 (FRONT_LEFT, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC myESC3 (BACK_LEFT, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC myESC4 (BACK_RIGHT, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
bool calibrated = false;

volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float roll_level_adjust, pitch_level_adjust;
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
double tot_FL_speed;
double tot_FR_speed;
double tot_BL_speed;
double tot_BR_speed;

double esc_1 = 0;
double esc_2 = 0;
double esc_3 = 0;
double esc_4 = 0;

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
  pitchPID.kD = .3;
  pitchPID.kP = 1.5;
  pitchPID.kI = 0; //0
  pitchPID.lastDerivative = 0;
  rollPID.kD = pitchPID.kD; //oscillation
  rollPID.kP = pitchPID.kP; //magnitude of difference between motors
  rollPID.kI = pitchPID.kI ; //0
  rollPID.lastDerivative = 0;
  yawPID.kD = .5; //oscillation
  yawPID.kP = 1; //magnitude of difference between motors
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

double range(double degree) {
  double longitude = degree;
  if (longitude > 180) {
    longitude -= 360;
  }
  return longitude;
}
void read_gyro(sensors_event_t sensor) {
  eulerAng.x = sensor.gyro.x;
  eulerAng.y = sensor.gyro.y;
  eulerAng.z = sensor.gyro.z;
  Serial.print("|\tx= ");
  Serial.print(range(eulerAng.x));
  // Serial.print(" |\ty= ");
  // Serial.print(eulerAng.y);
  // Serial.print(" |\tz= ");
  // Serial.println(eulerAng.z);
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
  level_x = range(eulerAng.x);
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
  throttle_speed = (pulseIn(INPUT_THROTTLE, HIGH) + throttle_speed)/2; //simple moving average
  desired_yaw = (pulseIn(INPUT_YAW, HIGH) + desired_yaw)/2;  
  rs0 = (pulseIn(INPUT_RS0, HIGH) + rs0)/2;
  rs1 = (pulseIn(INPUT_RS1, HIGH) + rs1)/2;
  desired_pitch = rs0 + (1500 - rs1);
  desired_roll = rs1 - (1500 - rs0);
  receiver_input_channel_2 = desired_pitch; //pitch
  receiver_input_channel_1 = desired_roll + 40; //roll
  receiver_input_channel_3 = throttle_speed; //throttle input
  receiver_input_channel_4 = desired_yaw + 15; //yaw

  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = - receiver_input_channel_1 + 1508;
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = - receiver_input_channel_1 + 1492;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 5.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = - receiver_input_channel_2 + 1508;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = - receiver_input_channel_2 + 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 5.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/10.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/10.0;
  }
  Serial.print(" |\tDesired Yaw= ");
  Serial.println(pid_yaw_setpoint);
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

  // Serial.print(" |\teulerAng.x= ");
  // Serial.print(eulerAng.x);
  // Serial.print(" |\teulerAng.y= ");
  // Serial.print(eulerAng.y);
  // Serial.print(" |\teulerAng.z= ");
  // Serial.print(eulerAng.z);

  // Four motor controller
  // (currently ignoring roll for simplicity)
  
  pid_output_pitch = pd(level_y-pid_pitch_setpoint, eulerAng.y, dt, pitchPID);
  pid_output_roll = pd(level_z-pid_roll_setpoint, eulerAng.z, dt, rollPID);
  pid_output_yaw = pd(level_x-pid_yaw_setpoint, range(eulerAng.x), dt, yawPID);

  W1_offset=-pid_output_pitch + pid_output_roll;// + pid_output_yaw; //front left
  W2_offset=-pid_output_pitch - pid_output_roll;// - pid_output_yaw; //front right
  W3_offset= pid_output_pitch - pid_output_roll;// + pid_output_yaw; //back right
  W4_offset= pid_output_pitch + pid_output_roll;// - pid_output_yaw; //back left
  

  esc_1 = throttle_speed - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)   
  esc_2 = throttle_speed + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
  esc_3 = throttle_speed + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
  esc_4 = throttle_speed - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)  
  

  // Serial.print(" |\tW1_offset= ");
  // Serial.print(W1_offset);
  // Serial.print(" |\tW2_offset= ");
  // Serial.print(W2_offset);
  // Serial.print(" |\tW3_offset= ");
  // Serial.print(W3_offset);
  // Serial.print(" |\tW4_offset= ");
  // Serial.print(W4_offset);

  tot_FL_speed = throttle_speed + W1_offset; //front left
  tot_FR_speed = throttle_speed + W2_offset; //front right
  tot_BR_speed = throttle_speed + W3_offset; //back right
  tot_BL_speed = throttle_speed + W4_offset; //back left

  Serial.print(" |\tFL= ");
  Serial.print(tot_FL_speed);
  Serial.print(" |\tFR= ");
  Serial.print(tot_FR_speed);
  Serial.print(" |\tBR= ");
  Serial.print(tot_BR_speed);
  Serial.print(" |\tBL= ");
  Serial.print(tot_BL_speed);

  //DO NOT REMOVE, SAFETY
  if (tot_FR_speed > SPEED_MID || tot_FL_speed > SPEED_MID || tot_BL_speed > SPEED_MID || tot_BR_speed > SPEED_MID) {
    myESC1.speed(SPEED_MIN);                                    // tell ESC to go to the oESC speed value
    myESC2.speed(SPEED_MIN); 
    myESC3.speed(SPEED_MIN);                                    // tell ESC to go to the oESC speed value
    myESC3.speed(SPEED_MIN); 
  } else {    
    myESC1.speed(tot_FR_speed);                                    // tell ESC to go to the oESC speed value
    myESC2.speed(tot_FL_speed);
    myESC3.speed(tot_BL_speed);                                    // tell ESC to go to the oESC speed value
    myESC4.speed(tot_BR_speed);
  }  

  delay(BNO055_SAMPLERATE_DELAY_MS);
}