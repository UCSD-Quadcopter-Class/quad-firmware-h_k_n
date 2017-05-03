#include <radio.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h> 


Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}



typedef struct{
  int magic; 
  int throttle; 
  int yaw; 
  int roll; 
  int pitch; 
} Controller; 

Controller control_signals; 

// set motor pins 
int motor1Pin = 19; 
int motor2Pin = 9; 
int motor3Pin = 8; 
int motor4Pin = 5;

// initialize gimbal values 
long yaw_target = 0;           
long throttle = 0;
long roll_target = 0;
long pitch_target = 0; 


// imu readings
int roll_read = 0;  
int pitch_read = 0;
int yaw_read = 0; 

// pid err
int roll_err = 0; 
int roll_prev_err = 0; 
int pitch_err = 0; 
int pitch_prev_err = 0; 
int yaw_err = 0; 
int yaw_prev_err = 0; 

// pid integral
int roll_integ = 0; 
int pitch_integ = 0; 
int yaw_integ = 0; 

// pid derivatives
int roll_deriv = 0; 
int pitch_deriv = 0; 
int yaw_deriv = 0; 


// pid variables
double kp = 1; 
double ki = 0.5; 
double kd = 1; 


// pid motor settings 
double roll_motor = 0; 
double pitch_motor = 0; 
double yaw_motor = 0; 


 
unsigned char data = 0; 

void setup() {
  Serial.begin(9600);
  rfBegin(23);
  pinMode(motor1Pin, OUTPUT); 
  setupSensor(); 
}

void printIMU()
{

  Serial.print("IMU roll = "); 
  Serial.println(roll_read); 
  Serial.print("IMU pitch = "); 
  Serial.println(pitch_read); 
  Serial.print("IMU yaw = "); 
  Serial.println(yaw_read); 
}

void pid()
{ 
  roll_err = roll_target - roll_read;
  roll_integ = (roll_integ/2) + roll_err;
  roll_deriv = roll_err - roll_prev_err;
  roll_prev_err = roll_err; 
  roll_motor = (kp*roll_err) + (ki*roll_integ) + (kd+roll_deriv);


  pitch_err = pitch_target - pitch_read;
  pitch_integ = (pitch_integ/2) + pitch_err;
  pitch_deriv = pitch_err - pitch_prev_err;
  pitch_prev_err = pitch_err; 
  pitch_motor = (kp*pitch_err) + (ki*pitch_integ) + (kd+pitch_deriv);

  yaw_err = yaw_target - yaw_read;
  yaw_integ = (yaw_integ/2) + yaw_err;
  yaw_deriv = yaw_err - yaw_prev_err;
  yaw_prev_err = yaw_err; 
  yaw_motor = (kp*yaw_err) + (ki*yaw_integ) + (kd+yaw_deriv);
}

void adjust_motors()
{
  // TODO: scale down roll_motor, pitch_motor, and yaw_motor
  analogWrite(motor1Pin, throttle + roll_motor - yaw_motor);
  analogWrite(motor2Pin, throttle + pitch_motor + yaw_motor); 
  analogWrite(motor3Pin, throttle - roll_motor - yaw_motor); 
  analogWrite(motor4Pin, throttle - pitch_motor + yaw_motor); 
  
}

void getIMU()
{ 

  sensors_vec_t orientation;
  ahrs.getOrientation(&orientation); 

  roll_read = orientation.roll; 
  pitch_read = orientation.pitch; 
  yaw_read = orientation.heading; 
  
}

void getRadio()
{
  if(rfAvailable()) {
    char bytesRead = rfRead((uint8_t*)&control_signals, sizeof(Controller)); 
    if(bytesRead == 10 && control_signals.magic == 0xBEAF) { 
      yaw_target = control_signals.yaw; 
      throttle = control_signals.throttle; 
      roll_target = control_signals.roll; 
      pitch_target = control_signals.pitch;  
    }
  }
  
}
void loop() {

  getRadio(); 
  getIMU(); 
  pid(); 
  adjust_motors(); 
  
}
