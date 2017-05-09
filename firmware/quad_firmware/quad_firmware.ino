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


long convert( long val, long minA, long maxA, long minB, long maxB)
{ 
  return ((val - minA) * (maxB-minB)) / (maxA-minA) + minB;

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
double ki = 0.1; 
double kd = 1; 


// pid motor settings 
double roll_motor = 0; 
double pitch_motor = 0; 
double yaw_motor = 0; 

int roll_readings[] = {0,0,0,0,0,0,0};
int pitch_readings[] = {0,0,0,0,0,0,0};
int yaw_readings[] = {0,0,0,0,0,0,0};

int roll_avg = 0; 
int pitch_avg = 0; 
int yaw_avg = 0; 

int sliding_avg_window = 7; 
 
unsigned char data = 0; 

void setup() {
  //Serial.begin(9600);
  Serial.begin(115200);
  Serial.println("hello"); 
  rfBegin(23);
  pinMode(motor1Pin, OUTPUT); 

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
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
  delay(500); 
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
  //Serial.println("adjusting motors"); 
  // TODO: scale down roll_motor, pitch_motor, and yaw_motor
  int speed1,speed2,speed3,speed4; 
  if (throttle < 30) {
    speed1 = 0; 
    speed2 = 0; 
    speed3 = 0; 
    speed4 = 0; 
  }
  else{
    //speed1 = throttle + roll_motor - yaw_motor; 
    //speed2 = throttle + pitch_motor + yaw_motor; 
    speed1 = throttle - pitch_motor - yaw_motor; 
    speed2 = throttle - pitch_motor + yaw_motor; 
    speed3 = throttle + pitch_motor - yaw_motor; 
    speed4 = throttle + pitch_motor + yaw_motor; 
    //speed3 = throttle - roll_motor - yaw_motor;
    //speed4 = throttle - pitch_motor + yaw_motor; 
  }
  if(speed1 > 255) speed1 = 255; 
  if(speed2 > 255) speed2 = 255; 
  if(speed3 > 255) speed3 = 255; 
  if(speed4 > 255) speed4 = 255; 

  if(speed1 < 0) speed1 = 0; 
  if(speed2 < 0) speed2 = 0; 
  if(speed3 < 0) speed3 = 0; 
  if(speed4 < 0) speed4 = 0;
  analogWrite(motor1Pin, speed1);
  analogWrite(motor2Pin, speed2); 
  analogWrite(motor3Pin, speed3); 
  analogWrite(motor4Pin, speed4); 
  
}

void getIMU()
{ 

  sensors_vec_t orientation;
  ahrs.getOrientation(&orientation); 

  int new_roll_read = orientation.roll; 
  int new_pitch_read = orientation.pitch; 
  int new_yaw_read = orientation.heading; 


  roll_avg = roll_avg + (new_roll_read/sliding_avg_window) - (roll_readings[6]/sliding_avg_window); 
  pitch_avg = pitch_avg + (new_pitch_read/sliding_avg_window) - (pitch_readings[6]/sliding_avg_window); 
  yaw_avg = yaw_avg + (new_yaw_read/sliding_avg_window) - (yaw_readings[6]/sliding_avg_window); 

  for (int i = 6; i >0 ; i--){ 
    roll_readings[i] = roll_readings[i-1]; 
    pitch_readings[i] = pitch_readings[i-1]; 
    yaw_readings[i] = yaw_readings[i-1];  
  }
  roll_readings[0] = new_roll_read; 
  pitch_readings[0] = new_pitch_read; 
  yaw_readings[0] = new_yaw_read; 
  
  roll_read = roll_avg; 
  pitch_read = pitch_avg; 
  yaw_read = yaw_avg; 

  /*
  roll_read = orientation.roll; 
  pitch_read = orientation.pitch; 
  yaw_read = orientation.heading; 
  */
  
}

void getRadio()
{
  if(rfAvailable()) {
    char bytesRead = rfRead((uint8_t*)&control_signals, sizeof(Controller)); 
    if(bytesRead == 10 && control_signals.magic == 0xBEAF) { 
      yaw_target = control_signals.yaw; 
      yaw_target = convert(yaw_target, 0, 1024, -100, 100);
      throttle = control_signals.throttle; 
      roll_target = control_signals.roll; 
      roll_target = convert(roll_target, 0, 1024, -100, 100); 
      pitch_target = control_signals.pitch;  
      pitch_target = convert(pitch_target, 0, 1024, -100, 100); 
      //Serial.println(throttle); 
    }
  }
  
}

void debug(){
  /*
  Serial.print("throttle = "); 
  Serial.println(throttle); 
  Serial.print("pitch adjust = "); 
  Serial.println(pitch_motor); 
  Serial.print("roll adjust = "); 
  Serial.println(roll_motor); 
  */
  /*
  Serial.print("pitch err = ");
  Serial.println(pitch_err); 
  Serial.print("pitch deriv = "); 
  Serial.println(pitch_deriv); 
  Serial.print("pitch integ = "); 
  Serial.println(pitch_integ); 
  */

  Serial.print(40); 
  Serial.print(" "); 
  Serial.print(-40);
  Serial.print(" ");  
  Serial.print(throttle - pitch_motor); 
  Serial.print(" "); 
  Serial.println(throttle + pitch_motor);

  /*
  Serial.print(40); 
  Serial.print(" "); 
  Serial.print(-40);
  Serial.print(" ");  
  Serial.print(pitch_err); 
  Serial.print(" "); 
  Serial.println(pitch_deriv);
  */
  /*
  Serial.print(40); 
  Serial.print(" "); 
  Serial.print(-40);
  Serial.print(" ");  
  Serial.print(yaw_err); 
  Serial.print(" "); 
  Serial.println(yaw_deriv);
  */
  //Serial.print("yaw adjust = "); 
  //Serial.println(yaw_motor); 
  //delay(00); 
  
}
void loop() {
  //Serial.println("starting loop"); 
  getRadio(); 
  getIMU(); 
  pid(); 
  adjust_motors();
  debug();  
  //printIMU(); 
  
}
