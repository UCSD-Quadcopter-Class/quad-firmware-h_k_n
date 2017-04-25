#include <serLCD.h>
#include <radio.h>

int yawPin = 0;
int throttlePin = 1;
int rollPin = 2;
int pitchPin = 3;

long yaw = 0;           // variable to store the value read
long throttle = 0;
long roll = 0;
long pitch = 0;

long throttlePWM = 0; 
long yawPWM  = 0; 
long rollPWM = 0; 
long pitchPWM = 0; 

serLCD lcd;
//
// throttle max = 864
long throttleMax = 864; 
// throttle min = 127
long throttleMin = 127; 
// yaw max = 840
long yawMax = 840; 
// yaw min = 94
long yawMin = 94; 
// roll max = 837
long rollMax = 837; 
// roll min = 96
long rollMin = 96; 
// pitch max = 840
long pitchMax = 840; 
// pitch min = 94
long pitchMin = 94; 

// throttle resting = 504
// 750 - 504 = 246
// yaw resting = 456
// 750 - 456 = 294
// roll resting = 459
// 750 - 459 = 291
// pitch resting = 468
// 750 - 468 = 282

// x0 = 127, x1 = 864, y0 = 0, y1 = 1500
// x0 = 94, x1 = 840, y0 = 0, y1 = 1500
// x0 = 96, x1 = 837, y0 = 0, y1 = 1500
// x0 = 94, x1 = 840, y0 = 1, y1 = 1500
// y = y0 + (y1 - y0) * (x - x0) / (x1 - x0)

long convert( long val, long minA, long maxA, long minB, long maxB)
{ 
  return ((val - minA) * (maxB-minB)) / (maxA-minA) + minB;

}

void setup()

{

  Serial.begin(9600);          //  setup serial
  rfBegin(23);
  lcd.home(); 
  lcd.display();

}



void loop()

{
  lcd.clear(); 
  // throttle
  // x0 = 127, x1 = 864, y0 = 0, y1 = 1500
  // yaw
  // x0 = 94, x1 = 840, y0 = 0, y1 = 1500
  // roll
  // x0 = 96, x1 = 837, y0 = 0, y1 = 1500
  // pitch
  // x0 = 94, x1 = 840, y0 = 1, y1 = 1500
  // y = y0 + (y1 - y0) * (x - x0) / (x1 - x0)

  // scale throttle
  throttle = analogRead(throttlePin);
  //throttle = convert(throttle, throttleMin, throttleMax, 0, 1500); 
  //throttle = 1500 * (throttle - 127) / (864 - 127);
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.setCursor(0, 2);
  lcd.print(convert(throttle, throttleMin, throttleMax, 0, 1500));

  // scale yaw
  yaw = analogRead(yawPin);
  //yaw = convert(yaw, yawMin, yawMax, 0, 1500); 
  //yaw = 1500  * (yaw - 94) / (830 - 94) ;
  lcd.setCursor(0, 8);
  lcd.print("Y:");
  lcd.setCursor(0, 10);
  lcd.print(convert(yaw, yawMin, yawMax, 0, 1500));




  // scale roll
  roll = analogRead(rollPin);
  //roll = convert(roll, rollMin, rollMax, 0, 1500); 
  //roll = 1500 * (roll - 96) / (837 - 96);
  lcd.setCursor(1, 0);
  lcd.print("R:");
  lcd.setCursor(1, 2);
  lcd.print(convert(roll, rollMin, rollMax, 0, 1500));


  // scale pitch
  pitch = analogRead(pitchPin);
  //pitch = convert(pitch, pitchMin, pitchMax, 0, 1500);
  //pitch = 1500 * (pitch - 94) / (840 - 94);
  lcd.setCursor(1, 8);
  lcd.print("P:");
  lcd.setCursor(1, 10);
  lcd.print(convert(pitch, pitchMin, pitchMax, 0, 1500));


  throttlePWM = convert(throttle, throttleMin, throttleMax, 0, 255); 
  if(throttlePWM > 255) throttlePWM = 255;
  else if(throttlePWM < 0) throttlePWM = 0; 
  //rfWrite('t'); 
  rfWrite(lowByte(throttlePWM));
  
  yawPWM = convert(yaw, yawMin, yawMax, 0, 255);
  if(yawPWM > 255) yawPWM = 255; 
  else if(yawPWM < 0) yawPWM = 0; 
  //rfWrite(lowByte(yawPWM));

  rollPWM = convert(roll, rollMin, rollMax, 0, 255); 
  if(rollPWM > 255) rollPWM = 255; 
  else if(rollPWM < 0) rollPWM = 0; 
  //rfWrite(lowByte(rollPWM));

  pitchPWM = convert(pitch, pitchMin, pitchMax, 0, 255); 
  if(pitchPWM > 255) pitchPWM = 255; 
  else if(pitchPWM < 0) pitchPWM = 0; 
  //rfWrite(lowByte(pitchPWM));
 

  Serial.print("yaw = ");
  Serial.println(yaw);
  Serial.print("throttle = ");
  Serial.println(throttle);
  Serial.print("roll = ");
  Serial.println(roll);
  Serial.print("pitch = ");
  Serial.println(pitch);
  Serial.println("###########################");
  Serial.print("yawPWM = ");
  Serial.println(yawPWM);
  Serial.print("throttlePWM = ");
  Serial.println(lowByte(throttlePWM));
  Serial.print("rollPWM = ");
  Serial.println(rollPWM);
  Serial.print("pitchPWM = ");
  Serial.println(pitchPWM);
  Serial.println("###########################");
  //delay(100);
}


