#include <serLCD.h>
#include <radio.h>


typedef struct{
  int magic; 
  int throttle; 
  int yaw; 
  int roll; 
  int pitch; 
} Controller; 

Controller control_signals; 
serLCD lcd;


// Set pin values 
int yawPin = 0;
int throttlePin = 1;
int rollPin = 2;
int pitchPin = 3;

// Initialize gimbal values 
long yaw = 0;           
long throttle = 0;
long roll = 0;
long pitch = 0;



// Gimbals max/min settings 
long throttleMax = 864; 
long throttleMin = 127; 

long yawMax = 840; 
long yawMin = 94; 

long rollMax = 837; 
long rollMin = 96; 

long pitchMax = 840; 
long pitchMin = 94; 







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
  
  control_signals.magic = 0xBEAF; 

}

void print_lcd()
{
  lcd.clear(); 
   
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.setCursor(0, 2);
  lcd.print(throttle);

  lcd.setCursor(0, 8);
  lcd.print("Y:");
  lcd.setCursor(0, 10);
  lcd.print(yaw);

  lcd.setCursor(1, 0);
  lcd.print("R:");
  lcd.setCursor(1, 2);
  lcd.print(roll);
  
  lcd.setCursor(1, 8);
  lcd.print("P:");
  lcd.setCursor(1, 10);
  lcd.print(pitch);

}

void getGimbals()
{

  // scale throttle
  throttle = analogRead(throttlePin);
  throttle = convert(throttle, throttleMin, throttleMax, 0, 255); 
  
  // scale yaw
  yaw = analogRead(yawPin);
  yaw = convert(yaw, yawMin, yawMax, 0, 1024); 

  // scale roll
  roll = analogRead(rollPin);
  roll = convert(roll, rollMin, rollMax, 0, 1024);  

  // scale pitch
  pitch = analogRead(pitchPin);
  pitch = convert(pitch, pitchMin, pitchMax, 0, 1024); 
}

void sendRadio()
{
  control_signals.throttle = throttle; 
  control_signals.yaw = yaw; 
  control_signals.pitch = pitch; 
  control_signals.roll = roll; 


  rfWrite((uint8_t*)&control_signals, sizeof(Controller));
}

void loop()
{

  print_lcd(); 

  getGimbals(); 
  
  sendRadio(); 
  
}


