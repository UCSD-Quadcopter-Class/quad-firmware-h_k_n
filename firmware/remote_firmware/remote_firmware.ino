#include <serLCD.h>
#include <radio.h>


typedef struct{
  int magic; 
  int throttle; 
  int yaw; 
  int roll; 
  int pitch; 
  int potL; 
  int potR; 
} Controller; 

Controller control_signals; 
serLCD lcd;


// Set pin values 
int yawPin = 0;
int throttlePin = 1;
int rollPin = 2;
int pitchPin = 3;
int potRight = 6; 
int potLeft = 7; 

int buttonRedPin = 17; 
int buttonBluePin = 16; 

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


int armed; 
int potLeftVal,potRightVal; 


const int ledRedPin = 24;      // the number of the LED pin


int ledRedState = HIGH;         
int buttonRedState;             
int lastButtonRedState = LOW;  


unsigned long lastDebounceRedTime = 0;  
unsigned long debounceDelay = 50;    





long convert( long val, long minA, long maxA, long minB, long maxB)
{ 
  return ((val - minA) * (maxB-minB)) / (maxA-minA) + minB;

}

void setup()

{

  Serial.begin(9600);          //  setup serial
  pinMode(buttonRedPin, INPUT_PULLUP);           
  pinMode(buttonBluePin, INPUT_PULLUP);
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
  control_signals.potR = potRightVal; 
  control_signals.potL = potLeftVal; 


  rfWrite((uint8_t*)&control_signals, sizeof(Controller));
}

void getPots()
{
  // scale throttle
  potRightVal = analogRead(potRight);
  potRightVal = convert(potRightVal, 110, 818, 0, 500); 
  potLeftVal = analogRead(potLeft); 
  potLeftVal = convert(potLeftVal, 110, 818, 0, 500); 

  // min pot value = 110
  // max pot value = 818

  
}
void debugPots()
{
  Serial.print("pot right = "); 
  Serial.print(potRightVal); 
  Serial.print(", pot left = "); 
  Serial.println(potLeftVal); 
}
void loop()
{
  
  
  if(!armed){
    int reading = digitalRead(buttonRedPin);
    Serial.print("reading = "); 
    Serial.print(reading); 
    Serial.print(", throttle = "); 
    Serial.println(throttle); 
    if (reading == 0 & throttle < 20){ 
      digitalWrite(ledRedPin, HIGH); 
      digitalWrite(7, HIGH); 
      armed = 1; 
    }
  }
  
  
  print_lcd(); 

  getGimbals(); 


  getPots(); 
  //debugPots(); 
  
  if(armed){
    sendRadio(); 
  }
 
  
}


