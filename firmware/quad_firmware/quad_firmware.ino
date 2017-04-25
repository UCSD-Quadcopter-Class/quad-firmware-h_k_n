#include <radio.h>

int throttlePin = 19; 
unsigned char data = 0; 

void setup() {
  Serial.begin(9600);
  rfBegin(23);
  pinMode(throttlePin, OUTPUT); 
 
}

void loop() {
  if (rfAvailable()){
    data = rfRead(); 
    analogWrite(throttlePin, data);
    Serial.println(data); 
  }


}
