
#include "myQuadratureEncoder.h"

Encoders leftEncoder(2,3);  

// constants won't change. Used here to set a pin number:
const int dirPin =  7;// the number of the LED pin
const int pulsePin =  6;// the number of the LED pin



unsigned long lastMilli = 0;       // will store last time LED was updated

int flg=0;
int Z=0;
void setup() {
  // set the digital pin as output:
  pinMode(dirPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);

  pinMode(4, INPUT_PULLUP);

  Z=digitalRead(4);


  digitalWrite(dirPin, HIGH);  
  digitalWrite(pulsePin, LOW);  

  Serial.begin(9600);
}

void loop() {
    if (flg==0 && digitalRead(4)!=Z){
      Z=digitalRead(4);
      flg=1;
    }
    
    digitalWrite(pulsePin, HIGH);
    delay(2);
    digitalWrite(pulsePin, LOW);
    
    delay(5);
    if (flg==0 && digitalRead(4)!=Z){
      Z=digitalRead(4);
      flg=1;
    }
    
  if(millis()-lastMilli > 50){ 
    
    long currentLeftEncoderCount = leftEncoder.getEncoderCount();
    
    Serial.print(currentLeftEncoderCount);
    Serial.print(", flg = ");
    Serial.print(flg);
    Serial.print(" , Z=");
    Serial.println(Z);
    
    
    lastMilli = millis();
  }
}
