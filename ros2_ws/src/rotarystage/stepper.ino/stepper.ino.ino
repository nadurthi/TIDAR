
#include "myQuadratureEncoder.h"

Encoders leftEncoder(2,3,4);  

// constants won't change. Used here to set a pin number:
const int dirPin =  7;// the number of the LED pin
const int pulsePin =  6;// the number of the LED pin

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

unsigned long lastMilli = 0;       // will store last time LED was updated

int flg=0;
int Z=0;
int steps=0;
void setup() {
  // set the digital pin as output:
  pinMode(dirPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);

//  pinMode(4, INPUT_PULLUP);

  Z=digitalRead(4);


  digitalWrite(dirPin, LOW);  
  digitalWrite(pulsePin, LOW);  

  Serial.begin(9600);


    leftEncoder.setEncoderCount(0);
}

void loop() {
    if (steps>0){
//      leftEncoder.setEncoderCount(0);
      long prevLeftEncoderCount = leftEncoder.getEncoderCount();
      long currentLeftEncoderCount=prevLeftEncoderCount;
      long totalencodercount = 10*(steps-1);
      int i=0;
//      Serial.print(steps);
//      Serial.print(", ");
//      Serial.print(prevLeftEncoderCount);
//      Serial.print(", ");
//      Serial.print(currentLeftEncoderCount);
//      Serial.print(", ");
//      Serial.println(totalencodercount);
      while(i<steps || (abs(currentLeftEncoderCount-prevLeftEncoderCount)<totalencodercount)){
        digitalWrite(pulsePin, HIGH);
        delay(1);
        digitalWrite(pulsePin, LOW);
        delay(2);
        ++i;
        currentLeftEncoderCount = leftEncoder.getEncoderCount();
      }
      Serial.println("ok");
      steps=0;
    }

    int j=0;
    if (Serial.available()>0){
      while(1)
      {
       char c = Serial.read();
       receivedChars[j]=c;
       ++j;
       if(c == '\n'){
          break;
       }
       
        if(c=='-')
            digitalWrite(dirPin, HIGH);
        else if(c=='+') 
            digitalWrite(dirPin, LOW);  
        else if(c=='s'){
            leftEncoder.setEncoderCount(0);
            Serial.println("ok");
        }
        else if(c=='g'){
            long EncoderCount = leftEncoder.getEncoderCount();
            Serial.println(EncoderCount);
            break;
        }
        else if(c=='h'){

          long EncoderCount = leftEncoder.getEncoderCount();
          if(EncoderCount>0)
            digitalWrite(dirPin, HIGH);
          else
            digitalWrite(dirPin, LOW);
          steps = abs(EncoderCount)/10-1;
          break;
         }
        else if(c=='0' || c=='1' || c=='2' || c=='3' || c=='4' || c=='5' || c=='6' || c=='7' || c=='8' || c=='9'){ 
            steps = steps * 10;
            steps = steps + (c - '0');  // Subtract '0' to adjust from ascii back to real numbers
          }
         
      }
      while(Serial.available()>0)
         char c = Serial.read();
    }
    
  
//  if(millis()-lastMilli > 50){ 
//    
//    long currentLeftEncoderCount = leftEncoder.getEncoderCount();
//    long currentRev = leftEncoder.getEncoderRev();
//    Serial.print("steps = ");
//    Serial.print(steps);
//    Serial.print(", revs = ");
//    Serial.print(currentRev);
//    Serial.print(" , count = ");
//    Serial.println(currentLeftEncoderCount);
//
//    
//    
//    lastMilli = millis();
//  }
}
