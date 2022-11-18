
#include "myQuadratureEncoder.h"
#include <Wire.h>
Encoders leftEncoder(2,3,4);  

// constants won't change. Used here to set a pin number:
const int dirPin =  7;// the number of the LED pin
const int pulsePin =  6;// the number of the LED pin

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
char command[25];
boolean newData = false;

unsigned long lastMilli = 0;       // will store last time LED was updated

int flg=0;
int Z=0;
int steploc=0;
int currsteploc=0;
bool cw=0;
char statusMsg[30];

void setup() {
  // set the digital pin as output:
  pinMode(dirPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);

//  pinMode(4, INPUT_PULLUP);

  Z=digitalRead(4);


  digitalWrite(dirPin, cw);  
  digitalWrite(pulsePin, LOW);  

  Wire.begin(0x33); 
  Wire.onRequest(reqData);
  Wire.onReceive(recvWithEndMarker);
  
//  Serial.begin(9600);

delay(800);
    leftEncoder.setEncoderCount(0);
}

void loop() {
    if (newData){
//      Serial.print("steploc=");
//      Serial.println(steploc);
//      Serial.print("currsteploc=");
//      Serial.println(currsteploc);

      if (steploc==currsteploc){
        sprintf(statusMsg, "<ok>");
        newData=false;
        return;
      }
      long prevLeftEncoderCount = leftEncoder.getEncoderCount();

      
      int i=0;
      int steps=0;
      if (steploc>currsteploc){
        digitalWrite(dirPin, cw);
        long totalencodercount = 10*(steploc-currsteploc);
        steps = steploc-currsteploc;
      }  
      else{
        digitalWrite(dirPin, !cw);
        long totalencodercount = 10*(currsteploc-steploc);
        steps = currsteploc-steploc;
      }
      
      while(i<steps){
        digitalWrite(pulsePin, HIGH);
        delay(3);
        digitalWrite(pulsePin, LOW);
        delay(5);
        ++i;
        
      }

      delay(10);
      long steploclong = long(steploc)*10; 
      while(abs(abs(leftEncoder.getEncoderCount())-steploclong)>10){
//        Serial.print(leftEncoder.getEncoderCount());
//        Serial.print(", steploc*10 = ");
//        Serial.println(steploclong);
        delay(100);
        }
      sprintf(statusMsg, "<ok>");
      
      currsteploc=steploc;
      steps=0;
      newData=false;

//      
    }




    
}
    
void reqData()
{
  Wire.write(statusMsg);
  sprintf(statusMsg, "<None>");
}


void recvWithEndMarker(int howMany)
{

   char rc;
    lastMilli = millis();
    int j=0;
    while(0 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    receivedChars[j]=c; 
    j=j+1;
    if(millis()-lastMilli>10)
      break;
  }
  
   receivedChars[j+1]='\0';
   j=j+1;

   bool st=0,et=0;
   int k=0;
   for(int i=0;i<j;++i){
    if (receivedChars[i]=='\0'){
      break;
    }
     if (receivedChars[i]=='<'){
        st=1;
        receivedChars[i]='\0';  
        continue;
     }
     if (receivedChars[i]=='>'){
      et=1;
      receivedChars[i]='\0';
      break;
     }
     if(st==1){
      command[k]=receivedChars[i];
      receivedChars[i]='\0';
      k=k+1;
     }
   }

   if(st==1 && et==1){
    command[k]='\0';
    newData = true;
   }
   else{
    command[0]='\0';
    newData = false;
   }

  parseCommand();
}

void parseCommand(){
  if(newData==false)
    return;
  int j=0;
  char c;
  steploc=0;
//  sprintf(statusMsg, "<None>");
  while(1)
  {
    c = command[j];
    if (c=='\0'){
      break;
    }
   ++j;

    if(c=='p'){
      sprintf(statusMsg, "<p>");
      newData=false;
      break;
    }
    else if(c=='f'){
        cw=!cw;
        leftEncoder.setEncoderCount(0);
        currsteploc=0;
        if(cw==0)
          sprintf(statusMsg, "<cw>");
        else
          sprintf(statusMsg, "<ccw>");
        newData=false;
        break;
    }
    else if(c=='s'){
        leftEncoder.setEncoderCount(0);
        currsteploc=0;
        newData=false;
        sprintf(statusMsg, "<s>");
        break;
    }
    else if(c=='g'){

        sprintf(statusMsg, "<%d>", currsteploc);

        newData=false;
        break;
    }
    else if(c=='e'){

         long LeftEncoderCount = leftEncoder.getEncoderCount();
        sprintf(statusMsg, "<%d>", LeftEncoderCount);

        newData=false;
        break;
    }
    else if(c=='0' || c=='1' || c=='2' || c=='3' || c=='4' || c=='5' || c=='6' || c=='7' || c=='8' || c=='9'){ 
        steploc = steploc * 10;
        steploc = steploc + (c - '0');  // Subtract '0' to adjust from ascii back to real numbers
      }
     
  }
  if(steploc>15999)
  {
    steploc=15999;
  }
  if(steploc<0)
  {
    steploc=0;
  }
}
