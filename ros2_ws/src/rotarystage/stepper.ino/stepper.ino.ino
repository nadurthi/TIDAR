
#include "myQuadratureEncoder.h"

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
    if (steploc!=currsteploc){

      long prevLeftEncoderCount = leftEncoder.getEncoderCount();
      long currentLeftEncoderCount=prevLeftEncoderCount;
      
      int i=0;
      int steps=0;
      if (steploc>currsteploc){
        digitalWrite(dirPin, LOW);
        long totalencodercount = 10*(steploc-currsteploc);
        steps = steploc-currsteploc;
      }  
      else{
        digitalWrite(dirPin, HIGH);
        long totalencodercount = 10*(currsteploc-steploc);
        steps = currsteploc-steploc;
      }
      
      while(i<steploc){
        digitalWrite(pulsePin, HIGH);
        delay(1);
        digitalWrite(pulsePin, LOW);
        delay(2);
        ++i;
        currentLeftEncoderCount = leftEncoder.getEncoderCount();
      }
      Serial.println("<>");
      currsteploc=steploc;
      steploc=0;
      steps=0;
    }


    if (Serial.available()>0){
      recvWithEndMarker();
      parseCommand();
    }

    
    }
    



void recvWithEndMarker()
{

   char rc;
    lastMilli = millis();
    int j=0;
   while (millis()-lastMilli < 100)
   {
      rc = Serial.read();
      receivedChars[j]=rc;  
      j=j+1;
      
   }
   receivedChars[j] = '\0'; // terminate the string
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
      et==1;
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
   
}

void parseCommand(){
  if(newData==false)
    return;
  int j=0;
  char c;
  steploc=0;
  while(1)
  {
    c = command[j];
    if (c=='\0')
      break;
   ++j;

    if(c=='p'){
      Serial.println("<>");
    }
    else if(c=='s'){
        leftEncoder.setEncoderCount(0);
        currsteploc=0;
        Serial.println("<>");
    }
    else if(c=='g'){
        char buff[25];
        sprintf(buff, "<%d>", currsteploc);
        Serial.println(buff);
        break;
    }
    else if(c=='e'){
        char buff[25];
         long LeftEncoderCount = leftEncoder.getEncoderCount();
        sprintf(buff, "<%d>", LeftEncoderCount);
        Serial.println(buff);
        break;
    }
    else if(c=='0' || c=='1' || c=='2' || c=='3' || c=='4' || c=='5' || c=='6' || c=='7' || c=='8' || c=='9'){ 
        steploc = steploc * 10;
        steploc = steploc + (c - '0');  // Subtract '0' to adjust from ascii back to real numbers
      }
     
  }
  newData=false;
}
