
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
bool cw=0;


void setup() {
  // set the digital pin as output:
  pinMode(dirPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);

//  pinMode(4, INPUT_PULLUP);

  Z=digitalRead(4);


  digitalWrite(dirPin, cw);  
  digitalWrite(pulsePin, LOW);  

  Serial.begin(9600);

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
        Serial.println("<ok>");
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
        Serial.print(leftEncoder.getEncoderCount());
        Serial.print(", steploc*10 = ");
        Serial.println(steploclong);
        delay(100);
        }
      Serial.println("<ok>");
      
      currsteploc=steploc;
      steps=0;
      newData=false;
//      Serial.print("steploc=");
//      Serial.println(steploc);
//      Serial.print("currsteploc=");
//      Serial.println(currsteploc);
//      Serial.println("--------");
//      
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
//   while (millis()-lastMilli < 100)
//   {
//      rc = Serial.read();
//      receivedChars[j]=rc;  
//      j=j+1;
//      if (Serial.available()<=0)
//      break;
//      
//   }
   j=Serial.readBytesUntil('>', receivedChars, 30);
   receivedChars[j]='>';
   receivedChars[j+1]='\0';
   j=j+1;
//   receivedChars[j] = '\0'; // terminate the string
//   Serial.println(j);
//   Serial.println(receivedChars);
   
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
//    Serial.print(command);
//    Serial.print(',');
//    Serial.print(st);
//    Serial.print(',');
//    Serial.println(et);
   if(st==1 && et==1){
    command[k]='\0';
    newData = true;
   }
   else{
    command[0]='\0';
    newData = false;
   }
//  Serial.print("command=");
//  Serial.println(command);
//  Serial.print("newData=");
//  Serial.println(newData); 
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
    if (c=='\0'){
      break;
    }
   ++j;

    if(c=='p'){
      Serial.println("<p>");
      newData=false;
      break;
    }
    else if(c=='f'){
        cw=!cw;
        leftEncoder.setEncoderCount(0);
        currsteploc=0;
        if(cw==0)
          Serial.println("<cw>");
        else
          Serial.println("<ccw>");
        newData=false;
        break;
    }
    else if(c=='s'){
        leftEncoder.setEncoderCount(0);
        currsteploc=0;
        newData=false;
        Serial.println("<s>");
        break;
    }
    else if(c=='g'){
        char buff[25];
        sprintf(buff, "<%d>", currsteploc);
        Serial.println(buff);
        newData=false;
        break;
    }
    else if(c=='e'){
        char buff[25];
         long LeftEncoderCount = leftEncoder.getEncoderCount();
        sprintf(buff, "<%d>", LeftEncoderCount);
        Serial.println(buff);
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
