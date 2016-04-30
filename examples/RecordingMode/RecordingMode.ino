#include <EEPROM.h>
#include "uarm_recording_mode.h"


char dataBuf[100] = {0};
bool startFlag = false;
bool endFlag = false;
int counter = 0;

void setup() {
  Serial.begin(115200);
  uarmRecord.init();          // initialize the uArm position
}

void loop() {
  uarmRecord.record(40);
  readSerial(40);
}

void readMode(){
  if(Serial.available())
  {
    char schar = Serial.read();
    counting(schar);
  }  
}

void readSerial(unsigned char playDelay){
  if(Serial.available())
  {
    char schar = Serial.read();
    counting(schar);
  }  
  if ( startFlag & endFlag){
    String stringBuf = String(dataBuf);
//    Serial.print("String Length: ");
//    Serial.println(stringBuf.length());
    int index = stringBuf.indexOf(",");
    int beginIndex = 0;  
    String arg;
    int mode = 0;
    long address = 0;
    int X = 0;
    int Y = 0;
    int Z = 0;
    int H = 0;    
    while(index > -1){
      arg = stringBuf.substring(beginIndex,index);
      //Serial.println(arg);
      beginIndex = index + 1;
      index = stringBuf.indexOf(",", beginIndex);
      if(arg.startsWith("M")){
         mode = (arg.substring(1, arg.length())).toInt();
      }
      else if(arg.startsWith("X")){
        X = (arg.substring(1, arg.length())).toInt();
      }
      else if(arg.startsWith("Y")){
        Y = (arg.substring(1, arg.length())).toInt();
      }
      else if(arg.startsWith("Z")){
        Z = (arg.substring(1, arg.length())).toInt();
      }
      else if(arg.startsWith("H")){
        H = (arg.substring(1, arg.length())).toInt();
      }                       
    }    
    arg = stringBuf.substring(beginIndex,index);
    if(arg.startsWith("M")){
       mode = (arg.substring(1, arg.length())).toInt();
    }
    else if(arg.startsWith("X")){
      X = (arg.substring(1, arg.length())).toInt();
    }
    else if(arg.startsWith("Y")){
      Y = (arg.substring(1, arg.length())).toInt();
    }
    else if(arg.startsWith("Z")){
      Z = (arg.substring(1, arg.length())).toInt();
    }
    else if(arg.startsWith("H")){
      H = (arg.substring(1, arg.length())).toInt();    
    }
    Serial.print("X:");
    Serial.print(X);
    Serial.print(", Y:");
    Serial.print(Y);
    Serial.print(", Z:");
    Serial.print(Z);
    Serial.print(", H:");
    Serial.println(H);    
    uarmRecord.play(X,Y,Z,H,playDelay);
    startFlag = false;
    endFlag = false;
    delay(50);    
  }

}

void counting(char schar){
  if(!startFlag && !endFlag) {
    if (schar == '[' ){
      startFlag = true;
      counter = 0;
    }
  } else if (startFlag && !endFlag) {
    if (schar == ']'){
      endFlag = true;
      dataBuf[counter] = '\0';
    }
    else {
      dataBuf[counter] = schar;
      counter++;
    } 
  }
}