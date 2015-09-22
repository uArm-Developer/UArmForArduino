/******************************************************************************
* File Name          : uArm_library.cpp
* Author             : Evan 
* Updated            : Jerry Song 
* Email              : jerry.song@evol.net
* Version            : V1.2 
* Date               : 12 Dec, 2014
* Modified Date      : 26 Aug, 2015
* Description        : 
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*******************************************************************************/

#include "uArm_Library.h"

uArmClass uarm;
int angleR;
int angleL;
int angleBottom;

uArmClass::uArmClass()
{
  g_servo_offset = 0.0;
}

void uArmClass::alert(byte times, byte runTime, byte stopTime)
{
  for(int ct=0; ct < times; ct++)
  {
    delay(stopTime);
    digitalWrite(BUZZER, HIGH);
    delay(runTime);
    digitalWrite(BUZZER, LOW);
  }
}


/* The code below is written by jerry song */

void uArmClass::writeAngle(byte servo_rot_angle, byte servo_left_angle, byte servo_right_angle, byte servo_hand_rot_angle, byte trigger)
{
  attachAll();


  servoRot.write(round(servo_rot_angle));
  servoL.write(round(servo_left_angle));
  servoR.write(round(servo_right_angle));
  servoHandRot.write((round(servo_hand_rot_angle)));


}

void uArmClass::writeAngle(byte servo_rot_angle, byte servo_left_angle, byte servo_right_angle, byte servo_hand_rot_angle)
{
  attachAll();

  servoRot.write(inputToReal(SERVO_ROT_NUM,round(servo_rot_angle)));
  servoL.write(inputToReal(SERVO_LEFT_NUM,round(servo_left_angle)));
  servoR.write(inputToReal(SERVO_RIGHT_NUM,round(servo_right_angle)));
  servoHandRot.write(inputToReal(SERVO_HAND_ROT_NUM,round(servo_hand_rot_angle)));


}

void uArmClass::attachAll()
{
  

  servoRot.attach(11);
  servoL.attach(13);
  servoR.attach(12);
  servoHandRot.attach(10);
}

void uArmClass::detachAll()
{
  
  servoRot.detach();
  servoL.detach();
  servoR.detach();
  servoHandRot.detach();

}


byte uArmClass::inputToReal(byte servo_num, byte input_angle)
{
  return (byte)constrain((input_angle + readServoOffset(servo_num)),0,180);
}

double uArmClass::readServoOffset(byte servo_num)
{


  if ((servo_num == 1)||(servo_num == 2)||(servo_num == 3))
  {
    g_servo_offset = (EEPROM.read(kAddrOffset + (servo_num-1)*2 +1))/10.00;

    if (EEPROM.read(kAddrOffset + (servo_num-1)*2 ) == 0)
      {g_servo_offset = - g_servo_offset;}

    return g_servo_offset;
  } 
  else if(servo_num == 4)
    return 0;
  else{
      Serial.println("Incorrect");
      
  }
}



void uArmClass::saveDataToRom(double data, int address)
{
  
  int dataWhole;
  
  byte Byte0;
  byte Byte1;
  byte Byte2;

  if(abs(data) < 1) {
    dataWhole = (int) (data*100);
  }
  else{
    dataWhole = (int) (data*10);
  }



  if (dataWhole > 0){ 
    Byte0 = 1;
  }
  else{ 
    Byte0 =0; 
  }

  dataWhole = abs(dataWhole);

  Byte1 = (( dataWhole >> 0) & 0xFF);
  Byte2 = (( dataWhole >> 8) & 0xFF);
  

  EEPROM.write( address, Byte0);
  EEPROM.write( address + 1, Byte1);
  EEPROM.write( address + 2, Byte2);


}

  
    
double uArmClass::readToAngle(double input_analog, byte servo_num, byte trigger)
{
  int address = 60+(servo_num-1)*6;

  for (int i = 0; i<6;i++){
      

    }

  double data_a = (EEPROM.read(address+1)+EEPROM.read(address+2)*256)/10.0;
  if (EEPROM.read(address)==0)
    {data_a = -data_a;}

  double data_b = (EEPROM.read(address+4)+EEPROM.read(address+5)*256)/100.0;
  if (EEPROM.read(address+3)==0)
    {data_b = -data_b;}


  if (trigger == 0){
    return (data_a + data_b*input_analog) - readServoOffset(servo_num);
  }
  if (trigger == 1){
    return (data_a + data_b*input_analog);
  }
}



double uArmClass::readAngle(byte servo_num)
{

  switch (servo_num)
  {
    case SERVO_ROT_NUM:
      
      return readToAngle(analogRead(kServoRotReadPin),SERVO_ROT_NUM,0);
      break;

    case SERVO_LEFT_NUM:
      return readToAngle(analogRead(kServoLeftReadPin),SERVO_LEFT_NUM,0);
      break;

    case SERVO_RIGHT_NUM:
      return readToAngle(analogRead(kServoRightReadPin),SERVO_RIGHT_NUM,0);
      break;

    case SERVO_HAND_ROT_NUM:
      return readToAngle(analogRead(kServoHandRotReadPin),SERVO_HAND_ROT_NUM,0);
      break;

    default:
      
      break;


  }

}

double uArmClass::readAngle(byte servo_num, byte trigger)
{

  switch (servo_num)
  {
    case SERVO_ROT_NUM:
      return readToAngle(analogRead(kServoRotReadPin),SERVO_ROT_NUM,trigger);
      break;

    case SERVO_LEFT_NUM:
      return readToAngle(analogRead(kServoLeftReadPin),SERVO_LEFT_NUM,trigger);
      break;

    case SERVO_RIGHT_NUM:
      return readToAngle(analogRead(kServoRightReadPin),SERVO_RIGHT_NUM,trigger);
      break;

    case SERVO_HAND_ROT_NUM:
      return readToAngle(analogRead(kServoHandRotReadPin),SERVO_HAND_ROT_NUM,trigger);
      break;

    default:
      
      break;


  }

}
