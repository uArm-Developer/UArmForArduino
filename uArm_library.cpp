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


  g_servo_rot.write(round(servo_rot_angle));
  g_servo_left.write(round(servo_left_angle));
  g_servo_right.write(round(servo_right_angle));
  g_servo_hand_rot.write((round(servo_hand_rot_angle)));


}

void uArmClass::writeAngle(byte servo_rot_angle, byte servo_left_angle, byte servo_right_angle, byte servo_hand_rot_angle)
{
  attachAll();

  g_servo_rot.write(inputToReal(SERVO_ROT_NUM,round(servo_rot_angle)));
  g_servo_left.write(inputToReal(SERVO_LEFT_NUM,round(servo_left_angle)));
  g_servo_right.write(inputToReal(SERVO_RIGHT_NUM,round(servo_right_angle)));
  g_servo_hand_rot.write(inputToReal(SERVO_HAND_ROT_NUM,round(servo_hand_rot_angle)));


}

void uArmClass::attachAll()
{
  

  g_servo_rot.attach(11);
  g_servo_left.attach(13);
  g_servo_right.attach(12);
  g_servo_hand_rot.attach(10);
}

void uArmClass::detachAll()
{
  
  g_servo_rot.detach();
  g_servo_left.detach();
  g_servo_right.detach();
  g_servo_hand_rot.detach();

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

/*Action control */

void uArmClass::calAngles(double x, double y, double z)
{

   if (z > (MATH_L1 + MATH_L3 + TopOffset))
    {
        z = MATH_L1 + MATH_L3 + TopOffset;
    }

    if (z < (MATH_L1 - MATH_L4 + BottomOffset))
    {
        z = MATH_L1 - MATH_L4 + BottomOffset;
    }


  g_y_in = (-y-MATH_L2)/MATH_L3;
  g_z_in = (z - MATH_L1) / MATH_L3;
  g_right_all = (1 - g_y_in*g_y_in - g_z_in*g_z_in - MATH_L43*MATH_L43) / (2 * MATH_L43);
  g_sqrt_z_y = sqrt(g_z_in*g_z_in + g_y_in*g_y_in);
  

  if (x == 0)
  {
    // Calculate value of theta 1
    g_theta_1 = 90;

    // Calculate value of theta 3
    if (g_z_in == 0) {
      g_phi = 90;
    }

    else {
    g_phi = atan(-g_y_in / g_z_in)*MATH_TRANS;
    }

    if (g_phi > 0) g_phi = g_phi - 180;

    g_theta_3 = asin(g_right_all / g_sqrt_z_y)*MATH_TRANS - g_phi;
    if(g_theta_3<0)
      {
        g_theta_3 = 0;
      }

    // Calculate value of theta 2
    
    g_theta_2 = asin((z + MATH_L4*sin(g_theta_3 / MATH_TRANS) - MATH_L1) / MATH_L3)*MATH_TRANS;
  }

  else
  {
    // Calculate value of theta 1

    g_theta_1 = atan(y / x)*MATH_TRANS;

    if (y / x > 0) {
      g_theta_1 = g_theta_1;
    }
    if (y / x < 0) {
      g_theta_1 = g_theta_1 + 180;
    }
    if (y == 0) {
      if (x > 0) g_theta_1 = 180;
      else g_theta_1 = 0;       
    }
    
    // Calculate value of theta 3

    g_x_in = (-x / cos(g_theta_1 / MATH_TRANS) - MATH_L2) / MATH_L3;

    if (g_z_in == 0){ g_phi = 90; }
      
    else{ g_phi = atan(-g_x_in / g_z_in)*MATH_TRANS; }
      
    if (g_phi > 0) {g_phi = g_phi - 180;}  
    
    g_sqrt_z_x = sqrt(g_z_in*g_z_in + g_x_in*g_x_in);

    g_right_all_2 = -1 * (g_z_in*g_z_in + g_x_in*g_x_in + MATH_L43*MATH_L43 - 1) / (2 * MATH_L43);
    g_theta_3 = asin(g_right_all_2 / g_sqrt_z_x)*MATH_TRANS;
    g_theta_3 = g_theta_3 - g_phi;

    if (g_theta_3 <0 ) {
      g_theta_3 = 0;
    }


    // Calculate value of theta 2

    g_theta_2 = asin(g_z_in + MATH_L43*sin(abs(g_theta_3 / MATH_TRANS)))*MATH_TRANS;

  }
  
    
  g_theta_1 = abs(g_theta_1);
  g_theta_2 = abs(g_theta_2);
  
  
  if (g_theta_3 < 0 ){}
  else{
    if ((calYonly(g_theta_1,g_theta_2, g_theta_3)>y+0.1)||(calYonly(g_theta_1,g_theta_2, g_theta_3)<y-0.1))
    {
      g_theta_2 = 180 - g_theta_2;
    }  
  }
  

  if(isnan(g_theta_1)||isinf(g_theta_1))
    {g_theta_1 = uarm.readAngle(1);}
  if(isnan(g_theta_2)||isinf(g_theta_2))
    {g_theta_2 = uarm.readAngle(2);}
  if(isnan(g_theta_3)||isinf(g_theta_3)||(g_theta_3<0))
    {g_theta_3 = uarm.readAngle(3);}

}

void uArmClass::interpolation(double init_val, double final_val)
{
  // by using the formula theta_t = l_a_0 + l_a_1 * t + l_a_2 * t^2 + l_a_3 * t^3
  // theta(0) = init_val; theta(t_f) = final_val
  // theta_dot(0) = 0; theta_dot(t_f) = 0

  double l_a_0;
  double l_a_1;
  double l_a_2;
  double l_a_3;
  double l_t_step;
  
  byte l_time_total = 1;

  l_a_0 = init_val;
  l_a_1 = 0;
  l_a_2 = (3 * (final_val - init_val)) / (l_time_total*l_time_total);
  l_a_3 = (-2 * (final_val - init_val)) / (l_time_total*l_time_total*l_time_total);

  for (byte i = 0; i < 50; i=i+1)
  {
    l_t_step = (l_time_total / 50.0) *i;
    g_interpol_val_arr[i] = l_a_0 + l_a_1 * (l_t_step) + l_a_2 * (l_t_step *l_t_step ) + l_a_3 * (l_t_step *l_t_step *l_t_step);  
  }
}

void uArmClass::calXYZ(double theta_1, double theta_2, double theta_3)
{
  
  g_l3_1 = MATH_L3 * cos(theta_2 / MATH_TRANS);
  g_l4_1 = MATH_L4*cos(theta_3 / MATH_TRANS);
  g_l5 = (MATH_L2 + MATH_L3*cos(theta_2 / MATH_TRANS) + MATH_L4*cos(theta_3 / MATH_TRANS));

  g_cal_x = -cos(abs(theta_1 / MATH_TRANS))*g_l5;
  g_cal_y = -sin(abs(theta_1 / MATH_TRANS))*g_l5;
  g_cal_z = MATH_L1 + MATH_L3*sin(abs(theta_2 / MATH_TRANS)) - MATH_L4*sin(abs(theta_3 / MATH_TRANS));

}

void uArmClass::calXYZ()
{
  
  calXYZ(
  uarm.readToAngle(analogRead(2),1,0),
  uarm.readToAngle(analogRead(0),2,0),
  uarm.readToAngle(analogRead(1),3,0));

}

void uArmClass::moveTo(double x, double y, double z)
{
  uarm.attachAll();
  
  double l_current_x;
  double l_current_y;
  double l_current_z;

  double x_arr[50];
  double y_arr[50];
  double z_arr[50];

  calXYZ();
  l_current_x = g_cal_x;
  l_current_y = g_cal_y;
  l_current_z = g_cal_z;

  interpolation(l_current_x, x);
  for (byte i = 0; i < 50; i++){
    x_arr[i] = g_interpol_val_arr[i];
    
  }

  interpolation(l_current_y, y);
  for (byte i = 0; i < 50; i++){
    y_arr[i] = g_interpol_val_arr[i];
    
  }

  interpolation(l_current_z, z); 
  for (byte i = 0; i < 50; i++){
    z_arr[i] = g_interpol_val_arr[i];
    
  }
    
  for (byte i = 0; i < 50; i++)
  {
    calAngles(x_arr[i],y_arr[i],z_arr[i]);

    uarm.writeAngle(g_theta_1, g_theta_2, g_theta_3,0);

    delay(40);

  }

}

void uArmClass::moveTo(double x, double y, double z, int relative, double time_spend)
{

  double x_arr[50];
  double y_arr[50];
  double z_arr[50];

  calXYZ();
  g_current_x = g_cal_x;
  g_current_y = g_cal_y;
  g_current_z = g_cal_z;
  
  if ((relative !=0)&&(relative != 1))
  { 
    relative = 0;

  }

  // if (time_spend <0)
  // {  
  //  time_spend = abs(time_spend);
    
  // }

  interpolation(g_current_x, g_current_x*relative+x);

  for (byte i = 0; i < 50; i++){

    x_arr[i] = g_interpol_val_arr[i];
    
  }

  interpolation(g_current_y, g_current_y*relative+y);
  
  for (byte i = 0; i < 50; i++){

    y_arr[i] = g_interpol_val_arr[i];
    
  }

  interpolation(g_current_z, g_current_z*relative+z); 
  
  for (byte i = 0; i < 50; i++){

    z_arr[i] = g_interpol_val_arr[i];
    
  }
    
  for (byte i = 0; i < 50; i++)
  {
    calAngles(x_arr[i],y_arr[i],z_arr[i]);
    uarm.writeAngle(g_theta_1, g_theta_2, g_theta_3,0);

    delay(time_spend*1000/50);

  }

}


void uArmClass::moveTo(double x, double y, double z, int relative, double time_spend, double servo_4_angle)
{

  double x_arr[50];
  double y_arr[50];
  double z_arr[50];

  calXYZ();
  g_current_x = g_cal_x;
  g_current_y = g_cal_y;
  g_current_z = g_cal_z;

  if ((relative !=0)&&(relative != 1))
  { 
    relative = 0;
  }

  if (time_spend <0)
  { 
    time_spend = abs(time_spend);
  }

  interpolation(g_current_x, g_current_x*relative+x);

  for (byte i = 0; i < 50; i++){

    x_arr[i] = g_interpol_val_arr[i];
    
  }

  interpolation(g_current_y, g_current_y*relative+y);
  
  for (byte i = 0; i < 50; i++){

    y_arr[i] = g_interpol_val_arr[i];
    
  }

  if ( g_current_z*relative+z>25)
    { interpolation(g_current_z, 25); }
  else
    { interpolation(g_current_z, g_current_z*relative+z); }
  
  for (byte i = 0; i < 50; i++){

    z_arr[i] = g_interpol_val_arr[i];
    
  }
    
  for (byte i = 0; i < 50; i++)
  {
    calAngles(x_arr[i],y_arr[i],z_arr[i]);
    uarm.writeAngle(g_theta_1, g_theta_2, g_theta_3, servo_4_angle);

    delay(time_spend*1000/50);
  }

}

void uArmClass::drawRec(double length_1, double length_2, double time_spend_per_length)
{

  moveTo(length_1,0,0,1,time_spend_per_length);
  moveTo(0,length_2,0,1,time_spend_per_length);
  moveTo(-length_1,0,0,1,time_spend_per_length);
  moveTo(0,-length_2,0,1,time_spend_per_length);
  
}

void uArmClass::drawCur(double length_1, double length_2, int angle, double time_spend)
{
  uarm.attachAll();
  double l_xp;
  double l_yp;
  
  calXYZ();
  g_current_x = g_cal_x;
  g_current_y = g_cal_y;
  g_current_z = g_cal_z;

  interpolation(0, angle/MATH_TRANS); 

  
  for (byte i = 0; i < 50; i++){

    l_xp = length_1 * cos(g_interpol_val_arr[i]);
    l_yp = length_2 * sin(g_interpol_val_arr[i]);

    calAngles( l_xp + g_current_x - length_1 , l_yp+ g_current_y , g_current_z);

    uarm.writeAngle(g_theta_1, g_theta_2, g_theta_3,0);

    delay(time_spend*1000/50);
  
  }

}

double uArmClass::calYonly(double theta_1, double theta_2, double theta_3)

{

    g_l3_1_2 = MATH_L3 * cos(theta_2 / MATH_TRANS);
    g_l4_1_2 = MATH_L4*cos(theta_3 / MATH_TRANS);
    g_l5_2 = (MATH_L2 + MATH_L3*cos(theta_2 / MATH_TRANS) + MATH_L4*cos(theta_3 / MATH_TRANS));

    return -sin(abs(theta_1 / MATH_TRANS))*g_l5_2;

}

void uArmClass::gripperCatch()
{
  g_servo_hand.attach(SERVO_HAND);
  g_servo_hand.write(HAND_ANGLE_CLOSE);
  digitalWrite(VALVE_EN, LOW); // valve disnable
  digitalWrite(PUMP_EN, HIGH); // pump enable
  g_gripper_reset = true;
}

void uArmClass::gripperRelease()
{
  if(g_gripper_reset)
  {
    g_servo_hand.attach(SERVO_HAND);
    g_servo_hand.write(HAND_ANGLE_OPEN);
    digitalWrite(VALVE_EN, HIGH); // valve enable, decompression
    digitalWrite(PUMP_EN, LOW);   // pump disnable
    g_gripper_reset= false;
  }
}
/* Action Control */
