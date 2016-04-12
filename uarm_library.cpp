/******************************************************************************
* File Name          : uArm_library.cpp
* Author             : Joey Song  
* Updated            : Joey Song, Alex Tan, Dave Corboy
* Email              : joey@ufactory.cc
* Version            : V1.3.1 
* Date               : 12 Dec, 2014
* Modified Date      : 17 Dec, 2015
* Description        : 
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*******************************************************************************/

#include "uarm_library.h"

uArmClass uarm;

uArmClass::uArmClass()
{
  g_offset_servo_rot = readEEPROMServoOffset(SERVO_ROT_NUM);
  g_offset_servo_left = readEEPROMServoOffset(SERVO_LEFT_NUM);
  g_offset_servo_right = readEEPROMServoOffset(SERVO_RIGHT_NUM);
  g_offset_servo_hand_rot = 0;
  g_offset_old_servo_left = EEPROM.read(2);
  if(EEPROM.read(1) == 1)
    g_offset_old_servo_left = -g_offset_old_servo_left;
  g_offset_old_servo_right = EEPROM.read(4);
  if(EEPROM.read(3) == 1)
    g_offset_old_servo_right = -g_offset_old_servo_right;
}

void uArmClass::init()
{
  moveTo(0, 15, 15);
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

  writeServoAngle(SERVO_ROT_NUM,round(servo_rot_angle),false);
  writeServoAngle(SERVO_LEFT_NUM,round(servo_left_angle),false);
  writeServoAngle(SERVO_RIGHT_NUM,round(servo_right_angle),false);
  writeServoAngle(SERVO_HAND_ROT_NUM,round(servo_hand_rot_angle),false);


  // refresh logical servo angle cache
  cur_rot = readAngle(SERVO_ROT_NUM);
  cur_left = readAngle(SERVO_LEFT_NUM);
  cur_right = readAngle(SERVO_RIGHT_NUM);
  cur_hand = readAngle(SERVO_HAND_ROT_NUM);
}

void uArmClass::writeServoAngle(byte servo_number, double servo_angle, boolean writeWithoffset)
{
  attachServo(servo_number);
  servo_angle = writeWithoffset ? round(inputToReal(servo_number,servo_angle)): round(servo_angle);
  servo_angle = constrain(servo_angle,0,180);
  switch(servo_number)
  {
    case SERVO_ROT_NUM:       g_servo_rot.write(servo_angle);
                              break;
    // case SERVO_LEFT_NUM:      
                              // g_servo_left.write(servo_angle);
                              // break;
    // case SERVO_RIGHT_NUM:     g_servo_right.write(servo_angle);
                              // break;
    case SERVO_HAND_ROT_NUM:  g_servo_hand_rot.write(servo_angle);
                              break;                        
    default:                  break;
  }
}

void uArmClass::writeLeftRightServoAngle(double servo_left_angle, double servo_right_angle, boolean writeWithoffset)
{
  servo_left_angle = constrain(servo_left_angle,0,120);
  servo_right_angle = constrain(servo_right_angle,0,120);
  servo_left_angle = writeWithoffset ? round(inputToReal(SERVO_LEFT_NUM,servo_left_angle)): round(servo_left_angle);
  servo_right_angle = writeWithoffset ? round(inputToReal(SERVO_RIGHT_NUM,servo_right_angle)): round(servo_right_angle);
  // Serial.println(servo_left_angle);
  // Serial.println(servo_right_angle);
  if(servo_left_angle + servo_right_angle > 180) 
  {
    // servo_right_angle = 160 - servo_left_angle;
    alert(1, 10, 0);
    // delay(10);
    return;
  }
    attachServo(SERVO_LEFT_NUM);
    attachServo(SERVO_RIGHT_NUM);    
    g_servo_left.write(servo_left_angle);
    g_servo_right.write(servo_right_angle);
}

void uArmClass::writeAngle(double servo_rot_angle, double servo_left_angle, double servo_right_angle, double servo_hand_rot_angle)
{
  attachAll();
  
  if(servo_left_angle < 10) servo_left_angle = 10;
  if(servo_left_angle > 120) servo_left_angle = 120;
  if(servo_right_angle < 10) servo_right_angle = 10;
  if(servo_right_angle > 110) servo_right_angle = 110;

  if(servo_left_angle + servo_right_angle > 160) 
  {
    servo_right_angle = 160 - servo_left_angle;
    return;
  }
  writeServoAngle(SERVO_ROT_NUM,servo_rot_angle,true);
  writeServoAngle(SERVO_LEFT_NUM,servo_left_angle,true);
  writeServoAngle(SERVO_RIGHT_NUM,servo_right_angle,true);
  writeServoAngle(SERVO_HAND_ROT_NUM,servo_hand_rot_angle,true);
  

  // refresh logical servo angle cache
  cur_rot = servo_rot_angle;
  cur_left = servo_left_angle;
  cur_right = servo_right_angle;
  cur_hand = servo_hand_rot_angle;
}

void uArmClass::attachAll()
{
  attachServo(SERVO_ROT_NUM);
  attachServo(SERVO_LEFT_NUM);
  attachServo(SERVO_RIGHT_NUM);
  attachServo(SERVO_HAND_ROT_NUM);
}

void uArmClass::attachServo(byte servo_number)
{
  switch(servo_number){
    case SERVO_ROT_NUM: 
    if(!g_servo_rot.attached()) {
      g_servo_rot.attach(SERVO_ROT_PIN);
      cur_rot = readAngle(SERVO_ROT_NUM);
    }
    break;
    case SERVO_LEFT_NUM:
    if (!g_servo_left.attached()) {
      g_servo_left.attach(SERVO_LEFT_PIN);
      cur_left = readAngle(SERVO_LEFT_NUM);
    }
    break;
    case SERVO_RIGHT_NUM:
    if (!g_servo_right.attached()) {
      g_servo_right.attach(SERVO_RIGHT_PIN);
      cur_right = readAngle(SERVO_RIGHT_NUM);
    } 
    break;
    case SERVO_HAND_ROT_NUM:
    if (!g_servo_hand_rot.attached()) {
      g_servo_hand_rot.attach(SERVO_HAND_PIN);
      cur_hand = readAngle(SERVO_HAND_ROT_NUM);
    }
    break;    
  }
}

void uArmClass::detachAll()
{
  g_servo_rot.detach();
  g_servo_left.detach();
  g_servo_right.detach();
  g_servo_hand_rot.detach();
}

byte uArmClass::inputToReal(byte servo_num, double input_angle)
{
  return (byte)round(constrain((input_angle + readServoOffset(servo_num)),0,180));
}

double uArmClass::readServoOffset(byte servo_num)
{
    switch(servo_num){
      case SERVO_ROT_NUM: return g_offset_servo_rot;
      case SERVO_LEFT_NUM: return g_offset_servo_left;
      case SERVO_RIGHT_NUM: return g_offset_servo_right;
      case SERVO_HAND_ROT_NUM: return g_offset_servo_hand_rot;
      default : return 0;
    }
}

double uArmClass::readEEPROMServoOffset(byte servo_num){
  double offset = (EEPROM.read(LINEAR_START_ADDRESS + (servo_num-1)*2 +1))/10.00;
  if (EEPROM.read(LINEAR_START_ADDRESS + (servo_num-1)*2 ) == 0)
      offset = - offset;
  return offset;
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
  int address = OFFSET_START_ADDRESS +(servo_num-1)*6;

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
  return readAngle(servo_num, 0);
}

double uArmClass::readAngle(byte servo_num, byte trigger)
{
  switch (servo_num)
  {
    case SERVO_ROT_NUM:
      return readToAngle(analogRead(SERVO_ROT_ANALOG_PIN),SERVO_ROT_NUM,trigger);
      break;

    case SERVO_LEFT_NUM:
      return readToAngle(analogRead(SERVO_LEFT_ANALOG_PIN),SERVO_LEFT_NUM,trigger);
      break;

    case SERVO_RIGHT_NUM:
      return readToAngle(analogRead(SERVO_RIGHT_ANALOG_PIN),SERVO_RIGHT_NUM,trigger);
      break;

    case SERVO_HAND_ROT_NUM:
      return readToAngle(analogRead(SERVO_HAND_ROT_ANALOG_PIN),SERVO_HAND_ROT_NUM,trigger);
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

  double x_in = 0.0;
  double y_in = 0.0;
  double z_in = 0.0;
  double right_all = 0.0;
  double right_all_2 = 0.0;
  double sqrt_z_x = 0.0;
  double sqrt_z_y = 0.0;
  double phi = 0.0;

  y_in = (-y-MATH_L2)/MATH_L3;
  z_in = (z - MATH_L1) / MATH_L3;
  right_all = (1 - y_in*y_in - z_in*z_in - MATH_L43*MATH_L43) / (2 * MATH_L43);
  sqrt_z_y = sqrt(z_in*z_in + y_in*y_in);

  if (x == 0)
  {
    // Calculate value of theta 1
    g_theta_1 = 90;

    // Calculate value of theta 3
    if (z_in == 0) {
      phi = 90;
    }

    else {
    phi = atan(-y_in / z_in)*MATH_TRANS;
    }

    if (phi > 0) phi = phi - 180;

    g_theta_3 = asin(right_all / sqrt_z_y)*MATH_TRANS - phi;
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

    x_in = (-x / cos(g_theta_1 / MATH_TRANS) - MATH_L2) / MATH_L3;

    if (z_in == 0){ phi = 90; }

    else{ phi = atan(-x_in / z_in)*MATH_TRANS; }

    if (phi > 0) {phi = phi - 180;}  

    sqrt_z_x = sqrt(z_in*z_in + x_in*x_in);

    right_all_2 = -1 * (z_in*z_in + x_in*x_in + MATH_L43*MATH_L43 - 1) / (2 * MATH_L43);
    g_theta_3 = asin(right_all_2 / sqrt_z_x)*MATH_TRANS;
    g_theta_3 = g_theta_3 - phi;

    if (g_theta_3 <0 ) {
      g_theta_3 = 0;
    }

    // Calculate value of theta 2
    g_theta_2 = asin(z_in + MATH_L43*sin(abs(g_theta_3 / MATH_TRANS)))*MATH_TRANS;
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


/* moveTo Only y & z */
// void uArmClass::moveTo(double y,double z)
// {
//   double origin_l = length;
//   double origin_h = height;

//   double cal_length = length;
//   double cal_height = height;

//   length = length - MATH_L2;
//   height = height - MATH_L1;
//   double l_a = length/MATH_L3;
//   double l_b = height/MATH_L3;
//   double l_c = MATH_L43;
//   double l_d = (1-l_a*l_a-l_b*l_b-l_c*l_c)/(2*l_c);
//   double l_e = l_d/(sqrt(l_a*l_a+l_b*l_b));

//   double l_phi = atan(l_a/l_b);

//   // cal local theta_3
//   double l_theta_3 = asin(l_e)+l_phi;  // cal theta_2
  
//   l_theta_3 = (l_theta_3 < -MATH_PI/2 ||l_theta_3 == -MATH_PI/2) ? l_theta_3+MATH_PI:l_theta_3; // set the range correctly
//   l_theta_3 = round(l_theta_3 * MATH_TRANS) == -90 ? MATH_PI/2 : l_theta_3;  // set special case

//   // cal local theta_2
//   double l_theta_2 = asin(l_b+sin(l_theta_3)*l_c);

//   calStretch(l_theta_2,l_theta_3,cal_length,cal_height);

//   l_theta_2 = l_theta_2*MATH_TRANS;
//   l_theta_3 = l_theta_3*MATH_TRANS;

//   if((abs(cal_length-length)>0.1)||(abs(cal_height-height)>0.1)||isnan(l_theta_2)||isinf(l_theta_2)||isnan(l_theta_3)||isinf(l_theta_3))
//   {
//     l_theta_2 = readAngle(SERVO_LEFT_NUM);
//     l_theta_3 = readAngle(SERVO_RIGHT_NUM);
//   }

//   l_theta_3= ((l_theta_3)<0) ? readAngle(SERVO_RIGHT_NUM):l_theta_3;

//   writeServoAngle(SERVO_LEFT_NUM,l_theta_2,1);
//   writeServoAngle(SERVO_RIGHT_NUM,l_theta_3,1);

// }

void uArmClass::writeStretch(double armStretch, double armHeight){
  int offsetL = g_offset_old_servo_left;
  int offsetR = g_offset_old_servo_right;
  armStretch = constrain(armStretch, ARM_STRETCH_MIN, ARM_STRETCH_MAX) + 68;
  armHeight  = constrain(armHeight, ARM_HEIGHT_MIN, ARM_HEIGHT_MAX);
  double xx = armStretch*armStretch + armHeight*armHeight;
  double xxx = ARM_B2 - ARM_A2 + xx;
  double angleB = acos((armStretch*xxx+armHeight*sqrt(4.0*ARM_B2*xx-xxx*xxx))/(xx*2.0*ARM_B))* RAD_TO_DEG;
  double yyy = ARM_A2-ARM_B2+xx;
  double angleA =acos((armStretch*yyy-armHeight*sqrt(4.0*ARM_A2*xx-yyy*yyy))/(xx*2.0*ARM_A))* RAD_TO_DEG;
  int angleR =(int)(angleB + offsetR - 4);//int angleR =angleB + 40 + offsetR;
  int angleL =(int)(angleA + offsetL + 16);//int angleL =25 + angleA + offsetL; 
  angleL = constrain(angleL, 5 + offsetL, 145 + offsetL);
  // writeServoAngle(SERVO_LEFT_NUM, angleL,1);
  // writeServoAngle(SERVO_RIGHT_NUM,angleR,1);
  writeLeftRightServoAngle(angleL,angleR,1);
}


void uArmClass::calStretch(double theta_2, double theta_3, double & l_length_get, double & l_height_get)
{
  l_length_get = cos(theta_2)*MATH_L3 + cos(theta_3)*MATH_L4 ;
  l_height_get = sin(theta_2)*MATH_L3 - sin(theta_3)*MATH_L4 ;
}

void uArmClass::calXYZ(double theta_1, double theta_2, double theta_3)
{

  double l5 = (MATH_L2 + MATH_L3*cos(theta_2 / MATH_TRANS) + MATH_L4*cos(theta_3 / MATH_TRANS));

  g_cal_x = -cos(abs(theta_1 / MATH_TRANS))*l5;
  g_cal_y = -sin(abs(theta_1 / MATH_TRANS))*l5;
  g_cal_z = MATH_L1 + MATH_L3*sin(abs(theta_2 / MATH_TRANS)) - MATH_L4*sin(abs(theta_3 / MATH_TRANS));
}

void uArmClass::calXYZ()
{
  calXYZ(
  uarm.readToAngle(analogRead(SERVO_ROT_ANALOG_PIN),SERVO_ROT_NUM,ABSOLUTE),
  uarm.readToAngle(analogRead(SERVO_LEFT_ANALOG_PIN),SERVO_LEFT_NUM,ABSOLUTE),
  uarm.readToAngle(analogRead(SERVO_RIGHT_ANALOG_PIN),SERVO_RIGHT_NUM,ABSOLUTE));
}

void uArmClass::interpolate(double start_val, double end_val, double (&interp_vals)[INTERP_INTVLS], byte ease_type) {
  double delta = end_val - start_val;
  for (byte f = 0; f < INTERP_INTVLS; f++) {
    switch (ease_type) {
      case INTERP_LINEAR :
        interp_vals[f] = delta * f / INTERP_INTVLS + start_val;
        break;
      case INTERP_EASE_INOUT :
        {
          float t = f / (INTERP_INTVLS / 2.0);
          if (t < 1) {
            interp_vals[f] = delta / 2 * t * t + start_val;
          } else {
            t--;
            interp_vals[f] = -delta / 2 * (t * (t - 2) - 1) + start_val;
          }
        }
        break;
      case INTERP_EASE_IN :
        {
          float t = (float)f / INTERP_INTVLS;
          interp_vals[f] = delta * t * t + start_val;
        }
        break;
      case INTERP_EASE_OUT :
        {
          float t = (float)f / INTERP_INTVLS;
          interp_vals[f] = -delta * t * (t - 2) + start_val;
        }
        break;
      case INTERP_EASE_INOUT_CUBIC :  // this is a compact version of Joey's original cubic ease-in/out
        {
          float t = (float)f / INTERP_INTVLS;
          interp_vals[f] = start_val + (3 * delta) * (t * t) + (-2 * delta) * (t * t * t);
        }
        break;
    }
  }
}

void uArmClass::moveToOpts(double x, double y, double z, double hand_angle, byte relative_flags, double time, byte path_type, byte ease_type) {

  attachAll();

  // find current position using cached servo values
  double current_x;
  double current_y;
  double current_z;
  getCalXYZ(cur_rot, cur_left, cur_right, current_x, current_y, current_z);

  // deal with relative xyz positioning
  byte posn_relative = (relative_flags & F_POSN_RELATIVE) ? 1 : 0;
  x = current_x * posn_relative + x;
  y = current_y * posn_relative + y;
  z = current_z * posn_relative + z;

  // find target angles
  double tgt_rot;
  double tgt_left;
  double tgt_right;
  uarm.getCalAngles(x, y, z, tgt_rot, tgt_left, tgt_right);

  // deal with relative hand orientation
  if (relative_flags & F_HAND_RELATIVE) {
    hand_angle += cur_hand;                                     // rotates a relative amount, ignoring base rotation
  } else if (relative_flags & F_HAND_ROT_REL) {
    hand_angle = hand_angle + cur_hand + (tgt_rot - cur_rot);   // rotates relative to base servo, 0 value keeps an object aligned through movement
  }
  
  if (time > 0) {
    if (path_type == PATH_ANGLES) {
      // we will calculate angle value targets
      double rot_array[INTERP_INTVLS];
      double left_array[INTERP_INTVLS];
      double right_array[INTERP_INTVLS];
      double hand_array[INTERP_INTVLS];

      interpolate(cur_rot, tgt_rot, rot_array, ease_type);
      interpolate(cur_left, tgt_left, left_array, ease_type);
      interpolate(cur_right, tgt_right, right_array, ease_type);
      interpolate(cur_hand, hand_angle, hand_array, ease_type);

      for (byte i = 0; i < INTERP_INTVLS; i++)
      {
        writeAngle(rot_array[i], left_array[i], right_array[i], hand_array[i]);
        delay(time * 1000 / INTERP_INTVLS);
      }
    } else if (path_type == PATH_LINEAR) {
      // we will calculate linear path targets
      double x_array[INTERP_INTVLS];
      double y_array[INTERP_INTVLS];
      double z_array[INTERP_INTVLS];
      double hand_array[INTERP_INTVLS];

      interpolate(current_x, x, x_array, ease_type);
      interpolate(current_y, y, y_array, ease_type);
      interpolate(current_z, z, z_array, ease_type);
      interpolate(cur_hand, hand_angle, hand_array, ease_type);

      for (byte i = 0; i < INTERP_INTVLS; i++)
      {
        double rot, left, right;
        getCalAngles(x_array[i], y_array[i], z_array[i], rot, left, right);
        writeAngle(rot, left, right, hand_array[i]);
        delay(time * 1000 / INTERP_INTVLS);
      }
    }
  }

  // set final target position at end of interpolation or "atOnce"
  writeAngle(tgt_rot, tgt_left, tgt_right, hand_angle);
}

double uArmClass::calYonly(double theta_1, double theta_2, double theta_3)
{
    //g_l3_1_2 = MATH_L3 * cos(theta_2 / MATH_TRANS);
    //g_l4_1_2 = MATH_L4*cos(theta_3 / MATH_TRANS);
    double l5_2 = (MATH_L2 + MATH_L3*cos(theta_2 / MATH_TRANS) + MATH_L4*cos(theta_3 / MATH_TRANS));

    return -sin(abs(theta_1 / MATH_TRANS))*l5_2;
}

void uArmClass::gripperCatch()
{
  g_servo_hand.attach(SERVO_HAND);
  g_servo_hand.write(HAND_ANGLE_CLOSE);
  digitalWrite(VALVE_EN, LOW); // valve disable
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
    digitalWrite(PUMP_EN, LOW);   // pump disable
    g_gripper_reset= false;
  }
}

/* Action Control */
void uArmClass::pumpOn()
{

   pinMode(PUMP_EN, OUTPUT);
   pinMode(VALVE_EN, OUTPUT);
   digitalWrite(VALVE_EN, LOW);
   digitalWrite(PUMP_EN, HIGH);
}

void uArmClass::pumpOff()
{
   pinMode(PUMP_EN, OUTPUT);
   pinMode(VALVE_EN, OUTPUT);
   digitalWrite(VALVE_EN, HIGH);
   digitalWrite(PUMP_EN, LOW);
   delay(50);
   digitalWrite(VALVE_EN,LOW);
}
