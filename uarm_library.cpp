/*!
   \file uarm_library.cpp
   \brief uArm Library for Arduino
   \author Joe Song
   \update Alex Tan, Dave Corboy
   \license GNU
   \copyright(c) 2016 UFactory Team. All right reserved
 */
#include "uarm_library.h"

uArmClass uarm;

uArmClass::uArmClass()
{

}


/*!
   \brief Use BUZZER for Alert
   \param times Beep Times
   \param runTime How Long from High to Low
   \param stopTime How Long from Low to High
 */
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

/*!
   \brief get the calibration data from the external eeprom
   \param rot the calibration data of rotation
   \param left the calibration data of left
   \param right the calibration data of right
 */

void uArmClass::read_servo_calibration_data(double *rot, double *left, double *right)// takes 1~2ms
{
  unsigned char calibration_data[DATA_LENGTH]; //get the calibration data around the data input
  unsigned int min_data_calibration_address;
//rot calibration data
  min_data_calibration_address = (((unsigned int)(*rot)  - (DATA_LENGTH >> 2)) * 2);
  iic_readbuf(calibration_data, EXTERNAL_EEPROM_DEVICE_ADDRESS, ROT_SERVO_ADDRESS + min_data_calibration_address, DATA_LENGTH);
  unsigned int deltaA = 0xffff, deltaB = 0, i, i_min = 0;
  for(i=0;i<(DATA_LENGTH >> 1);i++)
  {
      deltaB = abs ((calibration_data[i+i]<<8) + calibration_data[1+(i+i)] - (*rot) * 10);
      if(deltaA > deltaB)
      {
        i_min = i;
        deltaA = deltaB;
      } 
  }
  *rot = ((calibration_data[i_min+i_min]<<8) + calibration_data[1+(i_min+i_min)])/10.0;
//left calibration data
  deltaA = 0xffff;
  deltaB = 0;
  min_data_calibration_address = (((unsigned int)(*left) - (DATA_LENGTH >> 2)) * 2);
  iic_readbuf(calibration_data, EXTERNAL_EEPROM_DEVICE_ADDRESS, LEFT_SERVO_ADDRESS + min_data_calibration_address, DATA_LENGTH);
  for(i=0;i<(DATA_LENGTH >> 1);i++)
  {
      deltaB = abs ((calibration_data[i+i]<<8) + calibration_data[1+(i+i)] - (*left) * 10);
      if(deltaA > deltaB)
      {
        i_min = i;
        deltaA = deltaB;
      } 
  }
  *left = ((calibration_data[i_min+i_min]<<8) + calibration_data[1+(i_min+i_min)])/10.0;
//right calibration data
  deltaA = 0xffff;
  deltaB = 0;
  min_data_calibration_address = (((unsigned int)(*right) - (DATA_LENGTH >> 2)) * 2);
  iic_readbuf(calibration_data, EXTERNAL_EEPROM_DEVICE_ADDRESS, RIGHT_SERVO_ADDRESS + min_data_calibration_address, DATA_LENGTH);
  for(i=0;i<(DATA_LENGTH >> 1);i++)
  {
      deltaB = abs ((calibration_data[i+i]<<8) + calibration_data[1+(i+i)] - (*right) * 10);
      if(deltaA > deltaB)
      {
        i_min = i;
        deltaA = deltaB;
      } 
  }
  *right = ((calibration_data[i_min+i_min]<<8) + calibration_data[1+(i_min+i_min)])/10.0;
} 

/*!
   \brief Write 4 Servo Angles, servo_rot, servo_left, servo_right, servo_hand_rot
   \param servo_rot_angle SERVO_ROT_NUM
   \param servo_left_angle SERVO_LEFT_NUM
   \param servo_right_angle SERVO_RIGHT_NUM
   \param servo_hand_rot_angle SERVO_HAND_ROT_NUM
   \return SUCCESS, FAILED
 */
int uArmClass::write_servos_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle, double servo_hand_rot_angle)
{
        attach_all();
        write_servos_angle(servo_rot_angle, servo_left_angle, servo_right_angle);
        write_servo_angle(SERVO_HAND_ROT_NUM,servo_hand_rot_angle,true);
        cur_hand = servo_hand_rot_angle;
}

/*!
   \brief Write 3 Servo Angles, servo_rot, servo_left, servo_right
   \param servo_rot_angle SERVO_ROT_NUM
   \param servo_left_angle SERVO_LEFT_NUM
   \param servo_right_angle SERVO_RIGHT_NUM
   \return SUCCESS, FAILED
 */
int uArmClass::write_servos_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle)
{
        /*attach_servo(SERVO_ROT_NUM);
        attach_servo(SERVO_LEFT_NUM);
        attach_servo(SERVO_RIGHT_NUM);*/

        /*if(servo_left_angle < 10) servo_left_angle = 10;
        if(servo_left_angle > 120) servo_left_angle = 120;
        if(servo_right_angle < 10) servo_right_angle = 10;
        if(servo_right_angle > 110) servo_right_angle = 110;

        if(servo_left_angle + servo_right_angle > 160)
        {
                servo_right_angle = 160 - servo_left_angle;
                return FAILED;
        }*/
        write_servo_angle(SERVO_ROT_NUM,servo_rot_angle,true);
        write_servo_angle(SERVO_LEFT_NUM,servo_left_angle,true);
        write_servo_angle(SERVO_RIGHT_NUM,servo_right_angle,true);

        // refresh logical servo angle cache
        cur_rot = servo_rot_angle;
        cur_left = servo_left_angle;
        cur_right = servo_right_angle;
}

/*!
   \brief Write the angle to Servo
   \param servo_number SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \param servo_angle Servo target angle, 0.00 - 180.00
   \param writeWithoffset True: with Offset, False: without Offset
 */
void uArmClass::write_servo_angle(byte servo_number, double servo_angle, boolean writeWithoffset)
{
        attach_servo(servo_number);
        //servo_angle = writeWithoffset ? (servo_angle + read_servo_offset(servo_number)) : servo_angle;
        // = constrain(servo_angle,0.0,180.0);
        switch(servo_number)
        {
        case SERVO_ROT_NUM:       g_servo_rot.write(servo_angle);
                cur_rot = servo_angle;
                break;
        case SERVO_LEFT_NUM:      g_servo_left.write(servo_angle);
                cur_left = servo_angle;
                break;
        case SERVO_RIGHT_NUM:     g_servo_right.write(servo_angle);
                cur_right = servo_angle;
                break;
        case SERVO_HAND_ROT_NUM:  g_servo_hand_rot.write(servo_angle);
                cur_hand = servo_angle;
                break;
        default:                  break;
        }
}

/*!
   \brief Write the left Servo & Right Servo in the same time (Avoid demage the Servo)
   \param servo_left_angle left servo target angle
   \param servo_right_angle right servo target angle
   \param writeWithoffset True: with Offset, False: without Offset
 */
void uArmClass::write_left_right_servo_angle(double servo_left_angle, double servo_right_angle, boolean writeWithoffset)
{
        servo_left_angle = constrain(servo_left_angle,0,150);
        servo_right_angle = constrain(servo_right_angle,0,120);
        servo_left_angle = writeWithoffset ? (servo_left_angle + read_servo_offset(SERVO_LEFT_NUM)) : servo_left_angle;
        servo_right_angle = writeWithoffset ? (servo_right_angle + read_servo_offset(SERVO_RIGHT_NUM)) : servo_right_angle;
        if(servo_left_angle + servo_right_angle > 180) // if left angle & right angle exceed 180 degree, it might be caused damage
        {
                alert(1, 10, 0);
                return;
        }
        attach_servo(SERVO_LEFT_NUM);
        attach_servo(SERVO_RIGHT_NUM);
        g_servo_left.write(servo_left_angle);
        g_servo_right.write(servo_right_angle);
}

/*!
   \brief Attach All Servo
   \note Warning, if you attach left servo & right servo without a movement, it might be caused a demage
 */
void uArmClass::attach_all()
{
        attach_servo(SERVO_ROT_NUM);
        attach_servo(SERVO_LEFT_NUM);
        attach_servo(SERVO_RIGHT_NUM);
        attach_servo(SERVO_HAND_ROT_NUM);
}

/*!
   \brief Attach Servo, if servo has not been attached, attach the servo, and read the current Angle
   \param servo number SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
 */
void uArmClass::attach_servo(byte servo_number)
{
        switch(servo_number) {
        case SERVO_ROT_NUM:
                if(!g_servo_rot.attached()) {
                        g_servo_rot.attach(SERVO_ROT_PIN);
                        cur_rot = read_servo_angle(SERVO_ROT_NUM);
                }
                break;
        case SERVO_LEFT_NUM:
                if (!g_servo_left.attached()) {
                        g_servo_left.attach(SERVO_LEFT_PIN);
                        cur_left = read_servo_angle(SERVO_LEFT_NUM);
                }
                break;
        case SERVO_RIGHT_NUM:
                if (!g_servo_right.attached()) {
                        g_servo_right.attach(SERVO_RIGHT_PIN);
                        cur_right = read_servo_angle(SERVO_RIGHT_NUM);
                }
                break;
        case SERVO_HAND_ROT_NUM:
                if (!g_servo_hand_rot.attached()) {
                        g_servo_hand_rot.attach(SERVO_HAND_PIN);
                        cur_hand = read_servo_angle(SERVO_HAND_ROT_NUM);
                }
                break;
        }
}

/*!
   \brief Detach All servo, you could move the arm
 */
void uArmClass::detach_all_servos()
{
        g_servo_rot.detach();
        g_servo_left.detach();
        g_servo_right.detach();
        g_servo_hand_rot.detach();
}

/*!
   \brief Detach Servo by Servo Number, SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \param servo_number SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
 */
void uArmClass::detach_servo(byte servo_number)
{
        switch(servo_number) {
        case SERVO_ROT_NUM:
                g_servo_rot.detach();
                break;
        case SERVO_LEFT_NUM:
                g_servo_left.detach();
                break;
        case SERVO_RIGHT_NUM:
                g_servo_right.detach();
                break;
        case SERVO_HAND_ROT_NUM:
                g_servo_hand_rot.detach();
                break;
        }
}

/*!
   \brief Read Servo Offset from EEPROM. From OFFSET_START_ADDRESS, each offset occupy 4 bytes in rom
   \param servo_num SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \return Return servo offset
 */
double uArmClass::read_servo_offset(byte servo_num)
{
        double manual_servo_offset = 0.0f;
        EEPROM.get(MANUAL_OFFSET_ADDRESS + servo_num * sizeof(manual_servo_offset), manual_servo_offset);
        return manual_servo_offset;
}

/*!
   \brief Convert the Analog to Servo Angle, by this formatter, angle = intercept + slope * analog
   \param input_analog Analog Value
   \param servo_num SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \param withOffset true, false
   \return Return Servo Angle
 */
double uArmClass::analog_to_angle(int input_analog, byte servo_num, boolean withOffset)
{
        double intercept = 0.0f;
        double slope = 0.0f;
        //read_linear_offset(servo_num, intercept, slope);
        double angle = intercept + slope*input_analog;
        return withOffset ? angle + read_servo_offset(servo_num) : angle;
}

/*!
   \brief read Angle by servo_num without offset
   \param servo_num SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \return Return servo_num Angle
 */
double uArmClass::read_servo_angle(byte servo_num)
{
        return read_servo_angle(servo_num, false);
}

/*!
   \brief read Angle by servo_num
   \param servo_num SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \param withOffset true, false
   \return Return servo_num Angle
 */
double uArmClass::read_servo_angle(byte servo_num, boolean withOffset)
{
        double angle = 0;
        for (byte i = 0; i < 5; i++){
            switch (servo_num)
            {
            case SERVO_ROT_NUM:
                    angle += analog_to_angle(analogRead(SERVO_ROT_ANALOG_PIN),SERVO_ROT_NUM,withOffset);
                    break;

            case SERVO_LEFT_NUM:
                    angle += analog_to_angle(analogRead(SERVO_LEFT_ANALOG_PIN),SERVO_LEFT_NUM,withOffset);
                    break;

            case SERVO_RIGHT_NUM:
                    angle += analog_to_angle(analogRead(SERVO_RIGHT_ANALOG_PIN),SERVO_RIGHT_NUM,withOffset);
                    break;

            case SERVO_HAND_ROT_NUM:
                    angle += analog_to_angle(analogRead(SERVO_HAND_ROT_ANALOG_PIN),SERVO_HAND_ROT_NUM,withOffset);
                    break;

            default:
                    break;

            }
            delay(10);
        }
        return angle/5;
}

/** Calculate the angles from given coordinate x, y, z to theta_1, theta_2, theta_3
**/
/*!
   \brief Calculate the angles from given coordinate x, y, z to theta_1, theta_2, theta_3
   \param x X axis
   \param y Y axis
   \param z Z axis
   \param theta_1 SERVO_ROT_NUM servo angles
   \param theta_2 SERVO_LEFT_NUM servo angles
   \param theta_3 SERVO_RIGHT_NUM servo angles
 */
unsigned char uArmClass::coordinate_to_angle(double x, double y, double z, double& theta_1, double& theta_2, double& theta_3)//theta_1:rotation angle   theta_2:the angle of lower arm and horizon   theta_3:the angle of upper arm and horizon
{
  double x_in = 0.0;
  double z_in = 0.0;
  double right_all = 0.0;
  double sqrt_z_x = 0.0;
  double phi = 0.0;
  double theta_triangle = 0.0;
  z_in = (z - MATH_L1) / MATH_L3;

  if(y<0)
  {
  #ifdef DEBUG_MODE
    Serial.write("ERR0:coordinate_to_angle");
  #endif
    return OUT_OF_RANGE;
  }       
  // Calculate value of theta 1
  if(x==0)
  {
    theta_1 = 90;
  }
  else
  {
    //theta_1 = atan(y / x)*MATH_TRANS;

    if (x > 0)
    {
      theta_1 = atan(y / x)*MATH_TRANS;//angle tranfer 0-180 CCW
      //theta_1 = 180 - atan(y / x)*MATH_TRANS;//angle tranfer 0-180 CW
    }
    if (x < 0) 
    {
      theta_1 = 180 + atan(y / x)*MATH_TRANS;//angle tranfer  0-180 CCW
      //theta_1 = atan( (- y) / x)*MATH_TRANS;//angle tranfer  0-180 CW
    }

  }
                
  // Calculate value of theta 3
  if(theta_1!=90)
  {
    x_in = (x / cos(theta_1 / MATH_TRANS) - MATH_L2) / MATH_L3;
  }
  else
  {
    x_in = (y - MATH_L2) / MATH_L3;
  }


  phi = atan(z_in / x_in)*MATH_TRANS;//phi is the angle of line (from joint 2 to joint 4) with the horizon

  sqrt_z_x = sqrt(z_in*z_in + x_in*x_in);

  right_all = (sqrt_z_x*sqrt_z_x + MATH_L43*MATH_L43 - 1) / (2 * MATH_L43 * sqrt_z_x);//cosin law
  theta_3 = acos(right_all)*MATH_TRANS;//cosin law

  // Calculate value of theta 2
  right_all = (sqrt_z_x*sqrt_z_x + 1 - MATH_L43*MATH_L43) / (2 * sqrt_z_x);//cosin law
  theta_2 = acos(right_all)*MATH_TRANS;//cosin law

  //right_all = (MATH_L43*MATH_L43 + 1 - sqrt_z_x*sqrt_z_x) / (2 * MATH_L43);//cosin law
  //theta_triangle = acos(right_all)*MATH_TRANS;//used to detect the if theta_2>90 or not

  theta_2 = theta_2 + phi;
  theta_3 = theta_3 - phi;
  #ifdef DEBUG_MODE
  Serial.println("angle of servos(phi,theta_1,theta_2,theta_3)\n");
  Serial.println(phi,DEC);
  Serial.println(theta_1,DEC);
  Serial.println(theta_2,DEC);
  Serial.println(theta_3,DEC);
  #endif
  //determine if the angle can be reached
  if(x_in<=0)
  {
  #ifdef DEBUG_MODE
    Serial.write("ERR1:coordinate_to_angle");
  #endif
    return OUT_OF_RANGE;
  }
  if(((theta_2 - LEFT_SERVO_OFFSET) < L3_MIN_ANGLE)||((theta_2 - LEFT_SERVO_OFFSET) > L3_MAX_ANGLE))
  {
  #ifdef DEBUG_MODE
    Serial.write("ERR2:coordinate_to_angle");
  #endif
    return OUT_OF_RANGE;
  }
  if(((theta_3 - RIGHT_SERVO_OFFSET) < L4_MIN_ANGLE)||((theta_3 - RIGHT_SERVO_OFFSET) > L4_MAX_ANGLE))
  {
   #ifdef DEBUG_MODE
    Serial.write("ERR3:coordinate_to_angle");
  #endif
    return OUT_OF_RANGE;
  }
  if(((180 - theta_3 - theta_2)>L4L3_MAX_ANGLE)||((180 - theta_3 - theta_2)<L4L3_MIN_ANGLE))
  {
  #ifdef DEBUG_MODE
    Serial.write("ERR4:coordinate_to_angle");
  #endif
    return OUT_OF_RANGE;
  }
  /*if ((180 - theta_2 + theta_3)<L4L3_MIN_ANGLE)
  {
  #ifdef DEBUG_MODE
    Serial.write("ERR5:coordinate_to_angle");
  #endif
    return OUT_OF_RANGE;
  }*/


  return IN_RANGE;
}

/*!
   \brief Write Sretch & Height.
   \description This is an old control method to uArm. Using uarm's Stretch and height, , Height from -180 to 150
   \param armStretch Stretch from 0 to 195
   \param armHeight Height from -150 to 150
 */
void uArmClass::write_stretch_height(double armStretch, double armHeight){
/*        if(EEPROM.read(CALIBRATION_STRETCH_FLAG) != CONFIRM_FLAG) {
                alert(3, 200, 200);
                return;
        }
        double offsetL = 0;
        double offsetR = 0;

        EEPROM.get(OFFSET_STRETCH_START_ADDRESS, offsetL);
        EEPROM.get(OFFSET_STRETCH_START_ADDRESS + 4, offsetR);
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
        write_left_right_servo_angle(angleL,angleR,true);*/
}

/*!
   \brief Calculate X,Y,Z to g_current_x,g_current_y,g_current_z
 */
void uArmClass::get_current_xyz()
{
        double theta_1 = uarm.analog_to_angle(analogRead(SERVO_ROT_ANALOG_PIN),SERVO_ROT_NUM,false);
        double theta_2 = uarm.analog_to_angle(analogRead(SERVO_LEFT_ANALOG_PIN),SERVO_LEFT_NUM,false);
        double theta_3 = uarm.analog_to_angle(analogRead(SERVO_RIGHT_ANALOG_PIN),SERVO_RIGHT_NUM,false);
        get_current_xyz(theta_1, theta_2, theta_3);
}

/*!
   \brief Calculate X,Y,Z to g_current_x,g_current_y,g_current_z
 */
void uArmClass::get_current_xyz(double theta_1, double theta_2, double theta_3)
{
        double l5 = (MATH_L2 + MATH_L3*cos(theta_2 / MATH_TRANS) + MATH_L4*cos(theta_3 / MATH_TRANS));

        g_current_x = -cos(abs(theta_1 / MATH_TRANS))*l5;
        g_current_y = -sin(abs(theta_1 / MATH_TRANS))*l5;
        g_current_z = MATH_L1 + MATH_L3*sin(abs(theta_2 / MATH_TRANS)) - MATH_L4*sin(abs(theta_3 / MATH_TRANS));
}

/*!
   \brief "Genernate the position array"
   \param start_val Start Position
   \param end_val End Position
   \param interp_vals interpolation array
   \param ease_type INTERP_EASE_INOUT_CUBIC, INTERP_LINEAR, INTERP_EASE_INOUT, INTERP_EASE_IN, INTERP_EASE_OUT
*/
void uArmClass::interpolate(double start_val, double end_val, double *interp_vals, byte ease_type) {
        if(ease_type == INTERP_EASE_INOUT_CUBIC)// make sure all the data are still in range
        {
          start_val = start_val/10.0;
          end_val = end_val/10.0;
        }
        double delta = end_val - start_val;
        for (byte f = 0; f < INTERP_INTVLS; f++) {
                switch (ease_type) {
                case INTERP_LINEAR://linear moving
                        *(interp_vals+f) = delta * f / INTERP_INTVLS + start_val;
                        break;
                case INTERP_EASE_INOUT://
                {
                        float t = f / (INTERP_INTVLS / 2.0);
                        if (t < 1) {
                                *(interp_vals+f) = delta / 2 * t * t + start_val;
                        } else {
                                t--;
                                *(interp_vals+f)= -delta / 2 * (t * (t - 2) - 1) + start_val;
                        }
                }
                break;
                /*case INTERP_EASE_IN:
                {
                        float t = (float)f / INTERP_INTVLS;
                        *(interp_vals+f) = delta * t * t + start_val;
                }
                break;
                case INTERP_EASE_OUT:
                {
                        float t = (float)f / INTERP_INTVLS;
                        *(interp_vals+f) = -delta * t * (t - 2) + start_val;
                }
                break;*/
                case INTERP_EASE_INOUT_CUBIC: // this is a compact version of Joey's original cubic ease-in/out
                {
                        float t = (float)f / INTERP_INTVLS;
                        //*(interp_vals+f) = 10.0*(start_val + (3 * delta) * (t * t) + (-2 * delta) * (t * t * t));
                        *(interp_vals+f) = 10.0 * (start_val + t* t * delta * (3 + (-2) * t));
                }
                break;
                }
        }
}

/*!
   \brief Move To, Action Control Core Function
   \param x X Axis Value
   \param y Y Axis Value
   \param z Z Axis Value
   \param hand_angle Hand Axis
   \param relative_flags ABSOLUTE, RELATIVE
   \param enable_hand Enable Hand Axis
*/
  double current_x=0;
  double current_y=200;
  double current_z=100;   
unsigned char uArmClass::move_to(double x, double y, double z, double hand_angle, byte relative_flags, double time, byte ease_type, boolean enable_hand) {
  // find current position using cached servo values

  //angle_to_coordinate(cur_rot, cur_left, cur_right, current_x, current_y, current_z);

  // deal with relative xyz positioning
  if(relative_flags == RELATIVE)
  {
    x = current_x + x;
    y = current_y + y;
    z = current_z + z;
    //hand_angle = current_hand + hand_angle;
  }

  // find target angles
  double tgt_rot;
  double tgt_left;
  double tgt_right;
  //  detect if the xyz coordinate are in the range
  if(coordinate_to_angle(x, y, z, tgt_rot, tgt_left, tgt_right) == OUT_OF_RANGE)
  {
    return OUT_OF_RANGE;
  }
Serial.println(tgt_rot,DEC);
Serial.println(tgt_left,DEC);
Serial.println(tgt_right,DEC);
  //calculate the length and use the longest to determine the numbers of interpolation
  unsigned int delta_rot=abs(tgt_rot-cur_rot);
  unsigned int delta_left=abs(tgt_left-cur_left);
  unsigned int delta_right=abs(tgt_right-cur_right);

  INTERP_INTVLS = max(delta_rot,delta_left);
  INTERP_INTVLS = max(INTERP_INTVLS,delta_right);

  INTERP_INTVLS = (INTERP_INTVLS<80) ? INTERP_INTVLS : 80;
  INTERP_INTVLS = INTERP_INTVLS * time;// speed determine the number of interpolation
  //INTERP_INTVLS = 0;

  if (time > 0) 
  {
    // we will calculate linear path targets
    double x_array[INTERP_INTVLS];
    double y_array[INTERP_INTVLS];
    double z_array[INTERP_INTVLS];
    double hand_array[INTERP_INTVLS];

    interpolate(current_x, x, x_array, ease_type);// /10 means to make sure the t*t*t is still in the range
    interpolate(current_y, y, y_array, ease_type);
    interpolate(current_z, z, z_array, ease_type);
    interpolate(cur_hand, hand_angle, hand_array, ease_type);

    for (byte i = 0; i < INTERP_INTVLS; i++)
    {
      double rot, left, right;
      if(coordinate_to_angle(x_array[i], y_array[i], z_array[i], rot, left, right) == OUT_OF_RANGE)
      {
        return OUT_OF_RANGE;
      }

      left = left - LEFT_SERVO_OFFSET;//assembling offset
      right = right - RIGHT_SERVO_OFFSET;//assembling offset
      read_servo_calibration_data(&rot,&left,&right);

      //if(enable_hand)
      write_servos_angle(rot, left, right, hand_array[i]);
      //else
      //  write_servos_angle(rot, left, right);
      delay(time * 1000 / INTERP_INTVLS);
    }
  }

      
// the destination
#ifdef DEBUG_MODE
Serial.println(tgt_rot,DEC);
Serial.println(tgt_left,DEC);
Serial.println(tgt_right,DEC); 
#endif 
  tgt_left = tgt_left - LEFT_SERVO_OFFSET;//assembling offset
  tgt_right = tgt_right - RIGHT_SERVO_OFFSET;//assembling offset    
  read_servo_calibration_data(&tgt_rot,&tgt_left,&tgt_right);

#ifdef DEBUG_MODE
Serial.println(tgt_rot,DEC);
Serial.println(tgt_left,DEC);
Serial.println(tgt_right,DEC);
#endif
    // set final target position at end of interpolation or atOnce
    //if (enable_hand)
  write_servos_angle(tgt_rot, tgt_left, tgt_right, hand_angle);
    //else
    //  write_servo_angle(tgt_rot, tgt_left, tgt_right);

  current_x=x;
  current_y=y;
  current_z=z;

  return IN_RANGE;
}

/*!
   \brief Calculate Y
   \param theta_1
   \param theta_2
   \param theta_3
   \return Y Axis Value
*/
double uArmClass::angle_to_coordinate_y(double theta_1, double theta_2, double theta_3)
{
        double l5_2 = (MATH_L2 + MATH_L3*cos(theta_2 / MATH_TRANS) + MATH_L4*cos(theta_3 / MATH_TRANS));

        return -sin(abs(theta_1 / MATH_TRANS))*l5_2;
}

/*!
   \brief Close Gripper
*/
void uArmClass::gripper_catch()
{
        pinMode(GRIPPER, OUTPUT);
        digitalWrite(GRIPPER, LOW); // gripper catch
        g_gripper_reset = true;
}

/*!
   \brief Release Gripper
*/
void uArmClass::gripper_release()
{
        if(g_gripper_reset)
        {
                pinMode(GRIPPER, OUTPUT);
                digitalWrite(GRIPPER, HIGH); // gripper release
                g_gripper_reset= false;
        }
}

/*!
   \brief Turn on Pump
*/
void uArmClass::pump_on()
{

        pinMode(PUMP_EN, OUTPUT);
        pinMode(VALVE_EN, OUTPUT);
        digitalWrite(VALVE_EN, LOW);
        digitalWrite(PUMP_EN, HIGH);
}

/*!
   \brief Turn off Pump
*/
void uArmClass::pump_off()
{
        pinMode(PUMP_EN, OUTPUT);
        pinMode(VALVE_EN, OUTPUT);
        digitalWrite(VALVE_EN, HIGH);
        digitalWrite(PUMP_EN, LOW);
        delay(50);
        digitalWrite(VALVE_EN,LOW);
}


//*************************************private functions***************************************//
//**just used by the 512k external eeprom**//
void uArmClass::delay_us(){}

void uArmClass::iic_start()
{
  PORTC |= 0x20;//  SCL=1
  delay_us();
  PORTC |= 0x10;//  SDA = 1;
  delay_us();
  PORTC &= 0xEF;//  SDA = 0;
  delay_us();
  PORTC &= 0xDF;//  SCL=0
  delay_us();
}

void uArmClass::iic_stop()
{
  PORTC &= 0xDF;//  SCL=0
  delay_us();
  PORTC &= 0xEF;//  SDA = 0;
  delay_us();
  PORTC |= 0x20;//  SCL=1
  delay_us();
  PORTC |= 0x10;//  SDA = 1;
  delay_us();
}

//return 0:ACK=0
//return 1:NACK=1
unsigned char uArmClass::read_ack()
{
  unsigned char old_state;
  old_state = DDRC;
  DDRC = DDRC & 0xEF;//SDA INPUT
  PORTC |= 0x10;//  SDA = 1;
  delay_us();
  PORTC |= 0x20;//  SCL=1
  delay_us();
  if((PINC&0x10) == 0x10) // if(SDA)
  {
    PORTC &= 0xDF;//  SCL=0
    iic_stop();
    return 1; 
  }
  else
  {
    PORTC &= 0xDF;//  SCL=0
    
    DDRC = old_state;
    
    return 0;
  }
}

//ack=0:send ack
//ack=1:do not send ack
void uArmClass::send_ack()
{
  unsigned char old_state;
  old_state = DDRC;
  DDRC = DDRC | 0x10;//SDA OUTPUT
  PORTC &= 0xEF;//  SDA = 0;  
  delay_us();
  PORTC |= 0x20;//  SCL=1
  delay_us();
  PORTC &= 0xDF;//  SCL=0
  delay_us();
  DDRC = old_state;
  PORTC |= 0x10;//  SDA = 1;
  delay_us();
}

void uArmClass::iic_sendbyte(unsigned char dat)
{
  unsigned char i;
  for(i = 0;i < 8;i++)
  {
    if(dat & 0x80)
      PORTC |= 0x10;//  SDA = 1;
    else
      PORTC &= 0xEF;//  SDA = 0;
    dat <<= 1;
    delay_us();
    PORTC |= 0x20;//  SCL=1
    delay_us();
    PORTC &= 0xDF;//  SCL=0
  }
}

unsigned char uArmClass::iic_receivebyte()
{
  unsigned char i,byte = 0;
  unsigned char old_state;
  old_state = DDRC;
  DDRC = DDRC & 0xEF;//SDA INPUT
  for(i = 0;i < 8;i++)
  {
    PORTC |= 0x20;//  SCL=1
    delay_us();
    byte <<= 1;
    if((PINC&0x10) == 0x10) // if(SDA)
      byte |= 0x01;
    delay_us();
    PORTC &= 0xDF;//  SCL=0
    DDRC = old_state;
    delay_us();
  }
  return byte;
}

unsigned char uArmClass::iic_writebuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len)
{
  DDRC = DDRC | 0x30;
  PORTC = PORTC | 0x30;
  unsigned char length_of_data=0;//page write
  length_of_data = len;
  iic_start();
  iic_sendbyte(device_addr);
  if(read_ack())return 1;
  iic_sendbyte(addr>>8);
  if(read_ack())return 1;
  iic_sendbyte(addr%256);
  if(read_ack())return 1; 
  while(len != 0)
  {
    iic_sendbyte(*(buf + length_of_data - len));
    len--;
    if(read_ack())return 1;
    delay(5);
  }
  iic_stop();
  
  return 0;
}

unsigned char uArmClass::iic_readbuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len)
{
  DDRC = DDRC | 0x30;
  PORTC = PORTC | 0x30;
  unsigned char length_of_data=0;
  length_of_data = len;
  iic_start();
  iic_sendbyte(device_addr);
  if(read_ack())return 1;
  iic_sendbyte(addr>>8);
  if(read_ack())return 1;
  iic_sendbyte(addr%256);
  if(read_ack())return 1;
  iic_start();
  iic_sendbyte(device_addr+1);
  if(read_ack())return 1;

  while(len != 0)
  {
    *(buf + length_of_data - len) = iic_receivebyte();
    //Serial.println(*(buf + length_of_data - len),DEC);
    len--;
    if(len != 0){
      send_ack();
    }
  }
  iic_stop();
  return 0;
}
//*************************************end***************************************//