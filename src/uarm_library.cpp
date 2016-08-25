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

String message = "";
uArmClass::uArmClass()
{

}

/*!
   \brief check the arm status
   \Return true:free; false:busy
 */
bool uArmClass::available()
{
  if(move_times!=255)
  {
    return false;
  }

  return true;
}

/*!
   \brief process the uarm movement
   \no parameter
*/
void uArmClass::arm_process_commands()
{
        //get the uart command
        if(Serial.available())
        {
                message = Serial.readStringUntil(']') + ']';
                runCommand(message); // Run the command and send back the response
        }

        //movement function
        if(move_times!=255)
        {

                //if(move_times <= INTERP_INTVLS)--------------------------------------------------------------
                if((millis() - moveStartTime) >= (move_times * microMoveTime))// detect if it's time to move
                {
#ifdef PRODUCT_MKII
                        y_array[move_times] = y_array[move_times] - LEFT_SERVO_OFFSET; //assembling offset
                        z_array[move_times] = z_array[move_times] - RIGHT_SERVO_OFFSET; //assembling offset
                        x_array[move_times] = x_array[move_times] - ROT_SERVO_OFFSET; //rot offset

                        read_servo_calibration_data(&x_array[move_times], &y_array[move_times], &z_array[move_times]);
                        write_servos_angle(x_array[move_times], y_array[move_times], z_array[move_times]);
#else
                        write_servo_angle(x_array[move_times], y_array[move_times], z_array[move_times]);// let servo run - John Feng
#endif

      //hand rot as hand rot do not have the smooth array
      if(move_times == (INTERP_INTVLS / 4))
      {
		    write_servo_angle(SERVO_HAND_ROT_NUM, cur_hand);
      }
      
      move_times++;
      if(move_times > INTERP_INTVLS)
      {
        move_times = 255;//disable the move
      }
    }
  }

  //buzzer work------------------------------------------------------------------------------------
  if(buzzerStopTime != 0)
  {
    if(millis() >= buzzerStopTime)
    {
      noTone(BUZZER);
      buzzerStopTime = 0;
    }
  }

#ifdef PRODUCT_MKII
        //check the BT status------------------------------------------------------------------------
        digitalWrite(BT_DETEC,HIGH);//
        pinMode(BT_DETEC, INPUT);
        if(digitalRead(BT_DETEC)==HIGH)//do it here
        {
                sys_status = NORMAL_BT_CONNECTED_MODE;
        }
        else
        {
                sys_status = NORMAL_MODE;
        }
        pinMode(BT_DETEC,OUTPUT);
#endif
        //check the button4 status------------------------------------------------------------------------
        if(digitalRead(BTN_D4)==LOW)//check the D4 button
        {
                delay(50);
                //Serial.println("Test: whether get into learning mode");
                if(digitalRead(BTN_D4)==LOW)
                {
                        switch(sys_status)
                        {
                        case NORMAL_MODE:
                        case NORMAL_BT_CONNECTED_MODE:
                                sys_status = LEARNING_MODE;
                                addr = 0;//recording/playing address
                                detach_all_servos();
                                break;
                        case LEARNING_MODE:
                                //LEARNING_MODE_STOP is just used to notificate record() function to stop, once record() get it then change the sys_status to normal_mode
                                sys_status = LEARNING_MODE_STOP;//do not detec if BT is connected here, will do it seperatly
                                break;
                        default: break;
                        }
                }
                while(digitalRead(BTN_D4)==LOW) ; // make sure button is released
        }

        //check the button7 status-------------------------------------------------------------------------
        if(digitalRead(BTN_D7)==LOW)//check the D7 button
        {
                delay(50);
                if(digitalRead(BTN_D7)==LOW)
                {
                        //Serial.println("Test: whether BTN_D7 useful");
                        switch(sys_status)
                        {
                        case NORMAL_MODE:
                        case NORMAL_BT_CONNECTED_MODE:
                                delay(1000);
                                addr = 0;//recording/playing address
                                if(digitalRead(BTN_D7)==LOW)
                                        sys_status = LOOP_PLAY_MODE;
                                else
                                        sys_status = SINGLE_PLAY_MODE;
                                break;
                        case SINGLE_PLAY_MODE:
                        case LOOP_PLAY_MODE:
                                sys_status = NORMAL_MODE;
                                break;
#ifdef PRODUCT_MKII
                        case LEARNING_MODE:
                                if(digitalRead(PUMP_GRI_EN) == HIGH) //detec the status of pump and gri and do the opposite
                                        digitalWrite(PUMP_GRI_EN,LOW);
                                else
                                        digitalWrite(PUMP_GRI_EN,HIGH);
                                break;
#endif
                        }
                }
                while(digitalRead(BTN_D7)==LOW) ; // make sure button is released
        }

  //sys led function detec every 0.05s-----------------------------------------------------------------
  if(time_50ms != millis()%50)
  {
    time_50ms = millis()%50;
    if(time_50ms == 0)
    {
#ifdef PRODUCT_MKII
                        switch(sys_status)
                        {
                        case NORMAL_MODE:
                                if(time_ticks % 40 == 0) digitalWrite(SYS_LED,LOW);
                                else digitalWrite(SYS_LED,HIGH);
                                break;
                        case NORMAL_BT_CONNECTED_MODE:
                                digitalWrite(SYS_LED,LOW);
                                break;
                        case LEARNING_MODE:
                                if(time_ticks % 4 < 2) digitalWrite(SYS_LED,LOW);
                                else digitalWrite(SYS_LED,HIGH);
                                break;
                        case SINGLE_PLAY_MODE:
                        case LOOP_PLAY_MODE:
                                if(time_ticks % 40< 20) digitalWrite(SYS_LED,LOW);
                                else digitalWrite(SYS_LED,HIGH);
                                break;
                        }
                        time_ticks++;
#endif

      //learning&playing mode function****************
      switch(sys_status)//every 0.125s per point
      {
        case SINGLE_PLAY_MODE:
          //Serial.println("Test: whether single play mode");
          if(play() == false)
          {
            sys_status = NORMAL_MODE;
            addr = 0;
          }
          break;
        case LOOP_PLAY_MODE:
          //Serial.println("Test: whether loop play mode"); 
		  if(play() == false)
          {
            //sys_status = LOOP_PLAY_MODE;
            addr = 0;

          }
          break;
        case LEARNING_MODE:
        case LEARNING_MODE_STOP:
          if(record() == false)
          {
          	//Serial.println("Test: record stop");
            sys_status = NORMAL_MODE;
            addr = 0;
            attach_all();
            //Serial.println("Test: whether get out from learning mode");
          }
          break;
        default: break;
      }
      //learning mode function end*******************
    }
  }
//#endif

}

void uArmClass::arm_setup()
{
        pinMode(BTN_D4,INPUT_PULLUP);//special mode for calibration
        pinMode(BUZZER,OUTPUT);
#ifdef PRODUCT_MKII
        if(digitalRead(4)==LOW)
        {
                while(digitalRead(4)==LOW) ;
                write_servo_angle(90,90,0);
                while(1) ;
        }
#endif
        pinMode(LIMIT_SW, INPUT_PULLUP);
        pinMode(BTN_D7, INPUT_PULLUP);
#ifdef PRODUCT_MKII

        pinMode(PUMP_GRI_EN,OUTPUT);
        pinMode(SYS_LED,OUTPUT);
        digitalWrite(PUMP_GRI_EN,HIGH);//keep the pump off
#else
        pinMode(PUMP_EN,OUTPUT);
        pinMode(VALVE_EN,OUTPUT);
        pinMode(GRIPPER,OUTPUT);
#endif
#ifdef PRODUCT_MKII
        unsigned char strings[6];
        //TEST
        strings[0]=(int)(LEFT_SERVO_OFFSET*10)>>8;
        strings[1]=(int)(LEFT_SERVO_OFFSET*10);
        strings[2]=(int)(RIGHT_SERVO_OFFSET*10)>>8;
        strings[3]=(int)(RIGHT_SERVO_OFFSET*10);
        strings[4]=(int)(ROT_SERVO_OFFSET*10)>>8;
        strings[5]=(int)(ROT_SERVO_OFFSET*10);
        iic_writebuf(strings, EXTERNAL_EEPROM_SYS_ADDRESS, 0x870, 6);
        delay(500);
        //get the offset of assembling(address:0x870 *sequence[L R T]* each data 2 bytes) and the data is 10 times greater than the real in order to store easier
        iic_readbuf(strings, EXTERNAL_EEPROM_SYS_ADDRESS, 0x870, 6);
        LEFT_SERVO_OFFSET = ((int)(strings[0]<<8) + strings[1])/10.0;
        RIGHT_SERVO_OFFSET = ((int)(strings[2]<<8) + strings[3])/10.0;
        ROT_SERVO_OFFSET = ((int)(strings[4]<<8) + strings[5])/10.0;
#else
        LEFT_SERVO_OFFSET = read_servo_offset(SERVO_LEFT_NUM);
        RIGHT_SERVO_OFFSET = read_servo_offset(SERVO_RIGHT_NUM);
        ROT_SERVO_OFFSET = read_servo_offset(SERVO_ROT_NUM);
        HAND_ROT_SERVO_OFFSET = read_servo_offset(SERVO_HAND_ROT_NUM);

        // set the initial point - John Feng
        if (EEPROM.read(CALIBRATION_FLAG) != CONFIRM_FLAG)
        {
                alert(50, 10, 10);
        }
        uarm.move_to(10,100,150,false);
        attach_servo(SERVO_ROT_NUM);
        attach_servo(SERVO_LEFT_NUM);
        attach_servo(SERVO_RIGHT_NUM);
        attach_servo(SERVO_HAND_ROT_NUM);
#endif
}


/*!
   \brief Write 3 Servo Angles, servo_rot, servo_left, servo_right
   \param servo_rot_angle SERVO_ROT_NUM
   \param servo_left_angle SERVO_LEFT_NUM
   \param servo_right_angle SERVO_RIGHT_NUM
   \return SUCCESS, FAILED
 */
int uArmClass::write_servo_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle)
{
#ifdef PRODUCT_MKII
        write_servo_angle(SERVO_ROT_NUM,servo_rot_angle);
        write_servo_angle(SERVO_LEFT_NUM,servo_left_angle);
        write_servo_angle(SERVO_RIGHT_NUM,servo_right_angle);
#else
        write_servo_angle(SERVO_ROT_NUM,servo_rot_angle,true);
        write_servo_angle(SERVO_LEFT_NUM,servo_left_angle,true);
        write_servo_angle(SERVO_RIGHT_NUM,servo_right_angle,true);
#endif
        // refresh logical servo angle cache
        //cur_rot = servo_rot_angle;
        //cur_left = servo_left_angle;
        //cur_right = servo_right_angle;
}

/*!
   \brief Write the angle to Servo
   \param servo_number SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \param servo_angle Servo target angle, 0.00 - 180.00
   \param writeWithoffset True: with Offset, False: without Offset
 */
void uArmClass::write_servo_angle(byte servo_number, double servo_angle)
{
#ifdef PRODUCT_MKII
        attach_servo(servo_number);
        switch(servo_number)
        {
        case SERVO_ROT_NUM:
                g_servo_rot.write(servo_angle);// ,hand_speed);
                cur_rot = servo_angle;
                break;
        case SERVO_LEFT_NUM:
                g_servo_left.write(servo_angle);// ,hand_speed);
                cur_left = servo_angle;
                break;
        case SERVO_RIGHT_NUM:
                g_servo_right.write(servo_angle);// ,hand_speed);
                cur_right = servo_angle;
                break;
        case SERVO_HAND_ROT_NUM:  //servo_angle = constrain(servo_angle, 5, 180);
                g_servo_hand_rot.write(servo_angle,hand_speed);     //set the hand speed
                cur_hand = servo_angle;
                break;
        default: break;
        }
#else
		write_servo_angle(servo_number,servo_angle,true);
#endif			
}

#ifndef PRODUCT_MKII
void uArmClass::write_servo_angle(byte servo_number, double servo_angle, boolean writeWithoffset)
{
        // attach_servo(servo_number);
        servo_angle = writeWithoffset ? (servo_angle + read_servo_offset(servo_number)) : servo_angle;
        servo_angle = constrain(servo_angle,0.0,180.0);
        switch(servo_number)
        {
        case SERVO_ROT_NUM:       g_servo_rot.write(servo_angle);
                cur_rot = servo_angle - ROT_SERVO_OFFSET;
                break;
        case SERVO_LEFT_NUM:      g_servo_left.write(servo_angle);
                cur_left = servo_angle - LEFT_SERVO_OFFSET;
                break;
        case SERVO_RIGHT_NUM:     g_servo_right.write(servo_angle);
                cur_right = servo_angle - RIGHT_SERVO_OFFSET;
                break;
        case SERVO_HAND_ROT_NUM:  g_servo_hand_rot.write(servo_angle);
                cur_hand = servo_angle - read_servo_offset(SERVO_HAND_ROT_NUM);
                break;
        default:                  break;
        }
}
#endif

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
        //Serial.println("attach_all");
}

/*!
   \brief Attach Servo, if servo has not been attached, attach the servo, and read the current Angle
   \param servo number SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
 */
void uArmClass::attach_servo(byte servo_number)
{
#ifdef PRODUCT_MKII
        switch(servo_number) {
        case SERVO_ROT_NUM:
                if(!g_servo_rot.attached()) {
                        read_servo_angle(SERVO_ROT_NUM,true);
                        g_servo_rot.attach(SERVO_ROT_PIN);
                        g_servo_rot.write(cur_rot);
                }
                break;
        case SERVO_LEFT_NUM:
                if (!g_servo_left.attached()) {
                        read_servo_angle(SERVO_LEFT_NUM,true);
                        g_servo_left.attach(SERVO_LEFT_PIN);
                        g_servo_left.write(cur_left);
                }
                break;
        case SERVO_RIGHT_NUM:
                if (!g_servo_right.attached()) {
                        read_servo_angle(SERVO_RIGHT_NUM,true);
                        g_servo_right.attach(SERVO_RIGHT_PIN);
                        g_servo_right.write(cur_right);
                }
                break;
        case SERVO_HAND_ROT_NUM:
                if (!g_servo_hand_rot.attached()) {
                        read_servo_angle(SERVO_HAND_ROT_NUM,true);
                        g_servo_hand_rot.attach(SERVO_HAND_ROT_PIN,600,2400);
                        g_servo_hand_rot.write(cur_hand);
                }
                break;
        }

#else
        boolean is_linear_calibrated = false;
        if (EEPROM.read(CALIBRATION_LINEAR_FLAG) == CONFIRM_FLAG)
        {
                is_linear_calibrated = true;
        }
        switch(servo_number) {
        case SERVO_ROT_NUM:
                if (is_linear_calibrated == true) {
                        read_servo_angle(SERVO_ROT_NUM);
                        uarm.g_servo_rot.attach(SERVO_ROT_PIN);
                        uarm.g_servo_rot.write(cur_rot + ROT_SERVO_OFFSET);
                }
                else{
                        uarm.g_servo_rot.attach(SERVO_ROT_PIN);
                        uarm.g_servo_rot.write(90);
                }
                break;
        case SERVO_LEFT_NUM:
                if (is_linear_calibrated == true) {
                        uarm.read_servo_angle(SERVO_LEFT_NUM);
                        uarm.g_servo_left.attach(SERVO_LEFT_PIN);
                        uarm.g_servo_left.write(cur_left + LEFT_SERVO_OFFSET);
                }
                else{
                        uarm.g_servo_left.attach(SERVO_LEFT_PIN);
                        uarm.g_servo_left.write(100);
                }
                break;
        case SERVO_RIGHT_NUM:
                if (is_linear_calibrated == true) {
                        uarm.read_servo_angle(SERVO_RIGHT_NUM);
                        uarm.g_servo_right.attach(SERVO_RIGHT_PIN);
                        uarm.g_servo_right.write(cur_right + RIGHT_SERVO_OFFSET);
                }
                else{
                        uarm.g_servo_right.attach(SERVO_RIGHT_PIN);
                        uarm.g_servo_right.write(60);
                }
                break;
        case SERVO_HAND_ROT_NUM:
                if (is_linear_calibrated == true) {
                        uarm.read_servo_angle(SERVO_HAND_ROT_NUM);
                        uarm.g_servo_hand_rot.attach(SERVO_HAND_ROT_PIN);
                        uarm.g_servo_hand_rot.write(cur_hand + HAND_ROT_SERVO_OFFSET);
                }
                break;
        }
#endif
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
   \brief Convert the Analog to Servo Angle, by this formatter, angle = intercept + slope * analog
   \param input_analog Analog Value
   \param servo_num SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \param withOffset true, false
   \return Return Servo Angle
 */

double uArmClass::analog_to_angle(int input_analog, byte servo_num)
{
#ifdef PRODUCT_MKII
  unsigned char adc_calibration_data[DATA_LENGTH],data[4]; //get the calibration data around the data input
  unsigned int min_data_calibration_address, max_calibration_data, min_calibration_data;
  unsigned int angle_range_min, angle_range_max;
  switch(servo_num)
  {
    case  SERVO_ROT_NUM:      iic_readbuf(&data[0], EXTERNAL_EEPROM_SYS_ADDRESS, ROT_SERVO_ADDRESS + 360, 2);//get the min adc calibration data for the map() function
                              iic_readbuf(&data[2], EXTERNAL_EEPROM_SYS_ADDRESS, ROT_SERVO_ADDRESS + 360 + 358, 2);//get the max adc calibraiton data for the map() function
                              break;
    case  SERVO_LEFT_NUM:     iic_readbuf(&data[0], EXTERNAL_EEPROM_SYS_ADDRESS, LEFT_SERVO_ADDRESS + 360, 2);//get the min adc calibration data for the map() function
                              iic_readbuf(&data[2], EXTERNAL_EEPROM_SYS_ADDRESS, LEFT_SERVO_ADDRESS + 360 + 358, 2);//get the max adc calibraiton data for the map() function
                              break;
    case  SERVO_RIGHT_NUM:    iic_readbuf(&data[0], EXTERNAL_EEPROM_SYS_ADDRESS, RIGHT_SERVO_ADDRESS + 360, 2);//get the min adc calibration data for the map() function
                              iic_readbuf(&data[2], EXTERNAL_EEPROM_SYS_ADDRESS, RIGHT_SERVO_ADDRESS + 360 + 358, 2);//get the max adc calibraiton data for the map() function
                              break;
    default:                  break;
  }

  max_calibration_data = (data[2]<<8) + data[3];
  min_calibration_data = (data[0]<<8) + data[1];

  angle_range_min = map(input_analog, min_calibration_data, max_calibration_data, 1, 180) - (DATA_LENGTH>>2);
  min_data_calibration_address = (angle_range_min * 2);
  switch(servo_num)
  {
    case  SERVO_ROT_NUM:      iic_readbuf(adc_calibration_data, EXTERNAL_EEPROM_SYS_ADDRESS, ROT_SERVO_ADDRESS + min_data_calibration_address + 360, DATA_LENGTH);//360 means the adc calibration data offset
                              break;
    case  SERVO_LEFT_NUM:     iic_readbuf(adc_calibration_data, EXTERNAL_EEPROM_SYS_ADDRESS, LEFT_SERVO_ADDRESS + min_data_calibration_address + 360, DATA_LENGTH);//360 means the adc calibration data offset
                              break;
    case  SERVO_RIGHT_NUM:    iic_readbuf(adc_calibration_data, EXTERNAL_EEPROM_SYS_ADDRESS, RIGHT_SERVO_ADDRESS + min_data_calibration_address + 360, DATA_LENGTH);//360 means the adc calibration data offset
                              break;
    default:                  break;    
  }

  unsigned int deltaA = 0xffff, deltaB = 0, i, i_min = 0;
  for(i=0;i<(DATA_LENGTH >> 1);i++)
  {
      deltaB = abs ((adc_calibration_data[i+i]<<8) + adc_calibration_data[1+(i+i)] - input_analog);
      if(deltaA > deltaB)
      {
        i_min = i;
        deltaA = deltaB;
      }
  }

  angle_range_min = angle_range_min + i_min;
  angle_range_max = angle_range_min + 1;

  if((((adc_calibration_data[i_min+i_min]<<8) + adc_calibration_data[1+i_min+i_min]) - input_analog) >= 0)//determine if the current value bigger than the input_analog
  {
    //angle_rang_min = map(input_analog, min_calibration_data, max_calibration_data, 0, 180) - (DATA_LENGTH>>2);
    max_calibration_data = (adc_calibration_data[i_min+i_min]<<8) + adc_calibration_data[i_min+i_min+1];
    min_calibration_data = (adc_calibration_data[i_min+i_min-2]<<8) + adc_calibration_data[i_min+i_min-1];

  }
  else
  {
    angle_range_min++;//change the degree range
    angle_range_max++;
    max_calibration_data = (adc_calibration_data[i_min+i_min+2]<<8) + adc_calibration_data[i_min+i_min+3];
    min_calibration_data = (adc_calibration_data[i_min+i_min]<<8) + adc_calibration_data[i_min+i_min+1];
  }
 
  if(min_calibration_data < max_calibration_data)//return the angle
  {
    return ( 1.0 * (input_analog - min_calibration_data)/(max_calibration_data - min_calibration_data) + angle_range_min);
  }
  else
  {
    return (angle_range_min + angle_range_max) / 2.0;//angle from 1-180 but the address from 0-179
  } 
#else
		analog_to_angle(input_analog, servo_num, true);
#endif
}
#ifndef PRODUCT_MKII
double uArmClass::analog_to_angle(int input_analog, byte servo_num, bool withOffset)
{
        double intercept = 0.0f;
        double slope = 0.0f;
        read_linear_offset(servo_num, intercept, slope);
        double angle = intercept + slope*input_analog;
        return withOffset ? angle + read_servo_offset(servo_num) : angle;
}
#endif

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
unsigned char uArmClass::coordinate_to_angle(double x, double y, double z, double *theta_1, double *theta_2, double *theta_3)//theta_1:rotation angle   theta_2:the angle of lower arm and horizon   theta_3:the angle of upper arm and horizon
{
  double x_in = 0.0;
  double z_in = 0.0;
  double right_all = 0.0;
  double sqrt_z_x = 0.0;
  double phi = 0.0;
        x = (double)((int)(x*100)/100.0);
        y = (double)((int)(y*100)/100.0);
        z = (double)((int)(z*100)/100.0);

  z_in = (z - MATH_L1) / MATH_L3;
  if(move_to_the_closest_point == false)//if need the move to closest point we have to jump over the return function
  {
  	//check the range of x
  	if(y<0)
  	{
    	return OUT_OF_RANGE;
  	}
  	//check the range of z
  	if((z<ARM_HEIGHT_MIN)||(z>ARM_HEIGHT_MAX))
  	{
  		return OUT_OF_RANGE;
  	}
  }
  // Calculate value of theta 1: the rotation angle
  if(x==0)
  {
    (*theta_1) = 90;
  }
  else
  {
    //theta_1 = atan(y / x)*MATH_TRANS;

    if (x > 0)
    {
      *theta_1 = atan(y / x)*MATH_TRANS;//angle tranfer 0-180 CCW

    }
    if (x < 0) 
    {
      (*theta_1) = 180 + atan(y / x)*MATH_TRANS;//angle tranfer  0-180 CCW

    }

  }
            
  	// Calculate value of theta 3
  if((*theta_1)!=90)//x_in is the stretch
  {
    x_in = (x / cos((*theta_1) / MATH_TRANS) - MATH_L2) / MATH_L3;
  }
  else
  {
    x_in = (y - MATH_L2) / MATH_L3;
  }

  /*if(write_stretch_height_rot(x_in,z_in,theta_1,theta_2,theta_3)==IN_RANGE)
  {
  	return IN_RANGE;
  }
  else
  {
  	return OUT_OF_RANGE;
  }*/
  phi = atan(z_in / x_in)*MATH_TRANS;//phi is the angle of line (from joint 2 to joint 4) with the horizon

  sqrt_z_x = sqrt(z_in*z_in + x_in*x_in);

  right_all = (sqrt_z_x*sqrt_z_x + MATH_L43*MATH_L43 - 1) / (2 * MATH_L43 * sqrt_z_x);//cosin law
  (*theta_3) = acos(right_all)*MATH_TRANS;//cosin law

  // Calculate value of theta 2
  right_all = (sqrt_z_x*sqrt_z_x + 1 - MATH_L43*MATH_L43) / (2 * sqrt_z_x);//cosin law
  (*theta_2) = acos(right_all)*MATH_TRANS;//cosin law

  (*theta_2) = (*theta_2) + phi;
  (*theta_3) = (*theta_3) - phi;
  //determine if the angle can be reached
  if(isnan((*theta_1))||isnan((*theta_2))||isnan((*theta_3)))
  {
  	return OUT_OF_RANGE;
  }
  if((z_in <= ARM_HEIGHT_MIN) || (z_in >= (ARM_HEIGHT_MAX - MATH_L1) / MATH_L3))//check if height is in range
  {
  	return OUT_OF_RANGE;
  }
  if((x_in <= ARM_STRETCH_MIN) || (x_in >= (double)(ARM_STRETCH_MAX - MATH_L2) / MATH_L3)) //check if stretch is in range
  {
    return OUT_OF_RANGE;
  }
  if((((*theta_2) - LEFT_SERVO_OFFSET) < L3_MIN_ANGLE)||(((*theta_2) - LEFT_SERVO_OFFSET) > L3_MAX_ANGLE))//check the (*theta_2) in range
  {
    return OUT_OF_RANGE;
  }
  if((((*theta_3) - RIGHT_SERVO_OFFSET) < L4_MIN_ANGLE)||(((*theta_3) - RIGHT_SERVO_OFFSET) > L4_MAX_ANGLE))//check the (*theta_3) in range
  {
    return OUT_OF_RANGE;
  }
  if(((180 - (*theta_3) - (*theta_2))>L4L3_MAX_ANGLE)||((180 - (*theta_3) - (*theta_2))<L4L3_MIN_ANGLE))//check the angle of upper arm and lowe arm in range
  {
    return OUT_OF_RANGE;
  }

  return IN_RANGE;
}

#ifndef PRODUCT_MKII
/*!
   \brief Calculate X,Y,Z to g_current_x,g_current_y,g_current_z
 */
void uArmClass::get_current_xyz()
{
        double theta_1 = uarm.analog_to_angle(analogRead(SERVO_ROT_ANALOG_PIN),SERVO_ROT_NUM, false);
        double theta_2 = uarm.analog_to_angle(analogRead(SERVO_LEFT_ANALOG_PIN),SERVO_LEFT_NUM, false);
        double theta_3 = uarm.analog_to_angle(analogRead(SERVO_RIGHT_ANALOG_PIN),SERVO_RIGHT_NUM, false);
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
#endif
/*!
   \brief get the current rot left right angles
 */
void uArmClass::get_current_rotleftright()
{
#ifdef PRODUCT_MKII
read_servo_angle(SERVO_ROT_NUM);
read_servo_angle(SERVO_LEFT_NUM);
read_servo_angle(SERVO_RIGHT_NUM);
#else
cur_rot = read_servo_angle(SERVO_ROT_NUM);
cur_left = read_servo_angle(SERVO_LEFT_NUM);
cur_right = read_servo_angle(SERVO_RIGHT_NUM);
#endif
}
#ifdef PRODUCT_MKII
/*!
   \brief get the calibration data from the external eeprom
   \param rot the calibration data of rotation
   \param left the calibration data of left
   \param right the calibration data of right
 */

void uArmClass::read_servo_calibration_data(double *rot, double *left, double *right)
{

  calibration_data_to_servo_angle(rot,ROT_SERVO_ADDRESS);
  calibration_data_to_servo_angle(left,LEFT_SERVO_ADDRESS);
  calibration_data_to_servo_angle(right,RIGHT_SERVO_ADDRESS);

} 

/*!
   \brief check the external eeprom and transfer the ideal data to real angle data
   \param data the address of the variable
   \param address the section starting address of the external eeprom 
*/

void uArmClass::calibration_data_to_servo_angle(double *data,unsigned int address)
{
  unsigned char calibration_data[DATA_LENGTH]; //get the calibration data around the data input
  unsigned int min_data_calibration_address;
  double closest_data, another_closest_data;
  unsigned int deltaA = 0xffff, deltaB = 0, i, i_min = 0;
  deltaA = 0xffff;
  deltaB = 0;
  min_data_calibration_address = (((unsigned int)(*data) - (DATA_LENGTH >> 2)) * 2);
  iic_readbuf(calibration_data, EXTERNAL_EEPROM_SYS_ADDRESS, address + min_data_calibration_address, DATA_LENGTH);
  for(i=0;i<(DATA_LENGTH >> 1);i++)
  {
      deltaB = abs ((calibration_data[i+i]<<8) + calibration_data[1+(i+i)] - (*data) * 10);
      if(deltaA > deltaB)
      {
        i_min = i;
        deltaA = deltaB;
      } 
  }

  closest_data = ((calibration_data[i_min+i_min]<<8) + calibration_data[1+(i_min+i_min)])/10.0;//transfer the dat from ideal data to servo angles
  if((*data) >= closest_data)
  {
    another_closest_data = ((calibration_data[i_min+i_min+2]<<8) + calibration_data[3+i_min+i_min])/10.0;//bigger than closest
    if(another_closest_data == closest_data)
    {
      *data = min_data_calibration_address/2 + i_min + 1 + 0.5;
    }
    else
    {
      *data = 1.0 * (*data - closest_data) / (another_closest_data - closest_data) + min_data_calibration_address/2 + i_min + 1;
    }
  }
  else
  {
    another_closest_data = ((calibration_data[i_min+i_min-2]<<8) + calibration_data[i_min+i_min-1])/10.0;//smaller than closest
    if(another_closest_data == closest_data)
    {
      *data = min_data_calibration_address/2 + i_min + 0.5;
    }
    else
    {
      *data = 1.0 * (*data - another_closest_data) / (closest_data - another_closest_data) + min_data_calibration_address/2 + i_min;
    }
  }  
}

void uArmClass::read_servo_angle(byte servo_number, bool original_data)
{
  double angle = 0;
  unsigned int address;
  double *data;

  switch(servo_number) {
    case SERVO_ROT_NUM:
      address = ROT_SERVO_ADDRESS;
      data = &cur_rot;
      break;
    case SERVO_LEFT_NUM:
      address = LEFT_SERVO_ADDRESS;
      data = &cur_left;
      break;
    case SERVO_RIGHT_NUM:
      address = RIGHT_SERVO_ADDRESS;
      data = &cur_right;
      break;
    case SERVO_HAND_ROT_NUM:
				cur_hand = map(analogRead(SERVO_HAND_ROT_ANALOG_PIN), SERVO_9G_MIN, SERVO_9G_MAX, 0, 180); //g_servo_hand_rot.read();  // SERVO_HAND_ROT_ANALOG_PIN),SERVO_HAND_ROT_NUM);
      return;
      break;
  }

  unsigned int dat[8], temp;
  unsigned char i=0,j=0;
  for(i=0;i<8;i++)
  {
    switch(address)
    {
      case ROT_SERVO_ADDRESS: dat[i] = analogRead(SERVO_ROT_ANALOG_PIN);break;
      case LEFT_SERVO_ADDRESS: dat[i] = analogRead(SERVO_LEFT_ANALOG_PIN);break;
      case RIGHT_SERVO_ADDRESS: dat[i] = analogRead(SERVO_RIGHT_ANALOG_PIN);break;
      default:break;
    }
  }
  for(i=0;i<8;i++){//BULB to get the most accuracy data
    for(j=0;i+j<7;j++){
      if(dat[j]>dat[j+1]){
        temp = dat[j];
        dat[j] = dat[j+1];
        dat[j+1] = temp;
      }
    }
  }
  switch(address)
  {
    case ROT_SERVO_ADDRESS: (*data) = uarm.analog_to_angle((dat[2]+dat[3]+dat[4]+dat[5])/4,SERVO_ROT_NUM);break;
    case LEFT_SERVO_ADDRESS: (*data) = uarm.analog_to_angle((dat[2]+dat[3]+dat[4]+dat[5])/4,SERVO_LEFT_NUM);break;
    case RIGHT_SERVO_ADDRESS: (*data) = uarm.analog_to_angle((dat[2]+dat[3]+dat[4]+dat[5])/4,SERVO_RIGHT_NUM);break;
    default:break;
  }
  if((original_data == false)&&(servo_number != SERVO_HAND_ROT_NUM))//servo hand do not have the calibration data, jump over!
  {
    //check the external eeprom and transfer the real angle to ideal angle
    unsigned char ideal_angle[4];
    iic_readbuf(ideal_angle, EXTERNAL_EEPROM_SYS_ADDRESS, address + (((unsigned int)(*data) - 1) << 1), 4);
    (*data) = (double)(((ideal_angle[2] << 8) + ideal_angle[3]) - ((ideal_angle[0] << 8) + ideal_angle[1])) * ((*data) - (unsigned int)(*data)) + ((ideal_angle[0] << 8) + ideal_angle[1]);
    (*data) = (*data) / 10.0;
			    switch(servo_number) {
			      case SERVO_ROT_NUM:
			          (*data) += ROT_SERVO_OFFSET;
			          break;
			      case SERVO_LEFT_NUM:
			          (*data) += LEFT_SERVO_OFFSET;
			          break;
			      case SERVO_RIGHT_NUM:
			          (*data) += RIGHT_SERVO_OFFSET;
			          break;
			      default:
			          break;
			    }
			  }
        }
}
#else
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
        for (byte i = 0; i < 5; i++) {
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

/*!
   \brief Convert the Analog to Servo Angle, by this formatter, angle = intercept + slope * analog
   \param input_analog Analog Value
   \param servo_num SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \param withOffset true, false
   \return Return Servo Angle
 */

/*!
   \brief read Linear Offset from EEPROM, From LINEAR_INTERCEPT_START_ADDRESS & LINEAR_SLOPE_START_ADDRESS, each offset occupy 4 bytes in rom
   \param servo_num SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \param intercept_val get intercept_val
   \param slope_val get slope_val
 */
void uArmClass::read_linear_offset(byte servo_num, double& intercept_val, double& slope_val){
        EEPROM.get(LINEAR_INTERCEPT_START_ADDRESS + servo_num * sizeof(intercept_val), intercept_val);
        EEPROM.get(LINEAR_SLOPE_START_ADDRESS + servo_num * sizeof(slope_val), slope_val);
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
#endif

/*!
   \brief Calculate X,Y,Z to g_current_x,g_current_y,g_current_z
   \param *cur_rot the address of value we want to caculate
   \param *cur_left the address of value we want to caculate
   \param *cur_right the address of value we want to caculate
   \param *g_currnet_x the address of value we want to caculate
   \param *g_current_y the address of value we want to caculate
   \param *g_current_z the address of value we want to caculate
   \param for movement is the flage to detect if we should get the real current angle of the uarm
 */
unsigned char uArmClass::get_current_xyz(double *cur_rot, double *cur_left , double *cur_right, double *g_current_x, double *g_current_y, double *g_current_z, bool for_movement )
{
  if(for_movement==true){
    get_current_rotleftright();
  }

        //add the offset first
#ifndef PRODUCT_MKII
        *cur_left = *cur_left + LEFT_SERVO_OFFSET;
        *cur_right = *cur_right + RIGHT_SERVO_OFFSET;
#endif

  double stretch = MATH_L3 * cos((*cur_left) / MATH_TRANS) + MATH_L4 * cos((*cur_right) / MATH_TRANS) + MATH_L2;
  double height = MATH_L3 * sin((*cur_left) / MATH_TRANS) - MATH_L4 * sin((*cur_right) / MATH_TRANS) + MATH_L1;
  *g_current_x = stretch * cos((*cur_rot) / MATH_TRANS);
  *g_current_y = stretch * sin((*cur_rot) / MATH_TRANS);
  *g_current_z = height;

  //used in FK
  if(for_movement == false)
  {
    if((stretch < ARM_STRETCH_MIN)||(stretch > ARM_STRETCH_MAX))
    {
      return OUT_OF_RANGE;
    }
    if((height < ARM_HEIGHT_MIN)||(height > ARM_HEIGHT_MAX))
    {
      return OUT_OF_RANGE;
    }
    if((*cur_rot < 0)||(*cur_rot > 180))
    {
      return OUT_OF_RANGE; 
    }
  }
  return IN_RANGE;
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
   \param x X Axis Value if polar is true then x is the stretch
   \param y Y Axis Value if polar is true then y is the rot angle
   \param z Z Axis Value if polar is true then z is the height
   \param hand_angle Hand Axis
   \param relative_flags ABSOLUTE, RELATIVE
   \param enable_hand Enable Hand Axis
   \param polar is xyz coordinates or stretch&height&rot
*/

unsigned char uArmClass::move_to(double x, double y, double z, double hand_angle, byte relative_flags, double times, byte ease_type, boolean enable_hand, bool polar) {
    if (EEPROM.read(CALIBRATION_LINEAR_FLAG) != CONFIRM_FLAG)
    {
        alert(50, 10, 10);
        return FAILED;
    }
  if(polar == true)//change the stretch rot and height to xyz coordinates
  {
  	double stretch = x;
  	//Z and height is the same
  	//transfer stretch to xy
  	x = stretch * cos(y / MATH_TRANS);
  	y = stretch * sin(y / MATH_TRANS);
  }
  // get current angles of servos
  
    // find current position using cached servo values
    //double current_x;
    //double current_y;
    //double current_z;
    //angle_to_coordinate(cur_rot, cur_left, cur_right, current_x, current_y, current_z);

  // deal with relative xyz positioning
  if(relative_flags == RELATIVE)
  {
    x = g_current_x + x;
    y = g_current_x + y;
    z = g_current_z + z;
    //hand_angle = current_hand + hand_angle;
  }

  // find target angles
  double tgt_rot;
  double tgt_left;
  double tgt_right;

  //  detect if the xyz coordinate are in the range
  if(coordinate_to_angle(x, y, z, &tgt_rot, &tgt_left, &tgt_right) == OUT_OF_RANGE)
  {
  	//check if need to check the out_of_range
  	if(move_to_the_closest_point==false){
    	return OUT_OF_RANGE_IN_DST;
  	}
  }

  //calculate the length and use the longest to determine the numbers of interpolation
  unsigned int delta_rot=abs(tgt_rot-cur_rot);
  unsigned int delta_left=abs(tgt_left-cur_left);
  unsigned int delta_right=abs(tgt_right-cur_right);

  INTERP_INTVLS = max(delta_rot,delta_left);
  INTERP_INTVLS = max(INTERP_INTVLS,delta_right);

  INTERP_INTVLS = (INTERP_INTVLS<60) ? INTERP_INTVLS : 60;
  //INTERP_INTVLS =1;// INTERP_INTVLS * (10 / times);// speed determine the number of interpolation
  times = constrain(times, 100, 1000);
  hand_speed = times;//set the had rot speed

    interpolate(g_current_x, x, x_array, ease_type);// /10 means to make sure the t*t*t is still in the range
    interpolate(g_current_y, y, y_array, ease_type);
    interpolate(g_current_z, z, z_array, ease_type);


    //give the final destination value to the array
    x_array[INTERP_INTVLS] = x;
    y_array[INTERP_INTVLS] = y;
    z_array[INTERP_INTVLS] = z;

    double rot, left, right;
    double x_backup, y_backup, z_backup;
    for (byte i = 0; i <= INTERP_INTVLS; i++)
    {
      //check if all the data in range and give the tlr angles to the xyz array
      if(coordinate_to_angle(x_array[i], y_array[i], z_array[i], &rot, &left, &right) == OUT_OF_RANGE)
      {
      	if(move_to_the_closest_point==false){
      		return OUT_OF_RANGE_IN_PATH;
      	}
        else{
        	//find the last available xyz coordinates and change it to the destination
        	INTERP_INTVLS = (i <= 1)? 0 : (i - 1);
        	// give the late avaiable xyz coordinates to the variable
        	x = x_backup;
        	y = y_backup;
        	z = z_backup;

        	break;//get out of the for(;;) cycle
        }
      }
      else
      {
      	//backup the old xyz coordinates data first
      	x_backup = x_array[i];
      	y_backup = y_array[i];
      	z_backup = z_array[i];
        //change to the rot/left/right angles
        x_array[i] = rot;
        y_array[i] = left;
        z_array[i] = right;
      }
    }
  //caculate the distance from the destination
  double distance = pow(pow(x-g_current_x, 2) + pow(y-g_current_y, 2) + pow(z-g_current_z, 2), 0.5);
  moveStartTime = millis();// Speed of the robot in mm/s
  microMoveTime = distance / times * 1000.0 / INTERP_INTVLS;//the time for every step

	g_current_x = x;
	g_current_y = y;
	g_current_z = z;
	//cur_rot = rot;
	//cur_left = left;
	//cur_right = right;
	//cur_hand = hand_angle;
  move_times = 0;//start to caculate the movement
  return IN_RANGE;
}

/*!
   \brief Calculate X,Y,Z to g_current_x,g_current_y,g_current_z
 */
 /*
void uArmClass::get_current_xyz(double theta_1, double theta_2, double theta_3)
{
        double l5 = (MATH_L2 + MATH_L3*cos(theta_2 / MATH_TRANS) + MATH_L4*cos(theta_3 / MATH_TRANS));

        g_current_x = -cos(abs(theta_1 / MATH_TRANS))*l5;
        g_current_y = -sin(abs(theta_1 / MATH_TRANS))*l5;
        g_current_z = MATH_L1 + MATH_L3*sin(abs(theta_2 / MATH_TRANS)) - MATH_L4*sin(abs(theta_3 / MATH_TRANS));
*/

/*!
   \brief Gripper catch
*/
void uArmClass::gripper_catch(bool value)
{
#ifdef PRODUCT_MKII
  if(value)
  {
    digitalWrite(PUMP_GRI_EN, LOW);  // gripper and pump catch
  }
  else
  {
    digitalWrite(PUMP_GRI_EN, HIGH);  //gripper and pump off
  }
#else
  if(value)
  {
    digitalWrite(GRIPPER, LOW);  // gripper and pump catch
  }
  else
  {
    digitalWrite(GRIPPER, HIGH);  // gripper and pump off
  }
#endif
}
/*!
   \brief Pump catch
*/
void uArmClass::pump_catch(bool value)
{
#ifdef PRODUCT_MKII
  gripper_catch(value);
#else
  if(value)
  {
    digitalWrite(PUMP_EN, HIGH);  // pump catch
    digitalWrite(VALVE_EN, LOW);
  }
  else
  {
    digitalWrite(PUMP_EN, LOW);  // pump catch
    digitalWrite(VALVE_EN, HIGH);
  }
#endif
}
/*!
   \brief Get Gripper Status
*/
unsigned char uArmClass::gripper_status()
{
#ifdef PRODUCT_MKII
  if(digitalRead(PUMP_GRI_EN) == HIGH)
  {
    return STOP;//NOT WORKING
  }
  else
  {
    if(digitalRead(PUMP_GRI_STATUS) == HIGH)
    {
      return GRABBING;
    }
    else
    {
      return WORKING;
    }
  }
#else
  if(digitalRead(GRIPPER) == HIGH)
  {
  	return STOP;
  }
  else
  {
  	if(analogRead(GRIPPER_FEEDBACK) > 600)
  	{
  		return WORKING;
  	}
  	else
  	{
  		return GRABBING;
  	}
  }
#endif
}

#ifdef PRODUCT_MKII
/*!
   \brief Get Pump Status
*/
unsigned char uArmClass::pump_status()
{

  if(digitalRead(PUMP_GRI_EN) == HIGH)
  {
    return STOP;//NOT WORKING
  }
  else
  {
    if(analogRead(PUMP_GRI_STATUS) >= PUMP_GRABBING_CURRENT)
    {
      return GRABBING;
    }
    else
    {
      return WORKING;
    }
  }

}
#endif

//*************************************uart communication**************************************//
void uArmClass::runCommand(String cmnd){

    // To save memory, create the "[OK" and "]\n" right now, in flash memory
    String S   = F("[S]"); 
 	String S0  = F("[S0]");
 	String S1  = F("[S1]");
 	String S2  = F("[S2]");
 	String F   = F("[F]");
 	String F0  = F("[F0]");
 	String F1  = F("[F1]");
 	String errorResponse;

    // sMov Command----------------------------------------------------------
    if(cmnd.indexOf(F("sMov")) >= 0){
      String parameters[] = {F("X"), F("Y"), F("Z"), F("V")};
      double values[4];
      errorResponse = getValues(cmnd, parameters, 4, values);
      if(errorResponse.length() == 0) {								//means no err
      	Serial.println(S);// successful feedback send it immediately
      	//limit the speed
      	move_to_the_closest_point = true;
      	move_to(values[0], values[1], values[2], values[3], false);
      	move_to_the_closest_point = false;
  	  }

    }else
    
     //sPolS#H#R#--------------------------------------------------------------
    if(cmnd.indexOf(F("sPol")) >= 0){
      String parameters[] = {F("S"), F("R"), F("H"), F("V")};
      double values[4];
      errorResponse = getValues(cmnd, parameters, 4, values);
      if(errorResponse.length() == 0) {
      	Serial.println(S);// successful feedback send it immediately
      	//limit the speed
     	move_to_the_closest_point = true;
      	move_to(values[0], values[1], values[2], values[3], true);
      	move_to_the_closest_point = false;
	  }
    }else

    // sAttachS#----------------------------------------------------------------
    if(cmnd.indexOf(F("sAtt")) >= 0){
      String parameters[] = {F("S")};
      double values[1];
      String errorResponse        = getValues(cmnd, parameters, 1, values);
      if(errorResponse.length() == 0) {
      	Serial.println(S);// successful feedback send it immediately
      	attach_servo(values[0]);
  	  }
    }else

    // sDetachS#----------------------------------------------------------------
    if(cmnd.indexOf(F("sDet")) >= 0){
      String parameters[] = {F("S")};
      double values[1];
      String errorResponse        = getValues(cmnd, parameters, 1, values);
      if(errorResponse.length() == 0) {
      	Serial.println(S);// successful feedback send it immediately
      	detach_servo(values[0]);
      }
    }else

        // sServoN#V#--------------------------------------------------------------
        if(cmnd.indexOf(F("sSer")) >= 0)
        {

                String servoSetParameters[] = {F("N"), F("V")};
                double values[2];
      errorResponse = getValues(cmnd, servoSetParameters, 2, values);
      if(errorResponse.length() == 0) {
	  Serial.println(S);
#ifdef PRODUCT_MKII
      switch((int)values[0])
      {
        case 0:
          values[1] -= ROT_SERVO_OFFSET;
          calibration_data_to_servo_angle(&values[1],ROT_SERVO_ADDRESS);
          break;
        case 1:
          values[1] -= LEFT_SERVO_OFFSET;
          calibration_data_to_servo_angle(&values[1],LEFT_SERVO_ADDRESS);
          break;
        case 2:
          values[1] -= RIGHT_SERVO_OFFSET;
          calibration_data_to_servo_angle(&values[1],RIGHT_SERVO_ADDRESS);
          break;
        case 3:
          break;
      }
	  uarm.write_servo_angle(values[0], values[1]);
#else
              // in write_servo_angle function, add offset
                uarm.write_servo_angle(byte(values[0]), values[1], true);
#endif
        }
        }else

    //sPumpV#------------------------------------------------------------------
    if(cmnd.indexOf(F("sPum")) >= 0){
       String parameters[] = {F("V")};
       double values[1];
       String errorResponse        = getValues(cmnd, parameters, 1, values);
       if(errorResponse.length() == 0) {
      	 Serial.println(S);// successful feedback send it immediately

       	 if(values[0]==0)//off
                {
                        pump_catch(false);
                }else//on
                {
                        pump_catch(true);
                }
      }
    }else
        //sGripperV#----------------------------------------------------------------
        if(cmnd.indexOf(F("sGri")) >= 0) {
                String parameters[] = {F("V")};
                double values[1];
                String errorResponse        = getValues(cmnd, parameters, 1, values);
       if(errorResponse.length() == 0) {
      	 Serial.println(S);// successful feedback send it immediately
                if(values[0]==0)//release
                {
                        gripper_catch(false);
                }else//catch
                {
                        gripper_catch(true);
                }
       }
    }else

    //sBuzzF#T#-----------------------------------------------------------------
    if(cmnd.indexOf(F("sBuz")) >= 0){
      String parameters[] = { F("F"),F("T")};
      double values[2];
      String errorResponse        = getValues(cmnd, parameters, 2, values);
      if(errorResponse.length() == 0) {
      	Serial.println(S);// successful feedback send it immediately
      	tone(BUZZER, values[0]);
      	buzzerStopTime = millis() + int(values[1] * 1000.0); //sys_tick + values[1];
      }
        }else

    //sStp-------------------------------------------------------------------------
    if (cmnd.indexOf(F("sStp")) >= 0){
      	Serial.println(S);// successful feedback send it immediately
      	move_times = 255; //stop the movement
    }else

    //gVer----------------------------------------------------------------------
    if(cmnd.indexOf(F("gVer")) >= 0){
      Serial.println("[S" + String(current_ver) + "]");
    }else

    //gSimuX#Y#Z#V#-------------------------------------------------------------
    if(cmnd.indexOf(F("gSim")) >= 0){
      String parameters[] = {F("X"), F("Y"), F("Z")};
      double values[3];
      errorResponse = getValues(cmnd, parameters, 3, values);
      if(errorResponse.length() == 0) {
      	bool polar;
      	move_to_the_closest_point = false;//make sure move_to_the_closest_point is false so that we can get the out_of_range feedback
      	if(values[3]==1)
        	polar = true;
      	else
        	polar = false;
     	 switch(move_to(values[0], values[1], values[2], polar))
      	{
        	case IN_RANGE             :move_times=255;//disable move
                                  Serial.println(S0);
                                  break;
        	case OUT_OF_RANGE_IN_PATH :move_times=255;//disable move
                                  Serial.println(F0);
                                  break;
        	case OUT_OF_RANGE_IN_DST  :move_times=255;//disable move
                                  Serial.println(F1);
                                  break;
        	default:break;
      	}
  	  }
    }else
    //gCrd---------------------------------------------------------------------
    if(cmnd.indexOf(F("gCrd")) >= 0){
#ifdef PRODUCT_MKII
      get_current_xyz(&cur_rot, &cur_left, &cur_right, &g_current_x, &g_current_y, &g_current_z, true);
#else
                get_current_xyz(&cur_rot, &cur_left, &cur_right, &g_current_x, &g_current_y, &g_current_z, true);
#endif
      Serial.println("[SX" + String(g_current_x) + "Y" + String(g_current_y) + "Z" + String(g_current_z) + "]");
    }else

    //gPolS#R#H#--------------------------------------------------------------
    if(cmnd.indexOf(F("gPol")) >= 0){
#ifdef PRODUCT_MKII
      get_current_xyz(&cur_rot, &cur_left, &cur_right, &g_current_x, &g_current_y, &g_current_z, true);
#else
                void get_current_xyz();
#endif
      double stretch;
      stretch = sqrt(g_current_x * g_current_x + g_current_y * g_current_y);
      Serial.println("[SS" + String(stretch) + " R" + String(cur_rot) + " H" + String(g_current_z) + "]");

    }else
#ifdef PRODUCT_MKII
    //gPump---------------------------------------------------------------------
    if(cmnd.indexOf(F("gPum")) >= 0){
      switch(pump_status())
      {
        case GRABBING:Serial.println(S0);
                       break;
        case WORKING: Serial.println(S1);
                       break;
        case STOP:    Serial.println(S2);
                       break;
      }
    }else

    //gGipper-------------------------------------------------------------------
    if(cmnd.indexOf(F("gGri")) >= 0){
      switch(gripper_status())
      {
        case GRABBING:Serial.println(S0);
                       break;
        case WORKING: Serial.println(S1);
                       break;
        case STOP:    Serial.println(S2);
                       break;
      }
    }else
#endif
    //gAng---------------------------------------------------------------------
    if(cmnd.indexOf(F("gAng")) >= 0){
      get_current_rotleftright();
      read_servo_angle(SERVO_HAND_ROT_NUM);
      Serial.println("[ST" + String(cur_rot) + "L" + String(cur_left) + "R" + String(cur_right) + "F" + String(cur_hand) + "]");
    }else

    //gIKX#Y#Z#----------------------------------------------------------------
    if(cmnd.indexOf(F("gIK")) >= 0){
      String parameters[] = {F("X"), F("Y"), F("Z")};
      double values[3];
      errorResponse = getValues(cmnd, parameters, 3, values);
      if(errorResponse.length() == 0) {

      	double rot, left, right;
      	move_to_the_closest_point = false;
      	String letter;
      	if(coordinate_to_angle(values[0], values[1], values[2] , &rot, &left, &right) == OUT_OF_RANGE)
      	{
        	letter = F("[F");
      	}
      	else{
        	letter = F("[S");
        	left = left - LEFT_SERVO_OFFSET;//assembling offset
        	right = right - RIGHT_SERVO_OFFSET;//assembling offset
      	}
      	Serial.println(letter + "T" + String(rot) + "L" + String(left) + "R" + String(right) + "]");
      }
    }else

    //gFKT#L#R#-----------------------------------------------------------------
    // Get Forward Kinematics
    if(cmnd.indexOf(F("gFK")) >= 0){
      String parameters[] = {F("T"), F("L"), F("R")};
      double values[3];
      errorResponse = getValues(cmnd, parameters, 3, values);
      	if(errorResponse.length() == 0) {

      	double x, y, z;
      	String letter;
      	if(get_current_xyz(&values[0], &values[1], &values[2], &x, &y, &z, false) == OUT_OF_RANGE)
      	{
        	letter = F("[F");
      	}
      	else{
        	letter = F("[S");
      	}
      	Serial.println(letter + "X" + String(x) + "Y" + String(y) + "Z" + String(z) + "]");
      }
    }else

    //gMov-----------------------------------------------------------------------
    if(cmnd.indexOf(F("gMov")) >= 0){
      if(available()==false)
      {
        Serial.println(S);
      }
      else
      {
        Serial.println(F);
      }

        }else

    //gTip-----------------------------------------------------------------------
    if(cmnd.indexOf(F("gTip")) >= 0){
      if(digitalRead(LIMIT_SW))
      {
        Serial.println(S0);
      }
      else
      {
        Serial.println(S1);
      }
    }else
#ifdef PRODUCT_MKII
    //gPow-----------------------------------------------------------------------
    if(cmnd.indexOf(F("gPow")) >= 0){
      if(analogRead(POW_DET)>512)
        Serial.println(S);
      else
        Serial.println(F);
    }else
#endif
    //print err-------------------------------------------------------------------
    if(cmnd.length() > 0){
      Serial.println("[ERR3]");
    }
    else{
      Serial.println("");
    }
    if(errorResponse.length() > 0){// print the err infos
    	Serial.println(errorResponse);
    }
}

String uArmClass::getValues(String cmnd, String parameters[], int parameterCount, double *valueArray){
  int index[parameterCount];
  String errorMissingParameter = F("[ERR1]");
  String errorMissingValue     = F("[ERR2]");

  for(int p = 0; p < parameterCount; p++){
      index[p] = cmnd.indexOf(parameters[p]);
      if(index[p] == -1){return errorMissingParameter;}
  }
  
  //  Check that there is something between each parameter (AKA, the value)
  for(int p = 0; p < parameterCount; p++){   
    if(p < parameterCount - 1){
      if((index[p + 1] - index[p]) == 1){
        return errorMissingValue;
      }
      //Serial.println(index[p]);
      //Serial.println(cmnd.substring(index[p] + 1, index[p + 1]));
      valueArray[p] = cmnd.substring(index[p] + 1, index[p + 1]).toFloat();
      //Serial.println(valueArray[p]);
    }else{ 
      if(index[p] == cmnd.length() - 1){
        return errorMissingValue;
      }
      valueArray[p] = cmnd.substring(index[p] + 1).toFloat();
    }
  }
  //Serial.println(valueArray[0]);
  //Serial.println(valueArray[1]);
  //Serial.println(valueArray[2]);
  //Serial.println(valueArray[3]);
  return F("");
}


//*************************************private functions***************************************//
//**just used by the 512k external eeprom**//

void uArmClass::delay_us(){}

void uArmClass::iic_start()
{
#ifdef PRODUCT_MKII
  PORTB |= 0x01;// SCL=1
  delay_us();
  PORTB |= 0x02;// SDA=1
  delay_us();
  PORTB &= 0xFD;// SDA=0
  delay_us();
  PORTB &= 0xFE;// SCL=0
  delay_us();
#else
  PORTC |= 0x20;//  SCL=1
  delay_us();
  PORTC |= 0x10;//  SDA=1
  delay_us();
  PORTC &= 0xEF;//  SDA=0
  delay_us();
  PORTC &= 0xDF;//  SCL=0
  delay_us();
#endif
}

void uArmClass::iic_stop()
{
#ifdef PRODUCT_MKII
  PORTB &= 0xFE;// SCL=0
  delay_us();
  PORTB &= 0xFD;// SDA=0
  delay_us();
  PORTB |= 0x01;// SCL=1
  delay_us();
  PORTB |= 0x02;// SDA=1
  delay_us();
#else
  PORTC &= 0xDF;//  SCL=0
  delay_us();
  PORTC &= 0xEF;//  SDA=0
  delay_us();
  PORTC |= 0x20;//  SCL=1
  delay_us();
  PORTC |= 0x10;//  SDA=1
  delay_us();
#endif
}

//return 0:ACK=0
//return 1:NACK=1
unsigned char uArmClass::read_ack()
{
  unsigned char old_state;
#ifdef PRODUCT_MKII
  old_state = DDRB;
  DDRB = DDRB & 0xFD;//SDA INPUT
  PORTB |= 0x02;// SDA=1
  delay_us();
  PORTB |= 0x01;// SCL=1
  delay_us();
  if((PINB&0x02) == 0x02) // if(SDA)
  {
    PORTB &= 0xFE;// SCL=0
    iic_stop();
    return 1;
  }
  else{
    PORTB &= 0xFE;// SCL=0
    DDRB = old_state;
    return 0;
  }
#else
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
  else{
    PORTC &= 0xDF;//  SCL=0
    DDRC = old_state;
    return 0;
  }
#endif
}

//ack=0:send ack
//ack=1:do not send ack
void uArmClass::send_ack()
{
  unsigned char old_state;
#ifdef PRODUCT_MKII
  old_state = DDRB;
  DDRB = DDRB | 0x02;//SDA OUTPUT
  PORTB &= 0xFD;// SDA=0
  delay_us();
  PORTB |= 0x01;// SCL=1
  delay_us();
  PORTB &= 0xFE;// SCL=0
  delay_us();
  DDRB = old_state;
  PORTB |= 0x02;// SDA=1
  delay_us();
#else
  old_state = DDRC;
  DDRC = DDRC | 0x10;//SDA OUTPUT
  PORTC &= 0xEF;//  SDA=0
  delay_us();
  PORTC |= 0x20;//  SCL=1
  delay_us();
  PORTC &= 0xDF;//  SCL=0
  delay_us();
  DDRC = old_state;
  PORTC |= 0x10;//  SDA=1
  delay_us();
#endif
}

void uArmClass::iic_sendbyte(unsigned char dat)
{
  unsigned char i;
  for(i = 0;i < 8;i++)
  {
#ifdef PRODUCT_MKII
    if(dat & 0x80)
      PORTB |= 0x02;// SDA=1
    else
      PORTB &= 0xFD;// SDA=0
    dat <<= 1;
    delay_us();
    PORTB |= 0x01;// SCL=1
    delay_us();
    PORTB &= 0xFE;// SCL=0
#else
    if(dat & 0x80)
      PORTC |= 0x10;//  SDA = 1;
    else
      PORTC &= 0xEF;//  SDA = 0;
    dat <<= 1;
    delay_us();
    PORTC |= 0x20;//  SCL=1
    delay_us();
    PORTC &= 0xDF;//  SCL=0
#endif
  }
}

unsigned char uArmClass::iic_receivebyte()
{
  unsigned char i,byte = 0;
  unsigned char old_state;
#ifdef PRODUCT_MKII
  old_state = DDRB;
  DDRB = DDRB & 0xFD;//SDA INPUT
#else
  old_state = DDRC;
  DDRC = DDRC & 0xEF;//SDA INPUT
#endif
  for(i = 0;i < 8;i++)
  {
#ifdef PRODUCT_MKII
    PORTB |= 0x01;// SCL=1
    delay_us();
    byte <<= 1;
    if((PINB&0x02) == 0x02) // if(SDA)
      byte |= 0x01;
    delay_us();
    PORTB &= 0xFE;// SCL=0
    DDRB = old_state;
    delay_us();
#else
    PORTC |= 0x20;//  SCL=1
    delay_us();
    byte <<= 1;
    if((PINC&0x10) == 0x10) // if(SDA)
      byte |= 0x01;
    delay_us();
    PORTC &= 0xDF;//  SCL=0
    DDRC = old_state;
    delay_us();
#endif
  }
  return byte;
}

unsigned char uArmClass::iic_writebuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len)
{
#ifdef PRODUCT_MKII
  DDRB = DDRB | 0x03;
  PORTB = PORTB | 0x03;
#else
  DDRC = DDRC | 0x30;
  PORTC = PORTC | 0x30;
#endif
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
#ifdef PRODUCT_MKII
  DDRB = DDRB | 0x03;
  PORTB = PORTB | 0x03;
#else
  DDRC = DDRC | 0x30;
  PORTC = PORTC | 0x30;
#endif
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

                len--;
                if(len != 0) {
                        send_ack();
                }
        }
        iic_stop();
        return 0;
}

// #ifdef LATEST_HARDWARE
bool uArmClass::play()
{
        unsigned char data[5]; // 0: L  1: R  2: Rotation 3: hand rotation 4:gripper
        recording_read(addr, data, 5);
        //Serial.print("	LEFT ");
        //Serial.print(data[0]);
        //Serial.print("	RIGHT ");
        //Serial.print(data[1]);
        //Serial.print("	ROT ");
        //Serial.print(data[2]);
        //Serial.print("	data[3] ");
        //Serial.print(data[3]);
        //Serial.print("	data[4] ");
        //Serial.println(data[4]);
        if(data[0]!=255)
        {
#ifdef PRODUCT_MKII
		write_servos_angle((double)data[2], (double)data[0], (double)data[1]);
#else
                write_servo_angle((double)data[2] - ROT_SERVO_OFFSET, (double)data[0] - LEFT_SERVO_OFFSET, (double)data[1] - RIGHT_SERVO_OFFSET);
#endif
        }
        else
        {
                return false;
        }

        addr += 5;
        return true;
}

bool uArmClass::record()
{
        if(addr <= 65530)
        {
                unsigned char data[5]; // 0: L  1: R  2: Rotation 3: hand rotation 4:gripper
                if((addr != 65530)&&(sys_status != LEARNING_MODE_STOP))
                {
#ifdef PRODUCT_MKII
      read_servo_angle(SERVO_ROT_NUM, true);
      read_servo_angle(SERVO_LEFT_NUM, true);
      read_servo_angle(SERVO_RIGHT_NUM, true);
#else
                        get_current_rotleftright();
#endif
                        data[0] = (unsigned char)cur_left;
                        data[1] = (unsigned char)cur_right;
                        data[2] = (unsigned char)cur_rot;
                        //data[3] = (unsigned char)cur_hand;
#ifdef	PRODUCT_MKII
                        data[4] = (digitalRead(PUMP_GRI_EN) == LOW) ? 0x00 : 0x01;//get the gri and pump status - deleted since NO PUMP_GRI_EN - John Feng
#endif

                }
                else
                {
                        data[0] = 255;//255 is the ending flag
                        //Serial.println(data[0]);
                        recording_write(addr, data, 5);
                        /*if(sys_status == LEARNING_MODE_STOP)//LEARNING_MODE_STOP is just used to notificate record() function to stop, once record() get it then change the sys_status to normal_mode
                           {
                           sys_status = NORMAL_MODE;
                           }*/
                        return false;
                }
                recording_write(addr, data, 5);
                addr += 5;
                return true;
        }
        else
        {
                return false;
        }

}

void uArmClass::recording_write(unsigned int address, unsigned char * data_array, int num)
{
        unsigned char i=0;
        i=(address%128);
        // Since the eeprom's sector is 128 byte, if we want to write 5 bytes per cycle we need to care about when there's less than 5 bytes left
        if((i>=124)&&(num==5))
        {
                i=128-i;
                iic_writebuf(data_array, EXTERNAL_EEPROM_USER_ADDRESS, address, i);// write data
                delay(5);
                iic_writebuf(data_array + i, EXTERNAL_EEPROM_USER_ADDRESS, address + i, num - i);// write data
        }
        //if the left bytes are greater than 5, just do it
        else
        {
                iic_writebuf(data_array, EXTERNAL_EEPROM_USER_ADDRESS, address, num);// write data
        }
}

void uArmClass::recording_read(unsigned int address, unsigned char * data_array, int num)
{
        unsigned char i=0;
        i=(address%128);
        // Since the eeprom's sector is 128 byte, if we want to write 5 bytes per cycle we need to care about when there's less than 5 bytes left
        if((i>=124)&&(num==5))
        {
                i=128-i;
                iic_readbuf(data_array, EXTERNAL_EEPROM_USER_ADDRESS, address, i);// write data
                delay(5);
                iic_readbuf(data_array + i, EXTERNAL_EEPROM_USER_ADDRESS, address + i, num - i);// write data
        }
        //if the left bytes are greater than 5, just do it
        else
        {
                iic_readbuf(data_array, EXTERNAL_EEPROM_USER_ADDRESS, address, num);// write data
        }
}
//#endif
//*************************************end***************************************//
