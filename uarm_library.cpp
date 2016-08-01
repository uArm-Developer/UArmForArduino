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
  /*TCNT2   = 0;
  TCCR2A = (1 << WGM21); // Configure timer 2 for CTC mode
  TCCR2B = (1 << CS22); // Start timer at Fcpu/64
  TIMSK2 = (1 << OCIE2A); // Enable CTC interrupt
  OCR2A   = 249; // Set CTC compare value with a prescaler of 64  2ms*/
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
  if(Serial.available())
  {
    message = Serial.readStringUntil(']') + ']';
    Serial.println(runCommand(message));         // Run the command and send back the response
  }

  //move_to
  if(move_times!=255)
  {
    
    //if(move_times <= INTERP_INTVLS)
    if((millis() - moveStartTime) >= (move_times * microMoveTime))// detect if it's time to move
    {
     y_array[move_times] = y_array[move_times] - LEFT_SERVO_OFFSET;  //assembling offset
     z_array[move_times] = z_array[move_times] - RIGHT_SERVO_OFFSET; //assembling offset
     x_array[move_times] = x_array[move_times] - ROT_SERVO_OFFSET;   //rot offset

      read_servo_calibration_data(&x_array[move_times], &y_array[move_times], &z_array[move_times]);
      write_servos_angle(x_array[move_times], y_array[move_times], z_array[move_times]);

      //hand rot as hand rot do not have the smooth array
      if(move_times == (INTERP_INTVLS / 4))
      {
		    write_servo_angle(SERVO_HAND_ROT_NUM, hand_rot);
      }
      
      move_times++;
      if(move_times > INTERP_INTVLS)
      {
        move_times = 255;//disable the move

      }
    }
  }

  //buzzer work
  if(buzzerStopTime != 0)
  {
    if(millis() >= buzzerStopTime)
    {
      noTone(BUZZER);
      buzzerStopTime = 0;
    }
  }

}

void uArmClass::arm_setup()
{
  /*TCNT0   = 0;
  TCCR0A = (1 << WGM01); // Configure timer 0 for CTC mode
  TCCR0B = 0x03; // Start timer at Fcpu/64
  TIMSK0 = (1 << OCIE0A); // Enable CTC interrupt
  OCR0A   = 249; // Set CTC compare value with a prescaler of 64  1ms*/
  pinMode(BTN_D4,INPUT_PULLUP);//special mode for calibration
  if(digitalRead(4)==LOW)
  {
    while(digitalRead(4)==LOW);
        
    write_servos_angle(90,90,0);
    while(1);
  }

  pinMode(LIMIT_SW, INPUT_PULLUP);
  pinMode(BTN_D7, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
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
  iic_readbuf(calibration_data, EXTERNAL_EEPROM_DEVICE_ADDRESS, address + min_data_calibration_address, DATA_LENGTH);
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
        write_servo_angle(SERVO_HAND_ROT_NUM,servo_hand_rot_angle);
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

        write_servo_angle(SERVO_ROT_NUM,servo_rot_angle);
        write_servo_angle(SERVO_LEFT_NUM,servo_left_angle);
        write_servo_angle(SERVO_RIGHT_NUM,servo_right_angle);

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
        attach_servo(servo_number);
        //servo_angle = writeWithoffset ? (servo_angle + read_servo_offset(servo_number)) : servo_angle;
        // = constrain(servo_angle,0.0,180.0);
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
        		    g_servo_hand_rot.write(servo_angle ,hand_speed);//set the hand speed
                cur_hand = servo_angle;
                break;
        default:break;
        }
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
  double angleBefore;
  switch(servo_number) {
    case SERVO_ROT_NUM:
      if(!g_servo_rot.attached()) {
        read_servo_angle(SERVO_ROT_NUM);
        angleBefore = cur_rot;
        g_servo_rot.attach(SERVO_ROT_PIN);
        write_servo_angle(SERVO_ROT_NUM, angleBefore);
      }
      break;
    case SERVO_LEFT_NUM:
      if (!g_servo_left.attached()) {
        read_servo_angle(SERVO_LEFT_NUM);
        angleBefore = cur_left;
        g_servo_left.attach(SERVO_LEFT_PIN);
        write_servo_angle(SERVO_LEFT_NUM, angleBefore);
      }
      break;
    case SERVO_RIGHT_NUM:
      if (!g_servo_right.attached()) {
        read_servo_angle(SERVO_RIGHT_NUM);
        angleBefore = cur_right;
        g_servo_right.attach(SERVO_RIGHT_PIN);
        write_servo_angle(SERVO_RIGHT_NUM, angleBefore);
      }
      break;
    case SERVO_HAND_ROT_NUM:
      if (!g_servo_hand_rot.attached()) {
        read_servo_angle(SERVO_HAND_ROT_NUM);
        angleBefore = cur_hand;
        //Serial.println("Angle: " + String(cur_hand));
        g_servo_hand_rot.attach(SERVO_HAND_PIN);

        write_servo_angle(SERVO_HAND_ROT_NUM, angleBefore);
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
/*double uArmClass::read_servo_offset(byte servo_num)
{
        double manual_servo_offset = 0.0f;
        EEPROM.get(MANUAL_OFFSET_ADDRESS + servo_num * sizeof(manual_servo_offset), manual_servo_offset);
        return manual_servo_offset;
}*/

/*!
   \brief Convert the Analog to Servo Angle, by this formatter, angle = intercept + slope * analog
   \param input_analog Analog Value
   \param servo_num SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \param withOffset true, false
   \return Return Servo Angle
 */
double uArmClass::analog_to_angle(int input_analog, byte servo_num)
{
  unsigned char adc_calibration_data[DATA_LENGTH],data[4]; //get the calibration data around the data input
  unsigned int min_data_calibration_address, max_calibration_data, min_calibration_data;
  unsigned int angle_range_min, angle_range_max;
  switch(servo_num)
  {
    case  SERVO_ROT_NUM:      iic_readbuf(&data[0], EXTERNAL_EEPROM_DEVICE_ADDRESS, ROT_SERVO_ADDRESS + 360, 2);//get the min adc calibration data for the map() function
                              iic_readbuf(&data[2], EXTERNAL_EEPROM_DEVICE_ADDRESS, ROT_SERVO_ADDRESS + 360 + 358, 2);//get the max adc calibraiton data for the map() function
                              break;
    case  SERVO_LEFT_NUM:     iic_readbuf(&data[0], EXTERNAL_EEPROM_DEVICE_ADDRESS, LEFT_SERVO_ADDRESS + 360, 2);//get the min adc calibration data for the map() function
                              iic_readbuf(&data[2], EXTERNAL_EEPROM_DEVICE_ADDRESS, LEFT_SERVO_ADDRESS + 360 + 358, 2);//get the max adc calibraiton data for the map() function
                              break;
    case  SERVO_RIGHT_NUM:    iic_readbuf(&data[0], EXTERNAL_EEPROM_DEVICE_ADDRESS, RIGHT_SERVO_ADDRESS + 360, 2);//get the min adc calibration data for the map() function
                              iic_readbuf(&data[2], EXTERNAL_EEPROM_DEVICE_ADDRESS, RIGHT_SERVO_ADDRESS + 360 + 358, 2);//get the max adc calibraiton data for the map() function
                              break;
    default:                  break;
  }

  max_calibration_data = (data[2]<<8) + data[3];
  min_calibration_data = (data[0]<<8) + data[1];
  //Serial.println("++++++++++++++++++++++");
  //Serial.println(max_calibration_data,DEC);
  //Serial.println(min_calibration_data,DEC);

  angle_range_min = map(input_analog, min_calibration_data, max_calibration_data, 1, 180) - (DATA_LENGTH>>2);
  min_data_calibration_address = (angle_range_min * 2);
  switch(servo_num)
  {
    case  SERVO_ROT_NUM:      iic_readbuf(adc_calibration_data, EXTERNAL_EEPROM_DEVICE_ADDRESS, ROT_SERVO_ADDRESS + min_data_calibration_address + 360, DATA_LENGTH);//360 means the adc calibration data offset
                              break;
    case  SERVO_LEFT_NUM:     iic_readbuf(adc_calibration_data, EXTERNAL_EEPROM_DEVICE_ADDRESS, LEFT_SERVO_ADDRESS + min_data_calibration_address + 360, DATA_LENGTH);//360 means the adc calibration data offset
                              break;
    case  SERVO_RIGHT_NUM:    iic_readbuf(adc_calibration_data, EXTERNAL_EEPROM_DEVICE_ADDRESS, RIGHT_SERVO_ADDRESS + min_data_calibration_address + 360, DATA_LENGTH);//360 means the adc calibration data offset
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
  //Serial.println(angle_range_min,DEC);
  //Serial.println(angle_range_max,DEC);
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
unsigned char uArmClass::coordinate_to_angle(double x, double y, double z, double *theta_1, double *theta_2, double *theta_3)//theta_1:rotation angle   theta_2:the angle of lower arm and horizon   theta_3:the angle of upper arm and horizon
{
  double x_in = 0.0;
  double z_in = 0.0;
  double right_all = 0.0;
  double sqrt_z_x = 0.0;
  double phi = 0.0;

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

/*!
   \brief get the current rot left right angles
 */
void uArmClass::get_current_rotleftright()
{
read_servo_angle(SERVO_ROT_NUM);
read_servo_angle(SERVO_LEFT_NUM);
read_servo_angle(SERVO_RIGHT_NUM);
}

void uArmClass::read_servo_angle(byte servo_number) //double *data, unsigned int address)
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
      cur_hand = uarm.analog_to_angle(analogRead(SERVO_HAND_ROT_ANALOG_PIN), SERVO_HAND_ROT_NUM); //g_servo_hand_rot.read();  // SERVO_HAND_ROT_ANALOG_PIN),SERVO_HAND_ROT_NUM);
      return;
      break;
  }


  unsigned int dat[8], temp;
  unsigned char i=0,j=0;
  for(i=0;i<8;i++){
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
  //check the external eeprom and transfer the data to ideal angle
  unsigned char ideal_angle[4];
  iic_readbuf(ideal_angle, EXTERNAL_EEPROM_DEVICE_ADDRESS, address + (((unsigned int)(*data) - 1) << 1), 4);
  (*data) = (double)(((ideal_angle[2] << 8) + ideal_angle[3]) - ((ideal_angle[0] << 8) + ideal_angle[1])) * ((*data) - (unsigned int)(*data)) + ((ideal_angle[0] << 8) + ideal_angle[1]);
  (*data) = (*data) / 10.0;
}

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
  *cur_left = *cur_left + LEFT_SERVO_OFFSET;
  *cur_right = *cur_right + RIGHT_SERVO_OFFSET;

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
  if(polar == true)//change the stretch rot and height to xyz coordinates
  {
  	double stretch = x;
  	//Z and height is the same
  	//transfer stretch to xy
  	x = stretch * cos(y / MATH_TRANS);
  	y = stretch * sin(y / MATH_TRANS);
  }


  // get current angles of servos

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
  //INTERP_INTVLS = INTERP_INTVLS * (10 / times);// speed determine the number of interpolation
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
  //Serial.println(microMoveTime,DEC);

	g_current_x = x;
	g_current_y = y;
	g_current_z = z;
	cur_rot = rot;
	cur_left = left;
	cur_right = right;
	cur_hand = hand_angle;
	hand_rot = hand_angle;
    move_times = 0;//start to caculate the movement
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

/*!
  \brief systick
*/

/*ISR(TIMER0_COMPA_vect) 
{ 
  sys_tick++;
  PORTB ^= 0x01;
}*/

//*************************************uart communication**************************************//
String uArmClass::runCommand(String cmnd){
    
    // To save memory, create the "[OK" and "]\n" right now, in flash memory
  String S   = F("[S]"); 
 	String S0  = F("[S0]");
 	String S1  = F("[S1]");
 	String S2  = F("[S2]");
 	String F   = F("[F]");
 	String F0  = F("[F0]");
 	String F1  = F("[F1]");

    // sMov Command----------------------------------------------------------
    if(cmnd.indexOf(F("sMov")) >= 0){
      String parameters[] = {F("X"), F("Y"), F("Z"), F("V")};
      double values[4];
      String errorResponse = getValues(cmnd, parameters, 4, values);
      if(errorResponse.length() > 0) {return errorResponse;}

      //limit the speed
      move_to_the_closest_point = true; 
      /*if(move_to(values[0], values[1], values[2], values[3], false)!=IN_RANGE)
      {
        move_to_the_closest_point = false; //stop the move_to_the_closest_point function to avoid other problems
      	return F;
      }*/
      move_to(values[0], values[1], values[2], values[3], false);
      move_to_the_closest_point = false; 
      return S;
            
    }else

    //sPolS#H#R#--------------------------------------------------------------
    if(cmnd.indexOf(F("sPol")) >= 0){
      String parameters[] = {F("S"), F("R"), F("H"), F("V")};
      double values[4];
      String errorResponse = getValues(cmnd, parameters, 4, values);
      if(errorResponse.length() > 0) {return errorResponse;}

      //limit the speed
      move_to_the_closest_point = true;

      /*if(move_to(values[0], values[1], values[2], values[3], true)!=IN_RANGE)
      {
        move_to_the_closest_point = false; 
        return F;
      }*/
      move_to(values[0], values[1], values[2], values[3], true);
      move_to_the_closest_point = false; 
      return S;

    }else

    //gPolS#R#H#--------------------------------------------------------------
    if(cmnd.indexOf(F("gPol")) >= 0){
      get_current_xyz(&cur_rot, &cur_left, &cur_right, &g_current_x, &g_current_y, &g_current_z, true);
      double stretch;
      stretch = sqrt(g_current_x * g_current_x + g_current_y * g_current_y);
      return "[SS" + String(stretch) + " R" + String(cur_rot) + " H" + String(g_current_z) + "]";

    }else

    //gSimuX#Y#Z#-------------------------------------------------------------
    if(cmnd.indexOf(F("gSim")) >= 0){
      String parameters[] = {F("X"), F("Y"), F("Z")};
      double values[3];
      String errorResponse = getValues(cmnd, parameters, 3, values);
      if(errorResponse.length() > 0) {return errorResponse;}

      bool polar;
      move_to_the_closest_point = false;//make sure move_to_the_closest_point is false so that we can get the out_of_range feedback
      if(values[3]==1)
      	polar = true;
      else
      	polar = false;
      switch(move_to(values[0], values[1], values[2], polar))
      {
        case IN_RANGE             :move_times=255;//disable move
                                  return S0;
                                  break;
        case OUT_OF_RANGE_IN_PATH :move_times=255;//disable move
                                  return F0;
                                  break;
        case OUT_OF_RANGE_IN_DST  :move_times=255;//disable move
                                  return F1;
                                  break;
        default:break;
      }

    }else

    //gVer---------------------------------------------------------------------
    if(cmnd.indexOf(F("gVer")) >= 0){
      return "[S" + String(current_ver) + "]";

    }else

    // sServoN#V#--------------------------------------------------------------
    if(cmnd.indexOf(F("sSer")) >= 0)
    {

      String servoSetParameters[] = {F("N"), F("V")};
      double values[2];
      String errorResponse = getValues(cmnd, servoSetParameters, 2, values);
      if(errorResponse.length() > 0) {return errorResponse;}

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
       return S;
    }else

    //sPumpV#------------------------------------------------------------------
    if(cmnd.indexOf(F("sPum")) >= 0){
       String parameters[] = {F("V")};
       double values[1];
       String errorResponse        = getValues(cmnd, parameters, 1, values);
       if(errorResponse.length() > 0) {return errorResponse;}


       if(values[0]==0)//off
       {
        pump_off();
       }else//on
       {
        pump_on();
       }
       return S;
    }else
    
    //gPump---------------------------------------------------------------------
    if(cmnd.indexOf(F("gPum")) >= 0){
      return S0;//return S1;return S2;
    }else

    //sGripperV#----------------------------------------------------------------
    if(cmnd.indexOf(F("sGri")) >= 0){
       String parameters[] = {F("V")};
       double values[1];
       String errorResponse        = getValues(cmnd, parameters, 1, values);
       if(errorResponse.length() > 0) {return errorResponse;}

       if(values[0]==0)//release
       {
        gripper_release();
       }else//catch
       {
        gripper_catch();
       }
       return S;
    }else

    //gGipper-------------------------------------------------------------------
    if(cmnd.indexOf(F("gGri")) >= 0){
      return S0;//return S1;return S2;
    }else

    // sAttachS#----------------------------------------------------------------
    if(cmnd.indexOf(F("sAtt")) >= 0){
      String parameters[] = {F("S")};
      double values[1];
      String errorResponse        = getValues(cmnd, parameters, 1, values);
      if(errorResponse.length() > 0) {return errorResponse;}

      attach_servo(values[0]);
      return S;
    }else

    
    // sDetachS#----------------------------------------------------------------
    if(cmnd.indexOf(F("sDet")) >= 0){
      String parameters[] = {F("S")};
      double values[1];
      String errorResponse        = getValues(cmnd, parameters, 1, values);
      if(errorResponse.length() > 0) {return errorResponse;}

      detach_servo(values[0]);
      return S;
    }else

    //gCrd---------------------------------------------------------------------
    if(cmnd.indexOf(F("gCrd")) >= 0){
      get_current_xyz(&cur_rot, &cur_left, &cur_right, &g_current_x, &g_current_y, &g_current_z, true);
      return "[SX" + String(g_current_x) + "Y" + String(g_current_y) + "Z" + String(g_current_z) + "]";
    }else

    //gAng---------------------------------------------------------------------
    if(cmnd.indexOf(F("gAng")) >= 0){
      get_current_rotleftright();
      return "[ST" + String(cur_rot) + "L" + String(cur_left) + "R" + String(cur_right) + "]";
    }else

    //gIKX#Y#Z#----------------------------------------------------------------
    if(cmnd.indexOf(F("gIK")) >= 0){
      String parameters[] = {F("X"), F("Y"), F("Z")};
      double values[3];
      String errorResponse = getValues(cmnd, parameters, 3, values);
      if(errorResponse.length() > 0) {return errorResponse;}

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
      return  letter + "T" + String(rot) + "L" + String(left) + "R" + String(right) + "]";
    }else

    //gFKT#L#R#-----------------------------------------------------------------
    // Get Forward Kinematics
    if(cmnd.indexOf(F("gFK")) >= 0){
      String parameters[] = {F("T"), F("L"), F("R")};
      double values[3];
      String errorResponse = getValues(cmnd, parameters, 3, values);
      if(errorResponse.length() > 0) {return errorResponse;}

      double x, y, z;
      String letter;
      if(get_current_xyz(&values[0], &values[1], &values[2], &x, &y, &z, false) == OUT_OF_RANGE)
      {
        letter = F("[F");
      }
      else{
        letter = F("[S");
      }
      return letter + "X" + String(x) + "Y" + String(y) + "Z" + String(z) + "]";
    }else

    
    //gMov-----------------------------------------------------------------------
    if(cmnd.indexOf(F("gMov")) >= 0){
      if(available()==false)
      {
        return S;
      }
      else
      {
        return F;
      }

    }else
    
    //gTip-----------------------------------------------------------------------
    if(cmnd.indexOf(F("gTip")) >= 0){
      if(digitalRead(LIMIT_SW))
      {
        return S0;
      }
      else
      {
        return S1;
      }
    }else

    //sBuzzF#T#-------------------------------------------------------------------
    if(cmnd.indexOf(F("sBuz")) >= 0){
      String parameters[] = { F("F"),F("T")};
      double values[2];
      String errorResponse        = getValues(cmnd, parameters, 2, values);
      if(errorResponse.length() > 0) {return errorResponse;}

      tone(BUZZER, values[0]);
      buzzerStopTime = millis() + int(values[1] * 1000.0); //sys_tick + values[1];
      //Serial.println(buzzerStopTime);
      return S;
    }
    
    
    if(cmnd.length() > 0){

      return "[ERR3]";

    }else{
      return F("");
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
      valueArray[p] = cmnd.substring(index[p] + 1, index[p + 1]).toFloat();
    }else{ 
      if(index[p] == cmnd.length() - 1){
        return errorMissingValue;
      }
      valueArray[p] = cmnd.substring(index[p] + 1).toFloat();
    }
    
  }
  
  return F("");
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