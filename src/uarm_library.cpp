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

void uArmClass::init()
{
        if (EEPROM.read(CALIBRATION_FLAG) != CONFIRM_FLAG)
        {
                alert(50, 10, 10);
        }
        set_servo_status(true, SERVO_ROT_NUM);
        set_servo_status(true, SERVO_LEFT_NUM);
        set_servo_status(true, SERVO_RIGHT_NUM);
        set_servo_status(true, SERVO_HAND_ROT_NUM);
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
   \brief Write 4 Servo Angles, servo_rot, servo_left, servo_right, servo_hand_rot
   \param servo_rot_angle SERVO_ROT_NUM
   \param servo_left_angle SERVO_LEFT_NUM
   \param servo_right_angle SERVO_RIGHT_NUM
   \param servo_hand_rot_angle SERVO_HAND_ROT_NUM
   \return SUCCESS, FAILED
 */
int uArmClass::write_servo_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle, double servo_hand_rot_angle)
{
        // attach_all();
        write_servo_angle(servo_rot_angle, servo_left_angle, servo_right_angle);
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
int uArmClass::write_servo_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle)
{
        if(servo_left_angle < 10) servo_left_angle = 10;
        if(servo_left_angle > 120) servo_left_angle = 120;
        if(servo_right_angle < 10) servo_right_angle = 10;
        if(servo_right_angle > 110) servo_right_angle = 110;

        if(servo_left_angle + servo_right_angle > 160)
        {
                servo_right_angle = 160 - servo_left_angle;
                return FAILED;
        }
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
        // attach_servo(servo_number);
        servo_angle = writeWithoffset ? (servo_angle + read_servo_offset(servo_number)) : servo_angle;
        servo_angle = constrain(servo_angle,0.0,180.0);
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
        g_servo_left.write(servo_left_angle);
        g_servo_right.write(servo_right_angle);
}

/*!
   \brief Attach Servo, if servo has not been attached, attach the servo, and read the current Angle
   \param servo number SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
 */
boolean uArmClass::set_servo_status(boolean setAttached, byte servoNum){
        // Attach or detach a servo, and set the position instantly after doing so to prevent "snap"
        // Returns true or false if the servo was a valid number
        // Serial.println("Starting new func");
        boolean is_linear_calibrated = false;
        if (EEPROM.read(CALIBRATION_LINEAR_FLAG) == CONFIRM_FLAG)
        {
                is_linear_calibrated = true;
        }
        double angleBefore = 90;
        if(servoNum == SERVO_ROT_NUM) {
                if(setAttached) {
                        if (is_linear_calibrated == true) {
                                angleBefore = read_servo_angle(SERVO_ROT_NUM);
                                // Serial.println("New angle" + String(angleBefore));
                                uarm.g_servo_rot.attach(SERVO_ROT_PIN);
                                uarm.g_servo_rot.write(angleBefore);
                                cur_rot = angleBefore;
                        }
                        else{
                                uarm.g_servo_rot.attach(SERVO_ROT_PIN);
                                uarm.g_servo_rot.write(90);
                        }
                }else{
                        uarm.g_servo_rot.detach();
                }
        }else if(servoNum == SERVO_LEFT_NUM) {
                if(setAttached) {
                        if (is_linear_calibrated == true) {
                                angleBefore = uarm.read_servo_angle(SERVO_LEFT_NUM);
                                uarm.g_servo_left.attach(SERVO_LEFT_PIN);
                                uarm.g_servo_left.write(angleBefore);
                                cur_left = angleBefore;
                        }
                        else{
                                uarm.g_servo_left.attach(SERVO_LEFT_PIN);
                                uarm.g_servo_left.write(100);
                        }
                }else{
                        uarm.g_servo_left.detach();
                }
        }else if(servoNum == SERVO_RIGHT_NUM) {
                if(setAttached) {
                        if (is_linear_calibrated == true) {
                                angleBefore = uarm.read_servo_angle(SERVO_RIGHT_NUM);
                                uarm.g_servo_right.attach(SERVO_RIGHT_PIN);
                                uarm.g_servo_right.write(angleBefore);
                                cur_right = angleBefore;
                        }
                        else{
                                uarm.g_servo_right.attach(SERVO_RIGHT_PIN);
                                uarm.g_servo_right.write(60);
                        }
                }else{
                        uarm.g_servo_right.detach();
                }
        }else if(servoNum == SERVO_HAND_ROT_NUM) {
                if(setAttached) {
                        if (is_linear_calibrated == true) {
                                angleBefore = uarm.read_servo_angle(SERVO_HAND_ROT_NUM);
                                uarm.g_servo_hand_rot.attach(SERVO_HAND_PIN);
                                uarm.g_servo_hand_rot.write(angleBefore);
                                cur_hand = angleBefore;
                        }
                        else{
                                uarm.g_servo_hand_rot.attach(SERVO_HAND_PIN);
                                uarm.g_servo_hand_rot.write(90);
                        }
                }else{
                        uarm.g_servo_hand_rot.detach();
                }
        }else{
                return false;
        }
        return true;
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
        read_linear_offset(servo_num, intercept, slope);
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
void uArmClass::coordinate_to_angle(double x, double y, double z, double& theta_1, double& theta_2, double& theta_3)
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

        // get rid of divide by zero errors.
        // Because x is a double we need to check a range
        if (x <= 0.005 && x >= -0.005)
        {
                // Calculate value of theta 1
                theta_1 = 90;

                // Calculate value of theta 3
                if (z_in == 0) {
                        phi = 90;
                }

                else {
                        phi = atan(-y_in / z_in)*MATH_TRANS;
                }

                if (phi > 0) phi = phi - 180;

                theta_3 = asin(right_all / sqrt_z_y)*MATH_TRANS - phi;
                if(theta_3<0)
                {
                        theta_3 = 0;
                }

                // Calculate value of theta 2
                theta_2 = asin((z + MATH_L4*sin(theta_3 / MATH_TRANS) - MATH_L1) / MATH_L3)*MATH_TRANS;
        }
        else
        {
                // Calculate value of theta 1

                theta_1 = atan(y / x)*MATH_TRANS;

                if (y / x > 0) {
                        theta_1 = theta_1;
                }
                if (y / x < 0) {
                        theta_1 = theta_1 + 180;
                }
                if (y == 0) {
                        if (x > 0) theta_1 = 180;
                        else theta_1 = 0;
                }

                // Calculate value of theta 3

                x_in = (-x / cos(theta_1 / MATH_TRANS) - MATH_L2) / MATH_L3;

                if (z_in == 0) { phi = 90; }

                else{ phi = atan(-x_in / z_in)*MATH_TRANS; }

                if (phi > 0) {phi = phi - 180; }

                sqrt_z_x = sqrt(z_in*z_in + x_in*x_in);

                right_all_2 = -1 * (z_in*z_in + x_in*x_in + MATH_L43*MATH_L43 - 1) / (2 * MATH_L43);
                theta_3 = asin(right_all_2 / sqrt_z_x)*MATH_TRANS;
                theta_3 = theta_3 - phi;

                if (theta_1 <0 ) {
                        theta_1 = 0;
                }

                // Calculate value of theta 2
                theta_2 = asin(z_in + MATH_L43*sin(abs(theta_3 / MATH_TRANS)))*MATH_TRANS;
        }

        theta_1 = abs(theta_1);
        theta_2 = abs(theta_2);

        if (theta_3 < 0 ) {}
        else{
                if ((angle_to_coordinate_y(theta_1,theta_2, theta_3)>y+0.1)||(angle_to_coordinate_y(theta_1,theta_2, theta_3)<y-0.1))
                {
                        theta_2 = 180 - theta_2;
                }
        }

        if(isnan(theta_1)||isinf(theta_1))
        {theta_1 = uarm.read_servo_angle(SERVO_ROT_NUM); }
        if(isnan(theta_2)||isinf(theta_2))
        {theta_2 = uarm.read_servo_angle(SERVO_LEFT_NUM); }
        if(isnan(theta_3)||isinf(theta_3)||(theta_3<0))
        {theta_3 = uarm.read_servo_angle(SERVO_RIGHT_NUM); }
}

/*!
   \brief Write Sretch & Height.
   \description This is an old control method to uArm. Using uarm's Stretch and height, , Height from -180 to 150
   \param armStretch Stretch from 0 to 195
   \param armHeight Height from -150 to 150
 */
void uArmClass::write_stretch_height(double armStretch, double armHeight){
        double offsetL = 0;
        double offsetR = 0;
        if(EEPROM.read(CALIBRATION_STRETCH_FLAG) != CONFIRM_FLAG) {
                offsetL =  -10;
                offsetR =  -10;
        }
        else{
                EEPROM.get(OFFSET_STRETCH_START_ADDRESS, offsetL);
                EEPROM.get(OFFSET_STRETCH_START_ADDRESS + 4, offsetR);
        }
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
        write_left_right_servo_angle(angleL,angleR,true);
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
        double delta = end_val - start_val;
        for (byte f = 0; f < INTERP_INTVLS; f++) {
                switch (ease_type) {
                case INTERP_LINEAR:
                        *(interp_vals+f) = delta * f / INTERP_INTVLS + start_val;
                        break;
                case INTERP_EASE_INOUT:
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
                case INTERP_EASE_IN:
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
                break;
                case INTERP_EASE_INOUT_CUBIC: // this is a compact version of Joey's original cubic ease-in/out
                {
                        float t = (float)f / INTERP_INTVLS;
                        *(interp_vals+f) = start_val + (3 * delta) * (t * t) + (-2 * delta) * (t * t * t);
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
   \param path_type PATH_LINEAR, PATH_ANGLES
   \param enable_hand Enable Hand Axis
   \return SUCCESS, FAILED
 */
int uArmClass::move_to(double x, double y, double z, double hand_angle, byte relative_flags, double time, byte path_type, byte ease_type, boolean enable_hand) {
        float limit = sqrt((x*x + y*y));
        if (limit > 32)
        {
                float k = 32/limit;
                x = x * k;
                y = y * k;
        }
        // attach_all();
        // setServoStatus(true, SERVO_ROT_NUM);
        // setServoStatus(true, SERVO_LEFT_NUM);
        // setServoStatus(true, SERVO_RIGHT_NUM);
        // if(enable_hand)
        //         setServoStatus(true, SERVO_HAND_ROT_NUM);

        // find current position using cached servo values
        double current_x;
        double current_y;
        double current_z;
        angle_to_coordinate(cur_rot, cur_left, cur_right, current_x, current_y, current_z);


        // deal with relative xyz positioning
        byte posn_relative = (relative_flags & F_POSN_RELATIVE) ? 1 : 0;
        x = current_x * posn_relative + x;
        y = current_y * posn_relative + y;
        z = current_z * posn_relative + z;

        // find target angles
        double tgt_rot;
        double tgt_left;
        double tgt_right;
        coordinate_to_angle(x, y, z, tgt_rot, tgt_left, tgt_right);

        //calculate the length
        unsigned int delta_rot=abs(tgt_rot-cur_rot);
        unsigned int delta_left=abs(tgt_left-cur_left);
        unsigned int delta_right=abs(tgt_right-cur_right);

        //Serial.println(tgt_rot,DEC);

        INTERP_INTVLS = max(delta_rot,delta_left);
        INTERP_INTVLS = max(INTERP_INTVLS,delta_right);

        //Serial.println(INTERP_INTVLS,DEC);
        INTERP_INTVLS=(INTERP_INTVLS<80) ? INTERP_INTVLS : 80;

        // deal with relative hand orientation
        if (relative_flags & F_HAND_RELATIVE) {
                hand_angle += cur_hand;             // rotates a relative amount, ignoring base rotation
        } else if (relative_flags & F_HAND_ROT_REL) {
                hand_angle = hand_angle + cur_hand + (tgt_rot - cur_rot); // rotates relative to base servo, 0 value keeps an object aligned through movement
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
                                if(enable_hand)
                                        write_servo_angle(rot_array[i], left_array[i], right_array[i], hand_array[i]);
                                else
                                        write_servo_angle(rot_array[i], left_array[i], right_array[i]);
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
                                double rot,left, right;
                                coordinate_to_angle(x_array[i], y_array[i], z_array[i], rot, left, right);
                                if(enable_hand)
                                        write_servo_angle(rot, left, right, hand_array[i]);
                                else
                                        write_servo_angle(rot, left, right);
                                delay(time * 1000 / INTERP_INTVLS);
                        }
                }
        }

        // set final target position at end of interpolation or atOnce
        if (enable_hand)
                write_servo_angle(tgt_rot, tgt_left, tgt_right, hand_angle);
        else
                write_servo_angle(tgt_rot, tgt_left, tgt_right);
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
