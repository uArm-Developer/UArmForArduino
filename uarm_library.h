/*! ******************************************************************************
   \file uarm_library.h
   \brief uarm library header
   \author Joey Song
   \update Joey Song, Alex Tan, Dave Corboy
   \date 12/Dec/2014
   \License GNU
   \Copyright 2016 UFactory Team. All right reserved
* *******************************************************************************/
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "UFServo.h"
#include "linreg.h"

#ifndef uArm_library_h
#define uArm_library_h

// for the external eeprom 
#define EXTERNAL_EEPROM_DEVICE_ADDRESS  0xA0
#define DATA_LENGTH  0x20
#define LEFT_SERVO_ADDRESS   0x0000
#define RIGHT_SERVO_ADDRESS  0x02D0
#define ROT_SERVO_ADDRESS    0x05A0

#define RIGHT_SERVO_OFFSET    14.5//-1liebao   //1.8Degree
#define LEFT_SERVO_OFFSET     2.6//18.6liebao   //2.6Degree
#define ROT_SERVO_OFFSET     -0//-7liebao
//#define DEBUG_MODE 

#define current_ver         "0.9.4"

#define UARM_MAJOR_VERSION      1
#define UARM_MINOR_VERSION      6
#define UARM_BUGFIX             2

#define SERVO_ROT_NUM           0
#define SERVO_LEFT_NUM          1
#define SERVO_RIGHT_NUM         2
#define SERVO_HAND_ROT_NUM      3

#define SERVO_ROT_PIN           11
#define SERVO_LEFT_PIN          13
#define SERVO_RIGHT_PIN         12
#define SERVO_HAND_PIN          10

#define SERVO_ROT_ANALOG_PIN 2
#define SERVO_LEFT_ANALOG_PIN 0
#define SERVO_RIGHT_ANALOG_PIN 1
#define SERVO_HAND_ROT_ANALOG_PIN 3

// Old Control method Stretch / Height

#define ARM_STRETCH_MIN   0
#define ARM_STRETCH_MAX   280
#define ARM_HEIGHT_MIN   0
#define ARM_HEIGHT_MAX   200
#define L3_MAX_ANGLE     120
#define L3_MIN_ANGLE     5
#define L4_MAX_ANGLE     120
#define L4_MIN_ANGLE     5
#define L4L3_MAX_ANGLE     150
#define L4L3_MIN_ANGLE     30

#define LIMIT_SW                2    // LIMIT Switch Button
#define PUMP_EN                 6    // HIGH = Valve OPEN
#define VALVE_EN                5    // HIGH = Pump ON
#define BUZZER                  3    // HIGH = ON
#define BTN_D4                  4    // LOW = Pressed
#define BTN_D7                  7    // LOW = Pressed
#define GRIPPER                 9    // LOW = Catch


#define MATH_PI 3.141592653589793238463
#define MATH_TRANS  57.2958
#define MATH_L1 90.00
#define MATH_L2 21.17
#define MATH_L3 148.25
#define MATH_L4 160.2
#define MATH_L43 MATH_L4/MATH_L3

#define IN_RANGE             0
#define OUT_OF_RANGE_IN_DST  1
#define OUT_OF_RANGE_IN_PATH 2
#define OUT_OF_RANGE         3

#define RELATIVE 1
#define ABSOLUTE 0

#define TopOffset -1.5
#define BottomOffset 1.5

// Calibration Flag & OFFSET EEPROM ADDRESS
#define CALIBRATION_FLAG                    10
#define CALIBRATION_LINEAR_FLAG             11
#define CALIBRATION_MANUAL_FLAG             12
#define CALIBRATION_STRETCH_FLAG            13

#define LINEAR_INTERCEPT_START_ADDRESS      70
#define LINEAR_SLOPE_START_ADDRESS          50
#define MANUAL_OFFSET_ADDRESS               30
#define OFFSET_STRETCH_START_ADDRESS        20
#define SERIAL_NUMBER_ADDRESS               100

#define A

#define CONFIRM_FLAG                        0x80

// movement path types
#define PATH_LINEAR     0   // path based on linear interpolation
#define PATH_ANGLES     1   // path based on interpolation of servo angles

// movement absolute/relative flags
#define ABSOLUTE  0
#define RELATIVE  1
//#define F_HAND_RELATIVE 2   // standard relative, current + hand parameter
//#define F_HAND_ROT_REL  4   // hand keeps orientation relative to rotationxn servo (+/- hand parameter)


// interpolation types
#define INTERP_EASE_INOUT_CUBIC 0  // original cubic ease in/out
#define INTERP_LINEAR           1
#define INTERP_EASE_INOUT       2  // quadratic easing methods
#define INTERP_EASE_IN          3
#define INTERP_EASE_OUT         4

#define LINEAR_INTERCEPT        1
#define LINEAR_SLOPE            2

class uArmClass
{
public:
        uArmClass();
        void arm_setup();
        double read_servo_offset(byte servo_num);
        void read_servo_calibration_data(double *rot, double *left, double *right);
        void detach_servo(byte servo_num);
        void alert(byte times, byte runt_time, byte stop_time);
        void detach_all_servos();
        void write_servo_angle(byte servo_num, double servo_angle,  boolean with_offset);
        double read_servo_angle(byte servo_num);
        double analog_to_angle(int input_angle, byte servo_num);

        void arm_process_commands();
        bool available();

        unsigned char move_to(double x, double y, double z, double hand_angle, byte relative_flags, double time, byte ease_type, boolean enable_hand);
        unsigned char move_to(double x, double y,double z) {
                return move_to(x, y, z, 0, ABSOLUTE, 1.0, INTERP_EASE_INOUT_CUBIC, false);
                //return move_to(x, y, z, 0, ABSOLUTE, 1.0, INTERP_EASE_INOUT_CUBIC, false);
        }
        unsigned char move_to(double x, double y,double z,double hand_angle) {
                move_to(x, y, z, hand_angle, ABSOLUTE, 1.0, INTERP_EASE_INOUT_CUBIC, true);
        }
        /*void move_to(double x, double y,double z,int relative, double time) {
                move_to(x, y, z, 0, F_HAND_RELATIVE | (relative ? F_POSN_RELATIVE : 0), time, PATH_LINEAR, INTERP_EASE_INOUT_CUBIC, false);
        }
        void move_to(double x, double y,double z,int relative, double time, double servo_4_angle) {
                move_to(x, y, z, servo_4_angle, relative ? F_POSN_RELATIVE : 0, time, PATH_LINEAR, INTERP_EASE_INOUT_CUBIC, true);
        }
        void move_to(double x, double y, double z, int relative, double time, int servo_4_relative, double servo_4_angle) {
                move_to(x, y, z, servo_4_angle, (relative ? F_POSN_RELATIVE : 0) | (servo_4_relative ? F_HAND_RELATIVE : 0), time, PATH_LINEAR, INTERP_EASE_INOUT_CUBIC, true);
        }

        void move_to_at_once(double x, double y, double z) {
                move_to(x, y, z, 0, F_HAND_RELATIVE, 0.0, PATH_LINEAR, INTERP_LINEAR, false);
        }
        void move_to_at_once(double x, double y, double z, int relative, double servo_4_angle) {
                move_to(x, y, z, servo_4_angle, relative ? F_POSN_RELATIVE : 0, 0.0, PATH_LINEAR, INTERP_LINEAR, true);
        }*/

        void write_stretch_height(double stretch, double height);


        double get_current_x() {
                return g_current_x;
        }
        double get_current_y() {
                return g_current_y;
        }
        double get_current_z() {
                return g_current_z;
        }

        void get_current_xyz(double *cur_rot, double *cur_left, double *cur_right, double *g_current_x, double *g_current_y, double *g_current_z, bool for_movement );
        void get_current_rotleftright();
        void calibration_data_to_servo_angle(double *data,unsigned int address);
        void servo_angle_to_calibration_data(double *data,unsigned int address);
        /*void angle_to_coordinate(double& x, double& y, double &z) {
                get_current_xyz(); x = g_current_x; y = g_current_y; z = g_current_z;
        }
        void angle_to_coordinate(double theta_1, double theta_2, double theta_3, double& x, double& y, double &z) {
                get_current_xyz(theta_1, theta_2, theta_3); x = g_current_x; y = g_current_y; z = g_current_z;
        }*/
        unsigned char coordinate_to_angle(double x, double y, double z, double& theta_1, double& theta_2, double& theta_3);


        void gripper_catch();
        void gripper_release();
        void interpolate(double start_val, double end_val, double *interp_vals, byte ease_type);
        void pump_on();
        void pump_off();

        Servo g_servo_rot;
        Servo g_servo_left;
        Servo g_servo_right;
        Servo g_servo_hand_rot;
        Servo g_servo_hand;

        int write_servos_angle(byte servo_rot_angle, byte servo_left_angle, byte servo_right_angle, byte servo_hand_rot_angle, byte trigger);
        int write_servos_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle, double servo_hand_rot_angle);
        int write_servos_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle);
        void attach_all();
        void attach_servo(byte servo_num);
        String runCommand(String cmnd);
        void getCommandValues(String cmnd, String parameters[], int parameterCount, double *valueArray);
        String isValidCommand(String cmnd, String parameters[], int parameterCount);

private:
        void delay_us();
        void iic_start();
        void iic_stop();
        unsigned char read_ack();
        void send_ack();
        void iic_sendbyte(unsigned char dat);
        unsigned char iic_receivebyte();
        unsigned char iic_writebuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len);
        unsigned char iic_readbuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len);

protected:
        double cur_rot = 90;
        double cur_left = 90;
        double cur_right = 90;
        double cur_hand = 90;
        double angle_to_coordinate_y(double theta_1, double theta_2, double theta_3);

        double g_current_x = 0;
        double g_current_y = 200;
        double g_current_z = 100;

        boolean move_to_the_closest_point = false;

        boolean g_gripper_reset;
        unsigned int INTERP_INTVLS;

        unsigned char move_times = 255;//255 means no move
        int buzzerStopTime=0;
        // the arrays to store the xyz coordinates first and then change to the rot left right angles
        double x_array[61];
        double y_array[61];
        double z_array[61];
        //double hand_array[16];//to save the memory

};

extern uArmClass uarm;

#endif
