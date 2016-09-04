/*!
   \file uarm_library.h
   \brief uarm library header
   License GNU
   Copyright 2016 UFactory Team. All right reserved   
   \author Joey Song, Alex Tan, Dave Corboy
   \version MKII: H3-S2.0.9a  uArm Metal: H2-S2.0.9a
   \date 08/30/2016
*/
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "UFServo.h"
#include "linreg.h"

#ifndef uArm_library_h
#define uArm_library_h

//for the sys status
#define NORMAL_MODE                 0
#define NORMAL_BT_CONNECTED_MODE    1
#define LEARNING_MODE               2
#define SINGLE_PLAY_MODE            3
#define LOOP_PLAY_MODE              4

//for the record() function to stop recording
#define LEARNING_MODE_STOP          5 

// for the external eeprom
#ifdef PRODUCT_MKII
   #define EXTERNAL_EEPROM_SYS_ADDRESS 0xA2
   #define EXTERNAL_EEPROM_USER_ADDRESS 0xA0
#else
   // EEPROM for learning mode - John Feng
   #define EXTERNAL_EEPROM_USER_ADDRESS  0xA0 
#endif
#define DATA_LENGTH  0x20
#define LEFT_SERVO_ADDRESS   0x0000
#define RIGHT_SERVO_ADDRESS  0x02D0
#define ROT_SERVO_ADDRESS    0x05A0

#ifdef PRODUCT_MKII
	#define current_ver         "H3-S2.0.9a"
#else
   #define current_ver         "H2-S2.0.9a"
#endif

#define SERVO_ROT_NUM           0
#define SERVO_LEFT_NUM          1
#define SERVO_RIGHT_NUM         2
#define SERVO_HAND_ROT_NUM      3

#define SERVO_ROT_PIN           11
#define SERVO_LEFT_PIN          13
#define SERVO_RIGHT_PIN         12
#define SERVO_HAND_ROT_PIN      10

#define SERVO_ROT_ANALOG_PIN 2
#define SERVO_LEFT_ANALOG_PIN 0
#define SERVO_RIGHT_ANALOG_PIN 1
#define SERVO_HAND_ROT_ANALOG_PIN 3

// Old Control method Stretch / Height
#define ARM_STRETCH_MIN   0
#define ARM_STRETCH_MAX   300
#define ARM_HEIGHT_MIN    -100
#define ARM_HEIGHT_MAX    250
#define L3_MAX_ANGLE      120
#define L3_MIN_ANGLE      5
#define L4_MAX_ANGLE      120
#define L4_MIN_ANGLE      5
#define L4L3_MAX_ANGLE    150
#define L4L3_MIN_ANGLE    30

#define LIMIT_SW                2    // LIMIT Switch Button
#define BUZZER                  3    // HIGH = ON
#define BTN_D4                  4    // LOW = Pressed
#define BTN_D7                  7    // LOW = Pressed


#ifdef PRODUCT_MKII
    #define POW_DET                 A6   // power detection
    #define PUMP_GRI_STATUS         A7   // pump status feedback
    #define PUMP_GRI_EN             5    // HIGH = Pump ON
    #define SYS_LED                 6
    #define BT_DETEC                3    //share the port with buzzer
#else
    #define PUMP_EN                 6    // HIGH = Valve OPEN
    #define VALVE_EN                5    // HIGH = Pump ON
    #define GRIPPER                 9    // LOW = Catch
    #define GRIPPER_FEEDBACK        A6
#endif
#define MATH_PI 3.141592653589793238463
#define MATH_TRANS  57.2958
#define MATH_L1 90.00
#define MATH_L2 21.17
#define MATH_L3 148.25
#define MATH_L4 160.2
#define MATH_L43 MATH_L4/MATH_L3

//for the move_to function
#define IN_RANGE             0
#define OUT_OF_RANGE_IN_DST  1
#define OUT_OF_RANGE_IN_PATH 2
#define OUT_OF_RANGE         3

//#define RELATIVE 1
//#define ABSOLUTE 0
//for the pump gripper function
#define GRABBING        0
#define WORKING         1
#define STOP            2
#define PUMP_GRABBING_CURRENT 55
#ifdef PRODUCT_MKII
#define SERVO_9G_MAX    460
#define SERVO_9G_MIN    98

#else
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

#endif


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

#define SUCCESS                 1
#define FAILED                  -1

#define DATA_TYPE_BYTE          1
#define DATA_TYPE_INTEGER       2
#define DATA_TYPE_FLOAT         4

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
    void write_servo_angle(byte servo_num, double servo_angle);
    double analog_to_angle(int input_angle, byte servo_num);

    void arm_process_commands();
    bool available();

    unsigned char move_to(double x, double y, double z, double hand_angle, byte relative_flags, double times, byte ease_type, boolean enable_hand, bool polar);
    unsigned char move_to(double x, double y,double z, bool polar) {
        return move_to(x, y, z, 0, ABSOLUTE, 1.0, INTERP_EASE_INOUT_CUBIC, false, polar);
    }
    unsigned char move_to(double x, double y,double z,double times, bool polar) {
        return move_to(x, y, z, 0, ABSOLUTE, times, INTERP_EASE_INOUT_CUBIC, true, polar);
    }

    unsigned char get_current_xyz(double *cur_rot, double *cur_left, double *cur_right, double *g_current_x, double *g_current_y, double *g_current_z, bool for_movement );
    void get_current_rotleftright();
#ifdef PRODUCT_MKII
    void calibration_data_to_servo_angle(double *data,unsigned int address);
    void read_servo_angle(byte servo_number, bool original_data);
    void read_servo_angle(byte servo_number)
    {
        read_servo_angle(servo_number, false);
    }

#else
	int write_servo_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle);
	void write_servo_angle(byte servo_num, double servo_angle,  boolean with_offset);
	double read_servo_angle(byte servo_num);
	double read_servo_angle(byte servo_num, boolean with_offset);
	double analog_to_angle(int input_angle, byte servo_num, bool with_offset);
	void read_linear_offset(byte servo_num, double& intercept_val, double& slope_val);
	void get_current_xyz();
    void get_current_xyz(double theta_1, double theta_2, double theta_3);
#endif

    unsigned char coordinate_to_angle(double x, double y, double z, double *theta_1, double *theta_2, double *theta_3);

    void interpolate(double start_val, double end_val, double *interp_vals, byte ease_type);

    void gripper_catch(bool value);
	unsigned char gripper_status();
    void pump_catch(bool value);
    unsigned char pump_status();

    Servo g_servo_rot;
    Servo g_servo_left;
    Servo g_servo_right;
    Servo g_servo_hand_rot;

    int write_servos_angle(byte servo_rot_angle, byte servo_left_angle, byte servo_right_angle, byte servo_hand_rot_angle, byte trigger);
    int write_servos_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle, double servo_hand_rot_angle);
    int write_servos_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle);
    void attach_all();
    void attach_servo(byte servo_num);
    void runCommand(String cmnd);
    String getValues(String cmnd, String parameters[], int parameterCount, double *valueArray);

    // functions modified to be used for old version uArm
    //void angle_to_coordinate(double theta_1, double theta_2, double theta_3, double& x, double& y, double &z) {
    //    get_current_xyz(theta_1, theta_2, theta_3); x = g_current_x; y = g_current_y; z = g_current_z;
	//void get_current_xyz(double theta_1, double theta_2, double theta_3);
//	double read_servo_offset(byte servo_num);
//	boolean set_servo_status(boolean attach_state, byte servo_num);

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

    bool record();
    bool play();
    void recording_write(unsigned int address, unsigned char * data_array, int num);
    void recording_read(unsigned int address, unsigned char * data_array, int num);
protected:
    double cur_rot = 90;
    double cur_left = 90;
    double cur_right = 90;
    double cur_hand = 90;

    double g_current_x = 10;
    double g_current_y = 100;
    double g_current_z = 150;

    boolean move_to_the_closest_point = false;

    unsigned int INTERP_INTVLS;

    unsigned char move_times = 255;//255 means no move
    unsigned long buzzerStopTime=0;
    unsigned long moveStartTime=0;
    unsigned int microMoveTime=0;
    // the arrays to store the xyz coordinates first and then change to the rot left right angles
    double x_array[61];
    double y_array[61];
    double z_array[61];
    double hand_speed=10;//to save the memory

    //offset of assembling
    double RIGHT_SERVO_OFFSET;//12.5ALEX//5.6mine//-1liebao   //1.8Degree
    double LEFT_SERVO_OFFSET;//3.8ALEX//1mine//18.6liebao   //2.6Degree
    double ROT_SERVO_OFFSET;//7ALEX//0mine//-7liebao
    double HAND_ROT_SERVO_OFFSET;

    //sys status
    unsigned char sys_status = NORMAL_MODE;
    unsigned char time_50ms = 0;//used to change the led blink time
    unsigned char time_ticks = 0;

    //learning mode
    unsigned int addr = 0;

};

extern uArmClass uarm;

#endif
