/******************************************************************************
* File Name          : action_control.h
* Author             : Jerry.song
* Updated            : Alex Tan
* Version            : V0.3
* Created Date       : 12 Dec, 2014
* Modified Date      : 15 Sep, 2015
* Description        : 
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*******************************************************************************/


#include "uArm_library.h"
#include "linreg.h"


#ifndef uarm_action_control_h
#define uarm_action_control_h

#define SERVO_HAND              9     
#define HAND_ANGLE_OPEN         25
#define HAND_ANGLE_CLOSE        70
#define PUMP_EN                 6     
#define VALVE_EN                5     
#define MATH_PI	3.141592653589793238463
#define MATH_TRANS  57.2958
#define MATH_L1	(10.645+0.6)
#define MATH_L2	2.117
#define MATH_L3	14.825
#define MATH_L4	16.02
#define MATH_L43 MATH_L4/MATH_L3

#define RELATIVE 1
#define ABSOLUTE 0

class ActionControlClass{
public:
		ActionControlClass();
	    void moveTo(double x, double y,double z);
	    void moveTo(double x, double y,double z,int relative, double time);
	    void moveTo(double x, double y,double z,int relative, double time_sepnd, double servo_4_angle);
	    void drawCur(double length_1,double length_2,int angle, double time_spend);
	    void drawRec(double length_1,double length_2,double time_spend_per_length);

	    double getTheta1() const {return g_theta_1;}
	    double getTheta2() const {return g_theta_2;}
	    double getTheta3() const {return g_theta_3;}

	    double getCalX() {calXYZ(); return g_cal_x;}
	    double getCalY() {calXYZ(); return g_cal_y;}
	    double getCalZ() {calXYZ(); return g_cal_z;}

	    void calAngles(double x, double y, double z);

	    void calXYZ(double theta_1, double theta_2, double theta_3);
	    void calXYZ();

	    void gripperCatch();
	    void gripperRelease();
		void interpolation(double init_val, double final_val);

protected:
		double getInterPolValueArray(int num) const {return g_interpol_val_arr[10];}
		double calYonly(double theta_1, double theta_2, double theta_3);

		double g_x_in;
		double g_y_in;
		double g_z_in;
		double g_right_all;
		double g_sqrt_z_y;
		double g_theta_1;
		double g_theta_2;
		double g_theta_3;
		double g_phi;
		double g_right_all_2;
		double g_sqrt_z_x;
		double g_cal_x;
		double g_cal_y;
		double g_cal_z;

		double g_l3_1_2;
		double g_l4_1_2;
		double g_l5_2;

		double g_current_x;
		double g_current_y;
		double g_current_z;

		double g_interpol_val_arr[100];

		double g_l3_1;
		double g_l4_1;
		double g_l5;

		Servo g_servo_hand;
		boolean g_gripper_reset;

};

extern ActionControlClass actionControl;

#endif
