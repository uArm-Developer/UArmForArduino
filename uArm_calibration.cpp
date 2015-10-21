/******************************************************************************
* File Name          : calibration.cpp
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
#include "uArm_calibration.h"
#include "linreg.h"

CalibrationClass calib;

CalibrationClass::CalibrationClass(){
}




void CalibrationClass::calibrationServo(byte servo_num)
{

	const byte kServoRangeIni = 20 ;
	const byte kServoRangeFin = 100;	
	double l_angle_analog;
	double arr_real[16];
	double arr_input[16];
	int l_address = uarm.kAddrServo + (servo_num - 1) * 6;
	Serial.print("l_address: ");
	Serial.println(l_address);	
	Serial.print("servo ");
    Serial.println(servo_num);


	byte l_analog_pin;
	
	for (byte i = 0; i < (((kServoRangeFin-kServoRangeIni)/5)+1); i ++)
	{
		byte dot_i = 5*i;
		switch(servo_num)
		{
			case 1:
				l_analog_pin = 2;
				uarm.writeAngle(kServoRangeIni+dot_i, 60, 60, 0,1);

			
				break;

			case 2:
				l_analog_pin = 0;
				uarm.writeAngle(90, kServoRangeIni+dot_i, 30, 0,1);
			
				break;

			case 3:
				l_analog_pin = 1;
				uarm.writeAngle(90, 60 , kServoRangeIni+dot_i, 0,1);
			
				break;

			case 4:
				l_analog_pin = 3;
				uarm.writeAngle(90, 60, 60, kServoRangeIni+dot_i,1);
				break;

			default:
			
				break;
		}
		
		
		if(i == 0) {
			delay(2000);
		}

		for (int l = 0 ; l<3;l++) {
				l_angle_analog = analogRead(l_analog_pin);
				delay(100);
		}

		arr_real[i] = kServoRangeIni + dot_i;
		arr_input[i] = l_angle_analog;

		delay(800);

	}
	arr_real[0] = kServoRangeIni;

	LinearRegression lr(arr_input, arr_real, 16);
	Serial.print("lr.getA():");
	Serial.println(lr.getA());
	Serial.print("lr.getB():");
	Serial.println(lr.getB());
	uarm.saveDataToRom(lr.getA(),l_address);
	uarm.saveDataToRom(lr.getB(),l_address+3);

}


void CalibrationClass::calibrations(){
	
	for (int k = 1; k < 5; k++){

		calibrationServo(k);
		delay(2000);
	}

	setOffset();
	EEPROM.write(0, CALIBRATION_FLAG);

	Serial.println("All done!");
}


void CalibrationClass::setOffset()
{
	int setLoop = 1;

	uarm.detachAll();

	Serial.println("Put uarm in calibration posture (servo 1 to 3: 135, 50, 90 degree respectively), then input 1");
	while (setLoop){

		if (Serial.available()>0){
			
			char inputChar = Serial.read();

			if (inputChar=='1')
			{
				double offsetRot = uarm.readAngle(1,1) - 135;
				double offsetL = uarm.readAngle(2,1) - 50;
				double offsetR = uarm.readAngle(3,1) - 90;

                Serial.print("Offsets for servo 1 to 3 are ");
                Serial.println(offsetRot);
                Serial.println(offsetL);
                Serial.println(offsetR);


				if (abs(offsetRot)>25.4||abs(offsetL)>25.4||abs(offsetR)>25.4)
				{
					Serial.print("Check posture");
          

				}
				else{

					saveOffsetValue(offsetRot,1);
					saveOffsetValue(offsetL,2);
					saveOffsetValue(offsetR,3);
					setLoop = 0;
					Serial.println("Save offset done! ");
				}

				
			}
			else if(inputChar== 'e')
			{
				Serial.println("exit");
				setLoop = 0;
			}
		
			else{
				Serial.println("Incorrect, input again, or e to exit");
			}

		}

	}
}


void CalibrationClass::saveOffsetValue(double value, byte servo_num)
{
	int LSBs;
	int MSBs;

	if (value >0)
		LSBs = 1;
	else
		MSBs = 0;

	value *= 10;
	MSBs = (byte) value;

	EEPROM.write( uarm.kAddrOffset + (servo_num - 1)*2, abs(LSBs));
	EEPROM.write( uarm.kAddrOffset + (servo_num - 1)*2 +1, abs(MSBs));

}