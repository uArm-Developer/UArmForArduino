/**
  ******************************************************************************
  * @file	uArmController.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArmController.h" 
#include "uArmIIC.h"
#include "uArmRecorder.h"

uArmController::uArmController()
{
	
}

void uArmController::init()
{
	for (int i = SERVO_ROT_NUM; i < SERVO_COUNT; i++)
	{
        double servoOffset = 0;

		servoOffset = readServoAngleOffset(i);
        
        if (isnan(servoOffset))
        {
            servoOffset = 0;
            EEPROM.put(MANUAL_OFFSET_ADDRESS + i * sizeof(servoOffset), servoOffset);
        }
        
        mServoAngleOffset[i] = servoOffset;
        debugPrint("offset[%d]: %s\n", i, D(mServoAngleOffset[i]));
	}

    // mServoAngleOffset[0] = 6;
    // mServoAngleOffset[1] = 7;
    // mServoAngleOffset[2] = 4;
    // mServoAngleOffset[3] = 0;


    for (int k = 0; k < 3; k++)
    {
        
        delay(10);
        unsigned char data[2];
        unsigned int offset = (4 + k) * 1024 + 500;

        iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, offset, 2);
       
        mMaxAdcPos[k] = (data[0] << 8) + data[1];

        //Serial.println(mMaxAdcPos[k]);

    }

    mServo[SERVO_ROT_NUM].setPulseWidthRange(500, 2500);
    mServo[SERVO_LEFT_NUM].setPulseWidthRange(500, 2500);
    mServo[SERVO_RIGHT_NUM].setPulseWidthRange(500, 2500);
    mServo[SERVO_HAND_ROT_NUM].setPulseWidthRange(600, 2400);

	attachAllServo();  

    // writeServoAngle(SERVO_ROT_NUM, 90);
    // writeServoAngle(SERVO_LEFT_NUM, 90);
    // writeServoAngle(SERVO_RIGHT_NUM, 0);
    // writeServoAngle(SERVO_HAND_ROT_NUM, 90);   
    mCurAngle[0] = readServoAngle(SERVO_ROT_NUM, true);
    mCurAngle[1] = readServoAngle(SERVO_LEFT_NUM, true);
    mCurAngle[2] = readServoAngle(SERVO_RIGHT_NUM, true);
    mCurAngle[3] = readServoAngle(SERVO_HAND_ROT_NUM, true);
}



void uArmController::attachAllServo()
{
	for (int i = SERVO_ROT_NUM; i < SERVO_COUNT; i++)
	{
		mServo[i].attach(SERVO_CONTROL_PIN[i]);
	}
}

void uArmController::attachServo(byte servoNum)
{
    mServo[servoNum].attach(SERVO_CONTROL_PIN[servoNum]);
}

void uArmController::detachServo(byte servoNum)
{
	mServo[servoNum].detach();
}

void uArmController::detachAllServo()
{
	for (int i = SERVO_ROT_NUM; i < SERVO_COUNT; i++)
	{
		detachServo(i);
	}
}

void uArmController::writeServoAngle(double servoRotAngle, double servoLeftAngle, double servoRightAngle)
{
    writeServoAngle(SERVO_ROT_NUM, servoRotAngle);
    writeServoAngle(SERVO_LEFT_NUM, servoLeftAngle);
    writeServoAngle(SERVO_RIGHT_NUM, servoRightAngle);
}

double uArmController::getReverseServoAngle(byte servoNum, double servoAngle)
{
#ifdef MKII
    debugPrint("s=%d, angleIn=%s", servoNum, D(servoAngle));
    if (servoAngle < mCurAngle[servoNum])
    {
        unsigned char data[2];

            int i_val = 0;
            int addr = (4 + servoNum) * 1024 + 362 + servoAngle * 2;

            addr &= 0xfffe;

            //gRecorder.read(addr, data, 2);
            iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, addr, 2);

            i_val = (data[0] << 8) + data[1];
            debugPrint("addr=%d,i_val=%d", addr, i_val);
            servoAngle = servoAngle - (i_val) - 1;
    }
#endif
    return servoAngle;

}

void uArmController::writeServoAngle(byte servoNum, double servoAngle, boolean writeWithOffset )
{





	mCurAngle[servoNum] = servoAngle;
    servoAngle = writeWithOffset ? (servoAngle + mServoAngleOffset[servoNum]) : servoAngle;

    debugPrint("serveAngle1=%s", D(servoAngle));
#ifdef MKII
	
	servoAngle = getReverseServoAngle(servoNum, servoAngle);

    switch (servoNum)
    {
    case SERVO_ROT_NUM:
        readServoCalibrationData(ROT_SERVO_ADDRESS, servoAngle);
        break;

    case SERVO_LEFT_NUM:
        readServoCalibrationData(LEFT_SERVO_ADDRESS, servoAngle);
        break;

    case SERVO_RIGHT_NUM:
        readServoCalibrationData(RIGHT_SERVO_ADDRESS, servoAngle);
        break;

    }
#endif
    debugPrint("serveAngle2=%s", D(servoAngle));

    mServo[servoNum].write(servoAngle);
}


#ifdef MKII
void uArmController::readServoCalibrationData(unsigned int address, double& angle)
{
    unsigned char calibration_data[DATA_LENGTH]; //get the calibration data around the data input
    unsigned int min_data_calibration_address;
    double closest_data, another_closest_data;
    unsigned int deltaA = 0xffff, deltaB = 0, i, i_min = 0;
    deltaA = 0xffff;
    deltaB = 0;

    if (abs(angle - 0.0) < 0.001)
    {
        return;
    }

    if (angle < (DATA_LENGTH >> 2))
    {
        min_data_calibration_address = 0;
    }
    else if (angle > (180 - (DATA_LENGTH >> 2)))
    {
        min_data_calibration_address = (((unsigned int)180 - (DATA_LENGTH >> 1)) * 2);
    }
    else
    {
        min_data_calibration_address = (((unsigned int)angle - (DATA_LENGTH >> 2)) * 2);
    }

    // Serial.print("input");
    // Serial.println(angle);
    // Serial.print("min_data_calibration_address:");
    // Serial.println(min_data_calibration_address);

    //if (min_data_calibration_address < 0)
    //    min_data_calibration_address = 0;

    unsigned char dataLen = DATA_LENGTH;
    if (min_data_calibration_address + dataLen > 360)
    {
        dataLen = 360 - min_data_calibration_address;
    }

    // Serial.print("dataLen:");
    // Serial.println(dataLen);

    iic_readbuf(calibration_data, EXTERNAL_EEPROM_SYS_ADDRESS, address + min_data_calibration_address, dataLen);



    for(i=0;i<(dataLen >> 1);i++)
    {
        deltaB = abs ((calibration_data[i+i]<<8) + calibration_data[1+(i+i)] - angle * 10);
        if(deltaA > deltaB)
        {
            i_min = i;
            deltaA = deltaB;
        }
    }

    closest_data = ((calibration_data[i_min+i_min]<<8) + calibration_data[1+(i_min+i_min)])/10.0;//transfer the dat from ideal data to servo angles
    if(angle >= closest_data)
    {
        if (i_min == 0 || (i_min > 0 && (i_min-1)* 2 < dataLen))
        {
            another_closest_data = ((calibration_data[i_min+i_min+2]<<8) + calibration_data[3+i_min+i_min])/10.0;//bigger than closest
       
    // Serial.print("another_closest_data1:");
    // Serial.println(another_closest_data);

            if(abs(another_closest_data - closest_data) < 0.001)
            {
                angle = min_data_calibration_address/2 + i_min + 0.5;
            }
            else
            {
                angle = 1.0 * (angle - closest_data) / (another_closest_data - closest_data) + min_data_calibration_address/2 + i_min ;
            }
        }
        else
        {
            angle = min_data_calibration_address/2 + i_min + 0.5;
        }
    }
    else
    {
        if (i_min > 0)
        {
            another_closest_data = ((calibration_data[i_min+i_min-2]<<8) + calibration_data[i_min+i_min-1])/10.0;//smaller than closest
    //             Serial.print("another_closest_data2:");
    // Serial.println(another_closest_data);

            if(abs(another_closest_data - closest_data) < 0.001)
            {
                angle = min_data_calibration_address/2 + i_min - 1 + 0.5;
            }
            else
            {
                angle = 1.0 * (angle - another_closest_data) / (closest_data - another_closest_data) + min_data_calibration_address/2 + i_min - 1;
            }
        }
        else
        {
            angle = min_data_calibration_address/2 + i_min + 0.5;
        }
    }    

    //angle += 1.0;
    // Serial.print("output angle:");
    // Serial.println(angle);


}
#endif

double uArmController::readServoAngle(byte servoNum, boolean withOffset )
{
	double angle;

    if (servoNum == SERVO_HAND_ROT_NUM)
    {
        angle = map(getServoAnalogData(SERVO_HAND_ROT_ANALOG_PIN), SERVO_9G_MIN, SERVO_9G_MAX, 0, 180);
    }
    else
    {
        angle = analogToAngle(servoNum, getServoAnalogData(servoNum)); 
    }
	

	if (withOffset)
	{
		angle -= mServoAngleOffset[servoNum];
	}

    angle = constrain(angle, 0.00, 180.00);

	return angle;
}


double uArmController::readServoAngles(double& servoRotAngle, double& servoLeftAngle, double& servoRightAngle, boolean withOffset)
{
    servoRotAngle = readServoAngle(SERVO_ROT_NUM, withOffset);
    servoLeftAngle = readServoAngle(SERVO_LEFT_NUM, withOffset);
    servoRightAngle = readServoAngle(SERVO_RIGHT_NUM, withOffset);
}


double uArmController::getServoAngles(double& servoRotAngle, double& servoLeftAngle, double& servoRightAngle)
{

    servoRotAngle = mCurAngle[SERVO_ROT_NUM];
    servoLeftAngle = mCurAngle[SERVO_LEFT_NUM];
    servoRightAngle = mCurAngle[SERVO_RIGHT_NUM];

}

double uArmController::getServeAngle(byte servoNum)
{
    return mCurAngle[servoNum];
}

void uArmController::updateAllServoAngle(boolean withOffset)
{
	
	for (unsigned char servoNum = SERVO_ROT_NUM; servoNum < SERVO_COUNT; servoNum++)
	{
		mCurAngle[servoNum] = readServoAngle(servoNum, withOffset); 	
	}
	
}

void uArmController::gripperCatch()
{
	digitalWrite(GRIPPER, LOW); 
}

void uArmController::gripperRelease()
{
 	digitalWrite(GRIPPER, HIGH); 
}

unsigned char uArmController::gripperStatus()
{
#ifdef MKII
    //Serial.println(getAnalogData(GRIPPER_FEEDBACK));
    if (digitalRead(GRIPPER) == HIGH)
    {
        return STOP;//NOT WORKING
    }
    else
    {
        
        if (getAnalogData(GRIPPER_FEEDBACK) > 600)
        {
            return WORKING;
        }
        else
        {
            return GRABBING;
        }
    }    
#elif defined(METAL)    
    if(digitalRead(GRIPPER) == HIGH)
    {
        return STOP;
    }
    else
    {
        if(getAnalogData(GRIPPER_FEEDBACK) > 600)
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


unsigned char uArmController::pumpStatus()
{
#ifdef MKII
    if (digitalRead(PUMP_EN) == HIGH)
    {
        return STOP;
    }
    else
    {
        //Serial.println(getAnalogData(PUMP_FEEDBACK));
        if (getAnalogData(PUMP_FEEDBACK) <= PUMP_GRABBING_CURRENT)
        {
            return GRABBING;
        }
        else
        {
            return WORKING;
        }
    }
#elif defined (METAL)
    if (digitalRead(PUMP_EN) == HIGH)
    {
        return 1;
    }
    else 
    {
        return 0;
    }
#endif
}


void uArmController::pumpOn()
{
    #ifdef MKII
    digitalWrite(PUMP_EN, LOW); 
    #elif defined(METAL)
    digitalWrite(PUMP_EN, HIGH); 
    digitalWrite(VALVE_EN, LOW);
    #endif
}

void uArmController::pumpOff()
{


    #ifdef MKII
    digitalWrite(PUMP_EN, HIGH); 
    #elif defined(METAL)
    digitalWrite(PUMP_EN, LOW); 
    digitalWrite(VALVE_EN, HIGH);
    #endif
}

unsigned char uArmController::getCurrentXYZ(double& x, double& y, double& z)
{

	// 在XY平面的投影长度
    double stretch = MATH_LOWER_ARM * cos(mCurAngle[SERVO_LEFT_NUM] / MATH_TRANS) + MATH_UPPER_ARM * cos(mCurAngle[SERVO_RIGHT_NUM] / MATH_TRANS) + MATH_L2 + MATH_FRONT_HEADER;

	// 在Z轴的投影长度,
    double height = MATH_LOWER_ARM * sin(mCurAngle[SERVO_LEFT_NUM] / MATH_TRANS) - MATH_UPPER_ARM * sin(mCurAngle[SERVO_RIGHT_NUM] / MATH_TRANS) + MATH_L1;
    x = stretch * cos(mCurAngle[SERVO_ROT_NUM] / MATH_TRANS);
    y = stretch * sin(mCurAngle[SERVO_ROT_NUM] / MATH_TRANS);
    z = height;

    return IN_RANGE;
}

unsigned char uArmController::getXYZFromPolar(double& x, double& y, double& z, double s, double r, double h)
{
    double stretch = s;
    
    z = h;  
    x = s * cos(r / MATH_TRANS);
    y = s * sin(r / MATH_TRANS);
}


unsigned char uArmController::getXYZFromAngle(double& x, double& y, double& z, double rot, double left, double right)
{
    // 在XY平面的投影长度
    double stretch = MATH_LOWER_ARM * cos(left / MATH_TRANS) + MATH_UPPER_ARM * cos(right / MATH_TRANS) + MATH_L2 + MATH_FRONT_HEADER;

    // 在Z轴的投影长度,
    double height = MATH_LOWER_ARM * sin(left / MATH_TRANS) - MATH_UPPER_ARM * sin(right / MATH_TRANS) + MATH_L1;
    x = stretch * cos(rot / MATH_TRANS);
    y = stretch * sin(rot / MATH_TRANS);
    z = height;

    return IN_RANGE;    
}

unsigned char uArmController::coordianteToAngle(double x, double y, double z, double& angleRot, double& angleLeft, double& angleRight, boolean allowApproximate)
{
    double xIn = 0.0;
    double zIn = 0.0;
    double rightAll = 0.0;
    double sqrtZX = 0.0;
    double phi = 0.0;

    x = constrain(x,-3276,3276);
    y = constrain(y,-3276,3276);
    z = constrain(z,-3276,3276);
    x = (double)((int)(x*10)/10.0);
    y = (double)((int)(y*10)/10.0);
    z = (double)((int)(z*10)/10.0);

    zIn = (z - MATH_L1) / MATH_LOWER_ARM;

    if(!allowApproximate)//if need the move to closest point we have to jump over the return function
    {
        //check the range of x
        if(y<0)
        {
            return OUT_OF_RANGE_NO_SOLUTION;
        }
    }
    // Calculate value of theta 1: the rotation angle
    if(x == 0)
    {
            angleRot = 90;
    }
    else
    {
            if (x > 0)
            {
                    angleRot = atan(y / x) * MATH_TRANS;//angle tranfer 0-180 CCW 弧度转为角度
            }
            if (x < 0)
            {
                    angleRot = 180 + atan(y / x) * MATH_TRANS;//angle tranfer  0-180 CCW
            }
    }
    // Calculate value of theta 3
    if(angleRot != 90)//xIn is the stretch
    {
            xIn = (x / cos(angleRot / MATH_TRANS) - MATH_L2 - MATH_FRONT_HEADER) / MATH_LOWER_ARM;
    }
    else
    {
            xIn = (y - MATH_L2 - MATH_FRONT_HEADER) / MATH_LOWER_ARM;
    }   

    phi = atan(zIn / xIn) * MATH_TRANS;//phi is the angle of line (from joint 2 to joint 4) with the horizon

    sqrtZX = sqrt(zIn*zIn + xIn*xIn); // 节点2到节点4的长度

    rightAll = (sqrtZX*sqrtZX + MATH_UPPER_LOWER * MATH_UPPER_LOWER  - 1) / (2 * MATH_UPPER_LOWER  * sqrtZX);//cosin law
    angleRight = acos(rightAll) * MATH_TRANS;//cosin law

    // Calculate value of theta 2
    rightAll = (sqrtZX*sqrtZX + 1 - MATH_UPPER_LOWER * MATH_UPPER_LOWER ) / (2 * sqrtZX);//cosin law
    angleLeft = acos(rightAll) * MATH_TRANS;//cosin law

    angleLeft = angleLeft + phi;
    angleRight = angleRight - phi;


    //debugPrint("angle r=%s, l=%s, r=%s", D(angleRot), D(angleLeft), D(angleRight));
    //determine if the angle can be reached
    return limitRange(angleRot, angleLeft, angleRight);
}

unsigned char uArmController::limitRange(double& angleRot, double& angleLeft, double& angleRight)
{
    //determine if the angle can be reached
    if(isnan(angleRot)||isnan(angleLeft)||isnan(angleRight))
    {
            return OUT_OF_RANGE_NO_SOLUTION;
    }
    if(((angleLeft + mServoAngleOffset[SERVO_LEFT_NUM]) < LOWER_ARM_MIN_ANGLE)||((angleLeft + mServoAngleOffset[SERVO_LEFT_NUM]) > LOWER_ARM_MAX_ANGLE))//check the right in range
    {
            return OUT_OF_RANGE;
    }
    if(((angleRight + mServoAngleOffset[SERVO_RIGHT_NUM]) < UPPER_ARM_MIN_ANGLE)||((angleRight + mServoAngleOffset[SERVO_RIGHT_NUM]) > UPPER_ARM_MAX_ANGLE))//check the left in range
    {
            return OUT_OF_RANGE;
    }
    if(((180 - angleLeft - angleRight)>LOWER_UPPER_MAX_ANGLE)||((180 - angleLeft - angleRight)<LOWER_UPPER_MIN_ANGLE))//check the angle of upper arm and lowe arm in range
    {
            return OUT_OF_RANGE;
    }

    return IN_RANGE;
}



void uArmController::readLinearOffset(byte servoNum, double& interceptVal, double& slopeVal)
{
    EEPROM.get(LINEAR_INTERCEPT_START_ADDRESS + servoNum * sizeof(interceptVal), interceptVal);
    EEPROM.get(LINEAR_SLOPE_START_ADDRESS + servoNum * sizeof(slopeVal), slopeVal);
}


#ifdef MKII
double uArmController::analogToAngle(byte servoNum, int inputAnalog)
{


    int startAddr = (4 + servoNum) * 1024;
    // binary search
    bool done = false;
    int min = 0;
    int max = mMaxAdcPos[servoNum];
    int angle = (min + max) / 2;
    unsigned char data[2];
    int val = 0;

    //debugPrint("inputAnalog=%d", inputAnalog);
    while (!done && (min < max))
    {
        //debugPrint("angle=%d", angle);
        //gRecorder.read(startAddr+angle*2, data, 2);
        iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, startAddr+angle*2, 2);

        val = (data[0] << 8) + data[1];
        //debugPrint("val=%d", val);
        //Serial.print("val=");
        //Serial.println(val);

#ifdef METAL_MOTOR
        if (val == inputAnalog)
        {
            return angle;
        }
        else if (val > inputAnalog)
        {
            min = angle;
        }
        else
        {
            max = angle;
        }
#else
        if (val == inputAnalog)
        {
            return angle;
        }
        else if (val < inputAnalog)
        {
            min = angle;
        }
        else
        {
            max = angle;
        }        
#endif

        angle = (min + max) / 2;

        //debugPrint("addr2=%d", angle);
        if (angle == min || angle == max)
            break;
    }   


    // Serial.print("angle=");
    // Serial.println(angle);

    if (angle == 0 || angle == mMaxAdcPos[servoNum])
    {
        return angle;
    }

    unsigned char adc_data[6];

    iic_readbuf(adc_data, EXTERNAL_EEPROM_SYS_ADDRESS, startAddr+(angle-1)*2, 6);

    min = (adc_data[0] << 8) + adc_data[1];
    int mid = (adc_data[2] << 8) + adc_data[3];
    max = (adc_data[4] << 8) + adc_data[5];

    double angleMin = 0.0;
    double angleMax = 0.0;
    double angleValue = 0;

    if (inputAnalog > min && inputAnalog <= mid)
    {
        angleMin = angle - 1;
        angleMax = angle;
        max = mid;
    }
    else
    {
        angleMin = angle;
        angleMax = angle+1;
        min = mid;
    }


    angleValue = (inputAnalog - min) * (angleMax - angleMin) / (max - min) + angleMin ;

    // Serial.print("angleValue=");
    // Serial.println(angleValue);

    return angleValue;

    // els
    // {
    //     return 0;
    // }

/*    
    unsigned char adc_calibration_data[DATA_LENGTH],data[4]; //get the calibration data around the data input
    unsigned int min_data_calibration_address, max_calibration_data, min_calibration_data;
    unsigned int angle_range_min, angle_range_max;

    double angle = 0;

    debugPrint("servoNum=%d, input=%d", servoNum, inputAnalog);

    switch(servoNum)
    {
    case  SERVO_ROT_NUM:      
        iic_readbuf(&data[0], EXTERNAL_EEPROM_SYS_ADDRESS, ROT_SERVO_ADDRESS + 360, 2);//get the min adc calibration data for the map() function                      
        iic_readbuf(&data[2], EXTERNAL_EEPROM_SYS_ADDRESS, ROT_SERVO_ADDRESS + 360 + 358, 2);//get the max adc calibraiton data for the map() function
        break;

    case  SERVO_LEFT_NUM:     
        iic_readbuf(&data[0], EXTERNAL_EEPROM_SYS_ADDRESS, LEFT_SERVO_ADDRESS + 360, 2);//get the min adc calibration data for the map() function
        iic_readbuf(&data[2], EXTERNAL_EEPROM_SYS_ADDRESS, LEFT_SERVO_ADDRESS + 360 + 358, 2);//get the max adc calibraiton data for the map() function
        break;

    case  SERVO_RIGHT_NUM:    
        iic_readbuf(&data[0], EXTERNAL_EEPROM_SYS_ADDRESS, RIGHT_SERVO_ADDRESS + 360, 2);//get the min adc calibration data for the map() function
        iic_readbuf(&data[2], EXTERNAL_EEPROM_SYS_ADDRESS, RIGHT_SERVO_ADDRESS + 360 + 358, 2);//get the max adc calibraiton data for the map() function
        break;

    default:                  
        break;
    }

    max_calibration_data = (data[2]<<8) + data[3];
    min_calibration_data = (data[0]<<8) + data[1];

    //debugPrint("min%d, max=%d", min_calibration_data, max_calibration_data);

    angle_range_min = map(inputAnalog, min_calibration_data, max_calibration_data, 1, 180) - (DATA_LENGTH>>2);
    min_data_calibration_address = (angle_range_min * 2);


    //debugPrint("mindata%d, ", min_data_calibration_address);

    switch(servoNum)
    {
    case  SERVO_ROT_NUM:      
        iic_readbuf(adc_calibration_data, EXTERNAL_EEPROM_SYS_ADDRESS, ROT_SERVO_ADDRESS + min_data_calibration_address + 360, DATA_LENGTH);//360 means the adc calibration data offset
        break;
    case  SERVO_LEFT_NUM:     
        iic_readbuf(adc_calibration_data, EXTERNAL_EEPROM_SYS_ADDRESS, LEFT_SERVO_ADDRESS + min_data_calibration_address + 360, DATA_LENGTH);//360 means the adc calibration data offset
        break;
    case  SERVO_RIGHT_NUM:    
        iic_readbuf(adc_calibration_data, EXTERNAL_EEPROM_SYS_ADDRESS, RIGHT_SERVO_ADDRESS + min_data_calibration_address + 360, DATA_LENGTH);//360 means the adc calibration data offset
        break;
    default:                 
        break;
    }

    unsigned int deltaA = 0xffff, deltaB = 0, i, i_min = 0;
    for(i=0;i<(DATA_LENGTH >> 1);i++)
    {
        deltaB = abs ((adc_calibration_data[i+i]<<8) + adc_calibration_data[1+(i+i)] - inputAnalog);
        if(deltaA > deltaB)
        {
            i_min = i;
            deltaA = deltaB;
        }
    }

    angle_range_min = angle_range_min + i_min;
    angle_range_max = angle_range_min + 1;

    //debugPrint("mina%d, maxa=%d", angle_range_min, angle_range_max);

    if((((adc_calibration_data[i_min+i_min]<<8) + adc_calibration_data[1+i_min+i_min]) - inputAnalog) >= 0)//determine if the current value bigger than the inputAnalog
    {
    
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
        angle = ( 1.0 * (inputAnalog - min_calibration_data)/(max_calibration_data - min_calibration_data) + angle_range_min);
    }
    else
    {
        angle = (angle_range_min + angle_range_max) / 2.0;//angle from 1-180 but the address from 0-179
    }    

   //debugPrint("angle=%s", D(angle));


    return angle;
*/
}

#elif defined(METAL)

double uArmController::analogToAngle(byte servoNum, int inputAnalog)
{
    double intercept = 0.0f;
    double slope = 0.0f;



    readLinearOffset(servoNum, intercept, slope);


    double angle = intercept + slope * inputAnalog;  


    //debugPrint("analogToAngle: inter:%s, slo:%s, inpu:%d, angle:%s", D(intercept), D(slope), inputAnalog, D(angle));


    return angle;
}
#endif

void uArmController::sort(unsigned int array[], unsigned int len)
{
	unsigned char i=0,j=0;
	unsigned int temp = 0;

	for(i = 0; i < len; i++) 
	{
		for(j = 0; i+j < (len-1); j++) 
		{
			if(array[j] > array[j+1]) 
			{
				temp = array[j];
				array[j] = array[j+1];
				array[j+1] = temp;
			}
		}
	}	
}

unsigned int uArmController::getServoAnalogData(byte servoNum)
{
	// unsigned int dat[8];


	// for(int i = 0; i < 8; i++)
	// {
	// 	dat[i] = analogRead(SERVO_ANALOG_PIN[servoNum]);
	// }

	// sort(dat, 8);

	// unsigned int result = (dat[2]+dat[3]+dat[4]+dat[5])/4;

	// return result;
    return getAnalogData(SERVO_ANALOG_PIN[servoNum]);
}

unsigned int uArmController::getAnalogData(byte pin)
{
    unsigned int dat[8];


    for(int i = 0; i < 8; i++)
    {
        dat[i] = analogRead(pin);
    }

    sort(dat, 8);

    unsigned int result = (dat[2]+dat[3]+dat[4]+dat[5])/4;

    return result;    
}

double uArmController::readServoAngleOffset(byte servoNum)
{
	double manualServoOffset = 0.0f;

	EEPROM.get(MANUAL_OFFSET_ADDRESS + servoNum * sizeof(manualServoOffset), manualServoOffset);

	return manualServoOffset;	
}

