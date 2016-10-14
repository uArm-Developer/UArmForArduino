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

uArmController::uArmController()
{
	
}

void uArmController::init()
{
	for (int i = SERVO_ROT_NUM; i < SERVO_COUNT; i++)
	{
        double handServoOffset = 0;

		handServoOffset = readServoAngleOffset(i);
        
        if (isnan(handServoOffset))
        {
            handServoOffset = 0;
            EEPROM.put(MANUAL_OFFSET_ADDRESS + i * sizeof(handServoOffset), handServoOffset);
        }
        
        mServoAngleOffset[i] = handServoOffset;
        debugPrint("offset[%d]: %s\n", i, D(mServoAngleOffset[i]));
	}

	delay(1000);

	attachAllServo();  

    updatePosition();

}

void uArmController::updatePosition()
{
    for (int i = SERVO_ROT_NUM; i < SERVO_COUNT; i++)
    {
        updatePosition(i);
    }   
}

void uArmController::attachAllServo()
{
	for (int i = SERVO_ROT_NUM; i < SERVO_COUNT; i++)
	{
		attachServo(i);
	}
}

void uArmController::attachServo(byte servoNum)
{
	attachServo(servoNum, SERVO_CONTROL_PIN[servoNum], SERVO_ANALOG_MIN_VALUE[servoNum]);
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

void uArmController::writeServoAngle(byte servoNum, double servoAngle, boolean writeWithOffset )
{

	mCurAngle[servoNum] = servoAngle;
    servoAngle = writeWithOffset ? (servoAngle + mServoAngleOffset[servoNum]) : servoAngle;

    mServo[servoNum].write(servoAngle);
}

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
    if(digitalRead(GRIPPER) == HIGH)
    {
        return STOP;
    }
    else
    {
        if(getServoAnalogData(GRIPPER_FEEDBACK) > 600)
        {
            return WORKING;
        }
        else
        {
            return GRABBING;
        }
    }
}

void uArmController::pumpOn()
{
    digitalWrite(PUMP_EN, HIGH); 
    digitalWrite(VALVE_EN, LOW);
}

void uArmController::pumpOff()
{
    digitalWrite(PUMP_EN, LOW); 
    digitalWrite(VALVE_EN, HIGH);
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


    debugPrint("angle r=%s, l=%s, r=%s", D(angleRot), D(angleLeft), D(angleRight));
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

double uArmController::analogToAngle(byte servoNum, int inputAnalog)
{
    double intercept = 0.0f;
    double slope = 0.0f;



    readLinearOffset(servoNum, intercept, slope);


    double angle = intercept + slope * inputAnalog;  

#ifdef DEBUG
    debugPrint("analogToAngle: inter:%s, slo:%s, inpu:%d, angle:%s", D(intercept), D(slope), inputAnalog, D(angle));
#endif // DEBUG

    return angle;
}

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
	unsigned int dat[8];


	for(int i = 0; i < 8; i++)
	{
		dat[i] = analogRead(SERVO_ANALOG_PIN[servoNum]);
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

void uArmController::updatePosition(byte servoNum)
{
	if (servoNum == SERVO_HAND_ROT_NUM)
	{
		mCurAngle[SERVO_HAND_ROT_NUM] = map(getServoAnalogData(SERVO_HAND_ROT_ANALOG_PIN), SERVO_9G_MIN, SERVO_9G_MAX, 0, 180);
	}
	else
	{
		mCurAngle[servoNum] = readServoAngle(servoNum, true); 
	}

	if (EEPROM.read(CALIBRATION_LINEAR_FLAG) != CONFIRM_FLAG)
	{
        writeServoAngle(servoNum, DEFAULT_ANGLE);
	}
	else
	{
		writeServoAngle(servoNum, mCurAngle[servoNum]);
	}
	  	
}

void uArmController::attachServo(byte servoNum, byte pin, int valueMin)
{

	if (getServoAnalogData(SERVO_ANALOG_PIN[servoNum]) > valueMin) // Servo Protection
	{ 
        if (servoNum == SERVO_HAND_ROT_NUM)
        {
            mServo[servoNum].attach(pin, 600, 2400);
        }
        else
        {
            mServo[servoNum].attach(pin);
        }
		
		
	}	
}