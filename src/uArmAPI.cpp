/**
  ******************************************************************************
  * @file	uArmAPI.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-11-28
  ******************************************************************************
  */

#include "uArmAPI.h" 

uArmButton buttonMenu;
uArmButton buttonPlay;

#ifdef MKII
    uArmLed ledRed;
#endif

#ifdef DEBUG 
#define STEP_MAX	30	// no enough ram for debug. so reduce the size to enable debug
#else
#define STEP_MAX	60
#endif

#define INTERP_EASE_INOUT_CUBIC 0  // original cubic ease in/out
#define INTERP_LINEAR           1
#define INTERP_EASE_INOUT       2  // quadratic easing methods
#define INTERP_EASE_IN          3
#define INTERP_EASE_OUT         4

#define STEP_MAX_TIME				20	// ms

#define EXTERNAL_EEPROM_SYS_ADDRESS 0xA2
#define EXTERNAL_EEPROM_USER_ADDRESS  0xA0

#define PUMP_GRABBING_CURRENT 	55    

static int mCurStep;
static int mTotalSteps;
static unsigned int mTimePerStep;
static unsigned long mStartTime;
static double mPathX[STEP_MAX];
static double mPathY[STEP_MAX];
static double mPathZ[STEP_MAX];


static unsigned char _moveTo(double x, double y, double z, double speed);
static void _sort(unsigned int array[], unsigned int len);
static void _controllerRun();

void initHardware()
{

	pinMode(BTN_D4, INPUT_PULLUP);        //special mode for calibration
	pinMode(BUZZER, OUTPUT);
	pinMode(LIMIT_SW, INPUT_PULLUP);
	pinMode(BTN_D7, INPUT_PULLUP);
	pinMode(PUMP_EN, OUTPUT);
    pinMode(GRIPPER, OUTPUT);

    #ifdef MKII
    pinMode(SYS_LED, OUTPUT);

    digitalWrite(PUMP_EN, HIGH);//keep the pump off
    #endif 

    #ifdef METAL
	pinMode(VALVE_EN, OUTPUT);
    #endif


    buttonMenu.setPin(BTN_D4);
    buttonPlay.setPin(BTN_D7);

    buzzer.setPin(BUZZER);

    #ifdef MKII
    ledRed.setPin(SYS_LED);
    #endif


    

}

/*!
   \brief init components
 */
void uArmInit()
{

	initHardware();
	controller.init();

    mCurStep = -1;
    mTotalSteps = -1; 	
}

/*!
   \brief move to pos(x, y, z)
   \param x, y, z in mm
   \param speed: 
   			[0]: move to destination directly
   			[1~99]: change the dutycycle of servo (1~99%)
   			[100~1000]: mm/min, will do interpolation to control the speed and block process util move done
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE will move to the closest pos
   \return NO_NEED_TO_MOVE if it is already there
 */
unsigned char moveTo(double x, double y, double z, double speed)
{
	unsigned char result = IN_RANGE;

	debugPrint("moveTo: x=%f, y=%f, z=%f, speed=%f", x, y, z, speed);

	// when speed less than 100 mm/min, move to destination directly
	if (speed < 0)
	{
		return OUT_OF_RANGE_NO_SOLUTION;
	}
	else if (speed < 100)
	{
		unsigned char dutyCycle = map(speed, 0, 99, 0,  255);   
		
		controller.setServoSpeed(dutyCycle);

		double angleB, angleL, angleR;
		result = controller.xyzToAngle(x, y, z, angleB, angleL, angleR);
		if (result != OUT_OF_RANGE_NO_SOLUTION)
		{
			controller.writeServoAngle(angleB, angleL, angleR);
		}

		return result;		
	}
	else
	{
		controller.setServoSpeed(255);
		result = _moveTo(x, y, z, speed);

		if(result != OUT_OF_RANGE_NO_SOLUTION)
		{
			_controllerRun();
			
		}

		return result;
	}
}

/*!
   \brief move to pos(x, y, z) according to current pos
   \param x, y, z (mm)
   \param speed (mm/min)
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE will move to the closest pos
   \return NO_NEED_TO_MOVE if it is already there
 */
unsigned char relativeMove(double x, double y, double z, double speed)
{
    double x1, y1, z1;
    // get cur xyz
    controller.getCurrentXYZ(x1, y1, z1);

    x1 += x;
    y1 += y;
    z1 += z;

    return moveTo(x1, y1, z1, speed);	
}

/*!
   \brief move to pos of polor coordinates(s, r, h)
   \param s: stretch(mm)
   \param r: angle (0~180)
   \param h: height(mm)
   \param speed (mm/min)
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE will move to the closest pos
   \return NO_NEED_TO_MOVE if it is already there
 */
unsigned char moveToPol(double s, double r, double h, double speed)
{
	double x, y, z;

	polToXYZ(s, r, h, x, y, z);
	return moveTo(x, y, z, speed);		
}

/*!
   \brief attach servo(0~3)
   \param servoNumber: SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \return true or false
 */
bool attachServo(unsigned char servoNumber)
{
	if (servoNumber < SERVO_COUNT)
	{
		controller.attachServo(servoNumber);
		return true;
	}

	return false;
}

/*!
   \brief detach servo(0~3)
   \param servoNumber: SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \return true or false
 */
bool detachServo(unsigned char servoNumber)
{
	if (servoNumber < SERVO_COUNT)
	{
		controller.detachServo(servoNumber);
		return true;
	}

	return false;	
}

/*!
   \brief set servo angle
   \param servoNumber(0~3)
   \param angle (0~180)
   \return OK if everything is OK
   \return ERR_SERVO_INDEX_EXCEED_LIMIT if servoNumber not in range(0~3)
   \return ERR_ANGLE_OUT_OF_RANGE if angle not in range(0~180)
 */
unsigned char setServoAngle(unsigned char servoNumber, double angle)
{
	if (servoNumber >= SERVO_COUNT)
		return ERR_SERVO_INDEX_EXCEED_LIMIT;

	if (angle > 180 || angle < 0)
		return ERR_ANGLE_OUT_OF_RANGE;

	controller.writeServoAngle(servoNumber, angle);

	return OK;
}

/*!
   \brief get servo angle
   \param servoNumber(0~3)
   \return value of angle
   \return -1 if servoNumber not in range(0~3)
 */
double getServoAngle(unsigned char servoNumber)
{
	if (servoNumber >= SERVO_COUNT)
		return -1;	

	return controller.readServoAngle(servoNumber);
}


/*!
   \brief gripper work
 */
void gripperCatch()
{
	digitalWrite(GRIPPER, LOW); 
}

/*!
   \brief gripper stop
 */
void gripperRelease()
{
 	digitalWrite(GRIPPER, HIGH); 
}

/*!
   \brief get gripper status
   \return STOP if gripper is not working
   \return WORKING if gripper is working but not catched sth
   \return GRABBING if gripper got sth   
 */
unsigned char getGripperStatus()
{
#ifdef MKII
    //Serial.println(getAnalogData(GRIPPER_FEEDBACK));
    if (digitalRead(GRIPPER) == HIGH)
    {
        return STOP;//NOT WORKING
    }
    else
    {
        
        if (getAnalogPinValue(GRIPPER_FEEDBACK) > 600)
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
        if(getAnalogPinValue(GRIPPER_FEEDBACK) > 600)
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

/*!
   \brief get pump status
   \return STOP if pump is not working
   \return WORKING if pump is working but not catched sth
   \return GRABBING if pump got sth   
 */
unsigned char getPumpStatus()
{
#ifdef MKII
    if (digitalRead(PUMP_EN) == HIGH)
    {
        return STOP;
    }
    else
    {
        //Serial.println(getAnalogData(PUMP_FEEDBACK));
        if (getAnalogPinValue(PUMP_FEEDBACK) <= PUMP_GRABBING_CURRENT)
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

/*!
   \brief pump working
 */
void pumpOn()
{
    #ifdef MKII
    digitalWrite(PUMP_EN, LOW); 
    #elif defined(METAL)
    digitalWrite(PUMP_EN, HIGH); 
    digitalWrite(VALVE_EN, LOW);
    #endif
}

/*!
   \brief pump stop
 */
void pumpOff()
{


    #ifdef MKII
    digitalWrite(PUMP_EN, HIGH); 
    #elif defined(METAL)
    digitalWrite(PUMP_EN, LOW); 
    digitalWrite(VALVE_EN, HIGH);
    #endif
}


/*!
   \brief get tip status
   \return true if limit switch hit
 */
bool getTip()
{
	if (digitalRead(LIMIT_SW))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*!
   \brief get pin value
   \param pin of arduino
   \return HIGH or LOW
 */
int getDigitalPinValue(unsigned int pin)
{
	return digitalRead(pin);
}

/*!
   \brief set pin value
   \param pin of arduino
   \param value: HIGH or LOW
 */
void setDigitalPinValue(unsigned int pin, unsigned char value)
{
	if (value)
	{
		digitalWrite(pin, HIGH);
	}
	else
	{
		digitalWrite(pin, LOW);
	}
}

/*!
   \brief get analog value of pin
   \param pin of arduino
   \return value of analog data
 */
int getAnalogPinValue(unsigned int pin)
{
    unsigned int dat[8];


    for(int i = 0; i < 8; i++)
    {
        dat[i] = analogRead(pin);
    }

    _sort(dat, 8);

    unsigned int result = (dat[2]+dat[3]+dat[4]+dat[5])/4;

    return result;    
}

/*!
   \brief convert polor coordinates to Cartesian coordinate
   \param s(mm), r(0~180), h(mm)
   \output x, y, z(mm)
 */
void polToXYZ(double s, double r, double h, double& x, double& y, double& z)
{
    z = h;  
    x = s * cos(r / MATH_TRANS);
    y = s * sin(r / MATH_TRANS);	
}

/*!
   \brief check pos reachable
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE can move to the closest pos
 */
unsigned char validatePos(double x, double y, double z)
{
	double angleRot, angleLeft, angleRight;
	return controller.xyzToAngle(x, y, z, angleRot, angleLeft, angleRight, false);
}

/*!
   \brief get current pos
   \output x, y, z(mm)
 */
void getCurrentXYZ(double& x, double& y, double& z)
{
    controller.updateAllServoAngle();
    controller.getCurrentXYZ(x, y, z);	
}

/*!
   \brief get current pos of polor coordinates
   \output s(mm), r(0~180), h(mm)
 */
void getCurrentPosPol(double& s, double& r, double& h)
{

	double angleRot, angleLeft, angleRight;
	double x, y, z;

    controller.updateAllServoAngle();
	controller.getCurrentXYZ(x, y, z);
	controller.getServoAngles(angleRot, angleLeft, angleRight);
    double stretch;
    stretch = sqrt(x * x + y * y);

    s = stretch;
    r = angleRot;
    h = z;
}

/*!
   \brief get servo angles from pos(x, y, z)
   \param x, y, z(mm)
   \output angles of servo(0~180)
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE can move to the closest pos   
 */
unsigned char xyzToAngle(double x, double y, double z, double& angleRot, double& angleLeft, double& angleRight)
{
	return controller.xyzToAngle(x, y, z, angleRot, angleLeft, angleRight);
}

/*!
   \brief get  pos(x, y, z) from servo angles  
   \param angles of servo(0~180)
   \output x, y, z(mm)
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE can move to the closest pos   
 */
unsigned char angleToXYZ(double angleRot, double angleLeft, double angleRight, double& x, double& y, double& z)
{
	return controller.getXYZFromAngle(x, y, z, angleRot, angleLeft, angleRight);
}

/*!
   \brief get e2prom data
   \param device:  EEPROM_ON_CHIP, EEPROM_EXTERN_USER, EEPROM_EXTERN_SYSTEM
   \param addr: 0~2047(EEPROM_ON_CHIP), 0~65535(EEPROM_EXTERN_USER, EEPROM_EXTERN_SYSTEM)
   \param type: DATA_TYPE_BYTE, DATA_TYPE_INTEGER, DATA_TYPE_FLOAT
 */
double getE2PROMData(unsigned char device, unsigned int addr, unsigned char type)
{
   	double result = 0;

    uint8_t deviceAddr;
 

    union {
        float fdata;
        uint8_t data[4];
    } FData;


    switch(device)
    {

    case 0:

        switch(type)
        {
        case DATA_TYPE_BYTE:
        	{
                int val = EEPROM.read(addr);
                result = val;
                break;
        	}
        case DATA_TYPE_INTEGER:
        	{
                int i_val = 0;
                EEPROM.get(addr, i_val);
         		result = i_val;
                break;
        	}
        case DATA_TYPE_FLOAT:
        	{
                double f_val = 0.0f;
                EEPROM.get(addr,f_val);
            	result = f_val;
                break;
        	}
        }

        break;

    case 1:
        deviceAddr = EXTERNAL_EEPROM_USER_ADDRESS;
        break;

    case 2:
        deviceAddr = EXTERNAL_EEPROM_SYS_ADDRESS;
        break;

    default:
        return ADDRESS_ERROR;
    }

    
    if (device == 1 || device == 2)
    {
        int num = 0;
        switch(type)
        {
        case DATA_TYPE_BYTE:
            {
                num = 1;
                break;
            }
        case DATA_TYPE_INTEGER:
            {
                num = 2;
                break;
            }
        case DATA_TYPE_FLOAT:
            {
                num = 4;
                break;
            }
        default:
            return PARAMETER_ERROR;
        }

        unsigned char i=0;
        i = (addr % 128);
        // Since the eeprom's sector is 128 byte, if we want to write 5 bytes per cycle we need to care about when there's less than 5 bytes left
        if (i >= (129-num)) 
        {
            i = 128 - i;
            iic_readbuf(FData.data, deviceAddr, addr, i);// write data
            delay(5);
            iic_readbuf(FData.data + i, deviceAddr, addr + i, num - i);// write data
        }
        //if the left bytes are greater than 5, just do it
        else
        {
            iic_readbuf(FData.data, deviceAddr, addr, num);// write data
        }      


        switch(type)
        {
        case DATA_TYPE_BYTE:
            {
                result = FData.data[0];
                break;
            }
        case DATA_TYPE_INTEGER:
            {
                result = (FData.data[0] << 8) + FData.data[1];
                break;
            }
        case DATA_TYPE_FLOAT:
            {
                result = FData.fdata;
                break;
            }
        }
       

    }

    return result;

}

/*!
   \brief set e2prom data
   \param device:  EEPROM_ON_CHIP, EEPROM_EXTERN_USER, EEPROM_EXTERN_SYSTEM
   \param addr: 0~2047(EEPROM_ON_CHIP), 0~65535(EEPROM_EXTERN_USER, EEPROM_EXTERN_SYSTEM)
   \param type: DATA_TYPE_BYTE, DATA_TYPE_INTEGER, DATA_TYPE_FLOAT
   \param value: value to write
 */
double setE2PROMData(unsigned char device, unsigned int addr, unsigned char type, double value)
{
    uint8_t deviceAddr;

    union {
        float fdata;
        uint8_t data[4];
    } FData;

	switch(device)
    {

    case 0:    
        switch(type)
        {
        case DATA_TYPE_BYTE:
        	{
                byte b_val;
                b_val = byte(value);
                EEPROM.write(addr, b_val);
                break;
        	}
        case DATA_TYPE_INTEGER:
        	{
                int i_val = 0;
                i_val = int(value);
                EEPROM.put(addr, i_val);
                break;
        	}
        case DATA_TYPE_FLOAT:
        	{
        	    float f_val = 0.0f;
                f_val = float(value);
                EEPROM.put(addr,f_val);
                // Serial.println(f_val);
                break;
        	}
        }
        break;
    case 1:
        deviceAddr = EXTERNAL_EEPROM_USER_ADDRESS;
        break;

    case 2:
        deviceAddr = EXTERNAL_EEPROM_SYS_ADDRESS;
        break;

    default:
        return ADDRESS_ERROR;
    }       


    if (device == 1 || device == 2)
    {
        int num = 0;
        switch(type)
        {
        case DATA_TYPE_BYTE:
            {
                FData.data[0] = byte(value);
                num = 1;
                break;
            }
        case DATA_TYPE_INTEGER:
            {
                int i_val = 0;
                i_val = int(value); 
                FData.data[0] = (i_val & 0xff00) >> 8;
                FData.data[1] = i_val & 0xff;
                num = 2;
                break;
            }
        case DATA_TYPE_FLOAT:
            {
                FData.fdata = float(value);
                num = 4;
                break;
            }
        default:
            return PARAMETER_ERROR;
        }

        unsigned char i=0;
        i = (addr % 128);
        // Since the eeprom's sector is 128 byte, if we want to write 5 bytes per cycle we need to care about when there's less than 5 bytes left
        if (i >= (129-num)) 
        {
            i = 128 - i;
            iic_writebuf(FData.data, deviceAddr, addr, i);// write data
            delay(5);
            iic_writebuf(FData.data + i, deviceAddr, addr + i, num - i);// write data
        }
        //if the left bytes are greater than 5, just do it
        else
        {
            iic_writebuf(FData.data, deviceAddr, addr, num);// write data
        }      


       

    }

}

#ifdef MKII
/*!
   \brief stop move immediately
 */
void stopMove()
{
	mCurStep = -1;
}

/*!
   \brief is moving now
 */
bool isMoving()
{
	return (mCurStep != -1);
}

/*!
   \brief check if power plug in
 */
bool isPowerPlugIn()
{
    if (analogRead(POWER_DETECT) > 400)
        return true;
    else
        return false;
}
#endif 

////////////////////////////////////////////////////////////////////////////
// private functions

static void _sort(unsigned int array[], unsigned int len)
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

static void _interpolate(double startVal, double endVal, double *interpVals, int steps, byte easeType)
{

    startVal = startVal / 10.0;
    endVal = endVal / 10.0;

    double delta = endVal - startVal;
    for (byte i = 1; i <= steps; i++)
    {
        float t = (float)i / steps;
        //*(interp_vals+f) = 10.0*(start_val + (3 * delta) * (t * t) + (-2 * delta) * (t * t * t));
        *(interpVals + i - 1) = 10.0 * (startVal + t * t * delta * (3 + (-2) * t));
    }
}

static unsigned char _moveTo(double x, double y, double z, double speed)
{
	
	double angleRot = 0, angleLeft = 0, angleRight = 0;
	double curRot = 0, curLeft = 0, curRight = 0;
    double targetRot = 0;
    double targetLeft = 0;
    double targetRight = 0;
    double curX = 0;
    double curY = 0;
    double curZ = 0;
    int i = 0;
    int totalSteps = 0;
    unsigned int timePerStep;

    unsigned char status = 0;

    status = controller.xyzToAngle(x, y, z, targetRot, targetLeft, targetRight);
	debugPrint("target B=%f, L=%f, R=%f\r\n", curRot, curLeft, curRight);

    if (status == OUT_OF_RANGE_NO_SOLUTION)
    {
    	return OUT_OF_RANGE_NO_SOLUTION;
    }

    if (speed == 0)
    {
        mCurStep = -1;
        controller.writeServoAngle(targetRot, targetLeft, targetRight);
        return IN_RANGE;
    }

    // get current angles
    controller.getServoAngles(curRot, curLeft, curRight);
    // get current xyz
    controller.getCurrentXYZ(curX, curY, curZ);

	debugPrint("B=%f, L=%f, R=%f\r\n", curRot, curLeft, curRight);

    // calculate max steps
    totalSteps = max(abs(targetRot - curRot), abs(targetLeft - curLeft));
    totalSteps = max(totalSteps, abs(targetRight - curRight));

    if (totalSteps <= 0)
        return NO_NEED_TO_MOVE;

    totalSteps = totalSteps < STEP_MAX ? totalSteps : STEP_MAX;

    // calculate step time
    double distance = sqrt((x-curX) * (x-curX) + (y-curY) * (y-curY) + (z-curZ) * (z-curZ));
    speed = constrain(speed, 100, 1000);
    timePerStep = distance / speed * 1000.0 / totalSteps;


    // keep timePerStep <= STEP_MAX_TIME
    if (timePerStep > STEP_MAX_TIME)
    {
        double ratio = double(timePerStep) / STEP_MAX_TIME;

        if (totalSteps * ratio < STEP_MAX)
        {
            totalSteps *= ratio;
            timePerStep = STEP_MAX_TIME;
        }
        else
        {
            totalSteps = STEP_MAX;
            timePerStep = STEP_MAX_TIME;
        }
    }


    totalSteps = totalSteps < STEP_MAX ? totalSteps : STEP_MAX;

    debugPrint("totalSteps= %d\n", totalSteps);

    // trajectory planning
    _interpolate(curX, x, mPathX, totalSteps, INTERP_EASE_INOUT_CUBIC);
    _interpolate(curY, y, mPathY, totalSteps, INTERP_EASE_INOUT_CUBIC);
    _interpolate(curZ, z, mPathZ, totalSteps, INTERP_EASE_INOUT_CUBIC);

    for (i = 0; i < totalSteps; i++)
    {
    	status = controller.xyzToAngle(mPathX[i], mPathY[i], mPathZ[i], angleRot, angleLeft, angleRight);

    	if (status != IN_RANGE)
    	{
    		break;
    	}
    	else
    	{
    		mPathX[i] = angleRot;
    		mPathY[i] = angleLeft;
    		mPathZ[i] = angleRight;
    	}
    }

    if (i < totalSteps)
    {
    	debugPrint("i < totalSteps\r\n");
    	_interpolate(curRot, targetRot, mPathX, totalSteps, INTERP_EASE_INOUT_CUBIC);
    	_interpolate(curLeft, targetLeft, mPathY, totalSteps, INTERP_EASE_INOUT_CUBIC);
    	_interpolate(curRight, targetRight, mPathZ, totalSteps, INTERP_EASE_INOUT_CUBIC);    	
    }

    mPathX[totalSteps - 1] = targetRot;
    mPathY[totalSteps - 1] = targetLeft;
    mPathZ[totalSteps - 1] = targetRight;


    mTimePerStep = timePerStep;
    mTotalSteps = totalSteps;
    mCurStep = 0;
    mStartTime = millis();

    return IN_RANGE;
}



static void _controllerRun()
{


	while (mCurStep >= 0 && mCurStep < mTotalSteps)
	{

		if((millis() - mStartTime) >= (mCurStep * mTimePerStep)) 
		{

            // ignore the point if cannot reach
			if (controller.limitRange(mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]) != OUT_OF_RANGE_NO_SOLUTION)
			{
				debugPrint("curStep:%d, %f, %f, %f", mCurStep, mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]);
				if (mCurStep == (mTotalSteps - 1))
                {
                    double angles[3];
                    angles[0] = controller.getReverseServoAngle(0, mPathX[mCurStep]);
                    angles[1] = controller.getReverseServoAngle(1, mPathY[mCurStep]);
                    angles[2] = controller.getReverseServoAngle(2, mPathZ[mCurStep]);
                    //debugPrint("curStep:%d, %f, %f, %f", mCurStep, angles[0], angles[1], angles[2]);
                    controller.writeServoAngle(angles[0], angles[1], angles[2]);
                    //controller.writeServoAngle(mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]);
                }
                else
                
                {
                    controller.writeServoAngle(mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]);
                }
			}
              
            mCurStep++;


           	if (mCurStep >= mTotalSteps)       
           	{
           		mCurStep = -1;

               
           	}  
		}	

		manage_inactivity();
	}

}