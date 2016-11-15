/**
  ******************************************************************************
  * @file	uArmClass.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArm.h" 

#include "uArmComm.h"

uArmClass uArm;
extern uArmComm gComm;

uArmClass::uArmClass()
{
	mCurStep = -1;
	mTotalSteps = -1;
    mRecordAddr = 0;

    mReportInterval = 0; 

    mReportStartTime = millis();
    mTickStartTime = millis();
    mTickRecorderTime = millis();
}



void uArmClass::initHardware()
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



}

void uArmClass::setReportInterval(unsigned int interval)
{
    mReportInterval = interval;
}

void uArmClass::setup()
{

	initHardware();
	mController.init();

    mButtonD4.setPin(BTN_D4);
    mButtonD7.setPin(BTN_D7);

    gBuzzer.setPin(BUZZER);

    #ifdef MKII
    mLed.setPin(SYS_LED);
    #endif

    mCurStep = -1;
    mTotalSteps = -1;    


	uArm.moveTo(0, 150, 150);
    Serial.println("@1");
}


void uArmClass::controllerRun()
{


	if (mCurStep >= 0 && mCurStep < mTotalSteps)
	{

		if((millis() - mStartTime) >= (mCurStep * mTimePerStep)) 
		{

            // ignore the point if cannot reach
			if (mController.limitRange(mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]) != OUT_OF_RANGE_NO_SOLUTION)
			{
				//debugPrint("curStep:%d, %s, %s, %s", mCurStep, D(mPathX[mCurStep]), D(mPathY[mCurStep]), D(mPathZ[mCurStep]));
				/*if (mCurStep == (mTotalSteps - 1))
                {
                    double angles[3];
                    angles[0] = mController.getReverseServoAngle(0, mPathX[mCurStep]);
                    angles[1] = mController.getReverseServoAngle(1, mPathY[mCurStep]);
                    angles[2] = mController.getReverseServoAngle(2, mPathZ[mCurStep]);
                    debugPrint("curStep:%d, %s, %s, %s", mCurStep, D(angles[0]), D(angles[1]), D(angles[2]));
                    mController.writeServoAngle(angles[0], angles[1], angles[2]);
                    //mController.writeServoAngle(mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]);
                }
                else
                */
                {
                    mController.writeServoAngle(mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]);
                }
			}
              
            mCurStep++;


           	if (mCurStep >= mTotalSteps)       
           	{
           		mCurStep = -1;
               
           	}  
		}	
	}
    else
    {
        mCurStep = -1;

    }
}

void uArmClass::recorderTick()
{
    //sys led function detec every 0.05s-----------------------------------------------------------------

    switch(mSysStatus)//every 0.125s per point
    {
    case SINGLE_PLAY_MODE:
        if(play() == false)
        {
                mSysStatus = NORMAL_MODE;
                mRecordAddr = 0;
        }
        break;

    case LOOP_PLAY_MODE:

        if(play() == false)
        {
            mRecordAddr = 0;
        }
        break;

    case LEARNING_MODE:
    case LEARNING_MODE_STOP:
        if(record() == false)
        {
                mSysStatus = NORMAL_MODE;
                mRecordAddr = 0;
           
                mController.attachAllServo();

        }
        break;

    default: 
        break;
    }

}



void uArmClass::systemRun()
{
//check the button4 status------------------------------------------------------------------------

    if (mButtonD4.clicked())
    {
    	//debugPrint("btnD4 down");
        mButtonD4.clearEvent();
        switch (mSysStatus)
        {
        case NORMAL_MODE:
        case NORMAL_BT_CONNECTED_MODE:
            mSysStatus = LEARNING_MODE;
            mRecordAddr = 0;//recording/playing address
            mController.detachAllServo();
            break;

        case LEARNING_MODE:
            //LEARNING_MODE_STOP is just used to notificate record() function to stop, once record() get it then change the sys_status to normal_mode
            mSysStatus = LEARNING_MODE_STOP;//do not detec if BT is connected here, will do it seperatly
            
            mController.pumpOff();
         
            break;

        default: break;
        }
    }

    

    //check the button7 status-------------------------------------------------------------------------
    if (mButtonD7.longPressed())
    {
        mButtonD7.clearEvent();
        switch(mSysStatus)
        {
        case NORMAL_MODE:
        case NORMAL_BT_CONNECTED_MODE:
            mRecordAddr = 0;
            mSysStatus = LOOP_PLAY_MODE;
            break;

        case SINGLE_PLAY_MODE:
        case LOOP_PLAY_MODE:
            break;

        case LEARNING_MODE: 
            break;
        }
    }
    else if (mButtonD7.clicked())
    {
        mButtonD7.clearEvent();
            
    	//debugPrint("btnD7 down");

        switch(mSysStatus)
        {
        case NORMAL_MODE:
        case NORMAL_BT_CONNECTED_MODE:
            mRecordAddr = 0;//recording/playing address
            mSysStatus = SINGLE_PLAY_MODE;  // or play just one time
            
            break;

        case SINGLE_PLAY_MODE:
        case LOOP_PLAY_MODE:
            mController.pumpOff();
            mSysStatus = NORMAL_MODE;
            break;

        case LEARNING_MODE:
       
            if (mController.pumpStatus())
            {
                mController.pumpOff();
            }
            else
            {
                mController.pumpOn();
            }    
            break;
        }
    } 


    if (mReportInterval > 0)
    {
        if(millis() - mReportStartTime >= mReportInterval)
        {
            mReportStartTime = millis();
            gComm.reportPos();
        }
      
    }
	
}

void uArmClass::btDetect()
{
#ifdef MKII
    if (!gBuzzer.on() && ((mSysStatus == NORMAL_MODE) || (mSysStatus == NORMAL_BT_CONNECTED_MODE)))
    {
        pinMode(BT_DETECT_PIN, INPUT);
        digitalWrite(BT_DETECT_PIN,HIGH);

        if (digitalRead(BT_DETECT_PIN) == HIGH)//do it here
        {
            mLed.on();
            mSysStatus = NORMAL_BT_CONNECTED_MODE;
        }
        else
        {
            mLed.off();
            mSysStatus = NORMAL_MODE;
        }

        //pinMode(BT_DETECT_PIN, OUTPUT);
    }
#endif
}

void uArmClass::tickTaskRun()
{
    //recorderTick();
    mButtonD7.tick();
    mButtonD4.tick();
#ifdef MKII
    mLed.tick();
    btDetect();
#endif    
}

void uArmClass::run()
{
	gComm.run();
	controllerRun();
	systemRun();

    // if(mTime50ms != millis() % TICK_INTERVAL)   
    // {
    //     mTime50ms = millis() % TICK_INTERVAL;
    //     if(mTime50ms == 0)
    //     {
    //         tickTaskRun();
    //     }
    // }    

        if(millis() - mTickStartTime >= TICK_INTERVAL)
        {
            mTickStartTime = millis();
            tickTaskRun();
        }    


        if (millis() - mTickRecorderTime >= 50)
        {
            mTickRecorderTime= millis();
            recorderTick();
        }
}

void uArmClass::stopMove()
{
	mCurStep = -1;
}

bool uArmClass::isMoving()
{
	return (mCurStep != -1);
}


bool uArmClass::play()
{

    unsigned char data[5]; // 0: L  1: R  2: Rotation 3: hand rotation 4:gripper


    mRecorder.read(mRecordAddr, data, 5);
	debugPrint("mRecordAddr = %d, data=%d, %d, %d", mRecordAddr, data[0], data[1], data[2]);

    if(data[0] != 255)
    {
        //double x, y, z;
        //mController.getXYZFromAngle(x, y, z, (double)data[2], (double)data[0], (double)data[1]);
        moveToAngle((double)data[2], (double)data[0], (double)data[1]);
    	//mController.writeServoAngle((double)data[2], (double)data[0], (double)data[1]);
        mController.writeServoAngle(SERVO_HAND_ROT_NUM, (double)data[3]);
        unsigned char pumpStatus = mController.pumpStatus() > 0 ? 1 : 0;
        if (pumpStatus != data[4])
        {
            if (data[4])
            {
                mController.pumpOn();
            }
            else
            {
                mController.pumpOff();
            }   
        }
    }
    else
    {

        mController.pumpOff();
         
        return false;
    }

    mRecordAddr += 5;

    return true;
}

bool uArmClass::record()
{
	debugPrint("mRecordAddr = %d", mRecordAddr);

    if(mRecordAddr <= 65530)
    {
        unsigned char data[5]; // 0: L  1: R  2: Rotation 3: hand rotation 4:gripper
        if((mRecordAddr != 65530) && (mSysStatus != LEARNING_MODE_STOP))
        {
    		double rot, left, right;
    		//mController.updateAllServoAngle();
            mController.readServoAngles(rot, left, right);
			data[0] = (unsigned char)left;
			data[1] = (unsigned char)right;
            data[2] = (unsigned char)rot;
            data[3] = (unsigned char)mController.readServoAngle(SERVO_HAND_ROT_NUM);
            data[4] = mController.pumpStatus() > 0 ? 1 : 0;

            debugPrint("l=%d, r=%d, r= %d", data[0], data[1], data[2]);
        }
        else
        {
            data[0] = 255;//255 is the ending flag
            mRecorder.write(mRecordAddr, data, 5);

            return false;
        }

        mRecorder.write(mRecordAddr, data, 5);
        mRecordAddr += 5;

        return true;
    }
    else
    {
        return false;
    }

}


void uArmClass::interpolate(double startVal, double endVal, double *interpVals, int steps, byte easeType)
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

unsigned char uArmClass::moveTo(double x, double y, double z, double speed)
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

    status = mController.coordianteToAngle(x, y, z, targetRot, targetLeft, targetRight);

    debugPrint("moveTo status: %d, angle: %s, %s, %s\n", status, D(targetRot), D(targetLeft), D(targetRight));

    if (status == OUT_OF_RANGE_NO_SOLUTION)
    {
    	return OUT_OF_RANGE_NO_SOLUTION;
    }

    if (speed == 0)
    {
        mCurStep = -1;
        mController.writeServoAngle(targetRot, targetLeft, targetRight);
        return IN_RANGE;
    }

    // get current angles
    mController.getServoAngles(curRot, curLeft, curRight);
    // get current xyz
    mController.getCurrentXYZ(curX, curY, curZ);

    // calculate max steps
    totalSteps = max(abs(targetRot - curRot), abs(targetLeft - curLeft));
    totalSteps = max(totalSteps, abs(targetRight - curRight));

    if (totalSteps <= 0)
        return OUT_OF_RANGE_NO_SOLUTION;

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

    //debugPrint("totalSteps= %d\n", totalSteps);

    // trajectory planning
    interpolate(curX, x, mPathX, totalSteps, INTERP_EASE_INOUT_CUBIC);
    interpolate(curY, y, mPathY, totalSteps, INTERP_EASE_INOUT_CUBIC);
    interpolate(curZ, z, mPathZ, totalSteps, INTERP_EASE_INOUT_CUBIC);

    for (i = 0; i < totalSteps; i++)
    {
    	status = mController.coordianteToAngle(mPathX[i], mPathY[i], mPathZ[i], angleRot, angleLeft, angleRight);

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
    	interpolate(curRot, targetRot, mPathX, totalSteps, INTERP_EASE_INOUT_CUBIC);
    	interpolate(curLeft, targetLeft, mPathY, totalSteps, INTERP_EASE_INOUT_CUBIC);
    	interpolate(curRight, targetRight, mPathZ, totalSteps, INTERP_EASE_INOUT_CUBIC);    	
    }

    mPathX[totalSteps - 1] = targetRot;
    mPathY[totalSteps - 1] = targetLeft;
    mPathZ[totalSteps - 1] = targetRight;


#ifdef DEBUG 
    {
    	int i = 0;

    	for (i = 0; i < totalSteps; i++)
    	{
    		debugPrint("step%d, x=%s, y=%s, z=%s\n", i, D(mPathX[i]), D(mPathY[i]), D(mPathZ[i]));
    	}
    }
#endif   

    mTimePerStep = timePerStep;
    mTotalSteps = totalSteps;
    mCurStep = 0;
    mStartTime = millis();

    return IN_RANGE;
}

void uArmClass::interpolateEven(double startVal, double endVal, double *interpVals, int steps, byte easeType)
{

    startVal = startVal / 10.0;
    endVal = endVal / 10.0;

    double delta = endVal - startVal;
    float t = (float)delta / steps;
    for (byte i = 1; i <= steps; i++)
    {
       
        //*(interp_vals+f) = 10.0*(start_val + (3 * delta) * (t * t) + (-2 * delta) * (t * t * t));
        *(interpVals + i - 1) = 10.0 * (startVal + t * i);
    }
}


unsigned char uArmClass::moveToAngle(double targetRot, double targetLeft, double targetRight)
{
    

    double angleRot = 0, angleLeft = 0, angleRight = 0;
    double curRot = 0, curLeft = 0, curRight = 0;
    double x = 0;
    double y = 0;
    double z = 0;
    double curX = 0;
    double curY = 0;
    double curZ = 0;
    int i = 0;
    int totalSteps = 0;
    unsigned int timePerStep;

    unsigned char status = 0;

//    status = mController.coordianteToAngle(x, y, z, targetRot, targetLeft, targetRight);
    status = mController.getXYZFromAngle(x, y, z, targetRot, targetLeft, targetRight);

    debugPrint("moveTo status: %d, angle: %s, %s, %s\n", status, D(targetRot), D(targetLeft), D(targetRight));

    if (status == OUT_OF_RANGE_NO_SOLUTION)
    {
        return OUT_OF_RANGE_NO_SOLUTION;
    }

    // get current angles
    mController.getServoAngles(curRot, curLeft, curRight);
    // get current xyz
    mController.getCurrentXYZ(curX, curY, curZ);

    // calculate max steps
    totalSteps = max(abs(targetRot - curRot), abs(targetLeft - curLeft));
    totalSteps = max(totalSteps, abs(targetRight - curRight));

    if (totalSteps <= 0)
        return OUT_OF_RANGE_NO_SOLUTION;

    totalSteps = totalSteps < STEP_MAX ? totalSteps : STEP_MAX;

    // calculate step time
    //double distance = sqrt((x-curX) * (x-curX) + (y-curY) * (y-curY) + (z-curZ) * (z-curZ));
    //speed = constrain(speed, 100, 1000);
    timePerStep = (TICK_INTERVAL) / totalSteps;


    //keep timePerStep <= STEP_MAX_TIME
    // if (timePerStep > STEP_MAX_TIME)
    // {
    //     double ratio = double(timePerStep) / STEP_MAX_TIME;

    //     if (totalSteps * ratio < STEP_MAX)
    //     {
    //         totalSteps *= ratio;
    //         timePerStep = STEP_MAX_TIME;
    //     }
    //     else
    //     {
    //         totalSteps = STEP_MAX;
    //         timePerStep = STEP_MAX_TIME;
    //     }
    // }


    totalSteps = totalSteps < STEP_MAX ? totalSteps : STEP_MAX;

    //debugPrint("totalSteps= %d\n", totalSteps);

    // trajectory planning
    interpolateEven(curX, x, mPathX, totalSteps, INTERP_EASE_INOUT_CUBIC);
    interpolateEven(curY, y, mPathY, totalSteps, INTERP_EASE_INOUT_CUBIC);
    interpolateEven(curZ, z, mPathZ, totalSteps, INTERP_EASE_INOUT_CUBIC);

    for (i = 0; i < totalSteps; i++)
    {
        status = mController.coordianteToAngle(mPathX[i], mPathY[i], mPathZ[i], angleRot, angleLeft, angleRight);

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
        interpolateEven(curRot, targetRot, mPathX, totalSteps, INTERP_EASE_INOUT_CUBIC);
        interpolateEven(curLeft, targetLeft, mPathY, totalSteps, INTERP_EASE_INOUT_CUBIC);
        interpolateEven(curRight, targetRight, mPathZ, totalSteps, INTERP_EASE_INOUT_CUBIC);        
    }

    mPathX[totalSteps - 1] = targetRot;
    mPathY[totalSteps - 1] = targetLeft;
    mPathZ[totalSteps - 1] = targetRight;


#ifdef DEBUG 
    {
        int i = 0;

        for (i = 0; i < totalSteps; i++)
        {
            debugPrint("step%d, x=%s, y=%s, z=%s\n", i, D(mPathX[i]), D(mPathY[i]), D(mPathZ[i]));
        }
    }
#endif   

    mTimePerStep = timePerStep;
    mTotalSteps = totalSteps;
    mCurStep = 0;
    mStartTime = millis();

    return IN_RANGE;
}

#ifdef MKII
bool uArmClass::isPowerPlugIn()
{
    if (analogRead(POWER_DETECT) > 400)
        return true;
    else
        return false;
}
#endif 