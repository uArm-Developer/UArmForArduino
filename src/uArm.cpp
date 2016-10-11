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
}

void uArmClass::initHardware()
{
	pinMode(BTN_D4, INPUT_PULLUP);        //special mode for calibration
	pinMode(BUZZER, OUTPUT);
	pinMode(LIMIT_SW, INPUT_PULLUP);
	pinMode(BTN_D7, INPUT_PULLUP);
	pinMode(PUMP_EN, OUTPUT);
	pinMode(VALVE_EN, OUTPUT);
	pinMode(GRIPPER, OUTPUT);
}

void uArmClass::setup()
{
	initHardware();
	mController.init();

    mCurStep = -1;
    mTotalSteps = -1;    
}


void uArmClass::controllerRun()
{
	if (mCurStep >= 0 && mCurStep < mTotalSteps)
	{
		if((millis() - mStartTime) >= (mCurStep * mTimePerStep))
		{
			if (mController.limitRange(mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]) != OUT_OF_RANGE_NO_SOLUTION)
			{
				debugPrint("curStep:%d, %s, %s, %s", mCurStep, D(mPathX[mCurStep]), D(mPathY[mCurStep]), D(mPathZ[mCurStep]));
				mController.writeServoAngle(mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]);
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

void uArmClass::systemRun()
{
//check the button4 status------------------------------------------------------------------------
    if(digitalRead(BTN_D4) == LOW)//check the D4 button
    {
        delay(50);
            //Serial.println("Test: whether get into learning mode");
        if(digitalRead(BTN_D4) == LOW)
        {
        	debugPrint("btnD4 down");
        	mBuzzer.buzz(4000, 100);
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
                break;

            default: break;
            }
        }
        while(digitalRead(BTN_D4)==LOW) ; // make sure button is released
    }

    //check the button7 status-------------------------------------------------------------------------
    if(digitalRead(BTN_D7) == LOW)//check the D7 button
    {
        delay(50);
        if(digitalRead(BTN_D7) == LOW)
        {
                //Serial.println("Test: whether BTN_D7 useful");
        	debugPrint("btnD7 down");
        	mBuzzer.buzz(4000, 100);
            switch(mSysStatus)
            {
            case NORMAL_MODE:
            case NORMAL_BT_CONNECTED_MODE:
            	delay(100);
            	mBuzzer.stop();
                delay(900);
                mRecordAddr = 0;//recording/playing address

                if(digitalRead(BTN_D7)==LOW)		// ??延时1秒后,如果还是低,循环播放
                    mSysStatus = LOOP_PLAY_MODE;
                else
                    mSysStatus = SINGLE_PLAY_MODE;
                break;

            case SINGLE_PLAY_MODE:
            case LOOP_PLAY_MODE:
                mSysStatus = NORMAL_MODE;
                break;

            case LEARNING_MODE:
                break;
            }
        } 

        while (digitalRead(BTN_D7)==LOW) ; // make sure button is released
    }

    //sys led function detec every 0.05s-----------------------------------------------------------------
    if(mTime50ms != millis() % 50)
    {
        mTime50ms = millis() % 50;
        if(mTime50ms == 0)
        {
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
    }		
}

void uArmClass::run()
{
	gComm.run();
	mBuzzer.run();
	controllerRun();
	systemRun();
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
    	mController.writeServoAngle((double)data[2], (double)data[0], (double)data[1]);
    }
    else
    {
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
    		mController.updateAllServoAngle();
            mController.getServoAngles(rot, left, right);
			data[0] = (unsigned char)left;
			data[1] = (unsigned char)right;
            data[2] = (unsigned char)rot;
            //data[3] = (unsigned char)cur_hand;

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

unsigned char uArmClass::moveTo(double x, double y, double z, double speed = 100)
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

    unsigned char status = 0;

    status = mController.coordianteToAngle(x, y, z, targetRot, targetLeft, targetRight);

    debugPrint("moveTo status: %d, angle: %s, %s, %s\n", status, D(targetRot), D(targetLeft), D(targetRight));

    if (status == OUT_OF_RANGE_NO_SOLUTION)
    {
    	return OUT_OF_RANGE_NO_SOLUTION;
    }

    // get current angles
    mController.getServoAngles(curRot, curLeft, curRight);

    // calculate max steps
    totalSteps = max(abs(targetRot - curRot), abs(targetLeft - curLeft));
    totalSteps = max(totalSteps, abs(targetRight - curRight));

    totalSteps = totalSteps < STEP_MAX ? totalSteps : STEP_MAX;

    if (totalSteps <= 0)
    	return OUT_OF_RANGE_NO_SOLUTION;

    debugPrint("totalSteps= %d\n", totalSteps);

    // get current xyz
    mController.getCurrentXYZ(curX, curY, curZ);

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


    // calculate step time
    double distance = sqrt((x-curX) * (x-curX) + (y-curY) * (y-curY) + (z-curZ) * (z-curZ));
    speed = constrain(speed, 100, 1000);
    mTimePerStep = distance / speed * 1000.0 / totalSteps;
    mTotalSteps = totalSteps;
    mCurStep = 0;
    mStartTime = millis();

    return IN_RANGE;
}

