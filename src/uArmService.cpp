/**
  ******************************************************************************
  * @file	uArmService.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArm.h" 

#include "uArmComm.h"

uArmService service;



uArmService::uArmService()
{

    mRecordAddr = 0;

    mReportInterval = 0; 

    mButtonServiceDisable = false;

    mReportStartTime = millis();

    mTickRecorderTime = millis();
}

void uArmService::setButtonService(bool on)
{
    if (on)
    {
        mButtonServiceDisable = false;
    }
    else
    {
        mButtonServiceDisable = true;
    }
}



void uArmService::setReportInterval(unsigned int interval)
{
    mReportInterval = interval;
}

void uArmService::init()
{

}




void uArmService::recorderTick()
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
           
                controller.attachAllServo();

        }
        break;

    default: 
        break;
    }

}



void uArmService::systemRun()
{
//check the button4 status------------------------------------------------------------------------
    if (mButtonServiceDisable)
    {
        if (buttonMenu.longPressed())
        {
            buttonMenu.clearEvent();
            reportButtonEvent(0, EVENT_LONG_PRESS);
        }
        else if (buttonMenu.clicked())
        {
            //debugPrint("btnD4 down");
            buttonMenu.clearEvent();
            reportButtonEvent(0, EVENT_CLICK);
        }

        

        //check the button7 status-------------------------------------------------------------------------
        if (buttonPlay.longPressed())
        {
            buttonPlay.clearEvent();
            reportButtonEvent(1, EVENT_LONG_PRESS);
        }
        else if (buttonPlay.clicked())
        {
            buttonPlay.clearEvent();
            reportButtonEvent(1, EVENT_CLICK);
        }         
    }
    else
    {
        if (buttonMenu.clicked())
        {
        	//debugPrint("btnD4 down");
            buttonMenu.clearEvent();
            switch (mSysStatus)
            {
            case NORMAL_MODE:
            case NORMAL_BT_CONNECTED_MODE:
                mSysStatus = LEARNING_MODE;
                mRecordAddr = 0;//recording/playing address
                controller.detachAllServo();
                break;

            case LEARNING_MODE:
                //LEARNING_MODE_STOP is just used to notificate record() function to stop, once record() get it then change the sys_status to normal_mode
                mSysStatus = LEARNING_MODE_STOP;//do not detec if BT is connected here, will do it seperatly
                
                pumpOff();
             
                break;

            default: break;
            }
        }

        

        //check the button7 status-------------------------------------------------------------------------
        if (buttonPlay.longPressed())
        {
            buttonPlay.clearEvent();
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
        else if (buttonPlay.clicked())
        {
            buttonPlay.clearEvent();
                
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
                pumpOff();
                mSysStatus = NORMAL_MODE;
                break;

            case LEARNING_MODE:
           
                if (getPumpStatus())
                {
                    pumpOff();
                }
                else
                {
                    pumpOn();
                }    
                break;
            }
        } 
    }


    if (mReportInterval > 0)
    {
        if(millis() - mReportStartTime >= mReportInterval)
        {
            mReportStartTime = millis();
            reportPos();
        }
      
    }
	
}

void uArmService::btDetect()
{
#ifdef MKII
    if (!buzzer.on() && ((mSysStatus == NORMAL_MODE) || (mSysStatus == NORMAL_BT_CONNECTED_MODE)))
    {
        pinMode(BT_DETECT_PIN, INPUT);
        digitalWrite(BT_DETECT_PIN,HIGH);

        if (digitalRead(BT_DETECT_PIN) == HIGH)//do it here
        {
            ledRed.on();
            mSysStatus = NORMAL_BT_CONNECTED_MODE;
        }
        else
        {
            ledRed.off();
            mSysStatus = NORMAL_MODE;
        }

        //pinMode(BT_DETECT_PIN, OUTPUT);
    }
#endif
}


void uArmService::run()
{

	systemRun();

    if (millis() - mTickRecorderTime >= 50)
    {
        mTickRecorderTime= millis();
        recorderTick();
    }
}




bool uArmService::play()
{

    unsigned char data[5]; // 0: L  1: R  2: Rotation 3: hand rotation 4:gripper


    recorder.read(mRecordAddr, data, 5);
	debugPrint("mRecordAddr = %d, data=%d, %d, %d", mRecordAddr, data[0], data[1], data[2]);

    if(data[0] != 255)
    {
        //double x, y, z;
        //controller.getXYZFromAngle(x, y, z, (double)data[2], (double)data[0], (double)data[1]);
        //moveToAngle((double)data[2], (double)data[0], (double)data[1]);
    	controller.writeServoAngle((double)data[2], (double)data[0], (double)data[1]);
        controller.writeServoAngle(SERVO_HAND_ROT_NUM, (double)data[3]);
        unsigned char pumpStatus = getPumpStatus() > 0 ? 1 : 0;
        if (pumpStatus != data[4])
        {
            if (data[4])
            {
                pumpOn();
            }
            else
            {
                pumpOff();
            }   
        }
    }
    else
    {

        pumpOff();
         
        return false;
    }

    mRecordAddr += 5;

    return true;
}

bool uArmService::record()
{
	debugPrint("mRecordAddr = %d", mRecordAddr);

    if(mRecordAddr <= 65530)
    {
        unsigned char data[5]; // 0: L  1: R  2: Rotation 3: hand rotation 4:gripper
        if((mRecordAddr != 65530) && (mSysStatus != LEARNING_MODE_STOP))
        {
    		double rot, left, right;
    		//controller.updateAllServoAngle();
            controller.readServoAngles(rot, left, right);
			data[0] = (unsigned char)left;
			data[1] = (unsigned char)right;
            data[2] = (unsigned char)rot;
            data[3] = (unsigned char)controller.readServoAngle(SERVO_HAND_ROT_NUM);
            data[4] = getPumpStatus() > 0 ? 1 : 0;

            debugPrint("l=%d, r=%d, r= %d", data[0], data[1], data[2]);
        }
        else
        {
            data[0] = 255;//255 is the ending flag
            recorder.write(mRecordAddr, data, 5);

            return false;
        }

        recorder.write(mRecordAddr, data, 5);
        mRecordAddr += 5;

        return true;
    }
    else
    {
        return false;
    }

}




