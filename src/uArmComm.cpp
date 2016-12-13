/**
  ******************************************************************************
  * @file	gCommComm.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-10-08
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArmComm.h" 
#include "uArmRingBuffer.h"


static CommState commState = IDLE;
static unsigned char cmdReceived[COM_LEN_MAX];
static unsigned char cmdIndex = 0;

static uArmRingBuffer ringBuffer;

#define RESULT_BUFFER_SIZE  50

#define RING_BUFFER_SIZE    48
uint8_t bufData[RING_BUFFER_SIZE];

static void replyError(int serialNum, unsigned int errorCode)
{
    if (serialNum > 0)
    {
        Serial.print("$");
        Serial.print(serialNum);
        Serial.print(" ");
    }

    Serial.print("E");
    Serial.println(errorCode);   
}

static void replyOK(int serialNum)
{
    if (serialNum > 0)
    {
        Serial.print("$");
        Serial.print(serialNum);
        Serial.print(" ");
    }
    Serial.println("OK");   
}

static void replyResult(int serialNum, String result)
{
    if (serialNum > 0)
    {    
        Serial.print("$");
        Serial.print(serialNum);
        Serial.print(" ");
    }
    Serial.print("OK ");
    Serial.println(result);   
}

static void reportResult(int reportCode, String result)
{

    Serial.print("@");
    Serial.print(reportCode);
    Serial.print(" ");
    Serial.println(result);   
}

static unsigned char cmdMove(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 4)
        return PARAMETER_ERROR;

	if (moveTo(value[0], value[1], value[2], value[3]) != OUT_OF_RANGE_NO_SOLUTION)
	{
        replyOK(serialNum);
	}
	else
	{
		return OUT_OF_RANGE;
	}

    return 0;
}

static unsigned char cmdMovePol(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 4)
        return PARAMETER_ERROR;

    if (moveToPol(value[0], value[1], value[2], value[3]) != OUT_OF_RANGE_NO_SOLUTION)
    {
        replyOK(serialNum);
    }
    else
    {
        return OUT_OF_RANGE;
    }
 
    return 0;   
}

static unsigned char cmdSetAttachServo(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 1)
        return PARAMETER_ERROR;

	if (attachServo(value[0]))
    {
        replyOK(serialNum);
        return 0;
    }
    else
    {
        return OUT_OF_RANGE;
    }
  
}

static unsigned char cmdSetDetachServo(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 1)
        return PARAMETER_ERROR;

    if (detachServo(value[0]))
    {
        replyOK(serialNum);
        return 0;
    }
    else
    {
        return OUT_OF_RANGE;
    }

    return 0;
}

// static unsigned char cmdSetServoAngle(int serialNum, int parameterCount, double value[4])
// {
//     if (parameterCount != 2)
//         return PARAMETER_ERROR;

// 	uArm.mController.writeServoAngle(byte(value[0]), value[1], false);
//     replyOK(serialNum);

//     return 0;
// }

static unsigned char cmdSetServoAngleWithOffset(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 2)
        return PARAMETER_ERROR;    

	if (setServoAngle(byte(value[0]), value[1]) == OK)
    {
        replyOK(serialNum);
        return 0;
    }
    else
    {
        return PARAMETER_ERROR; 
    }

    

}

static unsigned char cmdSetPump(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 1)
        return PARAMETER_ERROR;

    if (value[0] == 0)//off
    {
        pumpOff();
    }
    else//on
    {
        pumpOn();
    }

    replyOK(serialNum);

    return 0;
}

static unsigned char cmdSetGripper(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 1)
        return PARAMETER_ERROR;

    if (value[0]==0)//release
    {
        gripperRelease();
    }
    else//catch
    {
        gripperCatch();
    }

    replyOK(serialNum);

    return 0;
}

static unsigned char cmdSetBuzz(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 2)
        return PARAMETER_ERROR;

	buzzer.buzz(value[0], value[1]*1000);    // convert to ms

    replyOK(serialNum);

    return 0;
}

#ifdef MKII
static unsigned char cmdStopMove(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

	stopMove();
    replyOK(serialNum);

    return 0;
}
#endif

static unsigned char cmdGetHWVersion(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;    

    char result[RESULT_BUFFER_SIZE];

    msprintf(result, "V%s", HW_VER);


	replyResult(serialNum, result);

    return 0;
}

static unsigned char cmdGetSWVersion(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;    

    char result[RESULT_BUFFER_SIZE];

    msprintf(result, "V%s", SW_VER);


    replyResult(serialNum, result);

    return 0;
}

static unsigned char cmdSimulatePos(int serialNum, int parameterCount, double value[4])
{


    if (parameterCount != 4)
        return PARAMETER_ERROR;

    if (value[3] == 1) //Polar
    {
        double s = value[0];
        double r = value[1];
        double h = value[2];

        polToXYZ(s, r, h, value[0], value[1], value[2]);
    }

	unsigned char status = validatePos(value[0], value[1], value[2]);


    char result[RESULT_BUFFER_SIZE];
    switch(status)
    {
    case IN_RANGE: 
    	strcpy(result, "V1");
        break;

    case OUT_OF_RANGE: 
    case OUT_OF_RANGE_NO_SOLUTION: 
    	strcpy(result, "V0");
       	break;
    default:                
    	break;
    }

    replyResult(serialNum, result);

    return 0;
}

static unsigned char cmdGetCurrentXYZ(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 0)
        return PARAMETER_ERROR;

    getCurrentXYZ(value[0], value[1], value[2]);
    
    char result[RESULT_BUFFER_SIZE];
    msprintf(result, "X%f Y%f Z%f", value[0], value[1], value[2]);

    replyResult(serialNum, result);
    return 0;	
}

static unsigned char cmdGetCurrentPosPol(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    getCurrentPosPol(value[0], value[1], value[2]);

    char result[RESULT_BUFFER_SIZE];
    msprintf(result, "S%f R%f H%f", value[0], value[1], value[2]);

    replyResult(serialNum, result);

    return 0;
}

static unsigned char cmdGetCurrentAngle(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    value[0] = getServoAngle(SERVO_ROT_NUM);
    value[1] = getServoAngle(SERVO_LEFT_NUM);
    value[2] = getServoAngle(SERVO_RIGHT_NUM);
    value[3] = getServoAngle(SERVO_HAND_ROT_NUM);

    char result[RESULT_BUFFER_SIZE];
    msprintf(result, "B%f L%f R%f H%f", value[0], value[1], value[2], value[3]);

    replyResult(serialNum, result);

    return 0;
}

static unsigned char cmdCoordinateToAngle(int serialNum, int parameterCount, double value[4])
{
    double rot, left, right;

    if (parameterCount != 3)
        return PARAMETER_ERROR;

    xyzToAngle(value[0], value[1], value[2], rot, left, right);

    value[0] = rot;
    value[1] = left;
    value[2] = right;

    char result[RESULT_BUFFER_SIZE];
    msprintf(result, "B%f L%f R%f", value[0], value[1], value[2]);

    replyResult(serialNum, result);


    return 0;
}

static unsigned char cmdAngleToXYZ(int serialNum, int parameterCount, double value[4])
{
    double x, y, z;
    bool success;

    if (parameterCount != 3)
        return PARAMETER_ERROR;

    if(angleToXYZ(value[0], value[1], value[2], x, y, z) == OUT_OF_RANGE)
    {
        success = false;
    }
    else
    {
        success = true;
    }

    value[0] = x;
    value[1] = y;
    value[2] = z;

    char result[RESULT_BUFFER_SIZE];
    msprintf(result, "X%f Y%f Z%f", value[0], value[1], value[2]);

    replyResult(serialNum, result);
    return 0;
}

#ifdef MKII
static unsigned char cmdIsMoving(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    char result[RESULT_BUFFER_SIZE];
	if(isMoving())
	{
        strcpy(result, "V1");
	}
	else
	{
        strcpy(result, "V0");
	}

    replyResult(serialNum, result);

    return 0;
}
#endif

static unsigned char cmdGetTip(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    char result[RESULT_BUFFER_SIZE];


    if(getTip())
    {
        strcpy(result, "V1");
    }
    else
    {
        strcpy(result, "V0");
    }

     replyResult(serialNum, result);

    return 0;
}

static unsigned char cmdGetDigitValue(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 1)
        return PARAMETER_ERROR;

    int val = getDigitalPinValue(value[0]);
 
    char result[RESULT_BUFFER_SIZE];
    msprintf(result, "V%d", val);

    replyResult(serialNum, result);
    return 0;
}

static unsigned char cmdSetDigitValue(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 2)
        return PARAMETER_ERROR;

    setDigitalPinValue(value[0], value[1]);

    replyOK(serialNum);
    return 0;
}

static unsigned char cmdGetAnalogValue(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 1)
        return PARAMETER_ERROR;

    int val = getAnalogPinValue(value[0]);

    char result[RESULT_BUFFER_SIZE];
    msprintf(result, "V%d", val);

    replyResult(serialNum, result);
    return 0;
}

static unsigned char cmdGetE2PROMData(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 3)
        return PARAMETER_ERROR;    

    char result[RESULT_BUFFER_SIZE];

    

    int device = int(value[0]);
    int type = value[2];
    uint32_t addr = value[1];

    double resultVal = getE2PROMData(device, addr, type);

    switch(type)
    {
    case DATA_TYPE_BYTE:
        {
            int val = resultVal;
            msprintf(result, "V%d", val);
            break;
        }
    case DATA_TYPE_INTEGER:
        {
            int i_val = resultVal;
            msprintf(result, "V%d", i_val);
            break;
        }
    case DATA_TYPE_FLOAT:
        {
            double f_val = resultVal;
            msprintf(result, "V%f", f_val);
            break;
        }
    }

    replyResult(serialNum, result);    

    return 0;

}

static unsigned char cmdSetE2PROMData(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 4)
        return PARAMETER_ERROR;    
    

    
    int type = value[2];
    int device = int(value[0]);
    uint32_t addr = value[1];

    setE2PROMData(device, addr, type, value[3]);

    replyOK(serialNum);

    return 0;

}

static unsigned char cmdSetButtonService(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 1)
        return PARAMETER_ERROR;

    if (value[0])
    {
        service.setButtonService(true); 
    }
    else
    {
        service.setButtonService(false); 
    }

    replyOK(serialNum);
    return 0;
}

static unsigned char cmdGetGripperStatus(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    unsigned char status = getGripperStatus();

    char result[RESULT_BUFFER_SIZE];
    msprintf(result, "V%d", status);
    replyResult(serialNum, result);
    return 0;
}





static unsigned char cmdGetPumpStatus(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 0)
        return PARAMETER_ERROR;

    char result[RESULT_BUFFER_SIZE];

    unsigned char status = getPumpStatus();

#ifdef MKII     
    
    msprintf(result, "V%d", status);

#elif defined(METAL)

    if (status)
        strcpy(result, "V1");
    else
        strcpy(result, "V0");

#endif

    replyResult(serialNum, result);

    return 0;
}


#ifdef MKII 
static unsigned char cmdGetPowerStatus(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    char result[RESULT_BUFFER_SIZE];

    if (isPowerPlugIn())
        strcpy(result, "V1");
    else
        strcpy(result, "V0");   

    replyResult(serialNum, result);

    return 0;
}
#endif 

// static unsigned char cmdGetServoAnalogData(int serialNum, int parameterCount, double value[4])
// {
//     if (parameterCount != 1)
//         return PARAMETER_ERROR;

//     unsigned int data = uArm.mController.getServoAnalogData(value[0]);
//     //Serial.println(data);
//     char result[RESULT_BUFFER_SIZE];


//     msprintf(result, "V%d", result);   

//     replyResult(serialNum, result);

//     return 0;
// }


static unsigned char cmdRelativeMove(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 4)
        return PARAMETER_ERROR;

    if (relativeMove(value[0], value[1], value[2], value[3]) != OUT_OF_RANGE_NO_SOLUTION)
    {
        replyOK(serialNum);
    }
    else
    {
        return OUT_OF_RANGE;
    }

    return 0;
}


static unsigned char cmdSetReportInterval(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 1)
        return PARAMETER_ERROR;


    service.setReportInterval(value[0]*1000);

    replyOK(serialNum);


    return 0;
}

static unsigned char cmdGetDeviceName(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    char result[RESULT_BUFFER_SIZE];

    msprintf(result, "V%s", DEVICE_NAME);
 

    replyResult(serialNum, result);

    return 0;
}

static unsigned char cmdGetAPIVersion(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    char result[RESULT_BUFFER_SIZE];

    msprintf(result, "V%s", SW_VER);
 

    replyResult(serialNum, result);

    return 0;
}

static unsigned char cmdGetDeviceUUID(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;


    char result[RESULT_BUFFER_SIZE];

    strcpy(result, "V1234567890");   

    replyResult(serialNum, result);

    return 0;
}

void reportButtonEvent(unsigned char buttonId, unsigned char event)
{
    char result[RESULT_BUFFER_SIZE];
    msprintf(result, "B%d V%d", buttonId, event); 
    reportResult(REPORT_BUTTON, result);  
}


void reportPos()
{
    double x, y, z, frontEndAngle;

    getCurrentXYZ(x, y, z);
    frontEndAngle = getServoAngle(SERVO_HAND_ROT_NUM);

    debugPrint("angle = %f", frontEndAngle);
    char result[RESULT_BUFFER_SIZE];
    msprintf(result, "X%f Y%f Z%f R%f", x, y, z, frontEndAngle);   

    reportResult(REPORT_POS, result);    

}

static void HandleMoveCmd(int cmdCode, int serialNum, int parameterCount, double value[4])
{
    unsigned char result = false;

    switch (cmdCode)
    {
    case 0:
        result = cmdMove(serialNum, parameterCount, value);
        break;

    case 201:
        result = cmdMovePol(serialNum, parameterCount, value);
        break;

    case 202:
        result = cmdSetServoAngleWithOffset(serialNum, parameterCount, value);
        break;

#ifdef MKII
    case 203:
        result = cmdStopMove(serialNum, parameterCount, value);
        break;
#endif

    case 204:
        result = cmdRelativeMove(serialNum, parameterCount, value);
        break;
        
    default:
        replyError(serialNum, NO_SUCH_CMD);
        return;
    }

    if (result > 0)
    {
        Serial.print("$");
        Serial.print(serialNum);
        Serial.print(" ");
        Serial.print("E");
        Serial.println(result);
    }
}

static void HandleSettingCmd(int cmdCode, int serialNum, int parameterCount, double value[4])
{
    unsigned char result = false;

    switch (cmdCode)
    {
    case 120:
        result = cmdSetReportInterval(serialNum, parameterCount, value);
        break;

#ifdef MKII
    case 200:
        result = cmdIsMoving(serialNum, parameterCount, value);
        break;
#endif

    case 201:
        result = cmdSetAttachServo(serialNum, parameterCount, value);
        break;

    case 202:
        result = cmdSetDetachServo(serialNum, parameterCount, value);
        break;

    case 210:
        result = cmdSetBuzz(serialNum, parameterCount, value);
        break;        

    case 211:
        result = cmdGetE2PROMData(serialNum, parameterCount, value);
        break;

    case 212:
        result = cmdSetE2PROMData(serialNum, parameterCount, value);
        break;

    case 213:
        result = cmdSetButtonService(serialNum, parameterCount, value);
        break;        

    case 220:
        result = cmdCoordinateToAngle(serialNum, parameterCount, value);
        break;

    case 221:
        result = cmdAngleToXYZ(serialNum, parameterCount, value);
        break;

    case 222:
        result = cmdSimulatePos(serialNum, parameterCount, value);
        break;

    case 231:
        result = cmdSetPump(serialNum, parameterCount, value);
        break;

    case 232:
        result = cmdSetGripper(serialNum, parameterCount, value);
        break;

    case 240:
        result = cmdSetDigitValue(serialNum, parameterCount, value);
        break;



    default:
        replyError(serialNum, NO_SUCH_CMD);
        return;
    }

    if (result > 0)
    {
        Serial.print("$");
        Serial.print(serialNum);
        Serial.print(" ");
        Serial.print("E");
        Serial.println(result);
    }
}

static void HandleQueryCmd(int cmdCode, int serialNum, int parameterCount, double value[4])
{
    unsigned char result = false;

    switch (cmdCode)
    {
    case 200:
        result = cmdGetCurrentAngle(serialNum, parameterCount, value);
        break;

    case 201:
        result = cmdGetDeviceName(serialNum, parameterCount, value);
        break;

    case 202:
        result = cmdGetHWVersion(serialNum, parameterCount, value);
        break;

    case 203:
        result = cmdGetSWVersion(serialNum, parameterCount, value);
        break;        

    case 204:
        result = cmdGetAPIVersion(serialNum, parameterCount, value);
        break;

    case 205:
        result = cmdGetDeviceUUID(serialNum, parameterCount, value);
        break;        

    case 220:
        result = cmdGetCurrentXYZ(serialNum, parameterCount, value);
        break;

    case 221:
        result = cmdGetCurrentPosPol(serialNum, parameterCount, value);
        break;

    case 231:
        result = cmdGetPumpStatus(serialNum, parameterCount, value);
        break;

    case 232:
        result = cmdGetGripperStatus(serialNum, parameterCount, value);
        break;

    case 233:
        result = cmdGetTip(serialNum, parameterCount, value);
        break;    

    case 240:
        result = cmdGetDigitValue(serialNum, parameterCount, value);
        break;

    case 241:
        result = cmdGetAnalogValue(serialNum, parameterCount, value);
        break;

    default:
        replyError(serialNum, NO_SUCH_CMD);
        return;
    }

    if (result > 0)
    {
       replyError(serialNum, result);
    }
}

static bool parseCommand(char *message)
{
    double value[6];
    int index = 0;
    bool hasSerialNum = false;
    debugPrint("message=%s", message);


    int len = strlen(message);

    char *pch;
    char cmdType;

    // skip white space
    while (len > 0)
    {
        if (isspace(message[len-1]))
        {
            message[len-1] = '\0';
        }
        else
        {
            break;
        }

        len--;
    }

    if (len <= 0)
        return false;



    

    if (message[0] == '#')
    {
        hasSerialNum = true;
    }

    pch = strtok(message, " ");
    while (pch != NULL && index < 6)
    {
        //debugPrint("pch=%s", pch);
        
        switch (index)
        {
        case 0:
            if (!hasSerialNum)
            {
                cmdType = *(pch);
            }
            value[index] = atof(pch+1);
            break;

        case 1:
            if (hasSerialNum)
            {
                cmdType = *(pch);
            }
            //debugPrint("cmdType=%d", cmdType);
            value[index] = atof(pch+1);
            break;

        default:
            value[index] = atof(pch+1);
            break;
        }


        //debugPrint("value=%f", value[index]);

        pch = strtok(NULL, " ");


        index++;

    }

    int serialNum = 0;
    int cmdCode = 0;
    int parameterCount = 0;
    int valueStartIndex = 0;

    if (hasSerialNum)
    {
        serialNum = value[0];
        cmdCode = value[1];
        parameterCount = index - 2;
        valueStartIndex = 2;
    }
    else
    {
        serialNum = 0;
        cmdCode = value[0];
        parameterCount = index - 1;        
        valueStartIndex = 1;
    }

    switch (cmdType)
    {
    case 'G':
        HandleMoveCmd(cmdCode, serialNum, parameterCount, value + valueStartIndex);
        break;

    case 'M':
        HandleSettingCmd(cmdCode, serialNum, parameterCount, value + valueStartIndex);
        break;

    case 'P':
        HandleQueryCmd(cmdCode, serialNum, parameterCount, value + valueStartIndex);
        break;

    }

}

static void handleSerialData(char data)
{
    static unsigned char cmdCount = 0;


    switch (commState)
    {
    case IDLE:
        if (data == '#' || data == 'G' || data == 'M' || data == 'P')
        {
            commState = START;
            cmdIndex = 0;
            if (data != '#')
            {
                cmdCount = 1;   // get cmd code
            }
            else
            {
                cmdCount = 0;
            }

            cmdReceived[cmdIndex++] = data;
        }
        break;
        
    case START:
        
        if (cmdIndex >= COM_LEN_MAX)
        {

            commState = IDLE;
        }
        else if (data == '#')   // restart 
        {
            cmdIndex = 0;
            cmdCount = 0;
            cmdReceived[cmdIndex++] = data;
        } 
        else if (data == 'G' || data == 'M' || data == 'P')    
        {
            if (cmdCount >= 1)  // restart
            {
                cmdIndex = 0;
                cmdReceived[cmdIndex++] = data;
            }
            else
            {
                cmdCount++;
                cmdReceived[cmdIndex++] = data;
            }
        }
        else if (data == '\n')
        {

            cmdReceived[cmdIndex] = '\0';

            parseCommand((char*)cmdReceived);
            commState = IDLE;
        }
        else
        {

            cmdReceived[cmdIndex++] = data;
        }
        break;
        
    default:
        commState = IDLE;
        break;
              
    }
}

void serialCmdRun()
{

    char data = -1;

    while (Serial.available())
    {
        data = Serial.read();

        if (data == -1)
        {
            return ;
        }
        else
        {
            handleSerialData(data);
        }
    }
}

void getSerialCmd()
{
    int data = -1;

    while (Serial.available())
    {
        data = Serial.read();

        if (data != -1)
        {
            ringBuffer.put((uint8_t)data);
        }
    }    
}

void handleSerialCmd()
{
    uint8_t data = 0;

    while (ringBuffer.get(&data))
    {
        handleSerialData(data);
    }
}

void serialCmdInit()
{
    ringBuffer.init(bufData, RING_BUFFER_SIZE);
}

