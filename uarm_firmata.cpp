#include <ConfigurableFirmata.h>

#include <string.h>
#include "uarm_firmata.h"


/* Constructor */
UArmFirmata::UArmFirmata()
{

}


boolean UArmFirmata::handleSysex(byte command, byte argc, byte *argv)
{
        if (command == UARM)
        {
                byte uarmCommand;
                uarmCommand = argv[0];

                // CMD 10 Read Angle
                if (uarmCommand == READ_ANGLE)
                {
                        byte servo_num = argv[1];
                        boolean withOffset = argv[2]; // if servo_offset = 0 there is offset inside
                        Firmata.write(START_SYSEX);
                        Firmata.write(UARM);

                        float angle = uarm.readAngle(servo_num,withOffset);
                        sendFloatAsThree7bitBytes(angle);

                        Firmata.write(END_SYSEX);
                        return true;
                }

                // CMD 11 Write Angle
                if (uarmCommand == WRITE_ANGLE)
                {
                        byte servo_num = argv[1];
                        double servo_angle = argv[2] + (argv[3] << 7) + float(argv[4])/100;
                        boolean writeWithOffset = argv[5];
                        uarm.writeServoAngle(servo_num,servo_angle,writeWithOffset);
                        return true;
                }

                // CMD 12 Read Coords
                if (uarmCommand == READ_COORDS)
                {
                        boolean readTrigger = argv[1];
                        if (readTrigger == 1)
                        {
                                Firmata.write(START_SYSEX);
                                Firmata.write(UARM);
                                uarm.calXYZ();
                                float x = uarm.getCalX();
                                float y = uarm.getCalY();
                                float z = uarm.getCalZ();
                                sendFloatAsFour7bitBytes(x);
                                sendFloatAsFour7bitBytes(y);
                                sendFloatAsFour7bitBytes(z);

                                Firmata.write(END_SYSEX);
                        }
                        return true;
                }

                // CMD 13 Write Coords
                if (uarmCommand == WRITE_COORDS)
                {
                        float x = argv[2] + (argv[3] << 7) + float(argv[4])/100;
                        if(argv[1] == 1)
                                x = -x;
                        float y = argv[6] + (argv[7] << 7) + float(argv[8])/100;
                        if(argv[5] == 1)
                                y = -y;
                        float z = argv[10] + (argv[11] << 7) + float(argv[12])/100;
                        if(argv[9] == 1)
                                z = -z;
                        float hand_angle = argv[13] + (argv[14] << 7) + float(argv[15])/100;
                        byte relative_flags = argv[16];
                        float time_spend = argv[17] + (argv[18] << 7) + float(argv[19])/100;
                        byte path_type = argv[20];
                        byte ease_type = argv[21];
                        // boolean enable_hand = argv[22];
                        delay(5);
                        uarm.moveToOpts(x,y,z,hand_angle,relative_flags,time_spend,path_type,ease_type);
                        delay(10);
                }

                // CMD 14 Read Digital
                if (uarmCommand == READ_DIGITAL)
                {
                        byte pin_num = argv[1];
                        byte pin_mode = argv[2]; // 0 means input   1 means input_pullup
                        {

                                Firmata.write(START_SYSEX);
                                Firmata.write(UARM);

                                pin_mode == 1 ? pinMode(pin_num, INPUT_PULLUP) : pinMode(pin_num, INPUT);

                                Firmata.write(digitalRead(pin_num));

                                Firmata.write(END_SYSEX);
                        }
                        return true;
                }

                // CMD 15 Write Digital
                if (uarmCommand == WRITE_DIGITAL)
                {
                        byte pin_num= argv[1];
                        pinMode(pin_num, OUTPUT);

                        byte pin_mode = argv[2];
                        pin_mode == 1 ? digitalWrite(pin_num,HIGH) : digitalWrite(pin_num,LOW);

                        return true;
                }

                // CMD 16 Read Analog
                if (uarmCommand == READ_ANALOG)
                {
                        byte pin_num = argv[1];
                        {

                                Firmata.write(START_SYSEX);
                                Firmata.write(UARM);
                                Firmata.sendValueAsTwo7bitBytes(analogRead(pin_num));
                                Firmata.write(END_SYSEX);
                        }
                        return true;
                }

                // CMD 17 Write Analog
                if (uarmCommand == WRITE_ANALOG)
                {
                        byte pin_num= argv[1];
                        pinMode(pin_num, OUTPUT);

                        double analog_val = argv[2] + (argv[3] << 7);
                        analogWrite(pin_num,constrain(analog_val,0,255));

                        return true;
                }

                // CMD 1A Read EEPROM
                if (uarmCommand == READ_EEPROM)
                {
                        byte data_type = argv[1];
                        byte eeprom_add = argv[2] + (argv[3] << 7);

                        Firmata.write(START_SYSEX);
                        Firmata.write(UARM);
                        Firmata.write(READ_EEPROM);
                        switch(data_type)
                        {
                        case DATA_TYPE_BYTE:
                        {
                                Firmata.sendValueAsTwo7bitBytes(EEPROM.read(eeprom_add));
                                break;
                        }
                        case DATA_TYPE_INTEGER:
                        {
                                int i_val = 0;
                                sendIntegerAsThree7bitBytes(EEPROM.get(eeprom_add, i_val));
                                break;
                        }
                        case DATA_TYPE_FLOAT:
                        {
                                float f_val = 0.0f;
                                sendFloatAsFour7bitBytes(EEPROM.get(eeprom_add,f_val));
                                break;
                        }
                        }
                        Firmata.write(END_SYSEX);
                        return true;
                }

                // CMD 1B Write EEPROM
                if (uarmCommand == WRITE_EEPROM)
                {
                        byte data_type = argv[1];
                        byte eeprom_add = argv[2] + (argv[3] << 7);
                        // byte eeprom_val = argv[3] + (argv[4] << 7);
                        // EEPROM.write(eeprom_add,eeprom_val);
                        switch(data_type)
                        {
                        case DATA_TYPE_BYTE:
                        {
                                EEPROM.write(eeprom_add,argv[4] + (argv[5] << 7));
                                break;
                        }
                        case DATA_TYPE_INTEGER:
                        {
                                int int_val = 0;
                                int_val = argv[5] + (argv[6] << 7);
                                int_val = ((argv[4] == 1) ? -int_val : int_val);
                                EEPROM.put(eeprom_add, int_val);
                                break;
                        }
                        case DATA_TYPE_FLOAT:
                        {
                                float f_val = argv[5] + (argv[6] << 7) + float(argv[7])/100;
                                f_val = ((argv[4] == 1) ? -f_val : f_val);
                                EEPROM.put(eeprom_add, f_val);
                                break;
                        }
                        }
                        return true;
                }

                // CMD 1C Servo Attach or Detach
                if (uarmCommand == DETACH_SERVO)
                {
                        uarm.detachAll();
                        return true;
                }

                // CMD 1D Pump Status
                if (uarmCommand == PUMP_STATUS)
                {
                        byte pump_status = argv[1];
                        pump_status == 1 ? uarm.pumpOn() : uarm.pumpOff();
                        return true;
                }
                // CMD 20 GRIPPER Status
                if (uarmCommand == GRIPPER_STATUS)
                {
                        byte gripper_status = argv[1];
                        gripper_status == 1 ? uarm.gripperCatch() : uarm.gripperRelease();
                        return true;
                }

                if (uarmCommand == WRITE_STRETCH)
                {
                        double length = argv[2] + (argv[3] << 7) + float(argv[4])/100;
                        length = argv[1] == 1 ? -length : length;
                        double height = argv[6] + (argv[7] << 7) + float(argv[8])/100;
                        height = argv[5] == 1 ? -height : height;
                        uarm.writeStretch(length,height);
                        return true;
                }

                if (uarmCommand == WRITE_LEFT_RIGHT_ANGLE)
                {
                        double left = argv[1] + (argv[2] << 7) + float(argv[3])/100;
                        double right = argv[4] + (argv[5] << 7) + float(argv[6])/100;
                        boolean withOffset = argv[7];
                        uarm.writeLeftRightServoAngle(left,right, withOffset);
                        return true;
                }

                if (uarmCommand == 0xBB)
                {
                        uarm.readServoOffset();
                        return true;
                }

        }
        return false;
}


void UArmFirmata::reset()
{

}

void UArmFirmata::report()
{

}


boolean UArmFirmata::handlePinMode(byte pin, int mode)
{
        return true;
}

void UArmFirmata::handleCapability(byte pin)
{

}

void UArmFirmata::sendFloatAsFour7bitBytes(float val){
        int int_val = val;
        int decimal_val = int(round((val - int_val)*100));
        Firmata.write(val > 0 ? 0 : 1);
        Firmata.write(abs(int_val) & B01111111 );
        Firmata.write(abs(int_val) >> 7 & B01111111);
        Firmata.write(abs(decimal_val) & 0x7F );
}

void UArmFirmata::sendFloatAsThree7bitBytes(float val){
        int int_val = val;
        int decimal_val = int(round((val - int_val)*100));
        Firmata.write(abs(int_val) & B01111111 );
        Firmata.write(abs(int_val) >> 7 & B01111111);
        Firmata.write(abs(decimal_val) & 0x7F );
}

void UArmFirmata::sendIntegerAsThree7bitBytes(int val){
        int symbol = val > 0 ? 0 : 1;
        Firmata.write(symbol);
        Firmata.write(abs(val) & B01111111 );
        Firmata.write(abs(val) >> 7 & B01111111);
}
