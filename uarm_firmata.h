
#ifndef UARMFIRMATA_h
#define UARMFIRMATA_h


#include <ConfigurableFirmata.h>
#include <FirmataFeature.h>

#define UARM						(0XAA)

#define READ_ANGLE                  (0X10)
#define WRITE_ANGLE                 (0X11)
#define READ_COORDS                 (0X12)
#define WRITE_COORDS                (0X13)
#define READ_DIGITAL                (0X14)
#define WRITE_DIGITAL               (0X15)
#define READ_ANALOG                 (0X16)
#define WRITE_ANALOG                (0X17)
#define READ_EEPROM                 (0X1A)
#define WRITE_EEPROM                (0X1B)  
#define SERVO_STATUS                (0X1C)
#define PUMP_STATUS                 (0X1D)
#define WRITE_STRETCH               (0X1E)

#include "EEPROM.h"
#include "uarm_library.h"

class UArmFirmata:public FirmataFeature
{
public:
  UArmFirmata();
  boolean handleSysex(byte command, byte argc, byte *argv);
  boolean handlePinMode(byte pin, int mode);
  void handleCapability(byte pin);
  void reset();
  void report();

private:
  static void sendFloatAsFour7bitBytes(float val);
  static void sendFloatAsThree7bitBytes(float val);
  static void sendFloatAsTwo7bitBytes(float val);
  static void sendFloatAsOne7bitBytes(float val);
};

#endif
