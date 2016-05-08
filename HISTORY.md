## [1.5.8] - 2016-05-07

### Fix
- Speed up Calibration, reduce delay
- Use writeAngle() Function in MoveTo

## [1.5.7] - 2016-05-07

### Fix

- Update SERVO_OFFST_ADDRESS to MANUAL_OFFSET_ADDRESS
- Update indent for syntax

### Changes

- Add Calibration.ino into examples


## [1.5.6] - 2016-05-06

### Fix

- Update SERIAL_NUMBER_ADDRESS to 100, Update CALIBRATION_STRETCH_FLAG mark value is CONFIRM_FLAG

## [1.5.5] - 2016-05-05

### Fix

- if SERIAL_NUMBER_ADDRESS equal CONFIRM_FLAG 0x80, then Serial Number exist in EEPROM

## [1.5.4] - 2016-05-02

### Fix

- if SERIAL_NUMBER_ADDRESS equal SERIAL_NUMBER_ADDRESS, then Serial Number exist in EEPROM

## [1.5.3] - 2016-05-02

### Changes

- Add readSerialNumber & writeSerialNumber function (Serial Number: Address 1024, size:14)


## [1.5.2] - 2016-04-29

### Changes

- Add Example, MoveTest, GetXYZ, Recording Mode


## [1.5.2] - 2016-04-29

### Changes

- Cancel v.1.5.0 read offset from EEPROM.


## [1.5.0] - 2016-04-14

### Changes

- Initialize servo offset & linear offset from EEPROM to global values, prevent read EEPROM every time

- Use [EEPROM.get()][a4e46a5d] & [EEPROM.put()][275bf48d] to read & write value in EEPROM instead of saveDataToRom()

  [a4e46a5d]: https://www.arduino.cc/en/Reference/EEPROMGet "EEPROM.get()"
  [275bf48d]: https://www.arduino.cc/en/Reference/EEPROMPut "EEPROM.put()"

- rename function readToAngle to analogToAngle
- Change Offset address in EEPROM

## [1.4.0] - 2016-04-12

### Changes

- uarm_library.cpp writeServoAngle resume LEFT & RIGHT ANGLE

- ToDo: remove LEFT & RIGHT ANGLE from WriteServoAngle (For Safety)


## [1.3.1] - 2016-04-12

### Changes

- uarm_library.cpp writeLeftRightServoAngle() Update the angle between left & right  

- Change Folder examples folder & stretch name From UarmFirmata to UArmFirmata  
