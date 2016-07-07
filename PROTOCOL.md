# Protocol Specification

## DATA TYPE

- FB, Means Firmata Byte, Each FB = 7bits Byte

Type                 | Short Name |        Range         |   Bits   | Actual Bytes
-------------------- | ---------- | :------------------: | :------: | :----------:
1FB Byte             | 1FB Byte   |       0 ~ 127        |    7     |      1       |
2FB unsigned Integer | 2FB uint   |      0 ~ 16383       |    14    |      2       |
3FB Integer          | 3FB int    |    -16383 ~ 16383    |    15    |      3       |
3FB Float            | 3FB float  |   0.00 ~ 16383.99    |    21    |      3       |
4FB Float            | 4FB float  | -16383.99 ~ 16383.99 |    22    |      4       |
-------------------  | -          | :------------------: | :------: |    :---:

### DATA TYPE - 1FB Byte

```c
1FB byte = bytes[0]
```

Sequence   |  Range  | Remark
---------- | :-----: | ------
0          | 0 ~ 127 |
---------- | :-----: | ------

### DATA TYPE - 2FB uint

```c
2FB unit = bytes[0] + bytes[1]<<7
```

Sequence |  Range  | Remark
-------- | :-----: | ------
0        | 0 ~ 127 | MSB
1        | 0 ~ 127 | LSB
-------- | :-----: | ------

### DATA TYPE - 3FB int

```c
3FB int = (bytes[0]==0 ? 1 : -1) * (bytes[1] + bytes[2]<<7)
```

Sequence |  Range  | Remark
-------- | :-----: | ------------
0        | 0 or 1  | Negative - 1
1        | 0 ~ 127 | MSB
2        | 0 ~ 127 | LSB
-------- | :-----: | ------------

### DATA TYPE - 3FB float

```c
3FB Float = bytes[0] + bytes[1]<<7 + bytes[2]/100.00
```

Sequence |  Range  | Remark
-------- | :-----: | ------------
0        | 0 ~ 127 | Integer MSB
1        | 0 ~ 127 | Integer LSB
2        | 0 ~ 99  | Decimal Val
-------- | :-----: | ------------

### DATA TYPE - 4FB float

```c
4FB Float = (bytes[0]==0 ? 1 : -1) * (bytes[1] + bytes[2]<<7 + bytes[3]/100.00)
```

Sequence |  Range  | Remark
-------- | :-----: | ------------
0        | 0 or 1  | Negative - 1
1        | 0 ~ 127 | Integer MSB
2        | 0 ~ 127 | Integer LSB
3        | 0 ~ 99  | Decimal Val
-------- | :-----: | ------------

## Protocol Request Message Type

Type                   | Command | Data                                         | Total Bytes | Remark
---------------------- | ------- | -------------------------------------------- | ----------- | ---------------------------------------------
UARM_CODE              | 0xAA    | N/A                                          | 1           | UARM First Level Command
Type                   | Command | Data                                         | Total Bytes | Remark
---                    | ---     | ---                                          | ---         | ---
READ_ANGLE             | 0x10    | 1fb Byte X2                                  | 6           | Return Servo Angle from uArm
WRITE_ANGLE            | 0x11    | 1fb Byte X1 + 3fb Float X1                   | 9           | Write Servo Angle to uArm
READ_COORDS            | 0x12    |                                              | 4           | Read Coordinate from uArm
WRITE_COORDS           | 0x13    | 4fb Float X5 + 1fb Byte X3                   | 27          | Move uArm to the Coordinate
READ_DIGITAL           | 0x14    | 1fb Byte X2                                  | 6           | Read Digital from uArm
WRITE_DIGITAL          | 0x15    | 1fb Byte X2                                  | 6           | Write Digital to uArm
READ_ANALOG            | 0x16    | 1fb Byte                                     | 5           | Read Analog from uArm
WRITE_ANALOG           | 0x17    | 1fb Byte + 4b Float                          | 9           | Write Analog to uArm
READ_EEPROM            | 0x1A    | 1fb Byte + 2b uint                           | 7           | Read the value from EEPROM
WRITE_EEPROM           | 0x1B    | 1fb Byte + 2b uint + 2b uint/3b int/4b float | 9/10/11     | Write the value to EEPROM
DETACH_SERVO           | 0x1C    | 0                                            | 4           | Detach All Servo
PUMP_STATUS            | 0x1D    | 1fb Byte                                     | 5           | Control Pump
GRIPPER_STATUS         | 0x20    | 1fb Byte                                     | 5           | Control Gripper
WRITE_STRETCH          | 0x1E    | 4b Float X2                                  | 12          | Move uArm to Position with Stretch and Height
WRITE_LEFT_RIGHT_ANGLE | 0x1F    | 3b Float X2                                  | 10          | Write Left & Right Servo Angle
READ_SERIAL_NUMBER     | 0x21    | 0                                            | 4           | Read Serial Number from EEPROM
WRITE_SERIAL_NUMBER    | 0x22    | 1fb Byte X14                                 | 18          | Write Serial Number from EEPROM
REPORT_LIBRARY_VERSION | 0x23    | 0                                            | 4           | Report uArm Library Version

### Request READ_ANGLE Message


Sequence | Type         | Command/Data | Bytes | Remark
-------- | ------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0         | 1     |
2        | UARM_CODE    | 0xAA         | 1     |
3        | READ_ANGLE   | 0x10         | 1     |
4        | Servo Number | 0/1/2/3      | 1     | SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
5        | with Offset  | 0/1          | 1     | 0 False, 1 True
6        | END_SYSEX    | 0xF7         | 1     |
-------- | ------------ | ------------ | 6     | ------------------------------------------------------------------

### Request WRITE_ANGLE Message


Sequence | Type         | Command/Data               | Bytes | Remark
-------- | ------------ | -------------------------- | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0                       | 1     |
2        | UARM_CODE    | 0xAA                       | 1     |
3        | WRITE_ANGLE  | 0x11                       | 1     |
4        | Servo Number | 0/1/2/3                    | 1     | SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
5        | Servo Angle  | 0.00 - 16383.99 (3b Float) | 3     |
6        | with Offset  | 0/1                        | 1     | 0 False, 1 True
7        | END_SYSEX    | 0xF7                       | 1     |
-------- | ------------ | ------------               | 9     | ------------------------------------------------------------------


### Request READ_COORDS Message


Sequence | Type         | Command/Data | Bytes | Remark
-------- | ------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0         | 1     |
2        | UARM_CODE    | 0x12         | 1     |
3        | READ_COORDS  | 0x10         | 1     |
4        | END_SYSEX    | 0xF7         | 1     |
-------- | ------------ | ------------ | 4     | ------------------------------------------------------------------

### Request WRITE_COORDS Message


Sequence | Type           | Command/Data                    | Bytes | Remark
-------- | -------------- | ------------------------------- | ----- | ----------------------------------------------------------------------------------------------------
1        | START_SYSEX    | 0xF0                            | 1     |
2        | UARM_CODE      | 0xAA                            | 1     |
3        | WRITE_COORDS   | 0x13                            | 1     |
4        | X Axis         | -16383.99 - 16383.99 (4b Float) | 4     |
5        | Y Axis         | -16383.99 - 16383.99 (4b Float) | 4     |
6        | Z Axis         | -16383.99 - 16383.99 (4b Float) | 4     |
7        | Hand Axis      | -16383.99 - 16383.99 (4b Float) | 4     |
8        | relative_flags | 0/1                             | 1     | 0 relative, 1 absolute
9        | time_spend     | -16383.99 - 16383.99 (4b Float) | 4     |
10       | path_type      | 0/1                             | 1     | 0 PATH_LINEAR, 1 PATH_ANGLES
11       | ease_type      | 0/1                             | 1     | 0 PATH_LINEAR, 1 PATH_ANGLES
12       | END_SYSEX      | 0xF7                            | 1     | 0 INTERP_EASE_INOUT_CUBIC, 1 INTERP_LINEAR, 2 INTERP_EASE_INOUT, 3 INTERP_EASE_IN, 4 INTERP_EASE_OUT
-------- | ------------   | ------------                    | 27    | ------------------------------------------------------------------

### Request READ_DIGITAL Message


Sequence | Type         | Command/Data | Bytes | Remark
-------- | ------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0         | 1     |
2        | UARM_CODE    | 0xAA         | 1     |
3        | READ_DIGITAL | 0x14         | 1     |
4        | pin number   | 0-127        | 1     | Pin Number
5        | pin mode     | 0/1          | 1     | 0 INPUT, 1 PULL_UP
6        | END_SYSEX    | 0xF7         | 1     |
-------- | ------------ | ------------ | 6     | ------------------------------------------------------------------

### Request WRITE_DIGITAL Message

Sequence | Type          | Command/Data | Bytes | Remark
-------- | ------------- | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX   | 0xF0         | 1     |
2        | UARM_CODE     | 0xAA         | 1     |
3        | WRITE_DIGITAL | 0x15         | 1     |
4        | pin number    | 0-127        | 1     | Pin Number
5        | Value         | 0/1          | 1     | 0 LOW, 1 HIGH
6        | END_SYSEX     | 0xF7         | 1     |
-------- | ------------  | ------------ | 6     | ------------------------------------------------------------------

### Request READ_ANALOG Message

Sequence | Type              | Command/Data | Bytes | Remark
-------- | ----------------- | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX       | 0xF0         | 1     |
2        | UARM_CODE         | 0xAA         | 1     |
3        | READ_ANALOG       | 0x16         | 1     |
4        | ANALOG PIN NUMBER | 0 - 127      | 1     |
6        | END_SYSEX         | 0xF7         | 1     |
-------- | ------------      | ------------ | 6     | ------------------------------------------------------------------

### Request WRITE_ANALOG Message

Sequence | Type         | Command/Data                | Bytes | Remark
-------- | ------------ | --------------------------- | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0                        | 1     |
2        | UARM_CODE    | 0xAA                        | 1     |
3        | WRITE_ANALOG | 0x17                        | 1     |
4        | PIN Number   | 0 - 127                     | 1     |
5        | Analog       | 0 - 16383 (2b unsigned int) | 2     |
6        | END_SYSEX    | 0xF7                        | 1     |
-------- | ------------ | ------------                | 8     | ------------------------------------------------------------------

### Request READ_EEPROM Message

Sequence | Type         | Command/Data | Bytes | Remark
-------- | ------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0         | 1     |
2        | UARM_CODE    | 0xAA         | 1     |
3        | READ_EEPROM  | 0x1A         | 1     |
4        | DATA TYPE    | 1/2/4        | 1     | DATA_TYPE_BYTE -1 , DATA_TYPE_INTEGER -2 , DATA_TYPE_FLOAT -4
5        | ADDRESS      | 0 - 16383    | 2     |
6        | END_SYSEX    | 0xF7         | 1     |
-------- | ------------ | ------------ | 7     | ------------------------------------------------------------------

### Request WRITE_EEPROM Message

Sequence | Type         | Command/Data                      | Bytes   | Remark
-------- | ------------ | --------------------------------- | ------- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0                              | 1       |
2        | UARM_CODE    | 0xAA                              | 1       |
3        | WRITE_EEPROM | 0x1B                              | 1       |
4        | DATA TYPE    | 1/2/4                             | 1       | DATA_TYPE_BYTE -1 , DATA_TYPE_INTEGER -2 , DATA_TYPE_FLOAT -4
5        | ADDRESS      | 0 - 16383                         | 2       |
6        | Data         | 2b unsigned int/ 3b int/ 4b Float | 2/3/4   |
7        | END_SYSEX    | 0xF7                              | 1       |
-------- | ------------ | ------------                      | 9/10/11 | ------------------------------------------------------------------

### Request DETACH_SERVO Message

Sequence | Type         | Command/Data | Bytes | Remark
-------- | ------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0         | 1     |
2        | UARM_CODE    | 0xAA         | 1     |
3        | DETACH_SERVO | 0x1C         | 1     |
4        | END_SYSEX    | 0xF7         | 1     |
-------- | ------------ | ------------ | 4     | ------------------------------------------------------------------

### Request PUMP_STATUS Message

Sequence | Type         | Command/Data | Bytes | Remark
-------- | ------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0         | 1     |
2        | UARM_CODE    | 0xAA         | 1     |
3        | PUMP_STATUS  | 0x1E         | 1     |
4        | data         | 0/1          | 1     | 0 Off, 1 On
5        | END_SYSEX    | 0xF7         | 1     |
-------- | ------------ | ------------ | 5     | ------------------------------------------------------------------

### Request GRIPPER_STATUS Message

Sequence | Type           | Command/Data | Bytes | Remark
-------- | -------------- | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX    | 0xF0         | 1     |
2        | UARM_CODE      | 0xAA         | 1     |
3        | GRIPPER_STATUS | 0x20         | 1     |
4        | data           | 0/1          | 1     | 0 Release, 1 Catch
5        | END_SYSEX      | 0xF7         | 1     |
-------- | ------------   | ------------ | 5     | ------------------------------------------------------------------

### Request WRITE_STRETCH Message

Sequence | Type          | Command/Data | Bytes | Remark
-------- | ------------- | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX   | 0xF0         | 1     |
2        | UARM_CODE     | 0xAA         | 1     |
3        | WRITE_STRETCH | 0x1E         | 1     |
4        | STRETCH       | 4b float     | 4     |
5        | HEIGHT        | 4b float     | 4     |
6        | END_SYSEX     | 0xF7         | 1     |
-------- | ------------  | ------------ | 12    | ------------------------------------------------------------------

### Request WRITE_LEFT_RIGHT_ANGLE Message

Sequence | Type                   | Command/Data | Bytes | Remark
-------- | ---------------------- | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX            | 0xF0         | 1     |
2        | UARM_CODE              | 0xAA         | 1     |
3        | WRITE_LEFT_RIGHT_ANGLE | 0x1F         | 1     |
4        | LEFT_ANGLE             | 3b float     | 3     |
5        | RIGHT_ANGLE            | 3b float     | 3     |
6        | END_SYSEX              | 0xF7         | 1     |
-------- | ------------           | ------------ | 10    | ------------------------------------------------------------------

### Request READ_SERIAL_NUMBER Message

Sequence | Type               | Command/Data | Bytes | Remark
-------- | ------------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX        | 0xF0         | 1     |
2        | UARM_CODE          | 0xAA         | 1     |
3        | READ_SERIAL_NUMBER | 0x21         | 1     |
4        | END_SYSEX          | 0xF7         | 1     |
-------- | ------------       | ------------ | 4     | ------------------------------------------------------------------

### Request WRITE_SERIAL_NUMBER Message

Sequence | Type                | Command/Data | Bytes | Remark
-------- | ------------------- | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX         | 0xF0         | 1     |
2        | UARM_CODE           | 0xAA         | 1     |
3        | WRITE_SERIAL_NUMBER | 0x22         | 1     |
4        | DATA                | 1fb Byte X14 | 14    | ASCII code
5        | END_SYSEX           | 0xF7         | 1     |
-------- | ------------        | ------------ | 18    | ------------------------------------------------------------------

### Request REPORT_LIBRARY_VERSION Message

Sequence | Type                   | Command/Data | Bytes | Remark
-------- | ---------------------- | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX            | 0xF0         | 1     |
2        | UARM_CODE              | 0xAA         | 1     |
3        | REPORT_LIBRARY_VERSION | 0x23         | 1     |
4        | END_SYSEX              | 0xF7         | 1     |
-------- | ------------           | ------------ | 4     | ------------------------------------------------------------------

## Protocol RESPONSE Message Type

Type                   | Command | Data                                               | Total Bytes | Remark
---------------------- | ------- | -------------------------------------------------- | ----------- | ----------------------------
UARM_CODE              | 0xAA    | N/A                                                | 1           | UARM First Level Command
Type                   | Command | Data                                               | Total Bytes | Remark
---                    | ---     | ---                                                | ---         | ---
READ_ANGLE             | 0x10    | 3fb Float                                          | 7           | Return Servo Angle from uArm
READ_COORDS            | 0x12    | 4fb Float X3                                       | 12          | Read Coordinate from uArm
READ_DIGITAL           | 0x14    | 1fb Byte X2                                        | 6           | Read Digital from uArm
READ_ANALOG            | 0x16    | 1fb Byte X3                                        | 7           | Read Analog from uArm
READ_EEPROM            | 0x1A    | 1fb Byte + 2fb uint + 2fb uint/3fb Float/4fb Float | 9/10/11     | Read the value from EEPROM
REPORT_LIBRARY_VERSION | 0x23    | 1fb Byte X3                                        | 4           | Report uArm Library Version

### Response READ_ANGLE Message

Sequence | Type         | Command/Data | Bytes | Remark
-------- | ------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0         | 1     |
2        | UARM_CODE    | 0xAA         | 1     |
3        | READ_ANGLE   | 0x10         | 1     |
4        | SERVO NUMBER | 0/1/2/3      | 1     | SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
5        | SERVO ANGLE  | 3fb float    | 3     |
6        | END_SYSEX    | 0xF7         | 1     |
-------- | ------------ | ------------ | 8     | ------------------------------------------------------------------

### Response READ_COORDS Message

Sequence | Type         | Command/Data | Bytes | Remark
-------- | ------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0         | 1     |
2        | UARM_CODE    | 0xAA         | 1     |
3        | READ_COORDS  | 0x12         | 1     |
4        | X Axis       | 4fb floats   | 4     |
5        | Y Axis       | 4fb floats   | 4     |
6        | Z Axis       | 4fb floats   | 4     |
7        | END_SYSEX    | 0xF7         | 1     |
-------- | ------------ | ------------ | 16    | ------------------------------------------------------------------

### Response READ_DIGITAL Message

Sequence | Type         | Command/Data | Bytes | Remark
-------- | ------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0         | 1     |
2        | UARM_CODE    | 0xAA         | 1     |
3        | READ_DIGITAL | 0x14         | 1     |
4        | PIN NUMBER   | 0 - 127      | 1     |
5        | VALUE        | 0/1          | 1     | HIGH : 1, LOW : 0
6        | END_SYSEX    | 0xF7         | 1     |
-------- | ------------ | ------------ | 6     | ------------------------------------------------------------------

### Response READ_ANALOG Message

Sequence | Type         | Command/Data | Bytes | Remark
-------- | ------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0         | 1     |
2        | UARM_CODE    | 0xAA         | 1     |
3        | READ_ANALOG  | 0x16         | 1     |
4        | PIN NUMBER   | 0 - 127      | 1     |
5        | ANALOG VALUE | 0 - 16383    | 2     |
6        | END_SYSEX    | 0xF7         | 1     |
-------- | ------------ | ------------ | 7     | ------------------------------------------------------------------

### Response READ_EEPROM Message

Sequence | Type         | Command/Data              | Bytes   | Remark
-------- | ------------ | ------------------------- | ------- | ------------------------------------------------------------------
1        | START_SYSEX  | 0xF0                      | 1       |
2        | UARM_CODE    | 0xAA                      | 1       |
3        | READ_EEPROM  | 0x1A                      | 1       |
4        | DATA TYPE    | 1/2/4                     | 1       | BYTE: 1, INTEGER: 2, FLOAT: 4
5        | ADDRESS      | 0 - 16383                 | 2       |
6        | DATA         | 1fb Byte/3b int/4fb Float | 2/3/4   |
7        | END_SYSEX    | 0xF7                      | 1       |
-------- | ------------ | ------------              | 9/10/11 | ------------------------------------------------------------------

### Response READ_SERIAL_NUMBER Message

Sequence | Type               | Command/Data | Bytes | Remark
-------- | ------------------ | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX        | 0xF0         | 1     |
2        | UARM_CODE          | 0xAA         | 1     |
3        | READ_SERIAL_NUMBER | 0x21         | 1     |
4        | DATA               | 1fb Byte X14 | 14    |
5        | END_SYSEX          | 0xF7         | 1     |
-------- | ------------       | ------------ | 18    | ------------------------------------------------------------------

### Response REPORT_LIBRARY_VERSION Message

Sequence | Type                   | Command/Data | Bytes | Remark
-------- | ---------------------- | ------------ | ----- | ------------------------------------------------------------------
1        | START_SYSEX            | 0xF0         | 1     |
2        | UARM_CODE              | 0xAA         | 1     |
3        | REPORT_LIBRARY_VERSION | 0x23         | 1     |
4        | DATA                   | 1fb Byte X3  | 3     | MAJOR Version, MINOR Version, BUGFIX
5        | END_SYSEX              | 0xF7         | 1     |
-------- | ------------           | ------------ | 7     | ------------------------------------------------------------------
