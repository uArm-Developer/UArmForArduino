/*
This Example is for Calibration on uArm.

1. It will wait you press D7 to start.
2. First it would auto detect the SERVO_LEFT maximum angle and SERVO_RIGHT minimum angle
3. Second it would collect hundreads of point for each servo to caculate the Linear regression 
4. Third it will wait you to move uArm to correct position to calibrate the SERVO_ROT, You could refer this picture.
https://raw.githubusercontent.com/uArm-Developer/UArmForArduino/dev/examples/Metal/calibration.png
*/

#include <Servo.h>
#include <EEPROM.h>
#include "linreg.h"
#define SERVO_ROT_NUM           0
#define SERVO_LEFT_NUM          1
#define SERVO_RIGHT_NUM         2
#define SERVO_HAND_ROT_NUM      3

#define SERVO_ROT_PIN           11
#define SERVO_LEFT_PIN          13
#define SERVO_RIGHT_PIN         12
#define SERVO_HAND_ROT_PIN      10

#define SERVO_ROT_ANALOG_PIN 2
#define SERVO_LEFT_ANALOG_PIN 0
#define SERVO_RIGHT_ANALOG_PIN 1
#define SERVO_HAND_ROT_ANALOG_PIN 3

// Calibration Flag & OFFSET EEPROM ADDRESS
#define CALIBRATION_FLAG                    10
#define CALIBRATION_LINEAR_FLAG             11
#define CALIBRATION_MANUAL_FLAG             12
#define CALIBRATION_STRETCH_FLAG            13

#define LINEAR_INTERCEPT_START_ADDRESS      70
#define LINEAR_SLOPE_START_ADDRESS          50
#define MANUAL_OFFSET_ADDRESS               30
#define OFFSET_STRETCH_START_ADDRESS        20
#define SERIAL_NUMBER_ADDRESS               100

#define CONFIRM_FLAG                        0x80

#define LIMIT_SW                2    // LIMIT Switch Button
#define BUZZER                  3    // HIGH = ON
#define BTN_D4                  4    // LOW = Pressed
#define BTN_D7                  7    // LOW = Pressed

#define PUMP_EN                 6    // HIGH = Valve OPEN
#define VALVE_EN                5    // HIGH = Pump ON


#define MAX_OFFSET_ANGLE            10
#define STANDARD_LEFT_MAX_ANGLE     150
#define STANDARD_RIGHT_MIN_ANGLE     20

void calibrate();
void alert(byte times, int runTime, int stopTime);
int getAverageAnalog(byte servo_number);

int current;         // Current state of the button
// (LOW is pressed b/c i'm using the pullup resistors)
long millis_held;    // How long the button was held (milliseconds)
long secs_held;      // How long the button was held (seconds)
long prev_secs_held; // How long the button was held in the previous check
byte previous = HIGH;
unsigned long firstTime; // how long since the button was first pressed


Servo leftServo;
Servo rightServo;
Servo rotServo;
Servo handServo;

int min_anagle = 90;
int left_init_pos = 90;
int right_init_pos = 90;
int analog_array[5]= {};
int angle_array[5]= {};
int left_max_angle = -1;
int right_min_angle = -1;

double linear_rot_a = 0.0f;
double linear_rot_b = 0.0f;

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);
  pinMode(BTN_D4, INPUT_PULLUP);
  pinMode(BTN_D7, INPUT_PULLUP);
  Serial.println("[STEP]READY");
  Serial.println("[INFO]Waiting For Start, Please Press D7 to Start.");
}

void loop() {
  if (digitalRead(BTN_D7) == LOW) {

    delay(20);
    if (digitalRead(BTN_D7) == LOW)
    {
      Serial.println("[STEP]START");
      alert(2, 100, 100);
      delay(2000);
      Serial.println("[STEP]LIMITATION");
      get_limit_values();
      delay(500);
      Serial.println("[STEP]LINEAR");
      linear_calibration();
      delay(500);
      Serial.println("[STEP]MANUAL");
      delay(500);
      Serial.println("[INFO]Waiting For Manual Calibration, Please Press D4 Once Completed.");
      waitingForManualCalibration();
      delay(500);
      EEPROM.write(CALIBRATION_FLAG, CONFIRM_FLAG);
      delay(500);
      Serial.println("[STEP]COMPLETED");
      alert(1, 500, 500);
    }
  }
}

void waitingForManualCalibration() {
  while (true) {
    if (digitalRead(BTN_D4) == LOW) {

      delay(20);
      if (digitalRead(BTN_D4) == LOW)
      {
        alert(2, 100, 100);
        delay(2000);
        int analog = getAverageAnalog(SERVO_ROT_NUM);
        double rot_offset = linear_rot_b * analog + linear_rot_a - 45;
        double left_offset = left_max_angle - 150;
        double right_offset = right_min_angle - 20;
        Serial.print("[INFO]SERVO_ROT_OFFSET:");
        Serial.println(rot_offset);
        Serial.print("[INFO]SERVO_LEFT_OFFSET:");
        Serial.println(left_offset);
        Serial.print("[INFO]SERVO_ROT_OFFSET:");
        Serial.println(right_offset);
        save_manual_servo_offset(SERVO_ROT_NUM, rot_offset);
        delay(10);
        save_manual_servo_offset(SERVO_LEFT_NUM, left_offset);
        delay(10);
        save_manual_servo_offset(SERVO_RIGHT_NUM, right_offset);
        delay(10);
        save_manual_servo_offset(SERVO_HAND_ROT_NUM, 0);
        delay(10);
        EEPROM.write(CALIBRATION_MANUAL_FLAG, CONFIRM_FLAG);
        break;
      }
    }
  }
}

void get_limit_values() {
  //left servo
  Serial.println("[INFO]Trying to catch the limition for left & right servo");
  leftServo.attach(SERVO_LEFT_PIN);
  leftServo.write(left_init_pos);
  delay(1000);
  for (int p = left_init_pos; p <= 180; p += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    leftServo.write(p);              // tell servo to go to position in variable 'pos'
    delay(50);                       // waits 15ms for the servo to reach the position
    Serial.print("[DEBUG]LEFT-");
    Serial.print(p);
    Serial.print(",");
    unsigned int dat[8], temp;
    unsigned char i = 0, j = 0;
    for (i = 0; i < 8; i++)
    {
      dat[i]= analogRead(SERVO_LEFT_ANALOG_PIN);

    }
    for (i = 0; i < 8; i++) { //BULB to get the most accuracy data
      for (j = 0; i + j < 7; j++) {
        if (dat[j]> dat[j + 1]) {
          temp = dat[j];
          dat[j]= dat[j + 1];
          dat[j + 1]= temp;
        }
      }
    }
    int val_analog = (dat[2]+ dat[3]+ dat[4]+ dat[5]) / 4;
    Serial.println(val_analog);
    int array_index = p % 5;
    analog_array[array_index]= val_analog;
    angle_array[array_index]= p;
    if (array_index == 4) {
      int average = 0;
      for (int k = 0; k < 3; k++) {
        average += analog_array[k + 1]- analog_array[k];
      }
      if (average <= 1) {
        left_max_angle = angle_array[0];
        break;
      }
    }

  }
  Serial.print("[INFO]Left_MAX_ANGLE:");
  Serial.println(left_max_angle);
  if (left_max_angle == -1) {
    left_max_angle = 180;
  }
  if (abs(left_max_angle - STANDARD_LEFT_MAX_ANGLE) > MAX_OFFSET_ANGLE) {
    alert(4, 100, 100);
  }
  else {
    alert(1, 300, 300);
  }
  leftServo.detach();
  delay(2000);
  //--------------------right servo------------------------
  for (int k = 0; k < 4; k++)
  {
    analog_array[k]= {0};
    angle_array[k]= {0};
  }
  rightServo.attach(SERVO_RIGHT_PIN);
  rightServo.write(right_init_pos);
  delay(1000);
  for (int p = right_init_pos; p >= 0; p -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    rightServo.write(p);              // tell servo to go to position in variable 'pos'
    delay(50);                       // waits 15ms for the servo to reach the position
    Serial.print("[DEBUG]RIGHT-");
    Serial.print(p);
    Serial.print(",");
    int val_analog = getAverageAnalog(SERVO_RIGHT_NUM);
    Serial.println(val_analog);
    int array_index = p % 5;
    analog_array[array_index]= val_analog;
    angle_array[array_index]= p;
    if (array_index == 0) {
      int average = 0;
      for (int k = 4; k > 1; k--) {
        average += analog_array[k]- analog_array[k - 1];
      }
      if (average <= 1) {
        right_min_angle = angle_array[4];
        break;
      }
    }

  }
  Serial.print("[INFO]Right_MIN_ANGLE:");
  Serial.println(right_min_angle);
  if (right_min_angle == -1) {
    right_min_angle = 0;
  }
  if (abs(right_min_angle - STANDARD_RIGHT_MIN_ANGLE) > MAX_OFFSET_ANGLE) {
    alert(4, 100, 100);
  }
  else {
    alert(1, 300, 300);
  }
  rightServo.detach();
}


/** Start Calibration all section
**/
void linear_calibration() {

  //        cleanEEPROM();
  for (int k = 0; k < 4; k++) {

    calibrate_linear_servo(k);
    delay(2000);
  }
  EEPROM.write(CALIBRATION_LINEAR_FLAG, CONFIRM_FLAG);
  //  //        manual_calibration_section();
  alert(1, 500, 500);
}

/** Calibrate each servo for linear offset
**/
void calibrate_linear_servo(byte servo_num)
{
  alert(1, 300, 300);
  double arr_real[90];
  double arr_input[90];
  int max_angle = 0;
  int min_angle = 0;
  switch (servo_num) {
    case SERVO_ROT_NUM:
      max_angle = 135;
      min_angle = 45;
      attachServo(servo_num);
      setServoAngle(servo_num, min_angle);
      break;
    case SERVO_LEFT_NUM:
      max_angle = left_max_angle - 5;
      min_angle = left_max_angle - 90 + 5;
      attachServo(servo_num);
      setServoAngle(servo_num, min_angle);
      break;
    case SERVO_RIGHT_NUM:
      min_angle = right_min_angle + 5;
      max_angle = right_min_angle + 90 - 5;
      attachServo(servo_num);
      setServoAngle(servo_num, min_angle);
      break;
    case SERVO_HAND_ROT_NUM:
      max_angle = 135;
      min_angle = 45;
      attachServo(servo_num);
      setServoAngle(servo_num, min_angle);
      break;
  }
  delay(1000);
  for (byte i = 0, angle = min_angle ; angle < max_angle; i ++, angle++) {
    setServoAngle(servo_num, angle);
    delay(100);
    arr_input[i]= angle;
    arr_real[i]= getAverageAnalog(servo_num);
    Serial.print("[DEBUG]SERVO_ANGLE_");
    Serial.print(servo_num);
    Serial.print(": ");
    Serial.print(angle);
    Serial.print(", Analog: ");
    Serial.println(arr_real[i]);
  }
  setServoAngle(servo_num, 90);
  delay(1000);
  detachServo(servo_num);




  LinearRegression lr(arr_real, arr_input, 90);
  if (servo_num == SERVO_ROT_NUM) {
    linear_rot_a = lr.getA();
    linear_rot_b = lr.getB();
  }
  Serial.print("[INFO]lr.getA():");
  Serial.println(lr.getA());
  Serial.print("[INFO]lr.getB():");
  Serial.println(lr.getB());
  save_linear_servo_offset(servo_num, lr.getA(), lr.getB());
}

void setServoAngle(byte servo_number, byte angle) {
  switch (servo_number) {
    case SERVO_ROT_NUM:
      rotServo.write(angle);
      break;
    case SERVO_LEFT_NUM:
      leftServo.write(angle);
      break;
    case SERVO_RIGHT_NUM:
      rightServo.write(angle);
      break;
    case SERVO_HAND_ROT_NUM:
      handServo.write(angle);
      break;
  }
}

int getAverageAnalog(byte servo_number) {
  byte analog_pin;
  switch (servo_number) {
    case SERVO_ROT_NUM:
      analog_pin = SERVO_ROT_ANALOG_PIN;
      break;
    case SERVO_LEFT_NUM:
      analog_pin = SERVO_LEFT_ANALOG_PIN;
      break;
    case SERVO_RIGHT_NUM:
      analog_pin = SERVO_RIGHT_ANALOG_PIN;
      break;
    case SERVO_HAND_ROT_NUM:
      analog_pin = SERVO_HAND_ROT_ANALOG_PIN;
      break;
  }
  unsigned int dat[8], temp;
  unsigned char i = 0, j = 0;
  for (i = 0; i < 8; i++)
  {
    dat[i]= analogRead(analog_pin);

  }
  for (i = 0; i < 8; i++) { //BULB to get the most accuracy data
    for (j = 0; i + j < 7; j++) {
      if (dat[j]> dat[j + 1]) {
        temp = dat[j];
        dat[j]= dat[j + 1];
        dat[j + 1]= temp;
      }
    }
  }
  int val_analog = (dat[2]+ dat[3]+ dat[4]+ dat[5]) / 4;
  return val_analog;
}

void attachServo(byte servo_number) {
  switch (servo_number) {
    case SERVO_ROT_NUM:
      if (!rotServo.attached()) {
        rotServo.attach(SERVO_ROT_PIN);
      }
      break;
    case SERVO_LEFT_NUM:
      if (!leftServo.attached()) {
        leftServo.attach(SERVO_LEFT_PIN);
      }
      break;
    case SERVO_RIGHT_NUM:
      if (!rightServo.attached()) {
        rightServo.attach(SERVO_RIGHT_PIN);
      }
      break;
    case SERVO_HAND_ROT_NUM:
      if (!handServo.attached()) {
        handServo.attach(SERVO_HAND_ROT_PIN);
      }
      break;
  }
}

void detachServo(byte servo_number) {
  switch (servo_number) {
    case SERVO_ROT_NUM:
      rotServo.detach();
      break;
    case SERVO_LEFT_NUM:
      leftServo.detach();
      break;
    case SERVO_RIGHT_NUM:
      rightServo.detach();
      break;
    case SERVO_HAND_ROT_NUM:
      handServo.detach();
      break;
  }
}

void alert(byte times, int runTime, int stopTime)
{
  for (int ct = 0; ct < times; ct++)
  {
    delay(stopTime);
    digitalWrite(BUZZER, HIGH);
    delay(runTime);
    digitalWrite(BUZZER, LOW);
  }
}

/** Save Manual Servo Offset

**/
void save_manual_servo_offset(byte servo_num, double offset)
{
  EEPROM.put(MANUAL_OFFSET_ADDRESS + servo_num * sizeof(offset), offset);
}

/** Save Linear Servo Offset intercept & slope
**/
void save_linear_servo_offset(byte servo_num, double intercept_val, double slope_val) {
  EEPROM.put(LINEAR_INTERCEPT_START_ADDRESS + servo_num * sizeof(intercept_val), intercept_val);
  EEPROM.put(LINEAR_SLOPE_START_ADDRESS + servo_num * sizeof(slope_val), slope_val);
}
