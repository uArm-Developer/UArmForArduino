#include <uarm_library.h>
//#include <UFServo.h>

// headers should must include these four headers

/**
   This sketch is a communication protocol for uArm using Serial.

   All commands must be wrapped in brackets- [command]
   Any string of commands must end with an endline char '\n' for the commands to be processed

   Commands:
   [moveX#Y#Z#S#]  Where # is a double, This will move the robot to an XYZ position, in S is speed in centimeters / second from the current location to goal location
                  Send Example:   [moveX15Y-15Z20S25]
                  Return Example: [OK moveX15Y-15Z20S2]

   [handV#]        Where # is an angle between 0 and 180. This will set the angle of the robots hand
                  Send Example:   [handV90]
                  Return Example: [OK handV90]

   [pumpV#]        Where # is either 1 or 0. 1 means pump on, 0 means pump off.
                  Send Example:   [pumpV1]
                  Return Example: [OK pumpV1]

   [attachS#]      Attach servo #. Same as servo #'s in uarm_library.h
                  Send Example:   [attachS1]
                  Return Example: [OK attachS1]

   [detachS#]      Detach servo #. Same as servo #'s in uarm_library.h
                  Send Example:   [detachS1]
                  Return Example: [OK detachS1]

   [buzzF#T#]      Set the buzzer to F Frequency for T time
                  Send Example:   [buzzF261.63T1]
                  Return Example: [OK buzzF261.63T1]


   [gcoords]       Returns the XYZ position of the robot
                  Send Example:   [gcoords]
                  Return Example: [coordsX#Y#Z#]

   [gAngleS#]      Returns the angle of any particular servo.
                  Send Example:   [angleA1]
                  Return Example: [angleA###.##]

   [gmoving]       Returns whether or not the robot is currently moving. Returns either 1 or 0 if it is moving or not.
                  Send Example: [gmoving]
                  Return Example: [movingM1]

   [gtip]          Returns whether or not the tip of the robot is currently pressed. Returns either 1 if the tip is pressed, 0 if not.
                  Send Example: [gtip]
                  Return Example: [tipV1]
 **/

//  The array of 'steps' the robot will take to get to it's desired location
byte currentStep = 255;  //The current 'step' the robot is in, in the movement array. -1 means it is not currently moving


// Set Pin variables
int tipPin    = 2;
int buzzerPin = 3;


// Movement Variables
unsigned int INTERP_INTVLS;
long goalTime;          //  When the robot intends to complete the move
double goalTimeStep;
double x_array[51];      // Maximum of 50 interp intervals in any move, so make each arr 80 long
double y_array[51];
double z_array[51];


//  These variables keep track of positional information about the robot.
double currentHand=90;   //  Current hand angle. Used for move commands that *require* a hand angle, due to uArm library design


// What time, in millis(), that the buzzer should stop. If it's -1, then the buzzer is off.
long buzzerStopTime = -1;

//  Communication variables
String message = "";



void setup() {
        Serial.begin(115200);
        Serial.setTimeout(30); //  Makes Serial.parseInt() not wait a long time after an integer has ended
        pinMode(tipPin,INPUT_PULLUP);
}


void loop() {
        // Get any commands that have been received and add it to the global message String for processing later
        while (Serial.available() > 0) {
                String msg = Serial.readStringUntil('>');
                message = message + String(msg);
        }



        // TODO: Reevaluate the priority of these. Should commands be added to message before processing? Should step be done b4 command is received?
        if(isTimeToMove()) {
                moveStep();
        }

        if(buzzerStopTime > 0) {
                if(buzzerStopTime < millis()) {
                        noTone(buzzerPin);
                        buzzerStopTime = -1;
                }
        }



        // If there is anything in the message string, then process if
        if(message.length() > 2) {
                String cmnd = parseNextCommand();
                String response = runCommand(cmnd);
                if(response.length() > 0) {
                        Serial.print(response);
                }
        }
}




String runCommand(String cmnd){
        String moveParameters[]         = {F("x"), F("y"), F("z"), F("s")};
        String pumpParameters[]         = {F("v")};
        String attachDetachParameters[] = {F("s")};
        String handParameters[]         = {F("v")};
        String buzzerParameters[]       = {F("f"), F("t")};
        String gServoParameters[]       = {F("s")};


        // To save memory, create the "[OK" and "]\n" right now, in flash memory
        String ok    = F("[OK ");
        String endB  = F("]\n");

        // Move Command
        if(cmnd.indexOf(F("move")) >= 0) {
                //Error/validity checking of command
                String errorResponse = isValidCommand(cmnd, moveParameters, 4);
                if(errorResponse.length() > 0) {return errorResponse; }

                //  Create action and respond
                float values[4];
                getCommandValues(cmnd, moveParameters, 4, values);
                setMove(values[0], values[1], values[2], values[3]);
                return ok + cmnd + endB;
        }

        // Set the status of the pump
        if(cmnd.indexOf(F("pump")) >= 0) {
                String errorResponse = isValidCommand(cmnd, pumpParameters, 1);
                if(errorResponse.length() > 0) {return errorResponse; }
                float values[1];
                getCommandValues(cmnd, pumpParameters, 1, values);
                if(values[0] > 0) {  uarm.pump_on();  }else if(values[0] <= 0) {  uarm.pump_off();  }
                return ok + cmnd + endB;
        }

        // Attach a servo
        if(cmnd.indexOf(F("attach")) >= 0) {
                String errorResponse = isValidCommand(cmnd, attachDetachParameters, 1);
                if(errorResponse.length() > 0) {return errorResponse; }
                float values[1];
                getCommandValues(cmnd, attachDetachParameters, 1, values);
                if(values[0] == 1) {           uarm.set_servo_status(true, SERVO_ROT_NUM); }else if(values[0] == 2) {     uarm.set_servo_status(true, SERVO_LEFT_NUM); }else if(values[0] == 3) {    uarm.set_servo_status(true, SERVO_RIGHT_NUM); }else if(values[0] == 4) { uarm.set_servo_status(true, SERVO_HAND_ROT_NUM); }else{ return F("[ERROR: Servo number does not exist]\n"); }
                return ok + cmnd + endB;
        }

        // Detach a servo
        if(cmnd.indexOf(F("detach")) >= 0) {
                String errorResponse = isValidCommand(cmnd, attachDetachParameters, 1);
                if(errorResponse.length() > 0) {return errorResponse; }
                float values[1];
                getCommandValues(cmnd, attachDetachParameters, 1, values);
                if(values[0] == 1) {      uarm.set_servo_status(false, SERVO_ROT_NUM); }else if(values[0] == 2) {uarm.set_servo_status(false, SERVO_LEFT_NUM); }else if(values[0] == 3) {uarm.set_servo_status(false, SERVO_RIGHT_NUM); }else if(values[0] == 4) {uarm.set_servo_status(false, SERVO_HAND_ROT_NUM); }else{ return "[ERROR: Servo number " + String(values[0]) + " does not exist]\n"; }
                return ok + cmnd + endB;
        }

        // Move the hands position
        if(cmnd.indexOf(F("hand")) >= 0) {
                String errorResponse = isValidCommand(cmnd, handParameters, 1);
                if(errorResponse.length() > 0) {return errorResponse; }
                float values[1];

                getCommandValues(cmnd, handParameters, 1, values);
                if(values[0] < 0 || values[0] > 180) {
                        return F("[ERROR: new value is not > 0 or < 180]");
                }
                currentHand = values[0]; //uarm.inputToReal(SERVO_HAND_ROT_NUM, values[0]);
                uarm.g_servo_hand_rot.write(currentHand);
                return ok + cmnd + endB;
        }
        //[buzzF261.63T1]

        // Set the Buzzer for a predetermined amount of time
        if(cmnd.indexOf(F("buzz")) >= 0) {
                String errorResponse = isValidCommand(cmnd, buzzerParameters, 2);
                if(errorResponse.length() > 0) {return errorResponse; }
                float values[2];

                getCommandValues(cmnd, buzzerParameters, 2, values);
                if(values[0] < 0 || values[1] < 0) {
                        return F("[ERROR: F & T should be > 0]");
                }
                tone(buzzerPin, values[0]);
                buzzerStopTime = millis() + int(float(values[1]) * float(1000));
                return ok + cmnd + endB;
        }

        // Get coords command
        if(cmnd.indexOf(F("gcoords")) >= 0) {
                double x = 0;   double y = 0;   double z = 0;
                //uarm.getCalXYZ(x, y, z);
                uarm.angle_to_coordinate(uarm.read_servo_angle(SERVO_ROT_NUM), uarm.read_servo_angle(SERVO_LEFT_NUM), uarm.read_servo_angle(SERVO_RIGHT_NUM), x, y, z);
                return "[coordsX" + String(x) + "Y" + String(y) + "Z" + String(z) + "]\n";
        }

        // Gets the angle of a particular servo
        if(cmnd.indexOf(F("gangle")) >= 0) {
                String errorResponse = isValidCommand(cmnd, gServoParameters, 1);
                if(errorResponse.length() > 0) {return errorResponse; }
                float values[1];

                getCommandValues(cmnd, gServoParameters, 1, values);
                float angle = uarm.read_servo_angle(values[0]);
                return "[angleA" + String(angle) + endB;
        }

        // Get whether or not the robot is currently moving
        if(cmnd.indexOf(F("gmoving")) >= 0) {
                int isMoving = 1;
                if(currentStep == 255) { isMoving = 0; }
                double timespend = goalTime - millis();
                double actTS = (timespend) / ((double) INTERP_INTVLS - currentStep);
                return "[movingM" + String(isMoving) + endB;
        }

        // Gets whether or not the tip sensor of the robot is pressed
        if(cmnd.indexOf(F("gtip")) >= 0) {
                int isTipPressed = digitalRead(2);
                return "[tipV" + String(isTipPressed) + endB;
        }


        if(cmnd.length() > 0) {
                return "[ERROR: No such command: " + cmnd + endB;
        }else{
                return F("");
        }
}




void getCommandValues(String cmnd, String parameters[], int parameterCount, float *valueArray){
        int index[parameterCount];
        for(int p = 0; p < parameterCount; p++) {
                index[p] = cmnd.indexOf(parameters[p]);
        }

        for(int p = 0; p < parameterCount; p++) {
                if(p < parameterCount - 1) {
                        valueArray[p] = cmnd.substring(index[p] + 1, index[p + 1]).toFloat();
                }else{
                        valueArray[p] = cmnd.substring(index[p] + 1).toFloat();
                }
        }
}




String isValidCommand(String cmnd, String parameters[], int parameterCount){
        int index[parameterCount];

        String errorMissingParameter = F("[ERROR: Missing Parameter ");
        String errorWrongOrder       = F("[ERROR: Incorrect Parameter order on parameter ");
        String endingBracket         = F("]\n");
        //  Get all indexes
        for(int p = 0; p < parameterCount; p++) {
                index[p] = cmnd.indexOf(parameters[p]);
                if(index[p] == -1) {return errorMissingParameter + parameters[p] + F(" ") + cmnd + endingBracket; }
        }

        //  Check that the commands are in the correct order
        for(int p = 0; p < parameterCount; p++) {
                if(parameterCount == 1) {break; }

                if(p < parameterCount - 1) {
                        if(!(index[p] < index[p + 1])) {
                                return errorWrongOrder + parameters[p] + endingBracket;
                        }
                }else if(!(index[p] > index[p-1])) {
                        return errorWrongOrder + parameters[p] + endingBracket;
                }
        }

        //  Check that there is something between each parameter (AKA, the value)
        for(int p = 0; p < parameterCount; p++) {
                if(p < parameterCount - 1) {
                        if((index[p + 1] - index[p]) == 1) {
                                return errorMissingParameter + parameters[p] + endingBracket;
                        }
                }else if(index[p] == cmnd.length() - 1) {
                        return errorMissingParameter + parameters[p] + endingBracket;
                }
        }

        return F("");
}

String parseNextCommand() {
        String None = F("");

        int cmndStart = message.indexOf('[');
        int cmndEnd   = message.indexOf(']');

        //PERFORM CLEANING-UP PROCEDURES
        if (cmndStart == -1 && cmndEnd == -1) {
                message = "";
                return None;
        } else if (cmndStart == -1 || cmndEnd == -1) {
                return None;
        }

        //If message starts with ][
        if (cmndStart > cmndEnd) {
                message.remove(0, cmndEnd + 1);
                return None;
        }

        // Finally, pull the command out, remove the brackets, make it lowercase, and return it
        String cmnd = message.substring(cmndStart + 1, cmndEnd);
        message.remove(0, cmndEnd + 1);
        cmnd.toLowerCase();
        return cmnd;
}


void setMove(double x, double y, double z, double goalSpeed) {


        // Limit the range of the uArm
        float limit = sqrt((x*x + y*y));
        if (limit > 32)
        {
                float k = 32/limit;
                x = x * k;
                y = y * k;
        }




        // find current XYZ position using cached servo values
        double current_x;
        double current_y;
        double current_z;

        //double cur_rot   = uarm.readAngle(SERVO_ROT_NUM);
        //double cur_right = uarm.readAngle(SERVO_RIGHT_NUM);
        //double cur_left  = uarm.readAngle(SERVO_LEFT_NUM);
        double cur_rot   = uarm.read_servo_angle(SERVO_ROT_NUM);
        double cur_left   = uarm.read_servo_angle(SERVO_LEFT_NUM);
        double cur_right = uarm.read_servo_angle(SERVO_RIGHT_NUM);


        uarm.angle_to_coordinate(cur_rot,  cur_left, cur_right, current_x, current_y, current_z);


        // find target angles
        double tgt_rot;
        double tgt_left;
        double tgt_right;
        uarm.coordinate_to_angle(x, y, z, tgt_rot, tgt_left, tgt_right);


        // calculate the length, to calculate the # of interpolations that will be necessary
        unsigned int delta_rot   = abs(tgt_rot   - cur_rot);
        unsigned int delta_left  = abs(tgt_left  - cur_left);
        unsigned int delta_right = abs(tgt_right - cur_right);

        // CUSTOM: Use the robots current position and its desired destination to calculate the amount of time for the move to occur within
        double distance   = pow(pow(x-current_x, 2) + pow(y-current_y, 2) + pow(z-current_z, 2), .5);


        // Calculate the number of interpolations for this move + " "
        INTERP_INTVLS = max(delta_rot,delta_left);
        INTERP_INTVLS = max(INTERP_INTVLS,delta_right);
        INTERP_INTVLS = (INTERP_INTVLS<50) ? INTERP_INTVLS : 50; // Max it out at 50



        // Create the movement path
        uarm.INTERP_INTVLS = INTERP_INTVLS;
        uarm.interpolate(current_x, x, x_array, INTERP_LINEAR);
        uarm.interpolate(current_y, y, y_array, INTERP_LINEAR);
        uarm.interpolate(current_z, z, z_array, INTERP_LINEAR);

        // Make the final cell of the interpolation be the actual destination
        x_array[INTERP_INTVLS] = x;
        y_array[INTERP_INTVLS] = y;
        z_array[INTERP_INTVLS] = z;

        // uarm.interpolate(cur_hand, hand_angle, hand_array, INTERP_LINEAR);  // TODO: ADD WRIST INTERPOLATIONS LATER

        double time_spend = distance / goalSpeed; // Speed of the robot in cm/s
        currentStep  = 0;
        goalTime     = millis() + time_spend * 1000;
        goalTimeStep = abs((time_spend * 1000) / INTERP_INTVLS); // How long each timestep *should* take
}


bool isTimeToMove(){
        // If not currently in "move" mode, return false
        if (currentStep >= INTERP_INTVLS) {
                return false;
        }


        // If it's currently due for another step
        double timeLeft = goalTime - millis();
        if (timeLeft / (INTERP_INTVLS - currentStep)    <     goalTimeStep) {
                return true;
        }


        // If the robot has run out of time, then of course it has to move immediately

        if(timeLeft <= 0) {
                return true;
        }


        return false;
}


void moveStep() {
        //  Move one 'step' towards the desired location, but only if the timing is right


        //Actually perform the step
        double rot,left, right;

        // Find target angle for the step, and write it
        uarm.coordinate_to_angle(x_array[currentStep], y_array[currentStep], z_array[currentStep], rot, left, right);


        uarm.write_servo_angle(rot, left, right, currentHand);
        currentStep += 1;


        if (currentStep == INTERP_INTVLS) {
                // Make the final move, to ensure arrival to the final destination.
                uarm.coordinate_to_angle(x_array[INTERP_INTVLS], y_array[INTERP_INTVLS], z_array[INTERP_INTVLS], rot, left, right);
                uarm.write_servo_angle(rot, left, right, currentHand);
                currentStep = 255;
        }
}
