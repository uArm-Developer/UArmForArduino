#include <EEPROM.h>
#include <Wire.h>
#include <uArm_library.h>
#include <Servo.h>

 // define uarm class

void hitPoint(int pointNumber);     // predefine a hitPoint function

void setup() {
      Wire.begin();        // join i2c bus (address optional for master)
      Serial.begin(9600);  // start serial port at 9600 bps
}


int pointNumber = 7;                // there are 7 points in total need to be hit
int theta4Offset = 90;              // the angle between the 4th servo and 1st servo
double h1 = 10;                     // the height of the point
double h2 = 5;                      // the height of the point above the hitting point (get ready to hit)
double y1 = -22;                    // the distance along the y axis

int timeDelay1 = 100;               // time delay
int timeDelayBetweenPoints = 500;   // time delay between hitting two points
int point1Delay = 1000;             // before hitting the first point, uarm will move above it in advance

double points[7][3] = {             // define 7 points, each point has an unique location (x,y,z)
  { 7.1,   y1,   h1} ,              // point 1
  { 4.5,   y1,   h1} ,              // point 2
  { 2.2,   y1,   h1} ,              // point 3
  { -0.3,  y1,   h1} ,              // point 4
  { -3.6,  y1,   h1} ,              // point 5
  { -6.3,  y1,   h1} ,              // point 6
  { -9,    y1,   h1} ,              // point 7
};


void loop() {
  
    if(Serial.available()>0)
    {
      char readSerial = Serial.read();      // read serial input
      Serial.println(readSerial);
  
      switch (readSerial)
      {
        // case 1 : execute hitting process
        case '1':                           // input number 1 to strart hitting
        {
          hitPoints();                      // hitPoints function shows below
          break;
        }
  
        // case 2: move to a home position (x,y,z) = (0,-20,10)
        case '2':
        {
          uarm.moveTo(0,-20,10);
          break;
        }
  
         // case 3: detach all servos that you can manually move uArm
         case '3':
        {
          uarm.detachAll();
          break;
        }

         // case 4: play a piece of music if you have a toy piano
         case '4':
        {
          playMusic();
          break;
        }
        
  
        // ignore other inputs
        default:
          break;
        }
      delay(50);
    }
}


// the whole hitting process
void hitPoints()
{
    moveToPoint(1,true);          // move to the point above the first point and get ready to hit
    delay(point1Delay);           
    
    for (int i = 1; i<pointNumber; i ++)    // execute hitting process from the first point to the last point
    {
      hitPoint(i);                          // each hit process
      delay(timeDelayBetweenPoints);
     }
   
}


// each hitting procedure
void hitPoint(int pointNumber)
{
    moveToPoint(pointNumber,true);      // move to the point above the point, 
    
    delay(timeDelay1);
    moveToPoint(pointNumber,false);     // move downwards
    
    delay(timeDelay1);
    moveToPoint(pointNumber,true);      // quick move upwards ( back )
  
}

// move uarm to the point
void moveToPoint(int pointNumber, boolean highPosition)
{
  if  (highPosition == true){           // "true" means the end-effector will move to the point above hitting point. The height of this point is h2 above the real hitting point
      uarm.calAngles(points[pointNumber-1][0],points[pointNumber-1][1],points[pointNumber-1][2]+h2);      // calculate the angle need to be executed by implement inverse kinematics
    }
  else{ 
      uarm.calAngles(points[pointNumber-1][0],points[pointNumber-1][1],points[pointNumber-1][2]);
    }
    
    delay(10);
    uarm.writeAngle(uarm.getTheta1(),uarm.getTheta2(),uarm.getTheta3(),uarm.getTheta1()+theta4Offset);    // execute calculated angles by getting each angle
}


// play a piece of music
void playMusic()
{
  // the music is called 
   int shortDelay = 200;
   int longDelay = 700;

    moveToPoint(1,true);
    delay(longDelay*2);
    
    hitPoint(1);
    delay(shortDelay);
    hitPoint(2);
    delay(longDelay);
    hitPoint(1);
    delay(shortDelay);
    hitPoint(2);
    delay(longDelay);
    hitPoint(1);
    delay(shortDelay);
    hitPoint(2); 
    delay(shortDelay);
    hitPoint(3);
    delay(shortDelay);
    hitPoint(4);
    delay(shortDelay);
    hitPoint(5);
    delay(shortDelay*4); 
   
    hitPoint(5);
    delay(shortDelay);
    hitPoint(3);
    delay(longDelay);

    hitPoint(5);
    delay(shortDelay);
    hitPoint(3);
    delay(longDelay);


    hitPoint(5);
    delay(shortDelay);
    hitPoint(4);
    delay(shortDelay);
    hitPoint(3);
    delay(shortDelay);
    hitPoint(2); 
    delay(shortDelay);
    hitPoint(1);
    delay(shortDelay*8);

}

