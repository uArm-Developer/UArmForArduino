/******************************************************************************
* File Name          : uArmLibrary.cpp
* Author             : Evan 
* Updated            : Jerry Song 
* Email              : jerry.song@evol.net
* Version            : V1.2 
* Date               : 12 Dec, 2014
* Modified Date      : 26 Aug, 2015
* Description        : 
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*******************************************************************************/

#include "uArm_Library_Metal.h"
#include <math.h>


uArmLibrary uArm;
int angleR;
int angleL;
int angleBottom;

uArmLibrary::uArmLibrary()
{
	heightLst				  = 0;
	height					  = 0;
	stretch					  = 0;
	rotation				  = 0;
	handRot	          = 0;
  lstTime					  = 0;
	delay_loop			  = 0;
	servoSpdR				  = 0;
	servoSpdL				  = 0;
	servoSpdRot			  = 100;
	servoSpdHand		  = 0;
	servoSpdHandRot	  = 0;
	leftServoLast		  = 110;
  rightServoLast	  = 100;
	rotServoLast		  = 90;
	handrotLast       = 90;
	sampleDelay			  = 10;
	playFlag				  = false;
	recordFlag			  = true;
	firstFlag				  = true;
	gripperRst			  = true;
	griperState[14]	  = 0;
	data[5] = 0;  // 0: L  1: R  2: Rotation 3: hand rotation 4:gripper
}

void uArmLibrary::init()
{
    // read offset data
 
    // initialization the pin
    pinMode(LIMIT_SW, INPUT_PULLUP);  digitalWrite(LIMIT_SW, HIGH);
    pinMode(BTN_D4,   INPUT_PULLUP);  digitalWrite(BTN_D4,   HIGH);
    pinMode(BTN_D7,   INPUT_PULLUP);  digitalWrite(BTN_D7,   HIGH);
    pinMode(BUZZER,   OUTPUT); digitalWrite(BUZZER,   LOW);
    pinMode(PUMP_EN,  OUTPUT); digitalWrite(PUMP_EN,  LOW);
    pinMode(VALVE_EN, OUTPUT); digitalWrite(VALVE_EN, LOW);
  if (EEPROM.read(0) == CALIBRATION_FLAG) // read of offset flag
    {
    // attaches the servo on pin to the servo object
   // servoL.attach(SERVO_L);
   // servoR.attach(SERVO_R);
   // servoRot.attach(SERVO_ROT);
  //  servoHand.attach(SERVO_HAND);
  //  servoHandRot.attach(SERVO_HAND_ROT);


    // initialization postion
    }
    else
    { // buzzer alert if calibration needed
    alert(3, 200, 200);
    }
  
}

void uArmLibrary::recordingMode(unsigned char _sampleDelay)
{
	sampleDelay = _sampleDelay;

  // D4 button - Recording mode

  if(!digitalRead(BTN_D4))
  {
    alert(2, 200, 100);
 
    delay(50);
		servoL.detach();
		servoR.detach();
		servoRot.detach();
		servoHandRot.detach();
		
    while(1)
		{
      // D4 button - recording
      if(!digitalRead(BTN_D4))
	     {
		      recordFlag = true;
          alert(1, 50, 0);
               lstTime = millis();
               while(!digitalRead(BTN_D4))
               {
                  if(millis() - lstTime > BTN_TIMEOUT_1000)//make the new data to the old data for long time storage
		              {
                     recordFlag = false;
		                 unsigned char dat[2];
                     readExternalEeprom(0x0000, &dat[0],1);
                     delay(5);
                     readExternalEeprom(0x8000, &dat[1],1);
                     delay(5);

                     if((dat[0]==new_data)&&(dat[1]==old_data))
                     {
                        //addr = 1;
                        dat[0] = old_data;
                        writeExternalEeprom(0x0000, dat,1);
                        delay(5);
                        dat[0] = new_data;                        
                        writeExternalEeprom(0x8000, dat,1);
                        delay(5);
                        dat[0] = 255;                        
                        writeExternalEeprom(0x8001, dat,1);
                        delay(5);                      
                     }

                     else if((dat[0]==old_data)&&(dat[1]==new_data))
                     {
                        dat[0] = new_data;
                        writeExternalEeprom(0x0000, dat,1);
                        delay(5);
                        dat[0] = 255;
                        writeExternalEeprom(0x0001, dat,1); 
                        delay(5);                      
                        dat[0] = old_data;                        
                        writeExternalEeprom(0x8000, dat,1); 
                        delay(5);
                     }

                     else
                     {
  	                    //addr = 1;
  	                    dat[0] = new_data;
  	                    writeExternalEeprom(0x0000, dat,1);	
  	                    delay(5);
  	                    dat[0] = 255;
  	                    writeExternalEeprom(0x0001, dat,1);	
  	                    delay(5);
  	                    dat[0] = old_data;
  	                    writeExternalEeprom(0x8000, dat,1);	
  	                    delay(5);
  	                    dat[0] = 255;
  	                    writeExternalEeprom(0x8001, dat,1);	
  	                    delay(5);
                     }   
      			         alert(1, 300, 0);
      			       
                     while(!digitalRead(BTN_D4));
		              }
		            }
               delay(20);
             
               if(recordFlag)
                 record(BTN_D4, BTN_D7);
	           }

      // D7 button - play
      if(!digitalRead(BTN_D7))
      {
        playFlag = false;
        alert(1, 100, 0); 
 
        lstTime = millis();
        while(!digitalRead(BTN_D7))
        {
         if(millis() - lstTime > BTN_TIMEOUT_1000)
          { 
            playFlag = true;
            alert(1, 300, 0);
            while(!digitalRead(BTN_D7));
          }
        }

        delay(20);
        play(BTN_D7);
	   }
	}
  }
}


void uArmLibrary::gripperCatch()
{
  servoHand.attach(SERVO_HAND);
  servoHand.write(HAND_ANGLE_CLOSE);
  digitalWrite(VALVE_EN, LOW); // valve disnable
  digitalWrite(PUMP_EN, HIGH); // pump enable
  gripperRst = true;
}

void uArmLibrary::gripperRelease()
{
	if(gripperRst)
	{
    servoHand.attach(SERVO_HAND);
    servoHand.write(HAND_ANGLE_OPEN);
    digitalWrite(VALVE_EN, HIGH); // valve enable, decompression
    digitalWrite(PUMP_EN, LOW);   // pump disnable
    gripperRst = false;
    delay_loop = 0;
  }
}

void uArmLibrary::gripperDetach()
{
	if(++delay_loop > 300000)        // delay release valve
	{
    servoHand.detach();
    digitalWrite(VALVE_EN, LOW); // valve disnable
    delay_loop=0;
  }
}

void uArmLibrary::gripperDirectDetach()
{
	servoHand.detach();
	digitalWrite(PUMP_EN, LOW);   // pump disnable
	digitalWrite(VALVE_EN, LOW); // valve disnable
}

void uArmLibrary::pumpOn()
{
	digitalWrite(PUMP_EN, HIGH);    // pump enable
}

void uArmLibrary::pumpOff()
{
	digitalWrite(PUMP_EN, LOW);     // pump disnable
}

void uArmLibrary::valveOn()
{
	digitalWrite(VALVE_EN, HIGH);   // valve enable, decompression
}

void uArmLibrary::valveOff()
{
	digitalWrite(VALVE_EN, LOW);    // valve disnable
}

void uArmLibrary::detachServo(char _servoNum)
{
	switch(_servoNum)
	{
		case SERVO_L:
		servoL.detach();
		break;
		case SERVO_R:
		servoR.detach();
		break;
		case SERVO_ROT:
		servoRot.detach();
		break;
		case SERVO_HAND_ROT:
		servoHandRot.detach();
		break;
		case SERVO_HAND:
		servoHand.detach();
		break;
		default: break;
	}
}

void uArmLibrary::sendData(byte _dataAdd, int _dataIn)
{
	Serial.write(0xFF); Serial.write(0xAA); // send data head
	Serial.write(_dataAdd);
	Serial.write(*((char *)(&_dataIn) + 1));
	Serial.write(*((char *)(&_dataIn)));
}

void uArmLibrary::alert(int _times, int _runTime, int _stopTime)
{
	for(int _ct=0; _ct < _times; _ct++)
	{
		delay(_stopTime);
		digitalWrite(BUZZER, HIGH);
		delay(_runTime);
		digitalWrite(BUZZER, LOW);
	}
}

void uArmLibrary::play(unsigned char buttonPin)
{
  unsigned char dat[2];
  readExternalEeprom(0x8000, &dat[0],1); 
  delay(5);
  //readExternalEeprom(0x0000, &dat[0],1);
  if(record_status==have_done_record_since_power)
  {
    if(dat[0]==new_data)
    {
      addr = 0x8001;
    }
    else 
    {
      addr = 1;
    }
  }
  else
  {
    if(dat[0]==old_data)
    {
      addr = 0x8001;
    }
    else 
    {
      addr = 1;
    }  	
  }
  

  attachAll();
  servoL.attach(SERVO_L);
  servoR.attach(SERVO_R);
  servoRot.attach(SERVO_ROT);
  servoHandRot.attach(SERVO_HAND_ROT);
  while(digitalRead(buttonPin))
  {
  	readExternalEeprom(addr, data, 5);
    if(data[0] == DATA_FLAG)
	{
		if(playFlag)
		{
            readExternalEeprom(0x8000, &dat[0],1); 
            if(record_status==have_done_record_since_power)
            {
              if(dat[0]==new_data)
              {
                addr = 0x8001;
              }
              else 
              {
                addr = 1;
              }
            }
            else
            {
              if(dat[0]==old_data)
              {
                addr = 0x8001;
              }
              else 
              {
                addr = 1;
              }  	
            }
            readExternalEeprom(addr, data, 5);
            delay(5);	
		}
		else break;
	}

    unsigned char leftServo  = data[0];
    unsigned char rightServo = data[1];
    unsigned char rotServo   = data[2];
    unsigned char handrot    = data[3];

		if(data[4]==0x01)
			gripperCatch();
		else
			gripperRelease();
       
    servoBufOutL(leftServoLast,  leftServo);
    servoBufOutR(rightServoLast, rightServo);
    servoBufOutRot(rotServoLast, rotServo);
    servoBufOutHandRot(handrotLast, handrot);
 
    leftServoLast  = leftServo;
    rightServoLast = rightServo;
    rotServoLast   = rotServo;
    handrotLast    = handrot;
    delay(sampleDelay);
    addr+=5; 
  }
  
  servoL.detach();
  servoR.detach();
  servoRot.detach();
  servoHandRot.detach();
  gripperDirectDetach();
  alert(1, 250, 100);
  delay(250);
}

void uArmLibrary::record(unsigned char buttonPin, unsigned char buttonPinC)
{
  unsigned char dat[2];
  readExternalEeprom(0x0000, &dat[0],1);
  delay(5);
  readExternalEeprom(0x8000, &dat[1],1);
  delay(5);
  if((dat[0]==new_data)&&(dat[1]==old_data))
  {
    addr = 1;
    dat[0] = new_data;
    writeExternalEeprom(0x0000, dat,1);  
    delay(5);
  }
  else if((dat[0]==old_data)&&(dat[1]==new_data))
  {
  	addr = 0x8001;
  	dat[0] = new_data;
  	writeExternalEeprom(0x8000, dat,1);  
  	delay(5);
  }
  else
  {
  	addr = 1;
  	dat[0] = new_data;
  	writeExternalEeprom(0x0000, dat,1);	 
  	dat[0] = old_data;
  	delay(5);
  	writeExternalEeprom(0x8000, dat,1);	
  	delay(5);
  }

  record_status = have_done_record_since_power;
  //unsigned char addrC = 0; 
  boolean btnSt = false;
  //memset(griperState,0,14); // reset griperState array
  gripperDirectDetach();
  
	while(digitalRead(buttonPin))
    {

    unsigned int leftServo  = (int) readAngle(servoLNumber);
    unsigned int rightServo = (int) readAngle(servoRNumber);
    unsigned int rotServo   = (int) readAngle(servoRotNumber);
    unsigned int handrot    = (int) readAngle(servoHandRotNumber);
    
		data[0] = leftServo;
		data[1] = rightServo;
		data[2] = rotServo;
		data[3] = handrot;

		if(!digitalRead(buttonPinC))
		{
			while(!digitalRead(buttonPinC));
			delay(10);
			if(btnSt)
			{
				gripperRelease();
				alert(2, 50, 50);
			}
			else
			{
				gripperCatch();
				alert(1, 50, 0);
			}
			btnSt = !btnSt;
		}


			
		data[4] = (btnSt == false) ? 0 : 1;

		if((addr % 32768) >= MEMORY_SERVO_PER)
		{
			alert(3, 50, 100);
			break;
		}

    writeExternalEeprom(addr, data, 5);
    addr += 5;
    delay(sampleDelay - 2);
    }
    
    data[0] = 255;

    writeExternalEeprom(addr, data,1);
    gripperDirectDetach();
    while(digitalRead(buttonPin));

    alert(2, 50, 100);

    firstFlag = false;

}

void uArmLibrary::writeExternalEeprom(unsigned int address, unsigned char * data_array, int num)
{
  unsigned int i=0,j=0;

  j=(address%128);
  if((j>=124)&&(num==5))
  {
  	j=128-j;
  	Wire.beginTransmission(0xa0>>1); // transmit to device #c0
  	Wire.write(byte(address>>8));
  	Wire.write(byte(address%256));//byte(first_byte_address&&0x00ff));//first_byte_address&&0xFF);
  	
    for(i=0;i<j;i++)
  	{
  	  Wire.write(*(data_array+i));              // sends one byte
  	}
  	
    Wire.endTransmission();    // stop transmitting
  	address+=j;
  	delay(5);
  	Wire.beginTransmission(0xa0>>1); // transmit to device #c0
  	Wire.write(byte(address>>8));
  	Wire.write(address%256);//byte(first_byte_address&&0x00ff));//first_byte_address&&0xFF);
  	
    for(i=0;i<num-j;i++)
  	{
  	  Wire.write(*(data_array+j+i));              // sends one byte
  	}
  
  	Wire.endTransmission();    // stop transmitting
  	
  }
  else
  {
  	Wire.beginTransmission(0xa0>>1); // transmit to device #c0
  	Wire.write(byte(address>>8));
  	Wire.write(byte(address%256));//byte(first_byte_address&&0x00ff));//first_byte_address&&0xFF);
  	for(i=0;i<num;i++)
  	{
  	  Wire.write(*(data_array+i));              // sends one byte
  	
  	}
    
  	Wire.endTransmission();    // stop transmitting
  }
}

void uArmLibrary::readExternalEeprom(unsigned int address, unsigned char * data_array, int num)
{
  int i=0;
  
  	Wire.beginTransmission(0xa0>>1); // transmit to device #c0
  	Wire.write(byte(address>>8));
  	Wire.write(byte(address%256));//byte(first_byte_address&&0x00ff));//first_byte_address&&0xFF);
  	Wire.endTransmission(); 
  	Wire.requestFrom(0xa0>>1,num);    // request 6 bytes from slave device #2
  	for(i=0;i<num;i++)//while (Wire.available())   // slave may send less than requested
  	{
    	*(data_array+i) = Wire.read(); // receive a byte as character
    //i++;
  	}
  
}

void uArmLibrary::servoBufOutL(unsigned char _lastDt, unsigned char _dt)
{

	_dt = inputToReal(2, _dt);

	if(abs(_dt - _lastDt) > BUFFER_OUTPUT)     // goes from Min degrees to Max degrees 
	  servoL.write(_dt);
	else 
	servoL.write(_dt); 
}

void uArmLibrary::servoBufOutR(unsigned char _lastDt, unsigned char _dt)
{
	_dt = inputToReal(3, _dt);
	if(abs(_dt - _lastDt) > BUFFER_OUTPUT)
		servoR.write(_dt);
	else 
	servoR.write(_dt);
}

void uArmLibrary::servoBufOutRot(unsigned char _lastDt, unsigned char _dt)
{
	_dt = inputToReal(1, _dt);
	if(abs(_dt - _lastDt) > BUFFER_OUTPUT)
		servoRot.write(_dt);
	else 
	servoRot.write(_dt);
}
void uArmLibrary::servoBufOutHandRot(unsigned char _lastDt, unsigned char _dt)
{
	_dt = inputToReal(4, _dt);
	if(abs(_dt - _lastDt) > BUFFER_OUTPUT)
		servoHandRot.write(_dt);
	else 
	servoHandRot.write(_dt);
}
//******************************************************************


/* The code below is written by jerry song */

void uArmLibrary::writeAngle(int servoRotAngle, int servoLAngle, int servoRAngle, int servoHandRotAngle, int trigger)
{
	attachAll();

	servoLAngle = round(servoLAngle);
	servoRAngle = round(servoRAngle);
	servoHandRotAngle = round(servoHandRotAngle);
	servoRotAngle = round(servoRotAngle);


	servoRot.write(servoRotAngle);
	servoL.write(servoLAngle);
	servoR.write(servoRAngle);
	servoHandRot.write(servoHandRotAngle);


}

void uArmLibrary::writeAngle(int servoRotAngle, int servoLAngle, int servoRAngle, int servoHandRotAngle)
{
	attachAll();

	servoLAngle = round(servoLAngle);
	servoRAngle = round(servoRAngle);
	servoHandRotAngle = round(servoHandRotAngle);
	servoRotAngle = round(servoRotAngle);


	servoRot.write(inputToReal(1,servoRotAngle));
	servoL.write(inputToReal(2,servoLAngle));
	servoR.write(inputToReal(3,servoRAngle));
	servoHandRot.write(inputToReal(4,servoHandRotAngle));


}

void uArmLibrary::attachAll()
{
	

	servoRot.attach(11);
	servoL.attach(13);
	servoR.attach(12);
	servoHandRot.attach(10);
}

void uArmLibrary::detachAll()
{
	
	servoRot.detach();
	servoL.detach();
	servoR.detach();
	servoHandRot.detach();

}


int uArmLibrary::inputToReal(int servoNumber, int inputAngle)
{

	int output = inputAngle + servoOffset(servoNumber);
	
	if (output > 180) output = 180;
	if (output < 0) output = 0;
	
	return output;


}


double uArmLibrary::servoOffset(int servoNumber)
{


	if ((servoNumber == 1)||(servoNumber == 2)||(servoNumber == 3))
	{
		SERVO_OFFSET = (EEPROM.read(addressOffset + (servoNumber-1)*2 +1))/10.00;

		if (EEPROM.read(addressOffset + (servoNumber-1)*2 ) == 0)
			{SERVO_OFFSET = -SERVO_OFFSET;}

		return SERVO_OFFSET;
	}	
	else if(servoNumber == 4)
		return 0;
	else{
			Serial.println("Incorrect");
			
	}
}


void uArmLibrary::calibrationServo(int servoNumber)
{

	int servoRangeIni = 20 ;
	int servoRangeFin = 100;
	double readAngleFromAnalog;
	double real[16];
	double input[16];
	int address = addressServo + (servoNumber-1)*6;
	
	Serial.print("servo ");
  Serial.println(servoNumber);


	int analogReadPin;
	
	for (int i = 0; i < (((servoRangeFin-servoRangeIni)/5)+1); i ++)
	{
		int dot_i = 5*i;
		switch(servoNumber)
		{
			case 1:
				analogReadPin = 2;
				writeAngle(servoRangeIni+dot_i, 60, 60, 0,1);

			
				break;

			case 2:
				analogReadPin = 0;
				writeAngle(90, servoRangeIni+dot_i, 30, 0,1);
			
				break;

			case 3:
				analogReadPin = 1;
				writeAngle(90, 60 , servoRangeIni+dot_i, 0,1);
			
				break;

			case 4:
				analogReadPin = 3;
				writeAngle(90, 60, 60, servoRangeIni+dot_i,1);
				break;

			default:
			
				break;
		}
		
		
		if(i == 0) {
			delay(2000);
		}

		for (int l = 0 ; l<3;l++) {
				readAngleFromAnalog = analogRead(analogReadPin);
				delay(100);
		}

		real[i] = servoRangeIni + dot_i;
		input[i] = readAngleFromAnalog;

		delay(800);

	}
	real[0] = servoRangeIni;

	LinearRegression lr(input, real, 16);

	saveDataToRom(lr.getA(),address);
	saveDataToRom(lr.getB(),address+3);

}


void uArmLibrary::calibrations(){
	
	for (int k = 1; k < 5; k++){

		calibrationServo(k);
		delay(2000);
	}

	setOffset();
	EEPROM.write(0, CALIBRATION_FLAG);

	Serial.println("All done!");
}


void uArmLibrary::saveDataToRom(double data, int address)
{
	
	int dataWhole;
	
	byte Byte0;
	byte Byte1;
	byte Byte2;

	if(abs(data) < 1)	{
		dataWhole = (int) (data*100);
	}
	else{
		dataWhole = (int) (data*10);
	}



	if (dataWhole > 0){ 
		Byte0 = 1;
	}
	else{ 
		Byte0 =0; 
	}

	dataWhole = abs(dataWhole);

	Byte1 = (( dataWhole >> 0) & 0xFF);
	Byte2 = (( dataWhole >> 8) & 0xFF);
	

	EEPROM.write( address, Byte0);
	EEPROM.write( address + 1, Byte1);
	EEPROM.write( address + 2, Byte2);


}

void uArmLibrary::calAngles(double x, double y, double z)
{
	yIn = (-y-l2)/l3;
	zIn = (z - l1) / l3;
	l43 = l4 / l3;
	Right_all = (1 - yIn*yIn - zIn*zIn - l43*l43) / (2 * l43);
	sqrt_z_y = sqrt(zIn*zIn + yIn*yIn);
	
  if(z > (l1+l3))
    {z = 25;}

	if (x == 0)
	{
		// Calculate value of theta 1
		theta_1 = 90;

		// Calculate value of theta 3
		if (zIn == 0) {
			phi = 90;
		}

		else {
		phi = atan(-yIn / zIn)*trans;
		}

		theta_3 = asin(Right_all / sqrt_z_y)*trans - phi;
		theta_3 = 180 - abs(theta_3);

		// Calculate value of theta 2
		
		theta_2 = asin((z + l4*sin(theta_3 / trans) - l1) / l3)*trans;
	}

	else
	{
		// Calculate value of theta 1

		theta_1 = atan(y / x)*trans;

		if (y / x > 0) {
			theta_1 = theta_1;
		}
		if (y / x < 0) {
			theta_1 = theta_1 + 180;
		}
		if (y == 0)	{
			if (x > 0) theta_1 = 180;
			else theta_1 = 0;				
		}
		
		// Calculate value of theta 3

		xIn = (-x / cos(theta_1 / trans) - l2) / l3;

		if (zIn == 0){ phi = 90; }
			
		else{ phi = atan(-xIn / zIn)*trans; }
			
		
		sqrt_z_x = sqrt(zIn*zIn + xIn*xIn);

		Right_all_2 = -1 * (zIn*zIn + xIn*xIn + l43*l43 - 1) / (2 * l43);
		theta_3 = asin(Right_all_2 / sqrt_z_x)*trans;
		theta_3 = theta_3 - phi;

		if (abs(theta_3) > 90) {
			theta_3 = 180 - abs(theta_3);
		}


		// Calculate value of theta 2

		theta_2 = asin(zIn + l43*sin(abs(theta_3 / trans)))*trans;

	}
	
		
	theta_1 = abs(theta_1);
	theta_2 = abs(theta_2);
	
  
	if (theta_3 < 0 ){}
  else{
    if ((calYonly(theta_1,theta_2, theta_3)>y+0.1)||(calYonly(theta_1,theta_2, theta_3)<y-0.1))
    {
      theta_2 = 180 - theta_2;
    }  
  }
  

  if(isnan(theta_1)||isinf(theta_1))
    {theta_1 = readAngle(1);}
  if(isnan(theta_2)||isinf(theta_2))
    {theta_2 = readAngle(2);}
  if(isnan(theta_3)||isinf(theta_3)||(theta_3<0))
    {theta_3 = readAngle(3);}

}

void uArmLibrary::interpolation(double initialValue, double finalValue)
{
	// by using the formula theta_t = a_0 + a_1 * t + a_2 * t^2 + a_3 * t^3
	// theta(0) = initialValue; theta(t_f) = finalValue
	// theta_dot(0) = 0; theta_dot(t_f) = 0

	double a_0;
	double a_1;
	double a_2;
	double a_3;
	double t_step;
	
	int timeTotal = 1;

	a_0 = initialValue;
	a_1 = 0;
	a_2 = (3 * (finalValue - initialValue)) / (timeTotal*timeTotal);
	a_3 = (-2 * (finalValue - initialValue)) / (timeTotal*timeTotal*timeTotal);

	for (int i = 0; i < 50; i=i+1)
	{
		t_step = (timeTotal / 50.0) *i;
		interpolValueArray[i] = a_0 + a_1 * (t_step) + a_2 * (t_step *t_step ) + a_3 * (t_step *t_step *t_step);	
	}
}

void uArmLibrary::calXYZ(double theta_1, double theta_2, double theta_3)
{
	
	l3_1 = l3 * cos(theta_2 / trans);
	l4_1 = l4*cos(theta_3 / trans);
	l5 = (l2 + l3*cos(theta_2 / trans) + l4*cos(theta_3 / trans));

	calX = -cos(abs(theta_1 / trans))*l5;
	calY = -sin(abs(theta_1 / trans))*l5;
	calZ = l1 + l3*sin(abs(theta_2 / trans)) - l4*sin(abs(theta_3 / trans));

}

void uArmLibrary::calXYZ()
{
	
	calXYZ(
	readToAngle(analogRead(2),1,0),
	readToAngle(analogRead(0),2,0),
	readToAngle(analogRead(1),3,0));

}




void uArmLibrary::moveTo(double x, double y, double z)
{
	attachAll();
	
	double currentX;
	double currentY;
	double currentZ;

	double xArray[50];
	double yArray[50];
	double zArray[50];

	calXYZ();
	currentX = calX;
	currentY = calY;
	currentZ = calZ;

	interpolation(currentX, x);
	for (int i = 0; i < 50; i++){
		xArray[i] = interpolValueArray[i];
		
	}

	interpolation(currentY, y);
	for (int i = 0; i < 50; i++){
		yArray[i] = interpolValueArray[i];
		
	}

	interpolation(currentZ, z); 
	for (int i = 0; i < 50; i++){
		zArray[i] = interpolValueArray[i];
		
	}
		
	for (int i = 0; i < 50; i++)
	{
		calAngles(xArray[i],yArray[i],zArray[i]);

		writeAngle(theta_1, theta_2, theta_3,0);

		delay(40);

	}

}

void uArmLibrary::moveTo(double x, double y, double z, int relative, double timeSpend)
{

	double xArray[50];
	double yArray[50];
	double zArray[50];

	calXYZ();
	currentX = calX;
	currentY = calY;
	currentZ = calZ;
	
	if ((relative !=0)&&(relative != 1))
	{	
		relative = 0;

	}

	// if (timeSpend <0)
	// {	
	// 	timeSpend = abs(timeSpend);
		
	// }

	interpolation(currentX, currentX*relative+x);

	for (int i = 0; i < 50; i++){

		xArray[i] = interpolValueArray[i];
		
	}

	interpolation(currentY, currentY*relative+y);
	
	for (int i = 0; i < 50; i++){

		yArray[i] = interpolValueArray[i];
		
	}

	interpolation(currentZ, currentZ*relative+z); 
	
	for (int i = 0; i < 50; i++){

		zArray[i] = interpolValueArray[i];
		
	}
		
	for (int i = 0; i < 50; i++)
	{
		calAngles(xArray[i],yArray[i],zArray[i]);
		writeAngle(theta_1, theta_2, theta_3,0);

		delay(timeSpend*1000/50);

	}

}


void uArmLibrary::moveTo(double x, double y, double z, int relative, double timeSpend, double servo_4_angle)
{

  double xArray[50];
  double yArray[50];
  double zArray[50];

  calXYZ();
  currentX = calX;
  currentY = calY;
  currentZ = calZ;

  if ((relative !=0)&&(relative != 1))
  { 
    relative = 0;
  }

  if (timeSpend <0)
  { 
    timeSpend = abs(timeSpend);
  }

  interpolation(currentX, currentX*relative+x);

  for (int i = 0; i < 50; i++){

    xArray[i] = interpolValueArray[i];
    
  }

  interpolation(currentY, currentY*relative+y);
  
  for (int i = 0; i < 50; i++){

    yArray[i] = interpolValueArray[i];
    
  }

  if ( currentZ*relative+z>25)
    { interpolation(currentZ, 25); }
  else
    { interpolation(currentZ, currentZ*relative+z); }
  
  for (int i = 0; i < 50; i++){

    zArray[i] = interpolValueArray[i];
    
  }
    
  for (int i = 0; i < 50; i++)
  {
    calAngles(xArray[i],yArray[i],zArray[i]);
    writeAngle(theta_1, theta_2, theta_3, servo_4_angle);

    delay(timeSpend*1000/50);
  }

}

void uArmLibrary::drawRec(double length_1, double length_2, double timeSpendPerLength)
{

	moveTo(length_1,0,0,1,timeSpendPerLength);
	moveTo(0,length_2,0,1,timeSpendPerLength);
	moveTo(-length_1,0,0,1,timeSpendPerLength);
	moveTo(0,-length_2,0,1,timeSpendPerLength);
	
}

void uArmLibrary::drawCur(double length_1, double length_2, int angle, double timeSpend)
{
	attachAll();
	double xp;
	double yp;
	
	calXYZ();
	currentX = calX;
	currentY = calY;
	currentZ = calZ;

	interpolation(0, angle/trans); 

	
	for (int i = 0; i < 50; i++){

		xp = length_1 * cos(interpolValueArray[i]);
		yp = length_2 * sin(interpolValueArray[i]);

		calAngles( xp + currentX - length_1 , yp+ currentY , currentZ);

		writeAngle(theta_1, theta_2, theta_3,0);

		delay(timeSpend*1000/50);
	
	}

}



double uArmLibrary::readToAngle(double inputAnalog, int servoNumber , int trigger)
{
	int address = 60+(servoNumber-1)*6;

	for (int i = 0; i<6;i++){
      

    }

	double data_a = (EEPROM.read(address+1)+EEPROM.read(address+2)*256)/10.0;
	if (EEPROM.read(address)==0)
		{data_a = -data_a;}

	double data_b = (EEPROM.read(address+4)+EEPROM.read(address+5)*256)/100.0;
	if (EEPROM.read(address+3)==0)
		{data_b = -data_b;}


	if (trigger == 0){
		return (data_a + data_b*inputAnalog) - servoOffset(servoNumber);
	}
	if (trigger == 1){
		return (data_a + data_b*inputAnalog);
	}
}



double uArmLibrary::readAngle(int servoNumber)
{

	switch (servoNumber)
	{
		case servoRotNumber:
			
			return readToAngle(analogRead(servoRotReadPin),servoRotNumber,0);
			break;

		case servoLNumber:
			return readToAngle(analogRead(servoLReadPin),servoLNumber,0);
			break;

		case servoRNumber:
			return readToAngle(analogRead(servoRReadPin),servoRNumber,0);
			break;

		case servoHandRotNumber:
			return readToAngle(analogRead(servoHandRotReadPin),servoHandRotNumber,0);
			break;

		default:
			
			break;


	}

}

double uArmLibrary::readAngle(int servoNumber, int trigger)
{

	switch (servoNumber)
	{
		case servoRotNumber:
			return readToAngle(analogRead(servoRotReadPin),servoRotNumber,trigger);
			break;

		case servoLNumber:
			return readToAngle(analogRead(servoLReadPin),servoLNumber,trigger);
			break;

		case servoRNumber:
			return readToAngle(analogRead(servoRReadPin),servoRNumber,trigger);
			break;

		case servoHandRotNumber:
			return readToAngle(analogRead(servoHandRotReadPin),servoHandRotNumber,trigger);
			break;

		default:
			
			break;


	}

}


void uArmLibrary::setOffset()
{
	int addressOffset = 90;
	int setLoop = 1;

	detachAll();

	Serial.println("Put uarm in calibration posture (servo 1 to 3: 135, 50, 90 degree respectively), then input 1");
	while (setLoop){

		if (Serial.available()>0){
			
			char inputChar = Serial.read();

			if (inputChar=='1')
			{
				double offsetRot = readAngle(1,1) - 135;
				double offsetL = readAngle(2,1) - 50;
				double offsetR = readAngle(3,1) - 90;

        Serial.print("Offsets for servo 1 to 3 are ");
        Serial.println(offsetRot);
        Serial.println(offsetL);
        Serial.println(offsetR);


				if (abs(offsetRot)>25.4||abs(offsetL)>25.4||abs(offsetR)>25.4)
				{
					Serial.print("Check posture");
          

				}
				else{

					saveOffsetValue(offsetRot,1);
					saveOffsetValue(offsetL,2);
					saveOffsetValue(offsetR,3);
					setLoop = 0;
					Serial.println("Save offset done! ");
				}

				
			}
			else if(inputChar== 'e')
			{
				Serial.println("exit");
				setLoop = 0;
			}
		
			else{
				Serial.println("Incorrect, input again, or e to exit");
			}

		}

	}
}


void uArmLibrary::saveOffsetValue(double value, int servoNumber)
{
	int Byte0;
	int Byte1;

	if (value >0)
		Byte0 = 1;
	else
		Byte0 = 0;

	value = value *10;
	Byte1 = (int) value;

	EEPROM.write( addressOffset + (servoNumber-1)*2, abs(Byte0));
	EEPROM.write( addressOffset + (servoNumber-1)*2 +1, abs(Byte1));

}



double uArmLibrary::calYonly(double theta_1, double theta_2, double theta_3)

{

    l3_1_2 = l3 * cos(theta_2 / trans);
    l4_1_2 = l4*cos(theta_3 / trans);
    l5_2 = (l2 + l3*cos(theta_2 / trans) + l4*cos(theta_3 / trans));

    return -sin(abs(theta_1 / trans))*l5_2;

}

/* The code above is written by jerry song*/