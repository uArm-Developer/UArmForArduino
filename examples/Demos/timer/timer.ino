
#include "uArm.h"

#define USE_SERIAL_CMD	1	// 1: use serial for control	0: just use arduino to control(release ROM and RAM space)

unsigned long tickStartTime = millis(); // get timestamp;

unsigned int timeOutCount = 0;
unsigned int second = 0;

void setup()
{
	Serial.begin(115200);
	Init(); // Don't remove

	debugPrint("debug start"); // uncomment DEBUG in uArmConfig.h to use debug function
	
	// TODO
	moveTo(0, 150, 150);	// initial pos

}

void loop()
{
	run(); // Don't remove

	// TODO

}

// time out every TICK_INTERVAL(50 ms default)
void tickTimeOut()
{
	timeOutCount++;

	// 1s time out
	if (timeOutCount > (1000 / TICK_INTERVAL))
	{
		second++;
		timeOutCount = 0;
		char buf[60];
		msprintf(buf, "%d seconds elapsed\r\n", second);
		Serial.print(buf);
	}
}

////////////////////////////////////////////////////////////
// DO NOT EDIT
void Init()
{
	uArmInit();	// Don't remove
	service.init();

	#if USE_SERIAL_CMD == 1
	serialCmdInit();
	
	#endif
}

void run()
{
	#if USE_SERIAL_CMD == 1
	handleSerialCmd();
	#endif

	manage_inactivity(); // Don't remove
}

void tickTaskRun()
{
	tickTimeOut();

    buttonPlay.tick();
    buttonMenu.tick();
#ifdef MKII
    ledRed.tick();
    service.btDetect();
#endif    
}

void manage_inactivity(void)
{
#if USE_SERIAL_CMD == 1
	getSerialCmd();	// for serial communication
#endif
	service.run();	// for led, button, bt etc.

	// because there is no other hardware timer available in UNO, so use a soft timer
	// it's necessary for button,led, bt
	// so Don't remove it if you need them
	if(millis() - tickStartTime >= TICK_INTERVAL)
	{
		tickStartTime = millis();
		tickTaskRun();
	}   
}
