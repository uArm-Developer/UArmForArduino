// Demo button
#include "uArm.h"

#define USE_SERIAL_CMD	0	// 1: use serial for control	0: just use arduino to control(release ROM and RAM space)

unsigned long tickStartTime = millis(); // get timestamp;

void setup()
{
	Serial.begin(115200);
	Init(); // Don't remove

	debugPrint("debug start"); // uncomment DEBUG in uArmConfig.h to use debug function
	
	// TODO
	service.setButtonService(false);	// disable build in button service
	moveTo(0, 150, 150);	// initial pos
}

void loop()
{
	run(); // Don't remove

	// TODO
	if (buttonMenu.longPressed())
	{
		buttonMenu.clearEvent();
		Serial.println("menu button long pressed event");
	}
	else if (buttonMenu.clicked())
	{
		buttonMenu.clearEvent();	// manually clear event

		Serial.println("menu button click event");
	}

	if (buttonPlay.longPressed())
	{
		buttonPlay.clearEvent();
		Serial.println("play button long pressed event");
	}
	else if (buttonPlay.clicked())
	{
		buttonPlay.clearEvent();	// manually clear event

		Serial.println("play button click event");
	}
}

// time out every TICK_INTERVAL(50 ms default)
void tickTimeOut()
{
	
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
