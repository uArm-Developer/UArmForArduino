/******************************************************************************
* File Name          : uArm_firmata.cpp
* Author             : Joey Song  
* Updated            : Joey Song
* Email              : joey@ufactory.cc
* Version            : V1.0 
* Date               : 17 Mar, 2016
* Modified Date      : 17 Mar, 2016
* Description        : 
* License            : 
* Copyright(C) 2016 UFactory Team. All right reserved.
*******************************************************************************/

#include <ConfigurableFirmata.h>
#include <FirmataExt.h>
#include <FirmataReporting.h>

#include "uarm_firmata.h"
#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <uarm_library.h>
#include <EEPROM.h>


FirmataExt firmataExt;
FirmataReporting reporting;
UArmFirmata uarmCom;

/*==============================================================================
 * FUNCTIONS
 *============================================================================*/
void systemResetCallback()
{
  firmataExt.reset();
}

/*==============================================================================
 * SETUP()
 *============================================================================*/
void setup()
{
  // TODO - pass the version of this firmware rather than the Firmata protocol
  // version. Making that change now however may break compatibility.
  Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);
  firmataExt.addFeature(reporting);
  firmataExt.addFeature(uarmCom);
  /* systemResetCallback is declared here (in ConfigurableFirmata.ino) */
  Firmata.attach(SYSTEM_RESET, systemResetCallback);
  Firmata.begin(57600);
  systemResetCallback();  // reset to default config
}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop()
{
  /* STREAMREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
  while (Firmata.available()) {
    Firmata.processInput();
  }


}
