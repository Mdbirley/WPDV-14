//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TFmini.ino
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <TFMPI2C.h>  // TFMini-Plus I2C Library v1.5.0
TFMPI2C tfmP1;        // Create a TFMini-Plus I2C object

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef GET_FIRMWARE_VERSION // For Max's version of the TFMini V1.5.0 library
#define GET_FIRMWARE_VERSION OBTAIN_FIRMWARE_VERSION
#define SOFT_RESET SYSTEM_RESET
#endif

static void tfmConfig(uint8_t addr)
{
    // Setup the device at I2C address 0x10
    printf("Set up device at address 0x%2x\r\n", addr);

    // - - Perform a system reset - - - - - - - - - - -
    printf( "System reset: ");
    if (tfmP1.sendCommand(SOFT_RESET, 0, addr)) // SYSTEM_RESET
      printf( "passed.\r\n");
    else tfmP1.printReply();

    delay(500);  //  Wait for device to complete Reset

    // - - Display the firmware version - - - - - - - - -
    printf( "Firmware version: ");
    if (tfmP1.sendCommand(GET_FIRMWARE_VERSION, 0, addr)) // OBTAIN_FIRMWARE_VERSION
    {
        printf( "%1u.", tfmP1.version[0]); // print three single numbers
        printf( "%1u.", tfmP1.version[1]); // each separated by a dot
        printf( "%1u\r\n", tfmP1.version[2]);
    }
    else
      tfmP1.printReply();

    // - - Set the data frame-rate to 100 - - - - - - - - -
    //  Frame rate directly proportional to noisiness of data
    printf( "Data-Frame rate: ");
    if (tfmP1.sendCommand(SET_FRAME_RATE, FRAME_100, addr))
        printf( "%2uHz.\r\n", FRAME_100);
    else
      tfmP1.printReply();

    // - - - - - - - - - - - - - - - - - - - - - - - -
    delay(500);                    // wait for half a second.
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tfmSetup(void)
{
    // Recover I2C Bus
    // Send numbers for Due SDA/SCL pins, includes Wire.begin()
    tfmP1.recoverI2CBus(20, 21);

    // Send some example commands to the TFMini-Plus
    tfmConfig(0x10);
    tfmConfig(0x14);
    tfmConfig(0x18);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tfmRead(void)
{
  int16_t tfDist[3] = {0};   // Distance to object in centimeters
  static uint32_t count = 0, errors = 0;

    //first sensor at 0x10*********************************************************
    tfmP1.getData(tfDist[0], 0x10);  // Get a frame of data
    if (tfmP1.status != TFMP_READY)    // If error...
    {
      tfmP1.printFrame(); // Display error and data frame
      errors++;
    }

#if 1 // Try just doing first
    //second sensor at 0x14*****************************************************
    tfmP1.getData(tfDist[1], 0x14);  // Get a frame of data
    if (tfmP1.status != TFMP_READY)    // If error...
    {
      tfmP1.printFrame(); // Display error and data frame
      errors++;
    }

    //Third sensor at 0x18*****************************************************
    tfmP1.getData( tfDist[2], 0x18);  // Get a frame of data
    if (tfmP1.status != TFMP_READY)    // If error...
    {
      tfmP1.printFrame(); // Display error and data frame
      errors++;
    }
#endif

#if 1 //def SerialRADIO // Debug Output
	if (SerialDEBUG)
  {
    char distStr[64];
    sprintf(distStr, "%03i,%03i,%03i,%u,%u\r\n", tfDist[0], tfDist[1], tfDist[2], count++, errors);
    SerialDEBUG.write(distStr);
  }
#endif // SerialRADIO

  distanceLeftEcho   = tfDist[0]; // Left/Right sense here may be wrong
  distanceCenterEcho = tfDist[1];
  distanceRightEcho  = tfDist[2];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
