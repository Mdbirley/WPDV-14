//////////////////////////////////////////////////////////////////////////////////////////////////
// Definitions, Libraries and Includes for WPDV
//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
//  ************** MOWER TYPE **************
//////////////////////////////////////////////////////////////////////////////////////////////////

//#define ROVER_ZEROTURN
#define ROVER_ELECTRIC_V2

//////////////////////////////////////////////////////////////////////////////////////////////////
// ELECTRIC V2 DEFAULTS
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef ROVER_ELECTRIC_V2

#define ROBOTEQ_MIXED	// How the Servo Inputs on the Roboteq are configured (ie not Tank Mode)

// Comment line below to use IMU as primary direction source
#define USE_DUAL  // Use Dual Antenna for position and direction

// Comment line below for navigation stacks without screen
#define USE_OLED // Screen on IMU/RADIO board

#define SerialRPI   Serial1    // Serial1 Pins  1(TX), 0(RX) SERCOM0
#define SerialDEBUG SerialUSB  //  Should make sure this opens
#define SerialIMU   Serial3    // Serial3 Pins 16(TX),17(RX) SERCOM1
#define SerialGPS   Serial2    // Serial2 Pins 18(TX),19(RX) SERCOM4
#define SerialMOTOR Serial4    // Serial4 Pins 14(TX),15(RX) SERCOM5
//#define SerialRADIO Serial5    // Serial5 Pins 51(TX),52(RX) SERCOM7
#define SerialSensorBoard Serial5    // Serial5 Pins 51(TX),52(RX) SERCOM7 (Grand Central Lidar/Sensor/Radio)

#define ADC_PIN_DRIVE_CURRENT A3
#define ADC_PIN_DRIVE_VOLTAGE A4
#define ADC_PIN_MOWER_CURRENT A5
#define ADC_PIN_MOWER_VOLTAGE A6

//#define CAMERA_DETECT_PIN 9

#endif // ROVER_ELECTRIC_V2

//////////////////////////////////////////////////////////////////////////////////////////////////
// ZERO TURN DEFAULTS
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef ROVER_ZEROTURN

#define USE_DUAL  // Use Dual Antenna for position and direction

#define SerialRPI   Serial1    // Serial1 Pins  1(TX), 0(RX) SERCOM0
#define SerialDEBUG SerialUSB  //  Should make sure this opens
#define SerialGPS   Serial2    // Serial2 Pins 18(TX),19(RX) SERCOM4
#define SerialRADIO Serial3    // Serial3 Pins 16(TX),17(RX) SERCOM1
#define SerialSONAR Serial4    // Serial4 Pins 14(TX),15(RX) SERCOM5
//#define SerialIMU Serial4    // Serial4 Pins 14(TX),15(RX) SERCOM5 (MKR 1300 Radio/IMU Combo)

//#define USE_ROLLING_ROAD // Rolling Road direct servo

#endif // ROVER_ZEROTURN

//////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef SerialRADIO
#define SerialSENSOR SerialRADIO // Radio on primary Navigation Board
#else
#define SerialSENSOR SerialSensorBoard // Sensor Configuration pipes to Radio
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
// GENERAL DEFAULTS
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef USE_DUAL
#define MAIN_LOOP_HZ   8  // Processing/Update Rate for main loop in Hertz (5 and 8 for GNSS, 20 for IMU)
#define USE_DUAL_CORRECTION 0.30 // Advanced Rear Antenna Location Forward by 30 cm
#else
#define MAIN_LOOP_HZ  50  // Processing/Update Rate for main loop in Hertz (5 and 8 for GNSS, 20 for IMU)
#endif // USE_DUAL

//------------------------------------------------------------------------------------------------

#define SerialRPI_BaudRate    230400
#define SerialGPS_BaudRate    115200
#define SerialIMU_BaudRate    230400 // IMU300
//#define SerialIMU_BaudRate    115200 // UM7
#define SerialSensorBoard_BaudRate   57600 // Grand Central Lidar/Sensor
#define SerialSONAR_BaudRate    9600 // MEGA Sonar   //TODO: Delete,Not used anymore
#define SerialMOTOR_BaudRate  115200 // Sabertooth or Roboteq MDC2230
#define SerialDEBUG_BaudRate  230400 // USB, So notional
#define SerialRADIO_BaudRate  115200 // LoRa Debug/Tracking Radio MKR 1300 11-June-2022
//#define SerialRADIO_BaudRate   38400 // LoRa Debug/Tracking Radio
//#define SerialRADIO_BaudRate   57600 // 3DR

//------------------------------------------------------------------------------------------------

//#define USE_TFMINI
//#define USE_INA260 // Locks Arduino if missing
#define USE_RC_TX_INPUTS
#define USE_SERVO_ACTUATORS
#define USE_OLED
#define USE_NEOPIXEL

//------------------------------------------------------------------------------------------------

#define LED_FIXQUALITY   53 //35 // From MAX 4-Aug-2020, will move to aDefinitions later, done
#define BUTTON_START     30
#define BUTTON_STOP      31
#define BUTTON_DOWNLOAD  32

//------------------------------------------------------------------------------------------------

#define DEBUG 1           // Debugging to print to serial for test purposes
#define DEBUGPRINT        // If you comment this line, the dPrint & dPrintLn lines are defined as blank.
//#define DEBUG_SPEED 0     // Debugging speed test purposes
#define DEBUG_IMU
#define DEBUG_RPI
//#define DEBUG_GPS
//#define DEBUG_STATE
//#define DEBUG_RC

//------------------------------------------------------------------------------------------------

#define REPORT_OBSTACLE

//////////////////////////////////////////////////////////////////////////////////////////////////
// Includes for assorted libraires
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>                                 // used by: GPS

#define ISNAN(X) (!((X)==(X)))

#include <vector>                                 // used for waypoint storage

#include <USBSabertooth.h>                        // Motor Controller driver library

//#include <waypointClass.h>                        // Custom class to manage GPS waypoints

#include <TinyGPS++.h>                            // GPS Library
#include <PID_v1.h>                               // PID Controller
#include <Servo.h>                                // used for zeroturn servos
//#include <Wire.h>
#include <TFMPI2C.h>
#include <Adafruit_INA260.h>                      // AdaFruit Current/Voltage Library
#include <Adafruit_NeoPixel.h>

#include "oled.h"

//////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef _VARIANT_GRAND_CENTRAL_M4_

//////////////////////////////////////////////////////////////////////////////////////////////////

#define SerialUSB SERIAL_PORT_USBVIRTUAL

extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;
extern SERCOM sercom6;
extern SERCOM sercom7;

extern Uart Serial1;
extern Uart Serial2;
extern Uart Serial3;
extern Uart Serial4;
extern Uart Serial5;

//////////////////////////////////////////////////////////////////////////////////////////////////

//Place Holder

//#define SerialIMU   Serial3    // Serial3 Pins 16(TX),17(RX) SERCOM1
//#define SerialSONAR Serial4    // Serial4 Pins 14(TX),15(RX) SERCOM5
//#define SerialRADIO Serial5    // Serial5 Pins 51(TX),52(RX) SERCOM7
//#define SerialRADIO Serial3    // Serial3 Pins 16(TX),17(RX) SERCOM1
//#define SerialSensorBoard Serial5    // Serial5 Pins 51(TX),52(RX) SERCOM7
//#define SerialRADIO_OUTONLY Serial2 // Serial2 via 18(TX)

// ARD  (0) RX <- RPI  (8) TX + GND (6)
// ARD  (1) TX -> RPI (10) RX
// ARD (19) RX <- GPS C94-M8P (J8.10) + GND (J8.8)

// C94-M8P
//
// |   J8  1   3   5   7   9   11  13  15  17  19
// |     +----------------------------------------+
// | /-\ | O   @   O   @   O   @   O   @   O   O  |
// | | | |                                        |
// | \_/ | @   O   O   @   O   @   O   @   O   O  |
// |     +----------------------------------------+
// |       2   4   6   8   10  12  14  16  18  20
// +==================================================
//
// J8.@  GND
// J8.10 TXD_GNSS (OUTBOUND)

// Servo Library to be using
// https://forum.arduino.cc/index.php?topic=639649.msg4329838#msg4329838

// Related threads 19 July 2021, 30 August 2021

//
// IMU 20-pin 1mm pitch Wiring
//
// Pin 2 GPS 1PPS Input
// Pin 5 IMU Tx, Output to Arduino, $IMU sentences
// Pin 10-12 (pick one) Power (3.3V ?  3-5 VDC)
// Pin 13-15 Ground
// Pin 19 GPS Receiver (GPS TX to IMU Rx)
//
// EVK Headers
// P2 (6-Pin)
//   CDBUS0 (1) (2) RX_IN  (IMU 19)
//   CDBUS1 (3) (4) TX_OUT (IMU 17)
//      GND (5) (6)
//
// EVK Wiring
// (2) to (3) Plumb GPS Input to USB Quad UART
// (3) to GPS Receiver, (5) Ground
//
// P4 (12-Pin)
//           GND (1)  (2) GND
// (IMU 6) SS_RX (3)  (4) DRDY (IMU 7)
//   (IMU 3) SCK (5)  (6) 1PPS (IMU 2)
//  (IMU 5) MOSI (7)  (8) NRST (IMU 8)
//  (IMU 4) MISO (9) (10) IO2 (IMU 9)
//           5V (11) (12) IO3 (IMU 1)
//
// EVK Wiring
// Use (1) GND, (6) 1PPS, MOSI (7) UART1 TX $IMU Output
//

//////////////////////////////////////////////////////////////////////////////////////////////////

void HardFault_Handler(void)
{
  digitalWrite(LED_BUILTIN, 1);
  digitalWrite(PIN_LED2, 0);
  digitalWrite(PIN_LED3, 0);
  while(1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

extern "C" char *sbrk(int i);

int FreeRam()
{
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

#else // _VARIANT_GRAND_CENTRAL_M4_

//////////////////////////////////////////////////////////////////////////////////////////////////

#error fail

#include <DueTimer.h>                             // interrupt timers

#define SerialRPI   SerialUSB
#define SerialDEBUG Serial
#define SerialGPS   Serial1
#define SerialIMU   Serial2
#define SerialMOTOR Serial3
//#define SerialSONAR Serial3

extern char _end;
extern "C" char *sbrk(int i);
char *ramstart = (char *)0x20070000;
char *ramend = (char *)0x20088000;

//////////////////////////////////////////////////////////////////////////////////////////////////

#endif // _VARIANT_GRAND_CENTRAL_M4_

//////////////////////////////////////////////////////////////////////////////////////////////////

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////

//Timer to reset Due during setup when the RPI communication will not automatically be established after poweron
unsigned long resetTimer;

struct latlon {
  double lat;
  double lon;
};

//////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef DEBUGPRINT    //Macros are usually in all capital letters.
#define dPrint(...) SerialDEBUG.print(__VA_ARGS__)     //dPrint is a macro, debug print
#define dPrintLn(...)  \
  SerialDEBUG.print(millis()); \
  SerialDEBUG.print(": ");  \
  SerialDEBUG.println(__VA_ARGS__)   //dPrintLn is a macro, debug print with new line
#else
#define dPrint(...)     //now defines a blank line
#define dPrintLn(...)   //now defines a blank line
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////

int imuMessageCount = 0;

//********Declare Variables to communicate with Raspberry PI****************

String serData = "";
boolean newData = false;

int countLoop = 0;

#define INVALID_GPS_SPEED -999  //used to check if GPS speed is valid

//////////////////////////////////////////////////////////////////////////////////////////////////

#define c_empty 0
#define c_download 1
#define c_start 2
#define c_stop 3
#define c_pause 4
#define c_downloadReady 5
#define c_advance 6
#define c_params  7
#define c_csv     8

//////////////////////////////////////////////////////////////////////////////////////////////////

#define S_INIT 1
#define S_WAITING 2
#define S_MOWING 3
#define S_PIVOT 4
#define S_HOLD 5
#define S_READY 6
#define S_MANUAL 7

//////////////////////////////////////////////////////////////////////////////////////////////////

int state = 0; // state for main routine
int prevState = 0;
int savedState; // saved state to return from hold state
unsigned long stateMillis;

int lastStateEnc = 0;

int cmd = 0;    // commands from raspberry PI
int manualMode = 0; // reporting manual mode

//////////////////////////////////////////////////////////////////////////////////////////////////

// Chassis choice, read from ini file
#define modelChassis  1
#define mainChassis   2
#define zeroturnChassis 3
#define ackermanChassis 4

int chassis = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////
//   Parameters downloaded from raspberry
//////////////////////////////////////////////////////////////////////////////////////////////////

vector <String> raspberryParameters;
// [CHASSIS] parameters
#define  dl_chassis 0
//[GENERAL] parameters
#define  dl_saveDat 1
#define  dl_printRadio1 2
#define  dl_objectAvoidance 3
#define  dl_fixQualityEnable 4
#define  dl_polygonEnable 5
//[MODEL],[MAIN],[ZEROTURN],[ACKERMAN] parameters
#define  dl_waypointTolerance 6
#define  dl_baudrateMotor 7
#define  dl_normalForwardSpeed 8
#define  dl_rampupTimeForwardSpeed 9
#define  dl_pivotHeadingTolerance 10
#define  dl_pivotMode 11
#define  dl_pivotForwardSpeed 12
#define  dl_pivotTurnSpeedBegin 13
#define  dl_pivotTurnSpeedMiddle 14
#define  dl_pivotTurnSpeedEnd 15
#define  dl_minHeadingError 16
#define  dl_maxHeadingError 17
#define  dl_startTurnspeedMinHEadingError 18
#define  dl_endTurnspeedMaxHEadingError 19
#define  dl_minObjectDistance 20
#define  dl_minTurnDistance 21
#define  dl_turnSpeedDuringAvoidance 22
#define  dl_PIDEnable 23
#define  dl_PIDDirection 24
#define  dl_Kp 25
#define  dl_Ki 26
#define  dl_Kd 27
//variables for zeroturn and ackerman servos
#define  dl_steeringSensitivity 28
#define  dl_servoLeftValueLeft 29
#define  dl_servoLeftValueMiddle 30
#define  dl_servoLeftValueRight 31
#define  dl_servoRightValueLeft 32
#define  dl_servoRightValueMiddle 33
#define  dl_servoRightValueRight 34
#define  dl_ztDirection 35
#define  dl_purePursuitD5 36
#define  dl_purePursuitPIDEnable 37
#define  dl_purePursuitKp 38
#define  dl_purePursuitKi 39
#define  dl_purePursuitKd 40
#define  dl_ppPIDDirection 41
#define  dl_movingAwayPivot 42
#define  dl_movingAwayTolerence 43
//speed control parameters
#define  dl_speedSetpoint 44
#define  dl_speedPIDEnable 45
#define  dl_speedKp 46
#define  dl_speedKi 47
#define  dl_speedKd 48
//Stanley controller parameters
#define  dl_stanleyEnable 49
#define  dl_stanleyGain 50
#define  dl_stanleyKp 51
#define  dl_stanleyKi 52
#define  dl_stanleyKd 53
#define  dl_pipeCrossTrackThreshold 54
#define  dl_pipeTurnSpeedLimit 55

// !!!! Must make sure the index numbers above match ParamList[] tables in decode.py of RPi side code !!!!

//DO WE NEED ZeroTurn AND Ackerman defines here?????????????????????
vector <int> compassCorrection;

// ****************************Declare GLOBAL VARIABLES*********************

// reporting options, read from ini file
int saveDat = 0;
int printRadio1 = 0;

// Variables for Sonars
// Analog Pins on the Arduino Due
const int leftEchoPin = 1;
const int centerEchoPin = 2;
const int rightEchoPin = 3;
// Digital Pin on the Due
const int triggerPin = 3;

int distanceLeftEcho; //cm
int distanceCenterEcho; //cm
int distanceRightEcho; //cm

int objectAvoidance = 0;    //0=no avoidance; 1=avoidence; 0= default; will be overwritten if available in config.txt file
int minObjectDistance = 20; //cm; 20 = default; will be overwritten if available in config.txt file
int minTurnDistance = 90;   //cm; 90 = default; will be overwritten if available in config.txt file
int turnSpeedDuringAvoidance = 60; //600 = default; will be overwritten if available in config.txt file

// Heading variables
double targetHeading;                          // where we want to go to reach current waypoint

double GPSHeading;
double GPSHeadingDual;

float GPSTemperature; // F 11-Jun-2022

int GPSHeadingQuality;
int GPSFixQuality;

int GPSSatelliteCount;

double IMUvsGPSHeadingError;

//REMOVE int DeltaHTG; // difference between target heading and gps heading
//ONLY mentioned in reporting.ino
int DeltaHTC; // difference between target heading and compass heading

double IMUPitch; // variables for extracting pitch and roll from the IMU message
double IMURoll;
double IMUHeading;

//TODO Need to migrate these into the configuration, doing manually for now to test
double IMUPitchTrim = 3.1; // Nominals at horizonal, we subtract them out
double IMURollTrim = -1.095;
double IMUHeadingTrim = 0.0;

int    IMUcorrectionIndex; // index to correction table, needs review

double IMUerrorCorrection; // From compass table

double headingEstimate;                         // UM7 heading after compass correction
double headingError;

double minHeadingError;
double maxHeadingError;
double startTurnspeedMinHEadingError;
double endTurnspeedMaxHEadingError;

// variables to calculate cross track distance
double R = 6371e3; // metres earth
double crossTrackDistance; //distance in cm from straight line between previous and next watpoint

//REMOVE-UNUSED int crossTrackDistPID = 0;

int crossTrackPIDEnable = 0; // *** CROSS TRACK PID **

// Variables for speed and turn
double forwardSpeed;
double turnSpeed;                     // value sent to motor to turn
int forwardSpeedRight = 0;
int forwardSpeedLeft = 0;
int normalForwardSpeed = 0;        // value sent to the MC for forward speed, 0= default; will be overwritten if available in config.txt file
int rampupTimeForwardSpeed = 0;    // rampuptime in seconds from zero to normalForwardSpeed, 0= default; will be overwritten if available in config.txt file

// Variables for PIVOT
int pivotMode = 0;               // 0 = pivot to right. other values explained in config.txt file, 0= default; will be overwritten if available in config.txt file
int pivotForwardSpeed = 0;       // value sent to the MC for forward speed during pivot; 0= default; will be overwritten if available in config.txt file
int pivotTurnSpeedBegin = 0;     // value sent to motor to turn during start pivot; 0= default; will be overwritten if available in config.txt file
int pivotTurnSpeedMiddle = 0;    // value sent to motor to turn during middle of pivot; 0= default; will be overwritten if available in config.txt file
int pivotTurnSpeedEnd = 0;       // value sent to motor to turn during end pivot; 0= default; will be overwritten if available in config.txt file
int pivotHeadingTolerance = 0;   // tolerance +/- (in degrees) to end pivot routine; 0= default; will be overwritten if available in config.txt file
int pivoting = 0;          // 0 = normal operation, 1 = pivot operation

//////////////////////////////////////////////////////////////////////////////////////////////////

int    movingAwayPivot = 0;						// moving away from waypoint pivot 0=disable 1=enable
double movingAwayTolerence = 0.0; 		// moving away from waypoint tolerence
int    movingAwayPivotTriggered = 0;	// used for reporting

double minDistanceToTarget = 100000; // minimum distance to target (current waypoint) set to large value

//////////////////////////////////////////////////////////////////////////////////////////////////

// Variables for GPS
double currentLat;  //front GPS1
double currentLong; //front GPS1
double currentAlt;

double forwardSpeedGPS = 0; // metres per second

double prevLat;
double prevLong;
double prevAlt;

double targetLat;
double targetLong;
double targetAlt;

double targetCourse; // Computed at waypoint
double targetDistance;

double distanceToTarget;                        // current distance to target (current waypoint)
double originalDistanceToTarget;                // distance to original waypoint when we started navigating to it
double distanceFromLastTarget;

int fixQualityEnable = 0;                      // 0=GPS quality check disabled; 1=GPS quality check enabled; 0= default; will be overwritten if available in config.txt file

//////////////////////////////////////////////////////////////////////////////////////////////////

int RoverRadioRSSI = 0;
int RoverRadioSNR  = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TinyGPSPlus GPS1;                              // The TinyGPS++ object for front GPS

#ifdef USE_GPS_LIB
TinyGPSCustom fixQual1(GPS1, "GPGGA", 6);      // $GPGGA sentence, 5 or 6th element
#endif // USE_GPS_LIB

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables for Com speeds
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//DEPRECATED static const uint32_t GPSBaud = 115200;
static uint32_t baudrateMotor; // read from config

//////////////////////////////////////////////////////////////////////////////////////////////////
// Variables for Geofence
//////////////////////////////////////////////////////////////////////////////////////////////////

int polygonEnable = 0 ; // 0=polygon disabled; 1=polygon enabled; 0= default; will be overwritten if available in config.txt file

//////////////////////////////////////////////////////////////////////////////////////////////////

// Variables for zeroturn and ackerman chassis servos
double steeringSensitivity = 1;     // range 0-2; 0 = no steering; 2= maximum steering; will be overwritten if available in config.txt file

int servoLeftValueLeft = 0;         // output to servo in most left position ; will be overwritten if available in config.txt file
int servoLeftValueMiddle = 90;      // output to servo in middle position; will be overwritten if available in config.txt file
int servoLeftValueRight = 180;      // output to servo in most right position; will be overwritten if available in config.txt file
int servoRightValueLeft = 0;        // output to servo in most left position; will be overwritten if available in config.txt file
int servoRightValueMiddle = 90;     // output to servo in middle position; will be overwritten if available in config.txt file
int servoRightValueRight = 180;     // output to servo in most right position; will be overwritten if available in config.txt file

int ztDirection = 0;                // zeroTurn Servo Directions 0=direct, 1=reverse; will be overwritten if available in config.txt file

//int leftServoOutput;                // output to left Servo -100 to 100 %
//int rightServoOutput;               // output to right Servo -100 to 100 %
//int leftServoOutputMapped;          // output mapped to 0-180 degrees
//int rightServoOutputMapped;         // output mapped to 0-180 degrees

//////////////////////////////////////////////////////////////////////////////////////////////////

// Variables for RC receiver
int readRadioManualAuto;      // variable used to change between manual and Auto
int readAileron;              // variable for the value from the receiver Aileron for left right steering
int readThrottle;             // Variable for the value from the receiver Throttle for fowrward backward
int readAileronMapped;        // variable for the mapped value from the receiver Aileron
int readThrottleMapped;       // Variable for the mapped value from the receiver Throttle

//////////////////////////////////////////////////////////////////////////////////////////////////

// Variables for PID Control BY ISLAM
double PIDSetpoint, PIDInput, PIDOutput;
int    PIDEnable = 0; //0=PID Control disabled; 1= PID Control enabled; 0= default; will be overwritten if available in config.txt file
int    PIDDirection = 0; //0=Direct Control; 1= Reversed Control; 0= default; will be overwritten if available in config.txt file
float Kp = 2.0; //default setting will be overwritten if available in config.txt file
float Ki = 0.5; //default setting will be overwritten if available in config.txt file
float Kd = 1.0; //default setting will be overwritten if available in config.txt file
// Should rename this
PID myPID(&PIDInput, &PIDOutput, &PIDSetpoint, Kp, Ki, Kd, DIRECT); //Create PID object (PIVOT)

// Placeholder
//double turnPIDSetpoint, turnPIDInput, turnPIDOutput;
//float turnKp = 2.0;
//float turnKi = 0.5;
//float turnKd = 1.0;
//PID turnPID(&turnPIDInput, &turnPIDOutput, &turnPIDSetpoint, turnKp, turnKi, turnKd, DIRECT); //Create PID object (TURNSPEED)

//REMOVE float prevHeadingError = 0; // keep precision (deprecated)
float p_headingError = 0;
float i_headingError = 0;
float d_headingError = 0;

double pipeCrossTrackThreshold = 10.0; // cm
double pipeTurnSpeedLimit = 1.0;

//////////////////////////////////////////////////////////////////////////////////////////////////
// Create Sabertooth Motor Controller object
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef SerialMOTOR
USBSabertoothSerial C(SerialMOTOR);
USBSabertooth motor(C, 128);              // Use address 128.
#endif // SerialMOTOR

//////////////////////////////////////////////////////////////////////////////////////////////////

// Variables for PID speed control

double speedSetpoint = 0.0; // NO, NOT HOW THIS WORKS!! default setting will be overwritten if available in config.txt file
double speedPIDOutput;
int    speedPIDEnable = 0; //0=speed PID Control disabled; 1= speed PID Control enabled; 0= default; will be overwritten if available in config.txt file
double speedKp = 1.0; //default setting will be overwritten if available in config.txt file
double speedKi = 0.5; //default setting will be overwritten if available in config.txt file
double speedKd = 0.0; //default setting will be overwritten if available in config.txt file
//Create speed PID object
PID speedPID(&forwardSpeedGPS, &speedPIDOutput, &speedSetpoint, speedKp, speedKi, speedKd, DIRECT);
int speedPIDServo[2];

//////////////////////////////////////////////////////////////////////////////////////////////////

class waypointClass
{
  public:
    waypointClass(double pLong = 0, double pLat = 0)
    {
      fLong = pLong;
      fLat = pLat;
    }

    double getLat(void) {
      return fLat;
    }
    double getLong(void) {
      return fLong;
    }

  private:
    double fLong, fLat;
}; // waypointClass

vector <waypointClass> waypointList;
//[NUMBER_WAYPOINTS] = {waypointClass(-2.90981484, 52.50201694)};

//////////////////////////////////////////////////////////////////////////////////////////////////
// Variables for way points
//////////////////////////////////////////////////////////////////////////////////////////////////

int numberWaypoints = 0;
int waypointNumber = 0;       // Current waypoint number; will run from 1 to (numberWaypoints);
int prevWaypointNumber = 0;
float waypointTolerance = 0;  // Proximity to waypoint before turning to the next WP

//////////////////////////////////////////////////////////////////////////////////////////////////

double myDistanceBetween(double lat1, double long1, double lat2, double long2);
double myCourseTo(double lat1, double long1, double lat2, double long2);

//////////////////////////////////////////////////////////////////////////////////////////////////

class obstacleClass
{
  public:
    obstacleClass(double pLong = 0, double pLat = 0, double pWidth = 0)
    {
      fLong = pLong;
      fLat = pLat;
      fWidth = pWidth;
    }

    double getLat(void) {
      return fLat;
    }

    double getLong(void) {
      return fLong;
    }

    double getWidth(void) {
      return fWidth;
    }

    double getDistance(double pLong, double pLat) {
    	double a = pLong - fLong; // Not in Metres
    	double b = pLat  - fLat;
      return(sqrt(a*a + b*b));
    }

    double getDistanceCheap(double pLong, double pLat) {
    	double a = pLong - fLong;
    	double b = pLat  - fLat;
      return(a*a + b*b); // Magnitude
    }

    double getDistanceAccurate(double pLong, double pLat) {
    	double delta = myDistanceBetween(pLat, pLong, fLat, fLong);
    	if (delta > (fWidth / 2))
    		delta -= (fWidth / 2);
    	else
    		delta = 0.0; // Within the Tree
      return(delta);
    }

    double getBearingAccurate(double pLong, double pLat) {
      return(myCourseTo(pLat, pLong, fLat, fLong));
    }
  private:
    double fLong, fLat, fWidth;
}; // obstacleClass

vector <obstacleClass> obstacleList;

//////////////////////////////////////////////////////////////////////////////////////////////////
// Variables for obstacles
//////////////////////////////////////////////////////////////////////////////////////////////////

int obstacleIdentified = -1;
double obstacleDistance = 0.0;
double obstacleBearing = 0.0;
double obstacleAngle = 0.0;

//////////////////////////////////////////////////////////////////////////////////////////////////

unsigned long statusupdateTimerDownload = 0;    // status update during download timer
const long intervalStatusupdateDownload = 1000; // every second

// Variables to define the execution speed for the main program part:
//MOVED unsigned long mainLoopTimer = 0;        // Main loop timer
//MOVED unsigned long currentMillis;
//MOVED unsigned long previousMillisMain = 0;   // used for main loop

unsigned long previousMillisReporting = 0;    // used for reporting

//DEPRECATED int8_t reportCounter = 0;

//const long intervalMain = 125;                // currently 125 msec in sync with the GPS data (8 Hz)
//const long intervalMain = 200;                // currently 200 msec in sync with the GPS data (5 Hz)
const unsigned long intervalMain = (1000 / MAIN_LOOP_HZ);

const unsigned long intervalReporting = 1000; // (1000) 1hz (250) 4 Hz (200) 5 Hz

unsigned long loopTime = 0;
unsigned long slackTime = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////
// Variables for Stanley
//////////////////////////////////////////////////////////////////////////////////////////////////

int  stanleyEnable = 0;
float stanleyGain = 10;

float stanleyKp = 0.5; // could perhaps be doubles, but don't need the resolution
float stanleyKi = 0;
float stanleyKd = 0;

float errorStanley = 0; // working variables
float stanleySecondTerm = 0;
float prevErrorStanley = 0; // (deprecated) moved to p_errorStanley

float p_errorStanley = 0;
float i_errorStanley = 0;
float d_errorStanley = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////
// Variables for pure pursuit
//////////////////////////////////////////////////////////////////////////////////////////////////

double purePursuitD5 = 0;
int purePursuitPIDEnable = 1;
int ppPIDDirection = 1;
double courseToAimPoint = 0;
double ppPIDCorrection = 0;
double aimPointLat = 0;
double aimPointLong = 0;

double ppPIDSetpoint, ppPIDInput, ppPIDOutput;

float purePursuitKp = 2.0;
float purePursuitKi = 0.5;
float purePursuitKd = 0.0;

//Create PID object
//PID ppPID(&ppPIDInput, &ppPIDOutput, &ppPIDSetpoint, purePursuitKp, purePursuitKi, purePursuitKd, DIRECT);

//////////////////////////////////////////////////////////////////////////////////////////////////
// ROBOTEQ stuff in moverover
//////////////////////////////////////////////////////////////////////////////////////////////////
// Variables for reading encoders
//////////////////////////////////////////////////////////////////////////////////////////////////

long encoderLeft;
long encoderRight;
long encoder1Start;
long encoder2Start;

double rpmLeft  = 0.0; // Computed by ztEncoder.ino
double rpmRight = 0.0;

#ifdef DEBUG_SPEED
float speedTestGain = 0.1;
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
// Variables for Voltage, Current, Power monitoring
//////////////////////////////////////////////////////////////////////////////////////////////////

int driveCountLongTerm;
double driveCurrentLongTerm;
float driveCurrentLongTermAverage;  // computed/reported reporting.ino, for radio.ino

int driveCount; // Counts for Average

float driveCurrent; // Peak
float driveVoltage;
float driveVoltageAverage;  // computed/reported reporting.ino, for radio.ino
float drivePower;
float drivePowerAverage;  // computed/reported reporting.ino, for radio.ino

int mowerCountLongTerm;
double mowerCurrentLongTerm;
float mowerCurrentLongTermAverage; // computed/reported reporting.ino, for radio.ino

int mowerCount;

float mowerCurrent; // Peak
float mowerVoltage;
float mowerVoltageAverage;  // computed/reported reporting.ino, for radio.ino
float mowerPower;
float mowerPowerAverage; // computed/reported reporting.ino, for radio.ino

//////////////////////////////////////////////////////////////////////////////////////////////////
