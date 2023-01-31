// Functions in this file...
//  [1]reporting()        Report all mission log variables to RPi every  200 milli seconds
//  [2]reportRPi()         Puts variables in correct order to be sent to RPi report
//  [3]sendStateRasp()      Send current rover state to RPi
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int radioReportingCycles = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void reporting(unsigned long currentMillis)
{
  if ((currentMillis - previousMillisReporting) >= intervalReporting)
  {
    reportRPi(currentMillis); // update and save required data

    if ((currentMillis - previousMillisReporting) >= 4000) // If we've been out of loop for too long, just catch up
       previousMillisReporting = currentMillis - (currentMillis % 1000);

    previousMillisReporting += intervalReporting; // Advance Timeline
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint32_t csvReportingTime;
static int csvPolygon;
static int csvWaypoint;
static double csvLeft;
static double csvRight;
static double csvDistanceToTarget;
static double csvObstacleDistance;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
  Work toward an enumerated list 29-Jun-2022

headings=['DateTime','Loop Timer (millis)','Manual','Pivot','RTKFixQual','HeadingQual','Current GPS Readings|Current Long','Current Lat','GPS Heading','IMUHeading', \
          'Current IMU Readings|IMURoll','IMUPitch','crossTrackDist','TargetHeadingWp','Calculated Errors|AimPointHeading','Heading Error GPS','TargetHeading Error IMU', \
          'Distance to target','speedGPS','ForwardSpeed','TurnSpeed','ForwardSpeedLeft','ForwardSpeedRight','EncoderLeftRPM','EncoderRightRPM','WP Number','WP Long','WP Lat', \
          'Prev WP Long','Prev WP Lat','AimPoint Long','AimPoint Lat','Polygon','Moving Away Pivot','LeftSonar','CenterSonar','RightSonar','LoopTime','SlackTime', \
          'Amps Drive','Volts Drive','Watts Drive','Amps MowDeck','Volts MowDeck','Watts MowDeck','Altitude', \
          'Obstacle', 'Obs Distance', 'Obs Bearing', 'Obs Angle' ]
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define CSV_TYPE_NULL       0

#define CSV_TYPE_INT        100
#define CSV_TYPE_UINT       200

#define CSV_TYPE_FLOAT      300
#define CSV_TYPE_FLOAT_1DP  301
#define CSV_TYPE_FLOAT_2DP  302
#define CSV_TYPE_FLOAT_3DP  303
#define CSV_TYPE_FLOAT_INT  310

#define CSV_TYPE_DOUBLE     400
#define CSV_TYPE_DOUBLE_1DP 401
#define CSV_TYPE_DOUBLE_2DP 402
#define CSV_TYPE_DOUBLE_3DP 403
#define CSV_TYPE_DOUBLE_4DP 404
#define CSV_TYPE_DOUBLE_8DP 408
#define CSV_TYPE_DOUBLE_INT 410

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _CSV_HEADINGS
{
  const char *Name;
  void *Pointer;
  int Type;
} CSV_HEADINGS;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CSV_HEADINGS csv_headings[] =
{
  //{ "DateTime", &var, CSV_TYPE_NULL },

  { "WP Number", &csvWaypoint, CSV_TYPE_INT },

  { "Manual", &manualMode, CSV_TYPE_INT },
  { "Pivot",  &pivoting,   CSV_TYPE_INT },

  { "RTKFixQual",  &GPSFixQuality,     CSV_TYPE_INT },
  { "HeadingQual", &GPSHeadingQuality, CSV_TYPE_INT },

  { "crossTrackDist", &crossTrackDistance, CSV_TYPE_DOUBLE_2DP },
  { "Distance to target", &csvDistanceToTarget, CSV_TYPE_DOUBLE_2DP },

  { "GPS Heading", &GPSHeading, CSV_TYPE_DOUBLE_2DP },
  { "IMUHeading", &IMUHeading, CSV_TYPE_DOUBLE_2DP },

  { "TargetHeadingWp", &targetHeading, CSV_TYPE_DOUBLE_2DP },
  { "AimPointHeading", &courseToAimPoint, CSV_TYPE_DOUBLE_2DP },

  { "TargetHeading Error IMU", &DeltaHTC, CSV_TYPE_DOUBLE_2DP },
  { "Heading Error GPS", &headingError, CSV_TYPE_DOUBLE_2DP },

  { "speedGPS", &forwardSpeedGPS, CSV_TYPE_DOUBLE_4DP }, // Metres/second

  { "ForwardSpeed", &forwardSpeed, CSV_TYPE_DOUBLE_2DP }, // Applied forward speed +/- 100
  { "TurnSpeed",    &turnSpeed,    CSV_TYPE_DOUBLE_2DP }, // Applied turn speed +/- 100

  { "ForwardSpeedLeft",  &forwardSpeedLeft,  CSV_TYPE_INT },
  { "ForwardSpeedRight", &forwardSpeedRight, CSV_TYPE_INT },

  { "EncoderLeftRPM",  &rpmLeft,  CSV_TYPE_DOUBLE_3DP },
  { "EncoderRightRPM", &rpmRight, CSV_TYPE_DOUBLE_3DP },

  { "Current Long", &currentLong, CSV_TYPE_DOUBLE_8DP },
  { "Current Lat", &currentLat, CSV_TYPE_DOUBLE_8DP },

  { "WP Long", &targetLong, CSV_TYPE_DOUBLE_8DP },
  { "WP Lat", &targetLat, CSV_TYPE_DOUBLE_8DP },

  { "Prev WP Long", &prevLong, CSV_TYPE_DOUBLE_8DP },
  { "Prev WP Lat", &prevLat, CSV_TYPE_DOUBLE_8DP },

  { "AimPoint Long", &aimPointLong, CSV_TYPE_DOUBLE_8DP },
  { "AimPoint Lat", &aimPointLat, CSV_TYPE_DOUBLE_8DP },

  { "Polygon", &csvPolygon, CSV_TYPE_INT },
  { "Moving Away Pivot", &movingAwayPivotTriggered, CSV_TYPE_INT },
  { "Min to Way", &minDistanceToTarget, CSV_TYPE_DOUBLE_2DP },

  { "LeftSonar",   &distanceLeftEcho,   CSV_TYPE_INT }, // Sonar/Lidar
  { "CenterSonar", &distanceCenterEcho, CSV_TYPE_INT },
  { "RightSonar",  &distanceRightEcho,  CSV_TYPE_INT },

  { "IMURoll", &IMURoll, CSV_TYPE_DOUBLE_2DP },
  { "IMUPitch", &IMUPitch, CSV_TYPE_DOUBLE_2DP },

#ifdef USE_INA260
  { "Amps Drive",    &driveCurrent, CSV_TYPE_FLOAT_3DP },
  { "Volts Drive",   &driveVoltage, CSV_TYPE_FLOAT_3DP },
  { "Watts Drive",   &drivePower,   CSV_TYPE_FLOAT_3DP },

  { "Amps MowDeck",  &mowerCurrent, CSV_TYPE_FLOAT_3DP },
  { "Volts MowDeck", &mowerVoltage, CSV_TYPE_FLOAT_3DP },
  { "Watts MowDeck", &mowerPower,   CSV_TYPE_FLOAT_3DP },
#else
  { "Amps Drive",    &driveCurrent,        CSV_TYPE_FLOAT_3DP },
  { "Volts Drive",   &driveVoltageAverage, CSV_TYPE_FLOAT_3DP },
  { "Watts Drive",   &drivePowerAverage,   CSV_TYPE_FLOAT_3DP },

  { "Amps MowDeck",  &mowerCurrent,        CSV_TYPE_FLOAT_3DP },
  { "Volts MowDeck", &mowerVoltageAverage, CSV_TYPE_FLOAT_3DP },
  { "Watts MowDeck", &mowerPowerAverage,   CSV_TYPE_FLOAT_3DP },
#endif // USE_INA260

  { "Altitude", &currentAlt, CSV_TYPE_DOUBLE_3DP },

  { "Obstacle",     &obstacleIdentified, CSV_TYPE_INT },          // Tree#
  { "Obs Distance", &csvObstacleDistance, CSV_TYPE_DOUBLE_2DP },  // Distance metres
  { "Obs Bearing",  &obstacleBearing, CSV_TYPE_DOUBLE_2DP },      // Direction from antenna to tree
  { "Obs Angle",    &obstacleAngle,   CSV_TYPE_DOUBLE_2DP },      // Direction from rover, corrected

  { "SpeedPID0",    &speedPIDServo[0],  CSV_TYPE_INT },
  { "SpeedPID1",    &speedPIDServo[1],  CSV_TYPE_INT },

  { "GPS Temp",     &GPSTemperature, CSV_TYPE_FLOAT_1DP },       // Temperature Sensors

  { "Radio RSSI",   &RoverRadioRSSI, CSV_TYPE_INT },              // LoRa Signal Peformance at Rover Receiver
  { "Radio SNR",    &RoverRadioSNR,  CSV_TYPE_INT },

  { "RC Manual PWM", &readRadioManualAuto, CSV_TYPE_INT },        // Pulse width on radio transmitter
  { "RC Throttle PWM", &readThrottle, CSV_TYPE_INT },
  { "RC Aileron PWM", &readAileron, CSV_TYPE_INT },

  { "Loop Timer (millis)", &csvReportingTime, CSV_TYPE_UINT }, // Scripts fussy on name
  { "LoopTime", &loopTime, CSV_TYPE_UINT },    // Time on Task (ms)
  { "SlackTime", &slackTime, CSV_TYPE_UINT },  // Time off Task (ms) 0 .. 100/200 ms

  { NULL, NULL, CSV_TYPE_NULL } // // End-of-Line for CSV record
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define CSV_FIELDS  5

void sendRPi_CSV(void) // Enumerate CSV Headings
{
  int i = 0;
  while(csv_headings[i].Name)
  {
    char string[64];
    char *s = string;
    if ((i % CSV_FIELDS) == 0)
    {
      if (i)
      {
        if (SerialRPI)   SerialRPI.println("");
        if (SerialDEBUG) SerialDEBUG.println("");
      }

      s += sprintf(s, "csv:");
    }
    else
      s += sprintf(s, ",");

    s += sprintf(s,"'%s'", csv_headings[i].Name);

    if (SerialRPI)   SerialRPI.print(string);
    if (SerialDEBUG) SerialDEBUG.print(string);
    i++;
  } // while

  if (SerialRPI)   SerialRPI.println("");
  if (SerialDEBUG) SerialDEBUG.println("");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*static*/ void reportRPi_Tabular(void)
{
  int i = 0;
  while(csv_headings[i].Name)
  {
    char string[32];
    char *s = string;

    if (i)
      *s++= ',';
    else
      s += sprintf(s, "rep:");

    switch(csv_headings[i].Type)
    {
      case CSV_TYPE_INT         : s += sprintf(s, "%d", *((int *)csv_headings[i].Pointer) ); break;
      case CSV_TYPE_UINT        : s += sprintf(s, "%u", *((unsigned int *)csv_headings[i].Pointer) ); break;

      case CSV_TYPE_FLOAT       : s += sprintf(s, "%f", *((float *)csv_headings[i].Pointer) ); break;
      case CSV_TYPE_FLOAT_1DP   : s += sprintf(s, "%.1f", *((float *)csv_headings[i].Pointer) ); break;
      case CSV_TYPE_FLOAT_2DP   : s += sprintf(s, "%.2f", *((float *)csv_headings[i].Pointer) ); break;
      case CSV_TYPE_FLOAT_3DP   : s += sprintf(s, "%.3f", *((float *)csv_headings[i].Pointer) ); break;
      case CSV_TYPE_FLOAT_INT   : s += sprintf(s, "%d", (int)*((float *)csv_headings[i].Pointer) ); break;

      case CSV_TYPE_DOUBLE      : s += sprintf(s, "%lf", *((double *)csv_headings[i].Pointer) ); break;
      case CSV_TYPE_DOUBLE_1DP  : s += sprintf(s, "%.1lf", *((double *)csv_headings[i].Pointer) ); break;
      case CSV_TYPE_DOUBLE_2DP  : s += sprintf(s, "%.2lf", *((double *)csv_headings[i].Pointer) ); break;
      case CSV_TYPE_DOUBLE_3DP  : s += sprintf(s, "%.3lf", *((double *)csv_headings[i].Pointer) ); break;
      case CSV_TYPE_DOUBLE_4DP  : s += sprintf(s, "%.4lf", *((double *)csv_headings[i].Pointer) ); break;
      case CSV_TYPE_DOUBLE_8DP  : s += sprintf(s, "%.8lf", *((double *)csv_headings[i].Pointer) ); break;
      case CSV_TYPE_DOUBLE_INT  : s += sprintf(s, "%d", (int)*((double *)csv_headings[i].Pointer) ); break;

      case CSV_TYPE_NULL        :
      default :
        *s = 0; // Trailing NUL
    }

    if (SerialRPI)   SerialRPI.print(string);
    if (SerialDEBUG) SerialDEBUG.print(string);

    i++;
  }

  if (SerialRPI)   SerialRPI.println(""); // End-of-Line for CSV record
  if (SerialDEBUG) SerialDEBUG.println("");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void reportRPi(unsigned long currentMillis)
{
  ////////////////////////////////////////////////
  // Pre-Reporting

  csvReportingTime = currentMillis;

  csvPolygon = pointInPolygon(currentLong, currentLat);

  // Internally 1 thru X, Here reporting 0 as HOME, 1 thru X-1 per MISSIONPLANNER
  csvWaypoint = numberWaypoints ? waypointNumber-1 : 999;

  // Distance in metres
  csvDistanceToTarget = distanceToTarget > 999.99 ? 999.99 : distanceToTarget;
  csvObstacleDistance = obstacleDistance > 999.99 ? 999.99 : obstacleDistance;

  if (chassis == zeroturnChassis)
  {
    csvLeft  = rpmLeft;  // RPM From Encoders
    csvRight = rpmRight;
  }
  else
  {
    csvLeft   = encoderLeft;  // Encoder Pulse
    csvRight  = encoderRight;
  }

#ifndef USE_INA260
  if (driveCount && (driveVoltage != 0.0))
    driveVoltageAverage = driveVoltage / (float)driveCount;
  else
    driveVoltageAverage = 0.0;

  if (driveCount && (drivePower != 0.0))
    drivePowerAverage = drivePower / (float)driveCount;
  else
    drivePowerAverage = 0.0;

  if (mowerCount && (mowerVoltage != 0.0))
    mowerVoltageAverage = mowerVoltage / (float)mowerCount;
  else
    mowerVoltageAverage = 0.0;

  if (mowerCount && (mowerPower != 0.0))
    mowerPowerAverage = mowerPower / (float)mowerCount;
  else
    mowerPowerAverage = 0.0;
#endif

  ////////////////////////////////////////////////
  // Reporting Phase

  if (saveDat) // Report to RPi  "rep:..."
  {
#if 1
    reportRPi_Tabular();
#else
    char string[32];

    sprintf(string,"rep:%u,%d,%d,",
      currentMillis,manualMode,pivoting);
    SerialRPI.print(string);

#if 1
    sprintf(string,"%d,%d,", // Simple/Original
      GPSFixQuality, GPSHeadingQuality); // Dual  (0=No Data,1=3D,4=Fixed,5=Float)
#else
    sprintf(string,"%d.%02d,%d,", // Extended w/satellite count
      GPSFixQuality, GPSSatelliteCount,
      GPSHeadingQuality); // Dual  (0=No Data,1=3D,4=Fixed,5=Float)
#endif
    SerialRPI.print(string);

    SerialRPI.print(currentLong, 8);
    SerialRPI.print(", ");
    SerialRPI.print(currentLat, 8);
    SerialRPI.print(",");

    //SerialRPI.print(currentAlt, 3);
    //SerialRPI.print(",");

    SerialRPI.print(GPSHeading, 2);
    SerialRPI.print(",");

    SerialRPI.print(IMUHeading, 2);
    SerialRPI.print(",");
    SerialRPI.print(IMURoll, 2);
    SerialRPI.print(",");
    SerialRPI.print(IMUPitch, 2);
    SerialRPI.print(",");

    if (waypointNumber > 1) // Only show past home
//    if ((waypointNumber > 1) && ((int)(forwardSpeed + 0.5) == normalForwardSpeed)) // Not pivoting, or correcting
    SerialRPI.print(crossTrackDistance, 2); // In cm, but show decimals
    else SerialRPI.print("-0.00"); // Flagging value

    SerialRPI.print(",");

    SerialRPI.print(targetHeading, 2);    // Degrees, from Dual GPS
    SerialRPI.print(",");
    SerialRPI.print(courseToAimPoint, 2); // Degrees
    SerialRPI.print(",");
    SerialRPI.print(headingError, 2);     // Degrees Rover current heading to target
    SerialRPI.print(",");
    SerialRPI.print(DeltaHTC);            // Degrees, Target Heading Error IMU ??
    SerialRPI.print(",");
    SerialRPI.print(distanceToTarget > 999.99 ? 999.99 : distanceToTarget); // Metres - TODO:FIX More Generally
    SerialRPI.print(",");
    SerialRPI.print(forwardSpeedGPS, 4);  // Metres/second
    SerialRPI.print(",");

    SerialRPI.print(forwardSpeed, 2);     // Applied forward speed +/- 100
    SerialRPI.print(",");
    SerialRPI.print(turnSpeed, 2);        // Applied turn speed +/- 100
    SerialRPI.print(",");

    SerialRPI.print(forwardSpeedLeft);    // Servo Degrees or Milliseconds
    SerialRPI.print(",");
    SerialRPI.print(forwardSpeedRight);
    SerialRPI.print(",");

    // Encoders (RPM or Pulses)
    SerialRPI.print(csvLeft, 3);
    SerialRPI.print(",");
    SerialRPI.print(csvRight, 3);
    SerialRPI.print(",");

    // Waypoint Information
    SerialRPI.print(csvWaypoint, DEC); // Internally 1 thru X, Here reporting 0 as HOME, 1 thru X-1 per MISSIONPLANNER
    SerialRPI.print(",");

    // Current waypoint
    SerialRPI.print(targetLong, 8);
    SerialRPI.print(",");
    SerialRPI.print(targetLat, 8);
    SerialRPI.print(",");

    // Previous waypoint
    SerialRPI.print(prevLong, 8);
    SerialRPI.print(",");
    SerialRPI.print(prevLat, 8);
    SerialRPI.print(",");

    SerialRPI.print(aimPointLong, 8);
    SerialRPI.print(",");
    SerialRPI.print(aimPointLat, 8);
    SerialRPI.print(",");

    // Geo-Fence
    SerialRPI.print(csvPolygon);
    SerialRPI.print(",");

    SerialRPI.print(movingAwayPivotTriggered);
    SerialRPI.print(",");

    // Sonar/Lidar
    SerialRPI.print(distanceLeftEcho);
    SerialRPI.print(",");
    SerialRPI.print(distanceCenterEcho);
    SerialRPI.print(",");
    SerialRPI.print(distanceRightEcho);
    SerialRPI.print(",");

    SerialRPI.print(loopTime); // Time on Task (ms)
    //SerialRPI.print(readRadioManualAuto); // Pulse width on radio transmitter
    SerialRPI.print(",");

    SerialRPI.print(slackTime); // Time off Task (ms) 0 .. 100/200 ms

#ifdef USE_INA260
    SerialRPI.print(",");
    SerialRPI.print(driveCurrent, 3); // A float
    SerialRPI.print(",");
    SerialRPI.print(driveVoltage, 3); // V float
    SerialRPI.print(",");
    SerialRPI.print(drivePower, 3);   // W float
    SerialRPI.print(",");
    SerialRPI.print(mowerCurrent, 3); // A float
    SerialRPI.print(",");
    SerialRPI.print(mowerVoltage, 3); // V float
    SerialRPI.print(",");
    SerialRPI.print(mowerPower, 3);   // W float
#else
    SerialRPI.print(",");
    SerialRPI.print(driveCurrent, 3); // A float
    SerialRPI.print(",");
    SerialRPI.print(driveVoltageAverage, 3); // V float
    SerialRPI.print(",");
    SerialRPI.print(drivePowerAverage, 3); // W float

    SerialRPI.print(",");
    SerialRPI.print(mowerCurrent, 3); // A float
    SerialRPI.print(",");
    SerialRPI.print(mowerVoltageAverage, 3); // V float
    SerialRPI.print(",");
    SerialRPI.print(mowerPowerAverage, 3); // W float
#endif

    SerialRPI.print(",");
    SerialRPI.print(currentAlt, 3); // Add to end, will refactor

    SerialRPI.print(",");
    SerialRPI.print(obstacleIdentified);     // Tree#
    SerialRPI.print(",");
    SerialRPI.print(csvObstacleDistance, 2); // Distance metres TODO:FIXME
    SerialRPI.print(",");
    SerialRPI.print(obstacleBearing, 2);     // Direction from antenna to tree
    SerialRPI.print(",");
    SerialRPI.print(obstacleAngle, 2);       // Direction from rover, corrected

    //SerialRPI.print(",");
    //SerialRPI.print(speedPIDEnable);
    //SerialRPI.print(",");
    //SerialRPI.print(normalForwardSpeed);

    SerialRPI.print(",");
    SerialRPI.print(speedPIDServo[0]);
    SerialRPI.print(",");
    SerialRPI.print(speedPIDServo[1]);

    SerialRPI.print(",");
    SerialRPI.print(GPSTemperature, 1); // Temperature Sensors

    SerialRPI.print(",");
    SerialRPI.print(RoverRadioRSSI); // LoRa Signal Peformance at Rover Receiver
    SerialRPI.print(",");
    SerialRPI.print(RoverRadioSNR);

    SerialRPI.println(""); // End-of-Line for CSV record
#endif // CSV

#if 0 // Debug to Arduino serial monitor
    if (SerialDEBUG)
    {
      SerialDEBUG.print("MODE MAN/AUTO:");
      SerialDEBUG.print(manualMode); // 1=Manual, 0=Auto
      SerialDEBUG.print(" READ-RADIO-MANUAL-AUTO:");
      SerialDEBUG.print(readRadioManualAuto);
      SerialDEBUG.print(" FORWARDSPEED:");
      SerialDEBUG.print(forwardSpeed, 2); // Applied forward speed +/- 100
      SerialDEBUG.print(" TURNSPEED:");
      SerialDEBUG.print(turnSpeed, 2);             // Applied turn speed +/- 100
      SerialDEBUG.println("");
    } // SerialDEBUG
#endif

#ifdef USE_OLED
{
char string[256];
updateWaypointEx();
sprintf(string, "#%-2d %6.2lf\n\n%14.8lf\n%14.8lf\n\n%14.8lf\n%14.8lf",
 csvWaypoint,
 csvDistanceToTarget,
 currentLong, currentLat,
 targetLong, targetLat);
ssd1306_Fill(Black);
ssd1306_OutString(string);
ssd1306_UpdateScreen();
}
#endif // USE_OLED

  } // if (saveDat)

  ////////////////////////////////////////////////

  if (printRadio1) // Data printed to Arduino serial monitor
  {
    if (SerialDEBUG)
    {
      //SerialDEBUG.print(millis() / 100);
      //SerialDEBUG.print(",");

#ifdef CAMERA_DETECT_PIN
  if (digitalRead(CAMERA_DETECT_PIN)) // Person In The Way
      SerialDEBUG.println("*** DUDE IN THE WAY ***");
  else
      SerialDEBUG.println("NO DUDE");
#endif // CAMERA_DETECT_PIN

      SerialDEBUG.print("Wp ");
      SerialDEBUG.print(numberWaypoints ? waypointNumber - 1 : 999, DEC); // 0-HOME
      SerialDEBUG.print(" of ");
      SerialDEBUG.print(numberWaypoints, DEC);
      SerialDEBUG.print(", ");
      SerialDEBUG.print("Speed ");
      SerialDEBUG.print(forwardSpeedGPS, 4); // Metres/second
      SerialDEBUG.print(", ");
      SerialDEBUG.print("DistM ");
      SerialDEBUG.print(distanceToTarget > 999.99 ? 999.99 : distanceToTarget); // Metres - TODO:FIX More Generally
      SerialDEBUG.print(", ");

      SerialDEBUG.print("FQ ");
      SerialDEBUG.print(GPSFixQuality);
      SerialDEBUG.print(":");
      SerialDEBUG.print(GPSHeadingQuality);
      SerialDEBUG.print(", ");

      //SerialDEBUG.print("GeoFence ");
      //SerialDEBUG.print(pointInPolygon(currentLong, currentLat));
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(movingAwayPivotTriggered);
      //SerialDEBUG.print(",");

      SerialDEBUG.print(currentLat, 8);
      SerialDEBUG.print(", ");
      SerialDEBUG.print(currentLong, 8);

      //SerialDEBUG.print(", IMU ");
      //SerialDEBUG.print(imuMessageCount);

#if 0
      SerialDEBUG.print(", GPS/IMU ");
      SerialDEBUG.print((int)encoderRight);
      SerialDEBUG.print(",");
      SerialDEBUG.print((int)encoderLeft);
      SerialDEBUG.print(",");
      SerialDEBUG.print(GPSFixQuality);
      SerialDEBUG.print(",");
      SerialDEBUG.print(GPSHeadingQuality);
      SerialDEBUG.print(",");
      SerialDEBUG.print(GPSHeadingDual, 2);

      SerialDEBUG.print(",");
      SerialDEBUG.print(radioReportingCycles);
#endif

#if 1
      SerialDEBUG.print(", SENSOR/LIDAR ");
      SerialDEBUG.print(distanceLeftEcho);
      SerialDEBUG.print(",");
      SerialDEBUG.print(distanceCenterEcho);
      SerialDEBUG.print(",");
      SerialDEBUG.print(distanceRightEcho);
#endif

      SerialDEBUG.println(" ");

      //SerialDEBUG.print(targetHeading, DEC);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(GPSHeading);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(IMUHeading);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(IMUerrorCorrection);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(DeltaHTC);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(headingError);
      //SerialDEBUG.print("....");
      //SerialDEBUG.print(crossTrackDistance, 0);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(forwardSpeed);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(turnSpeed);
      //SerialDEBUG.print("....");
      //SerialDEBUG.print(manualMode, 1);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(pivoting, 1);
      //SerialDEBUG.print(",");

      // Encoders
      //SerialDEBUG.print(encoderRight,0);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(encoderLeft,0);
      //SerialDEBUG.print(",");

      // IMU  pitch and roll
      //SerialDEBUG.print(IMURoll);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(IMUPitch);
      //SerialDEBUG.print(",");

      // Current  Location
      //SerialDEBUG.print(currentLat, 8);
      //SerialDEBUG.print(", ");
      //SerialDEBUG.print(currentLong, 8);
      //SerialDEBUG.print(",");

      // Way  point coordinates
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(targetLat, 8);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(targetLong, 8);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(targetLat - currentLat, 8);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(targetLong - currentLong, 8);
      //SerialDEBUG.print(" ");

      // Sonar/Lidar
      //SerialDEBUG.print(distanceLeftEcho);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(distanceCenterEcho);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(distanceRightEcho);
      //SerialDEBUG.print(",");

      // GPS
      //SerialDEBUG.print("lastReceivedGPSChars : ");
      //SerialDEBUG.print(lastReceivedGPSChars);
      //SerialDEBUG.print("strGPGGA : ");
      //SerialDEBUG.print(strGPGGA);
      //SerialDEBUG.print("strGPGLL : ");
      //SerialDEBUG.print(strGPGLL);
      //SerialDEBUG.print("strPOLYA : ");
      //SerialDEBUG.print(strPOLYA);
      //SerialDEBUG.print("GPSHeading : ");
      //SerialDEBUG.print(GPSHeading);

      // Other GPS
      //SerialDEBUG.print(pointInPolygon(currentLong, currentLat));
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(fixQuality2);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(GPS1.satellites.value());
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(GPS2.satellites.value());
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(distanceGPS12);
      //SerialDEBUG.print(", ");

      // Servos
      //SerialDEBUG.print(leftServoOutput);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(rightServoOutput);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(leftServoOutputMapped);
      //SerialDEBUG.print(",");
      //SerialDEBUG.print(rightServoOutputMapped);
      //SerialDEBUG.println(",");
    } // if (SerialDEBUG)

    // Move to Radio?

    if (driveCountLongTerm) // Compute long term stuff (for radio currently)
      driveCurrentLongTermAverage = (float)(driveCurrentLongTerm / (double)driveCountLongTerm);

    if (mowerCountLongTerm) // Compute long term stuff (for radio currently)
      mowerCurrentLongTermAverage = (float)(mowerCurrentLongTerm / (double)mowerCountLongTerm);

    //if ((radioReportingCycles % 20) == 0) // Save Radio from saturation
      radioReport(); // Send report to Ground Control Station

    radioReportingCycles = (radioReportingCycles + 1) % 100000;

#ifdef REPORT_OBSTACLE
    reportCloseObstacle(); // Report nearest obstacle (avoidence.ino)
#endif // REPORT_OBSTACLE
  } // if (printRadio1)

  ////////////////////////////////////////////////
  // Post Reporting

  movingAwayPivotTriggered = 0;

#ifndef USE_INA260
  driveCount = 0;
  drivePower = 0.0; // Average Power
  driveCurrent = 0.0; // Peak Current
  driveVoltage = 0.0; // Average Voltage

  mowerCount = 0;
  mowerPower = 0.0; // Average Power
  mowerCurrent = 0.0; // Peak Current
  mowerVoltage = 0.0; // Average Voltage
#endif // USE_INA260

  ////////////////////////////////////////////////

#ifdef USE_NEOPIXEL
  NeoPixelCycle();
#endif // USE_NEOPIXEL

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sendStateRaspEx(int state) // Send current rover state to RPi
{
  String sendStr = "sts:" + String(state) + ":";

  SerialRPI.println();
  SerialRPI.println(sendStr); // RPi Control Channel
  SerialRPI.flush();

  if (SerialDEBUG)
  {
    SerialDEBUG.print("RPI: OUT:");
    SerialDEBUG.println(sendStr); // Arduino Debug
  }

  prevState = state;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sendStateRasp(void) // Send current rover state to RPi
{
  String sendStr = "sts:" + String(state) + ":";

  SerialRPI.println(sendStr); // RPi Control Channel
  SerialRPI.flush();

  if (prevState != state)
  {
    prevState = state;
    if (SerialDEBUG)
    {
      SerialDEBUG.print("RPI: OUT:");
      SerialDEBUG.println(sendStr); // Arduino Debug
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
