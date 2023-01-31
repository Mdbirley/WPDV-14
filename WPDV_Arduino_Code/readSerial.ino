//Functions in this file...
//[1]readSerialGPS()            Read GPS serial stream @ Serial port 1 every 1 ms
//[2]parseGPS()                 Parse GPS data & update current rover GPS position and heading
//[x]readSerialSensorBoard      Read serial data from sensor board
//[x]parseSensorBoard           Parse serial data from sensor board
//[3]readSerialIMU()            Read UM7 / IMU300 serial stream @ Serial port 2
//[4]parseIMU()                 Parse UM7 / IMU300 data & update current rover IMU heading, roll and pitch
//[5]getIMUHeading()            Parse UM7 data to get current UM7 heading
//[6]getIMURoll()               Parse UM7 data to get current UM7 roll
//[7]getIMUPitch()              Parse UM7 data to get current UM7 pitch
//[10]readSerialRasp()          Read serial stream from RPi & check for command updates
//[11]parseRasp()               Parse RPi command request
//[11]readSerialRoboteQ()       Read serial stream from RoboteQ motor controller
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Refactoring to clean up, and process data as it arrives
//  Radio/GPS outputs RMC, GGA, POLYA, POLYB and GNHDT

#define MAX_NMEA 200

static char strGPGGA[MAX_NMEA];
static char strGPGLL[MAX_NMEA];
static char strGPRMC[MAX_NMEA];

//static char strGNHDT[MAX_NMEA];
//static char strPOLYA[MAX_NMEA];
//static char strGPGGX[MAX_NMEA];

static int newParseGPS(char *str);
static int newParseIMU(char *str);

static void parseIMU380(char *str);
static void parseSONAR(char *str);

//static void getIMUHeading(char *str);
//static void getIMURoll(char *str);
//static void getIMUPitch(char *str);

static void getIMURollPitchHeading(char *str);

static void getRMCSpeed(char *str);
static void getGGA(char *str);
static void getHDTHeading(char *str);
static void getTMPTemperature(char *str);
static void getRSSI(char *str);
static void getPOLYAHeading(char *str);
static void getPOLYBNorthingEasting(char *str);

static double imuLat = 0.0;
static double imuLon = 0.0;

static double dualLat = 0.0;
static double dualLon = 0.0;
static double dualAng = 0.0;

static int dualTimeout = 0;

static long GPSFixTime = 0;
static long GPSHeadingTime = 0;

static long GPSByteTime = 0;
static long GPSByteCount = 0;

int motionTimeout = 0;
double motionDistance = 0;
int motionState = 0;   // 0-MOVING,1-STALLED

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double NormalizeHeading(double head) // Convert +/-180 or 0..360
{
	if (head < 0.0)     head += 360.0;
  if (head >= 360.0)  head -= 360.0;
	return(head);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Implementations lacking strsep(), strtok() mishandles empty fields
//  and isn't thread safe

/*
 * Get next token from string *stringp, where tokens are possibly-empty
 * strings separated by characters from delim.
 *
 * Writes NULs into the string at *stringp to end tokens.
 * delim need not remain constant from call to call.
 * On return, *stringp points past the last NUL written (if there might
 * be further tokens), or is NULL (if there are definitely no more tokens).
 *
 * If *stringp is NULL, strsep returns NULL.
 */
char *_strsep(char **stringp, const char *delim)
{
  char *s;
  const char *spanp;
  int c, sc;
  char *tok;

  if ((s = *stringp) == NULL)
    return (NULL);
  for (tok = s;;) {
    c = *s++;
    spanp = delim;
    do {
      if ((sc = *spanp++) == c) {
        if (c == 0)
          s = NULL;
        else
          s[-1] = 0;
        *stringp = s;
        return (tok);
      }
    } while (sc != 0);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readSerialGPS(void) // Public Function
{
  static char buffer[MAX_NMEA]; // buffer for received GPS data
  static int index = 0; // Read GPS stream every 1 msec

  static boolean newGPSData = false;

  while((SerialGPS.available() > 0) && (newGPSData == false)) // Drop out at each complete line
  {
    char rc = SerialGPS.read();

    GPSByteCount++;
    GPSByteTime = millis();

#ifdef USE_GPS_LIB
    GPS1.encode(rc); // Feed byte to GPS library to digest
#endif // USE_GPS_LIB

    if (rc != '\r')
    {
      if (rc == '$') index = 0; // Resync NMEA

      if (rc != '\n') // Not /r or /n
        buffer[index++] = rc;

      if (index >= sizeof(buffer)) // Keep bounded
        index = sizeof(buffer) - 1;
    }
    else { // '\r'
      buffer[index] = '\0'; // Terminate the string
      index = 0;

      if (newParseGPS(buffer)) // Try to parse on-the-fly
        newGPSData = true;
    }
  } // while

  if (newGPSData) // One new line available
  {
    // Should be taken my new handlers

    if (strncmp(buffer, "$GPGLL", 6) == 0)
      strcpy(strGPGLL, buffer);
//    else if (strncmp(buffer, "$GPGGA", 6) == 0)
//      strcpy(strGPGGA, buffer);
//    else if (strncmp(buffer, "$GPRMC", 6) == 0)
//      strcpy(strGPRMC, buffer);
//    else if (strncmp(buffer, "$POLYA", 6) == 0)
//      strcpy(strPOLYA, buffer);
//    else if (strncmp(buffer, "$GNHDT", 6) == 0)
//      strcpy(strGNHDT, buffer);

    newGPSData = false;
  } // if (newGPSData)

  // Adding support for GPS Data Timeouts

  if ((millis() - GPSFixTime) > 2000) // GPS Fix Stalled
  {
    GPSFixQuality = 0; // Invalidate
  }

  if ((millis() - GPSHeadingTime) > 2000) // GPS Heading Stalled
  {
    GPSHeadingQuality = 0; // Invalidate
  }

  if ((millis() - GPSByteTime) > 2000) // GPS Data Stalled
  {
    GPSFixQuality = 0; // Invalidate
    GPSHeadingQuality = 0; // Invalidate
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int checksumNMEA(char *str)
{
  unsigned char checksum = 0;
  int i = 1;
  while(str[i] && (str[i] != '*'))
    checksum ^= str[i++];
  if (str[i++] == '*')
  {
    unsigned char sum = (int)strtol(&str[i], NULL, 16);
    if (checksum == sum) return(1);
  }
  return(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int newParseGPS(char *str) // Handle stream directly
{
  // Validate the checksum, so we don't injest birds..
  if ((strncmp(str, "$G", 2) == 0) && (checksumNMEA(str) == 0))
  {
    //encoderLeft++; // Failed(Debug)
    return(0); // Discard
  }
  else
  if ((strncmp(str, "$P", 2) == 0) && (checksumNMEA(str) == 0))
  {
    //encoderLeft++; // Failed(Debug)
    return(0); // Discard
  }

  //encoderRight++; // Taken(Debug)

  if (strncmp(str, "$IMU,", 5) == 0) // IMU, Not checksummed currently
  {
#ifndef USE_DUAL
    parseIMU380(str);
#endif // USE_DUAL

    return(0); // Processed
  }
  else if ((strncmp(str, "$GPHDT,", 7) == 0) || (strncmp(str, "$GNHDT,", 7) == 0))
  {
    getHDTHeading(str);
    return(0); // Processed
  }
  else if ((strncmp(str, "$GPGGA,", 7) == 0) || (strncmp(str, "$GNGGA,", 7) == 0))
  {
    getGGA(str);
    return(0); // Processed
  }
  else if ((strncmp(str, "$GPRMC,", 7) == 0) || (strncmp(str, "$GNRMC,", 7) == 0))
  {
    getRMCSpeed(str);
    return(0); // Processed
  }
  else if ((strncmp(str, "$GPGLL,", 7) == 0) || (strncmp(str, "$GNGLL,", 7) == 0))
  {
    // Injested
    return(0); // Processed
  }
  else if (strncmp(str, "$POLYA,", 7) == 0) // Antenna Pair Orientation
  {
    getPOLYAHeading(str);
    return(0); // Processed
  }
  else if (strncmp(str, "$POLYB,", 7) == 0) // Relative to Base Location
  {
    getPOLYBNorthingEasting(str);
    return(0); // Processed
  }
  else if (strncmp(str, "$PCHRA,", 7) == 0) // Relative to Base Location
  {
    getIMURollPitchHeading(str);
    return(0); // Processed
  }
  else if ((strncmp(str, "$GPTMP,", 7) == 0) || (strncmp(str, "$GNTMP,", 7) == 0))
  {
    getTMPTemperature(str);
    return(0); // Processed
  }
  else if (strncmp(str, "$LRSSI,", 7) == 0)
  {
    getRSSI(str);
    return(0); // Processed
  }

  return(1); // Not processed
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// $IMU(0),towms(1),lon(2),lat(3),roll(4),pitch(5),yaw(6)*

// currentLong
// currentLat
// GPSHeading
// forwardSpeedGPS

static void parseIMU380_TOW(char *str)
{
  char temp[MAX_NMEA];
  int i;
  int tow;
  str = strcpy(temp, str);
  for(i=0; i<7; i++)
  {
    char *token = _strsep(&str, ",*");
    if (!token) break;
    if (i == 1) // TOW
    {
      tow = atoi(token);
    }
    else if (i == 2) imuLon    = atof(token); // Longitude 9dp
    else if (i == 3) imuLat    = atof(token);
    else if (i == 4) IMURoll   = atof(token); // Roll 3dp
    else if (i == 5) // PITCH
    {
      if (token[0] == 0)
        IMUPitch = 123.456;
      else
        IMUPitch  = atof(token); // Pitch 3dp
    }
    else if (i == 6) // YAW
    {
      if (token[0] == 0)
        IMUHeading = 123.456;
      else
        IMUHeading = atof(token); // Yaw 3dp

#ifndef USE_DUAL
      currentLong = imuLon;
      currentLat  = imuLat;

      IMUvsGPSHeadingError = NormalizeHeading(IMUHeading) - NormalizeHeading(GPSHeading);

      GPSHeadingQuality = 9; // IMU
      GPSHeadingTime = millis();

      GPSHeading = NormalizeHeading(IMUHeading);
#else
      IMUvsGPSHeadingError = NormalizeHeading(IMUHeading) - NormalizeHeading(GPSHeadingDual);
#endif // USE_DUAL

#ifdef DEBUG_IMU
      if (((imuMessageCount % 25) == 0) && SerialDEBUG)
      {
        char str[64];
        sprintf(str, "IMU: %7.3lf, %7.3lf, %7.3lf", IMURoll, IMUPitch, IMUHeading);
        SerialDEBUG.println(str);
        sprintf(str, "IMU: LAT:%.8lf LON:%.8lf", currentLat, currentLong);
        SerialDEBUG.println(str);
      }
#endif // DEBUG_IMU
    }
  }

  //GPSFixQuality = 4; // Place Holder, assume IMU reporting solid info

  imuMessageCount++;
  if (imuMessageCount >= 1000000) imuMessageCount = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// $IMU(0),lon(1),lat(2),roll(3),pitch(4),yaw(5)* !!!### WITHOUT TOW ###!!!

// currentLong
// currentLat
// GPSHeading
// forwardSpeedGPS

static void parseIMU380(char *str)
{
  char temp[MAX_NMEA];
  int i;
  int tow;
  str = strcpy(temp, str);
  for(i=0; i<6; i++)
  {
    char *token = _strsep(&str, ",*");
    if (!token) break;
    if (i == 1) imuLon    = atof(token); // Longitude 9dp
    else if (i == 2) imuLat    = atof(token);
    else if (i == 3) IMURoll   = atof(token); // Roll 3dp
    else if (i == 4) // PITCH
    {
      if (token[0] == 0)
        IMUPitch = 123.456;
      else
        IMUPitch  = atof(token); // Pitch 3dp
    }
    else if (i == 5) // YAW
    {
      if (token[0] == 0)
        IMUHeading = 123.456;
      else
        IMUHeading = atof(token); // Yaw 3dp

#ifndef USE_DUAL
      currentLong = imuLon;
      currentLat  = imuLat;

      IMUvsGPSHeadingError = NormalizeHeading(IMUHeading) - NormalizeHeading(GPSHeading);

      GPSHeadingQuality = 9; // IMU
      GPSHeadingTime = millis();

      GPSHeading = NormalizeHeading(IMUHeading);
#else
      IMUvsGPSHeadingError = NormalizeHeading(IMUHeading) - NormalizeHeading(GPSHeadingDual);
#endif // USE_DUAL

#ifdef DEBUG_IMU
      if (((imuMessageCount % 25) == 0) && SerialDEBUG)
      {
        char str[64];
        sprintf(str, "IMU: %7.3lf, %7.3lf, %7.3lf", IMURoll, IMUPitch, IMUHeading);
        SerialDEBUG.println(str);
        sprintf(str, "IMU: LAT:%.8lf LON:%.8lf", currentLat, currentLong);
        SerialDEBUG.println(str);
      }
#endif // DEBUG_IMU
    }
  }

  //GPSFixQuality = 4; // Place Holder, assume IMU reporting solid info

  imuMessageCount++;
  if (imuMessageCount >= 1000000) imuMessageCount = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void parseSONAR(char *str) // $SONAR,left,right
{
  char temp[MAX_NMEA];
  int i;
  str = strcpy(temp, str);
  for(i=0; i<3; i++)
  {
    char *token = _strsep(&str, ",*");
    if (!token) break;
    if (i == 1) distanceLeftEcho = atoi(token); // integer, cm
    else if (i == 2) distanceRightEcho = atoi(token);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void parseGPS(void) // Public
{
#ifdef USE_GPS_LIB
  currentLong = GPS1.location.lng();
  currentLat = GPS1.location.lat();
  currentAlt = GPS1.altitude.meters();

    if (0)
    {
#ifdef DEBUG_SPEED
  //simulate actual speed for testing
  forwardSpeedGPS = (double) forwardSpeed * speedTestGain; //m/s
  //SerialDEBUG.print("gain:");
  //SerialDEBUG.print(speedTestGain);
  //SerialDEBUG.print(" calc forwardspeedGPS:");
  //SerialDEBUG.println(forwardSpeedGPS);
#else
  if (GPS1.speed.isValid()) {
    forwardSpeedGPS = (GPS1.speed.mps());
  }
  else {
    forwardSpeedGPS = INVALID_GPS_SPEED;
  }
#endif // DEBUG_SPEED
    }
    else
    {
      getRMCSpeed(strGPRMC);
    }
#endif // USE_GPS_LIB
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void getRMCSpeed(char *str) // $GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,0.0,E,A*20
{
  char temp[MAX_NMEA];
  int i;
  double lat, lon, cog;
  char lat_hemi, lon_hemi;
  str = strcpy(temp, str);
  for(i=0; i<13; i++)
  {
    char *token = _strsep(&str, ",*");
    if (!token) break;
        //  0 $GPRMC
        //  1 HHMMSS
        //  2 STATUS A/V
        //  3 LAT
        //  4 LAT HEMI
        //  5 LON
        //  6 LON HEMI
        //  7 SOG
        //  8 COG
        //  9 YYMMDD
        // 10
        // 11 POSMODE N=NOFIX,A=AUTONOMOUS,E=DR,F=FLOAT,R=FIX
        // 12 NAVSTATUS (4.1) A
    if ((i == 2) && (token[0] == 'V'))
    {
      forwardSpeedGPS = INVALID_GPS_SPEED;
      break;
    }
    else if (i == 3)
      lat = atof(token);
    else if (i == 4)
      lat_hemi = token[0];
    else if (i == 5)
      lon = atof(token);
    else if (i == 6)
      lon_hemi = token[0];
    else if (i == 7)
      forwardSpeedGPS = atof(token) / (3600.0 / 1852.0); // Pull as a floating point double (in knots, convert to m/s)
    else if (i == 8)
      cog = atof(token);
    else if (i == 11)
    {
      char posmode = token[0];

        int lat_deg, lon_deg;
        double lat_min, lon_min;

        lat_deg = (int)lat / 100;

        lat_min = lat - (lat_deg * 100);

        lat = (double)lat_deg + (lat_min / 60.0);

        if (lat_hemi == 'S')
          lat = -lat;

        lon_deg = (int)lon / 100;

        lon_min = lon - (lon_deg * 100);

        lon = (double)lon_deg + (lon_min / 60.0);

        if (lon_hemi == 'W')
          lon = -lon;

      dualLat = lat;
      dualLon = lon;
      dualAng = cog;

#ifndef USE_DUAL
      if ((imuLat == 0.0) && (imuLon == 0.0)) // When IMU not reporting, use GPS
      {
        currentLat  = dualLat;
        currentLong = dualLon;

        GPSHeadingQuality = 8; // RMC
        GPSHeadingTime = millis();

        GPSHeading = NormalizeHeading(dualAng);
      }
#endif // USE_DUAL
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int gpsMessageCount = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void getGGA(char *str) // $GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B
{
  char temp[MAX_NMEA];
  int i;
  double lat, lon;
  char lat_hemi, lon_hemi;
  str = strcpy(temp, str);
  for(i=0; i<15; i++)
  {
    char *token = _strsep(&str, ",*");
    if (!token) break;
        //  0 $GPGGA
        //  1 HHMMSS
        //  2 LAT
        //  3 LAT HEMI
        //  4 LON
        //  5 LON HEMI
        //  6 Quality
        //  7 Number of Satellites
        //  8 HDOP
        //  9 Altitiude
        // 10 M
        // 11 Geoid Separation
        // 12 M
        // 13 Differential Station
        // 14 Differential Age
    if (i == 2) lat = atof(token);
    else if (i == 3) lat_hemi = token[0];
    else if (i == 4) lon = atof(token);
    else if (i == 5) lon_hemi = token[0];
    else if (i == 6)
    {
      GPSFixQuality = atoi(token); // 4=Fixed, 5=Float
      GPSFixTime = millis();

      if (dualTimeout == 0)
        GPSHeadingQuality = 0; // Invalidate
      else
        dualTimeout--;

#ifndef USE_DUAL
      if ((imuLat == 0.0) && (imuLon == 0.0)) // When IMU not reporting, use GPS
#endif // USE_DUAL
      { // GGA
        int lat_deg, lon_deg;
        double lat_min, lon_min;

        lat_deg = (int)lat / 100;

        lat_min = lat - (lat_deg * 100);

        lat = (double)lat_deg + (lat_min / 60.0);

        if (lat_hemi == 'S')
          lat = -lat;

        lon_deg = (int)lon / 100;

        lon_min = lon - (lon_deg * 100);

        lon = (double)lon_deg + (lon_min / 60.0);

        if (lon_hemi == 'W')
          lon = -lon;

#ifdef USE_DUAL_CORRECTION
if ((GPSFixQuality == 4) || (GPSFixQuality == 5)) // Fixed or Float
{
  if (dualTimeout >= 3) // Valid Heading from Secondary
  {
    myAdjustLatLonByBearingDistance(&lat, &lon, GPSHeadingDual, USE_DUAL_CORRECTION); // Forward by 30 cm
	}
}
#endif // USE_DUAL_CORRECTION

#define SEMI_MAJOR              (6378137.0)                     // WGS84
#define EARTH_CIRCUM_METRES     (SEMI_MAJOR * 2.0 * PI)
#define DEG_TO_METRES           (EARTH_CIRCUM_METRES / 360.0)

{
  double x = (currentLat  - lat) * DEG_TO_METRES;
  double y = (currentLong - lon) * DEG_TO_METRES;
  double z = sqrt(x*x + y*y);
  motionDistance = z * 100.0; // cm
  if (z > 0.02) // 2cm
  {
    motionTimeout = 8 * 5; // 5 Seconds at 8 Hz
    motionState = 0.0; // MOVING
  }
  else
  {
    if (motionTimeout == 0) // Stopped
    {
      motionState = 1.0; // STALLED
      if ((forwardSpeed > 0.0) || (fabs(turnSpeed) > 0.0)) // Likely to be a few hundred
      {
        if (SerialDEBUG) SerialDEBUG.println("*** Emergency Stop - Stalled ***");
//        stopRover();
//        savedState = state;
//        setState(S_WAITING);
      }
      motionTimeout--; // Go negative so won't fire a second time
    }
    else
    if (motionTimeout > 0) // Active count?
      motionTimeout--;
  }
}

        currentLat  = lat; // Coordinated used everywhere else
        currentLong = lon;
      }
    }
    else if (i == 7) GPSSatelliteCount = atoi(token);
  } // for

#ifdef DEBUG_GPS
  gpsMessageCount++;
  if (((gpsMessageCount % 25) == 0) && SerialDEBUG)
  {
    char str[64];
    sprintf(str, "GPS: LAT:%.8lf LON:%.8lf", lat, lon);
    SerialDEBUG.println(str);
 }
#endif // DEBUG_GPS
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void getHDTHeading(char *str) // $GNHDT,75.5554,T*45
{
  char temp[MAX_NMEA];
  int i;
  double heading;
  str = strcpy(temp, str);
  for(i=0; i<2; i++)
  {
    char *token = _strsep(&str, ",*"); // Split NMEA string with ',' delimiter
    if (!token) break;
    //  0 $GNHDT
    //  1 HEADING
    //  2 TRUE
    if (i == 1)
    {
      heading = atof(token); // Pull as a floating point double

      dualAng = heading;

#ifdef USE_DUAL
      dualTimeout = 5;
      GPSHeadingDual = dualAng;
      GPSHeadingTime = millis();
      GPSHeading = dualAng;
      // Add in a rotation to compensate for orientation wrt vehicle
      GPSHeading = NormalizeHeading(GPSHeading); // Normalize angle
#endif // USE_DUAL

    }
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void getPOLYAHeading(char *str) // $POLYA,112233.444,N,E,D,BASELINE,ANGLE,FIXTYPE
{
  char tempNMEA[MAX_NMEA];
  int i;
  double n, e, d, base, angle;
  str = strcpy(tempNMEA, str);
  for(i=0; i<8; i++)
  {
    char *token = _strsep(&str, ",*"); // Split NMEA string with ',' delimiter
    if (!token) break;
        //  0 $POLYA
        //  1 HHMMSS.sss
        //  2 N
        //  3 E
        //  4 N
        //  5 BASELINE
        //  6 ANGLE
        //  7 FIXTYPE
    if (i == 2) n = atof(token);
    else if (i == 3) e = atof(token);
    else if (i == 4) d = atof(token);
    else if (i == 5) base = atof(token);
    else if (i == 6) angle = atof(token); // Pull as a floating point double
    else if (i == 7) {
      GPSHeadingQuality = -1;
      if (token[0] == 'A') GPSHeadingQuality = 1;
      else if (token[0] == 'B') GPSHeadingQuality = 5; // FLOAT
      else if (token[0] == 'C') GPSHeadingQuality = 4; // FIXED
      dualAng = angle;
#ifdef USE_DUAL
      dualTimeout = 5;
      GPSHeadingDual = dualAng;
      GPSHeadingTime = millis();
      GPSHeading = NormalizeHeading(dualAng);
#endif // USE_DUAL
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void getPOLYBNorthingEasting(char *str) // $POLYB,112233.444,LAT,LON,NORTHING,EASTING
{
  char tempNMEA[MAX_NMEA];
  int i;
  double lat, lon, north, east;
  str = strcpy(tempNMEA, str);
  for(i=0; i<7; i++)
  {
    char *token = _strsep(&str, ",*"); // Split NMEA string with ',' delimiter
    if (!token) break;
        //  0 $POLYB
        //  1 HHMMSS.sss
        //  2 LATITUDE
        //  3 LONGITUDE
        //  4 NORTHING (FROM BASE)
        //  5 EASTING
    if (i == 2) lat = atof(token);
    else if (i == 3) lon = atof(token);
    else if (i == 4) north = atof(token);
    else if (i == 5) east = atof(token);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void getTMPTemperature(char *str) // $GPTMP,time,123.0,C,234.5,F,123.0,C,234.5,F*45
{
  char temp[MAX_NMEA];
  int i;
  double temp_pos, temp_hed;
  str = strcpy(temp, str);
  for(i=0; i<9; i++)
  {
    char *token = _strsep(&str, ",*"); // Split NMEA string with ',' delimiter
    if (!token) break;
    //  0 $GPTMP
    //  1 TIME
    //  2 TEMP POSITION (C)
    //  3 C
    //  4 TEMP POSITION (F)
    //  5 F
    //  6 TEMP HEADING (C)
    //  7 C
    //  8 TEMP HEADING (F)
    //  9 F

    if (i == 4)
    {
      temp_pos = atof(token); // Pull as a floating point double

#ifndef USE_DUAL
      GPSTemperature = temp_pos; // Single Unit
#endif // USE_DUAL
    }
#ifdef USE_DUAL
    else if (i == 8)
    {
      temp_hed = atof(token); // Pull as a floating point double

      GPSTemperature = (temp_pos + temp_hed) / 2.0f; // Average of two units
    }
#endif // USE_DUAL
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void getRSSI(char *str) // $LRSSI,time,-40,20
{
  char temp[MAX_NMEA];
  int i;
  int rssi = 0, snr = 0;

  str = strcpy(temp, str);
  for(i=0; i<4; i++)
  {
    char *token = _strsep(&str, ",*"); // Split NMEA string with ',' delimiter
    if (!token) break;
    //  0 $LRSSI
    //  1 TIME
    //  2 RSSI
    //  3 SNR
    if (i == 2)
    {
      rssi = atoi(token); // Pull RSSI as int
    } else if (i == 3)
    {
      snr  = atoi(token); // Pull SNR as int
    }
  }

  RoverRadioRSSI = rssi;
  RoverRadioSNR  = snr;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lidar/Sensor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void parseSensorBoard(char *str)
{
  char *token  = strtok(str, ",:"); // Parse the Grand Central data "lid:l,m,r\n"
  if (token)
    token = strtok(NULL, ",");
  if (token)
  {
    distanceLeftEcho = atoi(token); // integer, cm
    token = strtok(NULL, ",");
  }
  if (token)
  {
    distanceCenterEcho = atoi(token); // integer, cm
    token = strtok(NULL, ",");
  }
  if (token)
  {
    distanceRightEcho = atoi(token);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readSerialSensorBoard(void)
{
#ifdef SerialSensorBoard
  static int index = 0;
  static char buffer[MAX_NMEA]; // buffer for Grand Central Lidar/Sensor Serial data

  while(SerialSensorBoard.available() > 0) // Read Grand Central Lidar/Sensor on Serial 3
  {
    char rc = SerialSensorBoard.read(); // Get the new byte
    buffer[index++] = rc;
    if ((rc == '\n') || (index == (sizeof(buffer) - 1))) // end-of-line or bounded
    {
      buffer[index] = '\0';
      index = 0;

      if (strncmp(buffer, "sonar:", 6) == 0)  //TODO changes for sonar test 2-11-22
        parseSensorBoard(buffer);
    } // if
  } // while
#endif // SerialSensorBoard
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sonar
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void newParseSonar(char *str)
{
  char *token  = strtok(str, ",*"); // Parse the MEGA Sonar data "x , y \n"
  if (token)
  {
    distanceLeftEcho = atoi(token); // integer, cm
    token = strtok(NULL, ",");
  }
  if (token)
  {
    distanceRightEcho = atoi(token);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readSerialSonar(void)  //TODO old code not used
{
#ifdef SerialSONAR
  static int index = 0;
  static char buffer[MAX_NMEA]; // buffer for MEGA Sonar Serial data

  while(SerialSONAR.available() > 0) // Read MEGA Sonar on Serial 3
  {
    char rc = SerialSONAR.read(); // Get the new byte
    buffer[index++] = rc;
    if ((rc == '\n') || (index == (sizeof(buffer) - 1))) // end-of-line or bounded
    {
      buffer[index] = '\0';
      index = 0;

      if (strncmp(buffer, "lid:", 4) == 0)
        newParseLidar(buffer);
      else
        newParseSonar(buffer);
    } // if
  } // while
#endif // SerialSONAR
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IMU
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// $GPRMB recommended minimum navigation information if you have a destination waypoint defined.
// $GPRMB,A,4.08,L,EGLL,EGLM,5130.02,N,00046.34,W,004.6,213.9,122.9,A*3D

/////////////////////////   Read & parse IMU UM7 / 300 serial stream      /////////////////////////

static char NMEA_Compass[MAX_NMEA] = "";

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readSerialIMU(void)
{
#ifdef SerialIMU
  static int index = 0;
  static char buffer[MAX_NMEA]; // buffer for Serial data

  while(SerialIMU.available() > 0) // Read IMU (UM7 or 300) on Serial 2
  {
    char rc = SerialIMU.read(); // Get the new byte
    if (rc == '$') index = 0; // Resync NMEA
    if ((rc == '\n') || (rc == '\r') || (index == (sizeof(buffer) - 1)))  // end-of-line or bounded
    {
      buffer[index] = '\0';

#ifdef DEBUG_IMU
      if (SerialDEBUG) SerialDEBUG.println(buffer);
#endif // DEBUG_IMU

      if (index)
        if (newParseIMU(buffer))
          strcpy(NMEA_Compass, buffer);

      index = 0;
    } // if
    else
      buffer[index++] = rc;
  } // while
#endif // SerialIMU
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void parseIMU(void) // Should refactor to a single pass parsing of parameters on single line
{
  if (strlen(NMEA_Compass) > 0)
  {
#if 0
    getIMUHeading(NMEA_Compass);
    getIMURoll(NMEA_Compass);
    getIMUPitch(NMEA_Compass);
#else
    getIMURollPitchHeading(NMEA_Compass);
#endif

    NMEA_Compass[0] = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int newParseIMU(char *str)
{
  if (strncmp(str, "$IMU,", 5) == 0)
  {
    parseIMU380(str);
    return(0);
  }
  else if (strncmp(str, "$SONAR,", 7) == 0)
  {
    parseSONAR(str);
    return(0);
  }
  return(1); // Not processed
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef NEVER
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Variables for Serial inputs 1 and 2 GPS and 0 UM7
char* findHeading;
char* findRoll;
char* findPitch;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void getIMUHeading(char *str)
{
  char tempNMEA[MAX_NMEA];
  int i;
  strcpy(tempNMEA, str);
  findHeading = strtok(tempNMEA, ",*");         // Split NMEA string with ',' delimiter
  for (i=0; i<4; i++)                           // Find the value in the fifth position of the string
  {
    findHeading = strtok(NULL, ",*");
  }
  IMUHeading = atof(findHeading); // Reports to 2dp
  IMUHeading -= IMUHeadingTrim;
  IMUHeading = NormalizeHeading(IMUHeading);

  // TODO: Understand and refactor this, and table it uses
  IMUcorrectionIndex = min(max(((int)IMUHeading - 1) / 10, 0), 35);     // Calculate the index from the compassCorrection
  IMUerrorCorrection = compassCorrection[IMUcorrectionIndex];

  headingEstimate = NormalizeHeading(IMUHeading + IMUerrorCorrection);  // Add the compass correctionfactor to the Heading
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void getIMURoll(char *str)
{
  char tempNMEA[MAX_NMEA];
  int i;
  strcpy(tempNMEA, str);
  findRoll = strtok(tempNMEA, ",*");            // Split NMEA string with ',' delimiter
  for (i=0; i<2; i++)                           // Find the value in the third position of the string
  {
    findRoll = strtok(NULL, ",*");
  }
  IMURoll = atof(findRoll); // Reports to 2dp
  IMURoll -= IMURollTrim;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void getIMUPitch(char *str)
{
  char tempNMEA[MAX_NMEA];
  int i;
  strcpy(tempNMEA, str);
  findPitch = strtok(tempNMEA, ",*");           // Split NMEA string with ',' delimiter
  for (i=0; i<3; i++)                           // Find the value in the fourth position of the string
  {
    findPitch = strtok(NULL, ",*");
  }
  IMUPitch = atof(findPitch); // Reports to 2dp
  IMUPitch -= IMUPitchTrim;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif // NEVER
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void getIMURollPitchHeading(char *str)
{
  char temp[MAX_NMEA];
  int i;
  str = strcpy(temp, str); // Which Sentence? $PCHRA,time,roll,pitch,yaw,heading,*checksum
  for(i=0; i<6; i++) // Enumerate through fields, once
  {
    char *token = _strsep(&str, ",*"); // Split NMEA string with ',' delimiter
    if (!token) break; // Check for NULL
    // 0=$PCHAR,1=time,2=roll,3=pitch,4=yaw,5=heading
    if (i == 2) IMURoll    = atof(token); // Reports to 2dp
    else if (i == 3) IMUPitch   = atof(token); // Reports to 2dp
    else if (i == 4) IMUHeading = atof(token); // Reports to 2dp
  }
  IMURoll 		-= IMURollTrim;
  IMUPitch  	-= IMUPitchTrim;
  IMUHeading	-= IMUHeadingTrim;
  IMUHeading = NormalizeHeading(IMUHeading);

  // TODO: Understand and refactor this, and table it uses
  IMUcorrectionIndex = min(max(((int)IMUHeading - 1) / 10, 0), 35);    // Calculate the index from the compassCorrection
  IMUerrorCorrection = compassCorrection[IMUcorrectionIndex];

  headingEstimate = NormalizeHeading(IMUHeading + IMUerrorCorrection); // Add the compass correctionfactor to the Heading
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RPi
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////   Read & parse RPi serial stream @ Serial port USB    /////////////////////////

void readSerialRasp(void) // Read serial stream from RPi
{
  static char buffer[MAX_NMEA];
  static int index = 0;
  static boolean recvInProgress = false;

  const char startMarker = '<';
  const char endMarker = '>';

//if (SerialDEBUG) SerialDEBUG.print('R');

  while((SerialRPI.available() > 0) && (newData == false))
  {
    char rc = SerialRPI.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker) {
        buffer[index] = rc;
        index++;
        if (index >= sizeof(buffer))
          index = sizeof(buffer) - 1;
      }
      else { // endMarker
        buffer[index] = '\0'; // terminate the string
        recvInProgress = false;
        index = 0;
        newData = true;
        serData = buffer;
      }
    }
    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }

//if (SerialDEBUG) SerialDEBUG.print('r');
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// newData flags a new processable message
// serData holds C++ string  cmd: par, way, com,  generally processed in download.ino

void parseRasp(void) // RPi check for command updates (called from main loop only)
{
  if (newData)
  {
    if (serData.substring(0, 4) == "cmd:") // command
    {
      cmd = serData.substring(4, 5).toInt();

#if 1 // DEBUG_RPI
      if (SerialDEBUG)
      {
      	SerialDEBUG.print("RPI: cmd: '");
      	SerialDEBUG.print(serData);
      	SerialDEBUG.print("' ");
      	SerialDEBUG.println(cmd);

      	if (cmd == 0) SerialDEBUG.println("CMD0 !!!!");
      }
#endif // DEBUG_RPI
    } // cmd
    else if (serData.substring(0, 4) == "adv:") // command
    {
      if (SerialDEBUG) SerialDEBUG.println("RPI: ADV");
      cmd = c_advance;
    }
    else
    {
      if (SerialDEBUG)
      {
      	SerialDEBUG.print("RPI: ");
      	SerialDEBUG.println(serData);
      }
    }

    newData = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RoboteQ
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MAX_RBQ_INPUT 16

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int roboteq_e[2]; // Closed Loop Error
int roboteq_f[2]; // Feedback
int roboteq_a[2]; // Motor Amps
int roboteq_v[2]; // Battery Voltage
int roboteq_m[2]; // Motor Command Applied
int roboteq_p[2]; // Power

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void processDataRoboteQ(const char * data)
{
  static char line1[MAX_RBQ_INPUT];
  static char line2[MAX_RBQ_INPUT];

#ifdef DEBUG_RBQ
  // Provide some debugging output so we can see what the RoboteQ is sending back
  if (SerialDEBUG)
  {
//  SerialDEBUG.print("DBG:ROBOTEQ [");
  	SerialDEBUG.print("RBQ: [");
  	SerialDEBUG.print(data);
  	SerialDEBUG.println("]");
  }
#endif // DEBUG_RBQ

  // copy line 2 to line 1
  strcpy(line1, line2);
  // copy data to line 2
  strcpy(line2, data);

  // Expects query and responses to be on subsequent lines and be in a specific format
  if (strncmp(line2, "C=", 2) == 0) // Encoders
  {
    if (strncmp(line1, "?C 1", 4) == 0) encoderRight = atol(line2 + 2);
    else if (strncmp(line1, "?C 2", 4) == 0) encoderLeft = atol(line2 + 2);
  }
  else if (strncmp(line2, "E=", 2) == 0) // Closed Loop Error
  {
    int value = atol(line2 + 2);
    if (strncmp(line1, "?E 1", 4) == 0)
      roboteq_e[0] = value;
    else if (strncmp(line1, "?E 2", 4) == 0)
      roboteq_e[1] = value;
  }
  else if (strncmp(line2, "F=", 2) == 0) // Feedback
  {
    int value = atol(line2 + 2);
    if (strncmp(line1, "?F 1", 4) == 0)
      roboteq_f[0] = value;
    else if (strncmp(line1, "?F 2", 4) == 0)
      roboteq_f[1] = value;
  }
  else if (strncmp(line2, "A=", 2) == 0) // Motor Current
  {
    int value = atol(line2 + 2);
    if (strncmp(line1, "?A 1", 4) == 0)
      roboteq_a[0] = value;
    else if (strncmp(line1, "?A 2", 4) == 0)
      roboteq_a[1] = value;
  }
  else if (strncmp(line2, "V=", 2) == 0) // Battery Voltage
  {
    int value = atol(line2 + 2);
    if (strncmp(line1, "?V 1", 4) == 0)
      roboteq_v[0] = value;
    else if (strncmp(line1, "?V 2", 4) == 0)
      roboteq_v[1] = value;
  }
  else if (strncmp(line2, "M=", 2) == 0) // Motor Command
  {
    int value = atol(line2 + 2);
    if (strncmp(line1, "?M 1", 4) == 0)
      roboteq_m[0] = value;
    else if (strncmp(line1, "?M 2", 4) == 0)
      roboteq_m[1] = value;
  }
  else if (strncmp(line2, "P=", 2) == 0) // Power
  {
    int value = atol(line2 + 2);
    if (strncmp(line1, "?P 1", 4) == 0)
      roboteq_p[0] = value;
    else if (strncmp(line1, "?P 2", 4) == 0)
      roboteq_p[1] = value;
  }
}  // end of processDataRoboteQ

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Process RoboteQ responses

static void processIncomingByteRoboteQ(const byte inByte)
{
  static unsigned int input_pos = 0;
  static char input_line[MAX_RBQ_INPUT];

  switch (inByte)
  {
    case '\r':   // carriage return
    case '\n':   // end of text
      input_line[input_pos] = 0;  // terminating null byte

      // terminator reached! process input_line here ...
      processDataRoboteQ(input_line);

      // reset buffer for next time
      input_pos = 0;
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_RBQ_INPUT - 1))
        input_line[input_pos++] = inByte;
      break;

  }  // end of switch

} // end of processIncomingByteRoboteQ

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readSerialRoboteQ()
{
#ifdef SerialMOTOR
  while(SerialMOTOR.available() > 0)
    processIncomingByteRoboteQ(SerialMOTOR.read());
#endif // SerialMOTOR
} // end of readSerialRoboteQ

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
