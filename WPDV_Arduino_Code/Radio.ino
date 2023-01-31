//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Radio.ino
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initRadio(void)
{
#ifdef SerialSENSOR
  if (SerialSENSOR)  SerialSENSOR.println("Opening SENSOR from NAV-BOARD,initRadio");
#endif // SerialSENSOR
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void parsePik(char *buffer)
{
#ifdef SerialDEBUG
  if (SerialDEBUG) SerialDEBUG.println(buffer);
#endif // SerialDEBUG
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readRadio(void)
{
#ifdef SerialSENSOR
{
  static int index = 0;
  static char buffer[250]; // Buffer for serial data

  int avail = SerialSENSOR.available();

  while(avail > 0) // Read Radio Serial while data available
  {
    char rc = SerialSENSOR.read(); // Get the new byte

    avail--;

    buffer[index++] = rc;

    if ((rc == '\n') || (index == (sizeof(buffer) - 1))) // end-of-line or bounded
    {
		  char *p;

			if ((rc == '\n') && index)
			  index--;

      buffer[index] = '\0';

			if (index)
			{
				if (p = strstr(buffer, "$PIK,")) // New Form
        	parsePik(p); // Need to checksum
      	else if (strncmp(buffer, "pik:", 4) == 0)
        	parsePik(buffer);
      	else if (p = strstr(buffer, "pik:"))
        	parsePik(p);

      	index = 0;
      	avail = SerialSENSOR.available();
      }
    } // if
  } // while
}
#endif // SerialSENSOR
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sendRadio(const char *head, const char *str) // Wrap outputs with checksum for integrity checking downstream
{
#ifdef SerialSENSOR
  static const char Hex[] = "0123456789ABCDEF";
	uint8_t checksum = 0;
	char tail[8], *t;

  int i, len;

  len = strlen(head);

 	for(i=1; i<len; i++) // Compute XOR checksum across head
		checksum ^= (uint8_t)head[i];

	len = strlen(str);

 	for(i=0; i<len; i++) // Compute XOR checksum across body
		checksum ^= (uint8_t)str[i];

  t = tail;

  *t++ = '*'; // Add * termination
	*t++ = Hex[(checksum >> 4) & 0x0F]; // Add hexidecimal checksum
	*t++ = Hex[(checksum >> 0) & 0x0F];
	*t++ = '\n';

	*t = 0;

	if (SerialSENSOR)
	{
  	SerialSENSOR.write(head, strlen(head));
  	SerialSENSOR.write(str,  strlen(str));
  	SerialSENSOR.write(tail, strlen(tail));
  }
#endif // SerialSENSOR
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void radioReport(void) // Compile and send whole report
{
#if defined(SerialRADIO) || defined(SerialSensorBoard)
{
  char string[250];
  int status = 100;

// Status 100, 200, 300 series warnings, error, etc
// Message STATE or ASCII ALERT
// FixQuality
// HeadingQuality
// WP
// WPDistance
// Speed
// XTE
// GPSHeading
// Aim Pt Heading
// Long
// Lat
//?? Mower/Drive Current, or Minutes Remaining ??
//?? Run Time

  // 5 Sep 2021 radio reporting

// Error Message
// Mower State
// Fix Quality
// Heading Quality
// Next Way Point
// Dist (m) to Way Point
// GPS Speed
// Cross Track Error (cm)
// GPS Heading
// Aim Pt Heading
// Drive Battery Voltage
// Mower Battery Voltage
// Drive Battery Amps
// Mower Battery Amps
// Drive Battery Capacity
// Mower Battery Capacity
// Latitude
// Longitude
// Temperature (deg C)


  // Status/Warnings, Higher Priority First

#ifdef CAMERA_DETECT_PIN
  if (digitalRead(CAMERA_DETECT_PIN)) status = 345; // Person In The Way
  else
#endif // CAMERA_DETECT_PIN
  if (pivotStallCheck(14000)) status = 307; // 14 Seconds, Error
  else
  if (driveVoltageAverage < 20.0) status = 315;
  else
  if ((mowerVoltageAverage < 20.0) && (mowerVoltageAverage != 0.0)) status = 320; // Low Voltage, but present
  else
  if ((driveCurrent > 13.0f) && (mowerCurrent > 17.0f)) status = 330;
  else
  if (distanceToTarget > 999.99) status = 333;
  else
  if (pivotStallCheck(7000)) status = 207; // 7 Seconds, Warning
  else
  if (driveVoltageAverage < 21.0) status = 201;
  else
  if (driveCurrent > 13.0f) status = 202; // Drive 13A limit
  else
  if (mowerVoltageAverage < 21.0) status = 203;
  else
  if (mowerCurrent > 17.0f) status = 204; // Mower 17A limit

  // Flag up basic chassis mismatch

#ifdef ROVER_ELECTRIC_V2
  if (chassis != mainChassis) status = 666; // ELECTRIC
#endif // ROVER_ELECTRIC_V2

#ifdef ROVER_ZEROTURN
  if (chassis != zeroturnChassis) status = 666; // GAS
#endif // ROVER_ZEROTURN

// Need some integrity check on this

  //                  1  2  3  4  5  6     7     8     9     10    11    12    13    14    15    16    17    18    19
  if (sprintf(string,"%d,%s,%d,%d,%d,%.2lf,%.2lf,%.2lf,%2.lf,%.2lf,%.2lf,%.2lf,%.3lf,%.3lf,%.2lf,%.2lf,%.8lf,%.8lf,%.1lf",
				status, // #1 100
				decodeState(state), // "MSG",
				GPSFixQuality,
				GPSHeadingQuality,
				waypointNumber,	// #5 WayPoint
				distanceToTarget > 999.99 ? 999.99 : distanceToTarget, // TODO:FIXME
				forwardSpeedGPS,	// Number (m/s)
				crossTrackDistance > 99.99 ? 99.99 : crossTrackDistance, // TODO:FIXME
				GPSHeading, // targetHeading, // TODO:FIXME Max wants GPSHeading
				courseToAimPoint, // #10
        driveVoltageAverage, mowerVoltageAverage, // %.2lf #11,#12
        //123.45, 678.91,
	      driveCurrentLongTermAverage, mowerCurrentLongTermAverage, // %.3lf #13,#14
        // driveBatteryCapacity,mowerBatteryCapacity, // #15,#16
        123.45, 678.91,
				currentLong, currentLat, // %.8lf #17,#18
        // temperature // #19
        GPSTemperature // 123.4
	     )	>= sizeof(string))
	{
#ifdef SerialSENSOR
    if (SerialSENSOR) SerialSENSOR.println("Buffer Exception");
#endif // SerialSENSOR
  }
	else
	{
	  sendRadio("$GSR,", string);

//#ifdef SerialSENSOR
//    if (SerialSENSOR) SerialSENSOR.write(string);
//#endif // SerialSENSOR
	}

  // Send condensed PIK related message

#ifdef SerialSENSOR
  //sprintf(string, "pik:%03d\n", status);
  //if (SerialSENSOR) SerialSENSOR.write(string);

  sprintf(string, "%03d", status);
  sendRadio("$PIK,", string);
#endif // SerialSENSOR

  // Recover PIK responses

#ifdef SerialSENSOR
  while(SerialSENSOR.available() > 0) // Ensure buffer is emptied, tied to GrandCentral failures
  {
    readRadio(); // Process Inbound PIK
  }
#endif // SerialSENSOR

}
#endif // SerialRADIO || SerialSensorBoard

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
