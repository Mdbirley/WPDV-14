//Functions in this file...
// [1]download()         Download full mission data & other settings from RPi
// [2]readParams()       Read serial stream from the RPi
// [3]fillVariablesFromDownload()Update global variables with values received from RPi
// [4]readWaypoints()      Fill the wayponts list for the upcoming mission
// [5]readCompassCorrection()  Get compass calibration data from the downloaded variables
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Supress some of the annoying warnings
//pragma GCC diagnostic ignored "-Wno-psabi"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MAX_DWLD 200

//////////////////////////////////////////////////////////////////////////////////////////////////

void initDownload(void) // Move to a common routine
{
  setState(-S_INIT);            // Reset mower state and report

  raspberryParameters.clear();  // Clear stored data received previously from RPi

  waypointList.clear();         // Clear waypointList from previous mission

  obstacleList.clear();         // Clear obstacleList from previous mission

  compassCorrection.clear();

  download();                   // Download mission data & other settings from RPi     ROOT: "download.ino"

  numberWaypoints = waypointList.size();      // first waypoint = zero

  if (SerialDEBUG) printDownload();           // Preview downloaded data from RPi on the serial monitor
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void download(void) // Download full mission data & other settings from RPi
{
  unsigned long ledMillis = millis();

  if (SerialDEBUG) SerialDEBUG.println("Downloading mission data from RPi");

  statusupdateTimerDownload = millis(); // Initialize status update timer during download (only used here)

  cmd = c_empty;

  while(true)
  {
    unsigned long currentMillis = millis(); // Entry time

    if ((currentMillis - ledMillis) >= 75) // 8 Hz Blink to indicate waiting for RPi interaction
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Heart Beat
      ledMillis = currentMillis;
    }

    if ((currentMillis - statusupdateTimerDownload) >= (intervalStatusupdateDownload * 5))   // Update status every second
    {
      static unsigned int i = 0;

      if (SerialDEBUG) SerialDEBUG.println("RPI/DEBUG: Initiating Download Request into RPi");

#ifdef SerialRADIO
      if (SerialRADIO) SerialRADIO.println("RPI/RADIO: Initiating Download Request into RPi");
#endif // SerialRADIO

 #if 0
      i = (i + 1) % 2;
      if (i)
        state = S_WAITING; // cause a WAITING/INIT transition
      else
 #endif
        state = S_INIT;  // Reset mower state

      sendStateRaspEx(state); // Send current rover state to RPi

      ledTask(); // Update LEDs to reflect the GPS status

      //statusupdateTimerDownload += intervalStatusupdateDownload;            // Advance the strike point, stops time line walking off
      statusupdateTimerDownload = currentMillis;
    }

    serialTask(); // Pump all serial including RPi and GPS

    if (newData == true) // New RPi Serial data
    {
      newData = false;

#if 1 // DEBUG_RPI
      if (SerialDEBUG)
      {
        SerialDEBUG.print("RPI: '");
        SerialDEBUG.print(serData);
        SerialDEBUG.println("' Download Waiting Loop");
      }
#endif

      if (serData.substring(0, 3) == "par") // parameters
      {
        readParams();
      }
      else if (serData.substring(0, 3) == "way") // waypoints
      {
        readWaypoints();
      }
      else if (serData.substring(0, 3) == "com") // compass correction
      {
        readCompassCorrection();
      }
      else if (serData.substring(0, 3) == "obs") // obstacles
      {
        readObstacles();
      }
      else if (serData.substring(0, 5) == "cmd:7")  // Request INI Parameter Enumeration
      {
        sendRPi_INI(); // Enumerate INI Parameters
      }
      else if (serData.substring(0, 5) == "cmd:8")  // Request CSV Headings Enumeration
      {
        sendRPi_CSV(); // Enumerate CSV Headings
      }
      else if (serData.substring(0, 5) == "cmd:5")  // RELATE TO download ready par!!
      {
        if (SerialDEBUG)
        {
          SerialDEBUG.print("received number off parameters from Due:");
          SerialDEBUG.println(raspberryParameters.size());

          SerialDEBUG.println("all parameters received, copy to Due parameters!");
        }

        fillVariablesFromDownload();

        cmd = c_empty;
        state = S_WAITING;
        sendStateRasp();                // Send current rover state to RPi "sts:xx" reporting.ini
        break; // out of while
      }
 #if 0
      else if (serData.substring(0, 4) == "cmd:")
      {
        cmd = serData.substring(4, 5).toInt();

        if (SerialDEBUG)
        {
          SerialDEBUG.print("RPI: cmdx: '");
          SerialDEBUG.print(serData);
          SerialDEBUG.print("' ");
          SerialDEBUG.println(cmd);
        }

        if (cmd != 1) break; // out of while, if not download
      }
 #endif
    } // (newData == true)

  } // while
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
        generalParamList=['saveData','printRadio1','objectAvoidance','fixQualityEnable','polygonEnable']
        chassisParamList=['waypointTolerance','baudrateMotor','normalForwardSpeed','rampupTimeForwardSpeed',
          'pivotHeadingTolerance','pivotMode','pivotForwardSpeed','pivotTurnSpeedBegin','pivotTurnSpeedMiddle',
          'pivotTurnSpeedEnd','minHeadingError','maxHeadingError','startTurnspeedMinHEadingError','endTurnspeedMaxHEadingError',
          'minObjectDistance','minTurnDistance','turnSpeedDuringAvoidance','PIDEnable','PIDDirection','Kp','Ki','Kd',
          'steeringSensitivity','servoLeftValueLeft','servoLeftValueMiddle','servoLeftValueRight',
          'servoRightValueLeft','servoRightValueMiddle','servoRightValueRight','ztDirection','purePursuitD5','purePursuitPIDEnable',
          'purePursuitKp','purePursuitKi','purePursuitKd','ppPIDDirection','movingAwayPivot','movingAwayTolerence',
          'speedSetpoint','speedPIDEnable','speedKp','speedKi','speedKd', 'stanleyEnable','stanleyGain','stanleyKp','stanleyKi','stanleyKd',
          'pipeCrossTrackThreshold','pipeTurnSpeedLimit']

*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define INI_CLASS_NULL    0

#define INI_CLASS_CHASSIS 0
#define INI_CLASS_GENERAL 1
#define INI_CLASS_PARAMS  2

#define INI_TYPE_NULL   0

#define INI_TYPE_INT    100
#define INI_TYPE_UINT   200
#define INI_TYPE_FLOAT  300
#define INI_TYPE_DOUBLE 400

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _INI_PARAMS
{
  const char *Name;
  void *Pointer;
  int Class;
  int Type;
} INI_PARAMS;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

INI_PARAMS ini_params[] = {
// [CHASSIS] parameters
  { "chassis",&chassis,INI_CLASS_CHASSIS,INI_TYPE_INT },

// [GENERAL] parameters
  { "saveData",        &saveDat,          INI_CLASS_GENERAL, INI_TYPE_INT },
  { "printRadio1",     &printRadio1,      INI_CLASS_GENERAL, INI_TYPE_INT },
  { "objectAvoidance", &objectAvoidance,  INI_CLASS_GENERAL, INI_TYPE_INT },
  { "fixQualityEnable",&fixQualityEnable, INI_CLASS_GENERAL, INI_TYPE_INT },
  { "polygonEnable",   &polygonEnable,    INI_CLASS_GENERAL, INI_TYPE_INT },
  
// [MODEL],[MAIN],[ZEROTURN],[ACKERMAN] parameters
  { "waypointTolerance", &waypointTolerance, INI_CLASS_PARAMS, INI_TYPE_FLOAT },
  { "baudrateMotor",     &baudrateMotor,     INI_CLASS_PARAMS, INI_TYPE_UINT },
  
  { "normalForwardSpeed",    &normalForwardSpeed,     INI_CLASS_PARAMS,INI_TYPE_INT },
  { "rampupTimeForwardSpeed",&rampupTimeForwardSpeed, INI_CLASS_PARAMS,INI_TYPE_INT },

  { "pivotHeadingTolerance",&pivotHeadingTolerance,INI_CLASS_PARAMS,INI_TYPE_INT },
  { "pivotMode",&pivotMode,INI_CLASS_PARAMS,INI_TYPE_INT },

  { "pivotForwardSpeed",   &pivotForwardSpeed,   INI_CLASS_PARAMS,INI_TYPE_INT },
  { "pivotTurnSpeedBegin", &pivotTurnSpeedBegin, INI_CLASS_PARAMS,INI_TYPE_INT },
  { "pivotTurnSpeedMiddle",&pivotTurnSpeedMiddle,INI_CLASS_PARAMS,INI_TYPE_INT },
  { "pivotTurnSpeedEnd",   &pivotTurnSpeedEnd,   INI_CLASS_PARAMS,INI_TYPE_INT },
  
  { "minHeadingError", &minHeadingError,INI_CLASS_PARAMS,INI_TYPE_DOUBLE },
  { "maxHeadingError", &maxHeadingError,INI_CLASS_PARAMS,INI_TYPE_DOUBLE },

  { "startTurnspeedMinHEadingError", &startTurnspeedMinHEadingError, INI_CLASS_PARAMS,INI_TYPE_DOUBLE },
  { "endTurnspeedMaxHEadingError",   &endTurnspeedMaxHEadingError,   INI_CLASS_PARAMS,INI_TYPE_DOUBLE },

  { "minObjectDistance",        &minObjectDistance,        INI_CLASS_PARAMS, INI_TYPE_INT },
  { "minTurnDistance",          &minTurnDistance,          INI_CLASS_PARAMS, INI_TYPE_INT },
  { "turnSpeedDuringAvoidance", &turnSpeedDuringAvoidance, INI_CLASS_PARAMS, INI_TYPE_INT },
  
  { "PIDEnable",&PIDEnable,INI_CLASS_PARAMS,INI_TYPE_INT },
  { "PIDDirection",&PIDDirection,INI_CLASS_PARAMS,INI_TYPE_INT },
  { "Kp",&Kp,INI_CLASS_PARAMS,INI_TYPE_FLOAT },
  { "Ki",&Ki,INI_CLASS_PARAMS,INI_TYPE_FLOAT },
  { "Kd",&Kd,INI_CLASS_PARAMS,INI_TYPE_FLOAT },
  
  { "steeringSensitivity", &steeringSensitivity,INI_CLASS_PARAMS,INI_TYPE_DOUBLE },

  { "servoLeftValueLeft",  &servoLeftValueLeft,  INI_CLASS_PARAMS,INI_TYPE_INT },
  { "servoLeftValueMiddle",&servoLeftValueMiddle,INI_CLASS_PARAMS,INI_TYPE_INT },
  { "servoLeftValueRight", &servoLeftValueRight, INI_CLASS_PARAMS,INI_TYPE_INT },

  { "servoRightValueLeft",  &servoRightValueLeft,  INI_CLASS_PARAMS,INI_TYPE_INT },
  { "servoRightValueMiddle",&servoRightValueMiddle,INI_CLASS_PARAMS,INI_TYPE_INT },
  { "servoRightValueRight", &servoRightValueRight, INI_CLASS_PARAMS,INI_TYPE_INT },
  
  { "ztDirection", &ztDirection,INI_CLASS_PARAMS,INI_TYPE_INT },

  { "purePursuitD5",        &purePursuitD5,        INI_CLASS_PARAMS, INI_TYPE_DOUBLE },
  { "purePursuitPIDEnable", &purePursuitPIDEnable, INI_CLASS_PARAMS, INI_TYPE_INT },
  { "purePursuitKp",        &purePursuitKp,        INI_CLASS_PARAMS, INI_TYPE_FLOAT },
  { "purePursuitKi",        &purePursuitKi,        INI_CLASS_PARAMS, INI_TYPE_FLOAT },
  { "purePursuitKd",        &purePursuitKd,        INI_CLASS_PARAMS, INI_TYPE_FLOAT },
  { "ppPIDDirection",       &ppPIDDirection,       INI_CLASS_PARAMS, INI_TYPE_INT },
  
  { "movingAwayPivot",     &movingAwayPivot,     INI_CLASS_PARAMS, INI_TYPE_INT },
  { "movingAwayTolerence", &movingAwayTolerence, INI_CLASS_PARAMS, INI_TYPE_DOUBLE },

  { "speedSetpoint",  &speedSetpoint,  INI_CLASS_PARAMS, INI_TYPE_DOUBLE },
  { "speedPIDEnable", &speedPIDEnable, INI_CLASS_PARAMS, INI_TYPE_INT },
  { "speedKp",        &speedKp,        INI_CLASS_PARAMS, INI_TYPE_DOUBLE },
  { "speedKi",        &speedKi,        INI_CLASS_PARAMS, INI_TYPE_DOUBLE },
  { "speedKd",        &speedKd,        INI_CLASS_PARAMS, INI_TYPE_DOUBLE },
  
  { "stanleyEnable",  &stanleyEnable, INI_CLASS_PARAMS, INI_TYPE_INT },
  { "stanleyGain",    &stanleyGain,   INI_CLASS_PARAMS, INI_TYPE_FLOAT },
  { "stanleyKp",      &stanleyKp,     INI_CLASS_PARAMS, INI_TYPE_FLOAT },
  { "stanleyKi",      &stanleyKi,     INI_CLASS_PARAMS, INI_TYPE_FLOAT },
  { "stanleyKd",      &stanleyKd,     INI_CLASS_PARAMS, INI_TYPE_FLOAT },

  { "pipeCrossTrackThreshold", &pipeCrossTrackThreshold, INI_CLASS_PARAMS, INI_TYPE_DOUBLE },
  { "pipeTurnSpeedLimit",      &pipeTurnSpeedLimit,      INI_CLASS_PARAMS, INI_TYPE_DOUBLE },

  { NULL, NULL, INI_CLASS_NULL, INI_TYPE_NULL }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define INI_FIELDS  3

void sendRPi_INI(void) // Enumerate INI Parameters
{
  int i = 0;
  while(ini_params[i].Name)
  {
    char string[64];
    char *s = string;
    if ((i % INI_FIELDS) == 0)
    {
      if (i)
      {
        if (SerialRPI)   SerialRPI.println("");
        if (SerialDEBUG) SerialDEBUG.println("");
      }

      s += sprintf(s, "ini:");
    }
    else
      s += sprintf(s, ",");

    s += sprintf(s,"%d,'%s'", i, ini_params[i].Name);

    if (SerialRPI)   SerialRPI.print(string);
    if (SerialDEBUG) SerialDEBUG.print(string);
    i++;
  } // while

  if (SerialRPI)   SerialRPI.println("");
  if (SerialDEBUG) SerialDEBUG.println("");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void readParams(void) // Read serial stream from the RPi
{
//  int index1 = 0; // ??
//  int index2 = 0; // ??
  char inputchar[MAX_DWLD];
  char *tmp;

  // par:xxxx

  serData.substring(4).toCharArray(inputchar, sizeof(inputchar)-1) ;  // Find number of parameters
  tmp = strtok(inputchar, ",");
  while(tmp)
  {
    if (SerialDEBUG)
    {
      SerialDEBUG.print("params: ");
      SerialDEBUG.println(tmp);
    }
    raspberryParameters.push_back(tmp);
    tmp = strtok(NULL, ",");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void fillVariablesFromDownload(void) // Update global variables with values received from RPi
{
  chassis = raspberryParameters[0].toInt();

  if (SerialDEBUG)
  {
    SerialDEBUG.print("Selected Chassis: ");
    switch (chassis)
    {
      case 1:
        SerialDEBUG.println("MODEL");
        break;
      case 2:
        SerialDEBUG.println("MAIN");
        break;
      case 3:
        SerialDEBUG.println("ZEROTURN");
        break;
      case 4:
        SerialDEBUG.println("ACKERMAN");
        break;
      default:
        SerialDEBUG.println("Chassis not selected");
        break;
    }
  }

  saveDat                       = raspberryParameters[dl_saveDat].toInt();
  printRadio1                   = raspberryParameters[dl_printRadio1].toInt();
  objectAvoidance               = raspberryParameters[dl_objectAvoidance].toInt();
  fixQualityEnable              = raspberryParameters[dl_fixQualityEnable].toInt();
  polygonEnable                 = raspberryParameters[dl_polygonEnable].toInt();
  waypointTolerance             = raspberryParameters[dl_waypointTolerance].toFloat();
  baudrateMotor                 = raspberryParameters[dl_baudrateMotor].toInt();

  normalForwardSpeed            = raspberryParameters[dl_normalForwardSpeed].toInt();
  rampupTimeForwardSpeed        = raspberryParameters[dl_rampupTimeForwardSpeed].toInt();

  pivotHeadingTolerance         = raspberryParameters[dl_pivotHeadingTolerance].toInt();
  pivotMode                     = raspberryParameters[dl_pivotMode].toInt();
  pivotForwardSpeed             = raspberryParameters[dl_pivotForwardSpeed].toInt();
  pivotTurnSpeedBegin           = raspberryParameters[dl_pivotTurnSpeedBegin].toInt();
  pivotTurnSpeedMiddle          = raspberryParameters[dl_pivotTurnSpeedMiddle].toInt();
  pivotTurnSpeedEnd             = raspberryParameters[dl_pivotTurnSpeedEnd].toInt();

  minHeadingError               = raspberryParameters[dl_minHeadingError].toInt();
  maxHeadingError               = raspberryParameters[dl_maxHeadingError].toInt();

  startTurnspeedMinHEadingError = raspberryParameters[dl_startTurnspeedMinHEadingError].toInt();
  endTurnspeedMaxHEadingError   = raspberryParameters[dl_endTurnspeedMaxHEadingError].toInt();

  minObjectDistance             = raspberryParameters[dl_minObjectDistance].toInt();
  minTurnDistance               = raspberryParameters[dl_minTurnDistance].toInt();
  turnSpeedDuringAvoidance      = raspberryParameters[dl_turnSpeedDuringAvoidance].toInt();

  PIDEnable                     = raspberryParameters[dl_PIDEnable].toInt();
  PIDDirection                  = raspberryParameters[dl_PIDDirection].toInt();
  Kp                            = raspberryParameters[dl_Kp].toFloat();
  Ki                            = raspberryParameters[dl_Ki].toFloat();
  Kd                            = raspberryParameters[dl_Kd].toFloat();

  purePursuitD5                 = raspberryParameters[dl_purePursuitD5].toFloat();
  purePursuitPIDEnable          = raspberryParameters[dl_purePursuitPIDEnable].toInt();
  purePursuitKp                 = raspberryParameters[dl_purePursuitKp].toFloat();
  purePursuitKi                 = raspberryParameters[dl_purePursuitKi].toFloat();
  purePursuitKd                 = raspberryParameters[dl_purePursuitKd].toFloat();
  ppPIDDirection                = raspberryParameters[dl_ppPIDDirection].toInt();

  movingAwayPivot               = raspberryParameters[dl_movingAwayPivot].toInt();
  movingAwayTolerence           = raspberryParameters[dl_movingAwayTolerence].toDouble();

  // forwardSpeed control parameters
  speedSetpoint                 = raspberryParameters[dl_speedSetpoint].toDouble(); // NO
  speedPIDEnable                = raspberryParameters[dl_speedPIDEnable].toInt();
  speedKp                       = raspberryParameters[dl_speedKp].toDouble();
  speedKi                       = raspberryParameters[dl_speedKi].toDouble();
  speedKd                       = raspberryParameters[dl_speedKd].toDouble();

  // Probably deprecated at this point
  stanleyEnable                 = raspberryParameters[dl_stanleyEnable].toInt();
  stanleyGain                   = raspberryParameters[dl_stanleyGain].toFloat();
  stanleyKp                     = raspberryParameters[dl_stanleyKp].toFloat();
  stanleyKi                     = raspberryParameters[dl_stanleyKi].toFloat();
  stanleyKd                     = raspberryParameters[dl_stanleyKd].toFloat();

 // TODO: Wait for RPi code to add these properly
 // pipeCrossTrackThreshold     = raspberryParameters[dl_pipeCrossTrackThreshold].toFloat();
 // pipeTurnSpeedLimit          = raspberryParameters[dl_pipeTurnSpeedLimit].toFloat();

  if (chassis == zeroturnChassis) // Variables for zeroturn chassis
  {
    steeringSensitivity   = raspberryParameters[dl_steeringSensitivity].toFloat();
    servoLeftValueLeft    = raspberryParameters[dl_servoLeftValueLeft].toInt();
    servoLeftValueMiddle  = raspberryParameters[dl_servoLeftValueMiddle].toInt();
    servoLeftValueRight   = raspberryParameters[dl_servoLeftValueRight].toInt();
    servoRightValueLeft   = raspberryParameters[dl_servoRightValueLeft].toInt();
    servoRightValueMiddle = raspberryParameters[dl_servoRightValueMiddle].toInt();
    servoRightValueRight  = raspberryParameters[dl_servoRightValueRight].toInt();
    ztDirection           = raspberryParameters[dl_ztDirection].toInt();
  }
  else
  if (chassis == ackermanChassis) // Variables for ackerman chassis
  {
    steeringSensitivity   = raspberryParameters[dl_steeringSensitivity].toFloat();
    servoLeftValueLeft    = raspberryParameters[dl_servoLeftValueLeft].toInt();
    servoLeftValueMiddle  = raspberryParameters[dl_servoLeftValueMiddle].toInt();
    servoLeftValueRight   = raspberryParameters[dl_servoLeftValueRight].toInt();
    servoRightValueLeft   = raspberryParameters[dl_servoRightValueLeft].toInt();
    servoRightValueMiddle = raspberryParameters[dl_servoRightValueMiddle].toInt();
    servoRightValueRight  = raspberryParameters[dl_servoRightValueRight].toInt();
    ztDirection           = raspberryParameters[dl_ztDirection].toInt();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void readWaypoints(void) // Fill the waypoints list for the upcoming mission
{
  String string1;
  String string2;
  char *tmp;
  char inputchar[MAX_DWLD];

  // way:xxx
  serData.substring(4).toCharArray(inputchar, sizeof(inputchar)-1);
  tmp = strtok(inputchar, ",");
  while(tmp)
  {
    string1 = tmp; // Longitude
    tmp = strtok(NULL, ",");
    string2 = tmp; // Latitude
    tmp = strtok(NULL, ",");
    waypointList.push_back(waypointClass(string2.toDouble(), string1.toDouble()));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void readCompassCorrection(void)
{
  String string1;
  char *tmp;
  char inputchar[MAX_DWLD];

  // com:xxx
  serData.substring(4).toCharArray(inputchar, sizeof(inputchar)-1);
  tmp = strtok(inputchar, ",");
  while(tmp)
  {
    string1 = tmp;
    tmp = strtok(NULL, ",");
    compassCorrection.push_back(string1.toInt());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void readObstacles(void)
{
  String string1;
  String string2;
  String string3;
  char *tmp;
  char inputchar[MAX_DWLD];

  // obs:xxx
  serData.substring(4).toCharArray(inputchar, sizeof(inputchar)-1);
  tmp = strtok(inputchar, ",");
  while(tmp)
  {
    string1 = tmp; // Longitude
    tmp = strtok(NULL, ",");
    string2 = tmp; // Latitude
    tmp = strtok(NULL, ",");
    string3 = tmp; // Width
    tmp = strtok(NULL, ",");
    obstacleList.push_back(obstacleClass(string2.toDouble(), string1.toDouble(), string3.toDouble() ));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void printDownload(void) // Preview downloaded data from RPi on the serial monitor
{
  if (SerialDEBUG)
  {
		SerialDEBUG.println("parameters");
		for(int	i	=	0; i < raspberryParameters.size(); i++)
		{
			SerialDEBUG.print( raspberryParameters[i]);
			SerialDEBUG.print(", ");
		}
		SerialDEBUG.println();

    int waypointCount = waypointList.size();

		SerialDEBUG.print("Number of Waypoints:");
		SerialDEBUG.println(waypointCount);
		for(int	i	=	0; i <	waypointCount; i++)
		{
			SerialDEBUG.print(" Way: ");
			SerialDEBUG.print(i + 1); // 1 is	Home Point
			SerialDEBUG.print(" Lat: ");
			SerialDEBUG.print(waypointList[i].getLat(), 8);
			SerialDEBUG.print(" Long: ");
			SerialDEBUG.println(waypointList[i].getLong(),	8);
		}
		SerialDEBUG.println();

		int	obstacleCount	=	obstacleList.size(); // ??

		SerialDEBUG.print("Number of Obstacles:");
		SerialDEBUG.println(obstacleCount);
		for(int	i	=	0; i < obstacleCount;	i++)
		{
			SerialDEBUG.print(" Obs: ");
			SerialDEBUG.print(i + 1);
			SerialDEBUG.print(" Lat: ");
			SerialDEBUG.print(obstacleList[i].getLat(), 8);
			SerialDEBUG.print(" Long: ");
			SerialDEBUG.println(obstacleList[i].getLong(),	8);
		}
		SerialDEBUG.println();

		int	compassCount = compassCorrection.size();

		SerialDEBUG.print("Number of Compass Corrections:");
		SerialDEBUG.println(compassCount);
		for(int	i	=	0; i < compassCount; i++)
		{
			SerialDEBUG.print(compassCorrection[i]);
			SerialDEBUG.print(", ");
		}
		SerialDEBUG.println();
  } // if (SerialDEBUG)
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
