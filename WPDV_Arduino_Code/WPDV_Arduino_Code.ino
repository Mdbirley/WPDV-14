//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////          YouNoMow        /////////////////////////////////////
///////////////////////////////////   Target Grand Central   /////////////////////////////////////
///////////////////////////////////       25-Sep-2022        /////////////////////////////////////
///////////////////////////////////         07:18 PM         /////////////////////////////////////
///////////////////////////////////       Modified By:       /////////////////////////////////////
///////////////////////////////////                           /////////////////////////////////////
///////////////////////////////////                          /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
/*
   NOTE: This code is a simplified version of the "WPDV_ALL_Models_Rasp" Master firmware modified
   for debugging purposes. Specifically to test and validate the path-following control algorithm
   independent from the other feaures supported by the master version.
*/

/* 19-Jun-2022
Added in GPS Temperatures, and LoRa RSSI/SNR
*/

/*  6-Apr-2022
Per 7-Jan-2022, needs work for Mixed Mode, reduced gain on TurnSpeed as drive oscillated
Dropped from 10.0 to 2.5 in moverRover
Max needs to get range better understood.
*/

/*  7-Jan-2022
Switching to Mixed Mode servo input on RoboteQ, away from Tank Sticks
Need to rework moveRover some more
*/

/* 17-Jan-2021 Plan
PA3 (Primary GPS) Goes to IMU GPS_RX_IN (P2.2) [GND P2.5]
PA0 (Radio Dual GPS) Goes to Arduino Serial#1 @ 115200 baud [GPS PORT]
IMU Debug Output (P4.7)  [GND P4.1 or P4.2] Goes to Arduino Serial#2 @ 230400 baud [IMU PORT]
Roboteq Serial Goes to Arduino Serial#3 @ 115200 baud [MOTOR PORT]

The Roboteq Script outputs AUTO/MANUAL info on it's serial port so we
can reflect that to RPi

We add info about NED Velocity perhaps to $IMU output string.
*/

/* 26-Jan-2021
Need to allow switch between DUAL GPS and IMU+GPS operation
*/

/* 23-July-2021
Adding PID support left/right via ztEncoder.ino and speedPIDEnable = 2
*/

/* 24-Oct-2021
Pin 6 Kill
Pin 7 PTO / Mow Deck Engage
*/

//////////////////////////////////////////////////////////////////////////////////////////////////

#include "aDefinitions.h"               // All variables definitions and file includes

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef _VARIANT_GRAND_CENTRAL_M4_
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Serial1 Pins  1(TX), 0(RX) SERCOM0
// Serial2 Pins 18(TX),19(RX) SERCOM4
// Serial3 Pins 16(TX),17(RX) SERCOM1
// Serial4 Pins 14(TX),15(RX) SERCOM5
// Serial5 Pins 51(TX),52(RX) SERCOM7
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Uart Serial2(&SERCOM_SERIAL2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);

void SERCOM4_0_Handler() { Serial2.IrqHandler(); }
void SERCOM4_1_Handler() { Serial2.IrqHandler(); }
void SERCOM4_2_Handler() { Serial2.IrqHandler(); }
void SERCOM4_3_Handler() { Serial2.IrqHandler(); }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Uart Serial3(&SERCOM_SERIAL3, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX);

void SERCOM1_0_Handler() { Serial3.IrqHandler(); }
void SERCOM1_1_Handler() { Serial3.IrqHandler(); }
void SERCOM1_2_Handler() { Serial3.IrqHandler(); }
void SERCOM1_3_Handler() { Serial3.IrqHandler(); }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Uart Serial4(&SERCOM_SERIAL4, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PAD_SERIAL4_RX, PAD_SERIAL4_TX);

void SERCOM5_0_Handler() { Serial4.IrqHandler(); }
void SERCOM5_1_Handler() { Serial4.IrqHandler(); }
void SERCOM5_2_Handler() { Serial4.IrqHandler(); }
void SERCOM5_3_Handler() { Serial4.IrqHandler(); }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PIN_SERIAL5_RX

// Serial5
#define PIN_SERIAL5_RX      (66) // 52 PD9
#define PAD_SERIAL5_RX      (SERCOM_RX_PAD_1)
#define PIN_SERIAL5_TX      (65) // 51 PD8
#define PAD_SERIAL5_TX      (UART_TX_PAD_0)
#define SERCOM_SERIAL5      sercom7

#endif // PIN_SERIAL5_RX

Uart Serial5(&SERCOM_SERIAL5, PIN_SERIAL5_RX, PIN_SERIAL5_TX, PAD_SERIAL5_RX, PAD_SERIAL5_TX);

///  O   6-PIN ISP HEADER
//
//   1  MISO  50/64    5V           2
//      PD11 SERCOM7
//      PAD[3]
//
//   3  SCK   52/66    MOSI  51/65  4
//      PD9  SERCOM7   PD8  SERCOM7
//      PAD[1] RX      PAD[0] TX
//
//   5  NRESET         GND          6

void SERCOM7_0_Handler() { Serial5.IrqHandler(); }
void SERCOM7_1_Handler() { Serial5.IrqHandler(); }
void SERCOM7_2_Handler() { Serial5.IrqHandler(); }
void SERCOM7_3_Handler() { Serial5.IrqHandler(); }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif // _VARIANT_GRAND_CENTRAL_M4_
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////

static unsigned long mainLoopTimer = 0;        // Main loop timer

//////////////////////////////////////////////////////////////////////////////////////////////////

void setState(int newState)
{
   if ((state != newState) || (newState < 0)) // Negative to force
   {
     state = abs(newState); // Reset mower state
     stateMillis = millis(); // Time of last state transition
     sendStateRasp(); // Send current rover state to RPi     ROOT: "reporting.ino"
   }
}

//////////////////////////////////////////////////////////////////////////////////////////////////

const char *decodeState(int State)
{
#ifdef ROVER_ELECTRIC_V2
  if (chassis != mainChassis) return("CHASSIS"); // ELECTRIC
#endif // ROVER_ELECTRIC_V2

#ifdef ROVER_ZEROTURN
  if (chassis != zeroturnChassis) return("CHASSIS"); // GAS
#endif // ROVER_ZEROTURN

  switch(abs(State))
  {
    case S_INIT    : return("INIT");
    case S_WAITING : return("WAITING");
    case S_MOWING  : return("MOWING");
    case S_PIVOT   : return("PIVOT");
    case S_HOLD    : return("HOLD");
    case S_READY   : return("READY");
    case S_MANUAL  : return("MANUAL");
    default : return("UNKNOWN");
  }
}

//************************************* Setup **************************************************

void setup(void)
{
  unsigned long currentMillis = millis(); // Entry time

  pinMode(LED_BUILTIN, OUTPUT); // 13 Flat Side of Diode to ground thru resistor (330R)

  pinMode(LED_FIXQUALITY, OUTPUT);

  pinMode(BUTTON_START, INPUT);
  pinMode(BUTTON_STOP, INPUT);
  pinMode(BUTTON_DOWNLOAD, INPUT);

#ifdef CAMERA_DETECT_PIN
  pinMode(CAMERA_DETECT_PIN, INPUT);
#endif // CAMERA_DETECT_PIN

#ifdef USE_OLED
  OLED_Init();
#endif // USE_OLED

#ifdef USE_NEOPIXEL
  NeoPixelInit();
#endif // USE_NEOPIXEL

  SerialDEBUG.begin(SerialDEBUG_BaudRate);  // Initialize serial monitor
  //TEST while(!SerialDEBUG) // REMOVE
  delay(100);                               // Delay to initialize serial port
  if (SerialDEBUG) SerialDEBUG.println("<Serial to Arduino is ready>");

  SerialRPI.begin(SerialRPI_BaudRate);  // Initialize communication to RPi
  delay(100);                           // Delay to initialize serial port
  if (SerialDEBUG) SerialDEBUG.println("Wait for Raspberry communication");

  //Reset temporary disabled because it will not help for the restart problem
  //resetTimer = millis();
  //while(!SerialRPI and (millis() - resetTimer < 5000)) {}   // Wait until connection is established
  //if (!SerialRPI) {
  //  if (SerialDEBUG) SerialDEBUG.println("Reset Due to establish RPi communication");
  //  delay(100);
  //  rstc_start_software_reset(RSTC); // Reset Due when no RPi communication after 5 seconds
  //}

#ifdef SerialRADIO
  SerialRADIO.begin(SerialRADIO_BaudRate);  // Setup serial port for communication to Debug/Tracking Radio
  if (SerialRADIO) SerialRADIO.println("Opening RADIO from NAV-BOARD");
#endif // SerialRADIO

#ifdef SerialSensorBoard
  SerialSensorBoard.begin(SerialSensorBoard_BaudRate); // Setup serial port for communications from Grand Central Lidar/Sensor
  if (SerialSensorBoard) SerialSensorBoard.println("Opening LIDAR from NAV-BOARD");
#endif // SerialSensorBoard

#ifdef SerialSONAR
  SerialSONAR.begin(SerialSONAR_BaudRate); // Setup serial port for communications from MEGA Sonar
  if (SerialSONAR) SerialSONAR.println("Opening SONAR from NAV-BOARD");
#endif // SerialSONAR

#ifdef SerialGPS
  SerialGPS.begin(SerialGPS_BaudRate);  // Setup serial port for communication to GPS board @115200 bps
  if (SerialGPS) SerialGPS.println("Opening GPS from NAV-BOARD");
#endif // SerialGPS

#ifdef SerialIMU
  SerialIMU.begin(SerialIMU_BaudRate);  // Setup serial port for communication to UM7 IMU @115200 bps or IMU300 @230400 bps
  if (SerialIMU) SerialIMU.println("Opening IMU from NAV-BOARD");
#endif // SerialIMU

  //--RADIO----------------------------------
  initRadio();

//TEST sendRPi_INI(); // Enumerate INI Parameters
//TEST sendRPi_CSV();
//TEST reportRPi_Tabular();

  //--RPI------------------------------------
  if (SerialDEBUG) SerialDEBUG.println("<Serial to Raspberry Pi is ready>");
  initDownload(); // Mower Init and Download

  //-----------------------------------------
  // Initialisation forwardSpeed control
  //-----------------------------------------
  //set output limits Speed PID 0-100%
  speedPID.SetOutputLimits(0.0, 100.0);

#ifdef DEBUG_SPEED
  if (SerialDEBUG)
  {
    SerialDEBUG.print("speedSetpoint:");
    SerialDEBUG.print(speedSetpoint);
    SerialDEBUG.print(" speedPIDEnable:");
    SerialDEBUG.println(speedPIDEnable);
    SerialDEBUG.print(" speedKp:");
    SerialDEBUG.print(speedKp);
    SerialDEBUG.print(" speedKi:");
    SerialDEBUG.print(speedKi);
    SerialDEBUG.print(" speedKd:");
    SerialDEBUG.println(speedKd);
  }
#endif

  //-----------------------------------------

#ifdef USE_INA260
  //if (chassis == zeroturnChassis)
  initCurrentVoltagePower();   // AdaFruit Sensor
#endif // USE_INA260

  //-----------------------------------------

#ifdef SerialMOTOR // Electric
  SerialMOTOR.begin(SerialMOTOR_BaudRate);  // Setup serial port for communication to Sabertooth or Roboteq MDC2230 @115200 bps
#endif // SerialMOTOR

#ifdef USE_TFMINI
  tfmSetup();
#endif // USE_TFMINI

  initRover(); // Initialize all motor driver settings & connections to arduino    ROOT: "moveRover.ino"

  //Timer.getAvailable().attachInterrupt(readSerialGPS).start(1000); // Call "readSerialGPS" every 1 msec to get GPS raw readings

  currentMillis = millis();
  delay(3000 - (currentMillis % 1000));           // Wait for GPS to start
  currentMillis += 3000 - (currentMillis % 1000); // Align strike point so service latency/lag is more apparent
  mainLoopTimer = currentMillis;                  // Initialize loop timer
  previousMillisReporting = currentMillis;        // Initialize reporting timer
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ledTask(void)
{
//IMU300     GPSFixQuality = atoi(fixQual1.value()); // read GPS quality

     if (GPSFixQuality < 4) // WHATEVER (OFF)
        digitalWrite(LED_FIXQUALITY, 0);
     else if (GPSFixQuality == 4) // RTK FIXED (ON)
        digitalWrite(LED_FIXQUALITY, 1);
     else if (GPSFixQuality == 5) // RTK FLOAT (BLINK)
        digitalWrite(LED_FIXQUALITY, !digitalRead(LED_FIXQUALITY));
     else
        digitalWrite(LED_FIXQUALITY, 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void serialTask(void)
{
  if (chassis == zeroturnChassis)
  {
#ifdef SerialSONAR
    readSerialSonar(); // Read MEGA Sonar serial stream on ZT
#endif // SerialSONAR
  }
  else if (chassis == mainChassis) // Electric
  {
#ifdef SerialMOTOR
    readSerialRoboteQ(); // RoboteQ - Electric UK
#endif // SerialMOTOR
  }

#ifdef SerialSensorBoard
  readSerialSensorBoard();
#endif // SerialSensorBoard

  readSerialGPS(); // Read GPS stream ROOT: "readSerial.ino"
  readSerialIMU(); // Read IMU stream ROOT: "readSerial.ino"
  readSerialRasp(); // Read serial stream from RPi
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void mowingReset(void)
{
  //resetEncoders();          // Zero out encoder values at starting Mowing
  //initPurePursuitPID();     // Initialize ppPID values at starting Mowing
  forwardSpeed = calcForwardSpeed(1); // Initialize acceleration process
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void currentMeasureInit(void)
{
  driveCountLongTerm = 0;
  driveCurrentLongTerm = 0.0;

  mowerCountLongTerm = 0;
  mowerCurrentLongTerm = 0.0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void currentMeasureTask(void)
{
#ifdef USE_INA260
    //if (chassis == zeroturnChassis)
    readCurrentVoltagePower();   // AdaFruit Sensor
#else // USE_INA260

//#define ADC_VOLT_SCALE 34.0
#define ADC_VOLT_SCALE 31.0 // 1024/33 31.0303

#if defined(ADC_PIN_DRIVE_CURRENT) && defined(ADC_PIN_DRIVE_VOLTAGE)
    if (chassis == mainChassis)
    {
      float Current = (float)analogRead(ADC_PIN_DRIVE_CURRENT) / 17.0; // Going to need to determine scaling 0..1023 for 3.3V? or more bits
      float Voltage = (float)analogRead(ADC_PIN_DRIVE_VOLTAGE) / ADC_VOLT_SCALE; // 756 -> 22 V 34.36
      if (Voltage < 8.0f) // Battery Not Connected / Test
      {
        Current = 0.0f;
        Voltage = 0.0f;
      }
      driveCurrent = max(driveCurrent, Current); // Peak current
      driveVoltage += Voltage; // Average Voltage
      drivePower += (Current * Voltage); // Going to average in reporting.ino
      driveCount++; // Measurements contributing to average

      driveCurrentLongTerm += Current;
      driveCountLongTerm++;
    }
#endif // ADC_PINs DRIVE

#if defined(ADC_PIN_MOWER_CURRENT) && defined(ADC_PIN_MOWER_VOLTAGE)
    if (chassis == mainChassis)
    {
      float Current = (float)analogRead(ADC_PIN_MOWER_CURRENT) / 17.0; // Going to need to determine scaling 0..1023 for 3.3V? or more bits
      float Voltage = (float)analogRead(ADC_PIN_MOWER_VOLTAGE) / ADC_VOLT_SCALE; // 756 -> 22 V 34.36
      if (Voltage < 8.0f) // Battery Not Connected / Test
      {
        Current = 0.0f;
        Voltage = 0.0f;
      }
      mowerCurrent = max(mowerCurrent, Current); // Peak current
      mowerVoltage += Voltage; // Average Voltage
      mowerPower += (Current * Voltage); // Going to average in reporting.ino
      mowerCount++; // Measurements contributing to average

      mowerCurrentLongTerm += Current;
      mowerCountLongTerm++;
    }
#endif // ADC_PINs MOWER

#endif // USE_INA260
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//************************************* LOOP ***************************************************

void loop(void)
{
  static int lastManual = 0;
  static int hz = 0;
  unsigned long currentMillis = millis(); // Entry time

  { // Heart Beat LED
    static uint32_t start = millis();
    if ((millis() - start) >= 1000) // One Second HeatBeat
    {
      static uint32_t led = 0;
      if (((led % 60) == 0) && SerialDEBUG) SerialDEBUG.println("$GCM4,NAVSYS"); // Minute Report
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Heart Beat
      led++;
      start += 1000;
    }
  }

  if (countLoop >= 3)
  {
    readTransmitter(); // Read control inputs from radio receiver     ROOT: "manual.ino"

//    if (manualMode)
//      sendStateRaspEx(S_MANUAL);

    currentMeasureTask();

#ifdef USE_TFMINI
    if (chassis == mainChassis) tfmRead(); // TFMini Plus Sonars
#endif // USE_TFMINI

    ledTask();

    countLoop = 0;
  }

  //Disabling here as a test, otherwise hitting at very high loop rate, moved into the 100/200 ms loop below ENGR:CT
  //sendStateRasp();                // Send current rover state ro RPi

  if (chassis == zeroturnChassis) ztEncoderUpdate(); // Check the Zero-Turn Encoders (frequently, nominally 1 KHz)

  serialTask(); // Pull any pending data, should be cleared in idle loop

  if ((currentMillis - mainLoopTimer) >= intervalMain)  // Fix the loop interval to 100 msec
  {
    if ((currentMillis - mainLoopTimer) >= 100000) // If we've been out of loop for too long, just catch up
       mainLoopTimer = currentMillis - (currentMillis % 1000);

    while((currentMillis - mainLoopTimer) >= intervalMain)
    {
      mainLoopTimer += intervalMain; // Advance the next strike point, rather than let time line walk off
    }

     hz = (hz + 1) % MAIN_LOOP_HZ;
     if (hz == 0)
       sendStateRasp();  // Send current rover state to RPi "sts:xx" reporting.ini

    // Test code for forwardSpeed Control
#ifdef DEBUG_SPEED
    if (SerialDEBUG)
      while(SerialDEBUG.available() > 0)
      {
        speedTestGain = SerialDEBUG.parseFloat();
        SerialDEBUG.read(); //remove line end
        SerialDEBUG.print("read gain:");
        SerialDEBUG.println(speedTestGain);
      }
#endif // DEBUG_SPEED

    parseIMU();                   // Update rover current roll, pitch & heading from IMU  ROOT: "readSerial.ino"

    parseGPS();                   // Update rover current GPS location & heading  ROOT: "readSerial.ino"

    calcHeadingError();           // Calculate heading error  ROOT: "calcHeadingError.ino"

    readTransmitter();    // Read control inputs from radio receiver     ROOT: "manual.ino"
    // Will set manualMode based on pulse width

    parseRasp();         // Check command updates "cmd:%d" from RPi
    // Each call may consume a command in the stream

    if (manualMode)       // If manual mode is on, rover is driven through RC transmitter
    {
#ifdef DEBUG_STATE
      if (SerialDEBUG) SerialDEBUG.print("M");
#endif // DEBUG_STATE

      lastManual = 1;

      if (state == S_PIVOT) // If pivoting, stop so we can regain control manually
      {
        pivotFinish();
        mowingReset();
        setState(S_MOWING);
      }

      manual(); // manual.ino, should convert settings for forwardSpeed and turnSpeed and call MoveRover()

      if (cmd == c_download) // Command received from RPi to download new mission data
      {
        cmd = c_empty;    // Flag command as taken

        stopRover();

        initDownload();   // Mower Init and Download

        resetWaypoint();  // Select and load Home Point

        setState(S_WAITING);
      }
      else if (cmd == c_advance) // Advance the current waypoint, whilst in Manual
      {
        cmd = c_empty; // Flag command as taken

        getNextWaypoint();  // Get the first wp from the waypointList to start the mission   ROOT: "course.ino
      }
      else if (cmd == c_downloadReady)
      {
        cmd = c_empty; // Flag command as taken

        if (SerialDEBUG) SerialDEBUG.println("<Arduino has been delivered operating parameters>");

        resetWaypoint(); // Select and load Home Point
      }
      else if (cmd == c_stop) // Switch rover state to WAIT on RPi command
      {
        cmd = c_empty; // Flag command as taken

        stopRover();
        setState(S_WAITING);
      }
      else if (cmd == c_start) // Command received from RPi to start new mission
      {
        cmd = c_empty; // Flag command as taken

        currentMeasureInit();

        if (numberWaypoints > 0) // Must have waypoints to start
        {
          resetWaypoint(); // Select and load Home Point
          mowingReset();
          setState(S_MOWING);
        }
        else
        {
          stopRover();
          setState(S_WAITING);
        }
      }

      updateWaypointEx(); // Update, so always current, either to Primary Waypoint, or Home Point
    }
    else  // In auto mode, control algorithm takes over the control
    {
#ifdef DEBUG_STATE
      if (SerialDEBUG) SerialDEBUG.print("A");
#endif // DEBUG_STATE

      if (lastManual) // Transition out of manual
      {
        double delta;

        if (SerialDEBUG) SerialDEBUG.println("<Transitioning from Manual>");

        updateWaypoint(); // Refresh geometry to get to the current waypoint, as it might have advanceded

        delta = GPSHeading  - targetCourse;
        if (delta > 180.0) delta -= 360.0;
        else if (delta < -180.0) delta += 360.0;

        if (fabs(delta) >= pivotHeadingTolerance) // Do we need to reorientate?
        {
          if (SerialDEBUG) SerialDEBUG.println("<Reorientation out of Manual>");
#if 0 // CRASHING RPi, PENDING REVIEW
          if (pivoting) pivotFinish();
          pivotReset();
          mowingReset();
          setState(S_PIVOT);
#endif
        }

        lastManual = 0;
      }


      switch(state)
      {
        //=================================
        case S_WAITING:               // Stop the rover & wait for RPi commands
        {
          if (lastStateEnc != state)
          {
            lastStateEnc = state;
            stopRover();
          }

          if (cmd == c_download)      // Command received from RPi to download new mission data
          {
            cmd = c_empty;   // Flag command as taken

            initDownload();  // Mower Init and Download

            resetWaypoint(); // Select and load Home Point
          }
          else if (cmd == c_downloadReady)
          {
            cmd = c_empty; // Flag command as taken

            if (SerialDEBUG) SerialDEBUG.println("<Arduino has been delivered operating parameters>");

            resetWaypoint(); // Select and load Home Point
          }
          else if (cmd == c_advance) // Advance the current waypoint, whilst waiting
          {
            cmd = c_empty; // Flag command as taken

            if (getNextWaypoint())  // Get the first wp from the waypointList to start the mission   ROOT: "course.ino
            {
              stopRover();
              setState(S_WAITING);
            }
            else
            {
              if (pivoting) pivotFinish();
              pivotReset();
              mowingReset();
              setState(S_PIVOT);
            }
          }
          else if (cmd == c_start) // Command received from RPi to start new mission
          {
            cmd = c_empty; // Flag command as taken

            currentMeasureInit();

            if (numberWaypoints > 0) // Must have waypoints to start
            {
              resetWaypoint(); // Select and load Home Point
              mowingReset();
              setState(S_MOWING);
            }
            else // No waypoints stop
            {
              stopRover();
              setState(S_WAITING);
            }
          }
          else
          {
            cmd = c_empty; // Flag command as taken

            if (SerialDEBUG) SerialDEBUG.println("<Command ignored in S_WAITING state>");
          }

          updateWaypointEx(); // Update, so always current, either to Primary Waypoint, or Home Point

          break; // S_WAITING
        }

        //=================================
        case S_MOWING:                // Mow from current wp to the upcoming wp in a straight path
        {
          if (lastStateEnc != state)
          {
            lastStateEnc = state;
            mowingReset();
          }

          if (cmd == c_pause)         // Switch rover state to HOLD on RPi command
          {
            cmd = c_empty;            // Flag command as taken
            savedState = state;       // S_MOWING
            setState(S_HOLD);
            break;
          }
          else if (cmd == c_stop)     // Switch rover state to WAIT on RPi command
          {
            cmd = c_empty;            // Flag command as taken
            savedState = state;       // S_MOWING
            stopRover();
            setState(S_WAITING);
            break;
          }
          else if (cmd == c_advance)  // Advance the current waypoint, whilst mowing
          {
            cmd = c_empty;            // Flag command as taken
            if (getNextWaypoint())    // Get the first wp from the waypointList to start the mission   ROOT: "course.ino
            {
              stopRover();
              setState(S_WAITING);
              break;
            }
            else
            {
              if (pivoting) pivotFinish();
              pivotReset();
              stopRover();
              setState(S_PIVOT);
            }
          }
          else
          {
            cmd = c_empty; // Flag command as taken

            if (SerialDEBUG) SerialDEBUG.println("<Command ignored in S_MOWING state, Stop to enter S_WAITING>");
          }

          if (state == S_MOWING)
            mower();                  // Mowing operation  ROOT: "aMower.ino"

          if (state == S_PIVOT)       // If transitioned
          {
             lastStateEnc = state;
             pivotReset();
             pivot();                 // Compute Pivot Motion
          }

          moveRover();                // Drive the vehicle in the right direction

          break; // S_MOWING
        }

        //=================================
        case S_PIVOT:                 // Direct the rover heading towards the upcoming wp
        {
          if (lastStateEnc != state)
          {
            lastStateEnc = state;
            pivotReset();
          }

          if (cmd == c_pause)         // Switch rover state to HOLD on RPi command
          {
            cmd = c_empty;            // Flag command as taken
            savedState = state;       // S_PIVOT
            setState(S_HOLD);
            break;
          }
          else if (cmd == c_stop)     // Switch rover state to WAIT on RPi command
          {
            cmd = c_empty;            // Flag command as taken
            savedState = state;       // S_PIVOT
            setState(S_WAITING);
            break;
          }
          else if (cmd == c_advance)  // Advance the current waypoint, whilst pivoting
          {
            cmd = c_empty;            // Flag command as taken
            pivotFinish();
            if (getNextWaypoint())    // Get the first wp from the waypointList to start the mission   ROOT: "course.ino
            {
              stopRover();
              setState(S_WAITING);
              break;
            }
            else
            {
              stopRover();
              pivotReset();
            }
          }
          else
          {
            cmd = c_empty; // Flag command as taken

            if (SerialDEBUG) SerialDEBUG.println("<Command ignored in S_PIVOT state, Stop to enter S_WAITING>");
          }

          pivot();

          if (state == S_MOWING)      // If transitioned
          {
            lastStateEnc = state;
            mowingReset();
            mower();                  // Compute Mower Motion
          }

          moveRover();                // Drive the vehicle in the right direction

          break; // S_PIVOT
        }

        //=================================
        case S_HOLD:                  // Pause the mission & wait for RPi command
        {
          if (lastStateEnc != state)
          {
            lastStateEnc = state;
            stopRover();
          }

          if (cmd == c_stop)          // Reset the mission & switch to WAIT state
          {
            cmd = c_empty;            // Flag command as taken
            savedState = state;       // S_HOLD

            stopRover();
            setState(S_WAITING);
          }
          else if ((cmd == c_start) && (savedState = S_MOWING)) // Proceed the mission & switch to MOWING state
          {
            cmd = c_empty;            // Flag command as taken
            savedState = state;       // S_HOLD

            mowingReset();
            setState(S_MOWING);
          }
          else if ((cmd == c_start) && (savedState = S_PIVOT))  // Proceed the mission & switch to PIVOT state
          {
            cmd = c_empty;            // Flag command as taken
            savedState = state;       // S_HOLD

            pivotReset();
            setState(S_PIVOT);
          }
          else if (cmd == c_advance)  // Advance the current waypoint, whilst holding
          {
            cmd = c_empty;            // Flag command as taken
            savedState = state;       // S_HOLD

            if (getNextWaypoint())    // Get the first wp from the waypointList to start the mission   ROOT: "course.ino
            {
              stopRover();
              setState(S_WAITING);
            }
          }
          else if (cmd == c_start)    // Not handled above, just start properly
          {
            cmd = c_empty;            // Flag command as taken
            savedState = state;       // S_HOLD

            currentMeasureInit();

            if (numberWaypoints > 0) // Must have waypoints to start
            {
              resetWaypoint(); // Select and load Home Point

              mowingReset();
              setState(S_MOWING);
            }
            else // No waypoints stop
            {
              stopRover();
              setState(S_WAITING);
            }
          }
          else
          {
            cmd = c_empty; // Flag command as taken

            if (SerialDEBUG) SerialDEBUG.println("<Command ignored in S_HOLD state, Stop to enter S_WAITING>");
          }

          updateWaypointEx(); // Update, so always current, either to Primary Waypoint, or Home Point

          break; // S_HOLD
        }

        //=================================
        case S_READY: // Mission Completed
        {
          if (lastStateEnc != state)
          {
            lastStateEnc = state;
            stopRover();              // Stop on transition to completion
          }

          if (cmd == c_stop)          // Switch rover state to WAIT on RPi command
          {
            cmd = c_empty;            // Flag command as taken
            savedState = state;       // S_READY

            stopRover();              // Explicitly Stop
            setState(S_WAITING);
          } // Adding below as seems appropriate to be able to do it here
          else if (cmd == c_download) // Command received from RPi to download new mission data
          {
            cmd = c_empty;   // Flag command as taken

            initDownload();  // Mower Init and Download

            resetWaypoint(); // Select and load Home Point
          }
          else if (cmd == c_downloadReady)
          {
            cmd = c_empty; // Flag command as taken

            if (SerialDEBUG) SerialDEBUG.println("<Arduino has been delivered operating parameters>");

            resetWaypoint(); // Select and load Home Point
          }
          else
          {
            cmd = c_empty; // Flag command as taken

            if (SerialDEBUG) SerialDEBUG.println("<Command ignored in S_READY state, Stop to enter S_WAITING>");
          }

          updateWaypointEx(); // Update, so always current, either to Primary Waypoint, or Home Point

          break; // S_READY
        }

        //=================================
        default:
        {
          if (SerialDEBUG) SerialDEBUG.println("<UNKNOWN STATE> Going Ready to Waiting");
          lastStateEnc = state;
          savedState = S_READY;
          setState(S_WAITING);
          break;
        }
      }
    } // (!manualMode)

    countLoop += 1;                       // Increment loop counter only at 100 msec intervals

    loopTime = millis() - currentMillis;  // Compute time on task

    // Put the reporting at end of loop
    reporting(currentMillis);             // Send mission log data to RPi every control loop iteration (100 msec)

    slackTime = 0;                        // Zero idle time count
  }
  else // Idle, not on 100/200 task
  {
     slackTime++; // Count spare 1ms loop/ticks outside of primary task
     //delay(1); // 1 ms, to keep units meaningful, avoid delay(), jams serial
     while((millis() - currentMillis) < 1) // Idle Task, Spend 1ms here to make slackTime units consitent
     {
        serialTask(); // Keep flushing / processing data
     }
    // Adding here for testing, should only fire at thresholds
    reporting(currentMillis);  // Send mission log data to RPi every control loop iteration (100 msec)
  }
} // loop()

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
