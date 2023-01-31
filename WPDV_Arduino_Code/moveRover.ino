// Functions in this file...
//  [1]initRover()        Initialize all motor driver settings & connections to arduino
//  [2]moveRover()        Passes output actuator commands for a selected chassis
//  [3]stopRover()        Stops rover actuators
//  [5]moveModelRover()
//  [6]moveMainRover()
//  [7]resetEncoders()      Zero out wheel encoders readings
//  [8]moveZeroTurnRover()
//  [9]moveAckermanRover()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SERVO_LEFT_PIN	10
#define SERVO_RIGHT_PIN	11

// Create left and rights servo objects that are used in Autonomous Mode
static Servo myservoLeft;  // Digital Pin on Arduino Due D10
static Servo myservoRight; //                            D11

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void initServos(void) // Actuator Outputs
{
#ifdef USE_SERVO_ACTUATORS
  myservoLeft.attach(SERVO_LEFT_PIN);   // Left servo output pin
  myservoRight.attach(SERVO_RIGHT_PIN);	// Right servo output pin
#endif // USE_SERVO_ACTUATORS
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void writeServos(int left, int right)
{
  myservoLeft.write(left); // 0..180, 90 center
  myservoRight.write(right);

  forwardSpeedLeft  = left; // Save these into log file
  forwardSpeedRight = right;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initRover(void) // Initialize all motor driver settings & connections to arduino
{
  if (chassis == modelChassis) // Making sure the modelChassis motors are set to Zero
  {
#ifdef SerialMOTOR
    motor.drive(0);                                   // Sabertooth commands
    motor.turn(0);                                    // Sabertooth commands
#endif // SerialMOTOR
  }
  else if (chassis == zeroturnChassis) // Setup pin connections for the ZeroTurn Servos/Switches
  {
    initTransmitter();  // Radio Control Transmitter Inputs
    initServos();       // Actuator Outputs
    ztEncoderInit();    // Initialize Zero-Turn Encoders
  }
  else if (chassis == ackermanChassis) // Setup pin connections for the Ackerman Servos/Switches
  {
    initTransmitter();  // Radio Control Transmitter Inputs
    initServos();       // Actuator Outputs
  }
  else if (chassis == mainChassis) // Setup pin connections for the Electric Servos/Switches
  {
    initTransmitter();  // Radio Control Transmitter Inputs
	}

  resetEncoders();      // Zero all encoder saved readings
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moveRover(void)
{
//IMU300  GPSFixQuality = atoi(fixQual1.value());           // read GPS quality

  if ((manualMode) || // Manual Mode Must **ALWAYS** WORK
    (((fixQualityEnable == 0) || (GPSFixQuality == 4 )) &&
      ((polygonEnable == 0) || (pointInPolygon(currentLong, currentLat)))) )
  { // Move only if enabled test GPS quality and inside the Geofence
    if (chassis == modelChassis)
    {
      moveModelRover(forwardSpeed, turnSpeed);
    }
    else if (chassis == mainChassis) // RoboteQ - Electric UK
    {
      moveMainRover(forwardSpeed, turnSpeed);
    }
    else if (chassis == zeroturnChassis) // Linear Actuators - Gas USA
    {
      moveZeroTurnRover(forwardSpeed, turnSpeed);
    }
    else if (chassis == ackermanChassis)
    {
      moveAckermanRover(forwardSpeed, turnSpeed);
    }
  }
  else // Out of Geofence or fixQuality not ok -> stop motor
  {
    dPrintLn("*** Emergency Stop ***");
    stopRover();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stopRover(void)
{
  forwardSpeed = 0;
  turnSpeed = 0;

  if (chassis == modelChassis)
  {
    moveModelRover(forwardSpeed, turnSpeed);
  }
  else if (chassis == mainChassis)
  {
    moveMainRover(forwardSpeed, turnSpeed);
  }
  else if (chassis == zeroturnChassis)
  {
    moveZeroTurnRover(forwardSpeed, turnSpeed);
  }
  else if (chassis == ackermanChassis)
  {
    moveAckermanRover(forwardSpeed, turnSpeed);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void moveModelRover(double forwardsp, double turnsp)
{
#ifdef SerialMOTOR
  motor.drive((int)(forwardsp * 10.0));
  motor.turn((int)(turnsp * 10.0));
#endif // SerialMOTOR
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Originally, and perhaps again
//  CHANNEL 1 in Mixed is the FWB/BWK Channel. Forward (1000),stopped is 0 Backward is (-1000)
//  CHANNEL 2 in Mixed is the TURN Channel.
//  Left turn for negative values(-1000), Right turn for positive values(+1000) and Straight(0)
// We've been using Tank Mode at the Roboteq for several years, adapting back 7-Jan-2022

static void moveMainRover(double forwardsp, double turnsp) // RoboteQ
{
//  int fwdSpeedLeft   = (int)(-(forwardsp + turnsp) * 10.0); // Perhaps remove the negation here?
//  int fwdSpeedRight  = (int)( (forwardsp - turnsp) * 10.0);
// ENGR:CT 18-Mar-2020 Need to revisit this formula as +/-100 on forwardsp and turnsp may result in values upto +/-2000, exceeding the +/-1000 expectations


// ENGR:CT 24-Jan-2021 New Algo
  int fwdSpeedLeft, fwdSpeedRight;

  if (state == S_PIVOT) // Rotate on the spot
  {
    if (turnsp > 0.0) // Right
    {
      fwdSpeedLeft  =  (int)(fabs(turnsp) * 10.0);
      fwdSpeedRight = -(int)(fabs(turnsp) * 10.0);
    }
    else // Left
    {
      fwdSpeedLeft  = -(int)(fabs(turnsp) * 10.0);
      fwdSpeedRight =  (int)(fabs(turnsp) * 10.0);
    }
  }
  else // Mowing
  {
    fwdSpeedLeft = fwdSpeedRight = (int)(forwardsp * 10.0);

    if (turnsp > 0.0) // Right
      fwdSpeedLeft  += (int)(fabs(turnsp) * 10.0);
    else
      fwdSpeedRight += (int)(fabs(turnsp) * 10.0);

    // Limit to prevent stall in hard turn
    if (fwdSpeedLeft > 900) fwdSpeedLeft = 900;
    else if (fwdSpeedLeft < -900) fwdSpeedLeft = -900;
    if (fwdSpeedRight > 900) fwdSpeedRight = 900;
    else if (fwdSpeedRight < -900) fwdSpeedRight = -900;
  }

#if 1
#ifdef SerialMOTOR
{
	static unsigned long motorQuery = millis();

	if ((millis() - motorQuery) > intervalReporting) // Perhaps Once a second, not too frequently
	{
	   motorQuery = millis();

#if 1
  SerialMOTOR.println("?C 1"); // RoboteQ Query Encoder, Motor#1
  SerialMOTOR.println("?C 2"); //  Motor#2
#else // Max reports this form breaks query
  SerialMOTOR.println("?C 1_?C 2 "); // RoboteQ Query Encoder, Motor#1 and Motor#2
#endif

	} // if
}
#endif // SerialMOTOR
#endif

  forwardSpeedRight = fwdSpeedRight; // For reporting purposes
  forwardSpeedLeft = fwdSpeedLeft;

if ((forwardSpeedRight < -1000) || (forwardSpeedRight > 1000)) // Range Check Right
{
  if (SerialDEBUG) SerialDEBUG.println("[RBQ] RANGE RIGHT!!");
  forwardSpeedRight = forwardSpeedLeft = 0; // STOP FOR SAFETY
}

if ((forwardSpeedLeft < -1000) || (forwardSpeedLeft > 1000)) // Range Check Left
{
  if (SerialDEBUG) SerialDEBUG.println("[RBQ] RANGE LEFT!!");
  forwardSpeedRight = forwardSpeedLeft = 0; // STOP FOR SAFETY
}

// 24-Jan-2021 Left Motor 1 (Encoder 1), Right Motor 2 (Encoder 2)
//  prior wiring/configuration may be different - BE AWARE!!

// 07-Jan-2022 Max wants mixed mode servo input, entire code above will need fixing..
//  or migrate to S commands to set MOTOR (TANK), rather than SERVO

#ifdef SerialMOTOR
#ifdef ROBOTEQ_MIXED // SPEED/TURN SERVO INPUTS

#define RATIO_FWD 10.0
#define RATIO_TURN 2.5  // 10 Too High, Oscillates

  if (manualMode)
  {
    forwardSpeedLeft  = 0; // For Safety
    forwardSpeedRight = 0;
  }
  else
  {
    forwardSpeedLeft  = (int)(forwardsp * RATIO_FWD); // For reporting purposes
    forwardSpeedRight = (int)(turnsp * RATIO_TURN);
  }

  SerialMOTOR.print("!G 1 "); // Proxy for Manual Servo Input
	SerialMOTOR.println((int)forwardSpeedLeft); // FWD

  SerialMOTOR.print("!G 2 ");
	SerialMOTOR.println((int)forwardSpeedRight); // TURN

// if we want RPM control, need to use !S x +/-xxx commands

#else // TANK SERVO INPUTS

  SerialMOTOR.print("!G 1 ");
  SerialMOTOR.println(fwdSpeedLeft); // You'd need to negate one of these, see above

  SerialMOTOR.print("!G 2 ");
  SerialMOTOR.println(fwdSpeedRight);

#endif // ROBOTEQ_MIXED

  SerialMOTOR.flush();

#endif // SerialMOTOR

#ifdef DEBUG_ROBOTEQ
	if (SerialDEBUG)
	{
		SerialDEBUG.print("DBG:ROBOTEQ FWD:");
		SerialDEBUG.print(forwardsp, DEC);
		SerialDEBUG.print(" TRN:");
		SerialDEBUG.print(turnsp,	DEC);
		SerialDEBUG.print(" L:");
		SerialDEBUG.print(forwardSpeedLeft,	DEC);
		SerialDEBUG.print(" R:");
		SerialDEBUG.println(forwardSpeedRight, DEC);
	}
#endif // DEBUG_ROBOTEQ

#if 0 // Max Reports this might be making rover wander, disable for now
#ifdef SerialMOTOR
  { // Cycle thru status reports/request
    static int rbq = 0;
    switch(rbq++)
    {
      case 0 : SerialMOTOR.println("?M 1_?M 2 "); break; // Motor command
      case 1 : SerialMOTOR.println("?P 1_?P 2 "); break; // Power
      case 2 : SerialMOTOR.println("?A 1_?A 2 "); break; // Motor Amps
      case 3 : SerialMOTOR.println("?V 1_?V 2 "); break; // Battery Volts
      case 4 : SerialMOTOR.println("?E 1_?E 2 "); break; // Feedback Error
      case 5 : SerialMOTOR.println("?F 1_?F 2 "); break; // Fault
      case 6 : SerialMOTOR.println("?FF_?FM "); // Fault Flags / Motor Status
      default : rbq = 0;
    }
  }
#endif // SerialMOTOR
#endif

  //SerialMOTOR.println("?M 1_?P 1_?A 1_?V 1_?V 2_?F 1_?FF_?FM"); // From George : Motor command, Power, Motor Amps, Battery Volts, Internal volts, Feedback, Fault flags, Motor status
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void resetEncoders(void)
{
  if (chassis == mainChassis) // RoboteQ
  {
#ifdef SerialMOTOR
    SerialMOTOR.println("!C 1 0"); // Clear Counter Motor#1
    SerialMOTOR.println("!C 2 0"); //  Motor#2
#endif // SerialMOTOR
  }
  else if (chassis == zeroturnChassis) // USA, Cog Sensor
  {
    encoderLeft = 0;
    encoderRight = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Use servo.writeMicroseconds()
// (2400.0 - 544.0) from MAX/MIN_PULSE_WIDTH

// https://www.servocity.com/hdls-2-50-12v
// Linear servo 1.96" travel, signal 1ms to 2ms
//
// 1000.0   44.22
// 1500.0   92.72
// 2000.0  141.21

static int compServoUs(double servo_deg)
{
  double servo_us = 544.0 + (servo_deg * (1856.0 / 180.0)); // 0..180 -> 544us to 2400us
  return((int)servo_us);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void ztMode3(double forwardsp, double turnsp)
{
  double leftservo, rightservo; // could be float
  double turn = turnsp * steeringSensitivity;

  if ((forwardsp == 0) && (turnsp != 0)) // Pivot
  {
    double turnpivot = min(max(fabs(turn), 0.0), 100.0); // 0 .. 100

    if (turnsp > 0) { // RIGHT
        leftservo  = +turnpivot;
        rightservo = -turnpivot;
    } else  { // LEFT
        leftservo  = -turnpivot;
        rightservo = +turnpivot;
    }
  } else { // Forward, with braking
    leftservo  = -((100.0 - max(+turn, 0.0)) * (double)forwardsp / 100.0); //range -100 to 100 %
    rightservo = -((100.0 - max(-turn, 0.0)) * (double)forwardsp / 100.0); //range -100 to 100 %
  }

  forwardSpeedLeft  = (int)(leftservo  * (45.0 / 100.0)) + servoLeftValueMiddle; // map() equivalent keeping intermediate precision
  forwardSpeedRight = (int)(rightservo * (45.0 / 100.0)) + servoRightValueMiddle; // +/-100 -> +/- 45 relative to center

  writeServos(forwardSpeedLeft, forwardSpeedRight); // 0..180, 90 center
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void ztMode4(double forwardsp, double turnsp)
{
  double leftservo, rightservo; // could be float
  double turn = turnsp * steeringSensitivity * 0.25; // Scale it down 1/4 th

  if ((forwardsp == 0) && (turnsp != 0)) // Pivot
  {
    double turnpivot = min(max(fabs(turn), 0.0), 100.0); // 0 .. 100
    if (turnsp > 0) { // RIGHT
      leftservo  = +turnpivot;
      rightservo = -turnpivot;
    } else  { // LEFT
      leftservo  = -turnpivot;
      rightservo = +turnpivot;
    }
  } else { // Forward, with braking
    leftservo  = -((100.0 - max(+turn, 0.0)) * (double)forwardsp / 100.0); //range -100 to 100 %
    rightservo = -((100.0 - max(-turn, 0.0)) * (double)forwardsp / 100.0); //range -100 to 100 %
  }

  forwardSpeedLeft  = compServoUs((leftservo  * (45.0 / 100.0)) + (double)servoLeftValueMiddle ); // map() equivalent keeping intermediate precision
  forwardSpeedRight = compServoUs((rightservo * (45.0 / 100.0)) + (double)servoRightValueMiddle); // +/-100 -> +/- 45 relative to center

  forwardSpeedLeft  = min(max(1000, forwardSpeedLeft ), 2000); // 1000-2000us Linear Servo limit from docs
  forwardSpeedRight = min(max(1000, forwardSpeedRight), 2000);

  myservoLeft.writeMicroseconds(forwardSpeedLeft); // 544..2400 us, 1472 us center
  myservoRight.writeMicroseconds(forwardSpeedRight);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void ztMode5(double fwdspeed, double trnspeed) // +RIGHT -LEFT
{
  static const int  leftTable[] = { 49,51,53,55,57,59,61,63,65,67,69,71,73,75,77,79,81,83,85, 88,88,88, 93,95,97,99,101,103,105,107,109,111,113,115,117,119,121,123,125,127,129 };
  static const int rightTable[] = { 49,51,53,55,57,59,61,63,65,67,69,71,73,75,77,79,81,83,85, 92,92,92, 93,95,97,99,101,103,105,107,109,111,113,115,117,119,121,123,125,127,129 };
  double fwd = max(min(fwdspeed, 100.0), -100.0); // ensure in scope
  double trn = max(min(trnspeed, 100.0), -100.0); // ensure in scope
  int leftservo;
  int rightservo;

  if ((fwd == 0.0) && (trn != 0.0)) // PIVOT
  {
    double trn = max(min(trnspeed * 1.5, 100.0), -100.0); // ensure in scope
    int idxl = (int)((100.0 + trn) /  5.0);
    int idxr = (int)((100.0 - trn) /  5.0);

    leftservo  =  leftTable[idxl];
    rightservo = rightTable[idxr];
  }
  else
  {
    int idx = (int)((100.0 - fwd) / 5.0);

    leftservo  =  leftTable[idx];
    rightservo = rightTable[idx];

    // +RIGHT -LEFT
    //  Left increases Right Wheel, Right increases Left Wheel

    if (trn >= 0.0) // +RIGHT
    {
      int lft = (int)(+trnspeed / 5.0);
      leftservo -= lft;
    }
    else // -LEFT
    {
      int rgt = (int)(-trnspeed / 5.0);
      rightservo -= rgt;
    }
  }

  leftservo  = max(min( leftservo, 135), 45); // ensure within physical limits
  rightservo = max(min(rightservo, 135), 45);

  writeServos(leftservo, rightservo);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void moveZeroTurnRover(double forwardsp, double turnsp)
{
  int leftServoOutputMapped;  // output mapped to 0-180 degrees
  int rightServoOutputMapped; // output mapped to 0-180 degrees
  int leftServoOutput;
  int rightServoOutput;

#ifdef USE_ROLLING_ROAD
if (manualMode == 2) // Rolling Road
{
  forwardSpeedLeft  = readThrottle; // Direct Pass-thru
  forwardSpeedRight = readAileron;
  myservoLeft.writeMicroseconds(forwardSpeedLeft); // 544..2400 us, 1472 us center
  myservoRight.writeMicroseconds(forwardSpeedRight);
  return;
}
#endif // USE_ROLLING_ROAD

  if (ztDirection == 5) // Mapped model 4 degree
  {
    ztMode5(forwardsp, turnsp);
    return; // Leave Here
  } else if (ztDirection == 4) // (MICROSECOND) Reverse and Swap, NEGATIVE TURNSP for LEFT - Values passed as percentages
  {
    ztMode4(forwardsp, turnsp);
    return; // Leave Here
  } else if (ztDirection == 3) // (DEGREES) Reverse and Swap, NEGATIVE TURNSP for LEFT - Values passed as percentages
  {
    ztMode3(forwardsp, turnsp);
    return; // Leave Here
  } else if (ztDirection == 2) { // Reverse and Swap, NEGATIVE TURNSP for LEFT
    // Values passed as percentages
    if ((forwardsp == 0.0) && (turnsp != 0.0)) // Pivot
    {
      int turn = (int)(min(max(fabs(turnsp) * steeringSensitivity, 0.0), 100.0)); // 0 .. 100

      if (turnsp > 0.0) // RIGHT
      {
        leftServoOutput  = +turn;
        rightServoOutput = -turn;
      }
      else // LEFT
      {
        leftServoOutput  = -turn;
        rightServoOutput = +turn;
      }
    }
    else // Forward, with skid braking
    {
      // Considerations here, a) Manual Mode needs reverse, b) turning needs forward motion
      double fwdsp = forwardsp;
      double skidsp = turnsp * steeringSensitivity; // otherwise half as effective as rotate, with counter rotating motors
      if (fwdsp >= 0.0) fwdsp = min(max(fwdsp, fabs(skidsp)), 100.0);
      // Keep within scope, so we don't counter rotate, or blow out math below
      skidsp = max(skidsp, -100.0);
      skidsp = min(skidsp,  100.0);
      leftServoOutput  = (int)(-((100.0 - max(+skidsp, 0.0)) * forwardsp / 100.0)); // range -100 to 100 %
      rightServoOutput = (int)(-((100.0 - max(-skidsp, 0.0)) * forwardsp / 100.0)); // range -100 to 100 %

      // Keep both either stopped or forward, shouldn't need this as scoped earlier, but just in case
      // Broke Manual
      //leftServoOutput  = min(leftServoOutput,  0);
      //rightServoOutput = min(rightServoOutput, 0);
    }
  } else { // zero or one
    leftServoOutput  = (int)(((100.0 - max(-turnsp * steeringSensitivity, 0.0)) * forwardsp / 100.0)); //range -100 to 100 %
    rightServoOutput = (int)(((100.0 - max(+turnsp * steeringSensitivity, 0.0)) * forwardsp / 100.0)); //range -100 to 100 %

    if (ztDirection == 1) { // Reverse, Levers Back/Forth Sense
      leftServoOutput  = -leftServoOutput;
      rightServoOutput = -rightServoOutput;
    }
  }

  // Could move PID stuff here
  if (speedPIDEnable == 2)
  {
    leftServoOutput  = AdjustLeft(leftServoOutput);
    rightServoOutput = AdjustRight(rightServoOutput);
    speedPIDServo[0] = leftServoOutput; // For logging/reporting
    speedPIDServo[1] = rightServoOutput;
  }

if (0) // From Max hardcoding, nominal range 45 .. 90 .. 135 (ie 90 +/- 45) right determined to have more forward center stick
{
  if (leftServoOutput <= 0) {
    leftServoOutputMapped = map(leftServoOutput, -100, 0, 45, 90);
  }
  else {
    leftServoOutputMapped = map(leftServoOutput, 0, 100, 90, 135);
  }
  if (rightServoOutput <= 0) {
    rightServoOutputMapped = map(rightServoOutput, -100, 0, 45, 95);
  }
  else {
    rightServoOutputMapped = map(rightServoOutput, 0, 100, 95, 135);
  }
}
else // From configuration file, Max needs to update and verify parameter acceptance
{
    if (leftServoOutput < 0) {
      leftServoOutputMapped = map(leftServoOutput, -100, 0, servoLeftValueLeft, servoLeftValueMiddle);
    }
    else {
      leftServoOutputMapped = map(leftServoOutput, 0, 100, servoLeftValueMiddle, servoLeftValueRight);
    }
    if (rightServoOutput < 0) {
      rightServoOutputMapped = map(rightServoOutput, -100, 0, servoRightValueLeft, servoRightValueMiddle);
    }
    else {
      rightServoOutputMapped = map(rightServoOutput, 0, 100, servoRightValueMiddle, servoRightValueRight);
    }
} // From configuration file or hard coded

  leftServoOutputMapped  = min(max(45, leftServoOutputMapped ), 141); // Physical limits of Linear Servo Stroke 1-2ms
  rightServoOutputMapped = min(max(45, rightServoOutputMapped), 141); // Limits on HydroDrive end might be tighter than full-scale

  writeServos(leftServoOutputMapped, rightServoOutputMapped);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void moveAckermanRover(double forwardsp, double turnsp)
{
  int leftServoOutputMapped;  // output mapped to 0-180 degrees
  int rightServoOutputMapped; // output mapped to 0-180 degrees

  int leftServoOutput  = (int)(((100.0 - max(-turnsp * steeringSensitivity, 0.0)) * forwardsp / 100.0)); //range -100 to 100 %
  int rightServoOutput = (int)(((100.0 - max(+turnsp * steeringSensitivity, 0.0)) * forwardsp / 100.0)); //range -100 to 100 %

  // reverse
  if (ztDirection == 1) {
    leftServoOutput = -leftServoOutput;
    rightServoOutput = -rightServoOutput;
  }

  if (leftServoOutput <= 0) {
    leftServoOutputMapped = map(leftServoOutput, -100, 0, 50, 95);
  }
  else {
    leftServoOutputMapped = map(leftServoOutput, 0, 100, 95, 140);
  }
  if (rightServoOutput <= 0) {
    rightServoOutputMapped = map(rightServoOutput, -100, 0, 50, 95);
  }
  else {
    rightServoOutputMapped = map(rightServoOutput, 0, 100, 95, 140);
  }
  /* values not being passed correctly
    if (leftServoOutput < 0) {
      leftServoOutputMapped = map(leftServoOutput, -100, 0, servoLeftValueLeft, servoLeftValueMiddle);
    }
    else {
      leftServoOutputMapped = map(leftServoOutput, 0, 100, servoLeftValueMiddle, servoLeftValueRight);
    }
    if (rightServoOutput < 0) {
      rightServoOutputMapped = map(rightServoOutput, -100, 0, servoRightValueLeft, servoRightValueMiddle);
    }
    else {
      rightServoOutputMapped = map(rightServoOutput, 0, 100, servoRightValueMiddle, servoRightValueRight);
    }
  */

  writeServos(leftServoOutputMapped, rightServoOutputMapped);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

