//Functions in this file...
//[1]pivot()      Pivot the rover upon reaching one way point towards the upcoming way point and changes
//                the state to S_MOWING
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int pivotInit = 0;               // used to initialize pivot operation
static double pivotStartHeadingError = 0.0;  // used to calculate turn direction based on headingError
static double pivotHeadingProgress;

static int pivotState = 0;
static int pivotCount = 0;
static int pivotThreshold = ((MAIN_LOOP_HZ + 1) / 2);
static int pivotTerminal  = MAIN_LOOP_HZ;

static double pivotFinalHeading = 0.0;
static int pivotFinalHeadingValid = 0;

static unsigned long pivotMillis;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pivotReset(void)
{
  resetEncoders();         // Zero out encoder values at starting Pivot
  pivotInit = 0;
  pivotFinalHeadingValid = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pivotFinish(void) // Terminate Pivot Operation
{
  pivotInit = 0; // Clear initialization flag so next entry starts
  pivoting  = 0; // Leave Pivot

  forwardSpeed = calcForwardSpeed(1);  // ensure ramp out of pivot/turn
  turnSpeed = 0.0;
  state = S_MOWING;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int pivotStallCheck(unsigned long testTime)
{
  unsigned long deltaTime = (millis() - pivotMillis);
  if (pivoting && (deltaTime > testTime))
    return(1);
  else
    return(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int pivotCheckAngleOfDeparture(double HeadingAngle)
{
  if (pivotFinalHeadingValid)
  {
    double delta = HeadingAngle - pivotFinalHeading;

    if (delta > 180.0) delta -= 360.0;
    else if (delta < -180.0) delta += 360.0;

    if (fabs(delta) >= 90.0) return(1); // Looks to gone past, angle gets bigger from here
  }

  return(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void pivotMain(int Count)
{
  // NEED TO REVISIT THIS ALL

  // ALWAYS GENERATES A POSITIVE NUMBER
  //calculate pivot speed depending on headingError
  //start at pivotTurnSpeedStart ramp up to pivotTurnSpeedMiddle and rampdown to pivotTurnSpeedEnd
  if (fabs(headingError) > 90.0)
  {
    //Experimental turnSpeed
    turnSpeed = pivotTurnSpeedBegin + ((pivotTurnSpeedMiddle - pivotTurnSpeedBegin) * (180.0 - fabs(headingError)) / 90.0);
  }
  else
  {
    //Experimental turnSpeed
    turnSpeed = pivotTurnSpeedEnd + ((pivotTurnSpeedMiddle - pivotTurnSpeedEnd) * fabs(headingError) / 90.0);
  }

// NEED TO CONVERT THIS TO DEGREES / SECOND
if (Count < (MAIN_LOOP_HZ * 1))
{
 turnSpeed = max(turnSpeed, pivotTurnSpeedBegin);
}
else
if (((Count > (MAIN_LOOP_HZ * 2)) && (pivotHeadingProgress < 2.0)))
{
  turnSpeed = pivotTurnSpeedMiddle;
}

  //calculate turndirection based on pivotMode
  //pivot mode 0 : pivot always to the right
  //pivot mode 1 : pivot always to the left
  //pivot mode 2 : pivot left on even waypoints and right on uneven waypoints
  //pivot mode 3 : pivot right on even waypoints and left on uneven waypoints
  //pivot mode 4 : pivot in the direction with the smallest headingError
  //pivot mode 5 : no pivot (return at start of method)

  // -LEFT  +RIGHT

  switch(pivotMode)
  {
    case 0:
      // pivot always to the right
      break;
    case 1:
      // pivot always to the left
      turnSpeed = -turnSpeed;
      break;
    case 2:
      //pivot left on even waypoints and right on uneven waypoints
      if ((waypointNumber % 2) == 0) turnSpeed = -turnSpeed; // LEFT
      break;
    case 3:
      //pivot right on even waypoints and left on uneven waypoints
      if ((waypointNumber % 2) == 1) turnSpeed = -turnSpeed; // LEFT
      break;
    case 4:
      //pivot in the direction with the smallest headingError
//      if (pivotStartHeadingError < 0)
      if (headingError < 0.0) // Needs Left Rotation, need current, starting value of no help if you've overshot
        turnSpeed = -turnSpeed; // LEFT
      break;
  }

#ifdef USE_SOMEFWDSPEED // Unhelpful when in closed loop, and motors counter-rotate
  forwardSpeed = max(forwardSpeed, fabs(turnSpeed * 0.333)); // 19-Sep-2020 ensure some forward speed when turning
#endif // USE_SOMEFWDSPEED
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pivot(void) // Primary Interface
{
  targetHeading = getCourseToWaypoint();     // Calculate heading to target from current rover GPS position
  headingError = targetHeading - GPSHeading; // Difference between current rover heading and course to the upcoming waypoint

  if (headingError < -180.0)  headingError += 360.0;    // Correction for compass wrap
  if (headingError >  180.0)  headingError -= 360.0;

  if (pivotInit == 0) // Initial Entry
  {
    if (fabs(headingError) <= pivotHeadingTolerance) // Already within tolerance
      pivotState = -1;
    else
      pivotState = 0;
    
    pivotCount = 0;

    pivotStartHeadingError = headingError; // Initial Turn at start/initiation of pivot

    // Not sure this is being used
    myPID.SetMode(MANUAL);
    PIDOutput = 0;

    //forwardSpeed = pivotForwardSpeed; // Nominally zero in ZT

    pivotInit = 1; // Completed Pivot Starting

    pivotMillis = millis(); // Start Time of Pivot

    pivoting = 1; // Actively Pivoting
  } // if (pivotInit)

  if (pivotState == -1) // Exit Pivot
  {
      pivotInit = 0; // Clear initialization flag so next entry starts
      pivoting  = 0; // Leave Pivot

      pivotFinalHeadingValid = 1;
      pivotFinalHeading = targetHeading;

      forwardSpeed = calcForwardSpeed(1);  // ensure ramp out of pivot/turn
      turnSpeed = 0.0;
      state = S_MOWING;
  }
  else if (pivotState == 0) // Initial Stop
  {
    unsigned long deltaTime = (millis() - pivotMillis);
    
    forwardSpeed = 0.0; // Stop Motion
    turnSpeed    = 0.0;
    
    if (deltaTime >= 1000) // Stop for at least a second
    {
      if (fabs(headingError) <= pivotHeadingTolerance) // Already within tolerance
        pivotState = -1; // Exit
      else
       pivotState++;
    }
    
    return;
  }
  else if (pivotState < pivotThreshold) // Initial Ramp Up
  {
    if (fabs(headingError) <= pivotHeadingTolerance) // Already within tolerance
      pivotState = -1; // Exit
    else
      pivotState++;
  }
  else if (pivotState == pivotThreshold)
  {
    pivotHeadingProgress = fabs(pivotStartHeadingError - headingError);

    if ((pivotMode == 5) || (fabs(headingError) <= pivotHeadingTolerance))
    {
      forwardSpeed = 0.0; // Stop Now
      turnSpeed    = 0.0;
      pivotState++; // Move to Ramp Down
      return;
    }

    pivotMain(pivotCount++); // Main Pivot Code
  }
  else if (pivotState > pivotThreshold) // Closing Ramp Down
  {
    forwardSpeed = 0.0; // Stop Rotation
    turnSpeed    = 0.0;

    if (pivotState >= pivotTerminal) // Dwell Complete
      pivotState = -1;
    else
      pivotState++;
    
    return;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
