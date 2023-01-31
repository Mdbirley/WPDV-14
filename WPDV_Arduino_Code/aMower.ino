//Functions in this file...
//[1]mower()          Mow from current wp to the upcoming wp in a straight path
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// targetCourse and targetDistance are computed once at waypoint transition
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MOVE_AWAY_TRIG1       1001
#define MOVE_AWAY_TRIG2       1002
#define MOVE_AWAY_INFLECTION  1003
#define MOVE_AWAY_TRIG4       1004

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int SafeToMove(void)
{
  if (!manual) return(1); // Manual, always in control

  if ((fixQualityEnable == 0) && (GPSFixQuality > 0)) // GPS FIX
    return(1);

  if ((fixQualityEnable != 0) && (GPSFixQuality == 4) || (GPSFixQuality == 5)) // RTK FIXED/FLOAT
    return(1);

  return(0); // Unsafe to move or navigate
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void mower() // Mow from current wp to the upcoming wp in a straight path
{
  int forceMovingAwayPivot = 0; // Check if we have reached the waypoint and change state to pivot
  double distanceFromLastWaypoint = getDistanceFromLastWaypoint();

  distanceToTarget = getDistanceToWaypoint();     // Calculate distance to next waypoint  ROOT: "wayPoint.ino"

  // 24-Oct-2021 Implementation

  if (SafeToMove())
  {
    // Current computed distance vs initial distance to waypoint

    //if (distanceToTarget > (targetDistance + waypointTolerance)) // Gotten further from target

    if (distanceToTarget > (targetDistance + movingAwayTolerence)) // Gotten further from target
    {
     #if 1
     #ifdef SerialRADIO
      if (SerialRADIO) SerialRADIO.println("<Moving away from Target>");
     #endif // SerialRADIO
     #else
      forceMovingAwayPivot = 1;
      movingAwayPivotTriggered = MOVE_AWAY_TRIG1;
     #endif
    }

    // Keep shrinking distance as we approach

    minDistanceToTarget = min(minDistanceToTarget, distanceToTarget);

    if (distanceToTarget > (minDistanceToTarget + movingAwayTolerence)) // Gotten further from target
    {
     #if 1
     #ifdef SerialRADIO
       if (SerialRADIO) SerialRADIO.println("<Not closing on Target>");
     #endif // SerialRADIO
     //#else // 17-Jul-2022
      forceMovingAwayPivot = 1;
      movingAwayPivotTriggered = MOVE_AWAY_TRIG2;
     #endif
    }

    if (pivotCheckAngleOfDeparture(GPSHeading)) // Angle has gone obtuse
    {
     #if 1
     #ifdef SerialRADIO
       if (SerialRADIO) SerialRADIO.println("<Driving past Target, inflection>");
     #endif // SerialRADIO
     //#else //17-Jul-2022
      forceMovingAwayPivot = 1;
      movingAwayPivotTriggered = MOVE_AWAY_INFLECTION;
     #endif
    }
  } // if SafeToMove()


#if 0 // Doesn't end up doing anything
  if (GPSFixQuality == 4) // RTK FIXED ??
  {
    minDistanceToTarget = min(minDistanceToTarget, distanceToTarget);

    if ((state == S_MOWING) && ((forwardSpeed + 0.5) >= normalForwardSpeed) && // Try to catch overshoot rather than track-to-track
        (distanceToTarget > waypointTolerance)) // Identify run-away condition, force pivot to next
    {
      double he = targetHeading - GPSHeading; // Difference between current rover heading and course to the waypoint
      while(he < -180.0) he += 360.0;   // Correction for compass wrap
      while(he >  180.0) he -= 360.0;
      //25-Jan-2021 if (fabs(he) > 120.0) forceMovingAwayPivot = 1;  // tending to oscillate in +/-180 range as pulling past
    }
  } // (GPSFixQuality == 4)
#endif


  crossTrackDistance = calcCrossTrackDist(); // Calculate cross track distance  ROOT: "calcCrossTrackDist.ino"

  // Need to review if short vs long should be different

  if (targetDistance > 2.0) // Long rather than Short section
    headingError = calcHeadingAIMError(); // Calculate heading error with respect to Aim Point  ROOT: "calcHeadingError.ino"
  else
    headingError = calcHeadingSIMPLEError(); // Short, always shoot for the waypoint

  forwardSpeed = calcForwardSpeed(0); // Ramp up motor speed after being stopped  ROOT: "moveRover.ino"


#if 0 // Acceleration, we will take time delayed, but won't let it race-off
if (targetDistance > 2.0) // Long rather than Short section
  if (distanceFromLastWaypoint < 2.0) // increase speed 80% slope first 2m, from 20% -> 100%
  {
    double derate = ((distanceFromLastWaypoint / 2.0) * 0.8) + 0.2;
    derate = (double)normalForwardSpeed * derate;
    forwardSpeed = min(forwardSpeed, derate);
  }
#endif

  // Need to figure out how to manage speed on short jogs

//---------------------------------------------------------------------------

  // ***** Compute the turn to hit the line/waypoint *****

  if ((waypointNumber <= 1) || (distanceToTarget <= purePursuitD5)) // Revert to simpliest for first/close waypoint hit
    turnSpeed = calcDesiredTurn();  // Alternative P-controller (SIMPLE)    ROOT: "calcDesiredTurn.ino"
  else if (purePursuitPIDEnable) purePursuitPID();
  else if (crossTrackPIDEnable)  turnSpeed = crossTrackPID(crossTrackDistance, headingError); // ROOT: "crossTrackPIO.ino"
  else if (stanleyEnable)        turnSpeed = stanley();
  else turnSpeed = calcDesiredTurn(); // Alternative P-controller (SIMPLE)    ROOT: "calcDesiredTurn.ino" limit min/max turn

//---------------------------------------------------------------------------

  if (!SafeToMove()) // Unsafe
  {
    forwardSpeed = 0;
    turnSpeed = 0;
  }

//---------------------------------------------------------------------------

// Need to check units for targetDistance and distanceToTarget, would explain odd gremlins..
#if 0
{
  static int i =0;
  if (i == 0)
  {
    char str[64];
    sprintf(str, "Distances %lf  %lf units?",targetDistance,distanceToTarget); // both metres, checked, had reported in cm
    if (SerialDEBUG) SerialDEBUG.println(str);
  }
  i = (i + 1) % MAIN_LOOP_HZ;
}
#endif

//---------------------------------------------------------------------------

#if 1 // Enable deceleration code (motors will clunk)

#if 1
#define DTO (2.0) // 2.0 m
#define DTOPERC 80.0  // drop speed 80% slope last 2m, 100% -> 20%
#else // Now more slower..
#define DTO (2.5) // 2.5 m
#define DTOPERC 97.0  // drop speed 97% slope last 2.5m, 100% -> 3%
#endif

if (targetDistance > DTO) // Long rather than Short section
{
  if (distanceToTarget < DTO) // drop speed 80% slope last 2m, 100% -> 20%
  {
    double derate = ((distanceToTarget / DTO) * (DTOPERC / 100.0)) + ((100.0 - DTOPERC) / 100.0);
    if (derate > 1.0) derate = 1.0; // Don't exceed parity
    derate = (double)normalForwardSpeed * derate;
    forwardSpeed = min(forwardSpeed, derate); // Lesser of these
  }
}
else // Short jogs
{
  double derate = 0.40; // 40% of 55 currently, saves overshoot of immediate waypoint
  derate = (double)normalForwardSpeed * derate;
  forwardSpeed = min(forwardSpeed, derate); // Lesser of these
}
#endif // Deceleration

//---------------------------------------------------------------------------

#ifdef USE_SOMEFWDSPEED
  forwardSpeed = max(forwardSpeed, fabs(turnSpeed)); // ENGR:CT 19-Sep-2020 ensure some forward speed when turning
#endif // USE_SOMEFWDSPEED

//---------------------------------------------------------------------------

  if (forwardSpeed >= (normalForwardSpeed + 0.1)) // Sanity Check with flagging
    forwardSpeed = (normalForwardSpeed + 0.1);

//---------------------------------------------------------------------------

#if 1 // Disabled 25-Jan-2021, Enabled 17-Jul-2022
  if ((movingAwayPivot == 1) && ((GPSFixQuality == 4) || (GPSFixQuality == 5)) && // RTK Fixed or Float
      (distanceToTarget > (minDistanceToTarget + movingAwayTolerence)))
  {
    #ifdef SerialRADIO
       if (SerialRADIO) SerialRADIO.println("<Classing movingAwayPivot>");
    #endif // SerialRADIO

    if (!forceMovingAwayPivot) // Not already triggered 25-Sep-2022
    {
      forceMovingAwayPivot = 1;
      movingAwayPivotTriggered = MOVE_AWAY_TRIG4;
    }
  }
#endif

//---------------------------------------------------------------------------

  if ((distanceToTarget <= waypointTolerance) || (forceMovingAwayPivot))  // Jog to next Waypoint
  {
    minDistanceToTarget = 100000.0; // minimum distance to target (current waypoint) set to large value - Reset ENGR:CT 21-Sep-2020

    if ((movingAwayPivotTriggered == MOVE_AWAY_TRIG1) ||
        (movingAwayPivotTriggered == MOVE_AWAY_TRIG2))
      backupWaypoint(); // Force reorientation toward current waypoint ENGR:CT 25-Sep-2022

    if (getNextWaypoint()) // !!! Might be missing final one !!!
    {
      if (SerialDEBUG) SerialDEBUG.println("<Mission Completed>");
      cmd = c_empty;
      state = S_READY; // Mower has completed all the way points and now in in the READY state
    }
    else
    {
      minDistanceToTarget = targetDistance; // Set by getNextWaypoint()

      #ifdef _NEVER_ // Code that transitioned
      if ((state == S_MOWING) && (fabs(calcHeadingSIMPLEError()) < (double)pivotHeadingTolerance)) return; // If angle of transition is low, just roll-on
      #endif // _NEVER_

      cmd = c_empty;
      if (state != S_PIVOT) // Should only be here if S_MOWING
      {
        state = S_PIVOT; // transition to pivot
        pivotReset(); // force initialization of pivot
      }
    } // if/else

  } // if
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

