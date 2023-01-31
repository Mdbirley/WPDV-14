//Functions in this file...
//[1]initPID()					Zero-out all errors for a stable start
//[2]purePursuitPID()			Correct rover heading to be always directing to the next AimPoint
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initPurePursuitPID() // Called whenever new waypoint or mower mode transition occurs
{
//  ppPID.SetOutputLimits(-100,100); // Range of turn +/-100
//  ppPID.SetMode(MANUAL); // Will reset itself when transitioned to AUTOMATIC

  p_headingError = 0;
  i_headingError = 0;
  d_headingError = 0;
}

void purePursuitPID() // Compute turnSpeed to reduce tracking error
{
  if (purePursuitPIDEnable == 1)
  {
    double local_headingError = headingError;

  // 28-Feb-2020 Disabling the two following code blocks had very deleterious effects

  if (1) // Use soft PID, always seems to go positive in an unhelpful way
  {
    if (fabs(local_headingError) >= minHeadingError) // Magnitude
    {
      if (local_headingError >= 0.0) // Sign
        local_headingError -= minHeadingError; // +POS
      else
        local_headingError += minHeadingError; // -NEG
    }
    else
      local_headingError = 0.0; // If below threshold, pull to zero (ie no action/response)

    if (fabs(local_headingError) >= maxHeadingError) // Max suggests +/- 5 degrees is typical range
    {
      if (local_headingError >= 0.0) // Sign
        local_headingError = +maxHeadingError; // +POS
      else
        local_headingError = -maxHeadingError; // -NEG
    }

    // PID Controller

if (1) {    // Change to zero to disable this behaviour - 28-Feb-2020 think this is critical
    if (((p_headingError > 0) && (local_headingError < 0)) || // Reset if direction of error has changed
        ((p_headingError < 0) && (local_headingError > 0)) )
      initPurePursuitPID();
}

//    i_headingError += local_headingError; // Integrate (infinite) 02-Feb-2021 this explodes at 50 Hz
    i_headingError += local_headingError / (double)MAIN_LOOP_HZ; // Integrate (infinite), lower dt
    d_headingError = local_headingError - p_headingError; // Differential
    p_headingError = local_headingError;

    turnSpeed = (purePursuitKp * p_headingError) + (purePursuitKi * i_headingError) + (purePursuitKd * d_headingError);
  }
//else // Use PID Library
//{
//  ppPID.SetMode(AUTOMATIC);
//  ppPIDInput = headingError; // double
//  ppPID.Compute(); // work PID magic ppPIDInput -> ppPIDOutput
//  turnSpeed = ppPIDOutput;   // double
//}

  } // (purePursuitPIDEnable == 1)
  else turnSpeed = headingError * 0.2;					// P-control with fixed gain if PID is disabled

//  if (pipeCrossTrackThreshold != 0.0) // Set to zero to disable
//  if ((waypointNumber > 1) && (fabs(crossTrackDistance) <= pipeCrossTrackThreshold)) // Within the Pipe (cm)
//  {
//    if (fabs(turnSpeed) > pipeTurnSpeedLimit) // (Magnitude) Constrain speed of approach within the Pipe
//    {
//      if (turnSpeed > 0.0) // (Sign)
//        turnSpeed = pipeTurnSpeedLimit;
//      else
//        turnSpeed = -pipeTurnSpeedLimit;
//    }
//  }

#define TURN_MAX 30.0 // 60.0 02-Feb-2021 tune down to prevent stall

  // Not sure of this, the range should be +/-100 and any physical constraining should be managed by the servo/acutator code
  if (turnSpeed >  TURN_MAX) turnSpeed =  TURN_MAX;					// Limit rover turnSpeed to [60, -60]
  if (turnSpeed < -TURN_MAX) turnSpeed = -TURN_MAX;
}
