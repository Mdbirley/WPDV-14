// Functions in this file...
//  [1]initStanley()  Zero-out all errors for a stable start
//  [2]stanley()      Correct rover heading to be always directing to the next AimPoint
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initStanley(void)
{
  p_errorStanley = 0;
  i_errorStanley = 0;
  d_errorStanley = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double stanley(void)
{
  // Think these are already precomputed
  headingError = calcHeadingError(); // Calculate heading error ROOT: "calcHeadingError.ino"
  crossTrackDistance = calcCrossTrackDist();  // Calculate cross track distance  ROOT: "calcCrossTrackDist.ino"

  if (stanleyEnable == 1)
  {
    // think we should already be in m/s, and shouldn't mod global copy anyway, cross track in cm
    //forwardSpeedGPS /= 3.6; // Convert ground speed from km/h to m/s
    stanleySecondTerm = crossTrackDistance / forwardSpeedGPS;
    stanleySecondTerm *= stanleyGain;
    stanleySecondTerm = atan(stanleySecondTerm);
    stanleySecondTerm = (stanleySecondTerm * 180.0) / 3.141592654;  // Convert from radians to degrees

    errorStanley = headingError + stanleySecondTerm; // steeringAngle = headingError + tan^-1(Ks * crossTrackDistance / forwardSpeed);

    if (errorStanley >=  200.0) errorStanley =  200.0; // Limit stanley output
    if (errorStanley <= -200.0) errorStanley = -200.0;

    i_errorStanley += errorStanley;
    d_errorStanley = errorStanley - p_errorStanley;
    p_errorStanley = errorStanley;

    turnSpeed = (stanleyKp * p_errorStanley) + (stanleyKi * i_errorStanley) + (stanleyKd * d_errorStanley);
  }
  else
    turnSpeed = headingError * 0.2;          // P-control with fixed gain if PID is disabled

  if (turnSpeed >  60.0) turnSpeed =  60.0;  // Limit rover turnSpeed to [60, -60]
  if (turnSpeed < -60.0) turnSpeed = -60.0;

  return(turnSpeed);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

