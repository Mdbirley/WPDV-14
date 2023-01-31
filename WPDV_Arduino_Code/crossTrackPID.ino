// Cross Track PID Code

// From aDefinitions.h, should perhaps localize here so it's far less clumsy..

//double turnPIDSetpoint, turnPIDInput, turnPIDOutput;
//float turnKp = 2.0;
//float turnKi = 0.5;
//float turnKd = 1.0;
//PID turnPID(&turnPIDInput, &turnPIDOutput, &turnPIDSetpoint, turnKp, turnKi, turnKd, DIRECT); //Create PID object (TURNSPEED)

// dt here is 125 ms (8Hz) or 200 ms (5Hz) based on intervalMain for loop()

double crossTrackPID(double crossTrackDistance, double headingError)
{
  // Compute turnSpeed
  
  // turnPID.SetMode(MANUAL / AUTOMATIC);
  // turnPID.Compute();
  // turnSpeed = turnPIDOutput;
  
  return(turnSpeed);  
}
