/*Forward Speed calculation based on the RPI parametersetting.
   - If the mower is in rust position (forwardSpeed = 0) the speed will be ramped-up to the desired value
   at during the time : 'rampupTimeForwardSPeed'
   the parameter forwardSpeed is the input value for the motor drivers in the moveRover routines
   the forwardSpeed calculation can be a fixed value or the based on a PID speed controller:
   parameter SpeedPID enabled -> forwardSpeed is the output of the speedPID whixh tries to keep the
                                 speed at the desired 'speedSetpoint'.
   parameter SpeedPID disabled -> forwardSpeed is set at the fixed value 'normalForwardSpeed'

   The actual speed will be read from the GPS and stored in the parameter forwardSpeedGPS.
   If the forwardSpeedGPS is invalid (error on GPS reading) the forwardSpeed will be 'normalForwardSpeed'

*/

double calcForwardSpeed(int Init)// Ramp up motor speed after being stopped
{
  static unsigned long previousMillisRamp = 0;
  static int startRamp = 0;

  if (Init) // Insure this gets cleared at transitions (out of pivot)
  {
    resetEncoders();          // Zero out encoder values at starting Mowing
    initPurePursuitPID();     // Initialize ppPID values at starting Mowing

    forwardSpeed = 0.0;
    startRamp = 0;
    previousMillisRamp = millis();
    return(forwardSpeed);
  }

  if ((forwardSpeed == 0.0) && (startRamp == 0))
  {
    startRamp = 1;
    previousMillisRamp = millis();
    return(forwardSpeed);
  }

  if (startRamp == 1)
  {
    unsigned long speedRampupTime = millis() - previousMillisRamp;

    if (speedRampupTime < (rampupTimeForwardSpeed * 1000))
    {
      forwardSpeed = ((double)speedRampupTime / ((double)rampupTimeForwardSpeed * 1000.0)) * normalForwardSpeed; // ratio speed vs time

      if (forwardSpeed >= normalForwardSpeed) // Keep it constrained, and terminate ramping
      {
         startRamp = 0;
         forwardSpeed = normalForwardSpeed;
      }
       
      if (speedPIDEnable == 1)
      {
        speedPID.SetMode(MANUAL);
        speedPIDOutput = forwardSpeed;
      }
    }
    else // beyond ramp time
    {
      startRamp = 0;
      forwardSpeed = normalForwardSpeed;
    }
  }
  else // (startRamp != 1)
  { //if the speedPID is enabled and we have a valid speed measurement, the speed will controlled at the desired speed setpoint based on the actual GPS speed
    if ((speedPIDEnable == 1) && (forwardSpeedGPS != INVALID_GPS_SPEED))
    {
      speedPID.SetMode(AUTOMATIC);
      speedPID.Compute();
      forwardSpeed = speedPIDOutput;
#ifdef DEBUG_SPEED
      //SerialDEBUG.print(" SP:");
      //SerialDEBUG.print(speedSetpoint);
      //SerialDEBUG.print(" PV:");
      //SerialDEBUG.print(forwardSpeedGPS);
      //SerialDEBUG.print(" OP:");
      //SerialDEBUG.println(speedPIDOutput);
#endif
    }
    else // not invalid
    {
      forwardSpeed = normalForwardSpeed;
    }
  }

  return(forwardSpeed);
}
