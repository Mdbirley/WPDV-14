//Functions in this file...
//[1]manual()         Manual mode to drive the rover directly from radio signals
//[2]readTransmitter()      Read radio transmitter commands
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef _VARIANT_GRAND_CENTRAL_M4_
#define pulseInLong pulseIn
#endif // _VARIANT_GRAND_CENTRAL_M4_

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void manual(void)
{
  // Map alternatively allow the transmitter to drive the servos

  forwardSpeed = map(readThrottle, 1000, 2000, -100, 100);
  forwardSpeed = max(min(forwardSpeed, 100.0), -100.0);

  turnSpeed =  map(readAileron, 1000, 2000, -100, 100);
  turnSpeed  = max(min(turnSpeed, 100.0), -100.0);

  moveRover();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define RC_AUX_MANUAL_PIN   2

#define RC_THROTTLE_PIN     8
#define RC_ELEVATOR_PIN     9

#define RC_KILL_SWITCH_PIN  6 // Not used in current builds
#define RC_PTO_SWITCH_PIN   7

#define RC_TIMEOUT_WINDOW   25000 // Should see within 20 ms

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initTransmitter(void) // Radio Control Transmitter Inputs
{
#ifdef USE_RC_TX_INPUTS

  pinMode(RC_AUX_MANUAL_PIN, INPUT); // Input from Aux2 Switch
  pinMode(RC_THROTTLE_PIN,   INPUT); // Input from the RC Channel Throttle (FORWARD/BACK - RIGHT WHEEL)
  pinMode(RC_ELEVATOR_PIN,   INPUT); // Input from the RC Channel Elevator (LEFT/RIGHT   - LEFT WHEEL)

// pinMode(RC_KILL_SWITCH_PIN, INPUT); // Input from KILL Switch (Grounds Coils)
// pinMode(RC_PTO_SWITCH_PIN,  INPUT); // Input from PTO  Switch (Deck Clutch)

#endif // USE_RC_TX_INPUTS
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Notes
//  Manual
//   Reads 1 absent signal
//   Reads 1096-1098 Automatic
//   Reads 1505
//   Reads 1914-1921 Manual

void readTransmitter(void)
{
#ifdef USE_RC_TX_INPUTS

  // Reads the radio inputs for testing Servos from radio; timeout by 5000 microseconds to prevent long time delays

  if (chassis == zeroturnChassis)
  {
    readRadioManualAuto =  pulseInLong(RC_AUX_MANUAL_PIN, HIGH, RC_TIMEOUT_WINDOW);

#ifdef USE_ROLLING_ROAD
    if ((readRadioManualAuto >= 1400) && (readRadioManualAuto <= 1600))// Center Position
    {
			manualMode = 2; // Rolling Road
    } else
#endif // USE_ROLLING_ROAD
		if (readRadioManualAuto > 0) // Negative for timeout error
		{
			if (readRadioManualAuto	>= 1200)
				manualMode = 1; // Manual
			else
				manualMode = 0; // Auto
		}
		else
		  manualMode = 1; // Manual

    if (manualMode)
    {
      readThrottle = pulseInLong(RC_THROTTLE_PIN, HIGH, RC_TIMEOUT_WINDOW);
      if (readThrottle < 0) readThrottle = 1500;

      readAileron  = pulseInLong(RC_ELEVATOR_PIN, HIGH, RC_TIMEOUT_WINDOW);
      if (readAileron < 0) readAileron = 1500;
    }

#ifdef DEBUG_RC // Debug RC input
    if (SerialDEBUG)
    {
      char str[64];
      sprintf(str,"RT:%d,%d,%d,%d",readRadioManualAuto,readThrottle,readAileron,manualMode);
      SerialDEBUG.println(str);
    }
#endif // DEBUG_RC
  } // (chassis == zeroturnChassis)
  else
  if (chassis == ackermanChassis)
  {
    readRadioManualAuto =  pulseInLong(RC_AUX_MANUAL_PIN, HIGH, RC_TIMEOUT_WINDOW);

    if (readRadioManualAuto >= 1200)
      manualMode = 1;
    else
      manualMode = 0;

    readThrottle = pulseInLong(RC_THROTTLE_PIN, HIGH, RC_TIMEOUT_WINDOW);
    readAileron  = pulseInLong(RC_ELEVATOR_PIN, HIGH, RC_TIMEOUT_WINDOW);

  } // (chassis == ackermanChassis)
  else
  if (chassis == mainChassis) // Electric (Y-Cable from Receiver to Roboteq)
  {
    readRadioManualAuto =  pulseInLong(RC_AUX_MANUAL_PIN, HIGH, RC_TIMEOUT_WINDOW);

		if (readRadioManualAuto > 0) // Negative for timeout error
		{
			if (readRadioManualAuto	>= 1200)
				manualMode = 1;
			else
				manualMode = 0;
		}
		else
			manualMode = 1; // Fail into Manual
  } // (chassis == mainChassis)
#endif // USE_RC_TX_INPUTS
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

