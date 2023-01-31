//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Zero-Turn Encoder Implementation  w/Speed Control PID
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RED 5V
// WHT SIG 2K PULL-UP to 5V internally, 3K3 PULL-DOWN on DUE to get 3.1V
// BLK GND
//
// D4
// D5
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define STAMPCOUNT 4
#define PULSEPERREVOLUTION 18 // Eighteen Cog Gear
#define STOPPEDMS 2500 // More than 500ms assumed stopped

#define leftWheelEncoderSensorPin  4
#define rightWheelEncoderSensorPin  5

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int leftWheelEncoderTriggered = 0;
static int leftWheelEncoderSensed = 0;
static unsigned long leftWheelStamp[STAMPCOUNT];
static int leftWheelStampIndex = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int rightWheelEncoderTriggered = 0;
static int rightWheelEncoderSensed = 0;
static unsigned long rightWheelStamp[STAMPCOUNT];
static int rightWheelStampIndex = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static double leftSpeedOutput,  rightSpeedOutput; // Will be the driving value
static double leftSpeedRequest, rightSpeedRequest; // The speed desired

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Create speed PID object
static PID leftSpeedPID( &rpmLeft,  &leftSpeedOutput,  &leftSpeedRequest,  speedKp, speedKi, speedKd, DIRECT);
static PID rightSpeedPID(&rpmRight, &rightSpeedOutput, &rightSpeedRequest, speedKp, speedKi, speedKd, DIRECT);

long deltaTime = 200; // 5 Hz

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// In manual mode we should return the request, zero should reset

double AdjustLeft(double request)
{
  leftSpeedRequest = fabs(request); // Always Positive

  if (manualMode || (request == 0.0))
  {
    leftSpeedPID.SetMode(MANUAL);
    leftSpeedOutput = fabs(request);
    return(request);
  }

  leftSpeedPID.SetMode(AUTOMATIC);
  leftSpeedPID.Compute();	// returns true if updating

  leftSpeedOutput = min(leftSpeedOutput, 100); // For sanity keep these in check

  if (request >= 0.0) // Map sign of input
    return(leftSpeedOutput);
  else
    return(-leftSpeedOutput);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double AdjustRight(double request)
{
  rightSpeedRequest = fabs(request); // Always Positive

  if (manualMode || (request == 0.0))
  {
    rightSpeedPID.SetMode(MANUAL);
    rightSpeedOutput = rightSpeedRequest;
    return(request);
  }

  rightSpeedPID.SetMode(AUTOMATIC);
  rightSpeedPID.Compute();

  rightSpeedOutput = min(rightSpeedOutput, 100); // For sanity keep these in check

  if (request >= 0.0) // Map sign of input
    return(rightSpeedOutput);
  else
    return(-rightSpeedOutput);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ztEncoderInit(void)
{
  pinMode(leftWheelEncoderSensorPin,  INPUT);  // Input from the Left Encoder
  pinMode(rightWheelEncoderSensorPin, INPUT);  // Input from the Right Encoder

  leftSpeedPID.SetMode(MANUAL);
  leftSpeedPID.SetOutputLimits(0.0, 90.0); // range of servo, in auto adjusts output to fit
  leftSpeedPID.SetSampleTime(deltaTime); // ms (ratios ki/kd)

  rightSpeedPID.SetMode(MANUAL);
  rightSpeedPID.SetOutputLimits(0.0, 90.0); // range of servo, in auto adjusts output to fit
  rightSpeedPID.SetSampleTime(deltaTime); // ms (ratios ki/kd)

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ztEncoderUpdate(void)
{
  unsigned long currentMillis = millis(); // Entry time
  unsigned long timeDelta;

  leftWheelEncoderSensed  = digitalRead(leftWheelEncoderSensorPin);
  rightWheelEncoderSensed = digitalRead(rightWheelEncoderSensorPin);

  if ((leftWheelEncoderSensed == LOW) && (leftWheelEncoderTriggered == 0)) // First HIGH to LOW transition
  {
    leftWheelEncoderTriggered = 1;

    encoderLeft++; // Global

		timeDelta = currentMillis - leftWheelStamp[leftWheelStampIndex];
		leftWheelStamp[leftWheelStampIndex] = currentMillis;
    leftWheelStampIndex	= (leftWheelStampIndex + 1) % STAMPCOUNT;
		if (timeDelta >= STOPPEDMS) // in milliseconds
		  rpmLeft = 0.0;
		else
    	rpmLeft = 60.0 / (timeDelta * (1.0 / (double)STAMPCOUNT) * ((double)PULSEPERREVOLUTION / 1000.0));
   }
   else if(leftWheelEncoderSensed == HIGH)
   {
     leftWheelEncoderTriggered = 0;
   }

  if ((rightWheelEncoderSensed == LOW) && (rightWheelEncoderTriggered == 0))
  {
    rightWheelEncoderTriggered = 1;

    encoderRight++; // Global

		timeDelta = currentMillis - rightWheelStamp[rightWheelStampIndex];
		rightWheelStamp[rightWheelStampIndex] = currentMillis;
    rightWheelStampIndex	= (rightWheelStampIndex + 1) % STAMPCOUNT;
		if (timeDelta >= STOPPEDMS) // in milliseconds
		  rpmRight = 0.0;
		else
	    rpmRight = 60.0 / (timeDelta * (1.0 / (double)STAMPCOUNT) * ((double)PULSEPERREVOLUTION / 1000.0));
   }
   else if(rightWheelEncoderSensed == HIGH)
   {
     rightWheelEncoderTriggered = 0;
   }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
