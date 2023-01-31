// Limits turning based on min/max values, so stops turn when within a given angle

double calcDesiredTurn(void)
{
  double localHeadingError = headingError + 0.0; // 2.2 degrees Right to compensate for Left tracking
  // Trim 0.9 was not enough, 2.0 not sure as ghost fogged test
  
// This code uses the headingerror to calculate a turnspeed. 

// The parameters in the config file can be set to either increase or reduce the turning response in response to the headingerror.

  if (fabs(localHeadingError) < minHeadingError) // No action if headingerror is small
  {
    return(0.0);
  }
  else
  {
    // map error between minHeadingError and maxHeadingError to a value between startTurnspeedMinHEadingError and endTurnspeedMaxHEadingError
    double tempturn;
    if (localHeadingError > 0) {
      tempturn = ((localHeadingError - minHeadingError) * (endTurnspeedMaxHEadingError - startTurnspeedMinHEadingError)
                        / (maxHeadingError - minHeadingError) + startTurnspeedMinHEadingError);
    }
    else {
      tempturn = ((localHeadingError + minHeadingError) * (endTurnspeedMaxHEadingError - startTurnspeedMinHEadingError)
                        / (maxHeadingError - minHeadingError) - startTurnspeedMinHEadingError);
    }

    // returns the turnSpeed
    return(max(min(tempturn, endTurnspeedMaxHEadingError), -endTurnspeedMaxHEadingError));   
  }
}
// End calcDesiredTurn()*******************************************************
