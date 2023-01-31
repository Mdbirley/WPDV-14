//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions in this file...
//  [1]calcCrossTrackDist()		Return perpendicular distance from straight line between previous and next waypoint in cm
//  [2]calcHeadingDiff()			Determines if crossTrackDist is to the left or right side of the intended path
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double calcCrossTrackDist(void) // Return perpendicular distance from straight line between previous and next waypoint in cm
{						
  // For debugging/graphing zero cross track during pivot or speed ramp
//  if (pivoting || (forwardSpeed != normalForwardSpeed))
  if (pivoting) // Just Pivoting
    return(0.0);
  
  if (waypointNumber > 1)
  {
    double distance12; //distance from previous to next waypoint
    double distance13; //distance from previous waypoint to current position
    double distance23; //distance from current position to next waypoint
    double perimeter;  //perimeter from distance12/12/23
    double error;
    double heading13;  // bearing from start point to current position
    double heading12;  //bearing from start point to end point
    double headingDiff; //diffence betwee heading12 and 13
    
    distance12 = GPS1.distanceBetween(targetLat, targetLong, prevLat, prevLong);
    distance13 = GPS1.distanceBetween(currentLat, currentLong, prevLat, prevLong);
    distance23 = GPS1.distanceBetween(currentLat, currentLong, targetLat, targetLong);

    perimeter = distance12 + distance13 + distance23;

    error = perimeter * (perimeter - (2.0 * distance13)) * (perimeter - (2.0 * distance12)) * (perimeter - (2.0 * distance23));
    
    if (error < 0.0) // Very small, but less than zero, negative numbers won't square-root (floating point)
      return(999.0); // Math error type number so max can see
    //return(0.0);
    
    error = sqrt(error) / (2.0 * distance12) * 100.0; //cm

    heading13 = GPS1.courseTo(currentLat, currentLong, prevLat, prevLong);
    heading12 = GPS1.courseTo(targetLat, targetLong, prevLat, prevLong);
    headingDiff = calcHeadingDiff(heading12, heading13);

    if (headingDiff < 0.0) {
      return(error);
    }
    else {
      return(-error);
    }
  }
  else return(0.0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static double calcHeadingDiff(double initial, double final)		//Determines if crossTrackDist is to the left or right side of the intended path
{
  if ((initial > 360.0) || (initial < 0.0) || (final > 360.0) || (final < 0.0))
  {
    //throw some error, or perhaps normalize?
  }

  double diff = final - initial;
  double absDiff = fabs(diff);

  if (absDiff <= 180.0)
  {
    //Edit 1:27pm
    return absDiff == 180.0 ? absDiff : diff; // avoid -180 case
  }
  else if (final > initial)
  {
    return absDiff - 360.0;
  }

  else
  {
    return 360.0 - absDiff;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
