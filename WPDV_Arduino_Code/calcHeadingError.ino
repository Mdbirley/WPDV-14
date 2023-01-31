//Functions in this file...
//[1]calcHeadingError()			Calculate the rover heading error; whether it is the difference between
//								the current GPS heading and course to aim point
//								OR
//								the current GPS heading and course to way point
// Use double's consistently to maintain precision throughout computations
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// We restricted this during testing, as we wanted it to use GPS heading, but also compute the aim point
//  New routines below to make it clear what the controller is using, might migrate to other tabs/files later

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double calcHeadingError() {								//Calculate difference between current rover heading & heading to aim point

  targetHeading = getCourseToWaypoint();				//Calculate heading to target from current rover GPS position

if (0) // GPS ONLY
{
  headingError = targetHeading - GPSHeading;			//Difference between current rover heading and course to the first waypoint
  if ((waypointNumber > 1) && (waypointNumber <= numberWaypoints)) // Compute when possible for logging, don't use
    courseToAimPoint = purePursuit(prevLat, prevLong, targetLat, targetLong, currentLat, currentLong, purePursuitD5);
  else
    courseToAimPoint = 0;
}
else // INVOLVING AIM
{
  courseToAimPoint = 0;
  if ((waypointNumber > 1) && (waypointNumber <= numberWaypoints)) {

    if (distanceToTarget <= purePursuitD5) headingError = targetHeading - GPSHeading; //Skip aim point calculation when we are approaching the target
    else {
      courseToAimPoint = purePursuit(prevLat, prevLong, targetLat, targetLong, currentLat, currentLong, purePursuitD5);

      if ISNAN(courseToAimPoint) {						//Switch to target heading error if pure pursuit returns a NaN
        courseToAimPoint = 0;
        aimPointLat = 0;
        aimPointLong = 0;
        headingError = targetHeading - GPSHeading;		//Difference between current rover heading and course to the first waypoint
      }
      else {
        headingError = courseToAimPoint - GPSHeading;	//Difference between current rover heading and course to aim point
      }
    }
  }
  else {
    headingError = targetHeading - GPSHeading;			//Difference between current rover heading and course to the first waypoint
  }
} // GPS

  if (headingError < -180.0)	headingError += 360.0;		//Correction for compass wrap
  if (headingError > 180.0)	headingError -= 360.0;

  return(headingError);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// The above routine will be deprecated

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double calcHeadingSIMPLEError() // Calculate difference between current rover heading and heading to way point
{
  targetHeading = getCourseToWaypoint(); // Calculate heading to target from current rover GPS position

  headingError = targetHeading - GPSHeading; // Difference between current rover heading and course to the waypoint (Default)

  if ((waypointNumber > 1) && (waypointNumber <= numberWaypoints)) // Compute when possible for logging, don't use
    courseToAimPoint = purePursuit(prevLat, prevLong, targetLat, targetLong, currentLat, currentLong, purePursuitD5);
  else
    courseToAimPoint = 0;

  if (headingError < -180.0) headingError += 360.0;		// Correction for compass wrap
  if (headingError >  180.0) headingError -= 360.0;

  return(headingError);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double calcHeadingAIMError() // Calculate difference between current rover heading & heading to aim point or way point
{
  targetHeading = getCourseToWaypoint(); // Calculate heading to target from current rover GPS position

  headingError = targetHeading - GPSHeading; // Difference between current rover heading and course to the waypoint (Default)

  if ((waypointNumber > 1) && (waypointNumber <= numberWaypoints)) // Compute when possible for logging
  {
    courseToAimPoint = purePursuit(prevLat, prevLong, targetLat, targetLong, currentLat, currentLong, purePursuitD5); // Advance aim point along track

    if (!isnan(courseToAimPoint) && (distanceToTarget > purePursuitD5)) // If enough distance to still run
      headingError = courseToAimPoint - GPSHeading;	// Difference between current rover heading and course to aim point (If Usable)
  }
  else
    courseToAimPoint = 0; // Clear it so it is more obviously not set

  if (headingError < -180.0) headingError += 360.0;		// Correction for compass wrap
  if (headingError >  180.0) headingError -= 360.0;

  return(headingError);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
