// Functions in this file...
//  [1]getCourseToWaypoint()    Returns the heading in degrees to the target  ROOT:"TinyGPS++.cpp"
//  [2]getDistanceToWaypoint()  Returns the distance to the next waypoint ROOT:"TinyGPS++.cpp"
//  [3]getNextWaypoint()      Update global variables with the values for upcoming waypoint
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 1 // No Offset
#define WAYPOINT_TRIM_LAT_NORTHING 0
#define WAYPOINT_TRIM_LON_EASTING  0
#else
#define WAYPOINT_TRIM_LAT_NORTHING (-0.0000074) // Parker Ranch South/West shift with respect to Mission Planner / Google Earth
#define WAYPOINT_TRIM_LON_EASTING  (-0.0000077) //  Quick Test, need these as variables
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double getCourseToWaypoint()	        // Returns the heading in degrees to the target  ROOT:"TinyGPS++.cpp"
{
  return GPS1.courseTo(currentLat, currentLong, targetLat, targetLong);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double getDistanceToWaypoint()        // Returns the distance to the next waypoint ROOT:"TinyGPS++.cpp"
{
  return GPS1.distanceBetween(currentLat, currentLong, targetLat, targetLong);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double getDistanceFromLastWaypoint()  // Returns the distance from the last waypoint ROOT:"TinyGPS++.cpp"
{
  return GPS1.distanceBetween(prevLat, prevLong, currentLat, currentLong);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updateWaypoint(void)
{
  targetDistance = GPS1.distanceBetween(currentLat, currentLong, targetLat, targetLong);
  targetCourse   = GPS1.courseTo(currentLat, currentLong, targetLat, targetLong);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updateWaypointEx(void)
{
  if ((waypointNumber == 0) || (waypointNumber > numberWaypoints)) // Use Home
  {
    targetLat  = waypointList[0].getLat()  + WAYPOINT_TRIM_LAT_NORTHING; // HOME
    targetLong = waypointList[0].getLong() + WAYPOINT_TRIM_LON_EASTING;
  }

  // TODO:REFACTOR THIS TO USE ONE SET OF VARIABLES
  distanceToTarget = targetDistance = GPS1.distanceBetween(currentLat, currentLong, targetLat, targetLong);
  targetHeading    = targetCourse   = GPS1.courseTo(currentLat, currentLong, targetLat, targetLong);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int getNextWaypoint(void) // Update global variables with the values for upcoming waypoint
{
  if (waypointNumber >= numberWaypoints)
  {
    waypointNumber = numberWaypoints + 1; // could go back to Home
    return(1);  // Last waypoint is reached and mission is finished
  }
  else
  {
    waypointNumber++; // Index

// Active waypoints 1 .. n, when 1 is home
//  Table is zero based

    targetLat  = waypointList[waypointNumber - 1].getLat()  + WAYPOINT_TRIM_LAT_NORTHING;
    targetLong = waypointList[waypointNumber - 1].getLong() + WAYPOINT_TRIM_LON_EASTING;

    if (polygonEnable && !pointInPolygon(targetLong, targetLat))
    {
      waypointNumber = numberWaypoints + 1;
      return(1); // FAIL
    }

    if ((targetLat == 0.0) && (targetLong == 0.0)) // Failure case
    {
      waypointNumber = numberWaypoints + 1;
      return(1); // FAIL
    }

    if (waypointNumber > 1)
    {
      prevLat  = waypointList[waypointNumber - 2].getLat()  + WAYPOINT_TRIM_LAT_NORTHING;
      prevLong = waypointList[waypointNumber - 2].getLong() + WAYPOINT_TRIM_LON_EASTING;
    }
    else // Use current location to determine route to first waypoint
    {
      prevLat  = currentLat;
      prevLong = currentLong;
    }

    targetDistance = GPS1.distanceBetween(prevLat, prevLong, targetLat, targetLong);
    targetCourse   = GPS1.courseTo(prevLat, prevLong, targetLat, targetLong);

    return(0); // Ok
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void backupWaypoint(void)
{
  if (waypointNumber > 1) // 1=Home
    waypointNumber--;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void resetWaypoint(void)
{
  waypointNumber = 0; // One is Home Point, next increments
  getNextWaypoint();  // Get the first wp from the waypointList to start the mission   ROOT: "course.ino
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
