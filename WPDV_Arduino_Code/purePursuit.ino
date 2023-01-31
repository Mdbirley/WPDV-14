//Functions in this file...
//[1]purePursuit()				Returns the course from current position to an imaginary point on the intended path at
//								a distance D5 from the current rover position
//[2]getLatLonByPointBrngDist()	Returns GPS coordinates for a position referenced to another known poistion at
//								some known distance
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// x1/y1  Previous Lat/Lon
// x2/y2  Target
// x3/y3  Current

double purePursuitOriginal(double x1, double y1, double x2, double y2, double x3, double y3, double D5) {
  double x4 = x2, y4 = y2; // Aim (Default Target)
  double D1 = GPS1.distanceBetween(x1, y1, x2, y2);  // Baseline
  double D2 = GPS1.distanceBetween(x2, y2, x3, y3);  // Distance to Target
  double D3 = GPS1.distanceBetween(x1, y1, x3, y3);  // Distance from Start

  double P = D1 + D2 + D3; // Perimeter

  double D4 = sqrt(P * (P - 2 * D3) * (P - 2 * D1) * (P - 2 * D2)) / (2 * D1); // Error?

  double Q4 = (D4 * D4);
  double Q5 = (D5 * D5);

  if (Q5 > Q4) // SQRT of negative is NaN
  {
    double D6 = sqrt(Q5 - Q4);
    double c2 = GPS1.courseTo(x1, y1, x2, y2); // Start to Target
    double c3 = GPS1.courseTo(x1, y1, x3, y3); // Start to Current
    double angle = c3 - c2;
    double D7 = fabs(D4 / tan(radians(angle))) + D6;

    if (D7 < D1) // Don't extend the aim point beyond the target
    {
      latlon ll = getLatLonByPointBrngDist(x1, y1, c2, D7); // Start Lat/Lon, Angle to Target
      x4 = ll.lat;
      y4 = ll.lon;
    }
  }

  aimPointLat = x4;
  aimPointLong = y4;

  return GPS1.courseTo(x3, y3, x4, y4); // Course Current to Aim
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MY_PI                   (3.1415926535898)               // ICD-200

#define MY_DEG_TO_RAD           (MY_PI / 180.0)
#define MY_RAD_TO_DEG           (180.0 / MY_PI)

#define METRES_IN_MILE          (1609.344)
#define KILOMETRES_IN_MILE      (METRES_IN_MILE / 1000.0)       // 1.609344
#define FEET_IN_MILE            (5280.0)
#define FEET_TO_METRES          (FEET_IN_MILE / METRES_IN_MILE) // 3.280839895

#if 0

#define EARTH_RADIUS            (6371000.0)
#define EARTH_CIRCUM_METRES     (EARTH_RADIUS * 2.0 * PI)
#define EARTH_CIRCUM_KILOMETRES (EARTH_CIRCUM_METRES / 1000.0)
#define EARTH_CIRCUM_MILES      (EARTH_CIRCUM_METRES / METRES_IN_MILE)
#define DEG_TO_METRES           (EARTH_CIRCUM_METRES / 360.0)

#else

#define SEMI_MAJOR              (6378137.0)                     // WGS84
#define EARTH_CIRCUM_METRES     (SEMI_MAJOR * 2.0 * PI)
#define EARTH_CIRCUM_KILOMETRES (EARTH_CIRCUM_METRES / 1000.0)
#define EARTH_CIRCUM_MILES      (EARTH_CIRCUM_METRES / METRES_IN_MILE)
#define DEG_TO_METRES           (EARTH_CIRCUM_METRES / 360.0)

#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double myRadians(double deg)
{
	return(deg * MY_DEG_TO_RAD);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double myDegrees(double rad)
{
	return(rad * MY_RAD_TO_DEG);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MY_TWO_PI (2.0 * MY_PI)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double myCourseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = myRadians(long2-long1);
  double rlat1 = myRadians(lat1);
  double rlat2 = myRadians(lat2);
  double a1 = sin(dlon) * cos(rlat2);
  double a2 = sin(rlat1) * cos(rlat2) * cos(dlon);
  a2 = cos(rlat1) * sin(rlat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += MY_TWO_PI;
  }
  return(myDegrees(a2));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double myDistanceBetween(double lat1, double long1, double lat2, double long2) // current, target
{
	double ns, ew;
	double baseline;

	ns = (lat2 - lat1) * DEG_TO_METRES; // POS NORTH, NEG SOUTH
	ew = (long2 - long1) * DEG_TO_METRES; // POS EAST, NEG WEST

	baseline = sqrt(ns*ns + ew*ew);

  return(baseline); // metres
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// x1/y1  Previous Lat/Lon
// x2/y2  Target
// x3/y3  Current

double purePursuit(double x1, double y1, double x2, double y2, double x3, double y3, double D5) {
  double x4 = x2, y4 = y2; // Aim (Default Target)
  double D1 = myDistanceBetween(x1, y1, x2, y2);  // Baseline
  double D2 = myDistanceBetween(x2, y2, x3, y3);  // Distance to Target
  double D3 = myDistanceBetween(x1, y1, x3, y3);  // Distance from Start

  double P = D1 + D2 + D3; // Perimeter

  double D4 = sqrt(P * (P - 2 * D3) * (P - 2 * D1) * (P - 2 * D2)) / (2 * D1); // Error?

  double Q4 = (D4 * D4);
  double Q5 = (D5 * D5);

  if (Q5 > Q4) // SQRT of negative is NaN
  {
    double D6 = sqrt(Q5 - Q4);
    double c2 = myCourseTo(x1, y1, x2, y2); // Start to Target
    double c3 = myCourseTo(x1, y1, x3, y3); // Start to Current
    double angle = c3 - c2;
    double D7 = fabs(D4 / tan(myRadians(angle))) + D6;

    if (D7 < D1) // Don't extend the aim point beyond the target
    {
      latlon ll = getLatLonByPointBrngDist(x1, y1, c2, D7); // Start Lat/Lon, Angle to Target
      x4 = ll.lat;
      y4 = ll.lon;
    }
  }

  aimPointLat = x4;
  aimPointLong = y4;

  return myCourseTo(x3, y3, x4, y4); // Course Current to Aim
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

latlon getLatLonByPointBrngDist(double lat1, double lon1, double brng, double d) {
  struct latlon result;
  double R = 6378.137; // radius of the Earth (km)
  d = d / 1000; // convert distance from metres to km
  brng = myRadians(brng); // convert bearing from degrees to radians

  lat1 = myRadians(lat1); // current lat point converted to radians
  lon1 = myRadians(lon1); // current long point converted to radians

  double lat2 = asin( sin(lat1) * cos(d / R) + cos(lat1) * sin(d / R) * cos(brng));
  double lon2 = lon1 + atan2(sin(brng) * sin(d / R) * cos(lat1), cos(d / R) - sin(lat1) * sin(lat2));

  result.lat = myDegrees(lat2);
  result.lon = myDegrees(lon2);
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void myAdjustLatLonByBearingDistance(double *lat, double *lon, double brng, double d)
{
  double lat1 = myRadians(*lat); // Convert latitude point to radians
  double lon1 = myRadians(*lon); // Convert longitude point to radians

  double sin_lat = sin(lat1);
  double cos_lat = cos(lat1);

	d *= (1.0 / SEMI_MAJOR); // (d / 1000) / 6378.137	Radius of Earth

  double sin_d = sin(d);
  double cos_d = cos(d);

  brng = myRadians(brng); // Convert bearing from degrees to radians

  double lat2 = asin( (sin_lat * cos_d) + (cos_lat * sin_d) * cos(brng));

// Could keep intermediate sin_lat2 value?

  double lon2 = lon1 + atan2(sin(brng) * (sin_d * cos_lat), cos_d - (sin_lat * sin(lat2)) );

  *lat = myDegrees(lat2);
  *lon = myDegrees(lon2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
