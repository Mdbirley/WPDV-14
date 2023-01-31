// avoidance.ino  Code to manage obstacle avoidance will live here

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initAvoidance(void)
{
  // Place Holder
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void reportCloseObstacle(void)
{
  int obstacleCount = obstacleList.size();
  int i, j = -1;
  double obstacleNearest = 999999.99; // unitless squared

  for(i=0; i<obstacleCount; i++)
  {
    double delta = obstacleList[i].getDistanceCheap(currentLong, currentLat); // squared value, for magnitude
    if (obstacleNearest > delta)
    {
      obstacleNearest = delta;
      j = i;
    }
  }

  if ((j >= 0) && (obstacleNearest > 0))
  {
    char string[64];
    double delta = obstacleList[j].getDistanceAccurate(currentLong, currentLat);
    double bearing = obstacleList[j].getBearingAccurate(currentLong, currentLat);
    sprintf(string, " Near %d %.2lf m  %.2lf m  %.2lf\n", j+1, sqrt(obstacleNearest) * 111319.4908, delta, bearing );
#ifdef SerialDEBUG
    if (SerialDEBUG) SerialDEBUG.write(string);
#endif // SerialDEBUG
//#ifdef SerialRADIO
//    if (SerialRADIO) SerialRADIO.write(string);
//#endif // SerialRADIO

    obstacleIdentified = j + 1;
    obstacleDistance   = delta;
    obstacleBearing    = bearing;
		obstacleAngle      = bearing - GPSHeading; // From rover's perspective
    if (obstacleAngle < -180.0) obstacleAngle += 360.0;          // Correction for compass wrap
    if (obstacleAngle > 180.0)  obstacleAngle -= 360.0;
  }
  else
  {
    obstacleIdentified = -1;
    obstacleDistance   = 999999.99;
    obstacleBearing    = 0.0;
    obstacleAngle      = 0.0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
