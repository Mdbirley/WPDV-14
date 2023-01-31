//Functions in this file...
//[1]pointInPolygon()			Used for the Geofence to determine if the current point is inside or outside the Geofence
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const uint8_t polySides = 7;                          // Geofence; 4 sides of polygon for Geofence. Can be N sides

// Huge Area encluding UK and USA
//float polyX[polySides] = { -2.0, -2.0, -3.0, -3.0  }; // E/W
//float polyY[polySides] = { 53.0, 52.0, 52.0, 53.0};   // N/S

// Only the Eastern Part of the Ranch in Texas
//double polyX[polySides] = { -98.742854, -98.742340, -98.743898, -98.744197}; // E/W
//double polyY[polySides] = {  30.394951,  30.394156,  30.393377,  30.394873}; // N/S

// Only the southern Part of the Ranch in Texas  next to house
//double polyX[polySides] = { -98.74472020, -98.743540,  -98.74447350, -98.74512790}; // E/W
//double polyY[polySides] = {  30.393001,    30.39345910, 30.39408840,  30.39410690}; // N/S

// Whole area around thehouse
double polyX[polySides] = { -98.74470680, -98.74355080, -98.74229010, -98.74284800, -98.74541760, -98.74495630, -98.74480610}; // E/W
double polyY[polySides] = {  30.39295710,  30.39342900,  30.39373670,  30.39501380,  30.39486580,  30.39366270,  30.39323240}; // N/S

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool pointInPolygon( double x, double y )
{
  int i, j = polySides - 1;
  bool oddNodes = false;

  for ( i = 0; i < polySides; i++ )
  {
    if ( (polyY[i] < y && polyY[j] >= y || polyY[j] < y && polyY[i] >= y) &&  (polyX[i] <= x || polyX[j] <= x) )
    {
      oddNodes ^= ( polyX[i] + (y - polyY[i]) / (polyY[j] - polyY[i]) * (polyX[j] - polyX[i]) < x );
    }

    j = i;
  }

  return oddNodes;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
