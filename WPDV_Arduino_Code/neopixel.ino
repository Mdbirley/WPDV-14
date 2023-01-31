//****************************************************************************
// NeoPixel Support
//****************************************************************************

#ifdef USE_NEOPIXEL

//****************************************************************************

#define USE_NEOPIXEL_STICK

#ifdef USE_NEOPIXEL_STICK

#define NEOPIXEL_MANUAL   3
#define NEOPIXEL_PIVOTING 4
#define NEOPIXEL_GPSGO    5
#define NEOPIXEL_GPSPOS   6
#define NEOPIXEL_GPSDIR   7

#define NEOPIXEL_STICK_PIN 10 // D10

Adafruit_NeoPixel neopixel(8, NEOPIXEL_STICK_PIN, NEO_GRB + NEO_KHZ800);
#else // USE_NEOPIXEL_STICK
//Adafruit_NeoPixel neopixel(1, 88, NEO_GRB + NEO_KHZ800); // PC24
#endif // USE_NEOPIXEL_STICK

#define ALERTBAR_PIN 11 // D11

#ifdef CAMERA_DETECT_PIN
Adafruit_NeoPixel alertbar(8,  ALERTBAR_PIN, NEO_GRB + NEO_KHZ800);
#endif // CAMERA_DETECT_PIN

//****************************************************************************

void NeoPixelInit(void)
{
#ifdef USE_NEOPIXEL_STICK
  neopixel.begin();
  neopixel.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
  neopixel.setPixelColor(0, 0, 128, 0); // n R G B
  neopixel.show(); // Initialize all pixels to 'off'
#endif // USE_NEOPIXEL_STICK

#ifdef CAMERA_DETECT_PIN
  alertbar.begin();
  alertbar.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
  alertbar.setPixelColor(0, 0, 128, 0); // n R G B  light green
  alertbar.setPixelColor(1, 128, 0, 0); // n R G B  light red
  alertbar.setPixelColor(2, 0, 128, 0); // n R G B  light green
  alertbar.setPixelColor(3, 128, 0, 0); // n R G B  light red
  alertbar.setPixelColor(4, 0, 128, 0); // n R G B  light green
  alertbar.setPixelColor(5, 128, 0, 0); // n R G B  light red
  alertbar.setPixelColor(6, 0, 128, 0); // n R G B  light green
  alertbar.setPixelColor(7, 128, 0, 0); // n R G B  light red
  alertbar.show(); // Initialize all pixels to 'off'
#endif // CAMERA_DETECT_PIN
}

//****************************************************************************

void NeoPixelCycle(void)
{
#ifdef USE_NEOPIXEL_STICK
  static uint32_t led = 0;
  switch(led % 3)
  {
    case 0 : neopixel.setPixelColor(0, neopixel.Color(255,   0,   0)); break; // Red
    case 1 : neopixel.setPixelColor(0, neopixel.Color(  0, 255,   0)); break; // Green
    case 2 : neopixel.setPixelColor(0, neopixel.Color(  0,   0, 255)); break; // Blue
  }
  led++;
#endif // USE_NEOPIXEL_STICK

#ifdef USE_NEOPIXEL_STICK
  if (manualMode)
    neopixel.setPixelColor(NEOPIXEL_MANUAL, neopixel.Color(255,   0,   0)); // Red
  else
    neopixel.setPixelColor(NEOPIXEL_MANUAL, neopixel.Color(  0, 255,   0)); // Green

  if (pivoting)
    neopixel.setPixelColor(NEOPIXEL_PIVOTING, neopixel.Color(255,   0,   0)); // Red
  else
    neopixel.setPixelColor(NEOPIXEL_PIVOTING, neopixel.Color(  0, 255,   0)); // Green

   if ((GPSFixQuality == 4) && (GPSHeadingQuality == 4))
    neopixel.setPixelColor(NEOPIXEL_GPSGO, neopixel.Color(  0, 255,   0)); // Green - RTK BOTH GOOD
   else
    neopixel.setPixelColor(NEOPIXEL_GPSGO, neopixel.Color(255,   0,   0)); // Red

   if (GPSFixQuality == 4)
    neopixel.setPixelColor(NEOPIXEL_GPSPOS, neopixel.Color(  0, 255,   0)); // Green - RTK FIXED
   else if (GPSFixQuality == 5)
    neopixel.setPixelColor(NEOPIXEL_GPSPOS, neopixel.Color(  0,   0, 255)); // Blue  - RTK FLOAT
   else
    neopixel.setPixelColor(NEOPIXEL_GPSPOS, neopixel.Color(255,   0,   0)); // Red
      
   if (GPSHeadingQuality == 4)
    neopixel.setPixelColor(NEOPIXEL_GPSDIR, neopixel.Color(  0, 255,   0)); // Green - RTK FIXED
   else if (GPSHeadingQuality == 5)
    neopixel.setPixelColor(NEOPIXEL_GPSDIR, neopixel.Color(  0,   0, 255)); // Blue  - RTK FLOAT
   else
    neopixel.setPixelColor(NEOPIXEL_GPSDIR, neopixel.Color(255,   0,   0)); // Red

  neopixel.show();
#endif // USE_NEOPIXEL_STICK

#ifdef CAMERA_DETECT_PIN
{
  int i;
  int dude = digitalRead(CAMERA_DETECT_PIN) ? 1 : 0; // Person In The Way
      
  for(i=0; i<8; i++)
    if (dude)
      alertbar.setPixelColor(i, alertbar.Color(255,   0,   0)); // Red
    else
      alertbar.setPixelColor(i, alertbar.Color(  0, 255,   0)); // Green

  alertbar.show();
}
#endif // CAMERA_DETECT_PIN
}

//****************************************************************************

#endif // USE_NEOPIXEL

//****************************************************************************
