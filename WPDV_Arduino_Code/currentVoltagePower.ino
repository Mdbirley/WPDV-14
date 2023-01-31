//////////////////////////////////////////////////////////////////////////////////////////////////
// AdaFruit Voltage/Current Code - 11-Jan-2022
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <Adafruit_INA260.h>                      // AdaFruit Current/Voltage Library

// Addresses 0x40, 0x41, 0x44, 0x45 using A0/A1, additional options via SCL,SDA

Adafruit_INA260 ina260a = Adafruit_INA260();
Adafruit_INA260 ina260b = Adafruit_INA260();

//////////////////////////////////////////////////////////////////////////////////////////////////

void initCurrentVoltagePower(void)
{
  if (!ina260a.begin(0x40)) if (SerialDEBUG) SerialDEBUG.println("Couldn't find INA260 chip#1");
  if (!ina260b.begin(0x41)) if (SerialDEBUG) SerialDEBUG.println("Couldn't find INA260 chip#2");
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void readCurrentVoltagePower(void)
{
  driveCurrent	= ina260a.readCurrent() * 1e-3; // mA to Amps
  driveVoltage	= ina260a.readBusVoltage() * 1e-3; // mV to voltage 12.3456
  drivePower		= ina260a.readPower() * 1e-3; // mW to Watts
  if (driveCurrent < 0.0) driveCurrent = 0.0;

  mowerCurrent	= ina260b.readCurrent() * 1e-3; // mA to Amps
  mowerVoltage	= ina260b.readBusVoltage() * 1e-3; // mV to voltage 12.3456
  mowerPower		= ina260b.readPower() * 1e-3; // mW to Watts
  if (mowerCurrent < 0.0) mowerCurrent = 0.0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//
//Older INA260
//  float adaCurrent;     // mA
//  float adaBusVoltage;  // mV
//  float adaPower;       // mW
//
// Older
//  adaCurrent		= ina260a.readCurrent() * 1e-3; // mA to Amps
//  adaBusVoltage = ina260a.readBusVoltage() * 1e-3; // mV to voltage 12.3456
//  adaPower			= ina260a.readPower() * 1e-3; // mW to Watts
//
//void dumpCurrentVoltagePower(void)
//{
//  SerialDEBUG.print("Current: ");
//  SerialDEBUG.print(adaCurrent);
//  SerialDEBUG.print(" mA ");
//
//  SerialDEBUG.print("Bus Voltage: ");
//  SerialDEBUG.print(adaBusVoltage, 2);
//  SerialDEBUG.print(" V ");
//
//  SerialDEBUG.print("Power: ");
//  SerialDEBUG.print(adaPower);
//  SerialDEBUG.println(" mW");
//}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
