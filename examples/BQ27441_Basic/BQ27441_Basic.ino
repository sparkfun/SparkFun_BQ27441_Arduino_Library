/******************************************************************************
BQ27441_Basic.ino
Basic BQ27441 Example Sketch
Jim Lindblom @ SparkFun Electronics
May 9, 2016
https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library

This example demonstrates how to initialize a BQ27441, set the capacity of
your LiPo battery, and read state-of-charge (SoC), voltage, current, and
remaining capacity.

Hardware Resources:
- Arduino Development Board
- SparkFun Battery Babysitter

Development environment specifics:
Arduino 1.6.7
SparkFun Battery Babysitter v1.0
Arduino Uno (any 'duino should do)
******************************************************************************/
#include <SparkFunBQ27441.h>

const int BATTERY_CAPACITY = 850; // e.g. 850mAh battery

void setupBQ27441(void)
{
  if (!lipo.begin())
  {
    Serial.println("Error: Unable to communicate with BQ27441.");
    Serial.println("  Check wiring and try again.");
    Serial.println("  (Battery must be plugged into Battery Babysitter!)");
    while (1) ;
  }
  Serial.println("Connected to BQ27441!");
  
  lipo.setCapacity(BATTERY_CAPACITY);
  Serial.print("Full charge capacity set to: ");
  Serial.println(String(lipo.fullChargeCapacity()) + " mAh");
}

void printBatteryStats()
{
  uint16_t soc = lipo.soc();
  uint16_t vRaw = lipo.voltage();
  float volts = lipo.convertVoltage(vRaw);
  int16_t current = lipo.current();
  uint16_t capacity = lipo.capacity();

  String toPrint = String(soc) + " % |\t";
  toPrint += String(volts) + " V |\t";
  toPrint += String(current) + " mA |\t";
  toPrint += String(capacity) + " mAh";
  Serial.println(toPrint);
}

void setup()
{
  Serial.begin(115200);
  setupBQ27441();
}

void loop() 
{
  printBatteryStats();
  delay(1000);
}
