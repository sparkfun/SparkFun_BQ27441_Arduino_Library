/******************************************************************************
BQ27441_GPOUT_BAT_LOW
BQ27441 Library GPOUT Example - Battery low Mode
Jim Lindblom @ SparkFun Electronics
May 9, 2016
https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library

Demonstrates how to use the BQ27441's BAT_LOW function on GPOUT. In this mode
GPOUT will become active whenever the battery goes below a set threshold.

In addition to I2C, connect GPOUT to pin 2 of your Arduino.

After uploading, open up the serial monitor to 115200 baud to view your 
battery's stats. They should only print when the percentage goes up or down
by 1%.

Hardware Resources:
- Arduino Development Board
- SparkFun Battery Babysitter

Development environment specifics:
Arduino 1.6.7
SparkFun Battery Babysitter v1.0
Arduino Uno (any 'duino should do)
******************************************************************************/
#include <SparkFunBQ27441.h>

// Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 850; // e.g. 850mAh battery

const byte SOCI_SET = 15; // Interrupt set threshold at 20%
const byte SOCI_CLR = 20; // Interrupt clear threshold at 25%
const byte SOCF_SET = 5; // Final threshold set at 5%
const byte SOCF_CLR = 10; // Final threshold clear at 10%

// Arduino pin connected to BQ27441's GPOUT pin
const int GPOUT_PIN = 2;

void setupBQ27441(void)
{
  pinMode(GPOUT_PIN, INPUT_PULLUP); // Set the GPOUT pin as an input w/ pullup
  
  // Use lipo.begin() to initialize the BQ27441-G1A and confirm that it's
  // connected and communicating.
  if (!lipo.begin()) // begin() will return true if communication is successful
  {
	// If communication fails, print an error message and loop forever.
    Serial.println("Error: Unable to communicate with BQ27441.");
    Serial.println("  Check wiring and try again.");
    Serial.println("  (Battery must be plugged into Battery Babysitter!)");
    while (1) ;
  }
  Serial.println("Connected to BQ27441!");

  // In this example, we'll manually enter and exit config mode. By controlling
  // config mode manually, you can set the chip up faster -- completing all of
  // the set up in a single config mode sweep.
  lipo.enterConfig(); // To configure the values below, you must be in config mode
  lipo.setCapacity(BATTERY_CAPACITY); // Set the battery capacity
  lipo.setGPOUTPolarity(LOW); // Set GPOUT to active-high
  lipo.setGPOUTFunction(BAT_LOW); // Set GPOUT to BAT_LOW mode
  lipo.setSOC1Thresholds(SOCI_SET, SOCI_CLR); // Set SOCI set and clear thresholds
  lipo.setSOCFThresholds(SOCF_SET, SOCF_CLR); // Set SOCF set and clear thresholds
  lipo.exitConfig();

  // Use lipo.GPOUTPolarity to read from the chip and confirm the changes
  if (lipo.GPOUTPolarity())
    Serial.println("GPOUT set to active-HIGH");
  else
    Serial.println("GPOUT set to active-LOW");

  // Use lipo.GPOUTFunction to confirm the functionality of GPOUT
  if (lipo.GPOUTFunction())
    Serial.println("GPOUT function set to BAT_LOW");
  else
    Serial.println("GPOUT function set to SOC_INT");

  // Read the set and clear thresholds to make sure they were set correctly
  Serial.println("SOC1 Set Threshold: " + String(lipo.SOC1SetThreshold()));
  Serial.println("SOC1 Clear Threshold: " + String(lipo.SOC1ClearThreshold()));
  Serial.println("SOCF Set Threshold: " + String(lipo.SOCFSetThreshold()));
  Serial.println("SOCF Clear Threshold: " + String(lipo.SOCFClearThreshold()));
}

void printBatteryStats()
{
  // Read battery stats from the BQ27441-G1A
  unsigned int soc = lipo.soc();  // Read state-of-charge (%)
  unsigned int volts = lipo.voltage(); // Read battery voltage (mV)
  int current = lipo.current(AVG); // Read average current (mA)
  unsigned int fullCapacity = lipo.capacity(FULL); // Read full capacity (mAh)
  unsigned int capacity = lipo.capacity(REMAIN); // Read remaining capacity (mAh)
  int power = lipo.power(); // Read average power draw (mW)
  int health = lipo.soh(); // Read state-of-health (%)

  // Assemble a string to print
  String toPrint = String(soc) + "% | ";
  toPrint += String(volts) + " mV | ";
  toPrint += String(current) + " mA | ";
  toPrint += String(capacity) + " / ";
  toPrint += String(fullCapacity) + " mAh | ";
  toPrint += String(power) + " mW | ";
  toPrint += String(health) + "%";
  toPrint += " | 0x" + String(lipo.flags(), HEX);
  
  // Print the string
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
  
  // If the GPOUT interrupt is active (low)
  if (!digitalRead(GPOUT_PIN))
  {
    // Check which of the flags triggered the interrupt
    if (lipo.socfFlag()) 
      Serial.println("<!-- WARNING: Battery Dangerously low -->");
    else if (lipo.socFlag())
      Serial.println("<!-- WARNING: Battery Low -->");
  }
  delay(1000);
}