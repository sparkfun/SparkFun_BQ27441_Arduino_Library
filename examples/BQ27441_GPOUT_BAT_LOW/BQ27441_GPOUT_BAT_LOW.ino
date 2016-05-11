#include <SparkFunBQ27441.h>

const unsigned int BATTERY_CAPACITY = 850; // e.g. 850mAh battery
unsigned int fullCapacity;

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

  lipo.enterConfig();
  lipo.setGPOUTPolarity(HIGH); // active-high
  lipo.setGPOUTFunction(BAT_LOW); // BAT_LOW mode
  lipo.setSOC1Thresholds(20, 25);
  lipo.setSOCFThresholds(5, 10);
  lipo.exitConfig();

  if (lipo.GPOUTPolarity())
    Serial.println("GPOUT set to active-HIGH");
  else
    Serial.println("GPOUT set to active-LOW");

  if (lipo.GPOUTFunction())
    Serial.println("GPOUT function set to BAT_LOW");
  else
    Serial.println("GPOUT function set to SOC_INT");

  Serial.println("SOC1 Set Threshold: " + String(lipo.SOC1SetThreshold()));
  Serial.println("SOC1 Clear Threshold: " + String(lipo.SOC1ClearThreshold()));
  Serial.println("SOCF Set Threshold: " + String(lipo.SOCFSetThreshold()));
  Serial.println("SOCF Clear Threshold: " + String(lipo.SOCFClearThreshold()));
}

void printBatteryStats()
{
  unsigned int soc = lipo.soc();
  unsigned int volts = lipo.voltage();
  int current = lipo.current(AVG);
  unsigned int capacity = lipo.capacity(REMAIN);
  int power = lipo.power();
  int health = lipo.soh();
  fullCapacity = lipo.capacity(FULL);

  String toPrint = String(soc) + "% | ";
  toPrint += String(volts) + " mV | ";
  toPrint += String(current) + " mA | ";
  toPrint += String(capacity) + " / ";
  toPrint += String(fullCapacity) + " mAh | ";
  toPrint += String(power) + " mW | ";
  toPrint += String(health) + "%";

  toPrint += " | 0x" + String(lipo.status(), HEX) + " | ";
  toPrint += "0x" + String(lipo.flags(), HEX) + " | ";
  
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
