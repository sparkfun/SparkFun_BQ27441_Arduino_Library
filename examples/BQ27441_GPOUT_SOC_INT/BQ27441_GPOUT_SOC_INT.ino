#include <SparkFunBQ27441.h>

const unsigned int BATTERY_CAPACITY = 850; // e.g. 850mAh battery

const int GPOUT_PIN = 2;

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
  lipo.setCapacity(BATTERY_CAPACITY);
  lipo.setGPOUTPolarity(LOW); // active-low
  lipo.setGPOUTFunction(SOC_INT); // SOC_INT mode
  lipo.setSOCIDelta(1);
  lipo.exitConfig();

  if (lipo.GPOUTPolarity())
    Serial.println("GPOUT set to active-HIGH");
  else
    Serial.println("GPOUT set to active-LOW");

  if (lipo.GPOUTFunction())
    Serial.println("GPOUT function set to BAT_LOW");
  else
    Serial.println("GPOUT function set to SOC_INT");

  Serial.println("SOCI Delta: " + String(lipo.sociDelta()));
  Serial.println();
  
  Serial.println("Testing GPOUT Pulse");
  lipo.pulseGPOUT();
  int timeout = 10000;
  while ((digitalRead(GPOUT_PIN)) && timeout--)
    delay(1);
  if (timeout > 0) 
  {
    Serial.print("GPOUT test successful!");
    Serial.println("(" + String(10000 - timeout) + ")");
    Serial.print("GPOUT will pulse whenever the SoC ");
    Serial.println("value changes by SOCI delta.");
    Serial.print("Or when the battery changes from");
    Serial.println(" charging to discharging, or vice-versa.");
    Serial.println();
  }
  else
  {
    Serial.println("GPOUT didn't pulse.");
    Serial.print("Make sure it's connected to pin ");
    Serial.print(GPOUT_PIN);
    Serial.println(" and reset.");
    while (1) ;
  }
}

void printBatteryStats()
{
  unsigned int soc = lipo.soc();
  unsigned int volts = lipo.voltage();
  int current = lipo.current(AVG);
  unsigned int capacity = lipo.capacity(REMAIN);
  int power = lipo.power();
  int health = lipo.soh();
  unsigned int fullCapacity = lipo.capacity(FULL);

  String toPrint = "[" + String(millis()/1000) + "] ";
  toPrint += String(soc) + "% | ";
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
  pinMode(GPOUT_PIN, INPUT_PULLUP);
  
  setupBQ27441();
}

void loop() 
{
  if (digitalRead(GPOUT_PIN) == LOW)
  {
    printBatteryStats();
  }
}
