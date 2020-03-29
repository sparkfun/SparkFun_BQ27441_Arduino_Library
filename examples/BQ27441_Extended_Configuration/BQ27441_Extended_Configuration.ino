#include <SparkFunBQ27441.h>

// Set BATTERY_CAPACITY to the design capacity of your battery in mAh.
const uint16_t BATTERY_CAPACITY = 2850;

//lowest operational voltage in mV
const uint16_t TERMINATE_VOLTAGE = 3000;

//current at which charger stops charging battery in mA
//in case of Sparkfun Battery Babysitter board:
// 100mA charge current --> 12mA
// 500mA charge current --> 60mA
const uint16_t TAPER_CURRENT = 60;

void setup()
{
    Serial.begin(115200);

    if (!lipo.begin()) // begin() will return true if communication is successful
    {
        // If communication fails, print an error message and loop forever.
        Serial.println("Error: Unable to communicate with BQ27441.");
        Serial.println("  Check wiring and try again.");
        Serial.println("  (Battery must be plugged into Battery Babysitter!)");
        while (1)
            ;
    }
    Serial.println("Connected to BQ27441!");

    if (lipo.itporFlag()) //write config parameters only if needed
    {
        Serial.println("Writing gague config");

        lipo.enterConfig();                 // To configure the values below, you must be in config mode
        lipo.setCapacity(BATTERY_CAPACITY); // Set the battery capacity

        /*
            Design Energy should be set to be Design Capacity × 3.7 if using the bq27441-G1A or Design
            Capacity × 3.8 if using the bq27441-G1B
        */
        lipo.setDesignEnergy(BATTERY_CAPACITY * 3.7f);

        /*
            Terminate Voltage should be set to the minimum operating voltage of your system. This is the target
            where the gauge typically reports 0% capacity
        */
        lipo.setTerminateVoltage(TERMINATE_VOLTAGE);

        /*
            Taper Rate = Design Capacity / (0.1 * Taper Current)
        */
        lipo.setTaperRate(10 * BATTERY_CAPACITY / TAPER_CURRENT);

        lipo.exitConfig(); // Exit config mode to save changes
    }
    else
    {
        Serial.println("Using existing gague config");
    }
}

void printBatteryStats()
{
    // Read battery stats from the BQ27441-G1A
    unsigned int soc = lipo.soc();                   // Read state-of-charge (%)
    unsigned int volts = lipo.voltage();             // Read battery voltage (mV)
    int current = lipo.current(AVG);                 // Read average current (mA)
    unsigned int fullCapacity = lipo.capacity(FULL); // Read full capacity (mAh)
    unsigned int capacity = lipo.capacity(REMAIN);   // Read remaining capacity (mAh)
    int power = lipo.power();                        // Read average power draw (mW)
    int health = lipo.soh();                         // Read state-of-health (%)

    // Assemble a string to print
    String toPrint = "[" + String(millis() / 1000) + "] ";
    toPrint += String(soc) + "% | ";
    toPrint += String(volts) + " mV | ";
    toPrint += String(current) + " mA | ";
    toPrint += String(capacity) + " / ";
    toPrint += String(fullCapacity) + " mAh | ";
    toPrint += String(power) + " mW | ";
    toPrint += String(health) + "%";

    //fast charging allowed
    if (lipo.chgFlag())
        toPrint += " CHG";

    //full charge detected
    if (lipo.fcFlag())
        toPrint += " FC";

    //battery is discharging
    if (lipo.dsgFlag())
        toPrint += " DSG";

    // Print the string
    Serial.println(toPrint);
}

void loop()
{
    printBatteryStats();
    delay(1000);
}
