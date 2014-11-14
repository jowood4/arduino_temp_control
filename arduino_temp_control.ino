#include "MAX31855.h"
#include "dht11.h"
#include "SD.h"
#include "Adafruit_MCP23017.h"
#include "Adafruit_RGBLCDShield.h"
#include "Wire.h"

#define DHTPIN 0     	// what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define MAX31855_DATA 1
#define MAX31855_CLK 2
#define MAX31855_LAT 3

#define SSR_PIN 4

DHT dht;
MAX31855 thermo;

void setup()
{
	thermo.setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT);

	Serial.begin(9600);
}
 
void loop()
{
	// Wait a few seconds between measurements.
	delay(2000);

	dht.read22(DHTPIN);
}
