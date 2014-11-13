#include "MAX31855.h"
#include "DHT.h"
#include "SD.h"
#include "LiquidCrystal.h"

#define DHTPIN 0     	// what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define MAX31855_DATA 1
#define MAX31855_CLK 2
#define MAX31855_LAT 3

#define SSR_PIN 4

DHT dht(DHTPIN, DHTTYPE);
MAX31855 thermo();

void setup()
{
	thermo.setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT);
	dht.begin();

	Serial.begin(9600);
}
 
void loop()
{
	// Wait a few seconds between measurements.
	delay(2000);

	// Reading temperature or humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	float h = dht.readHumidity();
	// Read temperature as Celsius
	float t = dht.readTemperature();
	// Read temperature as Fahrenheit
	float f = dht.readTemperature(true);

	// Check if any reads failed and exit early (to try again).
	if (isnan(h) || isnan(t) || isnan(f)) {
	Serial.println("Failed to read from DHT sensor!");
	return;
	}

	// Compute heat index
	// Must send in temp in Fahrenheit!
	float hi = dht.computeHeatIndex(f, h);

	Serial.print("Humidity: "); 
	Serial.print(h);
	Serial.print(" %\t");
	Serial.print("Temperature: "); 
	Serial.print(t);
	Serial.print(" *C ");
	Serial.print(f);
	Serial.print(" *F\t");
	Serial.print("Heat index: ");
	Serial.print(hi);
	Serial.println(" *F");
}
