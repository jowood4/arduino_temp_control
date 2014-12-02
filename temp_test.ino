#include "MAX31855.h"
#include "dht11.h"

#define SPI_transfer 1

#if defined(SPI_transfer)
#include "SPI.h"
#endif

#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define DHTPIN 6     	// what pin we're connected to

#define MAX31855_DATA 12
#define MAX31855_CLK 13
#define MAX31855_LAT0 10
#define MAX31855_LAT1 9
#define MAX31855_LAT2 8
#define MAX31855_LAT3 7

DHT dht;
MAX31855 thermo[4];


double CtoF(double temp_C)
{
  double temp_F = (temp_C * 1.8) + 32;
  return temp_F; 
}

void read_temps(void)
{
  for(uint8_t i = 0; i<4; i++)
  {
    thermo[i].read_temp();
  }
  dht.read22(DHTPIN);
}

void setup()
{
	//thermo.setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT0);
        thermo[0].setup(MAX31855_LAT0);
        thermo[1].setup(MAX31855_LAT1);
        thermo[2].setup(MAX31855_LAT2);
        thermo[3].setup(MAX31855_LAT3);

	Serial.begin(9600);
}
 
void loop()
{
    read_temps();
    Serial.println(CtoF(thermo[0].thermocouple_temp));
    Serial.println(CtoF(thermo[0].chip_temp));
    Serial.println(CtoF(thermo[1].thermocouple_temp));
    Serial.println(CtoF(thermo[1].chip_temp));
    Serial.println(CtoF(thermo[2].thermocouple_temp));
    Serial.println(CtoF(thermo[2].chip_temp));
    Serial.println(CtoF(thermo[3].thermocouple_temp));
    Serial.println(CtoF(thermo[3].chip_temp));
    Serial.println(CtoF(dht.temperature));
    Serial.println(CtoF(dht.humidity));

    delay(3000);    
}

