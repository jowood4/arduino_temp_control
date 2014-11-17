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

uint8_t state = 0;
DHT dht;
MAX31855 thermo;

void setup()
{
	thermo.setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT);

	Serial.begin(9600);

	lcd.begin(16, 2);
	lcd.setBacklight(WHITE);
}
 
void loop()
{
	switch (state) {
	case 0:
	        lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("Final Approach");
		lcd.setCursor(0,1);
      		lcd.print("Composites");
	      	
		delay(3000);
		state = 1;
		break;
	case 1:
	      	lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("Enter Max Temp");
		lcd.setCursor(6,1);
      		lcd.print("Deg F");
	      	break;
	case 2:
	      	lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("Enter Soak Time");
		lcd.setCursor(6,1);
      		lcd.print("Mins");
	      	break;
	case 3:
	      	lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("Enter Ramp Rate");
		lcd.setCursor(6,1);
      		lcd.print("Deg F/Min");
	      	break;
	case 4:
	      	lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("Enter Cool Down");
		lcd.setCursor(6,1);
      		lcd.print("Deg F/Min");
	      	break;
	case 5:
	      	lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("To Start");
		lcd.setCursor(0,1);
      		lcd.print("Press Enter");
	      	break;
	case 6:
  	}

	dht.read22(DHTPIN);
}
