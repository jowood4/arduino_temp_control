#include "MAX31855.h"
#include "dht11.h"
#include "SD.h"
#include "Adafruit_MCP23017.h"
#include "Adafruit_RGBLCDShield.h"
#include "Wire.h"

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define DHTPIN 0     	// what pin we're connected to
#define MAX31855_DATA 1
#define MAX31855_CLK 2
#define MAX31855_LAT 3
#define SSR_PIN 4

uint8_t state = 0;
uint8_t max_temp = 25;
uint8_t soak_time = 25;
uint8_t ramp_rate = 10;
uint8_t cool_down = 10;

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
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
		lcd.setCursor(4,1);
      		lcd.print("Deg F");
      
                max_temp = enter_value(max_temp);
                state = state + 1;
	      	break;
	case 2:
	      	lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("Enter Soak Time");
		lcd.setCursor(4,1);
      		lcd.print("Mins");
      
                soak_time = enter_value(soak_time);
                state = state + 1;
	      	break;
	case 3:
	      	lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("Enter Ramp Rate");
		lcd.setCursor(4,1);
      		lcd.print("Deg F/Min");
      
                ramp_rate = enter_value(ramp_rate);
                state = state + 1;
	      	break;
	case 4:
	      	lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("Enter Cool Down");
		lcd.setCursor(4,1);
      		lcd.print("Deg F/Min");
      
                cool_down = enter_value(cool_down);
                state = state + 1;
	      	break;
	case 5:
                lcd.noBlink();
                lcd.noCursor();
	      	lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("To Start");
		lcd.setCursor(0,1);
      		lcd.print("Press Enter");
      
                while(1)
                {
                  uint8_t buttons = lcd.readButtons();
                  if (buttons & BUTTON_SELECT)
                  {
                    break;
                  }
                }
                state = state + 1;
	      	break;
	case 6:
		//dht.read22(DHTPIN);
                lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("Running");
                while(1);	
  	}
}

uint8_t enter_value(uint8_t init_value)
{
  uint8_t digit = 0;
  uint8_t temp;
  
  lcd.blink();
  lcd.setCursor(0,1);
  lcd.print(0);
  lcd.setCursor(1,1);
  lcd.print(init_value);
  lcd.cursor();
  lcd.setCursor(0,1);
  
  while(1)
  {
    uint8_t buttons = lcd.readButtons();
    
    if (buttons) {
      if (buttons & BUTTON_UP) {
        switch (digit) 
        {
          case 0:
            temp = init_value + 100;
            break;
          case 1:
            temp = init_value + 10;
            break;
          case 2:
            temp = init_value + 1;
            break;
        }
        if(temp <= 255)
        {
          init_value = temp;
        }
        if(init_value < 100)
        {
          lcd.setCursor(0,1);
          lcd.print(0);
          lcd.setCursor(1,1);
          lcd.print(init_value);
          lcd.setCursor(digit,1);
          delay(500);
        }
        else
        {
          lcd.setCursor(0,1);
          lcd.print(init_value);
          lcd.setCursor(digit,1);
          delay(500);
        }
      }
      if (buttons & BUTTON_DOWN) {
        switch (digit) 
        {
          case 0:
            temp = init_value - 100;
            break;
          case 1:
            temp = init_value - 10;
            break;
          case 2:
            temp = init_value - 1;
            break;
        }
        if(temp >= 0)
        {
          init_value = temp;
        }
        if(init_value < 100)
        {
          lcd.setCursor(0,1);
          lcd.print(0);
          lcd.setCursor(1,1);
          lcd.print(init_value);
          lcd.setCursor(digit,1);
          delay(500);
        }
        else
        {
          lcd.setCursor(0,1);
          lcd.print(init_value);
          lcd.setCursor(digit,1);
          delay(500);
        }
      }
      if (buttons & BUTTON_LEFT) {
        if(digit > 0)
        {
          digit = digit - 1;
        }
        lcd.setCursor(digit,1);
        delay(500);
      }
      if (buttons & BUTTON_RIGHT) {
        if(digit < 2)
        {
          digit = digit + 1;
        }
        lcd.setCursor(digit,1);
        delay(500);
      }
      if (buttons & BUTTON_SELECT) {
        break;
      }
    }
  }
  return init_value;
}
