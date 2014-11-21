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
#define MAX31855_LAT0 3
#define MAX31855_LAT1 3
#define MAX31855_LAT2 3
#define MAX31855_LAT3 3
#define SSR_PIN 4

uint8_t state = 0;
uint8_t max_temp = 25;
uint8_t soak_time = 25;
uint8_t ramp_rate = 10;
uint8_t cool_down = 10;

uint16_t on_time;
uint16_t off_time;
uint8_t init_temp;

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
DHT dht;
MAX31855 thermo[4];

void setup()
{
	thermo[0].setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT0);
        thermo[1].setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT1);
        thermo[2].setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT2);
        thermo[3].setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT3);
        
        pinMode(SSR_PIN, OUTPUT);

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
	case 6:  //Ramp Up
                on_time = 100;
                off_time = 200;

                read_temps();
                init_temp = CtoF(thermo[0].thermocouple_temp);
                
                while(CtoF(thermo[0].thermocouple_temp) < max_temp)
                {
                  digitalWrite(SSR_PIN, 1);
                  delay(on_time);
                  digitalWrite(SSR_PIN, 0);
                  delay(off_time);
                  
                  read_temps();
                }
                state = state + 1;
                break;
        case 7:  //Soak Time
                run_soak_time();
                state = state + 1;
                break;
        case 8:  //Cool Down
                on_time = 100;
                off_time = 200;
                
                read_temps();
                
                while(CtoF(thermo[0].thermocouple_temp) > init_temp)
                {
                  digitalWrite(SSR_PIN, 1);
                  delay(on_time);
                  digitalWrite(SSR_PIN, 0);
                  delay(off_time);
                  
                  read_temps();
                }
                state = state + 1;
                break;
        case 9:  //Finished
                lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("Finished");
                while(1);	
  	}
}

void run_soak_time(void)
{
    uint8_t delta = 1;
    uint32_t initial_time;
    uint32_t elapsed_time;
    uint32_t diff_time_min;
    uint8_t screen = 0;
    
    on_time = 100;
    off_time = 100;
    
    initial_time = millis();
    elapsed_time = millis();
    diff_time_min = (elapsed_time - initial_time) / 60000;
    
    lcd.clear();
    print_elapsed(1, diff_time_min);
    print_time_left(0, soak_time - diff_time_min);
    
    while(diff_time_min < soak_time)
    {
      read_temps();
      if(CtoF(thermo[0].thermocouple_temp) < max_temp - delta)
      {
        digitalWrite(SSR_PIN, 1);
        delay(on_time);
        digitalWrite(SSR_PIN, 0);
        delay(off_time);
      }
      elapsed_time = millis();
      diff_time_min = (elapsed_time - initial_time) / 60000;
      
      uint8_t buttons = lcd.readButtons();
      if (buttons) {
        if(buttons & BUTTON_UP) 
        {
          if(screen == 0){ screen = 7;}
          else{screen--;}
          delay(500);
        }
        else if(buttons & BUTTON_DOWN) 
        {
          if(screen == 7){ screen = 0;}
          else{screen++;}
          delay(500);
        }
        lcd.clear();
        switch(screen)
        {
          case 0:
              print_elapsed(1, diff_time_min);
              print_time_left(0, soak_time - diff_time_min);
              break;
          case 1:
              print_time_left(1, soak_time - diff_time_min);
              print_thermo(0,0);
              break;
          case 2:
              print_thermo(0,1);
              print_thermo(1,0);
              break;
          case 3:
              print_thermo(1,1);
              print_thermo(2,0);
              break;
          case 4:
              print_thermo(2,1);
              print_thermo(3,0);
              break;
          case 5:
              print_thermo(3,1);
              print_dht(0,0);
              break;
          case 6:
              print_dht(0,1);
              print_dht(1,0);
              break;
          case 7:
              print_dht(1,1);
              print_elapsed(0, diff_time_min);
              break;
        } 
      }
   }
}

void print_elapsed(uint8_t row, uint32_t print_time)
{
    lcd.setCursor(0,row);
    lcd.print("Elapsed:");
    lcd.setCursor(13,row);
    lcd.print("min");
    lcd.setCursor(10,row);
    lcd.print(print_time);
}

void print_time_left(uint8_t row, uint32_t print_time)
{
    lcd.setCursor(0,row);
    lcd.print("To go:");
    lcd.setCursor(13,row);
    lcd.print("min");
    lcd.setCursor(10,row);
    lcd.print(print_time);
}

void print_thermo(uint8_t index, uint8_t row)
{
    lcd.setCursor(0,row);
    lcd.print("Temp");
    lcd.setCursor(5,row);
    lcd.print(index);
    lcd.setCursor(6,row);
    lcd.print(":");
    lcd.setCursor(12,row);
    lcd.print("degF");
    lcd.setCursor(8,row);
    lcd.print(CtoF(thermo[index].thermocouple_temp));
}

void print_dht(uint8_t index, uint8_t row)
{
    lcd.setCursor(0,row);
    
    if(index == 0)
    {
      lcd.print("DHT Temp:");
      lcd.setCursor(12,row);
      lcd.print("degF");
      lcd.setCursor(8,row);
      lcd.print(CtoF(dht.temperature));
    }
    else
    {
      lcd.print("DHT Hum:");
      lcd.setCursor(15,row);
      lcd.print("%");
      lcd.setCursor(8,row);
      lcd.print(dht.humidity);
    }
    

}

uint8_t CtoF(uint8_t temp_C)
{
  uint8_t temp_F = (temp_C * 1.8) + 32;
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
