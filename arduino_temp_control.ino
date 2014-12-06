#include <PID_v1.h>

#include "MAX31855.h"
#include "dht11.h"
#include "SD.h"
#include "Adafruit_MCP23017.h"
#include "Adafruit_RGBLCDShield.h"
#include "Wire.h"

#define SPI_transfer 1

#if defined(SPI_transfer)
#include "SPI.h"
#endif

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define DHTPIN 6     	// what pin we're connected to
#define MAX31855_DATA 12
#define MAX31855_CLK 13
#define MAX31855_LAT0 10
#define MAX31855_LAT1 9
#define MAX31855_LAT2 8
#define MAX31855_LAT3 7
#define SSR_PIN 5
#define SD_CSB 4

uint8_t state = 0;
uint8_t max_temp = 95;
uint8_t soak_time = 10;
uint8_t ramp_rate = 10;
uint8_t cool_down = 10;

File dataFile;

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
DHT dht;
MAX31855 thermo[4];

double init_temp;
double Input, Output, Setpoint;
uint32_t init_read = 1;  //flag to prevent 2 temp reads on first PID pass
uint32_t PID_interval = 1000;  //time in ms to run PID interval
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);
uint8_t screen = 0;  //index of menu to display while running

void setup()
{
        lcd.begin(16, 2);
	lcd.setBacklight(WHITE);
  
	//thermo[0].setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT0);
        //thermo[1].setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT1);
        //thermo[2].setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT2);
        //thermo[3].setup(MAX31855_DATA, MAX31855_CLK, MAX31855_LAT3);
        thermo[0].setup(MAX31855_LAT0);
        thermo[1].setup(MAX31855_LAT1);
        thermo[2].setup(MAX31855_LAT2);
        thermo[3].setup(MAX31855_LAT3);
        
        pinMode(SSR_PIN, OUTPUT);

	//Serial.begin(9600);

        //SD Card Init
        pinMode(10, OUTPUT);
        if (!SD.begin(SD_CSB))
        {
            lcd.clear();
    	    lcd.setCursor(0,0);
            lcd.print("SD Card Failed");
            return;
        }
        dataFile = SD.open("datalog.txt", FILE_WRITE);
        if (!dataFile)
        {
            lcd.clear();
    	    lcd.setCursor(0,0);
            lcd.print("SD File Error");
            return;
        }
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
		state = state + 1;
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
                run_ramp_time();
                state = state + 1;
                break;
        case 7:  //Soak Time
                //Serial.println("state 7");
                run_ramp_time();
                state = state + 1;
                break;
        case 8:  //Cool Down
                run_ramp_time();
                state = state + 1;
                break;
        case 9:  //Finished
                lcd.clear();
    		lcd.setCursor(0,0);
      		lcd.print("Finished");
                dataFile.close();
                while(1);	
  	}
}

void run_PID(double Kp, double Ki, double Kd, uint16_t WindowSize, uint32_t time_interval, double time_elapsed, double time_left)
{
    double ratio;
    uint32_t windowStartTime;
    uint8_t buttons;

    //Specify the links and initial tuning parameters
    myPID.SetOutputLimits(0, WindowSize);
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.SetMode(AUTOMATIC);
   
    if(init_read){init_read = 0;Setpoint = Setpoint + 1;}
    else{read_temps();}
    
    windowStartTime = millis();
    
    Input = CtoF(thermo[0].thermocouple_temp);
    //Input = CtoF(thermo[0].chip_temp);
    //Input = 25;
    //Setpoint = 100;
    myPID.Compute();
    
    while(Output == 0) myPID.Compute();
    ratio = Output / WindowSize;
    //ratio = Output / max_temp;

    digitalWrite(SSR_PIN, 1);
    while(millis() - windowStartTime < time_interval * ratio)
    {
      //Serial.println(millis());
      buttons = lcd.readButtons();
      //Serial.println(millis());
      if(buttons)
      {
        //Serial.println(buttons);
        running_menu(buttons, time_elapsed, time_left);
        delay(250);
      }
    }
    digitalWrite(SSR_PIN, 0);

    while(millis() - windowStartTime < time_interval)
    {
      //Serial.println(millis() - windowStartTime);
      buttons = lcd.readButtons();
      //Serial.println(millis());
      //Serial.println("here2");
      if(buttons)
      {
        //Serial.println(buttons);
        running_menu(buttons, time_elapsed, time_left);
        delay(250);
      }
    }
}

void run_ramp_time(void)
{
    uint32_t initial_time = millis();
    uint32_t elapsed_time, time1, time2;
    double diff_time_min;
    double ramp_time;
    double set_temp;
    
    if(state == 6)  //rising ramp, increasing temperature
    {
        read_temps();
        init_temp = CtoF(thermo[0].thermocouple_temp);
        //init_temp = CtoF(thermo[0].chip_temp);
        ramp_time = (max_temp - init_temp)/ramp_rate;
    }
    else if(state == 7) //soak time
    {
        ramp_time = soak_time; 
    }
    else if(state == 8)  //falling ramp, decreasing temperature
    {
        read_temps();
        ramp_time = (CtoF(thermo[0].thermocouple_temp) - init_temp)/cool_down;      
    }

    elapsed_time = millis();
    diff_time_min = (elapsed_time - initial_time) / 60000;
    
    lcd.clear();
    print_time(1, diff_time_min, 1);
    print_time(0, ramp_time - diff_time_min, 0);
    
    while(diff_time_min < ramp_time)
    {   
          if(state == 6)  //rising ramp, increasing temperature
          {
              //While increasing, Setpoint increases based on elapsed time
              set_temp = (diff_time_min * ramp_rate) + init_temp;
          }
          else if(state == 7) //soak time
          {
              set_temp = max_temp; 
          }
          else if(state == 8)  //falling ramp, decreasing temperature
          {
              //While decreasing, Setpoint increases based on elapsed time
              set_temp = max_temp - (diff_time_min * cool_down);      
          }
          Setpoint = set_temp;
          run_PID(2, 5, 1, 500, PID_interval, diff_time_min, ramp_time - diff_time_min);
          elapsed_time = millis();
          diff_time_min = (elapsed_time - initial_time) / 60000;
          running_menu(lcd.readButtons(), diff_time_min, ramp_time - diff_time_min);
          //Serial.println(diff_time_min);
    }
    //Serial.println("here");
}

void print_time(uint8_t row, uint32_t print_time, uint8_t elapsed)
{
    lcd.setCursor(0,row);
    if(elapsed == 1)
    {
      lcd.print("Elapsed:");
    }
    else
    {
      lcd.print("To go:");
    }
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
  //dht.read22(DHTPIN);
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

void write_SD(void)
{
    String dataString = "";
    dataString += "Time: ";
    //dataString += String(time);
    dataString += ", T0: ";
    dataString += String(CtoF(thermo[0].thermocouple_temp));
    dataString += "degF, T1: ";
    dataString += String(CtoF(thermo[1].thermocouple_temp));
    dataString += "degF, T2: ";
    dataString += String(CtoF(thermo[2].thermocouple_temp));
    dataString += "degF, T3: ";
    dataString += String(CtoF(thermo[3].thermocouple_temp));
    dataString += "degF, DHT Temp: ";
    dataString += String(CtoF(dht.temperature));
    dataString += "degF, DHT Humidity: ";
    dataString += String(CtoF(dht.humidity));
    dataString += "%";
    dataFile.println(dataString);
}

void running_menu(uint8_t buttons, double time_elapsed, double time_left)
{
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
             print_time(1, time_elapsed, 1);
             print_time(0, time_left, 0);
             break;
         case 1:
             print_time(1, time_left, 0);
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
             print_time(0, time_elapsed, 1);
             break;
      }
}
