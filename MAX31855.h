#ifndef MAX31855_H
#define MAX31855_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

class MAX31855_H
{
public:
	MAX31855(void);
	void setup(uint8_t data, uint8_t clock, uint8_t latch);
	uint8_t spi_shift(void);
	void read_temp(void);

        uint8_t thermocouple_temp;
        uint8_t fault;
        uint8_t chip_temp;
        uint8_t scv_fault;
        uint8_t scg_fault;
        uint8_t oc_fault;
private:
	uint8_t data;
	uint8_t clock;
	uint8_t latch;

};
#endif
