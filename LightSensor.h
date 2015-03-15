
#ifndef _LIGHTSENSOR_H_
#define _LIGHTSENSOR_H_

#include "mbed.h"

class LightSensor
{
private:
	Timer clock_timer;
	Ticker mean_aprox_timer;
	InterruptIn clock_int;
	void clock_irq();
	bool valid_clock_value;
	void (*handler)(void);
	void light_sensor_irq();
	void approximate();
	volatile int count;
public:
	LightSensor(PinName pin, void (*handler)(void));
	volatile int light_value;
	void start();
};

#endif /* _LIGHTSENSOR_H_ */
