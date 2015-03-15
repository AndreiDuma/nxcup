#include "LightSensor.h"
#include <time.h>

LightSensor::LightSensor(PinName pin, void (*handler)(void)) : clock_int(pin)
{
	this->handler = handler;
	this->light_value = 5;
	this->valid_clock_value = false;
	this->clock_timer.start();
	this->count = 0;
	/* seara neoane + led-uri: 0-4 fara leduri; 5-7 cu leduri  */
	/* lumina mai multa => valori mai mari */
}

void LightSensor::approximate()
{
	light_value = count;
	count = 0;
	handler();
}

void LightSensor::light_sensor_irq()
{
	count++;
	/*
	clock_timer.stop();

	if (valid_clock_value)
		light_value = clock_timer.read_us();
	else
		light_value = 5;

	valid_clock_value = true;

	clock_timer.reset();
	clock_timer.start();

	if (handler)
		handler();
	*/
}


void LightSensor::start()
{
	this->clock_int.rise(this, &LightSensor::light_sensor_irq);
	this->mean_aprox_timer.attach(this, &LightSensor::approximate, 0.0002);
}
