#ifndef _SPEED_SENSOR_H_
#define _SPEED_SENSOR_H_

#include "mbed.h"

class SpeedSensor {
private:
	PinName pin;
	InterruptIn hall_sensor_int;
	Timer speed_timer;
	Timeout watchdog;
	bool valid_read_speed;
	bool disable_int_when_stopped; /* WARNING: Might disable a lot more interrupt lines */
	void sensor_irq();
	void watchdog_irq();
public:
	SpeedSensor(PinName pin);
	void stop(); /* WARNING: Might disable a lot more interrupt lines */
	void start();
	volatile int speed;
	volatile int time_ms;
};

#endif /* _SPEED_SENSOR_H_ */
