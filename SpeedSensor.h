#ifndef _SPEED_SENSOR_H_
#define _SPEED_SENSOR_H_

#include "mbed.h"

class SpeedSensor {
private:
	PinName pin;
	InterruptIn hall_sensor_int;
	Timer speed_timer;
	Timeout watchdog;
	volatile bool valid_read_speed;
	void sensor_irq();
	void watchdog_irq();
	int weight_adjust;
	void (*handler)(void);
	void set_speed(int new_speed);
public:
	SpeedSensor(PinName pin, int weight_adjust, void (*handler)(void));
	void stop(); /* WARNING: Might disable a lot more interrupt lines */
	void start();
	volatile int speed;
	volatile int time_ms;
};

#endif /* _SPEED_SENSOR_H_ */
