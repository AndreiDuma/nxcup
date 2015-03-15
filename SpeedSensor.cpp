#include "SpeedSensor.h"
#include <cstddef>
#define MAX_TIME_UNIT 999999999
/* TODO: may be adjustable to a lower valuable when car is
 * actually running because it will be influenced by friction
 */
#define MIN_SPEED_T_REF_MS 270.0
#define MIN_SPEED_T_REF_SEC ((MIN_SPEED_T_REF_MS) / 1000)


SpeedSensor::SpeedSensor(PinName pin, int weight_adjust,
			 void (*handler)(void)) : hall_sensor_int(pin)
{
	this->pin = pin;
	this->weight_adjust = weight_adjust;
	this->handler = handler;
	start();
}

void SpeedSensor::sensor_irq()
{
	watchdog.detach();
	speed_timer.stop();

	if (valid_read_speed) {
		set_speed(MIN_SPEED_T_REF_MS + weight_adjust - speed_timer.read_ms());
		time_ms = speed_timer.read_ms();
		if (speed < 0) // sanity check
			speed = 0;
	} else {
		set_speed(0);
	}

	speed_timer.reset();
	speed_timer.start();
	valid_read_speed = true;
	watchdog.attach(this, &SpeedSensor::watchdog_irq, MIN_SPEED_T_REF_SEC);
}

void SpeedSensor::watchdog_irq()
{
	stop();
}

void SpeedSensor::stop()
{
	valid_read_speed = false;
	set_speed(0);
}

void SpeedSensor::start()
{
	valid_read_speed = false;
	speed_timer.reset();
	speed_timer.start();
	hall_sensor_int.fall(this, &SpeedSensor::sensor_irq);
	watchdog.attach(this, &SpeedSensor::watchdog_irq, MIN_SPEED_T_REF_SEC);
}

void SpeedSensor::set_speed(int new_speed)
{
	this->speed = new_speed;
	if (handler)
		handler();
}
