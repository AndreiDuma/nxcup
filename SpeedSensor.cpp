#include "SpeedSensor.h"
#include <cstddef>
#define MAX_TIME_UNIT 999999999
/* TODO: may be adjustable to a lower valuable when car is
 * actually running because it will be influenced by friction
 */
#define MIN_SPEED_T_REF_MS 250.0
#define MIN_SPEED_T_REF_SEC ((MIN_SPEED_T_REF_MS) / 1000)

// TODO Implement watchdog

SpeedSensor::SpeedSensor(PinName pin) : hall_sensor_int(pin)
{
	this->pin = pin;
	start();
}

void SpeedSensor::sensor_irq()
{
	watchdog.detach();
	speed_timer.stop();
	
	if (valid_read_speed) {
		time_ms = speed_timer.read_ms();
		speed =  MIN_SPEED_T_REF_MS - time_ms;
	} else {
		time_ms = MAX_TIME_UNIT;
		speed = 0;
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
	speed_timer.stop();
	speed_timer.reset();
	valid_read_speed = false;
	speed = 0;
	time_ms = MAX_TIME_UNIT;
}

void SpeedSensor::start()
{
	valid_read_speed = false;
	speed_timer.reset();
	speed_timer.start();
	hall_sensor_int.fall(this, &SpeedSensor::sensor_irq);
	watchdog.attach(this, &SpeedSensor::watchdog_irq, MIN_SPEED_T_REF_SEC);
}
