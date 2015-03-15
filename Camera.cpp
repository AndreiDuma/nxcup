#include "Camera.h"
#include "TFC.h"
#include "ComparatorIn.h"

#define REPEAT_5_TIMES(thingy) thingy; thingy; thingy; thingy; thingy
#define REPEAT_10_TIMES(thingy) \
	REPEAT_5_TIMES(thingy); \
	REPEAT_5_TIMES(thingy); \
	REPEAT_5_TIMES(thingy); \
	REPEAT_5_TIMES(thingy); \
	REPEAT_5_TIMES(thingy);

static inline void camera_half_clock_delay() __attribute__((always_inline));
static inline void camera_clock_delay() __attribute__((always_inline));

Camera::Camera(void (*handler)(void)) : comparator(PTC7, NC)
{
	this->handler = handler;
	this->left_line_pos = NO_LINE_DETECTED;
	this->right_line_pos = NO_LINE_DETECTED;
	this->comparator.treshold(1.0);
	this->comparator.hysteresis(3);
	this->pixels_integration_time_ms = 13;
}

static inline void camera_half_clock_delay()
{
	asm volatile("nop");
	asm volatile("nop");
}

static inline void camera_clock_delay()
{
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
}

void Camera::get_raw_data()
{
	int i;

	TAOS_CLK_LOW; /* sanity: clock reset */

	TAOS_SI_HIGH;
	camera_half_clock_delay();
	TAOS_CLK_HIGH;
	camera_half_clock_delay();
	TAOS_SI_LOW;
	camera_half_clock_delay();
	TAOS_CLK_LOW;
	camera_clock_delay();

	for (i = 0; i < 128; i++) {
		pixels[i] = comparator.status();
		TAOS_CLK_HIGH;
		camera_clock_delay();
		TAOS_CLK_LOW;
	}
}

void Camera::set_left_line()
{
	int i;

	/*
	for (i = 64; i >= 14; i--)
		if (pixels[i] == 0) {
			left_line_pos = i;
			return;
		}

	left_line_pos = NO_LINE_DETECTED;
	*/
	sum_left = 0;
	for (i = 64; i >= 0; i--)
		sum_left+=pixels[i];

}

void Camera::set_right_line()
{
	int i;
	/*
	for (i = 65; i < 114; i++)
		if (pixels[i] == 0) {
			while (pixels[i] == 0 && i < 114)
				i++;
			right_line_pos = i;
			return;
		}
	right_line_pos = NO_LINE_DETECTED;
	*/
	sum_right = 0;
	for (i = 65; i < 128; i++)
		sum_right+=pixels[i];
}

void Camera::iterate()
{
	get_raw_data();
	set_left_line();
	set_right_line();
	if (handler)
		handler();
	wait_ms(pixels_integration_time_ms);
}

void Camera::init_camera_read()
{
	get_raw_data();
	wait_ms(pixels_integration_time_ms);
}

int Camera::get_left_line()
{
	return left_line_pos;
}

int Camera::get_right_line()
{
	return right_line_pos;
}

void Camera::set_th(float th)
{
	comparator.treshold(th);
}

void Camera::set_integration_time(float integ_time_ms)
{
	pixels_integration_time_ms = integ_time_ms;
}
